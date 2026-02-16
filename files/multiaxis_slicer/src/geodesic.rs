/// Geodesic slicing via the Heat Method (Crane et al. 2013).
///
/// Computes geodesic distance on a triangle mesh surface, then extracts
/// level-set contours ("marching triangles") as non-planar layers.

use std::collections::HashMap;

use crate::geometry::{Contour, Point3D, Vector3D};
use crate::slicing::Layer;
use crate::mesh::Mesh;

// ── Data structures ─────────────────────────────────────────────────

/// Welded mesh topology with adjacency information.
pub struct MeshTopology {
    pub vertices: Vec<Point3D>,
    pub faces: Vec<[usize; 3]>,
    pub vertex_normals: Vec<Vector3D>,
    pub vertex_faces: Vec<Vec<usize>>,
}

/// Source region for geodesic distance computation.
#[derive(Clone)]
pub enum GeodesicSource {
    /// Auto-detect vertices at the bottom of the mesh (min Z ± tolerance).
    BottomBoundary,
    /// Nearest vertex to the given 3D point.
    Point(Point3D),
}

/// Configuration for geodesic slicing.
#[derive(Clone)]
pub struct GeodesicSlicerConfig {
    pub source: GeodesicSource,
    pub layer_height: f64,
    /// Multiplier on avg_edge_length² for the heat timestep (default 1.0).
    pub heat_timestep_factor: f64,
    /// Z tolerance for bottom boundary detection in mm (default 0.5).
    pub bottom_tolerance: f64,
}

impl Default for GeodesicSlicerConfig {
    fn default() -> Self {
        Self {
            source: GeodesicSource::BottomBoundary,
            layer_height: 0.2,
            heat_timestep_factor: 1.0,
            bottom_tolerance: 0.5,
        }
    }
}

/// Sparse symmetric matrix stored as diagonal + sorted off-diagonal triplets.
struct SparseSymmetric {
    n: usize,
    diag: Vec<f64>,
    /// Off-diagonal entries (i, j, value) with i < j. Each pair stored once.
    off_diag: Vec<(usize, usize, f64)>,
}

impl SparseSymmetric {
    /// Compute y = A * x
    fn mul_vec(&self, x: &[f64], y: &mut [f64]) {
        for i in 0..self.n {
            y[i] = self.diag[i] * x[i];
        }
        for &(i, j, val) in &self.off_diag {
            y[i] += val * x[j];
            y[j] += val * x[i];
        }
    }
}

// ── Topology construction ───────────────────────────────────────────

impl MeshTopology {
    /// Build welded topology from a triangle-soup Mesh.
    /// STL meshes have duplicated vertices per triangle; we weld by position.
    pub fn from_mesh(mesh: &Mesh) -> Self {
        let quantize = |v: f64| -> i64 { (v * 1e6).round() as i64 };

        let mut vertex_map: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut vertices: Vec<Point3D> = Vec::new();
        let mut faces: Vec<[usize; 3]> = Vec::new();

        for tri in &mesh.triangles {
            let mut face = [0usize; 3];
            for (k, v) in [tri.v0, tri.v1, tri.v2].iter().enumerate() {
                let key = (quantize(v.x), quantize(v.y), quantize(v.z));
                let idx = *vertex_map.entry(key).or_insert_with(|| {
                    let idx = vertices.len();
                    vertices.push(*v);
                    idx
                });
                face[k] = idx;
            }
            // Skip degenerate triangles
            if face[0] != face[1] && face[1] != face[2] && face[0] != face[2] {
                faces.push(face);
            }
        }

        // Build vertex → face adjacency
        let mut vertex_faces = vec![Vec::new(); vertices.len()];
        for (fi, face) in faces.iter().enumerate() {
            for &vi in face {
                vertex_faces[vi].push(fi);
            }
        }

        // Area-weighted vertex normals
        let mut vertex_normals = vec![Vector3D::zeros(); vertices.len()];
        for face in &faces {
            let v0 = vertices[face[0]];
            let v1 = vertices[face[1]];
            let v2 = vertices[face[2]];
            let e1 = v1 - v0;
            let e2 = v2 - v0;
            let normal = e1.cross(&e2); // length = 2 × area
            for &vi in face {
                vertex_normals[vi] += normal;
            }
        }
        for n in &mut vertex_normals {
            let len = n.norm();
            if len > 1e-12 {
                *n /= len;
            }
        }

        log::info!("  Geodesic topology: {} vertices, {} faces (welded from {} triangles)",
            vertices.len(), faces.len(), mesh.triangles.len());

        Self { vertices, faces, vertex_normals, vertex_faces }
    }

    fn num_vertices(&self) -> usize {
        self.vertices.len()
    }
}

// ── Cotangent Laplacian ─────────────────────────────────────────────

/// Build the cotangent Laplacian L and lumped mass matrix M.
/// Returns (mass_diagonal, laplacian_as_sparse_symmetric).
fn build_cotangent_laplacian(topo: &MeshTopology) -> (Vec<f64>, SparseSymmetric) {
    let n = topo.num_vertices();
    let mut mass = vec![0.0f64; n];

    // Accumulate off-diagonal weights in a map to merge duplicate edges
    let mut edge_weights: HashMap<(usize, usize), f64> = HashMap::new();
    let mut diag = vec![0.0f64; n];

    for face in &topo.faces {
        let [i, j, k] = *face;
        let vi = topo.vertices[i];
        let vj = topo.vertices[j];
        let vk = topo.vertices[k];

        // Triangle area
        let e1 = vj - vi;
        let e2 = vk - vi;
        let area = e1.cross(&e2).norm() * 0.5;
        if area < 1e-15 {
            continue;
        }

        // Lumped mass: 1/3 of area per vertex
        let third_area = area / 3.0;
        mass[i] += third_area;
        mass[j] += third_area;
        mass[k] += third_area;

        // Cotangent weights for each edge
        // Edge (j,k) opposite vertex i
        let ejk = vk - vj;
        let eji = vi - vj;
        let eki = vi - vk;
        let ekj = vj - vk;

        // cot(angle at i) for edge (j,k)
        let cot_i = {
            let a = vj - vi;
            let b = vk - vi;
            a.dot(&b) / a.cross(&b).norm().max(1e-12)
        };
        // cot(angle at j) for edge (i,k)
        let cot_j = {
            let a = vi - vj;
            let b = vk - vj;
            a.dot(&b) / a.cross(&b).norm().max(1e-12)
        };
        // cot(angle at k) for edge (i,j)
        let cot_k = {
            let a = vi - vk;
            let b = vj - vk;
            a.dot(&b) / a.cross(&b).norm().max(1e-12)
        };

        // Clamp to avoid instability from near-degenerate triangles
        let clamp = |c: f64| c.clamp(-100.0, 100.0);
        let cot_i = clamp(cot_i);
        let cot_j = clamp(cot_j);
        let cot_k = clamp(cot_k);

        // Each edge gets 0.5 * cot(opposite angle)
        // Edge (j,k): weight = 0.5 * cot_i
        let add_edge = |map: &mut HashMap<(usize, usize), f64>, a: usize, b: usize, w: f64| {
            let key = if a < b { (a, b) } else { (b, a) };
            *map.entry(key).or_insert(0.0) += w;
        };

        add_edge(&mut edge_weights, j, k, 0.5 * cot_i);
        add_edge(&mut edge_weights, i, k, 0.5 * cot_j);
        add_edge(&mut edge_weights, i, j, 0.5 * cot_k);
    }

    // Build sparse symmetric matrix
    let mut off_diag: Vec<(usize, usize, f64)> = Vec::with_capacity(edge_weights.len());
    for (&(i, j), &w) in &edge_weights {
        // Laplacian off-diagonal = -w (negative of cotangent weight)
        off_diag.push((i, j, -w));
        // Diagonal accumulates positive weights
        diag[i] += w;
        diag[j] += w;
    }

    // Ensure minimum mass for isolated vertices
    for m in &mut mass {
        if *m < 1e-12 {
            *m = 1e-12;
        }
    }

    let laplacian = SparseSymmetric { n, diag, off_diag };
    (mass, laplacian)
}

// ── Conjugate Gradient solver ───────────────────────────────────────

/// Preconditioned Conjugate Gradient for symmetric positive definite systems.
/// Solves A·x = b with Jacobi (diagonal) preconditioner.
fn conjugate_gradient(
    a: &SparseSymmetric,
    b: &[f64],
    max_iter: usize,
    tol: f64,
) -> Vec<f64> {
    let n = a.n;
    let mut x = vec![0.0; n];
    let mut r = b.to_vec();
    // Jacobi preconditioner: M^{-1} = 1/diag(A)
    let inv_diag: Vec<f64> = a.diag.iter().map(|&d| {
        if d.abs() > 1e-15 { 1.0 / d } else { 1.0 }
    }).collect();

    // z = M^{-1} r
    let mut z: Vec<f64> = r.iter().zip(&inv_diag).map(|(ri, mi)| ri * mi).collect();
    let mut p = z.clone();
    let mut rz: f64 = r.iter().zip(&z).map(|(ri, zi)| ri * zi).sum();

    let b_norm: f64 = b.iter().map(|bi| bi * bi).sum::<f64>().sqrt().max(1e-15);

    let mut ap = vec![0.0; n];

    for _iter in 0..max_iter {
        let r_norm: f64 = r.iter().map(|ri| ri * ri).sum::<f64>().sqrt();
        if r_norm / b_norm < tol {
            break;
        }

        a.mul_vec(&p, &mut ap);
        let pap: f64 = p.iter().zip(&ap).map(|(pi, api)| pi * api).sum();
        if pap.abs() < 1e-30 {
            break;
        }
        let alpha = rz / pap;

        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }

        for i in 0..n {
            z[i] = r[i] * inv_diag[i];
        }

        let rz_new: f64 = r.iter().zip(&z).map(|(ri, zi)| ri * zi).sum();
        let beta = rz_new / rz.max(1e-30);
        rz = rz_new;

        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }

    x
}

// ── Heat Method ─────────────────────────────────────────────────────

/// Compute geodesic distances from source vertices using the Heat Method.
fn compute_geodesic_distance(
    topo: &MeshTopology,
    mass: &[f64],
    laplacian: &SparseSymmetric,
    sources: &[usize],
    heat_timestep_factor: f64,
) -> Vec<f64> {
    let n = topo.num_vertices();

    // Compute average edge length for timestep
    let mut total_edge_len = 0.0;
    let mut edge_count = 0;
    for face in &topo.faces {
        for k in 0..3 {
            let a = face[k];
            let b = face[(k + 1) % 3];
            if a < b {
                total_edge_len += (topo.vertices[b] - topo.vertices[a]).norm();
                edge_count += 1;
            }
        }
    }
    let avg_edge = if edge_count > 0 { total_edge_len / edge_count as f64 } else { 1.0 };
    let t = heat_timestep_factor * avg_edge * avg_edge;
    log::info!("  Heat method: avg_edge={:.4}mm, timestep t={:.6}", avg_edge, t);

    // Step 1: Build heat system (M + t*L) and RHS = M*delta
    // The heat equation backward Euler: (M + t*L) u = M * delta_S
    let heat_diag: Vec<f64> = (0..n).map(|i| mass[i] + t * laplacian.diag[i]).collect();
    let heat_off_diag: Vec<(usize, usize, f64)> = laplacian.off_diag.iter()
        .map(|&(i, j, val)| (i, j, t * val))
        .collect();
    let heat_system = SparseSymmetric { n, diag: heat_diag, off_diag: heat_off_diag };

    let mut rhs = vec![0.0; n];
    for &s in sources {
        rhs[s] = mass[s];
    }

    let u = conjugate_gradient(&heat_system, &rhs, 500, 1e-8);

    // Step 2: Per-face normalized negative gradient of u
    let mut face_grad: Vec<Vector3D> = Vec::with_capacity(topo.faces.len());
    for face in &topo.faces {
        let [i, j, k] = *face;
        let vi = topo.vertices[i];
        let vj = topo.vertices[j];
        let vk = topo.vertices[k];

        let e_ij = vj - vi;
        let e_ik = vk - vi;
        let face_normal = e_ij.cross(&e_ik);
        let area2 = face_normal.norm(); // = 2 * area

        if area2 < 1e-15 {
            face_grad.push(Vector3D::zeros());
            continue;
        }
        let n_hat = face_normal / area2;

        // Gradient of scalar field on triangle:
        // ∇u = (1/(2A)) * Σ u_i * (N × e_opposite_i)
        // where e_opposite_i is the edge opposite to vertex i
        let e_jk = vk - vj;
        let e_ki = vi - vk;

        let grad = (u[i] * n_hat.cross(&e_jk)
                  + u[j] * n_hat.cross(&e_ki)
                  + u[k] * n_hat.cross(&e_ij)) / (area2 * 0.5); // divide by area (area2/2)

        // Wait -- the formula is ∇u = (1/(2A)) * Σ u_i * (N_hat × e_opposite)
        // But we computed N_hat = face_normal / |face_normal| where |face_normal| = 2A
        // So (1/(2A)) * u_i * (N_hat × e_opp) works directly, we just sum them
        // Let me redo this more carefully:
        let area = area2 * 0.5;
        let n_unit = face_normal / area2; // unit normal

        let grad_u = (1.0 / (2.0 * area)) * (
            u[i] * n_unit.cross(&e_jk)
          + u[j] * n_unit.cross(&e_ki)
          + u[k] * n_unit.cross(&e_ij)
        );

        let grad_norm = grad_u.norm();
        if grad_norm > 1e-15 {
            face_grad.push(-grad_u / grad_norm); // Normalized negative gradient
        } else {
            face_grad.push(Vector3D::zeros());
        }
    }

    // Step 3: Compute integrated divergence at each vertex
    let mut div = vec![0.0; n];
    for (fi, face) in topo.faces.iter().enumerate() {
        let [i, j, k] = *face;
        let vi = topo.vertices[i];
        let vj = topo.vertices[j];
        let vk = topo.vertices[k];
        let x_f = face_grad[fi];

        // For each half-edge in the face, accumulate divergence
        // div[i] += 0.5 * (cot(angle_j) * dot(e_ij, X) + cot(angle_k) * dot(e_ik, X))
        // where e_ij = vj - vi, e_ik = vk - vi
        let e_ij = vj - vi;
        let e_ik = vk - vi;
        let e_jk = vk - vj;
        let e_ji = vi - vj;
        let e_ki = vi - vk;
        let e_kj = vj - vk;

        // Cotangents (same as Laplacian build but we need them per-vertex here)
        let cross_i = (vj - vi).cross(&(vk - vi)).norm().max(1e-12);
        let cross_j = (vi - vj).cross(&(vk - vj)).norm().max(1e-12);
        let cross_k = (vi - vk).cross(&(vj - vk)).norm().max(1e-12);

        let cot_i = (vj - vi).dot(&(vk - vi)) / cross_i;
        let cot_j = (vi - vj).dot(&(vk - vj)) / cross_j;
        let cot_k = (vi - vk).dot(&(vj - vk)) / cross_k;

        // Divergence contribution for vertex i:
        // 0.5 * (cot(angle_k) * dot(e_ij, X) + cot(angle_j) * dot(e_ik, X))
        // Note: angle at k is opposite to edge ij, angle at j is opposite to edge ik
        div[i] += 0.5 * (cot_k * e_ij.dot(&x_f) + cot_j * e_ik.dot(&x_f));
        div[j] += 0.5 * (cot_i * e_jk.dot(&x_f) + cot_k * e_ji.dot(&x_f));
        div[k] += 0.5 * (cot_j * e_ki.dot(&x_f) + cot_i * e_kj.dot(&x_f));
    }

    // Step 4: Solve Poisson equation L*phi = div
    // The Laplacian is singular (constant in null space), so we pin one vertex
    // by adding a small value to diagonal
    let mut poisson_diag = laplacian.diag.clone();
    poisson_diag[0] += 1e-6; // Pin vertex 0 to make system non-singular

    let poisson_system = SparseSymmetric {
        n,
        diag: poisson_diag,
        off_diag: laplacian.off_diag.clone(),
    };

    let phi = conjugate_gradient(&poisson_system, &div, 500, 1e-8);

    // Step 5: Shift so source vertices have distance = 0
    // Use mean of source values as the baseline (more robust than global min)
    let source_mean: f64 = sources.iter().map(|&s| phi[s]).sum::<f64>() / sources.len() as f64;
    let mut distances: Vec<f64> = phi.iter().map(|&p| (p - source_mean).abs()).collect();
    // The Poisson solve may flip sign — check if source vertices have high values
    // If so, negate (distance should increase away from sources)
    let source_avg: f64 = sources.iter().map(|&s| distances[s]).sum::<f64>() / sources.len() as f64;
    let global_avg: f64 = distances.iter().sum::<f64>() / distances.len() as f64;
    if source_avg > global_avg {
        // Distances are inverted, flip them
        let max_d = distances.iter().cloned().fold(0.0f64, f64::max);
        for d in &mut distances {
            *d = max_d - *d;
        }
    }
    // Shift so min = 0
    let min_d = distances.iter().cloned().fold(f64::INFINITY, f64::min);
    for d in &mut distances {
        *d -= min_d;
    }

    let max_dist = distances.iter().cloned().fold(0.0f64, f64::max);
    log::info!("  Geodesic distances: min=0.0, max={:.3}mm, {} source vertices",
        max_dist, sources.len());

    distances
}

// ── Source vertex detection ─────────────────────────────────────────

fn find_source_vertices(topo: &MeshTopology, source: &GeodesicSource) -> Vec<usize> {
    match source {
        GeodesicSource::BottomBoundary => {
            let min_z = topo.vertices.iter().map(|v| v.z).fold(f64::INFINITY, f64::min);
            let tolerance = 0.5; // mm
            let sources: Vec<usize> = (0..topo.vertices.len())
                .filter(|&i| topo.vertices[i].z <= min_z + tolerance)
                .collect();
            log::info!("  Bottom boundary source: {} vertices at z <= {:.2}mm",
                sources.len(), min_z + tolerance);
            sources
        }
        GeodesicSource::Point(p) => {
            let nearest = (0..topo.vertices.len())
                .min_by(|&a, &b| {
                    let da = (topo.vertices[a] - p).norm_squared();
                    let db = (topo.vertices[b] - p).norm_squared();
                    da.partial_cmp(&db).unwrap()
                })
                .unwrap_or(0);
            log::info!("  Point source: vertex {} at ({:.2}, {:.2}, {:.2})",
                nearest, topo.vertices[nearest].x, topo.vertices[nearest].y, topo.vertices[nearest].z);
            vec![nearest]
        }
    }
}

// ── Level set extraction (marching triangles) ───────────────────────

/// Extract contours at a given geodesic distance value from the mesh.
fn extract_level_set(
    topo: &MeshTopology,
    distances: &[f64],
    level: f64,
) -> Vec<Contour> {
    // For each face, find edges that the level crosses and generate a segment
    // Key: (vertex_a, vertex_b) with a < b → interpolated point
    let mut segments: Vec<(Point3D, Point3D)> = Vec::new();

    for face in &topo.faces {
        let [i, j, k] = *face;
        let di = distances[i] - level;
        let dj = distances[j] - level;
        let dk = distances[k] - level;

        // Find edges where the level crosses (sign change)
        let mut crossings: Vec<Point3D> = Vec::new();

        let check_edge = |a: usize, b: usize, da: f64, db: f64, crossings: &mut Vec<Point3D>| {
            if (da > 0.0 && db < 0.0) || (da < 0.0 && db > 0.0) {
                let t = da / (da - db);
                let p = topo.vertices[a] + t * (topo.vertices[b] - topo.vertices[a]);
                crossings.push(p);
            } else if da.abs() < 1e-12 {
                crossings.push(topo.vertices[a]);
            }
        };

        check_edge(i, j, di, dj, &mut crossings);
        check_edge(j, k, dj, dk, &mut crossings);
        check_edge(k, i, dk, di, &mut crossings);

        if crossings.len() >= 2 {
            segments.push((crossings[0], crossings[1]));
        }
    }

    if segments.is_empty() {
        return Vec::new();
    }

    // Chain segments into contours by matching endpoints
    chain_segments_into_contours(segments)
}

/// Chain line segments into closed/open contours by matching endpoints.
fn chain_segments_into_contours(segments: Vec<(Point3D, Point3D)>) -> Vec<Contour> {
    if segments.is_empty() {
        return Vec::new();
    }

    // Quantize points for hashing
    let quantize = |p: &Point3D| -> (i64, i64, i64) {
        ((p.x * 1e4).round() as i64, (p.y * 1e4).round() as i64, (p.z * 1e4).round() as i64)
    };

    // Build adjacency: point_key → list of (segment_index, other_end_key)
    let mut adj: HashMap<(i64, i64, i64), Vec<(usize, (i64, i64, i64))>> = HashMap::new();
    let mut seg_points: Vec<(Point3D, Point3D)> = Vec::with_capacity(segments.len());
    let mut used = vec![false; segments.len()];

    for (idx, (a, b)) in segments.iter().enumerate() {
        let ka = quantize(a);
        let kb = quantize(b);
        adj.entry(ka).or_default().push((idx, kb));
        adj.entry(kb).or_default().push((idx, ka));
        seg_points.push((*a, *b));
    }

    let mut contours = Vec::new();

    for start_idx in 0..segments.len() {
        if used[start_idx] {
            continue;
        }
        used[start_idx] = true;

        let (a, b) = seg_points[start_idx];
        let mut chain = vec![a, b];
        let mut current_key = quantize(&b);
        let start_key = quantize(&a);

        // Walk forward
        loop {
            let mut found = false;
            if let Some(neighbors) = adj.get(&current_key) {
                for &(seg_idx, other_key) in neighbors {
                    if !used[seg_idx] {
                        used[seg_idx] = true;
                        // Find which end of the segment matches current_key
                        let (sa, sb) = seg_points[seg_idx];
                        let ska = quantize(&sa);
                        let next_pt = if ska == current_key { sb } else { sa };
                        chain.push(next_pt);
                        current_key = quantize(&next_pt);
                        found = true;
                        break;
                    }
                }
            }
            if !found {
                break;
            }
            // Check if we looped back to start
            if current_key == start_key {
                break;
            }
        }

        let closed = current_key == start_key;
        if chain.len() >= 3 || (chain.len() >= 2 && !closed) {
            contours.push(Contour::new(chain, closed));
        }
    }

    contours
}

// ── Top-level entry point ───────────────────────────────────────────

/// Slice a mesh using geodesic distance field (Heat Method).
/// Returns layers with contours that conform to the mesh surface curvature.
pub fn geodesic_slice(mesh: &Mesh, config: &GeodesicSlicerConfig) -> Vec<Layer> {
    log::info!("=== Geodesic Slicing (Heat Method) ===");

    // Step 1: Build topology
    log::info!("Step 1: Building mesh topology...");
    let topo = MeshTopology::from_mesh(mesh);
    if topo.vertices.is_empty() || topo.faces.is_empty() {
        log::error!("  Empty mesh topology, cannot slice");
        return Vec::new();
    }

    // Step 2: Build cotangent Laplacian
    log::info!("Step 2: Building cotangent Laplacian...");
    let (mass, laplacian) = build_cotangent_laplacian(&topo);
    log::info!("  Laplacian: {} vertices, {} off-diagonal entries",
        laplacian.n, laplacian.off_diag.len());

    // Step 3: Find source vertices
    log::info!("Step 3: Finding source vertices...");
    let mut source = config.source.clone();
    // Apply tolerance from config for bottom boundary
    let sources = match &source {
        GeodesicSource::BottomBoundary => {
            let min_z = topo.vertices.iter().map(|v| v.z).fold(f64::INFINITY, f64::min);
            (0..topo.vertices.len())
                .filter(|&i| topo.vertices[i].z <= min_z + config.bottom_tolerance)
                .collect::<Vec<_>>()
        }
        GeodesicSource::Point(p) => {
            find_source_vertices(&topo, &source)
        }
    };

    if sources.is_empty() {
        log::error!("  No source vertices found");
        return Vec::new();
    }
    log::info!("  Found {} source vertices", sources.len());

    // Step 4: Compute geodesic distances
    log::info!("Step 4: Computing geodesic distance field (Heat Method)...");
    let distances = compute_geodesic_distance(
        &topo, &mass, &laplacian, &sources, config.heat_timestep_factor
    );

    // Step 5: Extract level-set layers
    log::info!("Step 5: Extracting geodesic layers...");
    let max_dist = distances.iter().cloned().fold(0.0f64, f64::max);
    let num_layers = (max_dist / config.layer_height).ceil() as usize;
    if num_layers == 0 {
        log::warn!("  Max geodesic distance is 0, no layers to extract");
        return Vec::new();
    }

    let mut layers = Vec::with_capacity(num_layers);
    for i in 1..=num_layers {
        let level = i as f64 * config.layer_height;
        if level > max_dist {
            break;
        }
        let contours = extract_level_set(&topo, &distances, level);
        if !contours.is_empty() {
            layers.push(Layer::new(level, contours, config.layer_height));
        }
    }

    log::info!("=== Geodesic slicing complete: {} layers ===", layers.len());
    layers
}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Triangle;

    fn make_flat_square_mesh() -> Mesh {
        // 1x1 flat square at z=0, split into 2 triangles
        let t1 = Triangle::new(
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(1.0, 1.0, 0.0),
        );
        let t2 = Triangle::new(
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(1.0, 1.0, 0.0),
            Point3D::new(0.0, 1.0, 0.0),
        );
        Mesh::new(vec![t1, t2]).unwrap()
    }

    fn make_cube_mesh() -> Mesh {
        // Simple 10x10x10 cube
        let s = 10.0;
        let corners = [
            Point3D::new(0.0, 0.0, 0.0), // 0: bottom-front-left
            Point3D::new(s,   0.0, 0.0),   // 1: bottom-front-right
            Point3D::new(s,   s,   0.0),   // 2: bottom-back-right
            Point3D::new(0.0, s,   0.0),   // 3: bottom-back-left
            Point3D::new(0.0, 0.0, s),     // 4: top-front-left
            Point3D::new(s,   0.0, s),     // 5: top-front-right
            Point3D::new(s,   s,   s),     // 6: top-back-right
            Point3D::new(0.0, s,   s),     // 7: top-back-left
        ];
        let faces: Vec<[usize; 3]> = vec![
            // bottom
            [0, 2, 1], [0, 3, 2],
            // top
            [4, 5, 6], [4, 6, 7],
            // front
            [0, 1, 5], [0, 5, 4],
            // back
            [2, 3, 7], [2, 7, 6],
            // left
            [0, 4, 7], [0, 7, 3],
            // right
            [1, 2, 6], [1, 6, 5],
        ];
        let triangles: Vec<Triangle> = faces.iter().map(|f| {
            Triangle::new(corners[f[0]], corners[f[1]], corners[f[2]])
        }).collect();
        Mesh::new(triangles).unwrap()
    }

    #[test]
    fn test_topology_welding() {
        let mesh = make_flat_square_mesh();
        let topo = MeshTopology::from_mesh(&mesh);
        // 2 triangles share edge, should weld to 4 unique vertices
        assert_eq!(topo.vertices.len(), 4);
        assert_eq!(topo.faces.len(), 2);
    }

    #[test]
    fn test_cg_solver() {
        // Solve simple diagonal system: 2x = 4 → x = 2
        let a = SparseSymmetric {
            n: 3,
            diag: vec![2.0, 3.0, 1.0],
            off_diag: vec![],
        };
        let b = vec![4.0, 9.0, 5.0];
        let x = conjugate_gradient(&a, &b, 100, 1e-10);
        assert!((x[0] - 2.0).abs() < 1e-6);
        assert!((x[1] - 3.0).abs() < 1e-6);
        assert!((x[2] - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_cg_solver_with_off_diagonal() {
        // Solve [[2, 1], [1, 2]] * [x, y] = [3, 3] → x=1, y=1
        let a = SparseSymmetric {
            n: 2,
            diag: vec![2.0, 2.0],
            off_diag: vec![(0, 1, 1.0)],
        };
        let b = vec![3.0, 3.0];
        let x = conjugate_gradient(&a, &b, 100, 1e-10);
        assert!((x[0] - 1.0).abs() < 1e-6);
        assert!((x[1] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_bottom_boundary_detection() {
        let mesh = make_cube_mesh();
        let topo = MeshTopology::from_mesh(&mesh);
        let sources = find_source_vertices(&topo, &GeodesicSource::BottomBoundary);
        // Bottom face of cube: 4 vertices at z=0
        assert_eq!(sources.len(), 4);
        for &s in &sources {
            assert!(topo.vertices[s].z.abs() < 1e-6);
        }
    }

    #[test]
    fn test_geodesic_distance_flat() {
        // On a flat square, geodesic distance from corner (0,0) should approximate Euclidean
        let mesh = make_flat_square_mesh();
        let topo = MeshTopology::from_mesh(&mesh);
        let (mass, laplacian) = build_cotangent_laplacian(&topo);

        // Find vertex at (0,0,0)
        let origin_idx = (0..topo.vertices.len())
            .find(|&i| topo.vertices[i].x.abs() < 1e-6 && topo.vertices[i].y.abs() < 1e-6)
            .unwrap();

        let distances = compute_geodesic_distance(&topo, &mass, &laplacian, &[origin_idx], 1.0);

        // Distances should be non-negative after shift
        assert!(distances.iter().all(|&d| d >= -1e-6), "All distances should be >= 0");

        // The range of distances should be non-trivial
        let max_dist = distances.iter().cloned().fold(0.0f64, f64::max);
        assert!(max_dist > 0.1, "Max distance should be > 0, got {}", max_dist);

        // Source vertex should have smaller distance than the far corner
        let far_corner = (0..topo.vertices.len())
            .find(|&i| (topo.vertices[i].x - 1.0).abs() < 1e-6 && (topo.vertices[i].y - 1.0).abs() < 1e-6)
            .unwrap();
        assert!(distances[origin_idx] < distances[far_corner],
            "Origin ({:.4}) should be < far corner ({:.4})", distances[origin_idx], distances[far_corner]);
    }

    #[test]
    fn test_level_set_extraction() {
        // On a cube, geodesic from bottom should produce contours at intermediate heights
        let mesh = make_cube_mesh();
        let config = GeodesicSlicerConfig {
            source: GeodesicSource::BottomBoundary,
            layer_height: 3.0,
            heat_timestep_factor: 1.0,
            bottom_tolerance: 0.5,
        };
        let layers = geodesic_slice(&mesh, &config);
        // Should get some layers (cube is 10mm tall, geodesic dist ~10mm, so ~3 layers at 3mm spacing)
        assert!(!layers.is_empty(), "Expected at least one layer from cube geodesic slice");
        // Each layer should have at least one contour
        for layer in &layers {
            assert!(!layer.contours.is_empty(), "Layer at d={:.1} has no contours", layer.z);
        }
    }
}
