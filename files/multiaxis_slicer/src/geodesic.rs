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
    /// Multiplier on avg_edge_length² for the finest-scale heat timestep.
    /// In multi-scale mode this is the base (smallest) factor; larger scales are added
    /// automatically at 2× increments until full mesh coverage is achieved.
    pub heat_timestep_factor: f64,
    /// Z tolerance for bottom boundary detection in mm (default 0.1).
    pub bottom_tolerance: f64,
    /// Use multi-scale heat method: run at several timesteps and fuse per-vertex
    /// using the finest scale that still reaches each vertex.  Gives fine detail in
    /// thin features (feet, ears) AND full-mesh coverage simultaneously.
    pub use_multiscale: bool,
    /// Number of doubling scales in multi-scale mode (default 6 = factor × [1,2,4,8,16,32]).
    pub num_scales: usize,
    /// Use adaptive (variable-diffusivity) heat method.
    /// Per-face kappa = adaptive_kappa_base × (avg_edge_length)².
    /// Small faces (thin features) get small κ → heat slows down → sharp local gradients.
    /// Large faces (smooth regions) get large κ → heat spreads fast → full coverage.
    /// Can be combined with multi-scale for best results.
    pub use_adaptive: bool,
    /// Base scaling factor for per-face diffusivity κ (default 6.0).
    /// Increase for faster spread in large regions; decrease to preserve more feature detail.
    pub adaptive_kappa_base: f64,
}

impl Default for GeodesicSlicerConfig {
    fn default() -> Self {
        Self {
            source: GeodesicSource::BottomBoundary,
            layer_height: 0.2,
            heat_timestep_factor: 1.0,
            bottom_tolerance: 0.1,
            use_multiscale: true,
            num_scales: 6,
            use_adaptive: false,
            adaptive_kappa_base: 6.0,
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

/// Lumped mass matrix M (diagonal): area/3 per adjacent face.
fn compute_lumped_mass(topo: &MeshTopology) -> Vec<f64> {
    let n = topo.num_vertices();
    let mut mass = vec![0.0f64; n];
    for face in &topo.faces {
        let [i, j, k] = *face;
        let area = (topo.vertices[j] - topo.vertices[i])
            .cross(&(topo.vertices[k] - topo.vertices[i]))
            .norm() * 0.5;
        if area < 1e-15 { continue; }
        let third = area / 3.0;
        mass[i] += third;
        mass[j] += third;
        mass[k] += third;
    }
    for m in &mut mass {
        if *m < 1e-12 { *m = 1e-12; }
    }
    mass
}

/// Build the cotangent stiffness matrix, with an optional per-face diffusivity κ.
/// If `kappa_per_face` is empty, κ=1 everywhere (standard isotropic Laplacian).
/// Adaptive usage: supply κ(f) = base_factor × (avg_edge_of_f)² so that the
/// heat equation diffuses slowly in small-feature regions (toes) and quickly in
/// large-feature regions (body), preserving detail while achieving full coverage.
fn build_cotangent_stiffness(topo: &MeshTopology, kappa_per_face: &[f64]) -> SparseSymmetric {
    let n = topo.num_vertices();
    let mut edge_weights: HashMap<(usize, usize), f64> = HashMap::new();
    let mut diag = vec![0.0f64; n];

    let add_edge = |map: &mut HashMap<(usize, usize), f64>, a: usize, b: usize, w: f64| {
        let key = if a < b { (a, b) } else { (b, a) };
        *map.entry(key).or_insert(0.0) += w;
    };

    for (f_idx, face) in topo.faces.iter().enumerate() {
        let kappa = kappa_per_face.get(f_idx).copied().unwrap_or(1.0);
        let [i, j, k] = *face;
        let vi = topo.vertices[i];
        let vj = topo.vertices[j];
        let vk = topo.vertices[k];

        let area = (vj - vi).cross(&(vk - vi)).norm() * 0.5;
        if area < 1e-15 { continue; }

        let clamp = |c: f64| c.clamp(-100.0, 100.0);
        let cot_i = clamp({ let a = vj-vi; let b = vk-vi; a.dot(&b) / a.cross(&b).norm().max(1e-12) });
        let cot_j = clamp({ let a = vi-vj; let b = vk-vj; a.dot(&b) / a.cross(&b).norm().max(1e-12) });
        let cot_k = clamp({ let a = vi-vk; let b = vj-vk; a.dot(&b) / a.cross(&b).norm().max(1e-12) });

        add_edge(&mut edge_weights, j, k, 0.5 * cot_i * kappa);
        add_edge(&mut edge_weights, i, k, 0.5 * cot_j * kappa);
        add_edge(&mut edge_weights, i, j, 0.5 * cot_k * kappa);
    }

    let mut off_diag: Vec<(usize, usize, f64)> = Vec::with_capacity(edge_weights.len());
    for (&(i, j), &w) in &edge_weights {
        off_diag.push((i, j, -w));
        diag[i] += w;
        diag[j] += w;
    }
    SparseSymmetric { n, diag, off_diag }
}

/// Build the cotangent Laplacian L and lumped mass matrix M.
/// Returns (mass_diagonal, laplacian_as_sparse_symmetric).
fn build_cotangent_laplacian(topo: &MeshTopology) -> (Vec<f64>, SparseSymmetric) {
    let mass = compute_lumped_mass(topo);
    let laplacian = build_cotangent_stiffness(topo, &[]);
    (mass, laplacian)
}

/// Per-face diffusivity κ for the adaptive heat method.
/// κ(f) = base_factor × (avg_edge_length(f))², clamped to [0.05, 50].
/// This makes heat diffuse proportionally to local feature size:
///   – Small triangles (thin features) → small κ → slow diffusion → sharp contours.
///   – Large triangles (smooth body) → large κ → fast diffusion → full coverage.
fn compute_kappa_per_face(topo: &MeshTopology, base_factor: f64) -> Vec<f64> {
    topo.faces.iter().map(|face| {
        let [i, j, k] = *face;
        let e0 = (topo.vertices[j] - topo.vertices[i]).norm();
        let e1 = (topo.vertices[k] - topo.vertices[j]).norm();
        let e2 = (topo.vertices[i] - topo.vertices[k]).norm();
        let avg_sq = ((e0 + e1 + e2) / 3.0).powi(2);
        (base_factor * avg_sq).clamp(0.05, 50.0)
    }).collect()
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

/// Average edge length across all faces.
fn compute_avg_edge(topo: &MeshTopology) -> f64 {
    let mut total = 0.0;
    let mut count = 0usize;
    for face in &topo.faces {
        for k in 0..3 {
            let a = face[k];
            let b = face[(k + 1) % 3];
            if a < b {
                total += (topo.vertices[b] - topo.vertices[a]).norm();
                count += 1;
            }
        }
    }
    if count > 0 { total / count as f64 } else { 1.0 }
}

/// Run one heat + Poisson pass at a fixed timestep `t`.
///
/// `heat_laplacian`   — stiffness used for the backward-Euler heat step.
///                      Pass the **adaptive** Laplacian (with per-face κ) here when using
///                      the adaptive heat method; the isotropic Laplacian otherwise.
/// `poisson_laplacian` — stiffness used for the Poisson (∇φ = div X) step.
///                       Always pass the **isotropic** Laplacian here; this ensures the
///                       distance field is defined in Euclidean geometry regardless of κ.
///
/// Returns `(u_heat, distances)`:
///   - `u_heat` — raw heat values (needed for multi-scale coverage detection)
///   - `distances` — geodesic distances, min-shifted to 0 at sources
fn geodesic_at_t(
    topo: &MeshTopology,
    mass: &[f64],
    heat_laplacian: &SparseSymmetric,
    poisson_laplacian: &SparseSymmetric,
    sources: &[usize],
    t: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = topo.num_vertices();

    // Step 1: Build heat system (M + t*L_heat) and RHS = M*delta
    // The heat equation backward Euler: (M + t*L) u = M * delta_S
    let heat_diag: Vec<f64> = (0..n).map(|i| mass[i] + t * heat_laplacian.diag[i]).collect();
    let heat_off_diag: Vec<(usize, usize, f64)> = heat_laplacian.off_diag.iter()
        .map(|&(i, j, val)| (i, j, t * val))
        .collect();
    let heat_system = SparseSymmetric { n, diag: heat_diag, off_diag: heat_off_diag };

    let mut rhs = vec![0.0; n];
    for &s in sources {
        rhs[s] = mass[s];
    }

    let u = conjugate_gradient(&heat_system, &rhs, 2000, 1e-8);

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

        let area = area2 * 0.5;

        // Gradient of scalar field: ∇u = (1/(2A)) * Σ u_i * (N_hat × e_opposite_i)
        // n_hat = face_normal / area2 is the unit normal (area2 = |face_normal| = 2A)
        let grad_u = (1.0 / (2.0 * area)) * (
            u[i] * n_hat.cross(&e_jk)
          + u[j] * n_hat.cross(&e_ki)
          + u[k] * n_hat.cross(&e_ij)
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

    // Step 4: Solve Poisson equation L*phi = div using the ISOTROPIC Laplacian.
    // (Using poisson_laplacian, not heat_laplacian — distance must be Euclidean.)
    let mut poisson_diag = poisson_laplacian.diag.clone();
    poisson_diag[0] += 1e-6; // Pin vertex 0 to make system non-singular

    let poisson_system = SparseSymmetric {
        n,
        diag: poisson_diag,
        off_diag: poisson_laplacian.off_diag.clone(),
    };

    let phi = conjugate_gradient(&poisson_system, &div, 2000, 1e-8);

    // Step 5: Shift so source vertices have distance = 0.
    // Do NOT use abs() — that creates a mirror zero-crossing on the far side of the field,
    // producing level sets that expand from both source AND anti-source simultaneously.
    let source_mean: f64 = sources.iter().map(|&s| phi[s]).sum::<f64>() / sources.len() as f64;
    let mut distances: Vec<f64> = phi.iter().map(|&p| p - source_mean).collect();
    // The Poisson solve may flip sign — check if source vertices have above-average distances.
    // Distance should INCREASE away from sources, so sources must have the MINIMUM.
    let source_avg: f64 = sources.iter().map(|&s| distances[s]).sum::<f64>() / sources.len() as f64;
    let global_avg: f64 = distances.iter().sum::<f64>() / distances.len() as f64;
    if source_avg > global_avg {
        // Poisson gave decreasing phi from source — negate so sources become minimum
        for d in &mut distances {
            *d = -*d;
        }
    }
    // Clamp numerical negatives (vertices with phi slightly below source_mean due to
    // solver error) to zero — treat them as "at source" rather than as far-away points
    for d in &mut distances {
        if *d < 0.0 { *d = 0.0; }
    }
    // Shift so minimum = 0 (should already be ~0 after clamp, but ensure exact zero)
    let min_d = distances.iter().cloned().fold(f64::INFINITY, f64::min);
    for d in &mut distances {
        *d -= min_d;
    }

    (u, distances)
}

/// Single-scale geodesic distance.
fn compute_geodesic_distance(
    topo: &MeshTopology,
    mass: &[f64],
    heat_laplacian: &SparseSymmetric,
    poisson_laplacian: &SparseSymmetric,
    sources: &[usize],
    heat_timestep_factor: f64,
) -> Vec<f64> {
    let avg_edge = compute_avg_edge(topo);
    let t = heat_timestep_factor * avg_edge * avg_edge;
    log::info!("  Heat method: avg_edge={:.4}mm, timestep t={:.6}", avg_edge, t);
    let (_u, phi) = geodesic_at_t(topo, mass, heat_laplacian, poisson_laplacian, sources, t);
    let max_dist = phi.iter().cloned().fold(0.0f64, f64::max);
    log::info!("  Geodesic distances: min=0.0, max={:.3}mm, {} source vertices",
        max_dist, sources.len());
    phi
}

/// Multi-scale geodesic distance fusion.
///
/// Runs `num_scales` heat+Poisson solves at doubling timesteps
/// `base_factor × [1, 2, 4, …, 2^(num_scales-1)] × avg_edge²`.
///
/// `heat_laplacian`    — used for the backward-Euler heat step (may be adaptive).
/// `poisson_laplacian` — used for the Poisson step (always isotropic).
///
/// For each vertex we pick the **finest** (smallest t) scale at which the
/// heat field is non-negligible (> 1e-6 × peak heat).  This gives:
///   • Accurate, locally-detailed distances near the source (small t)
///   • Full-mesh coverage for distant vertices (large t kicks in)
///
/// Result: detail in thin features (toes, ears) + global coverage — no
/// single timestep can achieve both simultaneously.
fn compute_multiscale_geodesic_distance(
    topo: &MeshTopology,
    mass: &[f64],
    heat_laplacian: &SparseSymmetric,
    poisson_laplacian: &SparseSymmetric,
    sources: &[usize],
    base_factor: f64,
    num_scales: usize,
) -> Vec<f64> {
    let n = topo.num_vertices();
    let avg_edge = compute_avg_edge(topo);
    let avg_sq = avg_edge * avg_edge;

    log::info!("  Multi-scale: avg_edge={:.4}mm, {} scales, base_factor={:.1}",
        avg_edge, num_scales, base_factor);

    let mut all_heat: Vec<Vec<f64>> = Vec::with_capacity(num_scales);
    let mut all_phi:  Vec<Vec<f64>> = Vec::with_capacity(num_scales);
    let mut max_heat: Vec<f64>      = Vec::with_capacity(num_scales);

    for k in 0..num_scales {
        let factor = base_factor * (1usize << k) as f64;
        let t = factor * avg_sq;
        let (u, phi) = geodesic_at_t(topo, mass, heat_laplacian, poisson_laplacian, sources, t);
        let max_u = u.iter().cloned().fold(0.0f64, f64::max);
        let max_d = phi.iter().cloned().fold(0.0f64, f64::max);
        log::info!("    Scale {}: factor={:.1}, t={:.3}mm², max_heat={:.3e}, max_dist={:.2}mm",
            k, factor, t, max_u, max_d);
        max_heat.push(max_u);
        all_heat.push(u);
        all_phi.push(phi);
    }

    // Per-vertex: use finest scale where heat value > 1e-6 × peak heat at that scale.
    // Finer scale = smaller t = more accurate local distances.
    const THRESHOLD: f64 = 1e-6;
    let mut final_phi = vec![0.0f64; n];
    for v in 0..n {
        let mut selected = false;
        for k in 0..num_scales {
            if max_heat[k] > 0.0 && all_heat[k][v] > THRESHOLD * max_heat[k] {
                final_phi[v] = all_phi[k][v];
                selected = true;
                break;
            }
        }
        if !selected {
            // No scale reached this vertex — fall back to coarsest available
            final_phi[v] = all_phi[num_scales - 1][v];
        }
    }

    // Ensure minimum distance is exactly 0
    let min_d = final_phi.iter().cloned().fold(f64::INFINITY, f64::min);
    if min_d > 0.0 { for d in &mut final_phi { *d -= min_d; } }

    let max_dist = final_phi.iter().cloned().fold(0.0f64, f64::max);
    log::info!("  Multi-scale geodesic: max_dist={:.3}mm", max_dist);
    final_phi
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

    // Step 4: Compute geodesic distances (single-scale or multi-scale, isotropic or adaptive).
    //
    // Adaptive mode (use_adaptive=true): build an adaptive Laplacian where each face's
    // diffusivity κ(f) = adaptive_kappa_base × (avg_edge_length(f))².  Small triangles
    // (thin features, toes) get small κ → slow diffusion → sharp local gradients.
    // Large triangles (body) get large κ → fast spread → full coverage.
    // The Poisson step always uses the ISOTROPIC Laplacian so distances are Euclidean.
    let adapt_lap_opt: Option<SparseSymmetric> = if config.use_adaptive {
        log::info!("  Adaptive mode: building per-face kappa Laplacian (base={:.1})...",
            config.adaptive_kappa_base);
        let kappa = compute_kappa_per_face(&topo, config.adaptive_kappa_base);
        let kappa_min = kappa.iter().cloned().fold(f64::INFINITY, f64::min);
        let kappa_max = kappa.iter().cloned().fold(0.0f64, f64::max);
        log::info!("  κ range: [{:.3}, {:.3}]", kappa_min, kappa_max);
        Some(build_cotangent_stiffness(&topo, &kappa))
    } else {
        None
    };
    let heat_lap = adapt_lap_opt.as_ref().unwrap_or(&laplacian);

    let distances = if config.use_multiscale {
        log::info!("Step 4: Computing MULTI-SCALE{}geodesic distances ({} scales, base factor={:.1})...",
            if config.use_adaptive { " ADAPTIVE " } else { " " },
            config.num_scales, config.heat_timestep_factor);
        compute_multiscale_geodesic_distance(
            &topo, &mass, heat_lap, &laplacian, &sources,
            config.heat_timestep_factor, config.num_scales,
        )
    } else {
        log::info!("Step 4: Computing{}geodesic distance field (Heat Method)...",
            if config.use_adaptive { " ADAPTIVE " } else { " " });
        compute_geodesic_distance(
            &topo, &mass, heat_lap, &laplacian, &sources, config.heat_timestep_factor
        )
    };

    // Step 5: Extract level-set layers
    log::info!("Step 5: Extracting geodesic layers...");
    let max_dist = distances.iter().cloned().fold(0.0f64, f64::max);

    // The source vertices define the "bottom" plane. Geodesic from the bottom boundary
    // wraps all the way around a closed mesh, producing distances >> model height.
    // Cap at the model's Z range so we only produce layers for the "outward" wavefront.
    let source_z_min = sources.iter()
        .map(|&s| topo.vertices[s].z)
        .fold(f64::INFINITY, f64::min);
    let model_z_max = topo.vertices.iter().map(|v| v.z).fold(f64::NEG_INFINITY, f64::max);
    let z_range = (model_z_max - source_z_min).max(config.layer_height);
    let effective_max_dist = max_dist.min(z_range);
    log::info!("  Z range: {:.2}mm, geodesic max: {:.2}mm, effective cap: {:.2}mm",
        z_range, max_dist, effective_max_dist);

    let num_layers = (effective_max_dist / config.layer_height).ceil() as usize;
    if num_layers == 0 {
        log::warn!("  Effective max geodesic distance is 0, no layers to extract");
        return Vec::new();
    }

    let mut layers = Vec::with_capacity(num_layers);
    for i in 1..=num_layers {
        let level = i as f64 * config.layer_height;
        if level > effective_max_dist {
            break;
        }
        let raw_contours = extract_level_set(&topo, &distances, level);

        // Filter 1: drop contours that lie entirely on the source plane (flat-base flooding).
        // Filter 2: drop contours that contain an artifact segment — both endpoints at z_min
        //           AND XY distance > 10mm.  Same bad-STL-triangle issue as conical mode.
        const ARTIFACT_Z_EPS: f64 = 0.001;
        const ARTIFACT_SEG_SQ: f64 = 100.0; // (10mm)²
        let contours: Vec<_> = raw_contours.into_iter()
            .filter(|c| {
                // Keep only contours with at least one point above the source plane
                if !c.points.iter().any(|p| p.z > source_z_min + ARTIFACT_Z_EPS) {
                    return false;
                }
                // Discard contours containing artifact segments (long jumps at z_min)
                let pts = &c.points;
                let n = pts.len();
                for i in 0..n {
                    let p1 = &pts[i];
                    let p2 = &pts[(i + 1) % n];
                    if p1.z < source_z_min + ARTIFACT_Z_EPS && p2.z < source_z_min + ARTIFACT_Z_EPS {
                        let dx = p2.x - p1.x;
                        let dy = p2.y - p1.y;
                        if dx * dx + dy * dy > ARTIFACT_SEG_SQ { return false; }
                    }
                }
                true
            })
            .collect();

        if !contours.is_empty() {
            // Use average Z of contour points so the layer is ordered by print height.
            let z_sum: f64 = contours.iter().flat_map(|c| c.points.iter()).map(|p| p.z).sum();
            let z_cnt: usize = contours.iter().map(|c| c.points.len()).sum();
            let avg_z = if z_cnt > 0 { z_sum / z_cnt as f64 } else { source_z_min + level };
            layers.push(Layer::new(avg_z, contours, config.layer_height));
        }
    }

    // Keep layers in geodesic distance order (level 1, 2, 3, …).
    // A Z-sort was previously used here but caused "layers start from top AND bottom and meet in
    // the middle": surfaces reached by long geodesic paths (e.g. the inside of a hull) have large
    // geodesic distance but low Z; Z-sorting promoted those to the front of the print, producing
    // the mirrored-wavefront artifact. Geodesic order guarantees each layer is one step further
    // from the source (base), which is the correct bottom-up surface-following sequence.

    // Deduplicate consecutive layers at nearly identical Z heights (merge contours)
    let mut merged_layers: Vec<Layer> = Vec::new();
    for layer in layers {
        if let Some(last) = merged_layers.last_mut() {
            if (layer.z - last.z).abs() < config.layer_height * 0.1 {
                // Merge contours into existing layer
                last.contours.extend(layer.contours);
                continue;
            }
        }
        merged_layers.push(layer);
    }

    log::info!("=== Geodesic slicing complete: {} layers (sorted by Z) ===", merged_layers.len());
    merged_layers
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

        let distances = compute_geodesic_distance(&topo, &mass, &laplacian, &laplacian, &[origin_idx], 1.0);

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
            use_multiscale: false,
            num_scales: 4,
            use_adaptive: false,
            adaptive_kappa_base: 6.0,
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
