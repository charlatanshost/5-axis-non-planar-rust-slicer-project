// Heat Method for Computing Geodesic Distances on Surface Meshes
// Based on "The Heat Method for Distance Computation" (Crane et al.)
//
// Algorithm (3 stages):
// 1. Heat Diffusion: Solve (I - t·M⁻¹·L)u = u₀
// 2. Vector Field: Compute normalized gradients X = -∇u / |∇u|
// 3. Distance Recovery: Solve Poisson equation Δφ = ∇·X

use crate::geometry::{Point3D, Vector3D, Triangle};
use crate::mesh::Mesh;
use sprs::{CsMat, TriMat};
use rayon::prelude::*;
use std::collections::HashMap;

/// Configuration for heat method
#[derive(Debug, Clone)]
pub struct HeatMethodConfig {
    /// Time parameter for heat diffusion (typically mean_edge_length²)
    pub time_step: f64,

    /// Source vertices (indices) for distance computation
    pub sources: Vec<usize>,

    /// Whether to use implicit (true) or explicit (false) time integration
    pub use_implicit: bool,

    /// Number of smoothing iterations for final distance field
    pub smoothing_iterations: usize,
}

impl Default for HeatMethodConfig {
    fn default() -> Self {
        Self {
            time_step: 0.0, // Will be computed from mean edge length
            sources: vec![],
            use_implicit: true,
            smoothing_iterations: 3,
        }
    }
}

/// Result of heat method computation
pub struct HeatMethodResult {
    /// Geodesic distance for each vertex
    pub distances: Vec<f64>,

    /// Per-triangle scalar values (average of vertices)
    pub triangle_scalars: Vec<f64>,

    /// Minimum distance
    pub min_distance: f64,

    /// Maximum distance
    pub max_distance: f64,
}

/// Compute geodesic distances using the heat method
pub fn compute_geodesic_distances(
    mesh: &Mesh,
    mut config: HeatMethodConfig,
) -> HeatMethodResult {
    log::info!("Computing geodesic distances via heat method...");

    // Build vertex-based representation
    let (vertices, vertex_map) = build_vertex_list(mesh);
    let n = vertices.len();

    log::info!("  Mesh has {} unique vertices", n);

    // Compute time step if not provided
    if config.time_step == 0.0 {
        config.time_step = compute_mean_edge_length_squared(mesh, &vertices, &vertex_map);
        log::info!("  Auto time step: {:.6}", config.time_step);
    }

    // Step 1: Build cotangent Laplacian and mass matrix
    log::info!("  Building cotangent Laplacian matrix...");
    let laplacian = build_cotangent_laplacian(mesh, &vertices, &vertex_map);
    let mass_matrix = build_mass_matrix(mesh, &vertices, &vertex_map);

    // Step 2: Solve heat diffusion
    log::info!("  Solving heat diffusion...");
    let heat = solve_heat_diffusion(&laplacian, &mass_matrix, config.time_step, &config.sources, n, config.use_implicit);

    // Debug: Check for NaN/inf in heat values
    let nan_count = heat.iter().filter(|&&h| !h.is_finite()).count();
    let nonzero_count = heat.iter().filter(|&&h| h.abs() > 1e-10).count();
    let max_heat = heat.iter().copied().fold(0.0f64, f64::max);
    log::info!("  Heat diffusion result: {} nonzero values, max = {:.6}, {} non-finite",
        nonzero_count, max_heat, nan_count);
    if nan_count > 0 {
        log::warn!("  WARNING: {} heat values are non-finite!", nan_count);
    }

    // Step 3: Compute normalized gradient field
    log::info!("  Computing gradient field...");
    let gradients = compute_gradient_field(mesh, &heat, &vertices, &vertex_map);

    // Debug: Check for NaN/inf in gradients
    let bad_grad_count = gradients.iter().filter(|g| !g.x.is_finite() || !g.y.is_finite() || !g.z.is_finite()).count();
    if bad_grad_count > 0 {
        log::warn!("  WARNING: {} gradients are non-finite!", bad_grad_count);
    }

    // Step 4: Recover distances via Poisson solve
    log::info!("  Recovering distances (Poisson solve)...");
    let distances = recover_distances(&laplacian, &gradients, mesh, &vertices, &vertex_map);

    // Normalize distances
    let min_distance = distances.iter().copied().fold(f64::INFINITY, f64::min);
    let max_distance = distances.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    // Compute per-triangle scalars
    let triangle_scalars = compute_triangle_scalars(mesh, &distances, &vertex_map);

    log::info!("  Distance range: [{:.3}, {:.3}]", min_distance, max_distance);

    HeatMethodResult {
        distances,
        triangle_scalars,
        min_distance,
        max_distance,
    }
}

/// Build unique vertex list and mapping
fn build_vertex_list(mesh: &Mesh) -> (Vec<Point3D>, HashMap<u64, usize>) {
    let mut vertices = Vec::new();
    let mut vertex_map = HashMap::new();

    for triangle in &mesh.triangles {
        for &vertex in &[triangle.v0, triangle.v1, triangle.v2] {
            let hash = hash_point(&vertex);
            vertex_map.entry(hash).or_insert_with(|| {
                let idx = vertices.len();
                vertices.push(vertex);
                idx
            });
        }
    }

    (vertices, vertex_map)
}

/// Build cotangent-weighted Laplacian matrix
/// Formula: weight(i,j) = (cot α + cot β) / 2
/// where α and β are angles opposite to edge (i,j)
fn build_cotangent_laplacian(
    mesh: &Mesh,
    vertices: &[Point3D],
    vertex_map: &HashMap<u64, usize>,
) -> CsMat<f64> {
    let n = vertices.len();
    let mut triplet_mat = TriMat::new((n, n));

    // Build edge weights from triangles
    let mut edge_weights: HashMap<(usize, usize), f64> = HashMap::new();

    for triangle in &mesh.triangles {
        let v0_idx = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
        let v1_idx = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
        let v2_idx = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

        // Compute cotangent weights for each edge
        let edges = [(v0_idx, v1_idx, v2_idx), (v1_idx, v2_idx, v0_idx), (v2_idx, v0_idx, v1_idx)];

        for &(i, j, k) in &edges {
            let pi = vertices[i];
            let pj = vertices[j];
            let pk = vertices[k];

            // Cotangent of angle at pk opposite to edge (i,j)
            let cot = compute_cotangent(&pi, &pj, &pk);

            // Add to edge weight (sum over adjacent triangles)
            let edge = if i < j { (i, j) } else { (j, i) };
            *edge_weights.entry(edge).or_insert(0.0) += cot / 2.0;
        }
    }

    // Build Laplacian matrix from edge weights
    for (&(i, j), &weight) in &edge_weights {
        triplet_mat.add_triplet(i, j, -weight);
        triplet_mat.add_triplet(j, i, -weight);
    }

    // Diagonal entries (negative sum of off-diagonal)
    let mut diagonal = vec![0.0; n];
    for (&(i, j), &weight) in &edge_weights {
        diagonal[i] += weight;
        diagonal[j] += weight;
    }

    for (i, &d) in diagonal.iter().enumerate() {
        triplet_mat.add_triplet(i, i, d);
    }

    // Convert to CSR format
    triplet_mat.to_csr()
}

/// Build mass matrix (diagonal matrix of vertex areas)
/// Each vertex gets 1/3 of the area of adjacent triangles
fn build_mass_matrix(
    mesh: &Mesh,
    _vertices: &[Point3D],
    vertex_map: &HashMap<u64, usize>,
) -> Vec<f64> {
    let n = vertex_map.len();
    let mut mass = vec![0.0; n];

    for triangle in &mesh.triangles {
        let area = triangle.area();
        let v0_idx = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
        let v1_idx = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
        let v2_idx = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

        // Each vertex gets 1/3 of triangle area
        mass[v0_idx] += area / 3.0;
        mass[v1_idx] += area / 3.0;
        mass[v2_idx] += area / 3.0;
    }

    mass
}

/// Compute cotangent of angle at pk opposite to edge (pi, pj)
fn compute_cotangent(pi: &Point3D, pj: &Point3D, pk: &Point3D) -> f64 {
    let u = *pi - *pk;
    let v = *pj - *pk;

    let dot = u.dot(&v);
    let cross = u.cross(&v).norm();

    if cross < 1e-10 {
        return 0.0; // Degenerate triangle
    }

    dot / cross
}

/// Compute mean edge length squared for time step
fn compute_mean_edge_length_squared(
    mesh: &Mesh,
    _vertices: &[Point3D],
    vertex_map: &HashMap<u64, usize>,
) -> f64 {
    let mut sum = 0.0;
    let mut count = 0;
    let mut edges = std::collections::HashSet::new();

    for triangle in &mesh.triangles {
        let v0_idx = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
        let v1_idx = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
        let v2_idx = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

        for &(i, j) in &[(v0_idx, v1_idx), (v1_idx, v2_idx), (v2_idx, v0_idx)] {
            let edge = if i < j { (i, j) } else { (j, i) };
            if edges.insert(edge) {
                let length_sq = (triangle.v0 - triangle.v1).norm_squared();
                sum += length_sq;
                count += 1;
            }
        }
    }

    sum / count as f64
}

/// Solve heat diffusion: (I - t·M⁻¹·L)u = u₀
fn solve_heat_diffusion(
    laplacian: &CsMat<f64>,
    mass: &[f64],
    time_step: f64,
    sources: &[usize],
    n: usize,
    use_implicit: bool,
) -> Vec<f64> {
    // Initialize with source condition
    let mut u = vec![0.0; n];
    for &source in sources {
        if source < n {
            u[source] = 1.0;
        }
    }

    if use_implicit {
        // Implicit time integration: (M - t·L)·u_new = M·u_old
        // More stable but requires solving a linear system
        use sprs::TriMat;

        // Build system matrix: M - t·L
        // For implicit heat method: (M + t·L)·u = M·δ (since L is negative semi-definite)
        let mut trimat = TriMat::new((n, n));

        for i in 0..n {
            // Diagonal: mass + t * diagonal_of_L
            if let Some(row) = laplacian.outer_view(i) {
                let mut diag_val = mass[i];
                for (col_idx, &val) in row.iter() {
                    if i == col_idx {
                        diag_val += time_step * val;
                    }
                }
                if diag_val.abs() > 1e-10 {
                    trimat.add_triplet(i, i, diag_val);
                }
            }

            // Off-diagonal: t * off_diagonal_of_L
            if let Some(row) = laplacian.outer_view(i) {
                for (col_idx, &val) in row.iter() {
                    if i != col_idx && val.abs() > 1e-10 {
                        trimat.add_triplet(i, col_idx, time_step * val);
                    }
                }
            }
        }

        // Build sparse matrix
        let system_mat: sprs::CsMat<f64> = trimat.to_csr();

        // Right-hand side: M·u (where u has 1.0 at sources, 0.0 elsewhere)
        let mut rhs = vec![0.0; n];
        for i in 0..n {
            rhs[i] = mass[i] * u[i];
        }

        // Solve using iterative method (Conjugate Gradient)
        // For now, use simple Gauss-Seidel iteration
        let mut u_new = u.clone();
        for _iter in 0..100 {
            let mut max_change: f64 = 0.0;
            for i in 0..n {
                let mut sum = rhs[i];
                let mut diag = 1.0;

                if let Some(row) = system_mat.outer_view(i) {
                    for (col_idx, &val) in row.iter() {
                        if i == col_idx {
                            diag = val;
                        } else {
                            sum -= val * u_new[col_idx];
                        }
                    }
                }

                if diag.abs() > 1e-10 {
                    let new_val = sum / diag;
                    max_change = max_change.max((new_val - u_new[i]).abs());
                    u_new[i] = new_val;
                }
            }

            // Check convergence
            if max_change < 1e-6 {
                break;
            }
        }

        u_new
    } else {
        // Explicit time integration: u_new = u_old + t·M⁻¹·L·u_old
        let mut u_new = u.clone();
        for i in 0..n {
            let mut laplacian_u = 0.0;

            // Get row from sparse matrix
            for (col_idx, &val) in laplacian.outer_view(i).unwrap().iter() {
                laplacian_u += val * u[col_idx];
            }

            // Safety check: prevent division by zero or very small mass
            if mass[i] > 1e-10 {
                let du = time_step * laplacian_u / mass[i];
                // Clamp to prevent instability
                let clamped_du = du.max(-10.0).min(10.0);
                u_new[i] = u[i] + clamped_du;
            } else {
                // Keep original value if mass is too small
                u_new[i] = u[i];
            }
        }

        u_new
    }
}

/// Compute per-face gradient field
fn compute_gradient_field(
    mesh: &Mesh,
    heat: &[f64],
    _vertices: &[Point3D],
    vertex_map: &HashMap<u64, usize>,
) -> Vec<Vector3D> {
    mesh.triangles
        .par_iter()
        .map(|triangle| {
            let v0_idx = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
            let v1_idx = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
            let v2_idx = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

            let u0 = heat[v0_idx];
            let u1 = heat[v1_idx];
            let u2 = heat[v2_idx];

            // Compute gradient using barycentric coordinates
            let e1 = triangle.v1 - triangle.v0;
            let e2 = triangle.v2 - triangle.v0;
            let normal = e1.cross(&e2);
            let area2 = normal.norm();

            if area2 < 1e-10 {
                return Vector3D::zeros();
            }

            let grad = (normal.cross(&e2) * (u1 - u0) + e1.cross(&normal) * (u2 - u0)) / area2;

            // Normalize
            let norm = grad.norm();
            if norm < 1e-10 {
                Vector3D::zeros()
            } else {
                -grad / norm // Negative for inward direction
            }
        })
        .collect()
}

/// Recover distances by solving Poisson equation: Δφ = ∇·X
fn recover_distances(
    laplacian: &CsMat<f64>,
    gradients: &[Vector3D],
    mesh: &Mesh,
    _vertices: &[Point3D],
    vertex_map: &HashMap<u64, usize>,
) -> Vec<f64> {
    let n = vertex_map.len();

    // Compute divergence at each vertex
    let mut divergence = vec![0.0; n];

    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        let v0_idx = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
        let v1_idx = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
        let v2_idx = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

        let grad = gradients[tri_idx];
        let area = triangle.area();
        let normal = triangle.normal();

        // Safety check: skip degenerate triangles
        if !area.is_finite() || area < 1e-10 {
            continue;
        }

        let div_contrib = grad.dot(&normal) * area / 3.0;

        // Safety check: skip if contribution is non-finite
        if !div_contrib.is_finite() {
            continue;
        }

        // Distribute divergence to vertices
        divergence[v0_idx] += div_contrib;
        divergence[v1_idx] += div_contrib;
        divergence[v2_idx] += div_contrib;
    }

    // Debug: Check for NaN in divergence
    let nan_div_count = divergence.iter().filter(|&&d| !d.is_finite()).count();
    if nan_div_count > 0 {
        log::warn!("  WARNING: {} divergence values are non-finite!", nan_div_count);
    }

    // Solve Laplacian system (simplified - use iterative method)
    // For now, use simple approximation
    let mut distances = vec![0.0; n];

    // Simple Jacobi iteration
    for _iter in 0..100 {
        let mut new_distances = distances.clone();
        for i in 0..n {
            let mut sum = 0.0;
            let mut weight_sum = 0.0;

            // Get row from sparse matrix
            if let Some(row) = laplacian.outer_view(i) {
                for (col_idx, &val) in row.iter() {
                    if i != col_idx {
                        sum += -val * distances[col_idx];
                        weight_sum += -val;
                    }
                }
            }

            if weight_sum > 1e-10 && divergence[i].is_finite() {
                let new_val = (divergence[i] - sum) / weight_sum;
                if new_val.is_finite() {
                    new_distances[i] = new_val;
                }
            }
        }
        distances = new_distances;
    }

    // Debug: Check distances after iteration
    let nan_dist_count = distances.iter().filter(|&&d| !d.is_finite()).count();
    if nan_dist_count > 0 {
        log::warn!("  WARNING: {} distance values are non-finite after Jacobi iteration!", nan_dist_count);
    }

    // Shift to make minimum zero
    let min = distances.iter().copied().fold(f64::INFINITY, f64::min);

    // Safety check: if min is not finite, something went wrong
    if !min.is_finite() {
        log::warn!("  WARNING: Heat method produced non-finite distances, returning zeros");
        return vec![0.0; n];
    }

    for d in &mut distances {
        *d -= min;
        // Clamp to reasonable range
        *d = d.max(0.0).min(1000.0);
    }

    distances
}

/// Compute per-triangle scalar values from vertex distances
fn compute_triangle_scalars(
    mesh: &Mesh,
    distances: &[f64],
    vertex_map: &HashMap<u64, usize>,
) -> Vec<f64> {
    mesh.triangles
        .par_iter()
        .map(|triangle| {
            let v0_idx = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
            let v1_idx = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
            let v2_idx = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

            (distances[v0_idx] + distances[v1_idx] + distances[v2_idx]) / 3.0
        })
        .collect()
}

/// Hash a point for vertex deduplication
fn hash_point(p: &Point3D) -> u64 {
    let x = (p.x * 1000000.0).round() as i64;
    let y = (p.y * 1000000.0).round() as i64;
    let z = (p.z * 1000000.0).round() as i64;

    let mut hash = 0u64;
    hash = hash.wrapping_mul(73856093) ^ (x as u64);
    hash = hash.wrapping_mul(19349663) ^ (y as u64);
    hash = hash.wrapping_mul(83492791) ^ (z as u64);
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cotangent_computation() {
        let p0 = Point3D::new(0.0, 0.0, 0.0);
        let p1 = Point3D::new(1.0, 0.0, 0.0);
        let p2 = Point3D::new(0.5, 0.866, 0.0); // 60 degree angle

        let cot = compute_cotangent(&p0, &p1, &p2);
        let expected = 1.0 / (60.0_f64.to_radians().tan());

        assert!((cot - expected).abs() < 0.01);
    }
}
