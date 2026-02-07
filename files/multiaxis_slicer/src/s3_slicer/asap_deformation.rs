// ASAP (As-Rigid-As-Possible) Deformation
// Based on Sorkine and Alexa "As-Rigid-As-Possible Surface Modeling" (2007)
//
// Algorithm:
// 1. Local step: Compute optimal rotation for each element via SVD
// 2. Global step: Solve sparse linear system for new vertex positions
// 3. Iterate until convergence
//
// Energy: E = Σ_i Σ_j∈N(i) w_ij ||p'_i - p'_j - R_i(p_i - p_j)||²
// Where:
// - p_i, p_j = rest pose positions
// - p'_i, p'_j = deformed positions
// - R_i = optimal rotation for cell containing edge (i,j)
// - w_ij = cotangent weight

use crate::geometry::{Point3D, Vector3D};
use crate::mesh::Mesh;
use crate::s3_slicer::quaternion_field::QuaternionField;
use nalgebra::{Matrix3, Vector3 as NVector3, SVD};
use sprs::{CsMat, TriMat};
use std::collections::HashMap;

/// Configuration for ASAP deformation
#[derive(Debug, Clone)]
pub struct AsapConfig {
    /// Maximum number of iterations
    pub max_iterations: usize,

    /// Convergence threshold (change in vertex positions)
    pub convergence_threshold: f64,

    /// Weight for quaternion field alignment
    pub quaternion_weight: f64,

    /// Weight for positional constraints
    pub constraint_weight: f64,

    /// Use cotangent weights (vs uniform)
    pub use_cotangent_weights: bool,
}

impl Default for AsapConfig {
    fn default() -> Self {
        Self {
            max_iterations: 10,
            convergence_threshold: 1e-4,
            quaternion_weight: 1.0,
            constraint_weight: 100.0,
            use_cotangent_weights: true,
        }
    }
}

/// ASAP deformation solver
pub struct AsapSolver {
    /// Original mesh
    original_mesh: Mesh,

    /// Quaternion field for rotation guidance
    quaternion_field: QuaternionField,

    /// Configuration
    config: AsapConfig,

    /// Vertex mapping (hash -> index)
    vertex_map: HashMap<u64, usize>,

    /// Unique vertices
    vertices: Vec<Point3D>,

    /// Rest-pose edge vectors for each triangle
    rest_edges: Vec<[Vector3D; 3]>,

    /// Cotangent weights for edges
    edge_weights: HashMap<(usize, usize), f64>,
}

impl AsapSolver {
    /// Create new ASAP solver
    pub fn new(mesh: Mesh, quaternion_field: QuaternionField, config: AsapConfig) -> Self {
        log::info!("Initializing ASAP deformation solver...");

        // Build vertex list and mapping
        let (vertices, vertex_map) = build_vertex_list(&mesh);
        log::info!("  Mesh has {} unique vertices", vertices.len());

        // Compute rest-pose edge vectors
        let rest_edges = compute_rest_edges(&mesh);

        // Compute edge weights (cotangent or uniform)
        let edge_weights = if config.use_cotangent_weights {
            compute_cotangent_weights(&mesh, &vertices, &vertex_map)
        } else {
            compute_uniform_weights(&mesh, &vertex_map)
        };

        log::info!("  Computed {} edge weights", edge_weights.len());

        Self {
            original_mesh: mesh,
            quaternion_field,
            config,
            vertex_map,
            vertices,
            rest_edges,
            edge_weights,
        }
    }

    /// Solve for deformed mesh
    pub fn solve(&self) -> Mesh {
        log::info!("Solving ASAP deformation ({} vertices)...", self.vertices.len());
        log::info!("  Max iterations: {}", self.config.max_iterations);
        log::info!("  Convergence threshold: {:.2e}", self.config.convergence_threshold);

        // Initialize deformed positions to rest positions
        let mut deformed_positions = self.vertices.clone();

        // ASAP iteration
        for iteration in 0..self.config.max_iterations {
            log::info!("  ASAP iteration {}/{}...", iteration + 1, self.config.max_iterations);

            // Local step: Compute optimal rotations
            log::info!("    Local step (computing rotations)...");
            let rotations = self.compute_optimal_rotations(&deformed_positions);

            // Global step: Solve for new positions
            log::info!("    Global step (solving linear system)...");
            let new_positions = self.solve_global_step(&deformed_positions, &rotations);

            // Check convergence
            let max_displacement = compute_max_displacement(&deformed_positions, &new_positions);
            log::info!("    Max displacement: {:.6}", max_displacement);

            deformed_positions = new_positions;

            if max_displacement < self.config.convergence_threshold {
                log::info!("  Converged after {} iterations", iteration + 1);
                break;
            }
        }

        log::info!("  Building deformed mesh...");
        // Build deformed mesh
        self.build_deformed_mesh(&deformed_positions)
    }

    /// Local step: Compute optimal rotation for each triangle
    fn compute_optimal_rotations(&self, deformed_positions: &[Point3D]) -> Vec<Matrix3<f64>> {
        self.original_mesh
            .triangles
            .iter()
            .enumerate()
            .map(|(tri_idx, triangle)| {
                // Get vertex indices
                let v0_idx = *self.vertex_map.get(&hash_point(&triangle.v0)).unwrap();
                let v1_idx = *self.vertex_map.get(&hash_point(&triangle.v1)).unwrap();
                let v2_idx = *self.vertex_map.get(&hash_point(&triangle.v2)).unwrap();

                // Get deformed positions
                let p0 = deformed_positions[v0_idx];
                let p1 = deformed_positions[v1_idx];
                let p2 = deformed_positions[v2_idx];

                // Compute deformed edge vectors
                let e1_def = p1 - p0;
                let e2_def = p2 - p0;

                // Get rest-pose edge vectors
                let rest_edges = &self.rest_edges[tri_idx];
                let e1_rest = rest_edges[0];
                let e2_rest = rest_edges[1];

                // Compute optimal rotation via SVD
                compute_optimal_rotation(e1_rest, e2_rest, e1_def, e2_def,
                                         &self.quaternion_field, tri_idx, self.config.quaternion_weight)
            })
            .collect()
    }

    /// Global step: Solve sparse linear system for new vertex positions
    fn solve_global_step(
        &self,
        current_positions: &[Point3D],
        rotations: &[Matrix3<f64>],
    ) -> Vec<Point3D> {
        let n = self.vertices.len();

        // Build sparse system matrix and RHS
        // System: L * p' = b
        // Where L is the Laplacian matrix and b incorporates rotations

        let mut triplets = TriMat::new((n, n));
        let mut rhs_x = vec![0.0; n];
        let mut rhs_y = vec![0.0; n];
        let mut rhs_z = vec![0.0; n];

        // Build system from triangles
        for (tri_idx, triangle) in self.original_mesh.triangles.iter().enumerate() {
            let v0_idx = *self.vertex_map.get(&hash_point(&triangle.v0)).unwrap();
            let v1_idx = *self.vertex_map.get(&hash_point(&triangle.v1)).unwrap();
            let v2_idx = *self.vertex_map.get(&hash_point(&triangle.v2)).unwrap();

            let rotation = &rotations[tri_idx];
            let rest_edges = &self.rest_edges[tri_idx];

            // Add contributions from each edge
            let edges = [
                (v0_idx, v1_idx, rest_edges[0]),
                (v0_idx, v2_idx, rest_edges[1]),
                (v1_idx, v2_idx, rest_edges[2]),
            ];

            for &(i, j, rest_edge) in &edges {
                let weight = self.edge_weights.get(&edge_key(i, j)).copied().unwrap_or(1.0);

                // Rotated rest edge
                let rotated = rotation * to_nalgebra(&rest_edge);

                // Add to system matrix
                triplets.add_triplet(i, i, weight);
                triplets.add_triplet(i, j, -weight);
                triplets.add_triplet(j, i, -weight);
                triplets.add_triplet(j, j, weight);

                // Add to RHS
                rhs_x[i] += weight * rotated[0];
                rhs_y[i] += weight * rotated[1];
                rhs_z[i] += weight * rotated[2];

                rhs_x[j] -= weight * rotated[0];
                rhs_y[j] -= weight * rotated[1];
                rhs_z[j] -= weight * rotated[2];
            }
        }

        // Add positional constraints (fix bottom vertices)
        // Use a height-based gradient: vertices at bottom are strongly fixed,
        // vertices higher up get progressively weaker constraints
        let min_z = self.vertices.iter().map(|v| v.z).fold(f64::INFINITY, f64::min);
        let max_z = self.vertices.iter().map(|v| v.z).fold(f64::NEG_INFINITY, f64::max);
        let height_range = (max_z - min_z).max(1.0);

        // Fixed layer thickness - vertices within this range are strongly fixed
        let fixed_layer_thickness = 2.0_f64.min(height_range * 0.1); // 2mm or 10% of height

        for (idx, vertex) in self.vertices.iter().enumerate() {
            let height_above_bottom = vertex.z - min_z;

            if height_above_bottom < fixed_layer_thickness {
                // Bottom layer: strongly fixed
                let weight = self.config.constraint_weight;
                triplets.add_triplet(idx, idx, weight);
                rhs_x[idx] += weight * vertex.x;
                rhs_y[idx] += weight * vertex.y;
                rhs_z[idx] += weight * vertex.z;
            } else {
                // Above bottom: add a soft regularization to prevent wild deformations
                // Weight decreases with height (further from bottom = more freedom to move)
                let height_fraction = height_above_bottom / height_range;
                let soft_weight = self.config.constraint_weight * 0.01 * (1.0 - height_fraction).max(0.0);

                if soft_weight > 0.1 {
                    triplets.add_triplet(idx, idx, soft_weight);
                    rhs_x[idx] += soft_weight * vertex.x;
                    rhs_y[idx] += soft_weight * vertex.y;
                    rhs_z[idx] += soft_weight * vertex.z;
                }
            }
        }

        // Convert to CSR and solve
        let system_matrix = triplets.to_csr();

        // Solve three systems (one per coordinate)
        let solution_x = solve_sparse_system(&system_matrix, &rhs_x);
        let solution_y = solve_sparse_system(&system_matrix, &rhs_y);
        let solution_z = solve_sparse_system(&system_matrix, &rhs_z);

        // Build result
        (0..n)
            .map(|i| Point3D::new(solution_x[i], solution_y[i], solution_z[i]))
            .collect()
    }

    /// Build deformed mesh from deformed vertex positions
    fn build_deformed_mesh(&self, deformed_positions: &[Point3D]) -> Mesh {
        // Safety check: Verify all deformed positions are finite
        let invalid_verts = deformed_positions.iter().enumerate()
            .filter(|(_, p)| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite())
            .collect::<Vec<_>>();

        if !invalid_verts.is_empty() {
            log::error!("  ERROR: {} deformed vertices have non-finite coordinates!", invalid_verts.len());
            for (idx, p) in invalid_verts.iter().take(5) {
                log::error!("    Vertex {}: ({}, {}, {})", idx, p.x, p.y, p.z);
            }
            log::error!("  Using original mesh instead of deformed mesh due to invalid coordinates");
            return self.original_mesh.clone();
        }

        let deformed_triangles = self
            .original_mesh
            .triangles
            .iter()
            .map(|triangle| {
                let v0_idx = *self.vertex_map.get(&hash_point(&triangle.v0)).unwrap();
                let v1_idx = *self.vertex_map.get(&hash_point(&triangle.v1)).unwrap();
                let v2_idx = *self.vertex_map.get(&hash_point(&triangle.v2)).unwrap();

                crate::geometry::Triangle {
                    v0: deformed_positions[v0_idx],
                    v1: deformed_positions[v1_idx],
                    v2: deformed_positions[v2_idx],
                }
            })
            .collect();

        Mesh::new(deformed_triangles).unwrap()
    }
}

/// Compute optimal rotation matrix via SVD
/// Aligns rest-pose edges with deformed edges, guided by quaternion field
///
/// The quaternion field provides a TARGET rotation that guides the deformation.
/// We blend the SVD-computed rotation (from edge correspondences) with the
/// quaternion target rotation using spherical interpolation.
///
/// This is the correct approach - we DON'T add quaternions to the covariance matrix.
fn compute_optimal_rotation(
    e1_rest: Vector3D,
    e2_rest: Vector3D,
    e1_def: Vector3D,
    e2_def: Vector3D,
    quaternion_field: &QuaternionField,
    tri_idx: usize,
    quaternion_weight: f64,
) -> Matrix3<f64> {
    // Step 1: Build covariance matrix from edge correspondences ONLY
    // C = Σ (deformed * rest^T)
    let mut covariance = Matrix3::zeros();

    // Add edge contributions
    for (rest, def) in [(e1_rest, e1_def), (e2_rest, e2_def)] {
        let rest_vec = to_nalgebra(&rest);
        let def_vec = to_nalgebra(&def);
        covariance += def_vec * rest_vec.transpose();
    }

    // Step 2: Extract optimal rotation from covariance via SVD
    let svd = SVD::new(covariance, true, true);

    let edge_rotation = if let (Some(u), Some(v_t)) = (svd.u, svd.v_t) {
        // Optimal rotation: R = U * V^T
        let mut rotation = u * v_t;

        // Ensure proper rotation (det = 1, not reflection)
        if rotation.determinant() < 0.0 {
            let mut u_corrected = u;
            u_corrected[(0, 2)] *= -1.0;
            u_corrected[(1, 2)] *= -1.0;
            u_corrected[(2, 2)] *= -1.0;
            rotation = u_corrected * v_t;
        }

        rotation
    } else {
        Matrix3::identity()
    };

    // Step 3: Blend with quaternion field target rotation
    // The quaternion field provides guidance, not a hard constraint
    if quaternion_weight > 0.0 && tri_idx < quaternion_field.rotations.len() {
        let target_quat = quaternion_field.rotations[tri_idx];

        // Skip if target is identity (no rotation guidance for this triangle)
        if target_quat.angle() < 1e-6 {
            return edge_rotation;
        }

        // Convert edge rotation to quaternion for blending
        let edge_rot_matrix = nalgebra::Rotation3::from_matrix_unchecked(edge_rotation);
        let edge_quat = nalgebra::UnitQuaternion::from_rotation_matrix(&edge_rot_matrix);

        // Spherical interpolation: blend edge-based rotation with quaternion field target
        // quaternion_weight of 0.3 means 30% guidance from quaternion field, 70% from edges
        let blended_quat = edge_quat.slerp(&target_quat, quaternion_weight.min(1.0));

        // Convert back to rotation matrix
        blended_quat.to_rotation_matrix().into_inner()
    } else {
        edge_rotation
    }
}

/// Solve sparse linear system using iterative method (Gauss-Seidel with early termination)
fn solve_sparse_system(matrix: &CsMat<f64>, rhs: &[f64]) -> Vec<f64> {
    let n = rhs.len();

    // Initialize with RHS values (better starting point than zeros)
    let mut solution: Vec<f64> = rhs.iter().map(|&r| {
        if r.is_finite() { r * 0.01 } else { 0.0 }
    }).collect();

    // Limit iterations based on mesh size - smaller meshes need fewer iterations
    let max_iterations = (20 + n / 500).min(50);
    let convergence_threshold = 1e-4;

    for iteration in 0..max_iterations {
        let mut max_change = 0.0f64;

        // Gauss-Seidel (use updated values immediately - faster convergence than Jacobi)
        for i in 0..n {
            let mut sum = rhs[i];
            let mut diagonal = 0.0;

            if let Some(row) = matrix.outer_view(i) {
                for (j, &value) in row.iter() {
                    if i == j {
                        diagonal = value;
                    } else {
                        sum -= value * solution[j];
                    }
                }
            }

            if diagonal.abs() > 1e-10 {
                let candidate = sum / diagonal;
                if candidate.is_finite() {
                    let change = (candidate - solution[i]).abs();
                    max_change = max_change.max(change);
                    solution[i] = candidate;
                }
            }
        }

        // Early termination if converged
        if max_change < convergence_threshold {
            break;
        }
    }

    // Final safety check
    for v in &mut solution {
        if !v.is_finite() {
            *v = 0.0;
        }
    }

    solution
}

/// Build vertex list and mapping
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

/// Compute rest-pose edge vectors for each triangle
fn compute_rest_edges(mesh: &Mesh) -> Vec<[Vector3D; 3]> {
    mesh.triangles
        .iter()
        .map(|triangle| {
            let e1 = triangle.v1 - triangle.v0;
            let e2 = triangle.v2 - triangle.v0;
            let e3 = triangle.v2 - triangle.v1;
            [e1, e2, e3]
        })
        .collect()
}

/// Compute cotangent weights for edges
fn compute_cotangent_weights(
    mesh: &Mesh,
    vertices: &[Point3D],
    vertex_map: &HashMap<u64, usize>,
) -> HashMap<(usize, usize), f64> {
    let mut weights = HashMap::new();

    for triangle in &mesh.triangles {
        let v0_idx = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
        let v1_idx = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
        let v2_idx = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

        // Compute cotangent weights
        let edges = [(v0_idx, v1_idx, v2_idx), (v0_idx, v2_idx, v1_idx), (v1_idx, v2_idx, v0_idx)];

        for &(i, j, k) in &edges {
            let pi = vertices[i];
            let pj = vertices[j];
            let pk = vertices[k];

            let cot = compute_cotangent(&pi, &pj, &pk);
            *weights.entry(edge_key(i, j)).or_insert(0.0) += cot / 2.0;
        }
    }

    weights
}

/// Compute uniform weights (1.0 for all edges)
fn compute_uniform_weights(mesh: &Mesh, vertex_map: &HashMap<u64, usize>) -> HashMap<(usize, usize), f64> {
    let mut weights = HashMap::new();

    for triangle in &mesh.triangles {
        let v0_idx = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
        let v1_idx = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
        let v2_idx = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

        weights.insert(edge_key(v0_idx, v1_idx), 1.0);
        weights.insert(edge_key(v0_idx, v2_idx), 1.0);
        weights.insert(edge_key(v1_idx, v2_idx), 1.0);
    }

    weights
}

/// Compute cotangent of angle at pk opposite to edge (pi, pj)
fn compute_cotangent(pi: &Point3D, pj: &Point3D, pk: &Point3D) -> f64 {
    let u = *pi - *pk;
    let v = *pj - *pk;

    let dot = u.dot(&v);
    let cross = u.cross(&v).norm();

    if cross < 1e-10 {
        return 0.0;
    }

    dot / cross
}

/// Compute maximum displacement between two position sets
fn compute_max_displacement(old_pos: &[Point3D], new_pos: &[Point3D]) -> f64 {
    old_pos
        .iter()
        .zip(new_pos.iter())
        .map(|(old, new)| (*old - *new).norm())
        .fold(0.0, f64::max)
}

/// Convert Vector3D to nalgebra Vector3
fn to_nalgebra(v: &Vector3D) -> NVector3<f64> {
    NVector3::new(v.x, v.y, v.z)
}

/// Convert quaternion to rotation matrix
fn quaternion_to_matrix(q: &nalgebra::UnitQuaternion<f64>) -> Matrix3<f64> {
    q.to_rotation_matrix().into_inner()
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

/// Create edge key (ordered)
fn edge_key(i: usize, j: usize) -> (usize, usize) {
    if i < j {
        (i, j)
    } else {
        (j, i)
    }
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

    #[test]
    fn test_optimal_rotation_identity() {
        use crate::s3_slicer::quaternion_field::QuaternionFieldConfig;

        let e1 = Vector3D::new(1.0, 0.0, 0.0);
        let e2 = Vector3D::new(0.0, 1.0, 0.0);

        // Same edges should give identity rotation
        let qfield = QuaternionField {
            rotations: vec![nalgebra::UnitQuaternion::identity()],
            energy: 0.0,
            config: QuaternionFieldConfig::default(),
        };

        let rotation = compute_optimal_rotation(e1, e2, e1, e2, &qfield, 0, 0.0);

        // Check if close to identity
        let identity = Matrix3::identity();
        let diff = (rotation - identity).norm();
        assert!(diff < 0.01);
    }
}
