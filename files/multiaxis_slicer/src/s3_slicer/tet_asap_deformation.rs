// Volumetric ASAP (As-Similar-As-Possible) Deformation for Tetrahedral Meshes
//
// Key difference from surface ARAP: allows per-tet SCALING in addition to rotation.
// This is critical for the S3-Slicer paper's approach where tets can stretch/compress.
//
// Algorithm:
// 1. Local step: For each tet, compute optimal rotation+scaling via SVD of deformation gradient
//    - F = D' * D^{-1} (deformed edges / rest edges)
//    - SVD(F) = U * S * V^T -> R = U*V^T, scaling = S
// 2. Global step: Solve Laplacian system L*p' = b for new vertex positions
// 3. Iterate until convergence

use crate::geometry::{Point3D, Vector3D};
use crate::s3_slicer::tet_mesh::TetMesh;
use crate::s3_slicer::tet_quaternion_field::TetQuaternionField;
use nalgebra::{Matrix3, Vector3 as NVector3, SVD};
use sprs::TriMat;
use std::collections::HashMap;

/// Configuration for tet ASAP deformation
#[derive(Debug, Clone)]
pub struct TetAsapConfig {
    /// Maximum iterations
    pub max_iterations: usize,

    /// Convergence threshold
    pub convergence_threshold: f64,

    /// Weight for quaternion field guidance (0-1)
    /// Higher = stronger alignment with quaternion field
    pub quaternion_weight: f64,

    /// Weight for positional constraints (bottom vertices)
    pub constraint_weight: f64,

    /// Allow scaling (true = ASAP, false = ARAP)
    pub allow_scaling: bool,
}

impl Default for TetAsapConfig {
    fn default() -> Self {
        Self {
            max_iterations: 5,
            convergence_threshold: 1e-3,
            quaternion_weight: 0.3,
            constraint_weight: 500.0,
            allow_scaling: true, // ASAP by default
        }
    }
}

/// Tet ASAP deformation solver
pub struct TetAsapSolver {
    /// Rest-pose tet mesh
    rest_mesh: TetMesh,

    /// Quaternion field guidance
    quaternion_field: TetQuaternionField,

    /// Configuration
    config: TetAsapConfig,

    /// Pre-computed: inverse rest edge matrices D^{-1} for each tet
    rest_edge_inverses: Vec<Option<Matrix3<f64>>>,

    /// Pre-computed: edge weights (stiffness)
    edge_weights: HashMap<(usize, usize), f64>,

    /// Bottom vertices (constrained)
    constrained_vertices: Vec<usize>,
}

impl TetAsapSolver {
    pub fn new(
        tet_mesh: TetMesh,
        quaternion_field: TetQuaternionField,
        config: TetAsapConfig,
    ) -> Self {
        log::info!("Initializing TetASAP solver...");
        log::info!("  Vertices: {}, Tets: {}", tet_mesh.vertices.len(), tet_mesh.tets.len());

        // Pre-compute inverse rest edge matrices
        let rest_edge_inverses: Vec<Option<Matrix3<f64>>> = tet_mesh.tets.iter()
            .map(|tet| {
                let d = tet.edge_matrix(&tet_mesh.vertices);
                d.try_inverse()
            })
            .collect();

        let degenerate_count = rest_edge_inverses.iter().filter(|inv| inv.is_none()).count();
        if degenerate_count > 0 {
            log::warn!("  {} degenerate tets (singular edge matrix)", degenerate_count);
        }

        // Compute edge weights (volume-based stiffness)
        let edge_weights = compute_edge_weights(&tet_mesh);
        log::info!("  Edge weights computed: {} edges", edge_weights.len());

        // Find constrained vertices (bottom 5%)
        let constrained_vertices = tet_mesh.bottom_vertices(0.05);
        log::info!("  Constrained (bottom) vertices: {}", constrained_vertices.len());

        Self {
            rest_mesh: tet_mesh,
            quaternion_field,
            config,
            rest_edge_inverses,
            edge_weights,
            constrained_vertices,
        }
    }

    /// Solve for deformed tet mesh
    pub fn solve(&self) -> TetMesh {
        log::info!("Solving TetASAP deformation...");

        let n = self.rest_mesh.vertices.len();
        let mut positions = self.rest_mesh.vertices.clone();

        for iteration in 0..self.config.max_iterations {
            log::info!("  Iteration {}/{}...", iteration + 1, self.config.max_iterations);

            // Local step: compute optimal transforms per tet
            let transforms = self.local_step(&positions);

            // Global step: solve for new positions
            let new_positions = self.global_step(&transforms);

            // Check convergence
            let max_disp = positions.iter().zip(new_positions.iter())
                .map(|(old, new)| (*old - *new).norm())
                .fold(0.0f64, f64::max);

            log::info!("    Max displacement: {:.6}", max_disp);
            positions = new_positions;

            if max_disp < self.config.convergence_threshold {
                log::info!("  Converged after {} iterations", iteration + 1);
                break;
            }
        }

        // Validate positions
        let invalid = positions.iter().filter(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()).count();
        if invalid > 0 {
            log::error!("  {} invalid vertex positions! Falling back to rest mesh", invalid);
            return self.rest_mesh.clone();
        }

        // Build deformed mesh
        let mut deformed = self.rest_mesh.clone();
        deformed.vertices = positions;
        deformed.recompute_bounds();

        let quality = deformed.check_quality();
        log::info!("  Deformed mesh: {} inverted tets, {} degenerate tets",
            quality.inverted_tets, quality.degenerate_tets);

        deformed
    }

    /// Local step: compute optimal rotation+scaling per tet via SVD
    fn local_step(&self, current_positions: &[Point3D]) -> Vec<Matrix3<f64>> {
        self.rest_mesh.tets.iter().enumerate()
            .map(|(ti, tet)| {
                // Get inverse rest edge matrix
                let d_inv = match self.rest_edge_inverses[ti] {
                    Some(inv) => inv,
                    None => return Matrix3::identity(), // Skip degenerate tets
                };

                // Compute deformed edge matrix
                let d_prime = tet.edge_matrix(current_positions);

                // Deformation gradient: F = D' * D^{-1}
                let f = d_prime * d_inv;

                // SVD: F = U * S * V^T
                let svd = SVD::new(f, true, true);

                let (u, v_t) = match (svd.u, svd.v_t) {
                    (Some(u), Some(vt)) => (u, vt),
                    _ => return Matrix3::identity(),
                };

                // Optimal rotation: R = U * V^T
                let mut rotation = u * v_t;

                // Ensure proper rotation (det > 0)
                if rotation.determinant() < 0.0 {
                    let mut u_corr = u;
                    u_corr[(0, 2)] *= -1.0;
                    u_corr[(1, 2)] *= -1.0;
                    u_corr[(2, 2)] *= -1.0;
                    rotation = u_corr * v_t;
                }

                // ASAP: include scaling (S from SVD)
                let transform = if self.config.allow_scaling {
                    // T = R * S = U * diag(singular_values) * V^T = F itself
                    // But we limit extreme scaling
                    let s = &svd.singular_values;
                    let scale_limit = 2.0; // Max 2x scaling
                    let s_clamped = nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::new(
                        s[0].clamp(1.0 / scale_limit, scale_limit),
                        s[1].clamp(1.0 / scale_limit, scale_limit),
                        s[2].clamp(1.0 / scale_limit, scale_limit),
                    ));
                    rotation * s_clamped
                } else {
                    // ARAP: rotation only
                    rotation
                };

                // Blend with quaternion field guidance
                if self.config.quaternion_weight > 0.0 {
                    let target_quat = self.quaternion_field.rotation_at(ti);
                    if target_quat.angle() > 1e-6 {
                        let target_mat = target_quat.to_rotation_matrix().into_inner();

                        // Blend: (1-w)*transform + w*target
                        let w = self.config.quaternion_weight.min(1.0);
                        transform * (1.0 - w) + target_mat * w
                    } else {
                        transform
                    }
                } else {
                    transform
                }
            })
            .collect()
    }

    /// Global step: solve Laplacian system for new vertex positions
    fn global_step(&self, transforms: &[Matrix3<f64>]) -> Vec<Point3D> {
        let n = self.rest_mesh.vertices.len();

        let mut triplets = TriMat::new((n, n));
        let mut rhs_x = vec![0.0; n];
        let mut rhs_y = vec![0.0; n];
        let mut rhs_z = vec![0.0; n];

        // Build Laplacian from tet edges
        for (ti, tet) in self.rest_mesh.tets.iter().enumerate() {
            let transform = &transforms[ti];

            // For each edge of this tet
            for (a, b) in tet.edges() {
                let weight = self.edge_weights.get(&edge_key(a, b)).copied().unwrap_or(1.0);

                // Rest-pose edge vector
                let rest_edge = self.rest_mesh.vertices[b] - self.rest_mesh.vertices[a];
                let rest_vec = NVector3::new(rest_edge.x, rest_edge.y, rest_edge.z);

                // Apply transform
                let transformed = transform * rest_vec;

                // Add to Laplacian
                triplets.add_triplet(a, a, weight);
                triplets.add_triplet(a, b, -weight);
                triplets.add_triplet(b, b, weight);
                triplets.add_triplet(b, a, -weight);

                // Add to RHS
                rhs_x[a] += weight * transformed[0];
                rhs_y[a] += weight * transformed[1];
                rhs_z[a] += weight * transformed[2];
                rhs_x[b] -= weight * transformed[0];
                rhs_y[b] -= weight * transformed[1];
                rhs_z[b] -= weight * transformed[2];
            }
        }

        // Add positional constraints
        for &vi in &self.constrained_vertices {
            let v = &self.rest_mesh.vertices[vi];
            let w = self.config.constraint_weight;

            triplets.add_triplet(vi, vi, w);
            rhs_x[vi] += w * v.x;
            rhs_y[vi] += w * v.y;
            rhs_z[vi] += w * v.z;
        }

        // Solve
        let matrix = triplets.to_csr();
        let sol_x = solve_gauss_seidel(&matrix, &rhs_x, 30);
        let sol_y = solve_gauss_seidel(&matrix, &rhs_y, 30);
        let sol_z = solve_gauss_seidel(&matrix, &rhs_z, 30);

        (0..n)
            .map(|i| Point3D::new(sol_x[i], sol_y[i], sol_z[i]))
            .collect()
    }
}

/// Compute edge weights based on tet volumes (stiffer tets = higher weight)
fn compute_edge_weights(tet_mesh: &TetMesh) -> HashMap<(usize, usize), f64> {
    let mut weights: HashMap<(usize, usize), f64> = HashMap::new();

    for tet in &tet_mesh.tets {
        let vol = tet.volume(&tet_mesh.vertices).max(1e-10);
        // Weight proportional to tet volume (larger tets = stiffer edges)
        let w = vol.powf(1.0 / 3.0); // Cube root for better scaling

        for (a, b) in tet.edges() {
            let key = edge_key(a, b);
            *weights.entry(key).or_insert(0.0) += w;
        }
    }

    weights
}

/// Gauss-Seidel iterative solver
fn solve_gauss_seidel(matrix: &sprs::CsMat<f64>, rhs: &[f64], max_iter: usize) -> Vec<f64> {
    let n = rhs.len();
    let mut solution: Vec<f64> = rhs.iter().map(|&r| if r.is_finite() { r * 0.01 } else { 0.0 }).collect();

    for _ in 0..max_iter {
        let mut max_change = 0.0f64;

        for i in 0..n {
            let mut sum = rhs[i];
            let mut diagonal = 0.0;

            if let Some(row) = matrix.outer_view(i) {
                for (j, &val) in row.iter() {
                    if i == j {
                        diagonal = val;
                    } else {
                        sum -= val * solution[j];
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

        if max_change < 1e-4 {
            break;
        }
    }

    for v in &mut solution {
        if !v.is_finite() {
            *v = 0.0;
        }
    }

    solution
}

/// Edge key (ordered pair)
fn edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s3_slicer::tet_mesh::Tetrahedron;
    use crate::s3_slicer::quaternion_field::QuaternionFieldConfig;

    fn make_test_tet_mesh() -> TetMesh {
        // A simple box-like tet mesh (5 tets forming a box)
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0), // 0
            Point3D::new(10.0, 0.0, 0.0), // 1
            Point3D::new(10.0, 10.0, 0.0), // 2
            Point3D::new(0.0, 10.0, 0.0), // 3
            Point3D::new(5.0, 5.0, 10.0), // 4 (top center)
        ];

        let tets = vec![
            Tetrahedron::new(0, 1, 2, 4),
            Tetrahedron::new(0, 2, 3, 4),
        ];

        TetMesh::new(vertices, tets)
    }

    #[test]
    fn test_tet_asap_solver_creates() {
        let mesh = make_test_tet_mesh();
        let config = QuaternionFieldConfig::default();
        let qfield = TetQuaternionField::optimize(&mesh, config);
        let asap_config = TetAsapConfig::default();

        let solver = TetAsapSolver::new(mesh, qfield, asap_config);
        assert!(!solver.rest_mesh.tets.is_empty());
    }

    #[test]
    fn test_tet_asap_solve() {
        let mesh = make_test_tet_mesh();
        let config = QuaternionFieldConfig::default();
        let qfield = TetQuaternionField::optimize(&mesh, config);
        let asap_config = TetAsapConfig::default();

        let solver = TetAsapSolver::new(mesh.clone(), qfield, asap_config);
        let deformed = solver.solve();

        // Should have same topology
        assert_eq!(deformed.vertices.len(), mesh.vertices.len());
        assert_eq!(deformed.tets.len(), mesh.tets.len());

        // Bottom vertices should be approximately fixed
        assert!((deformed.vertices[0].z - 0.0).abs() < 1.0);
    }

    #[test]
    fn test_tet_asap_no_scaling() {
        let mesh = make_test_tet_mesh();
        let config = QuaternionFieldConfig::default();
        let qfield = TetQuaternionField::optimize(&mesh, config);
        let mut asap_config = TetAsapConfig::default();
        asap_config.allow_scaling = false; // ARAP mode

        let solver = TetAsapSolver::new(mesh.clone(), qfield, asap_config);
        let deformed = solver.solve();

        assert_eq!(deformed.vertices.len(), mesh.vertices.len());
    }
}
