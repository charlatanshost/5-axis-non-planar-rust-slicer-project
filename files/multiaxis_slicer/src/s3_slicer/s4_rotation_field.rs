// S4-style rotation field derived from Dijkstra path length gradients
//
// Instead of optimizing a quaternion field from scratch (S3 approach), the S4
// approach uses the Dijkstra distance gradient to determine rotation *direction*
// and computes rotation *magnitude* from overhang analysis. The result is then
// smoothed via iterative neighbor SLERP.
//
// This produces a TetQuaternionField-compatible output that can be passed
// directly to TetAsapSolver.

use crate::geometry::Vector3D;
use crate::s3_slicer::tet_mesh::TetMesh;
use crate::s3_slicer::tet_dijkstra_field::TetDijkstraField;
use crate::s3_slicer::tet_quaternion_field::TetQuaternionField;
use crate::s3_slicer::quaternion_field::QuaternionFieldConfig;
use nalgebra::UnitQuaternion;
use rayon::prelude::*;

/// Configuration for S4 rotation field computation
#[derive(Debug, Clone)]
pub struct S4RotationConfig {
    /// Build direction (default: +Z)
    pub build_direction: Vector3D,

    /// Overhang threshold in degrees — faces steeper than this get rotated
    pub overhang_threshold: f64,

    /// Maximum rotation angle per tet (degrees)
    pub max_rotation_degrees: f64,

    /// Z-bias for Dijkstra edge weights (0 = Euclidean, 1 = pure |ΔZ|).
    /// Higher values make the distance field track actual print height rather than
    /// graph path length, preventing topologically-close features at different heights
    /// (e.g. the two ears of the Stanford Bunny) from landing on the same layer.
    pub z_bias: f64,

    /// Number of smoothing iterations for the rotation field
    pub smoothing_iterations: usize,

    /// Smoothing weight (0..1) — how much to blend with neighbors per iteration
    pub smoothness_weight: f64,
}

impl Default for S4RotationConfig {
    fn default() -> Self {
        Self {
            build_direction: Vector3D::new(0.0, 0.0, 1.0),
            overhang_threshold: 45.0,
            max_rotation_degrees: 15.0,
            z_bias: 0.8,
            smoothing_iterations: 25,
            smoothness_weight: 0.5,
        }
    }
}

/// S4-style rotation field: per-tet rotations derived from Dijkstra gradients
/// and overhang analysis, smoothed for continuity.
pub struct S4RotationField {
    /// Per-tet rotation quaternion
    pub rotations: Vec<UnitQuaternion<f64>>,

    /// Per-tet rotation magnitude in radians (for diagnostics)
    pub rotation_magnitudes: Vec<f64>,

    /// Energy of the field (sum of squared neighbor rotation differences)
    pub energy: f64,
}

impl S4RotationField {
    /// Compute S4 rotation field from Dijkstra gradients.
    ///
    /// Algorithm:
    /// 1. For each tet with a surface face, compute overhang angle
    /// 2. Compute rotation axis from Dijkstra gradient (perpendicular to gradient and build dir)
    /// 3. Compute rotation magnitude from overhang severity
    /// 4. Propagate to interior tets using neighbor averaging
    /// 5. Smooth via iterative SLERP with face-adjacent neighbors
    pub fn compute(
        tet_mesh: &TetMesh,
        dijkstra_field: &TetDijkstraField,
        config: &S4RotationConfig,
    ) -> Self {
        let num_tets = tet_mesh.tets.len();
        log::info!("Computing S4 rotation field for {} tets...", num_tets);
        log::info!("  Overhang threshold: {:.1} deg, max rotation: {:.1} deg",
            config.overhang_threshold, config.max_rotation_degrees);

        // Step 1: Compute initial rotations from overhang analysis
        let (mut rotations, magnitudes) =
            compute_initial_rotations(tet_mesh, dijkstra_field, config);

        let non_identity = rotations.iter().filter(|q| q.angle() > 0.01).count();
        log::info!("  Initial rotations: {} non-identity ({:.1}%)",
            non_identity, 100.0 * non_identity as f64 / num_tets as f64);

        // Step 2: Smooth rotation field
        for iter in 0..config.smoothing_iterations {
            let new_rotations: Vec<UnitQuaternion<f64>> = (0..num_tets)
                .into_par_iter()
                .map(|ti| {
                    smooth_single_tet(
                        ti,
                        &rotations,
                        tet_mesh,
                        config.smoothness_weight,
                    )
                })
                .collect();
            rotations = new_rotations;

            if iter % 10 == 0 {
                log::debug!("  Smoothing iteration {}/{}", iter, config.smoothing_iterations);
            }
        }

        // Sanitize: replace NaN/Inf with identity
        for q in &mut rotations {
            if !q.coords.iter().all(|c| c.is_finite()) {
                *q = UnitQuaternion::identity();
            }
        }

        // Compute energy (sum of squared neighbor rotation differences)
        let energy = compute_smoothness_energy(&rotations, tet_mesh);

        let non_identity = rotations.iter().filter(|q| q.angle() > 0.01).count();
        let max_angle = rotations.iter().map(|q| q.angle()).fold(0.0f64, f64::max);
        log::info!("  S4 rotation field complete:");
        log::info!("    Non-identity: {} ({:.1}%)", non_identity,
            100.0 * non_identity as f64 / num_tets as f64);
        log::info!("    Max rotation: {:.2} deg", max_angle.to_degrees());
        log::info!("    Smoothness energy: {:.4}", energy);

        S4RotationField {
            rotations,
            rotation_magnitudes: magnitudes,
            energy,
        }
    }

    /// Convert to TetQuaternionField for compatibility with TetAsapSolver.
    ///
    /// This allows the S4 rotation field to be consumed by the existing ASAP
    /// deformation solver without any modifications.
    pub fn to_tet_quaternion_field(&self, config: &QuaternionFieldConfig) -> TetQuaternionField {
        TetQuaternionField {
            rotations: self.rotations.clone(),
            energy: self.energy,
            config: config.clone(),
        }
    }

    /// Get rotation for a specific tet
    pub fn rotation_at(&self, tet_index: usize) -> UnitQuaternion<f64> {
        self.rotations.get(tet_index).copied().unwrap_or(UnitQuaternion::identity())
    }
}

/// Compute initial per-tet rotations from overhang analysis and Dijkstra gradients.
///
/// For each tet:
/// - If it has a surface face (boundary), compute the overhang angle
/// - If overhang exceeds threshold, compute rotation to fix it
/// - Rotation axis: perpendicular to surface normal projected onto horizontal plane
/// - Rotation magnitude: proportional to how much the overhang exceeds the threshold
/// - Interior tets: interpolate from distance field (gradient direction)
fn compute_initial_rotations(
    tet_mesh: &TetMesh,
    dijkstra_field: &TetDijkstraField,
    config: &S4RotationConfig,
) -> (Vec<UnitQuaternion<f64>>, Vec<f64>) {
    let num_tets = tet_mesh.tets.len();
    let overhang_rad = config.overhang_threshold.to_radians();
    let max_rad = config.max_rotation_degrees.to_radians();
    let min_z = -overhang_rad.sin(); // Normal Z below this = overhang

    let mut rotations = vec![UnitQuaternion::identity(); num_tets];
    let mut magnitudes = vec![0.0f64; num_tets];

    for ti in 0..num_tets {
        let neighbors = &tet_mesh.tet_neighbors[ti];

        // Find surface face normals (boundary faces where neighbor is None)
        let mut surface_normal = Vector3D::zeros();
        let mut has_surface = false;

        let face_normals = tet_mesh.tets[ti].face_normals(&tet_mesh.vertices);

        for fi in 0..4 {
            if neighbors[fi].is_none() {
                let n = face_normals[fi];
                let area = n.norm();
                if area > 1e-10 {
                    surface_normal += n; // area-weighted
                    has_surface = true;
                }
            }
        }

        if has_surface {
            let norm = surface_normal.norm();
            if norm > 1e-10 {
                let unit_normal = surface_normal / norm;

                // Check overhang: normal.z < min_z means overhang
                if unit_normal.z < min_z {
                    // Compute rotation to fix overhang
                    let horiz_len = (unit_normal.x * unit_normal.x + unit_normal.y * unit_normal.y).sqrt();

                    // Rotation axis: perpendicular to the normal's horizontal projection,
                    // oriented so that a POSITIVE rotation lifts the overhanging side UP.
                    // For normal (nx, ny, nz_neg): axis = (ny, -nx, 0) / ||horiz||
                    // This ensures cross(axis, outward_dir) points upward (+Z).
                    let rotation_axis = if horiz_len > 1e-6 {
                        Vector3D::new(unit_normal.y / horiz_len, -unit_normal.x / horiz_len, 0.0)
                    } else {
                        // Normal pointing straight down — use Dijkstra gradient for direction
                        let grad = dijkstra_field.gradient_at(ti);
                        let horiz = Vector3D::new(grad.x, grad.y, 0.0);
                        let h_len = horiz.norm();
                        if h_len > 1e-6 {
                            Vector3D::new(horiz.y / h_len, -horiz.x / h_len, 0.0)
                        } else {
                            Vector3D::new(1.0, 0.0, 0.0)
                        }
                    };

                    // Rotation amount: how much to rotate to reach threshold
                    let current_angle = (-unit_normal.z).asin(); // angle below horizontal
                    let target_angle = overhang_rad;
                    let rotation_amount = (current_angle - target_angle).clamp(0.0, max_rad);

                    if rotation_amount > 1e-6 {
                        let q = UnitQuaternion::from_axis_angle(
                            &nalgebra::Unit::new_normalize(rotation_axis),
                            rotation_amount,
                        );
                        if q.coords.iter().all(|c| c.is_finite()) {
                            rotations[ti] = q;
                            magnitudes[ti] = rotation_amount;
                        }
                    }
                }
            }
        } else {
            // Interior tet: use Dijkstra gradient to determine rotation direction
            // Scale rotation by how far from base (tets near base need less rotation)
            let normalized_dist = dijkstra_field.normalized_distance(ti);

            // Only rotate interior tets that are "high up" (past the first 20%)
            if normalized_dist > 0.2 {
                // Look at neighbor rotations to estimate what interior should have
                let mut neighbor_count = 0;
                let mut neighbor_mag_sum = 0.0;
                let mut neighbor_axis_sum = Vector3D::zeros();

                for fi in 0..4 {
                    if let Some(ni) = neighbors[fi] {
                        if magnitudes[ni] > 1e-6 {
                            neighbor_count += 1;
                            neighbor_mag_sum += magnitudes[ni];
                            if let Some(axis) = rotations[ni].axis() {
                                neighbor_axis_sum += axis.into_inner() * magnitudes[ni];
                            }
                        }
                    }
                }

                if neighbor_count > 0 {
                    let avg_mag = neighbor_mag_sum / neighbor_count as f64;
                    let axis_norm = neighbor_axis_sum.norm();
                    if axis_norm > 1e-10 && avg_mag > 1e-6 {
                        let axis = neighbor_axis_sum / axis_norm;
                        let q = UnitQuaternion::from_axis_angle(
                            &nalgebra::Unit::new_normalize(axis),
                            avg_mag * 0.5, // Dampen interior rotations
                        );
                        if q.coords.iter().all(|c| c.is_finite()) {
                            rotations[ti] = q;
                            magnitudes[ti] = avg_mag * 0.5;
                        }
                    }
                }
            }
        }
    }

    (rotations, magnitudes)
}

/// Smooth a single tet's rotation by SLERP with face-adjacent neighbors.
fn smooth_single_tet(
    ti: usize,
    rotations: &[UnitQuaternion<f64>],
    tet_mesh: &TetMesh,
    smoothness_weight: f64,
) -> UnitQuaternion<f64> {
    let current = rotations[ti];
    let neighbors = &tet_mesh.tet_neighbors[ti];

    let neighbor_indices: Vec<usize> = neighbors.iter().filter_map(|n| *n).collect();

    if neighbor_indices.is_empty() {
        return current;
    }

    let mut result = current;
    let mut count = 1.0;

    for &ni in &neighbor_indices {
        if ni >= rotations.len() {
            continue;
        }
        let neighbor_rot = &rotations[ni];
        if !neighbor_rot.coords.iter().all(|c| c.is_finite()) {
            continue;
        }

        let weight = smoothness_weight / (count + 1.0);
        if weight > 0.0 && weight < 1.0 {
            let blended = result.slerp(neighbor_rot, weight);
            if blended.coords.iter().all(|c| c.is_finite()) {
                result = blended;
                count += 1.0;
            }
        }
    }

    if result.coords.iter().all(|c| c.is_finite()) {
        result
    } else {
        current
    }
}

/// Compute smoothness energy: sum of squared angle differences between adjacent tets.
fn compute_smoothness_energy(
    rotations: &[UnitQuaternion<f64>],
    tet_mesh: &TetMesh,
) -> f64 {
    let mut energy = 0.0;
    for (ti, neighbors) in tet_mesh.tet_neighbors.iter().enumerate() {
        for ni in neighbors.iter().filter_map(|n| *n) {
            let diff = rotations[ti].inverse() * rotations[ni];
            let angle = diff.angle();
            energy += angle * angle;
        }
    }
    energy
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point3D;
    use crate::s3_slicer::tet_mesh::{TetMesh, Tetrahedron};

    fn make_test_mesh() -> TetMesh {
        // Two tets stacked: bottom has upward-facing surface, top has overhang
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),  // 0
            Point3D::new(10.0, 0.0, 0.0), // 1
            Point3D::new(5.0, 10.0, 0.0), // 2
            Point3D::new(5.0, 5.0, 10.0), // 3 - apex
            Point3D::new(15.0, 5.0, 10.0),// 4 - creates overhang on tet 1
        ];

        let tets = vec![
            Tetrahedron { vertices: [0, 1, 2, 3] }, // bottom - upright
            Tetrahedron { vertices: [1, 2, 3, 4] }, // top - has overhang
        ];

        TetMesh::new(vertices, tets)
    }

    #[test]
    fn test_s4_rotation_field_compute() {
        let mesh = make_test_mesh();
        let dijkstra = TetDijkstraField::compute(&mesh, 0.8);
        let config = S4RotationConfig::default();

        let field = S4RotationField::compute(&mesh, &dijkstra, &config);

        assert_eq!(field.rotations.len(), 2);
        assert!(field.energy >= 0.0);

        // All rotations should be valid quaternions
        for q in &field.rotations {
            assert!(q.coords.iter().all(|c| c.is_finite()));
        }
    }

    #[test]
    fn test_to_tet_quaternion_field() {
        let mesh = make_test_mesh();
        let dijkstra = TetDijkstraField::compute(&mesh, 0.8);
        let s4_config = S4RotationConfig::default();
        let field = S4RotationField::compute(&mesh, &dijkstra, &s4_config);

        let quat_config = QuaternionFieldConfig::default();
        let tet_qfield = field.to_tet_quaternion_field(&quat_config);

        assert_eq!(tet_qfield.rotations.len(), field.rotations.len());
        // Rotations should be identical
        for (a, b) in tet_qfield.rotations.iter().zip(field.rotations.iter()) {
            assert!((a.angle() - b.angle()).abs() < 1e-10);
        }
    }

    #[test]
    fn test_smoothing_reduces_energy() {
        let mesh = make_test_mesh();
        let dijkstra = TetDijkstraField::compute(&mesh, 0.8);

        // With no smoothing
        let config_unsmoothed = S4RotationConfig {
            smoothing_iterations: 0,
            ..S4RotationConfig::default()
        };
        let field_unsmoothed = S4RotationField::compute(&mesh, &dijkstra, &config_unsmoothed);

        // With smoothing
        let config_smoothed = S4RotationConfig {
            smoothing_iterations: 25,
            ..S4RotationConfig::default()
        };
        let field_smoothed = S4RotationField::compute(&mesh, &dijkstra, &config_smoothed);

        // Smoothed energy should be <= unsmoothed (or both zero for simple mesh)
        assert!(
            field_smoothed.energy <= field_unsmoothed.energy + 1e-6,
            "Smoothed energy ({}) should be <= unsmoothed energy ({})",
            field_smoothed.energy, field_unsmoothed.energy
        );
    }
}
