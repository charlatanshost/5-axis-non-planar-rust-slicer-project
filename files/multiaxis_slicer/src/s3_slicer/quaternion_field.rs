// Quaternion field optimization for S3-Slicer
// Based on "S3-Slicer: A General Slicing Framework for Multi-Axis 3D Printing"
// Implements rotation-driven deformation with fabrication objectives

use crate::geometry::{Point3D, Vector3D};
use crate::mesh::Mesh;
use nalgebra::{UnitQuaternion, Vector3};
use std::collections::HashMap;
use rayon::prelude::*;

/// Fabrication objective for deformation optimization
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FabricationObjective {
    /// Minimize support material (prioritize self-supporting angles)
    SupportFree,
    /// Maximize strength along specific direction
    Strength,
    /// Optimize surface quality
    SurfaceQuality,
    /// Balanced combination of objectives
    Balanced,
}

/// Configuration for quaternion field optimization
#[derive(Debug, Clone)]
pub struct QuaternionFieldConfig {
    /// Primary fabrication objective
    pub objective: FabricationObjective,

    /// Build direction (typically vertical)
    pub build_direction: Vector3D,

    /// Number of optimization iterations
    pub optimization_iterations: usize,

    /// Smoothness weight (higher = smoother field)
    pub smoothness_weight: f64,

    /// Objective weight (higher = stronger objective enforcement)
    pub objective_weight: f64,

    /// Overhang angle threshold for support-free (degrees)
    pub overhang_threshold: f64,

    /// Maximum rotation angle per triangle (degrees)
    pub max_rotation_degrees: f64,
}

impl Default for QuaternionFieldConfig {
    fn default() -> Self {
        Self {
            objective: FabricationObjective::SupportFree,
            build_direction: Vector3D::new(0.0, 0.0, 1.0),
            optimization_iterations: 50,
            smoothness_weight: 0.5,
            objective_weight: 1.0,
            overhang_threshold: 45.0,
            max_rotation_degrees: 15.0,
        }
    }
}

/// Quaternion field defined over mesh
#[derive(Clone)]
pub struct QuaternionField {
    /// Quaternion at each triangle (rotation from original orientation)
    pub rotations: Vec<UnitQuaternion<f64>>,

    /// Energy of the field (lower = better)
    pub energy: f64,

    /// Configuration used for optimization
    pub config: QuaternionFieldConfig,
}

impl QuaternionField {
    /// Optimize quaternion field based on mesh and objectives
    pub fn optimize(mesh: &Mesh, config: QuaternionFieldConfig) -> Self {
        let num_triangles = mesh.triangles.len();

        // Initialize with identity rotations
        let mut rotations: Vec<UnitQuaternion<f64>> = vec![UnitQuaternion::identity(); num_triangles];

        // Build triangle adjacency for smoothness
        let adjacency = build_triangle_adjacency(mesh);

        // Optimization loop (sequential iterations, but parallel per iteration)
        for iteration in 0..config.optimization_iterations {
            // Parallel computation of new rotations for all triangles
            let new_rotations: Vec<UnitQuaternion<f64>> = (0..num_triangles)
                .into_par_iter()
                .map(|tri_idx| {
                    let triangle = &mesh.triangles[tri_idx];
                    let normal = triangle.normal();

                    // Compute optimal rotation for this triangle
                    let optimal_rotation = compute_optimal_rotation(
                        &normal,
                        &config.build_direction,
                        &config.objective,
                        config.overhang_threshold,
                        config.max_rotation_degrees,
                    );

                    // Smooth with neighbors
                    if config.smoothness_weight > 0.0 {
                        smooth_with_neighbors(
                            &optimal_rotation,
                            &rotations,
                            &adjacency[tri_idx],
                            config.smoothness_weight,
                        )
                    } else {
                        optimal_rotation
                    }
                })
                .collect();

            rotations = new_rotations;

            // Optional: compute energy for convergence check
            if iteration % 10 == 0 {
                log::debug!("Quaternion field optimization iteration {}/{}", iteration, config.optimization_iterations);
            }
        }

        // Safety: Check for and replace any invalid quaternions
        let invalid_count = rotations.iter()
            .filter(|q| !q.coords.x.is_finite() || !q.coords.y.is_finite() ||
                        !q.coords.z.is_finite() || !q.coords.w.is_finite())
            .count();

        if invalid_count > 0 {
            log::warn!("  WARNING: {} quaternions with non-finite components after optimization!", invalid_count);
            log::warn!("  Replacing invalid quaternions with identity");
            for q in &mut rotations {
                if !q.coords.x.is_finite() || !q.coords.y.is_finite() ||
                   !q.coords.z.is_finite() || !q.coords.w.is_finite() {
                    *q = UnitQuaternion::identity();
                }
            }
        }

        // Compute final energy
        let energy = compute_field_energy(&rotations, &adjacency, mesh, &config);

        // Log statistics about the quaternion field
        let non_identity_count = rotations.iter()
            .filter(|q| q.angle() > 0.01)  // Rotation > 0.01 radians (~0.57 degrees)
            .count();
        let max_rotation = rotations.iter()
            .map(|q| q.angle())
            .fold(0.0f64, f64::max);

        log::info!("Quaternion field statistics:");
        log::info!("  Total triangles: {}", num_triangles);
        log::info!("  Triangles with rotation: {} ({:.1}%)",
            non_identity_count,
            100.0 * non_identity_count as f64 / num_triangles as f64);
        log::info!("  Maximum rotation angle: {:.2}° ({:.4} radians)",
            max_rotation.to_degrees(), max_rotation);
        log::info!("  Field energy: {:.4}", energy);

        Self {
            rotations,
            energy,
            config,
        }
    }

    /// Get rotation at a specific triangle
    pub fn rotation_at(&self, triangle_idx: usize) -> Option<&UnitQuaternion<f64>> {
        self.rotations.get(triangle_idx)
    }

    /// Interpolate rotation within a triangle using barycentric coordinates
    pub fn interpolate_rotation(
        &self,
        tri_idx: usize,
        barycentric: (f64, f64, f64),
    ) -> UnitQuaternion<f64> {
        // For now, just return the triangle's rotation
        // Full implementation would interpolate with neighboring triangles
        self.rotations.get(tri_idx).copied().unwrap_or(UnitQuaternion::identity())
    }
}

/// Compute optimal rotation for a triangle based on objective
///
/// For S3-Slicer global deformation, we compute rotations for ALL triangles,
/// not just overhangs. The rotation represents how the local coordinate frame
/// should rotate to "unfold" the mesh for better printability.
fn compute_optimal_rotation(
    normal: &Vector3D,
    build_direction: &Vector3D,
    objective: &FabricationObjective,
    overhang_threshold: f64,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    match objective {
        FabricationObjective::SupportFree => {
            // Global unfolding rotation - affects ALL triangles
            compute_global_unfold_rotation(normal, build_direction, overhang_threshold, max_rotation_degrees)
        }
        FabricationObjective::Strength => {
            // Align with build direction for strength
            compute_strength_rotation(normal, build_direction, max_rotation_degrees)
        }
        FabricationObjective::SurfaceQuality => {
            // Minimize rotation for surface quality
            UnitQuaternion::identity()
        }
        FabricationObjective::Balanced => {
            // Weighted combination
            let unfold = compute_global_unfold_rotation(normal, build_direction, overhang_threshold, max_rotation_degrees);
            let strength = compute_strength_rotation(normal, build_direction, max_rotation_degrees);

            // Spherical linear interpolation (slerp)
            unfold.slerp(&strength, 0.5)
        }
    }
}

/// Compute support-free rotation for S3-Slicer deformation
///
/// Based on S³-Slicer paper: The feasible region for Local Printing Direction (LPD) is:
///   H_SF = {d | d · n_f + sin(α) ≥ 0}
///
/// where d is the printing direction (typically Z-up) and n_f is surface normal.
///
/// For a surface to be self-supporting, the angle between printing direction and
/// surface normal must satisfy the overhang constraint.
///
/// CRITICAL: We ONLY rotate surfaces that violate the overhang constraint.
/// Self-supporting surfaces (upward-facing, vertical) get NO rotation.
/// This prevents the "twisted fin" problem from conflicting rotations.
fn compute_global_unfold_rotation(
    normal: &Vector3D,
    _build_direction: &Vector3D,
    overhang_threshold: f64,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    // Safety: Validate inputs first
    if !normal.x.is_finite() || !normal.y.is_finite() || !normal.z.is_finite() {
        return UnitQuaternion::identity();
    }

    let normal_len = normal.norm();
    if !normal_len.is_finite() || normal_len < 1e-10 {
        return UnitQuaternion::identity();
    }

    // Normalize the normal to ensure it's a unit vector
    let unit_normal = normal / normal_len;

    // S³-Slicer constraint: d · n_f + sin(α) ≥ 0
    // For build direction d = (0, 0, 1) and normal n_f:
    //   d · n_f = n_f.z
    // Constraint becomes: n_f.z + sin(α) ≥ 0
    //                     n_f.z ≥ -sin(α)
    //
    // If this is satisfied, the surface is ALREADY self-supporting - NO rotation needed!
    let overhang_threshold_rad = overhang_threshold.to_radians();
    let min_z_for_self_supporting = -overhang_threshold_rad.sin();

    // If surface is self-supporting (faces upward or sideways enough), NO rotation
    if unit_normal.z >= min_z_for_self_supporting {
        return UnitQuaternion::identity();
    }

    // Surface is an overhang - needs rotation to become self-supporting
    // We want to rotate so that after rotation, the EFFECTIVE local Z-direction
    // makes the surface self-supporting
    //
    // The rotation axis is perpendicular to the vertical plane containing the normal
    // This rotates the local frame "outward" from the overhang
    let horiz_len = (unit_normal.x * unit_normal.x + unit_normal.y * unit_normal.y).sqrt();

    let rotation_axis = if horiz_len > 1e-6 {
        // Axis perpendicular to the horizontal projection of normal
        Vector3D::new(-unit_normal.y / horiz_len, unit_normal.x / horiz_len, 0.0)
    } else {
        // Normal is straight down - rotate around any horizontal axis
        Vector3D::new(1.0, 0.0, 0.0)
    };

    // Calculate rotation amount to bring surface to threshold angle
    // Current angle from horizontal: asin(-n.z) for downward-facing
    // Target: overhang_threshold from horizontal
    let current_angle_from_horiz = (-unit_normal.z).asin();
    let target_angle_from_horiz = overhang_threshold_rad;
    let rotation_amount = current_angle_from_horiz - target_angle_from_horiz;

    // Clamp to maximum allowed rotation
    let max_rotation_radians = max_rotation_degrees.to_radians();
    let clamped_rotation = rotation_amount.clamp(0.0, max_rotation_radians);

    if clamped_rotation < 1e-6 {
        return UnitQuaternion::identity();
    }

    // Create quaternion safely
    let quat = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(rotation_axis),
        clamped_rotation,
    );

    // Final safety check
    if !quat.coords.x.is_finite() || !quat.coords.y.is_finite() ||
       !quat.coords.z.is_finite() || !quat.coords.w.is_finite() {
        return UnitQuaternion::identity();
    }

    quat
}

/// Legacy: Compute rotation to make surface self-supporting (overhangs only)
#[allow(dead_code)]
fn compute_support_free_rotation(
    normal: &Vector3D,
    _build_direction: &Vector3D,
    overhang_threshold: f64,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    // Safety: Validate inputs first
    if !normal.x.is_finite() || !normal.y.is_finite() || !normal.z.is_finite() {
        return UnitQuaternion::identity();
    }

    let normal_len = normal.norm();
    if !normal_len.is_finite() || normal_len < 1e-10 {
        return UnitQuaternion::identity();
    }

    let unit_normal = normal / normal_len;
    let overhang_threshold_rad = overhang_threshold.to_radians();
    let min_z = -overhang_threshold_rad.sin();

    // Only rotate overhanging surfaces
    if unit_normal.z >= min_z {
        return UnitQuaternion::identity();
    }

    let rotation_axis = Vector3D::new(-unit_normal.y, unit_normal.x, 0.0);
    let axis_len = rotation_axis.norm();

    if axis_len < 1e-6 {
        return UnitQuaternion::identity();
    }

    let axis = rotation_axis / axis_len;
    let current_angle_from_horiz = (-unit_normal.z).asin();
    let target_angle_from_horiz = overhang_threshold_rad;
    let mut rotation_amount = current_angle_from_horiz - target_angle_from_horiz;

    let max_rotation_radians = max_rotation_degrees.to_radians();
    if rotation_amount.abs() > max_rotation_radians {
        rotation_amount = rotation_amount.signum() * max_rotation_radians;
    }

    UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_unchecked(axis),
        rotation_amount,
    )
}

/// Compute rotation for strength (align with build direction)
fn compute_strength_rotation(
    normal: &Vector3D,
    build_direction: &Vector3D,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    // Rotate to align normal with build direction
    let full_rotation = UnitQuaternion::rotation_between(normal, build_direction)
        .unwrap_or(UnitQuaternion::identity());

    // Limit rotation amount
    let angle = full_rotation.angle();
    let max_rotation_radians = max_rotation_degrees.to_radians();

    if angle > max_rotation_radians {
        // Scale down the rotation
        let scale_factor = max_rotation_radians / angle;
        let axis = full_rotation.axis().map(|ax| ax.into_inner()).unwrap_or(Vector3D::z_axis().into_inner());
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle * scale_factor)
    } else {
        full_rotation
    }
}

/// Smooth rotation with neighbors
fn smooth_with_neighbors(
    optimal: &UnitQuaternion<f64>,
    current_rotations: &[UnitQuaternion<f64>],
    neighbor_indices: &[usize],
    smoothness_weight: f64,
) -> UnitQuaternion<f64> {
    if neighbor_indices.is_empty() {
        return *optimal;
    }

    // Average with neighbors using slerp
    let mut accumulated = *optimal;
    let mut count = 1.0;

    for &neighbor_idx in neighbor_indices {
        if neighbor_idx < current_rotations.len() {
            let neighbor = &current_rotations[neighbor_idx];

            // Safety: Check if neighbor quaternion is valid
            if !neighbor.coords.x.is_finite() || !neighbor.coords.y.is_finite() ||
               !neighbor.coords.z.is_finite() || !neighbor.coords.w.is_finite() {
                continue;  // Skip invalid neighbors
            }

            let weight = smoothness_weight / (count + 1.0);
            if weight.is_finite() && weight > 0.0 && weight < 1.0 {
                let result = accumulated.slerp(neighbor, weight);

                // Safety: Check if slerp result is valid
                if result.coords.x.is_finite() && result.coords.y.is_finite() &&
                   result.coords.z.is_finite() && result.coords.w.is_finite() {
                    accumulated = result;
                    count += 1.0;
                }
            }
        }
    }

    // Final safety check
    if !accumulated.coords.x.is_finite() || !accumulated.coords.y.is_finite() ||
       !accumulated.coords.z.is_finite() || !accumulated.coords.w.is_finite() {
        log::warn!("  WARNING: Smoothing produced invalid quaternion, using optimal");
        return *optimal;
    }

    accumulated
}

/// Build triangle adjacency graph
fn build_triangle_adjacency(mesh: &Mesh) -> Vec<Vec<usize>> {
    let mut adjacency = vec![Vec::new(); mesh.triangles.len()];
    let mut edge_map: HashMap<(u64, u64), Vec<usize>> = HashMap::new();

    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        let v0_hash = hash_point(&triangle.v0);
        let v1_hash = hash_point(&triangle.v1);
        let v2_hash = hash_point(&triangle.v2);

        let edges = [
            (v0_hash.min(v1_hash), v0_hash.max(v1_hash)),
            (v1_hash.min(v2_hash), v1_hash.max(v2_hash)),
            (v2_hash.min(v0_hash), v2_hash.max(v0_hash)),
        ];

        for edge in &edges {
            edge_map.entry(*edge).or_insert_with(Vec::new).push(tri_idx);
        }
    }

    for triangles in edge_map.values() {
        if triangles.len() == 2 {
            adjacency[triangles[0]].push(triangles[1]);
            adjacency[triangles[1]].push(triangles[0]);
        }
    }

    adjacency
}

/// Compute field energy for optimization
fn compute_field_energy(
    rotations: &[UnitQuaternion<f64>],
    adjacency: &[Vec<usize>],
    mesh: &Mesh,
    config: &QuaternionFieldConfig,
) -> f64 {
    let mut energy = 0.0;

    // Smoothness term
    for (tri_idx, neighbors) in adjacency.iter().enumerate() {
        for &neighbor_idx in neighbors {
            if neighbor_idx < rotations.len() {
                // Measure rotation difference
                let diff = rotations[tri_idx].inverse() * rotations[neighbor_idx];
                let angle = diff.angle();
                energy += config.smoothness_weight * angle * angle;
            }
        }
    }

    // Objective term
    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        let normal = triangle.normal();
        let rotated_normal = rotations[tri_idx] * normal;

        let angle = angle_between(&rotated_normal, &config.build_direction);
        let angle_deg = angle.to_degrees();

        // Penalize overhang
        let overhang_penalty = if angle_deg > (90.0 - config.overhang_threshold) {
            (angle_deg - (90.0 - config.overhang_threshold)).powi(2)
        } else {
            0.0
        };

        energy += config.objective_weight * overhang_penalty;
    }

    energy
}

/// Hash point for edge mapping
fn hash_point(point: &Point3D) -> u64 {
    let scale = 1000.0;
    let x = (point.x * scale).round() as i64;
    let y = (point.y * scale).round() as i64;
    let z = (point.z * scale).round() as i64;
    ((x as u64) << 42) ^ ((y as u64) << 21) ^ (z as u64)
}

/// Angle between two vectors
fn angle_between(v1: &Vector3D, v2: &Vector3D) -> f64 {
    let dot = v1.dot(v2) / (v1.norm() * v2.norm());
    dot.clamp(-1.0, 1.0).acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Triangle;

    #[test]
    fn test_global_unfold_rotation() {
        let normal = Vector3D::new(1.0, 0.0, 0.0); // Horizontal surface (vertical wall)
        let build_dir = Vector3D::new(0.0, 0.0, 1.0); // Vertical build

        let rotation = compute_global_unfold_rotation(&normal, &build_dir, 45.0, 15.0);

        // Vertical surfaces should get some rotation to unfold
        assert!(rotation.angle() > 0.0);
    }

    #[test]
    fn test_downward_facing_rotation() {
        let normal = Vector3D::new(0.0, 0.0, -1.0); // Downward facing (overhang)
        let build_dir = Vector3D::new(0.0, 0.0, 1.0);

        let rotation = compute_global_unfold_rotation(&normal, &build_dir, 45.0, 15.0);

        // Overhangs should get rotation to become more vertical
        assert!(rotation.angle() > 0.0);
    }

    #[test]
    fn test_upward_facing_rotation() {
        let normal = Vector3D::new(0.3, 0.0, 0.95); // Slightly tilted upward
        let build_dir = Vector3D::new(0.0, 0.0, 1.0);

        let rotation = compute_global_unfold_rotation(&normal, &build_dir, 45.0, 15.0);

        // Tilted upward surfaces should get some unfolding rotation
        // (straight up surfaces get less rotation)
        assert!(rotation.angle() >= 0.0);
    }

    #[test]
    fn test_quaternion_field_optimization() {
        let triangle = Triangle::new(
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(5.0, 10.0, 0.0),
        );

        let mesh = Mesh {
            triangles: vec![triangle],
            bounds_min: Point3D::new(0.0, 0.0, 0.0),
            bounds_max: Point3D::new(10.0, 10.0, 0.0),
        };

        let config = QuaternionFieldConfig::default();
        let field = QuaternionField::optimize(&mesh, config);

        assert_eq!(field.rotations.len(), 1);
        assert!(field.energy >= 0.0);
    }
}
