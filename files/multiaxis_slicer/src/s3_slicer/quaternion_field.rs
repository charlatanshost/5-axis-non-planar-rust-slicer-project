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
fn compute_optimal_rotation(
    normal: &Vector3D,
    build_direction: &Vector3D,
    objective: &FabricationObjective,
    overhang_threshold: f64,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    match objective {
        FabricationObjective::SupportFree => {
            // Rotate to make surface self-supporting
            compute_support_free_rotation(normal, build_direction, overhang_threshold, max_rotation_degrees)
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
            let support_free = compute_support_free_rotation(normal, build_direction, overhang_threshold, max_rotation_degrees);
            let strength = compute_strength_rotation(normal, build_direction, max_rotation_degrees);

            // Spherical linear interpolation (slerp)
            support_free.slerp(&strength, 0.5)
        }
    }
}

/// Compute rotation to make surface self-supporting
fn compute_support_free_rotation(
    normal: &Vector3D,
    build_direction: &Vector3D,
    overhang_threshold: f64,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    // Safety: Validate inputs first
    if !normal.x.is_finite() || !normal.y.is_finite() || !normal.z.is_finite() {
        return UnitQuaternion::identity();
    }
    if !build_direction.x.is_finite() || !build_direction.y.is_finite() || !build_direction.z.is_finite() {
        return UnitQuaternion::identity();
    }

    let normal_len = normal.norm();
    if !normal_len.is_finite() || normal_len < 1e-10 {
        // Degenerate triangle with zero or invalid normal
        return UnitQuaternion::identity();
    }

    // Normalize the normal to ensure it's a unit vector
    let unit_normal = normal / normal_len;

    // Check if already self-supporting
    let angle = angle_between(&unit_normal, build_direction);
    if !angle.is_finite() {
        return UnitQuaternion::identity();
    }

    let angle_deg = angle.to_degrees();
    let threshold_angle = 90.0 - overhang_threshold;

    if angle_deg <= threshold_angle {
        // Already self-supporting, no rotation needed
        return UnitQuaternion::identity();
    }

    // Need to rotate to reach threshold angle
    let target_angle = threshold_angle.to_radians();

    // Rotation axis: perpendicular to normal and build direction
    let rotation_axis = unit_normal.cross(build_direction);

    // Safety: Check if cross product is valid
    if !rotation_axis.x.is_finite() || !rotation_axis.y.is_finite() || !rotation_axis.z.is_finite() {
        return UnitQuaternion::identity();
    }

    let axis_norm = rotation_axis.norm();
    if !axis_norm.is_finite() || axis_norm < 1e-6 {
        // Normal parallel to build direction, cannot rotate
        return UnitQuaternion::identity();
    }

    let axis = rotation_axis / axis_norm;  // Manual normalization

    // Safety check: verify axis is valid after division (should never fail now)
    if !axis.x.is_finite() || !axis.y.is_finite() || !axis.z.is_finite() {
        return UnitQuaternion::identity();
    }

    // Rotation amount to reach target angle
    // NOTE: target_angle - angle (not angle - target_angle) to rotate TOWARD vertical
    // Positive rotation makes surface more vertical (away from mesh center)
    let mut rotation_amount = target_angle - angle;

    if !rotation_amount.is_finite() || rotation_amount.abs() < 1e-10 {
        return UnitQuaternion::identity();
    }

    // CRITICAL: Limit maximum rotation to prevent extreme deformations
    // S3-Slicer uses gradual adjustments, not aggressive flips
    let max_rotation_radians = max_rotation_degrees.to_radians();
    if rotation_amount.abs() > max_rotation_radians {
        rotation_amount = rotation_amount.signum() * max_rotation_radians;
    }

    // Create quaternion safely
    let quat = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_unchecked(axis),  // Use unchecked since we already normalized
        rotation_amount,
    );

    // Final safety check: verify quaternion is valid
    if !quat.coords.x.is_finite() || !quat.coords.y.is_finite() ||
       !quat.coords.z.is_finite() || !quat.coords.w.is_finite() {
        log::warn!("  WARNING: Invalid quaternion produced in support-free rotation");
        return UnitQuaternion::identity();
    }

    quat
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
    fn test_support_free_rotation() {
        let normal = Vector3D::new(1.0, 0.0, 0.0); // Horizontal surface
        let build_dir = Vector3D::new(0.0, 0.0, 1.0); // Vertical build

        let rotation = compute_support_free_rotation(&normal, &build_dir, 45.0);

        // Should rotate to make surface self-supporting
        assert!(rotation.angle() > 0.0);
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
