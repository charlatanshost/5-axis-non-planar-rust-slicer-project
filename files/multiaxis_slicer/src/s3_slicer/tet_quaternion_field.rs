// Volumetric Quaternion Field for Tetrahedral Meshes
// Per-tetrahedron rotation field optimized for fabrication objectives
// Uses tet face adjacency for smoothness (shared faces instead of shared edges)

use crate::geometry::Vector3D;
use crate::s3_slicer::tet_mesh::TetMesh;
use crate::s3_slicer::quaternion_field::{QuaternionFieldConfig, FabricationObjective};
use nalgebra::UnitQuaternion;
use rayon::prelude::*;

/// Quaternion field defined over a tetrahedral mesh
#[derive(Clone)]
pub struct TetQuaternionField {
    /// Quaternion rotation per tetrahedron
    pub rotations: Vec<UnitQuaternion<f64>>,

    /// Energy of the field (lower = better)
    pub energy: f64,

    /// Configuration used
    pub config: QuaternionFieldConfig,
}

impl TetQuaternionField {
    /// Optimize quaternion field on a tetrahedral mesh
    pub fn optimize(tet_mesh: &TetMesh, config: QuaternionFieldConfig) -> Self {
        let num_tets = tet_mesh.tets.len();
        log::info!("Optimizing tet quaternion field for {} tetrahedra...", num_tets);

        // Initialize with identity rotations
        let mut rotations: Vec<UnitQuaternion<f64>> = vec![UnitQuaternion::identity(); num_tets];

        // Pre-compute representative normals for each tet
        // Use volume-weighted average of face normals (biased toward surface-facing)
        let tet_normals: Vec<Vector3D> = (0..num_tets)
            .map(|ti| compute_tet_representative_normal(tet_mesh, ti))
            .collect();

        // Per-tet objective rotation, computed once.
        let objectives: Vec<UnitQuaternion<f64>> = (0..num_tets)
            .into_par_iter()
            .map(|ti| {
                compute_optimal_rotation(
                    &tet_normals[ti],
                    &config.build_direction,
                    &config.objective,
                    config.overhang_threshold,
                    config.max_rotation_degrees,
                )
            })
            .collect();

        // Which tets genuinely have something to ask for. Only a tet with a
        // surface face can see an overhang; `compute_tet_representative_normal`
        // hands interior tets a synthetic upward normal, which no fabrication
        // objective ever wants to rotate.
        let anchored: Vec<bool> = (0..num_tets)
            .map(|ti| {
                objectives[ti].angle() > 1e-6
                    && tet_mesh.tet_neighbors[ti].iter().any(|nb| nb.is_none())
            })
            .collect();

        // Base tets stay put so the build plate never moves. Without this the
        // diffusion below would carry rotation all the way down and tilt the
        // part off the bed.
        let z_lo = tet_mesh.bounds_min.z;
        let z_range = (tet_mesh.bounds_max.z - z_lo).max(1e-10);
        let base: Vec<bool> = (0..num_tets)
            .map(|ti| {
                let c = tet_mesh.tets[ti].centroid(&tet_mesh.vertices);
                (c.z - z_lo) / z_range < BASE_PIN_FRACTION
            })
            .collect();

        let num_anchored = anchored.iter().filter(|&&a| a).count();
        let num_base = base.iter().filter(|&&b| b).count();
        log::info!("    Anchored (overhang) tets: {}, pinned base tets: {}",
            num_anchored, num_base);

        // Relaxation.
        //
        // This used to recompute `optimal` from the tet's own normal on every
        // pass and blend that with the neighbours, which re-injected identity
        // into the interior every iteration and so forced the field to decay
        // exponentially away from the surface. Measured on a sphere whose tet
        // mesh is 13 adjacency hops deep, the mean rotation fell 2.14 deg ->
        // 0.44 -> 0.10 -> 0.02 and was zero by the fourth hop; 400 iterations
        // gave bit-identical results to 50, because that decay is the fixed
        // point rather than a failure to converge. The thin shell that
        // survived was then averaged away by the global ASAP solve, which saw
        // 95% of its edges demanding no change at all, leaving a total
        // deformation of 0.026mm on a 48mm model.
        //
        // Instead, treat it as what it should be: a diffusion with boundary
        // conditions. Overhang tets hold their objective, base tets hold
        // identity, and every other tet is free to take the average of its
        // neighbours without being dragged back toward identity. The field
        // then varies smoothly through the volume.
        for iteration in 0..config.optimization_iterations {
            let new_rotations: Vec<UnitQuaternion<f64>> = (0..num_tets)
                .into_par_iter()
                .map(|ti| {
                    if base[ti] {
                        return UnitQuaternion::identity();
                    }

                    let neighbors: Vec<usize> = tet_mesh.tet_neighbors[ti]
                        .iter()
                        .filter_map(|n| *n)
                        .collect();

                    if anchored[ti] {
                        // Hold the objective, but let smoothing soften the
                        // boundary between neighbouring overhang regions.
                        if config.smoothness_weight > 0.0 {
                            smooth_with_neighbors(
                                &objectives[ti],
                                &rotations,
                                &neighbors,
                                config.smoothness_weight,
                            )
                        } else {
                            objectives[ti]
                        }
                    } else {
                        // Free tet: pure neighbour average, no identity pull.
                        average_neighbors(&rotations, &neighbors)
                            .unwrap_or(rotations[ti])
                    }
                })
                .collect();

            rotations = new_rotations;

            if iteration % 10 == 0 {
                log::debug!("  Tet quaternion field iteration {}/{}", iteration, config.optimization_iterations);
            }
        }

        // Replace invalid quaternions
        let invalid_count = rotations.iter()
            .filter(|q| !q.coords.x.is_finite() || !q.coords.y.is_finite() ||
                        !q.coords.z.is_finite() || !q.coords.w.is_finite())
            .count();

        if invalid_count > 0 {
            log::warn!("  {} invalid quaternions replaced with identity", invalid_count);
            for q in &mut rotations {
                if !q.coords.x.is_finite() || !q.coords.y.is_finite() ||
                   !q.coords.z.is_finite() || !q.coords.w.is_finite() {
                    *q = UnitQuaternion::identity();
                }
            }
        }

        // Compute energy
        let energy = compute_field_energy(&rotations, tet_mesh, &tet_normals, &config);

        let non_identity = rotations.iter().filter(|q| q.angle() > 0.01).count();
        let max_angle = rotations.iter().map(|q| q.angle()).fold(0.0f64, f64::max);

        log::info!("  Tet quaternion field optimized:");
        log::info!("    Tets with rotation: {} ({:.1}%)", non_identity, 100.0 * non_identity as f64 / num_tets as f64);
        log::info!("    Max rotation: {:.2} deg", max_angle.to_degrees());
        log::info!("    Energy: {:.4}", energy);

        Self { rotations, energy, config }
    }

    /// Get rotation for a specific tet
    pub fn rotation_at(&self, tet_idx: usize) -> UnitQuaternion<f64> {
        self.rotations.get(tet_idx).copied().unwrap_or(UnitQuaternion::identity())
    }

    /// Interpolate rotation at a point using tet-local barycentric coordinates
    pub fn interpolate_at_vertex(&self, tet_mesh: &TetMesh, vertex_idx: usize) -> UnitQuaternion<f64> {
        let tets = &tet_mesh.vertex_tets[vertex_idx];
        if tets.is_empty() {
            return UnitQuaternion::identity();
        }

        // Average rotations from all tets containing this vertex
        let mut result = self.rotation_at(tets[0]);
        for &ti in tets.iter().skip(1) {
            let weight = 1.0 / (tets.len() as f64);
            let r = self.rotation_at(ti);
            result = result.slerp(&r, weight);
        }
        result
    }
}

/// Compute a representative normal direction for a tetrahedron
/// For interior tets: use gradient of Z (build direction proxy)
/// For surface tets: use the surface face normal
fn compute_tet_representative_normal(tet_mesh: &TetMesh, tet_idx: usize) -> Vector3D {
    let tet = &tet_mesh.tets[tet_idx];
    let neighbors = &tet_mesh.tet_neighbors[tet_idx];

    // Check which faces are on the surface (no neighbor)
    let faces = tet.faces();
    let normals = tet.face_normals(&tet_mesh.vertices);

    let mut surface_normal = Vector3D::new(0.0, 0.0, 0.0);
    let mut has_surface_face = false;

    for (fi, neighbor) in neighbors.iter().enumerate() {
        if neighbor.is_none() {
            // This face is on the surface
            let n = normals[fi];
            let area = n.norm();
            if area > 1e-10 {
                surface_normal += n; // Area-weighted (already unnormalized = 2*area)
                has_surface_face = true;
            }
        }
    }

    if has_surface_face {
        let len = surface_normal.norm();
        if len > 1e-10 {
            return surface_normal / len;
        }
    }

    // Interior tet: use Z-gradient as proxy (how quickly Z changes across this tet)
    let centroid = tet.centroid(&tet_mesh.vertices);
    let z_range = tet_mesh.bounds_max.z - tet_mesh.bounds_min.z;
    let height_param = if z_range > 1e-10 {
        (centroid.z - tet_mesh.bounds_min.z) / z_range
    } else {
        0.5
    };

    // For interior tets, blend between vertical and slightly tilted
    // based on height - bottom tets face more upward, top tets vary
    Vector3D::new(0.0, 0.0, 1.0 - height_param * 0.5)
        .normalize()
}

/// Compute optimal rotation for fabrication objective
fn compute_optimal_rotation(
    normal: &Vector3D,
    build_direction: &Vector3D,
    objective: &FabricationObjective,
    overhang_threshold: f64,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    match objective {
        FabricationObjective::SupportFree => {
            compute_unfold_rotation(normal, overhang_threshold, max_rotation_degrees)
        }
        FabricationObjective::Strength => {
            compute_strength_rotation(normal, build_direction, max_rotation_degrees)
        }
        FabricationObjective::SurfaceQuality => {
            UnitQuaternion::identity()
        }
        FabricationObjective::Balanced => {
            let unfold = compute_unfold_rotation(normal, overhang_threshold, max_rotation_degrees);
            let strength = compute_strength_rotation(normal, build_direction, max_rotation_degrees);
            unfold.slerp(&strength, 0.5)
        }
    }
}

/// Compute support-free unfolding rotation
fn compute_unfold_rotation(
    normal: &Vector3D,
    overhang_threshold: f64,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    if !normal.x.is_finite() || !normal.y.is_finite() || !normal.z.is_finite() {
        return UnitQuaternion::identity();
    }

    let normal_len = normal.norm();
    if normal_len < 1e-10 {
        return UnitQuaternion::identity();
    }

    let unit_normal = normal / normal_len;
    let overhang_rad = overhang_threshold.to_radians();
    let min_z = -overhang_rad.sin();

    // Already self-supporting
    if unit_normal.z >= min_z {
        return UnitQuaternion::identity();
    }

    // Compute rotation to fix overhang
    let horiz_len = (unit_normal.x * unit_normal.x + unit_normal.y * unit_normal.y).sqrt();

    let rotation_axis = if horiz_len > 1e-6 {
        Vector3D::new(-unit_normal.y / horiz_len, unit_normal.x / horiz_len, 0.0)
    } else {
        Vector3D::new(1.0, 0.0, 0.0)
    };

    let current_angle = (-unit_normal.z).asin();
    let target_angle = overhang_rad;
    let rotation_amount = current_angle - target_angle;

    let max_rad = max_rotation_degrees.to_radians();
    let clamped = rotation_amount.clamp(0.0, max_rad);

    if clamped < 1e-6 {
        return UnitQuaternion::identity();
    }

    let quat = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(rotation_axis),
        clamped,
    );

    if quat.coords.iter().all(|c| c.is_finite()) {
        quat
    } else {
        UnitQuaternion::identity()
    }
}

/// Compute strength rotation
fn compute_strength_rotation(
    normal: &Vector3D,
    build_direction: &Vector3D,
    max_rotation_degrees: f64,
) -> UnitQuaternion<f64> {
    let full_rotation = UnitQuaternion::rotation_between(normal, build_direction)
        .unwrap_or(UnitQuaternion::identity());

    let angle = full_rotation.angle();
    let max_rad = max_rotation_degrees.to_radians();

    if angle > max_rad {
        let scale = max_rad / angle;
        let axis = full_rotation.axis().map(|ax| ax.into_inner())
            .unwrap_or(Vector3D::new(0.0, 0.0, 1.0));
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle * scale)
    } else {
        full_rotation
    }
}

/// Smooth rotation with face-adjacent neighbors
/// Fraction of the model height whose tets are pinned to identity, so the
/// part stays flat on the build plate while rotation diffuses above it.
const BASE_PIN_FRACTION: f64 = 0.05;

/// Running slerp average of the neighbours' rotations.
///
/// Returns `None` when there is nothing to average, so the caller can keep its
/// current value rather than snapping to identity.
fn average_neighbors(
    current_rotations: &[UnitQuaternion<f64>],
    neighbor_indices: &[usize],
) -> Option<UnitQuaternion<f64>> {
    let mut accumulated: Option<UnitQuaternion<f64>> = None;
    let mut count = 0.0f64;

    for &ni in neighbor_indices {
        let Some(neighbor) = current_rotations.get(ni) else { continue };
        if !neighbor.coords.iter().all(|c| c.is_finite()) {
            continue;
        }
        match accumulated {
            None => {
                accumulated = Some(*neighbor);
                count = 1.0;
            }
            Some(acc) => {
                // Weight 1/(n+1) folds the (n+1)-th sample into the mean.
                let t = 1.0 / (count + 1.0);
                let blended = acc.slerp(neighbor, t);
                if blended.coords.iter().all(|c| c.is_finite()) {
                    accumulated = Some(blended);
                    count += 1.0;
                }
            }
        }
    }

    accumulated
}

fn smooth_with_neighbors(
    optimal: &UnitQuaternion<f64>,
    current_rotations: &[UnitQuaternion<f64>],
    neighbor_indices: &[usize],
    smoothness_weight: f64,
) -> UnitQuaternion<f64> {
    if neighbor_indices.is_empty() {
        return *optimal;
    }

    let mut accumulated = *optimal;
    let mut count = 1.0;

    for &ni in neighbor_indices {
        if ni < current_rotations.len() {
            let neighbor = &current_rotations[ni];
            if !neighbor.coords.iter().all(|c| c.is_finite()) {
                continue;
            }

            let weight = smoothness_weight / (count + 1.0);
            if weight > 0.0 && weight < 1.0 {
                let result = accumulated.slerp(neighbor, weight);
                if result.coords.iter().all(|c| c.is_finite()) {
                    accumulated = result;
                    count += 1.0;
                }
            }
        }
    }

    if accumulated.coords.iter().all(|c| c.is_finite()) {
        accumulated
    } else {
        *optimal
    }
}

/// Compute field energy
fn compute_field_energy(
    rotations: &[UnitQuaternion<f64>],
    tet_mesh: &TetMesh,
    tet_normals: &[Vector3D],
    config: &QuaternionFieldConfig,
) -> f64 {
    let mut energy = 0.0;

    // Smoothness: penalize rotation differences between adjacent tets
    for (ti, neighbors) in tet_mesh.tet_neighbors.iter().enumerate() {
        for neighbor in neighbors.iter().filter_map(|n| *n) {
            let diff = rotations[ti].inverse() * rotations[neighbor];
            let angle = diff.angle();
            energy += config.smoothness_weight * angle * angle;
        }
    }

    // Objective: penalize overhangs after rotation
    for (ti, normal) in tet_normals.iter().enumerate() {
        let rotated_normal = rotations[ti] * *normal;
        let cos_angle = rotated_normal.dot(&config.build_direction)
            / (rotated_normal.norm() * config.build_direction.norm()).max(1e-10);
        let angle_deg = cos_angle.clamp(-1.0, 1.0).acos().to_degrees();

        let overhang_penalty = if angle_deg > (90.0 - config.overhang_threshold) {
            (angle_deg - (90.0 - config.overhang_threshold)).powi(2)
        } else {
            0.0
        };

        energy += config.objective_weight * overhang_penalty;
    }

    energy
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point3D;
    use crate::s3_slicer::tet_mesh::Tetrahedron;

    fn rot_z(deg: f64) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(&Vector3D::z_axis(), deg.to_radians())
    }

    /// Regression test for D-10.
    ///
    /// A tet with no fabrication objective of its own must be free to take its
    /// neighbours' rotation. The relaxation used to recompute the objective
    /// from the tet's own normal every pass and blend that in, and
    /// `compute_tet_representative_normal` hands interior tets a synthetic
    /// upward normal that no objective ever rotates -- so identity was
    /// re-injected into the interior on every iteration and the field decayed
    /// exponentially from the surface.
    ///
    /// `average_neighbors` is the replacement, and the property that matters
    /// is that it never drags a value toward identity.
    #[test]
    fn test_average_neighbors_does_not_pull_toward_identity() {
        let a = rot_z(30.0);
        let rotations = vec![a, a, a];

        let avg = average_neighbors(&rotations, &[0, 1, 2])
            .expect("three neighbours should average");

        // Averaging equal rotations must reproduce them, not dilute them.
        assert!(
            (avg.angle() - a.angle()).abs() < 1e-9,
            "average of three 30 deg rotations gave {:.4} deg",
            avg.angle().to_degrees()
        );
    }

    #[test]
    fn test_average_neighbors_interpolates() {
        // 0 and 60 degrees about the same axis average to 30.
        let rotations = vec![rot_z(0.0), rot_z(60.0)];
        let avg = average_neighbors(&rotations, &[0, 1]).expect("two neighbours");
        assert!(
            (avg.angle().to_degrees() - 30.0).abs() < 1e-6,
            "expected 30 deg, got {:.4}",
            avg.angle().to_degrees()
        );
    }

    #[test]
    fn test_average_neighbors_reports_nothing_to_average() {
        let rotations = vec![rot_z(30.0)];
        // No neighbours at all: the caller must be able to keep its own value
        // rather than being handed identity.
        assert!(average_neighbors(&rotations, &[]).is_none());
        // Out-of-range indices are skipped, not treated as identity.
        assert!(average_neighbors(&rotations, &[99]).is_none());
    }

    #[test]
    fn test_average_neighbors_skips_non_finite() {
        let good = rot_z(45.0);
        let bad = UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(
            f64::NAN, 0.0, 0.0, 0.0,
        ));
        let rotations = vec![bad, good];

        let avg = average_neighbors(&rotations, &[0, 1]).expect("one good neighbour");
        assert!((avg.angle() - good.angle()).abs() < 1e-9);
        assert!(avg.coords.iter().all(|c| c.is_finite()));
    }

    fn make_simple_tet_mesh() -> TetMesh {
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(5.0, 10.0, 0.0),
            Point3D::new(5.0, 5.0, 10.0),
            Point3D::new(5.0, 5.0, -5.0), // Below, creates overhang
        ];

        let tets = vec![
            Tetrahedron::new(0, 1, 2, 3),
            Tetrahedron::new(0, 1, 2, 4),
        ];

        TetMesh::new(vertices, tets)
    }

    #[test]
    fn test_tet_quaternion_field_optimization() {
        let tet_mesh = make_simple_tet_mesh();
        let config = QuaternionFieldConfig::default();
        let field = TetQuaternionField::optimize(&tet_mesh, config);

        assert_eq!(field.rotations.len(), 2);
        assert!(field.energy >= 0.0);
    }

    #[test]
    fn test_rotation_at() {
        let tet_mesh = make_simple_tet_mesh();
        let config = QuaternionFieldConfig::default();
        let field = TetQuaternionField::optimize(&tet_mesh, config);

        let r = field.rotation_at(0);
        assert!(r.coords.iter().all(|c| c.is_finite()));
    }

    #[test]
    fn test_interpolate_at_vertex() {
        let tet_mesh = make_simple_tet_mesh();
        let config = QuaternionFieldConfig::default();
        let field = TetQuaternionField::optimize(&tet_mesh, config);

        // Vertex 0 is shared by both tets
        let r = field.interpolate_at_vertex(&tet_mesh, 0);
        assert!(r.coords.iter().all(|c| c.is_finite()));
    }
}
