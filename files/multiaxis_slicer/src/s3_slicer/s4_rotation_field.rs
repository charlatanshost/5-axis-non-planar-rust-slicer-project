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
    /// Compute S4 rotation field from overhang analysis and Dijkstra ordering.
    ///
    /// Algorithm:
    /// 1. Surface overhang tets get quaternion via normal × Z (Phase 1).
    /// 2. All Phase 1 quaternions are re-expressed on the **dominant axis** (the axis of
    ///    the largest-magnitude Phase 1 tet).  This eliminates conflicting axes:  when
    ///    multiple overhangs face different directions, SLERP-averaging conflicting axes
    ///    in `deform_tet_mesh_direct` produces degenerate rotations → mesh collapse.
    ///    With one consistent axis, propagation and averaging are stable.
    /// 3. Propagate full quaternions upward through Dijkstra ordering (Phase 2).
    /// 4. Smooth via iterative SLERP with face-adjacent neighbors (softens boundaries).
    pub fn compute(
        tet_mesh: &TetMesh,
        dijkstra_field: &TetDijkstraField,
        config: &S4RotationConfig,
    ) -> Self {
        let num_tets = tet_mesh.tets.len();
        log::info!("Computing S4 rotation field for {} tets...", num_tets);
        log::info!("  Overhang threshold: {:.1} deg, max rotation: {:.1} deg",
            config.overhang_threshold, config.max_rotation_degrees);

        // Step 1–2: surface normal detection + dominant-axis normalization.
        // Phase 2 (upward propagation) is intentionally omitted — it created a sharp
        // rotation cliff at the belly/legs boundary, which combined with the global
        // anchor in deform_tet_mesh_direct caused a visible shear tear in the mesh.
        // Instead, diffusion (Step 3 below) spreads the Phase 1 rotations smoothly.
        let (mut rotations, magnitudes) =
            compute_initial_rotations(tet_mesh, dijkstra_field, config);

        let non_identity = rotations.iter().filter(|q| q.angle() > 0.01).count();
        let max_angle = rotations.iter().map(|q| q.angle()).fold(0.0f64, f64::max);
        log::info!("  After Phase 1 + normalization: {} non-identity ({:.1}%), max: {:.2}°",
            non_identity, 100.0 * non_identity as f64 / num_tets as f64,
            max_angle.to_degrees());

        // Step 3: Diffusion smoothing — spreads Phase 1 surface rotations into the
        // interior while keeping base tets (build plate) pinned to identity.
        // This replaces the old Phase 2 upward propagation:  instead of a discrete
        // "cliff" at the belly/legs boundary, we get a smooth gradient from the
        // overhang surface tets (full rotation) decaying toward the build plate (zero).
        let base_tets: std::collections::HashSet<usize> =
            dijkstra_field.base_tets.iter().cloned().collect();

        for iter in 0..config.smoothing_iterations {
            let new_rotations: Vec<UnitQuaternion<f64>> = (0..num_tets)
                .into_par_iter()
                .map(|ti| {
                    if base_tets.contains(&ti) {
                        UnitQuaternion::identity() // pin build-plate tets to zero
                    } else {
                        smooth_single_tet(ti, &rotations, tet_mesh, config.smoothness_weight)
                    }
                })
                .collect();
            rotations = new_rotations;

            if iter % 10 == 0 {
                log::debug!("  Diffusion iter {}/{}", iter, config.smoothing_iterations);
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

/// Compute per-tet rotations from surface overhang analysis with dominant-axis normalization.
///
/// Two-phase algorithm (Phase 2 / upward propagation intentionally removed):
///
/// **Phase 1 — Surface quaternions**: For each surface tet with a downward-facing
/// boundary face, compute rotation quaternion: axis = `normal × build_dir`, magnitude =
/// how much the face exceeds the overhang threshold (clamped to max_rotation).
/// Interior tets remain identity.
///
/// **Dominant axis normalization**: Multiple overhang regions face different directions,
/// so Phase 1 produces conflicting axes.  SLERP-averaging conflicting axes at shared
/// vertices produces degenerate rotations.  Fix: find the most severe Phase 1 tet
/// and re-express ALL Phase 1 quaternions on that same dominant axis.
///
/// Diffusion (controlled by `smoothing_iterations` in `S4RotationField::compute`)
/// then spreads these surface rotations smoothly into the interior, with base tets
/// pinned to identity so the build plate never moves.
fn compute_initial_rotations(
    tet_mesh: &TetMesh,
    dijkstra_field: &TetDijkstraField,
    config: &S4RotationConfig,
) -> (Vec<UnitQuaternion<f64>>, Vec<f64>) {
    let num_tets = tet_mesh.tets.len();
    let overhang_rad = config.overhang_threshold.to_radians();
    let max_rad = config.max_rotation_degrees.to_radians();
    let min_z = -overhang_rad.sin(); // Normal Z below this threshold = overhang

    // ── Phase 1: surface overhang tets → per-tet quaternion via normal × Z ────
    let mut rotations = vec![UnitQuaternion::identity(); num_tets];

    for ti in 0..num_tets {
        let neighbors = &tet_mesh.tet_neighbors[ti];

        // Accumulate area-weighted normals from boundary (surface) faces
        let mut surface_normal = Vector3D::zeros();
        let mut has_surface = false;
        let face_normals = tet_mesh.tets[ti].face_normals(&tet_mesh.vertices);

        for fi in 0..4 {
            if neighbors[fi].is_none() {
                let n = face_normals[fi];
                let area = n.norm();
                if area > 1e-10 {
                    surface_normal += n;
                    has_surface = true;
                }
            }
        }

        if !has_surface { continue; }
        let norm = surface_normal.norm();
        if norm < 1e-10 { continue; }
        let unit_normal = surface_normal / norm;
        if unit_normal.z >= min_z { continue; } // not an overhang

        let current_angle = (-unit_normal.z).asin().clamp(0.0, std::f64::consts::FRAC_PI_2);
        let rotation_amount = (current_angle - overhang_rad).clamp(0.0, max_rad);
        if rotation_amount < 1e-6 { continue; }

        // Axis = normal × build_dir = (ny, -nx, 0) — the horizontal axis that
        // lifts this overhang toward +Z.
        let horiz_len = (unit_normal.x * unit_normal.x + unit_normal.y * unit_normal.y).sqrt();
        let rotation_axis = if horiz_len > 1e-6 {
            Vector3D::new(unit_normal.y / horiz_len, -unit_normal.x / horiz_len, 0.0)
        } else {
            Vector3D::new(1.0, 0.0, 0.0)
        };

        let q = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(rotation_axis),
            rotation_amount,
        );
        if q.coords.iter().all(|c| c.is_finite()) {
            rotations[ti] = q;
        }
    }

    let phase1_count = rotations.iter().filter(|q| q.angle() > 1e-6).count();
    let phase1_max   = rotations.iter().map(|q| q.angle()).fold(0.0f64, f64::max);
    log::info!("  Phase 1: {} surface overhang tets, max: {:.2}°",
        phase1_count, phase1_max.to_degrees());

    // ── Dominant axis normalization ───────────────────────────────────────────
    // Find the most-severe Phase 1 tet and use its axis for ALL Phase 1 tets.
    // This guarantees that every rotation in the field shares the same axis →
    // SLERP averages and upward propagation are numerically stable (no collapse).
    let dominant_axis: Option<Vector3D> = (0..num_tets)
        .filter(|&ti| rotations[ti].angle() > 1e-6)
        .max_by(|&a, &b| {
            rotations[a].angle().partial_cmp(&rotations[b].angle())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .and_then(|dom| rotations[dom].axis())
        .map(|ax| ax.into_inner());

    if let Some(dom_ax) = dominant_axis {
        let dom_unit = nalgebra::Unit::new_normalize(dom_ax);
        let mut normalized = 0usize;
        for ti in 0..num_tets {
            let mag = rotations[ti].angle();
            if mag > 1e-6 {
                rotations[ti] = UnitQuaternion::from_axis_angle(&dom_unit, mag);
                normalized += 1;
            }
        }
        log::info!("  Dominant axis: ({:.3},{:.3},{:.3}), normalized {} tets",
            dom_ax.x, dom_ax.y, dom_ax.z, normalized);
    } else {
        log::info!("  No overhang tets found in Phase 1 — field will be identity.");
    }

    // Phase 2 (upward propagation) removed — it caused a sharp rotation cliff at
    // the boundary between propagated and non-propagated tets, which combined with
    // the global anchor in deform_tet_mesh_direct to create a shear tear.
    // Diffusion in S4RotationField::compute() spreads the Phase 1 values smoothly.

    let magnitudes: Vec<f64> = rotations.iter().map(|q| q.angle()).collect();
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
