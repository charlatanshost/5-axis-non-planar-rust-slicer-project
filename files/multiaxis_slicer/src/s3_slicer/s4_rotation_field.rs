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

    /// Enable multi-axis overhang detection.
    /// When true, instead of forcing all Phase 1 rotations onto a single dominant axis,
    /// clusters overhangs by axis direction and handles up to 2 dominant directions.
    /// Better for models with overhangs facing multiple directions (e.g. bunny ears + belly).
    pub multi_axis: bool,
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
            multi_axis: false,
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
        Self::compute_for_surface(tet_mesh, dijkstra_field, config, None)
    }

    /// As `compute`, reading overhangs off the model's own surface.
    ///
    /// Without `surface` the overhang direction comes from the boundary faces
    /// of the tet mesh. That is the model's surface only if the tet mesh
    /// conforms to it. The grid tetrahedralizer, which is what any model over
    /// 10 000 triangles gets, does not: a slope is a staircase of level steps
    /// and vertical risers. A level step faces straight down, which says
    /// nothing about which way the slope faces, and every one of them was
    /// given the same arbitrary rotation axis. On an inverted pyramid that
    /// tilted two opposite faces the same way: one with its overhang, the
    /// other against it.
    pub fn compute_for_surface(
        tet_mesh: &TetMesh,
        dijkstra_field: &TetDijkstraField,
        config: &S4RotationConfig,
        surface: Option<&crate::mesh::Mesh>,
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
            compute_initial_rotations(tet_mesh, dijkstra_field, config, surface);

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

        // The overhang tets are boundary conditions too. Diffusion with only
        // the base pinned has one steady state, identity everywhere, and the
        // iteration heads for it: the Phase 1 rotations were averaged away
        // with their neighbours until nothing was left. A Benchy asked for
        // 15 degrees came out with 1.00 at most and 0.8 % of tets rotated at
        // all, so the mode sliced planar. Holding the seeds makes this an
        // interpolation between "rotate this much here" and "not at all on
        // the bed", which is what the comment above describes.
        let seeds: Vec<Option<UnitQuaternion<f64>>> = rotations
            .iter()
            .enumerate()
            .map(|(ti, q)| (q.angle() > 1e-6 && !base_tets.contains(&ti)).then_some(*q))
            .collect();

        for iter in 0..config.smoothing_iterations {
            let new_rotations: Vec<UnitQuaternion<f64>> = (0..num_tets)
                .into_par_iter()
                .map(|ti| {
                    if base_tets.contains(&ti) {
                        UnitQuaternion::identity() // pin build-plate tets to zero
                    } else if let Some(seed) = seeds[ti] {
                        seed // pin overhang tets to what they asked for
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
    surface: Option<&crate::mesh::Mesh>,
) -> (Vec<UnitQuaternion<f64>>, Vec<f64>) {
    let num_tets = tet_mesh.tets.len();
    let overhang_rad = config.overhang_threshold.to_radians();
    let max_rad = config.max_rotation_degrees.to_radians();
    let min_z = -overhang_rad.sin(); // Normal Z below this threshold = overhang

    // ── Phase 1: surface overhang tets → per-tet quaternion via normal × Z ────
    let mut rotations = vec![UnitQuaternion::identity(); num_tets];

    // Area-weighted outward normal of the surface passing through each tet;
    // zero where there is none.
    let tet_normals: Vec<Vector3D> = match surface {
        Some(surface) => surface_normals_per_tet(tet_mesh, surface),
        None => (0..num_tets)
            .map(|ti| {
                let neighbors = &tet_mesh.tet_neighbors[ti];
                let face_normals = tet_mesh.tets[ti].face_normals(&tet_mesh.vertices);
                let mut sum = Vector3D::zeros();
                for fi in 0..4 {
                    if neighbors[fi].is_none() && face_normals[fi].norm() > 1e-10 {
                        sum += face_normals[fi];
                    }
                }
                sum
            })
            .collect(),
    };

    for ti in 0..num_tets {
        let surface_normal = tet_normals[ti];
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
        // A face looking straight down has no sideways direction of its own.
        // It used to be given the X axis regardless, which tilts the
        // underside of a cap the same way all round: right on one side,
        // wrong on the other. What does have a direction there is the route
        // back to the bed. The Dijkstra distance grows outward along the
        // underside, away from whatever holds it up, and that is the way
        // such a face should be treated as facing. If that is level too,
        // nothing says which way to turn, so leave the tet to its neighbours.
        let (hx, hy) = if horiz_len >= 0.05 {
            (unit_normal.x / horiz_len, unit_normal.y / horiz_len)
        } else {
            let g = dijkstra_field.gradients.get(ti).copied().unwrap_or_else(Vector3D::zeros);
            let g_horiz = (g.x * g.x + g.y * g.y).sqrt();
            if !g_horiz.is_finite() || g_horiz < 0.05 * g.norm().max(1e-12) { continue; }
            (g.x / g_horiz, g.y / g_horiz)
        };
        let rotation_axis = Vector3D::new(hy, -hx, 0.0);

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

    // ── Axis normalization ──────────────────────────────────────────────────
    if config.multi_axis {
        // Multi-axis mode: cluster Phase 1 rotations by axis direction,
        // keep up to 2 dominant clusters, normalize within each cluster.
        normalize_multi_axis(&mut rotations, num_tets);
    } else {
        // Single-axis mode: force all Phase 1 rotations onto the dominant axis.
        normalize_single_axis(&mut rotations, num_tets);
    }

    // Phase 2 (upward propagation) removed — it caused a sharp rotation cliff at
    // the boundary between propagated and non-propagated tets, which combined with
    // the global anchor in deform_tet_mesh_direct to create a shear tear.
    // Diffusion in S4RotationField::compute() spreads the Phase 1 values smoothly.

    let magnitudes: Vec<f64> = rotations.iter().map(|q| q.angle()).collect();
    (rotations, magnitudes)
}

/// Sum of the model surface's area-weighted normals in each tet.
///
/// The surface is refined first so that one large triangle contributes to
/// every tet it crosses and not only to the one holding its centroid.
fn surface_normals_per_tet(tet_mesh: &TetMesh, surface: &crate::mesh::Mesh) -> Vec<Vector3D> {
    use crate::s3_slicer::surface_layers::{refine_edge_limit, refine_surface};
    use crate::s3_slicer::tet_point_location::TetSpatialGrid;

    let grid = TetSpatialGrid::build(tet_mesh);
    let triangles = refine_surface(surface, refine_edge_limit(surface));

    let located: Vec<(usize, Vector3D)> = triangles
        .par_iter()
        .filter_map(|t| {
            // Cross product: length is twice the area, direction is outward
            // for a consistently wound mesh.
            let n = (t.v1 - t.v0).cross(&(t.v2 - t.v0));
            if n.norm() < 1e-12 {
                return None;
            }
            let c = crate::geometry::Point3D::new(
                (t.v0.x + t.v1.x + t.v2.x) / 3.0,
                (t.v0.y + t.v1.y + t.v2.y) / 3.0,
                (t.v0.z + t.v1.z + t.v2.z) / 3.0,
            );
            let loc = grid
                .find_containing_tet(&c, tet_mesh)
                .unwrap_or_else(|| grid.find_nearest_tet(&c, tet_mesh));
            Some((loc.tet_index, n))
        })
        .collect();

    let mut normals = vec![Vector3D::zeros(); tet_mesh.tets.len()];
    for (ti, n) in located {
        normals[ti] += n;
    }
    normals
}

/// The part of a rotation that is about `axis`: same axis, signed angle.
///
/// Overhangs on opposite sides of a part ask for rotations about opposite
/// axes. Putting them on a common axis has to keep that: the far side's
/// rotation becomes a NEGATIVE angle about the shared axis. Rebuilding every
/// rotation as (shared axis, |angle|) instead turned the far side the same
/// way as the near side, which is the wrong way for it -- on an inverted
/// pyramid two faces were tilted to follow their overhang and the two
/// opposite were tilted against it by the same amount.
///
/// An overhang facing along the axis has no component about it and is left
/// alone, rather than being rotated at full strength in a direction that
/// does nothing for it.
fn project_onto_axis(
    q: &UnitQuaternion<f64>,
    axis: &nalgebra::Unit<Vector3D>,
) -> UnitQuaternion<f64> {
    match q.axis() {
        Some(own) => {
            let signed = q.angle() * own.dot(axis);
            if signed.abs() < 1e-9 {
                UnitQuaternion::identity()
            } else {
                UnitQuaternion::from_axis_angle(axis, signed)
            }
        }
        None => UnitQuaternion::identity(),
    }
}

/// Single-axis normalization: force all Phase 1 rotations onto the dominant axis.
/// This guarantees stable SLERP averaging at the cost of multi-directional accuracy.
fn normalize_single_axis(rotations: &mut [UnitQuaternion<f64>], num_tets: usize) {
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
            if rotations[ti].angle() > 1e-6 {
                rotations[ti] = project_onto_axis(&rotations[ti], &dom_unit);
                normalized += 1;
            }
        }
        log::info!("  Dominant axis: ({:.3},{:.3},{:.3}), normalized {} tets",
            dom_ax.x, dom_ax.y, dom_ax.z, normalized);
    } else {
        log::info!("  No overhang tets found in Phase 1 — field will be identity.");
    }
}

/// Multi-axis normalization: cluster Phase 1 rotation axes into groups,
/// keep the top 2 clusters, and normalize within each cluster to its centroid axis.
/// This handles models with overhangs facing different directions better than
/// single-axis mode, while still preventing degenerate SLERP from too many axes.
fn normalize_multi_axis(rotations: &mut [UnitQuaternion<f64>], num_tets: usize) {
    // Collect all non-identity rotation axes with their magnitudes
    let mut axes: Vec<(usize, Vector3D, f64)> = Vec::new();
    for ti in 0..num_tets {
        let mag = rotations[ti].angle();
        if mag > 1e-6 {
            if let Some(ax) = rotations[ti].axis() {
                axes.push((ti, ax.into_inner(), mag));
            }
        }
    }

    if axes.is_empty() {
        log::info!("  No overhang tets found in Phase 1 — field will be identity.");
        return;
    }

    if axes.len() == 1 {
        log::info!("  Single overhang tet — no clustering needed.");
        return;
    }

    // Simple 2-means clustering by axis direction (using dot product as similarity)
    // Initialize with the most-severe axis and its most-different counterpart
    let primary_idx = axes.iter().enumerate()
        .max_by(|a, b| a.1.2.partial_cmp(&b.1.2).unwrap())
        .map(|(i, _)| i).unwrap();
    let primary_axis = axes[primary_idx].1;

    let secondary_idx = axes.iter().enumerate()
        .filter(|(i, _)| *i != primary_idx)
        .min_by(|(_, (_, ax_a, _)), (_, (_, ax_b, _))| {
            let dot_a = ax_a.dot(&primary_axis).abs();
            let dot_b = ax_b.dot(&primary_axis).abs();
            dot_a.partial_cmp(&dot_b).unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i);

    // If all axes are nearly parallel (dot > 0.8), fall back to single-axis
    let use_two_clusters = if let Some(sec_idx) = secondary_idx {
        let dot = axes[sec_idx].1.dot(&primary_axis).abs();
        dot < 0.8 // Only use 2 clusters if axes are sufficiently different
    } else {
        false
    };

    if !use_two_clusters {
        log::info!("  Multi-axis: all axes nearly parallel — using single dominant axis.");
        normalize_single_axis(rotations, num_tets);
        return;
    }

    let secondary_axis = axes[secondary_idx.unwrap()].1;

    // Assign each tet to the closer cluster
    let mut cluster1_sum = Vector3D::zeros();
    let mut cluster2_sum = Vector3D::zeros();
    let mut cluster1_count = 0usize;
    let mut cluster2_count = 0usize;

    // An axis and its negative are the same line, and a cluster holds both:
    // the two opposite faces of a part. Flip each onto the seed's side before
    // summing, or they cancel and the "mean axis" is whatever rounding left.
    for &(_ti, ax, mag) in &axes {
        let d1 = ax.dot(&primary_axis);
        let d2 = ax.dot(&secondary_axis);
        if d1.abs() >= d2.abs() {
            cluster1_sum += ax * (mag * d1.signum()); // weight by magnitude
            cluster1_count += 1;
        } else {
            cluster2_sum += ax * (mag * d2.signum());
            cluster2_count += 1;
        }
    }

    // Compute cluster centroid axes
    let c1_norm = cluster1_sum.norm();
    let c2_norm = cluster2_sum.norm();
    let c1_axis = if c1_norm > 1e-10 { cluster1_sum / c1_norm } else { primary_axis };
    let c2_axis = if c2_norm > 1e-10 { cluster2_sum / c2_norm } else { secondary_axis };

    let c1_unit = nalgebra::Unit::new_normalize(c1_axis);
    let c2_unit = nalgebra::Unit::new_normalize(c2_axis);

    // Normalize each tet to its cluster's axis
    let mut normalized = 0usize;
    for &(ti, ax, _) in &axes {
        let dot1 = ax.dot(&c1_axis).abs();
        let dot2 = ax.dot(&c2_axis).abs();
        let unit = if dot1 >= dot2 { &c1_unit } else { &c2_unit };
        rotations[ti] = project_onto_axis(&rotations[ti], unit);
        normalized += 1;
    }

    log::info!("  Multi-axis clustering: {} tets in cluster 1 ({:.3},{:.3},{:.3}), {} in cluster 2 ({:.3},{:.3},{:.3})",
        cluster1_count, c1_axis.x, c1_axis.y, c1_axis.z,
        cluster2_count, c2_axis.x, c2_axis.y, c2_axis.z);
    log::info!("  Normalized {} tets across 2 clusters", normalized);
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

    /// A closed axis-aligned box as triangles.
    fn box_tris(lo: [f64; 3], hi: [f64; 3]) -> Vec<crate::geometry::Triangle> {
        use crate::geometry::Triangle;
        let p = |x: usize, y: usize, z: usize| Point3D::new(
            if x == 0 { lo[0] } else { hi[0] },
            if y == 0 { lo[1] } else { hi[1] },
            if z == 0 { lo[2] } else { hi[2] },
        );
        let v = [p(0,0,0), p(1,0,0), p(1,1,0), p(0,1,0), p(0,0,1), p(1,0,1), p(1,1,1), p(0,1,1)];
        let mut t = Vec::new();
        for &(a, b, c, d) in &[(0,3,2,1), (4,5,6,7), (0,1,5,4), (1,2,6,5), (2,3,7,6), (3,0,4,7)] {
            t.push(Triangle::new(v[a], v[b], v[c]));
            t.push(Triangle::new(v[a], v[c], v[d]));
        }
        t
    }

    /// Regression test for D-24. Overhang tets are where the rotation is
    /// asked for, and the diffusion that spreads it inward must hold them
    /// there. It used to pin only the bed, so the requested rotation was
    /// averaged away: a Benchy asked for 15 degrees ended with 1.00 at most
    /// and the mode sliced planar.
    ///
    /// A mushroom -- a wide cap on a narrow stem -- has an unmistakable
    /// overhang under the cap and a bed to diffuse towards.
    #[test]
    fn test_overhang_rotations_survive_diffusion() {
        let mut tris = box_tris([8.0, 8.0, 0.0], [12.0, 12.0, 10.0]);
        tris.extend(box_tris([0.0, 0.0, 10.0], [20.0, 20.0, 14.0]));
        let surface = crate::mesh::Mesh::new(tris).unwrap();
        let mesh = TetMesh::from_surface_mesh_grid(&surface, 14).expect("grid tet mesh");
        let dijkstra = TetDijkstraField::compute(&mesh, 0.8);

        let config = S4RotationConfig { smoothing_iterations: 50, ..S4RotationConfig::default() };
        let seeded = S4RotationField::compute(
            &mesh, &dijkstra, &S4RotationConfig { smoothing_iterations: 0, ..config.clone() });
        let seeds = seeded.rotations.iter().filter(|q| q.angle() > 1e-6).count();
        assert!(seeds > 10, "the underside of the cap should seed rotations, got {}", seeds);

        let field = S4RotationField::compute(&mesh, &dijkstra, &config);
        let max_deg = field.rotations.iter().map(|q| q.angle()).fold(0.0f64, f64::max).to_degrees();
        assert!(
            max_deg > config.max_rotation_degrees - 0.1,
            "requested {} degrees under the overhang, {:.2} left after diffusion",
            config.max_rotation_degrees, max_deg
        );

        // And it must still have spread: more tets rotated than were seeded.
        let rotated = field.rotations.iter().filter(|q| q.angle() > 0.01).count();
        assert!(rotated > seeds, "diffusion did not spread the rotation ({} of {} seeds)", rotated, seeds);
        eprintln!("seeds {}, rotated after diffusion {}, max {:.2} deg", seeds, rotated, max_deg);
    }

    fn about(x: f64, y: f64, degrees: f64) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3D::new(x, y, 0.0)),
            degrees.to_radians(),
        )
    }

    /// Rotation vector: axis scaled by angle. Two rotations are the same turn
    /// exactly when these agree, whatever axis each happens to be stored on.
    fn rotation_vector(q: &UnitQuaternion<f64>) -> Vector3D {
        q.axis().map(|a| a.into_inner() * q.angle()).unwrap_or_else(Vector3D::zeros)
    }

    /// Regression test for D-26. Overhangs on opposite sides of a part ask
    /// for opposite rotations. Normalising onto a shared axis rebuilt each as
    /// (shared axis, |angle|), which turned the far side the same way as the
    /// near side: against its own overhang.
    #[test]
    fn test_axis_normalisation_keeps_the_direction_of_rotation() {
        // Single axis: two opposite faces.
        let asked = vec![about(0.0, 1.0, 12.0), about(0.0, -1.0, 10.0)];
        let mut got = asked.clone();
        normalize_single_axis(&mut got, 2);
        for (a, g) in asked.iter().zip(got.iter()) {
            assert!(
                (rotation_vector(a) - rotation_vector(g)).norm() < 1e-9,
                "single-axis: asked for {:?}, got {:?}", rotation_vector(a), rotation_vector(g)
            );
        }

        // Two axes: the four faces of a pyramid.
        let asked = vec![
            about(0.0, 1.0, 12.0), about(0.0, -1.0, 10.0),
            about(1.0, 0.0, 11.0), about(-1.0, 0.0, 9.0),
        ];
        let mut got = asked.clone();
        normalize_multi_axis(&mut got, 4);
        for (a, g) in asked.iter().zip(got.iter()) {
            assert!(
                (rotation_vector(a) - rotation_vector(g)).norm() < 1e-6,
                "multi-axis: asked for {:?}, got {:?}", rotation_vector(a), rotation_vector(g)
            );
        }

        // A rotation square to the shared axis has nothing to contribute and
        // must not be applied at full strength about it.
        let mut got = vec![about(0.0, 1.0, 12.0), about(1.0, 0.0, 10.0)];
        normalize_single_axis(&mut got, 2);
        assert!(got[1].angle() < 1e-9, "a perpendicular overhang was rotated {:.2} deg", got[1].angle().to_degrees());
    }

    /// Inverted square frustum: half-width `a` on the bed, `b` at height `h`.
    fn inverted_frustum(a: f64, b: f64, h: f64) -> crate::mesh::Mesh {
        use crate::geometry::Triangle;
        let p = |x: f64, y: f64, z: f64| Point3D::new(x, y, z);
        let lo = [p(-a, -a, 0.0), p(a, -a, 0.0), p(a, a, 0.0), p(-a, a, 0.0)];
        let hi = [p(-b, -b, h), p(b, -b, h), p(b, b, h), p(-b, b, h)];
        let mut t = vec![
            Triangle::new(lo[0], lo[2], lo[1]), Triangle::new(lo[0], lo[3], lo[2]),
            Triangle::new(hi[0], hi[1], hi[2]), Triangle::new(hi[0], hi[2], hi[3]),
        ];
        for i in 0..4 {
            let j = (i + 1) % 4;
            t.push(Triangle::new(lo[i], lo[j], hi[j]));
            t.push(Triangle::new(lo[i], hi[j], hi[i]));
        }
        crate::mesh::Mesh::new(t).unwrap()
    }

    /// Largest difference, in degrees, between the rotation each seed on a
    /// flat face of the frustum was given and `expected`, and how many such
    /// seeds there were. Corner tets, where two faces meet, are left out.
    fn seed_error_on_faces(
        mesh: &TetMesh,
        surface: &crate::mesh::Mesh,
        field: &S4RotationField,
        expected: f64,
    ) -> (f64, usize) {
        let truth = surface_normals_per_tet(mesh, surface);
        let (mut worst, mut seeds) = (0.0f64, 0usize);
        for (ti, q) in field.rotations.iter().enumerate() {
            if truth[ti].norm() < 1e-9 {
                continue;
            }
            let n = truth[ti].normalize();
            // Purely one sloping face: not a corner, and not an edge shared
            // with the top or the base, where the summed normal is a blend.
            let face_nz = -10.0 / (164.0f64).sqrt();
            let on_one_face = (n.z - face_nz).abs() < 0.01 && (n.x.abs() < 0.02 || n.y.abs() < 0.02);
            if !on_one_face {
                continue;
            }
            seeds += 1;
            // Signed: the rotation must also be the right way round, which
            // is the way that makes this face overhang less.
            let helps = (q * n).z >= n.z - 1e-9;
            let got = if helps { q.angle().to_degrees() } else { -q.angle().to_degrees() };
            worst = worst.max((got - expected).abs());
        }
        (worst, seeds)
    }

    /// Regression test for D-27. On a grid tet mesh a slope is a staircase of
    /// level steps and vertical risers. Read off that, an overhang's angle is
    /// whatever mix of step and riser a tet happens to hold, and its
    /// direction, for a level step, is no direction at all. Read off the
    /// model, every tet on a face gets that face's rotation.
    #[test]
    fn test_overhangs_are_read_from_the_model_not_the_tet_staircase() {
        // Faces rise 8 over a run of 10: 51.34 degrees past vertical.
        let surface = inverted_frustum(5.0, 15.0, 8.0);
        let mesh = TetMesh::from_surface_mesh_grid(&surface, 16).expect("grid tet mesh");
        let dijkstra = TetDijkstraField::compute(&mesh, 0.8);
        // No diffusion: judge the seeds themselves.
        let config = S4RotationConfig {
            overhang_threshold: 20.0,
            max_rotation_degrees: 40.0,
            multi_axis: true,
            smoothing_iterations: 0,
            ..S4RotationConfig::default()
        };
        let expected = (10.0f64 / (164.0f64).sqrt()).asin().to_degrees() - 20.0;

        let field = S4RotationField::compute_for_surface(&mesh, &dijkstra, &config, Some(&surface));
        let (worst, seeds) = seed_error_on_faces(&mesh, &surface, &field, expected);
        assert!(seeds > 40, "expected seeds on all four faces, got {}", seeds);
        assert!(
            worst < 0.5,
            "seeds on the faces are up to {:.2} deg from the {:.2} their overhang calls for",
            worst, expected
        );

        // Control: the same thing read off the tet mesh's own boundary.
        let staircase = S4RotationField::compute(&mesh, &dijkstra, &config);
        let (worst_staircase, _) = seed_error_on_faces(&mesh, &surface, &staircase, expected);
        assert!(
            worst_staircase > 5.0,
            "staircase seeding was within {:.2} deg, so this test is not exercising the defect",
            worst_staircase
        );
        eprintln!("seed error on faces: from the model {:.2} deg, from the staircase {:.2} deg", worst, worst_staircase);
    }
}
