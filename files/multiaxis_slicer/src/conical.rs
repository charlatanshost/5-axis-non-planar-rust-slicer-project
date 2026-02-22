//! Conical slicing pipeline
//!
//! Implements the RotBot/Transform conical slicing approach:
//! 1. Forward transform: shift vertex Z by ±r*tan(cone_angle)
//! 2. Planar slice the deformed mesh using existing Slicer
//! 3. Back-transform: reverse the Z shift on each contour point
//!
//! No tet mesh, no quaternion field, no Dijkstra — just coordinate transforms.
//! Reference: <https://github.com/RotBotSlicer/Transform>

use crate::geometry::{Point3D, Triangle, Contour};
use crate::mesh::Mesh;
use crate::slicing::{Slicer, SlicingConfig, Layer};

/// Direction of the conical slicing cone
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConicalDirection {
    /// Layers cone outward — outer regions pushed down in deformed space.
    /// Most common for overhangs radiating away from center.
    Outward,
    /// Layers cone inward — outer regions pushed up in deformed space.
    /// For central peaks or spires.
    Inward,
}

/// Execute the complete conical slicing pipeline.
///
/// Steps:
/// 1. Forward-transform mesh vertices: Z += sign * r * tan(angle)
/// 2. Planar slice the deformed mesh (reuses existing `Slicer`)
/// 3. Back-transform each contour point to original space
pub fn execute_conical_pipeline(
    mesh: &Mesh,
    slicing_config: &SlicingConfig,
    cone_angle_degrees: f64,
    center_x: f64,
    center_y: f64,
    direction: ConicalDirection,
    use_artifact_filter: bool,
) -> Vec<Layer> {
    let cone_angle_rad = cone_angle_degrees.to_radians();
    let tan_angle = cone_angle_rad.tan();
    let sign = match direction {
        ConicalDirection::Outward => -1.0,
        ConicalDirection::Inward => 1.0,
    };

    log::info!("=== Conical Slicing Pipeline ===");
    log::info!("  Cone angle: {:.1} deg (tan={:.4})", cone_angle_degrees, tan_angle);
    log::info!("  Center: ({:.2}, {:.2})", center_x, center_y);
    log::info!("  Direction: {:?}", direction);

    // Step 1: Forward transform — create deformed mesh
    log::info!("Step 1/3: Applying conical forward transform...");
    let deformed_triangles: Vec<Triangle> = mesh.triangles.iter()
        .map(|tri| Triangle::new(
            conical_forward(tri.v0, center_x, center_y, tan_angle, sign),
            conical_forward(tri.v1, center_x, center_y, tan_angle, sign),
            conical_forward(tri.v2, center_x, center_y, tan_angle, sign),
        ))
        .collect();

    let deformed_mesh = match Mesh::new(deformed_triangles) {
        Ok(m) => m,
        Err(e) => {
            log::error!("Failed to create deformed mesh: {}", e);
            return Vec::new();
        }
    };

    log::info!("  Original Z range: {:.2} to {:.2}", mesh.bounds_min.z, mesh.bounds_max.z);
    log::info!("  Deformed Z range: {:.2} to {:.2}",
        deformed_mesh.bounds_min.z, deformed_mesh.bounds_max.z);

    // Step 2: Planar slice the deformed mesh
    log::info!("Step 2/3: Planar slicing deformed mesh...");
    let slicer = Slicer::new(slicing_config.clone());
    let planar_layers = match slicer.slice(&deformed_mesh) {
        Ok(layers) => layers,
        Err(e) => {
            log::error!("Planar slicing of deformed mesh failed: {}", e);
            return Vec::new();
        }
    };
    log::info!("  Planar slicer produced {} layers", planar_layers.len());

    // Step 3: Back-transform contour points to original space
    log::info!("Step 3/3: Back-transforming contour points to original space...");
    log::info!("  Artifact filter: {}", if use_artifact_filter { "on" } else { "off (original)" });
    let bed_z = mesh.bounds_min.z;
    let mut layers: Vec<Layer> = planar_layers.into_iter()
        .map(|layer| {
            let contours: Vec<Contour> = layer.contours.into_iter()
                .flat_map(|contour| {
                    let points: Vec<Point3D> = contour.points.into_iter()
                        .map(|p| {
                            let mut p = conical_inverse(p, center_x, center_y, tan_angle, sign);
                            // Clamp to bed level (prevents numerical artifacts below build plate)
                            if p.z < bed_z { p.z = bed_z; }
                            p
                        })
                        .collect();
                    let back_xf = Contour::new(points, contour.closed);
                    if use_artifact_filter {
                        // Split at artifact segments: artifact edges (both pts at bed_z,
                        // XY > 10mm) are removed individually so the legitimate rim arc
                        // survives while pure-artifact triangles vanish.
                        split_at_artifact_segments(back_xf, bed_z)
                    } else {
                        // Original behaviour: pass through unchanged after clamping.
                        vec![back_xf]
                    }
                })
                .collect();

            // Compute average Z of back-transformed points
            let avg_z = {
                let total: f64 = contours.iter()
                    .flat_map(|c| c.points.iter())
                    .map(|p| p.z)
                    .sum();
                let count: usize = contours.iter().map(|c| c.points.len()).sum();
                if count > 0 { total / count as f64 } else { layer.z }
            };

            Layer::new(avg_z, contours, layer.layer_height)
        })
        .collect();

    // Sort layers by their true deformed_z (the D value — the planar slice height in deformed
    // space).  Sorting by avg back-transformed Z gives wrong order for the bottom clamped layers:
    // many layers all have avg_z ≈ bed_z because their inner points are clamped, so the sort
    // becomes arbitrary within that cluster, causing higher deformed_z layers to be printed before
    // lower ones ("printing in the air" with no support below).
    //
    // We recover D for each layer as min(p.z + signed_tan × r) over all contour points.
    // Clamped points give D_computed > D (too high); unclamped give exactly D.
    // The minimum is always the true D.
    let signed_tan = sign * tan_angle;
    layers.sort_by(|a, b| {
        let deformed_z_of = |layer: &Layer| -> f64 {
            layer.contours.iter()
                .flat_map(|c| c.points.iter())
                .map(|p| {
                    let dx = p.x - center_x;
                    let dy = p.y - center_y;
                    let r = (dx * dx + dy * dy).sqrt();
                    p.z + signed_tan * r
                })
                .fold(f64::INFINITY, f64::min)
        };
        let da = deformed_z_of(a);
        let db = deformed_z_of(b);
        da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
    });

    // ── Floating contour filter ────────────────────────────────────────────────
    // In Outward conical mode, features at large radius have low deformed_z (D)
    // even though they are physically high above the bed.  Isolated outer
    // features (tail, side protrusions) appear early in D order with no
    // printed material directly below them.
    //
    // Fix: maintain a 2D bin grid of "max Z printed so far at each XY cell".
    // A contour is supported if EVERY one of its points has grid Z within
    // z_tolerance below the point's real Z.  Unsupported contours go into a
    // deferred pool and are flushed once the grid fills in below them.
    //
    // Using per-XY grid (not a global max) avoids the false-positive where a
    // tall body feature raises the global max and incorrectly "supports" a
    // distant floating contour that has no material below it at its own XY.
    {
        let z_tolerance = (slicing_config.layer_height * 3.0).max(0.4);

        // 64 × 64 grid covering the mesh XY extent.
        // Each cell holds the maximum real Z of any accepted contour point in it.
        // Initialised to bed_z — the build plate always provides support.
        const NX: usize = 64;
        const NY: usize = 64;
        let gx_min = mesh.bounds_min.x;
        let gx_max = mesh.bounds_max.x;
        let gy_min = mesh.bounds_min.y;
        let gy_max = mesh.bounds_max.y;
        let dx = (gx_max - gx_min).max(1e-6) / NX as f64;
        let dy = (gy_max - gy_min).max(1e-6) / NY as f64;
        let mut z_grid = vec![bed_z; NX * NY];

        // Inline helpers (avoids borrow conflicts with z_grid).
        macro_rules! grid_idx {
            ($x:expr, $y:expr) => {{
                let ix = (($x - gx_min) / dx).floor() as usize;
                let iy = (($y - gy_min) / dy).floor() as usize;
                iy.min(NY - 1) * NX + ix.min(NX - 1)
            }};
        }
        macro_rules! grid_supported {
            ($contour:expr) => {
                $contour.points.iter().all(|p| {
                    z_grid[grid_idx!(p.x, p.y)] >= p.z - z_tolerance
                })
            };
        }
        macro_rules! grid_update {
            ($contour:expr) => {
                for p in &$contour.points {
                    let idx = grid_idx!(p.x, p.y);
                    if p.z > z_grid[idx] { z_grid[idx] = p.z; }
                }
            };
        }

        // (min_z_real, layer_nominal_z, layer_height, contour)
        let mut deferred: Vec<(f64, f64, f64, Contour)> = Vec::new();
        let mut ordered: Vec<Layer> = Vec::with_capacity(layers.len());

        for layer in layers {
            let lh  = layer.layer_height;
            let nom = layer.z;

            // Partition this layer's contours into grounded and floating.
            let mut grounded: Vec<Contour> = Vec::new();
            for contour in layer.contours {
                if grid_supported!(contour) {
                    grounded.push(contour);
                } else {
                    let min_z = contour.points.iter().map(|p| p.z)
                        .fold(f64::INFINITY, f64::min);
                    deferred.push((min_z, nom, lh, contour));
                }
            }

            // Accept grounded contours and update the grid.
            for c in &grounded { grid_update!(c); }
            if !grounded.is_empty() {
                ordered.push(Layer::new(nom, grounded, lh));
            }

            // Flush deferred contours that have become reachable.
            let pending = std::mem::take(&mut deferred);
            for (min_z, d_nom, d_lh, contour) in pending {
                if grid_supported!(contour) {
                    grid_update!(contour);
                    let avg_z = contour.points.iter().map(|p| p.z).sum::<f64>()
                        / contour.points.len().max(1) as f64;
                    ordered.push(Layer::new(avg_z, vec![contour], d_lh));
                } else {
                    deferred.push((min_z, d_nom, d_lh, contour));
                }
            }
        }

        // Append remaining deferred contours sorted by min_z_real.
        deferred.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let n_deferred = deferred.len();
        for (min_z, nom, lh, contour) in deferred {
            ordered.push(Layer::new(nom.max(min_z), vec![contour], lh));
        }

        if n_deferred > 0 {
            log::warn!(
                "  Floating contour filter: {} contour(s) had no printable support \
                 and were appended at end — consider enabling support structures.",
                n_deferred
            );
        }

        layers = ordered;
    }
    // ──────────────────────────────────────────────────────────────────────────

    let total_contours: usize = layers.iter().map(|l| l.contours.len()).sum();
    let total_points: usize = layers.iter()
        .flat_map(|l| l.contours.iter())
        .map(|c| c.points.len())
        .sum();

    log::info!("=== Conical Pipeline Complete ===");
    log::info!("  {} layers, {} contours, {} points", layers.len(), total_contours, total_points);

    layers
}

/// Create a conically-deformed copy of the mesh (for preview).
pub fn conical_deform_mesh(
    mesh: &Mesh,
    cone_angle_degrees: f64,
    center_x: f64,
    center_y: f64,
    direction: ConicalDirection,
) -> Option<Mesh> {
    let tan_angle = cone_angle_degrees.to_radians().tan();
    let sign = match direction {
        ConicalDirection::Outward => -1.0,
        ConicalDirection::Inward => 1.0,
    };

    let deformed_triangles: Vec<Triangle> = mesh.triangles.iter()
        .map(|tri| Triangle::new(
            conical_forward(tri.v0, center_x, center_y, tan_angle, sign),
            conical_forward(tri.v1, center_x, center_y, tan_angle, sign),
            conical_forward(tri.v2, center_x, center_y, tan_angle, sign),
        ))
        .collect();

    Mesh::new(deformed_triangles).ok()
}

/// Forward transform: shift Z based on radial distance from center.
///
/// `sign = -1` for outward (pushes outer regions down), `+1` for inward.
#[inline]
fn conical_forward(vertex: Point3D, cx: f64, cy: f64, tan_angle: f64, sign: f64) -> Point3D {
    let dx = vertex.x - cx;
    let dy = vertex.y - cy;
    let r = (dx * dx + dy * dy).sqrt();
    Point3D::new(vertex.x, vertex.y, vertex.z + sign * r * tan_angle)
}

/// Inverse transform: reverse the Z shift.
#[inline]
fn conical_inverse(point: Point3D, cx: f64, cy: f64, tan_angle: f64, sign: f64) -> Point3D {
    let dx = point.x - cx;
    let dy = point.y - cy;
    let r = (dx * dx + dy * dy).sqrt();
    Point3D::new(point.x, point.y, point.z - sign * r * tan_angle)
}

/// Split a back-transformed contour at "artifact" edges.
///
/// An artifact edge has both endpoints clamped to bed level AND spans more than 10 mm in XY.
/// These come from bad STL facets (20-60 mm edges all at z_min) that the conical transform
/// maps to extreme deformed-z values.
///
/// Strategy:
/// * If no artifact edges are found → return the contour unchanged.
/// * Otherwise, traverse the ring starting after the first artifact, collecting runs of
///   good vertices.  Each time an artifact edge is hit, flush the current run as an *open*
///   arc (closed = false) if it has ≥ 3 points, then start a new run.
/// * Pure-artifact contours (artifact triangles) produce runs of only 1-2 points → discarded.
/// * The legitimate bottom rim, which may contain 1-2 artifact edges, becomes 1-2 large
///   open arcs that together cover almost the entire perimeter.
fn split_at_artifact_segments(contour: Contour, bed_z: f64) -> Vec<Contour> {
    const Z_EPS: f64 = 0.001;      // mm above bed_z — "at bed level"
    const MAX_SEG_SQ: f64 = 100.0; // (10 mm)² — artifact threshold
    const MIN_ARC_PTS: usize = 3;  // discard runs shorter than this

    let closed = contour.closed;
    let pts = contour.points;
    let n = pts.len();

    if n < 2 || !closed {
        return vec![Contour::new(pts, closed)];
    }

    // Mark artifact edges.  Edge i connects pts[i] → pts[(i+1)%n].
    let artifact: Vec<bool> = (0..n).map(|i| {
        let j = (i + 1) % n;
        if pts[i].z < bed_z + Z_EPS && pts[j].z < bed_z + Z_EPS {
            let dx = pts[j].x - pts[i].x;
            let dy = pts[j].y - pts[i].y;
            dx * dx + dy * dy > MAX_SEG_SQ
        } else {
            false
        }
    }).collect();

    // No artifacts → return unchanged.
    if !artifact.iter().any(|&a| a) {
        return vec![Contour::new(pts, closed)];
    }

    // Find the first artifact edge so we can start the ring traversal right after it.
    let first_art = artifact.iter().position(|&a| a).unwrap();

    let mut result: Vec<Contour> = Vec::new();
    // Begin with the vertex immediately after the first artifact edge.
    let mut run: Vec<Point3D> = vec![pts[(first_art + 1) % n]];

    for k in 1..n {
        let edge = (first_art + k) % n;
        let next_vert = (first_art + k + 1) % n;

        if artifact[edge] {
            // Flush current run as an open arc, then start fresh.
            if run.len() >= MIN_ARC_PTS {
                result.push(Contour::new(run, false));
            }
            run = vec![pts[next_vert]];
        } else {
            run.push(pts[next_vert]);
        }
    }
    // Flush the final run (ends at vertex `first_art`, just before the starting artifact).
    if run.len() >= MIN_ARC_PTS {
        result.push(Contour::new(run, false));
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_conical_forward_inverse_roundtrip() {
        let p = Point3D::new(10.0, 5.0, 20.0);
        let cx = 0.0;
        let cy = 0.0;
        let tan_angle = 1.0; // 45 degrees
        let sign = -1.0; // outward

        let deformed = conical_forward(p, cx, cy, tan_angle, sign);
        let recovered = conical_inverse(deformed, cx, cy, tan_angle, sign);

        assert!((recovered.x - p.x).abs() < 1e-10);
        assert!((recovered.y - p.y).abs() < 1e-10);
        assert!((recovered.z - p.z).abs() < 1e-10);
    }

    #[test]
    fn test_conical_center_point_unchanged() {
        let p = Point3D::new(5.0, 5.0, 10.0);
        let deformed = conical_forward(p, 5.0, 5.0, 1.0, -1.0);
        assert!((deformed.z - p.z).abs() < 1e-10, "Point at cone center should not change Z");
    }

    #[test]
    fn test_conical_outward_lowers_outer_points() {
        // Outward (sign=-1): points far from center should have lower Z
        let center = Point3D::new(0.0, 0.0, 10.0);
        let outer = Point3D::new(10.0, 0.0, 10.0);
        let tan_angle = 1.0; // 45 deg

        let center_def = conical_forward(center, 0.0, 0.0, tan_angle, -1.0);
        let outer_def = conical_forward(outer, 0.0, 0.0, tan_angle, -1.0);

        assert!((center_def.z - 10.0).abs() < 1e-10, "Center Z unchanged");
        assert!(outer_def.z < center_def.z, "Outer point should be pushed down (outward mode)");
        assert!((outer_def.z - 0.0).abs() < 1e-10, "10mm radius at 45deg → Z drops by 10");
    }

    #[test]
    fn test_conical_inward_raises_outer_points() {
        let outer = Point3D::new(10.0, 0.0, 10.0);
        let tan_angle = 1.0;
        let deformed = conical_forward(outer, 0.0, 0.0, tan_angle, 1.0);
        assert!(deformed.z > 10.0, "Outer point should be pushed up (inward mode)");
        assert!((deformed.z - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_conical_pipeline_produces_layers() {
        // Create a simple box mesh (10x10x10)
        let triangles = vec![
            // Bottom face
            Triangle::new(Point3D::new(0.0, 0.0, 0.0), Point3D::new(10.0, 0.0, 0.0), Point3D::new(10.0, 10.0, 0.0)),
            Triangle::new(Point3D::new(0.0, 0.0, 0.0), Point3D::new(10.0, 10.0, 0.0), Point3D::new(0.0, 10.0, 0.0)),
            // Top face
            Triangle::new(Point3D::new(0.0, 0.0, 10.0), Point3D::new(10.0, 10.0, 10.0), Point3D::new(10.0, 0.0, 10.0)),
            Triangle::new(Point3D::new(0.0, 0.0, 10.0), Point3D::new(0.0, 10.0, 10.0), Point3D::new(10.0, 10.0, 10.0)),
            // Front face
            Triangle::new(Point3D::new(0.0, 0.0, 0.0), Point3D::new(10.0, 0.0, 10.0), Point3D::new(10.0, 0.0, 0.0)),
            Triangle::new(Point3D::new(0.0, 0.0, 0.0), Point3D::new(0.0, 0.0, 10.0), Point3D::new(10.0, 0.0, 10.0)),
            // Back face
            Triangle::new(Point3D::new(0.0, 10.0, 0.0), Point3D::new(10.0, 10.0, 0.0), Point3D::new(10.0, 10.0, 10.0)),
            Triangle::new(Point3D::new(0.0, 10.0, 0.0), Point3D::new(10.0, 10.0, 10.0), Point3D::new(0.0, 10.0, 10.0)),
            // Left face
            Triangle::new(Point3D::new(0.0, 0.0, 0.0), Point3D::new(0.0, 10.0, 0.0), Point3D::new(0.0, 10.0, 10.0)),
            Triangle::new(Point3D::new(0.0, 0.0, 0.0), Point3D::new(0.0, 10.0, 10.0), Point3D::new(0.0, 0.0, 10.0)),
            // Right face
            Triangle::new(Point3D::new(10.0, 0.0, 0.0), Point3D::new(10.0, 0.0, 10.0), Point3D::new(10.0, 10.0, 10.0)),
            Triangle::new(Point3D::new(10.0, 0.0, 0.0), Point3D::new(10.0, 10.0, 10.0), Point3D::new(10.0, 10.0, 0.0)),
        ];

        let mesh = Mesh::new(triangles).unwrap();
        let config = SlicingConfig::default();

        let layers = execute_conical_pipeline(
            &mesh, &config, 45.0, 5.0, 5.0, ConicalDirection::Outward, true,
        );

        assert!(!layers.is_empty(), "Conical pipeline should produce layers");

        // Each layer's contour points should have varying Z (non-planar)
        for layer in &layers {
            for contour in &layer.contours {
                if contour.points.len() >= 2 {
                    let z_min = contour.points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
                    let z_max = contour.points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
                    // Back-transformed contours should have Z variation (non-planar)
                    assert!(z_max - z_min > 0.001,
                        "Layer at z={:.2} should have non-planar Z variation, got range {:.4}",
                        layer.z, z_max - z_min);
                }
            }
        }
    }
}
