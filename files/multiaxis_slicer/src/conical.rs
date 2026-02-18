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
    let bed_z = mesh.bounds_min.z;
    let mut layers: Vec<Layer> = planar_layers.into_iter()
        .map(|layer| {
            let contours: Vec<Contour> = layer.contours.into_iter()
                .map(|contour| {
                    let points: Vec<Point3D> = contour.points.into_iter()
                        .map(|p| {
                            let mut p = conical_inverse(p, center_x, center_y, tan_angle, sign);
                            // Clamp to bed level (prevents numerical artifacts below build plate)
                            if p.z < bed_z { p.z = bed_z; }
                            p
                        })
                        .collect();
                    Contour::new(points, contour.closed)
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

    // Sort layers by Z for consistent bottom-up printing
    layers.sort_by(|a, b| a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal));

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
            &mesh, &config, 45.0, 5.0, 5.0, ConicalDirection::Outward,
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
