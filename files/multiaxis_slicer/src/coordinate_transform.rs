//! Cylindrical and spherical coordinate-transform slicing pipelines.
//!
//! Both follow the same pattern as `conical.rs`:
//! 1. Forward transform: map mesh vertices from Cartesian → cylindrical/spherical coords
//! 2. Planar slice the deformed mesh (constant Z in deformed space = constant r in original)
//! 3. Back-transform: recover original Cartesian coordinates from sliced contour points
//!
//! **Cylindrical**: layers become concentric cylinders around the Z axis.
//! Forward: (x,y,z) → (theta, z_orig, r)   where r = radial distance from axis center
//! Inverse: (theta, z_orig, r) → (cx+r·cos θ, cy+r·sin θ, z_orig)
//!
//! **Spherical**: layers become concentric spherical shells around a center point.
//! Forward: (x,y,z) → (theta, phi, r)  where r = distance from center
//! Inverse: (theta, phi, r) → (cx+r·sin φ·cos θ, cy+r·sin φ·sin θ, cz+r·cos φ)
//!
//! **Branch-cut note**: Both transforms have a discontinuity at theta = ±π (the "back"
//! of the object relative to the positive X axis). Triangles that span this cut will
//! produce a thin artifact line. For typical objects this affects only a small region;
//! rotating the mesh 180° before slicing moves the cut to a mesh-free zone if needed.

use crate::geometry::{Point3D, Triangle, Contour};
use crate::mesh::Mesh;
use crate::slicing::{Slicer, SlicingConfig, Layer};

// ─── Cylindrical slicing ──────────────────────────────────────────────────────

/// Execute the complete cylindrical slicing pipeline.
///
/// Layers are concentric cylinders at increasing radial distances from the
/// given center axis (parallel to Z). The `layer_height` in `slicing_config`
/// controls the radial spacing between shells.
///
/// # Steps
/// 1. Forward-transform: map (x,y,z) → (theta, z_orig, r)
/// 2. Planar slice in (theta, z, r) space — cuts at constant r
/// 3. Back-transform: recover original Cartesian positions
pub fn execute_cylindrical_pipeline(
    mesh: &Mesh,
    slicing_config: &SlicingConfig,
    center_x: f64,
    center_y: f64,
) -> Vec<Layer> {
    log::info!("=== Cylindrical Slicing Pipeline ===");
    log::info!("  Axis center: ({:.2}, {:.2})", center_x, center_y);

    // Step 1: Forward transform
    log::info!("Step 1/3: Applying cylindrical forward transform...");
    // Keep all triangles so the mesh stays watertight (no holes → closed contours at every layer).
    // Artifact contours are filtered after back-transform (contour-level detection, not mesh-level).
    let deformed_triangles: Vec<Triangle> = mesh.triangles.iter()
        .filter_map(|tri| {
            let v0 = cylindrical_forward(tri.v0, center_x, center_y);
            let v1 = cylindrical_forward(tri.v1, center_x, center_y);
            let v2 = cylindrical_forward(tri.v2, center_x, center_y);
            // Skip degenerate triangles near the axis (r → 0)
            if v0.z < 1e-3 && v1.z < 1e-3 && v2.z < 1e-3 {
                return None;
            }
            Some(Triangle::new(v0, v1, v2))
        })
        .collect();

    let deformed_mesh = match Mesh::new(deformed_triangles) {
        Ok(m) => m,
        Err(e) => {
            log::error!("Failed to create cylindrical deformed mesh: {}", e);
            return Vec::new();
        }
    };

    log::info!("  Original r range: 0 to {:.2}mm",
        mesh.triangles.iter().flat_map(|t| [t.v0, t.v1, t.v2])
            .map(|v| ((v.x-center_x).powi(2) + (v.y-center_y).powi(2)).sqrt())
            .fold(0.0f64, f64::max));
    log::info!("  Deformed Z range: {:.2} to {:.2} (= r range)",
        deformed_mesh.bounds_min.z, deformed_mesh.bounds_max.z);

    // Step 2: Planar slice (constant Z in deformed space = constant r)
    log::info!("Step 2/3: Planar slicing deformed mesh...");
    let slicer = Slicer::new(slicing_config.clone());
    let planar_layers = match slicer.slice(&deformed_mesh) {
        Ok(layers) => layers,
        Err(e) => {
            log::error!("Planar slicing failed: {}", e);
            return Vec::new();
        }
    };
    log::info!("  Planar slicer produced {} layers", planar_layers.len());

    // Step 3: Back-transform
    log::info!("Step 3/3: Back-transforming contour points...");
    let z_min = mesh.bounds_min.z;
    let mut layers: Vec<Layer> = planar_layers.into_iter()
        .map(|layer| {
            let contours: Vec<Contour> = layer.contours.into_iter()
                .map(|contour| {
                    let points: Vec<Point3D> = contour.points.into_iter()
                        .map(|p| cylindrical_inverse(p, center_x, center_y))
                        .collect();
                    Contour::new(points, contour.closed)
                })
                .filter(|c| {
                    if !c.closed { return false; }
                    // Discard artifact contours: any segment with both endpoints at z_min
                    // AND XY distance > 10mm (bad STL base-plate triangles, edges 20-60mm).
                    const Z_EPS: f64 = 0.001;
                    const MAX_SEG_SQ: f64 = 100.0; // (10mm)²
                    let pts = &c.points;
                    let n = pts.len();
                    for i in 0..n {
                        let p1 = &pts[i];
                        let p2 = &pts[(i + 1) % n];
                        if p1.z < z_min + Z_EPS && p2.z < z_min + Z_EPS {
                            let dx = p2.x - p1.x;
                            let dy = p2.y - p1.y;
                            if dx * dx + dy * dy > MAX_SEG_SQ { return false; }
                        }
                    }
                    true
                })
                .collect();

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

    layers.sort_by(|a, b| a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal));

    let total_contours: usize = layers.iter().map(|l| l.contours.len()).sum();
    log::info!("=== Cylindrical Pipeline Complete: {} layers, {} contours ===",
        layers.len(), total_contours);

    layers
}

/// Create a cylindrically-deformed preview mesh.
pub fn cylindrical_deform_mesh(
    mesh: &Mesh,
    center_x: f64,
    center_y: f64,
) -> Option<Mesh> {
    let deformed: Vec<Triangle> = mesh.triangles.iter()
        .filter_map(|tri| {
            let v0 = cylindrical_forward(tri.v0, center_x, center_y);
            let v1 = cylindrical_forward(tri.v1, center_x, center_y);
            let v2 = cylindrical_forward(tri.v2, center_x, center_y);
            if v0.z < 1e-3 && v1.z < 1e-3 && v2.z < 1e-3 { return None; }
            Some(Triangle::new(v0, v1, v2))
        })
        .collect();
    Mesh::new(deformed).ok()
}

/// Forward cylindrical transform: (x,y,z) → (theta, z_orig, r).
///
/// The output Z coordinate is the radial distance r, so planar slices at
/// constant Z in the deformed space correspond to cylindrical shells in the
/// original space.
#[inline]
fn cylindrical_forward(v: Point3D, cx: f64, cy: f64) -> Point3D {
    let dx = v.x - cx;
    let dy = v.y - cy;
    let r = (dx * dx + dy * dy).sqrt();
    let theta = dy.atan2(dx); // atan2(y, x) → [-π, π]
    Point3D::new(theta, v.z, r) // x=theta, y=z_orig, z=r (the slicing axis)
}

/// Inverse cylindrical transform: (theta, z_orig, r) → (x, y, z).
#[inline]
fn cylindrical_inverse(p: Point3D, cx: f64, cy: f64) -> Point3D {
    let (theta, z_orig, r) = (p.x, p.y, p.z);
    Point3D::new(
        cx + r * theta.cos(),
        cy + r * theta.sin(),
        z_orig,
    )
}

// ─── Spherical slicing ────────────────────────────────────────────────────────

/// Execute the complete spherical slicing pipeline.
///
/// Layers are concentric spherical shells at increasing distances from `center`.
/// The `layer_height` in `slicing_config` controls the radial shell spacing.
///
/// # Steps
/// 1. Forward-transform: map (x,y,z) → (theta, phi, r)
/// 2. Planar slice — cuts at constant r
/// 3. Back-transform: recover original Cartesian positions
pub fn execute_spherical_pipeline(
    mesh: &Mesh,
    slicing_config: &SlicingConfig,
    center_x: f64,
    center_y: f64,
    center_z: f64,
) -> Vec<Layer> {
    log::info!("=== Spherical Slicing Pipeline ===");
    log::info!("  Center: ({:.2}, {:.2}, {:.2})", center_x, center_y, center_z);

    // Step 1: Forward transform
    log::info!("Step 1/3: Applying spherical forward transform...");
    // Keep all triangles so the mesh stays watertight. Artifact contours are filtered after
    // back-transform (contour-level detection), same approach as conical and cylindrical.
    let deformed_triangles: Vec<Triangle> = mesh.triangles.iter()
        .filter_map(|tri| {
            let v0 = spherical_forward(tri.v0, center_x, center_y, center_z);
            let v1 = spherical_forward(tri.v1, center_x, center_y, center_z);
            let v2 = spherical_forward(tri.v2, center_x, center_y, center_z);
            // Skip triangles with any vertex at the singularity (r ≈ 0)
            if v0.z < 1e-6 || v1.z < 1e-6 || v2.z < 1e-6 {
                return None;
            }
            Some(Triangle::new(v0, v1, v2))
        })
        .collect();

    let deformed_mesh = match Mesh::new(deformed_triangles) {
        Ok(m) => m,
        Err(e) => {
            log::error!("Failed to create spherical deformed mesh: {}", e);
            return Vec::new();
        }
    };

    log::info!("  Deformed Z range: {:.2} to {:.2} (= r range)",
        deformed_mesh.bounds_min.z, deformed_mesh.bounds_max.z);

    // Step 2: Planar slice
    log::info!("Step 2/3: Planar slicing deformed mesh...");
    let slicer = Slicer::new(slicing_config.clone());
    let planar_layers = match slicer.slice(&deformed_mesh) {
        Ok(layers) => layers,
        Err(e) => {
            log::error!("Planar slicing failed: {}", e);
            return Vec::new();
        }
    };
    log::info!("  Planar slicer produced {} layers", planar_layers.len());

    // Step 3: Back-transform
    log::info!("Step 3/3: Back-transforming contour points...");
    let z_min_sph = mesh.bounds_min.z;
    let mut layers: Vec<Layer> = planar_layers.into_iter()
        .map(|layer| {
            let contours: Vec<Contour> = layer.contours.into_iter()
                .map(|contour| {
                    let points: Vec<Point3D> = contour.points.into_iter()
                        .map(|p| spherical_inverse(p, center_x, center_y, center_z))
                        .collect();
                    Contour::new(points, contour.closed)
                })
                .filter(|c| {
                    if !c.closed { return false; }
                    const Z_EPS: f64 = 0.001;
                    const MAX_SEG_SQ: f64 = 100.0; // (10mm)²
                    let pts = &c.points;
                    let n = pts.len();
                    for i in 0..n {
                        let p1 = &pts[i];
                        let p2 = &pts[(i + 1) % n];
                        if p1.z < z_min_sph + Z_EPS && p2.z < z_min_sph + Z_EPS {
                            let dx = p2.x - p1.x;
                            let dy = p2.y - p1.y;
                            if dx * dx + dy * dy > MAX_SEG_SQ { return false; }
                        }
                    }
                    true
                })
                .collect();

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

    layers.sort_by(|a, b| a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal));

    let total_contours: usize = layers.iter().map(|l| l.contours.len()).sum();
    log::info!("=== Spherical Pipeline Complete: {} layers, {} contours ===",
        layers.len(), total_contours);

    layers
}

/// Create a spherically-deformed preview mesh.
pub fn spherical_deform_mesh(
    mesh: &Mesh,
    center_x: f64,
    center_y: f64,
    center_z: f64,
) -> Option<Mesh> {
    let deformed: Vec<Triangle> = mesh.triangles.iter()
        .filter_map(|tri| {
            let v0 = spherical_forward(tri.v0, center_x, center_y, center_z);
            let v1 = spherical_forward(tri.v1, center_x, center_y, center_z);
            let v2 = spherical_forward(tri.v2, center_x, center_y, center_z);
            if v0.z < 1e-6 || v1.z < 1e-6 || v2.z < 1e-6 { return None; }
            Some(Triangle::new(v0, v1, v2))
        })
        .collect();
    Mesh::new(deformed).ok()
}

/// Forward spherical transform: (x,y,z) → (theta, phi, r).
///
/// Output Z = r (radial distance), so planar slices at constant Z give
/// spherical shells in the original space.
/// Output X = theta ∈ [-π, π], Output Y = phi ∈ [0, π].
#[inline]
fn spherical_forward(v: Point3D, cx: f64, cy: f64, cz: f64) -> Point3D {
    let dx = v.x - cx;
    let dy = v.y - cy;
    let dz = v.z - cz;
    let r = (dx * dx + dy * dy + dz * dz).sqrt();
    if r < 1e-10 {
        return Point3D::new(0.0, 0.0, 0.0);
    }
    let theta = dy.atan2(dx);
    let phi = (dz / r).clamp(-1.0, 1.0).acos(); // clamp for floating-point safety
    Point3D::new(theta, phi, r) // x=theta, y=phi, z=r (slicing axis)
}

/// Inverse spherical transform: (theta, phi, r) → (x, y, z).
#[inline]
fn spherical_inverse(p: Point3D, cx: f64, cy: f64, cz: f64) -> Point3D {
    let (theta, phi, r) = (p.x, p.y, p.z);
    Point3D::new(
        cx + r * phi.sin() * theta.cos(),
        cy + r * phi.sin() * theta.sin(),
        cz + r * phi.cos(),
    )
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cylindrical_roundtrip() {
        let p = Point3D::new(10.0, 5.0, 20.0);
        let (cx, cy) = (0.0, 0.0);
        let fwd = cylindrical_forward(p, cx, cy);
        let back = cylindrical_inverse(fwd, cx, cy);
        assert!((back.x - p.x).abs() < 1e-10, "x roundtrip");
        assert!((back.y - p.y).abs() < 1e-10, "y roundtrip");
        assert!((back.z - p.z).abs() < 1e-10, "z roundtrip");
    }

    #[test]
    fn test_cylindrical_r_is_z_axis() {
        // The Z coordinate of the deformed point should equal the radial distance
        let p = Point3D::new(3.0, 4.0, 7.0);
        let fwd = cylindrical_forward(p, 0.0, 0.0);
        let r = 5.0; // sqrt(3²+4²)
        assert!((fwd.z - r).abs() < 1e-10, "z_def should equal radial r");
    }

    #[test]
    fn test_cylindrical_center_point() {
        // Point on the axis should map to r=0 and return near-origin
        let p = Point3D::new(0.0, 0.0, 15.0);
        let fwd = cylindrical_forward(p, 0.0, 0.0);
        assert!(fwd.z < 1e-9, "r should be 0 for axis point");
        // y_def should be the original z
        assert!((fwd.y - 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_spherical_roundtrip() {
        let p = Point3D::new(3.0, 4.0, 5.0);
        let (cx, cy, cz) = (0.0, 0.0, 0.0);
        let fwd = spherical_forward(p, cx, cy, cz);
        let back = spherical_inverse(fwd, cx, cy, cz);
        assert!((back.x - p.x).abs() < 1e-9, "x roundtrip");
        assert!((back.y - p.y).abs() < 1e-9, "y roundtrip");
        assert!((back.z - p.z).abs() < 1e-9, "z roundtrip");
    }

    #[test]
    fn test_spherical_r_is_z_axis() {
        let p = Point3D::new(3.0, 0.0, 4.0);
        let fwd = spherical_forward(p, 0.0, 0.0, 0.0);
        let r = 5.0; // sqrt(3²+4²)
        assert!((fwd.z - r).abs() < 1e-10, "z_def should equal r");
    }

    #[test]
    fn test_spherical_center_offset() {
        // With a non-zero center, relative distance should be preserved
        let p = Point3D::new(13.0, 0.0, 4.0);
        let (cx, cy, cz) = (10.0, 0.0, 0.0);
        let fwd = spherical_forward(p, cx, cy, cz);
        // dx=3, dy=0, dz=4 → r=5
        assert!((fwd.z - 5.0).abs() < 1e-10, "r from offset center");
    }

    #[test]
    fn test_cylindrical_pipeline_produces_layers() {
        // Triangles must span a wide range of r (radial distance) so that
        // the cylindrical transform produces z_def = r variation across many
        // slicing planes. Here r goes from ~1 to ~10.
        let triangles = vec![
            // Faces connecting r≈1 inner ring to r≈10 outer ring
            Triangle::new(Point3D::new(1.0, 0.5, 0.0), Point3D::new(10.0, 0.0, 5.0), Point3D::new(10.0, 1.0, 5.0)),
            Triangle::new(Point3D::new(1.0, 0.5, 0.0), Point3D::new(1.5, 0.0, 0.0), Point3D::new(10.0, 0.0, 5.0)),
            Triangle::new(Point3D::new(1.5, 0.0, 0.0), Point3D::new(10.0, 1.0, 5.0), Point3D::new(10.0, 0.0, 5.0)),
            Triangle::new(Point3D::new(1.0, 0.5, 0.0), Point3D::new(10.0, 1.0, 5.0), Point3D::new(1.5, 0.0, 0.0)),
        ];
        let mesh = Mesh::new(triangles).unwrap();
        let config = SlicingConfig::default();
        let layers = execute_cylindrical_pipeline(&mesh, &config, 0.0, 0.0);
        // With r varying from ~1 to ~10 and layer_height=0.2, expect many layers
        assert!(!layers.is_empty(), "cylindrical pipeline should produce layers (r spans ~1..10 → deformed z varies widely)");
    }

    #[test]
    fn test_spherical_pipeline_produces_layers() {
        // Vertices at very different r values from origin so that r (= z_def) varies widely.
        // r values: (2,0,0)→2, (0,8,0)→8, (0,0,6)→6, (5,5,0)→7.07
        let triangles = vec![
            Triangle::new(Point3D::new(2.0, 0.0, 0.0), Point3D::new(0.0, 8.0, 0.0), Point3D::new(0.0, 0.0, 6.0)),
            Triangle::new(Point3D::new(2.0, 0.0, 0.0), Point3D::new(0.0, 0.0, 6.0), Point3D::new(5.0, 5.0, 0.0)),
            Triangle::new(Point3D::new(0.0, 8.0, 0.0), Point3D::new(5.0, 5.0, 0.0), Point3D::new(0.0, 0.0, 6.0)),
            Triangle::new(Point3D::new(2.0, 0.0, 0.0), Point3D::new(5.0, 5.0, 0.0), Point3D::new(0.0, 8.0, 0.0)),
        ];
        let mesh = Mesh::new(triangles).unwrap();
        let config = SlicingConfig::default();
        let layers = execute_spherical_pipeline(&mesh, &config, 0.0, 0.0, 0.0);
        // With r varying from 2 to ~8 and layer_height=0.2, expect many layers
        assert!(!layers.is_empty(), "spherical pipeline should produce layers (r spans 2..8 → deformed z varies widely)");
    }
}
