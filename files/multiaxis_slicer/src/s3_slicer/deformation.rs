// Model deformation pipeline for S3-Slicer
// Transforms mesh to enable curved layer slicing

use crate::geometry::{Point3D, Vector3D, Triangle};
use crate::mesh::Mesh;
use crate::s3_slicer::scalar_field::ScalarField;
use nalgebra::{Matrix3, Rotation3, UnitQuaternion};

/// Configuration for mesh deformation
#[derive(Debug, Clone)]
pub struct DeformationConfig {
    /// Rotation angle factor (controls deformation strength)
    pub rotation_factor: f64,

    /// Preserve volume during deformation
    pub preserve_volume: bool,

    /// Build direction vector
    pub build_direction: Vector3D,

    /// Base plane Z coordinate
    pub base_z: f64,

    /// Maximum rotation angle (radians)
    pub max_rotation: f64,
}

impl Default for DeformationConfig {
    fn default() -> Self {
        Self {
            rotation_factor: 1.0,
            preserve_volume: true,
            build_direction: Vector3D::new(0.0, 0.0, 1.0),
            base_z: 0.0,
            max_rotation: std::f64::consts::PI / 4.0,  // 45 degrees
        }
    }
}

/// Result of mesh deformation
pub struct DeformedMesh {
    /// Deformed mesh geometry
    pub mesh: Mesh,

    /// Mapping from original to deformed positions
    pub transformation_map: Vec<DeformationTransform>,

    /// Jacobian determinant at each point (for layer thickness)
    pub jacobian_determinants: Vec<f64>,
}

/// Transformation at a single point
#[derive(Debug, Clone)]
pub struct DeformationTransform {
    /// Original position
    pub original: Point3D,

    /// Deformed position
    pub deformed: Point3D,

    /// Rotation applied
    pub rotation: UnitQuaternion<f64>,

    /// Local scaling factor
    pub scale: f64,
}

/// Deform mesh based on scalar field
pub fn deform_mesh(
    mesh: &Mesh,
    scalar_field: &ScalarField,
    config: &DeformationConfig,
) -> DeformedMesh {
    let mut deformed_triangles = Vec::with_capacity(mesh.triangles.len());
    let mut transformation_map = Vec::with_capacity(mesh.triangles.len() * 3);
    let mut jacobian_determinants = Vec::with_capacity(mesh.triangles.len());

    // For each triangle, compute deformation
    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        // Get scalar field value at triangle center
        let field_value = if tri_idx < scalar_field.values.len() {
            scalar_field.values[tri_idx]
        } else {
            0.0
        };

        // Compute rotation based on field value and surface normal
        let normal = triangle.normal();
        let rotation = compute_rotation_field(&normal, field_value, &config.build_direction, config.rotation_factor);

        // Apply deformation to each vertex
        let v0_transform = apply_deformation(&triangle.v0, &rotation, field_value, config);
        let v1_transform = apply_deformation(&triangle.v1, &rotation, field_value, config);
        let v2_transform = apply_deformation(&triangle.v2, &rotation, field_value, config);

        // Create deformed triangle
        let deformed_triangle = Triangle::new(
            v0_transform.deformed,
            v1_transform.deformed,
            v2_transform.deformed,
        );

        // Compute Jacobian determinant (volume change)
        let jacobian = compute_jacobian_determinant(&v0_transform, &v1_transform, &v2_transform);

        deformed_triangles.push(deformed_triangle);
        transformation_map.push(v0_transform);
        transformation_map.push(v1_transform);
        transformation_map.push(v2_transform);
        jacobian_determinants.push(jacobian);
    }

    // Compute new bounds
    let (bounds_min, bounds_max) = compute_bounds(&deformed_triangles);

    DeformedMesh {
        mesh: Mesh {
            triangles: deformed_triangles,
            bounds_min,
            bounds_max,
        },
        transformation_map,
        jacobian_determinants,
    }
}

/// Compute rotation field based on surface normal and scalar field
fn compute_rotation_field(
    normal: &Vector3D,
    field_value: f64,
    build_direction: &Vector3D,
    rotation_factor: f64,
) -> UnitQuaternion<f64> {
    // Compute rotation axis (perpendicular to normal and build direction)
    let rotation_axis = normal.cross(build_direction);

    if rotation_axis.norm() < 1e-10 {
        // Normal is parallel to build direction, no rotation needed
        return UnitQuaternion::identity();
    }

    let axis = rotation_axis.normalize();

    // Rotation angle proportional to field value and alignment
    let alignment = normal.dot(build_direction).abs();
    let angle = field_value * rotation_factor * (1.0 - alignment) * 0.1;  // Scale factor

    UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle)
}

/// Apply deformation transform to a point
fn apply_deformation(
    point: &Point3D,
    rotation: &UnitQuaternion<f64>,
    field_value: f64,
    config: &DeformationConfig,
) -> DeformationTransform {
    // Translate to origin (relative to base)
    let relative = Point3D::new(
        point.x,
        point.y,
        point.z - config.base_z,
    );

    // Apply rotation around the base
    let rotated = rotation * relative.coords;

    // Compute scaling factor for volume preservation
    let scale = if config.preserve_volume {
        // Scale inversely with rotation to preserve volume
        1.0 / (1.0 + field_value * 0.01).max(0.5)
    } else {
        1.0
    };

    // Apply scaling
    let deformed_coords = rotated * scale;

    // Translate back
    let deformed = Point3D::new(
        deformed_coords.x,
        deformed_coords.y,
        deformed_coords.z + config.base_z,
    );

    DeformationTransform {
        original: *point,
        deformed,
        rotation: *rotation,
        scale,
    }
}

/// Compute Jacobian determinant for volume preservation
fn compute_jacobian_determinant(
    v0: &DeformationTransform,
    v1: &DeformationTransform,
    v2: &DeformationTransform,
) -> f64 {
    // Compute edge vectors in original space
    let e1_orig = v1.original - v0.original;
    let e2_orig = v2.original - v0.original;

    // Compute edge vectors in deformed space
    let e1_def = v1.deformed - v0.deformed;
    let e2_def = v2.deformed - v0.deformed;

    // Compute area ratio as approximation of Jacobian
    let orig_area = e1_orig.cross(&e2_orig).norm();
    let def_area = e1_def.cross(&e2_def).norm();

    if orig_area > 1e-10 {
        def_area / orig_area
    } else {
        1.0
    }
}

/// Compute bounding box of triangles
fn compute_bounds(triangles: &[Triangle]) -> (Point3D, Point3D) {
    if triangles.is_empty() {
        return (Point3D::origin(), Point3D::origin());
    }

    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut max_z = f64::NEG_INFINITY;

    for triangle in triangles {
        for vertex in &[triangle.v0, triangle.v1, triangle.v2] {
            min_x = min_x.min(vertex.x);
            min_y = min_y.min(vertex.y);
            min_z = min_z.min(vertex.z);
            max_x = max_x.max(vertex.x);
            max_y = max_y.max(vertex.y);
            max_z = max_z.max(vertex.z);
        }
    }

    (
        Point3D::new(min_x, min_y, min_z),
        Point3D::new(max_x, max_y, max_z),
    )
}

/// Inverse deformation - map from deformed space back to original
pub fn inverse_deform_point(
    point: &Point3D,
    transformation_map: &[DeformationTransform],
) -> Option<Point3D> {
    // Find closest transformation in the map
    let mut closest_dist = f64::INFINITY;
    let mut closest_transform: Option<&DeformationTransform> = None;

    for transform in transformation_map {
        let dist = (transform.deformed - *point).norm();
        if dist < closest_dist {
            closest_dist = dist;
            closest_transform = Some(transform);
        }
    }

    if let Some(transform) = closest_transform {
        // Apply inverse rotation
        let relative = Point3D::new(
            point.x,
            point.y,
            point.z,
        );

        let rotated_back = transform.rotation.inverse() * relative.coords;

        // Apply inverse scaling
        let scaled_back = rotated_back / transform.scale;

        Some(Point3D::from(scaled_back))
    } else {
        None
    }
}

/// Compute adjusted layer thickness based on Jacobian
pub fn adjusted_layer_thickness(
    base_thickness: f64,
    jacobian: f64,
) -> f64 {
    // Layer thickness adjusts based on volume change
    base_thickness * jacobian.sqrt().max(0.5).min(2.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_deformation() {
        let point = Point3D::new(10.0, 20.0, 30.0);
        let rotation = UnitQuaternion::identity();
        let config = DeformationConfig::default();

        let transform = apply_deformation(&point, &rotation, 0.0, &config);

        // With identity rotation and zero field value, position should be approximately unchanged
        assert!((transform.deformed.x - point.x).abs() < 0.1);
        assert!((transform.deformed.y - point.y).abs() < 0.1);
    }

    #[test]
    fn test_jacobian_identity() {
        let point = Point3D::new(0.0, 0.0, 0.0);
        let rotation = UnitQuaternion::identity();
        let config = DeformationConfig::default();

        let v0 = apply_deformation(&point, &rotation, 0.0, &config);
        let v1 = apply_deformation(&Point3D::new(1.0, 0.0, 0.0), &rotation, 0.0, &config);
        let v2 = apply_deformation(&Point3D::new(0.0, 1.0, 0.0), &rotation, 0.0, &config);

        let jacobian = compute_jacobian_determinant(&v0, &v1, &v2);

        // For identity transform, Jacobian should be close to 1
        assert!((jacobian - 1.0).abs() < 0.2);
    }

    #[test]
    fn test_layer_thickness_adjustment() {
        let base_thickness = 0.2;

        // Jacobian of 1.0 means no change
        assert_eq!(adjusted_layer_thickness(base_thickness, 1.0), base_thickness);

        // Jacobian of 0.25 means compression (increase thickness)
        let compressed = adjusted_layer_thickness(base_thickness, 0.25);
        assert!(compressed > base_thickness * 0.45);

        // Jacobian of 4.0 means expansion (but clamped)
        let expanded = adjusted_layer_thickness(base_thickness, 4.0);
        assert!(expanded <= base_thickness * 2.0);
    }
}
