// Overhang detection for automatic support generation
// Based on SIGGRAPH Asia 2022: Support Generation for Curved RoboFDM

use crate::geometry::{Point3D, Vector3D, Triangle};
use crate::mesh::Mesh;
use crate::centroidal_axis::CentroidalAxis;
use serde::{Deserialize, Serialize};

/// Configuration for overhang detection
#[derive(Debug, Clone)]
pub struct OverhangConfig {
    /// Maximum angle from vertical before requiring support (degrees)
    /// Typical: 45° for most materials
    pub overhang_angle: f64,

    /// Minimum area of overhang region to generate support (mm²)
    pub min_area_threshold: f64,

    /// Use curved layer normals instead of vertical
    pub use_curved_layers: bool,
}

impl Default for OverhangConfig {
    fn default() -> Self {
        Self {
            overhang_angle: 45.0,
            min_area_threshold: 5.0,  // 5mm² minimum
            use_curved_layers: true,
        }
    }
}

/// Represents a face that needs support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OverhangFace {
    /// Triangle index in original mesh
    pub triangle_idx: usize,

    /// Center point of the overhang face
    pub center: Point3D,

    /// Normal vector of the face
    pub normal: Vector3D,

    /// Angle from build direction (degrees)
    pub overhang_angle: f64,

    /// Severity score (0.0 to 1.0, higher = more severe)
    pub severity: f64,

    /// Layer index where this overhang occurs
    pub layer_idx: usize,
}

/// Result of overhang analysis
#[derive(Debug, Clone)]
pub struct OverhangAnalysis {
    pub overhang_faces: Vec<OverhangFace>,
    pub total_overhang_area: f64,
    pub requires_support: bool,
}

/// Detect overhangs in a mesh that need support
pub fn detect_overhangs(
    mesh: &Mesh,
    config: &OverhangConfig,
    centroidal_axis: Option<&CentroidalAxis>,
) -> OverhangAnalysis {
    let mut overhang_faces = Vec::new();
    let mut total_area = 0.0;

    // Calculate approximate layer for each triangle (for curved layer support)
    let z_min = mesh.bounds_min.z;
    let z_max = mesh.bounds_max.z;
    let layer_height = 0.2; // Default, should come from config
    let num_layers = ((z_max - z_min) / layer_height).ceil() as usize;

    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        // Calculate triangle center and normal
        let center = Point3D::new(
            (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0,
            (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0,
            (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0,
        );

        let normal = triangle.normal();

        // Determine build direction at this location
        let build_direction = if config.use_curved_layers && centroidal_axis.is_some() {
            // Use local build direction from centroidal axis
            let layer_idx = ((center.z - z_min) / layer_height).floor() as usize;
            let layer_idx = layer_idx.min(num_layers.saturating_sub(1));

            centroidal_axis
                .unwrap()
                .slicing_direction_at(layer_idx)
        } else {
            // Use vertical build direction
            Vector3D::new(0.0, 0.0, 1.0)
        };

        // Calculate angle between normal and build direction
        let angle_rad = angle_between_vectors(&normal, &build_direction);
        let angle_deg = angle_rad.to_degrees();

        // Check if this face is an overhang
        // If angle > 90° + overhang_angle, the face is pointing downward and needs support
        let is_overhang = angle_deg > (90.0 + config.overhang_angle);

        if is_overhang {
            // Calculate face area
            let area = triangle.area();

            if area >= config.min_area_threshold {
                // Calculate severity (how much support is needed)
                // More severe = steeper angle
                let severity = ((angle_deg - 90.0 - config.overhang_angle) / config.overhang_angle)
                    .min(1.0)
                    .max(0.0);

                let layer_idx = ((center.z - z_min) / layer_height).floor() as usize;

                overhang_faces.push(OverhangFace {
                    triangle_idx: tri_idx,
                    center,
                    normal,
                    overhang_angle: angle_deg,
                    severity,
                    layer_idx,
                });

                total_area += area;
            }
        }
    }

    // Sort by layer (bottom to top) for tree generation
    overhang_faces.sort_by(|a, b| a.layer_idx.cmp(&b.layer_idx));

    OverhangAnalysis {
        requires_support: !overhang_faces.is_empty(),
        total_overhang_area: total_area,
        overhang_faces,
    }
}

/// Calculate angle between two vectors (in radians)
fn angle_between_vectors(v1: &Vector3D, v2: &Vector3D) -> f64 {
    let v1_norm = v1.normalize();
    let v2_norm = v2.normalize();

    let dot = v1_norm.dot(&v2_norm).clamp(-1.0, 1.0);
    dot.acos()
}

/// Group nearby overhang faces into clusters for support generation
pub fn cluster_overhangs(
    overhang_faces: &[OverhangFace],
    cluster_distance: f64,
) -> Vec<Vec<usize>> {
    if overhang_faces.is_empty() {
        return Vec::new();
    }

    let mut clusters: Vec<Vec<usize>> = Vec::new();
    let mut assigned = vec![false; overhang_faces.len()];

    for i in 0..overhang_faces.len() {
        if assigned[i] {
            continue;
        }

        // Start new cluster
        let mut cluster = vec![i];
        assigned[i] = true;

        // Find nearby faces to add to cluster
        for j in (i + 1)..overhang_faces.len() {
            if assigned[j] {
                continue;
            }

            // Check if face j is close to any face in current cluster
            let is_nearby = cluster.iter().any(|&cluster_idx| {
                let dist = (overhang_faces[j].center - overhang_faces[cluster_idx].center).norm();
                dist < cluster_distance
            });

            if is_nearby {
                cluster.push(j);
                assigned[j] = true;
            }
        }

        clusters.push(cluster);
    }

    clusters
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_calculation() {
        let up = Vector3D::new(0.0, 0.0, 1.0);
        let down = Vector3D::new(0.0, 0.0, -1.0);

        let angle = angle_between_vectors(&up, &down);
        assert!((angle - std::f64::consts::PI).abs() < 0.001);
    }

    #[test]
    fn test_overhang_detection_vertical() {
        // Create a simple triangle pointing down
        let triangle = Triangle::new(
            Point3D::new(0.0, 0.0, 10.0),
            Point3D::new(10.0, 0.0, 10.0),
            Point3D::new(5.0, 5.0, 5.0),
        );

        let normal = triangle.normal();
        let up = Vector3D::new(0.0, 0.0, 1.0);

        let angle_deg = angle_between_vectors(&normal, &up).to_degrees();

        // Should be > 90° (pointing somewhat downward)
        assert!(angle_deg > 90.0);
    }
}
