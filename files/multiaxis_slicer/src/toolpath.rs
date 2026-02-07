use crate::geometry::{Point3D, Vector3D};
use crate::slicing::Layer;
use crate::centroidal_axis::CentroidalAxis;
use crate::toolpath_patterns::{ToolpathPattern, ToolpathConfig, generate_spiral_pattern, generate_zigzag_pattern, generate_contour_pattern};
use serde::{Deserialize, Serialize};

/// Toolpath for a single layer
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Toolpath {
    pub paths: Vec<ToolpathSegment>,
    pub z: f64,
}

/// A segment of a toolpath with position and orientation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolpathSegment {
    pub position: Point3D,
    pub orientation: Vector3D, // Tool orientation (for 5-axis)
    pub extrusion: f64,        // Amount to extrude
    pub feedrate: f64,         // Movement speed
}

pub struct ToolpathGenerator {
    pub nozzle_diameter: f64,
    pub layer_height: f64,
    pub extrusion_width: f64,
    pub pattern_config: ToolpathConfig,
}

impl ToolpathGenerator {
    pub fn new(nozzle_diameter: f64, layer_height: f64) -> Self {
        Self {
            nozzle_diameter,
            layer_height,
            extrusion_width: nozzle_diameter * 1.2,
            pattern_config: ToolpathConfig::default(),
        }
    }

    pub fn with_pattern(mut self, pattern: ToolpathPattern) -> Self {
        self.pattern_config.pattern = pattern;
        self
    }

    pub fn with_config(mut self, config: ToolpathConfig) -> Self {
        self.pattern_config = config;
        self
    }

    /// Generate toolpaths from sliced layers with optional centroidal axis
    pub fn generate(&self, layers: &[Layer]) -> Vec<Toolpath> {
        // Calculate centroidal axis for non-planar orientation
        let centroidal_axis = CentroidalAxis::compute(layers, 15.0);

        layers
            .iter()
            .enumerate()
            .map(|(layer_idx, layer)| self.generate_layer_toolpath(layer, layer_idx, &centroidal_axis))
            .collect()
    }

    fn generate_layer_toolpath(&self, layer: &Layer, layer_idx: usize, centroidal_axis: &CentroidalAxis) -> Toolpath {
        let mut paths = Vec::new();

        // Get the slicing direction from centroidal axis
        let slicing_direction = centroidal_axis.slicing_direction_at(layer_idx);

        // Get the centroid for this layer if available
        let layer_centroid = if layer_idx < centroidal_axis.centroids.len() {
            centroidal_axis.centroids[layer_idx]
        } else {
            Point3D::new(0.0, 0.0, layer.z)
        };

        // Generate points based on selected pattern
        let pattern_points = match self.pattern_config.pattern {
            ToolpathPattern::Spiral => generate_spiral_pattern(layer, &self.pattern_config),
            ToolpathPattern::Zigzag => generate_zigzag_pattern(layer, &self.pattern_config),
            ToolpathPattern::Contour => generate_contour_pattern(layer, &self.pattern_config),
        };

        // Convert pattern points to toolpath segments
        for i in 0..pattern_points.len().saturating_sub(1) {
            let p1 = pattern_points[i];
            let p2 = pattern_points[i + 1];

                // Apply non-planar offset to point positions
                // Offset the Z-coordinate based on distance from centroid and slicing direction
                let apply_nonplanar_offset = |p: Point3D| -> Point3D {
                    if slicing_direction.norm() > 0.001 {
                        let slicing_dir_norm = slicing_direction.normalize();

                        // Calculate radial distance from centroid in XY plane
                        let dx = p.x - layer_centroid.x;
                        let dy = p.y - layer_centroid.y;
                        let radial_dist = (dx * dx + dy * dy).sqrt();

                        // Apply Z offset based on slicing direction tilt
                        // The offset depends on the tilt of the slicing direction
                        let z_offset = radial_dist * (slicing_dir_norm.x * dx / (radial_dist + 1e-6)
                                                     + slicing_dir_norm.y * dy / (radial_dist + 1e-6));

                        Point3D::new(p.x, p.y, p.z + z_offset * 0.5) // Scale factor for smoothness
                    } else {
                        p
                    }
                };

            let p1_adjusted = apply_nonplanar_offset(p1);
            let p2_adjusted = apply_nonplanar_offset(p2);

            let distance = (p2_adjusted - p1_adjusted).norm();

            // Improved extrusion calculation based on volume
            // Volume = width * height * distance (cross-section * length)
            let extrusion = self.calculate_extrusion_volume(distance, self.extrusion_width, layer.layer_height);

            // Calculate tool orientation based on surface normal
            // For non-planar printing, we want the tool to be perpendicular to the build direction
            let orientation = if slicing_direction.norm() > 0.001 {
                // Use the slicing direction as the primary tool orientation
                slicing_direction.normalize()
            } else {
                // Fallback to vertical
                Vector3D::new(0.0, 0.0, 1.0)
            };

            paths.push(ToolpathSegment {
                position: p1_adjusted,
                orientation,
                extrusion,
                feedrate: 50.0, // mm/s
            });
        }

        Toolpath {
            paths,
            z: layer.z,
        }
    }

    /// Calculate extrusion based on volume (SIGGRAPH Asia 2022 method)
    /// Takes into account waypoint width, height, and distance
    fn calculate_extrusion_volume(&self, distance: f64, width: f64, height: f64) -> f64 {
        // Volume of material extruded = cross_section_area * distance
        // Cross section is approximately rectangular: width * height
        let extrusion_volume = width * height * distance;

        // Convert volume to filament length
        let filament_diameter = 1.75_f64; // mm (standard)
        let filament_area = std::f64::consts::PI * (filament_diameter / 2.0).powi(2);

        extrusion_volume / filament_area
    }

    /// Legacy method for compatibility
    fn calculate_extrusion(&self, distance: f64) -> f64 {
        self.calculate_extrusion_volume(distance, self.extrusion_width, self.layer_height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Contour;

    #[test]
    fn test_toolpath_generation() {
        let points = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(10.0, 10.0, 0.0),
        ];
        let contour = Contour::new(points, false);
        let layer = Layer::new(0.2, vec![contour], 0.2);

        let generator = ToolpathGenerator::new(0.4, 0.2);
        let centroidal_axis = CentroidalAxis::compute(&[layer.clone()], 45.0);
        let toolpath = generator.generate_layer_toolpath(&layer, 0, &centroidal_axis);

        assert!(!toolpath.paths.is_empty());
    }
}
