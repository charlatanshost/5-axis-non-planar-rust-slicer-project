use crate::geometry::{Point3D, Vector3D};
use crate::slicing::Layer;
use crate::centroidal_axis::CentroidalAxis;
use crate::toolpath_patterns::{ToolpathPattern, ToolpathConfig, generate_spiral_pattern, generate_zigzag_pattern, generate_contour_pattern, generate_clipped_infill, resample_contour};
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

        // Check if this layer already has curved Z (e.g. from S4 untransform).
        // If contour points have varying Z within a contour, skip the centroidal axis offset.
        let has_curved_z = layer.contours.iter().any(|c| {
            if c.points.len() < 2 { return false; }
            let z_min = c.points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
            let z_max = c.points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
            (z_max - z_min) > 0.01 // More than 10 microns of Z variation
        });

        // Non-planar offset: only apply for planar layers that need centroidal axis adjustment
        let apply_nonplanar_offset = |p: Point3D| -> Point3D {
            if has_curved_z {
                return p; // Already has correct 3D positions from S4 untransform
            }
            if slicing_direction.norm() > 0.001 {
                let slicing_dir_norm = slicing_direction.normalize();
                let dx = p.x - layer_centroid.x;
                let dy = p.y - layer_centroid.y;
                let radial_dist = (dx * dx + dy * dy).sqrt();
                let z_offset = radial_dist * (slicing_dir_norm.x * dx / (radial_dist + 1e-6)
                                             + slicing_dir_norm.y * dy / (radial_dist + 1e-6));
                Point3D::new(p.x, p.y, p.z + z_offset * 0.5)
            } else {
                p
            }
        };

        let orientation = if !has_curved_z && slicing_direction.norm() > 0.001 {
            slicing_direction.normalize()
        } else {
            Vector3D::new(0.0, 0.0, 1.0)
        };

        // For Contour pattern, generate wall loops + infill
        if matches!(self.pattern_config.pattern, ToolpathPattern::Contour) {
            self.generate_walls_and_infill(layer, &apply_nonplanar_offset, orientation, &mut paths);
        } else {
            // For Spiral/Zigzag patterns, use existing merged-point approach
            let pattern_points = match self.pattern_config.pattern {
                ToolpathPattern::Spiral => generate_spiral_pattern(layer, &self.pattern_config),
                ToolpathPattern::Zigzag => generate_zigzag_pattern(layer, &self.pattern_config),
                ToolpathPattern::Contour => unreachable!(),
            };

            for i in 0..pattern_points.len().saturating_sub(1) {
                let p1 = apply_nonplanar_offset(pattern_points[i]);
                let p2 = apply_nonplanar_offset(pattern_points[i + 1]);
                let distance = (p2 - p1).norm();
                let extrusion = self.calculate_extrusion_volume(distance, self.extrusion_width, layer.layer_height);

                paths.push(ToolpathSegment {
                    position: p1,
                    orientation,
                    extrusion,
                    feedrate: self.pattern_config.print_speed,
                });
            }
        }

        Toolpath {
            paths,
            z: layer.z,
        }
    }

    /// Generate wall loops (perimeters) + infill for a layer.
    /// For each contour: offset inward N times for walls, then fill interior with infill pattern.
    fn generate_walls_and_infill(
        &self,
        layer: &Layer,
        apply_nonplanar_offset: &impl Fn(Point3D) -> Point3D,
        orientation: Vector3D,
        paths: &mut Vec<ToolpathSegment>,
    ) {
        use crate::contour_offset::generate_wall_loops;

        let wall_count = self.pattern_config.wall_count;

        for contour in &layer.contours {
            if contour.points.len() < 3 {
                continue;
            }

            // Generate wall loops (outermost to innermost)
            let wall_loops = generate_wall_loops(contour, wall_count, self.pattern_config.line_width);

            // If no wall loops could be generated (contour too small), just trace the original
            if wall_loops.is_empty() {
                self.extrude_contour(contour, apply_nonplanar_offset, orientation, paths);
                continue;
            }

            // Extrude each wall loop (outer first for surface quality)
            for wall_contour in &wall_loops {
                self.extrude_contour(wall_contour, apply_nonplanar_offset, orientation, paths);
            }

            // Generate infill inside the innermost wall
            if self.pattern_config.infill_density > 0.01 {
                let innermost = wall_loops.last().unwrap();

                let infill_lines = generate_clipped_infill(
                    innermost,
                    &self.pattern_config,
                    layer.z,
                    Some(contour), // Validate against original contour
                );

                for line in &infill_lines {
                    if line.len() < 2 {
                        continue;
                    }

                    // Travel to infill line start
                    if !paths.is_empty() {
                        paths.push(ToolpathSegment {
                            position: apply_nonplanar_offset(line[0]),
                            orientation,
                            extrusion: 0.0,
                            feedrate: self.pattern_config.travel_speed,
                        });
                    }

                    // Extrude along infill line
                    for i in 0..line.len().saturating_sub(1) {
                        let p1 = apply_nonplanar_offset(line[i]);
                        let p2 = apply_nonplanar_offset(line[i + 1]);
                        let distance = (p2 - p1).norm();
                        let extrusion = self.calculate_extrusion_volume(distance, self.extrusion_width, layer.layer_height);

                        paths.push(ToolpathSegment {
                            position: p1,
                            orientation,
                            extrusion,
                            feedrate: self.pattern_config.infill_speed,
                        });
                    }
                }
            }
        }
    }

    /// Extrude along a single contour with travel move to start.
    fn extrude_contour(
        &self,
        contour: &crate::geometry::Contour,
        apply_nonplanar_offset: &impl Fn(Point3D) -> Point3D,
        orientation: Vector3D,
        paths: &mut Vec<ToolpathSegment>,
    ) {
        let contour_points = resample_contour(&contour.points, self.pattern_config.node_distance);
        if contour_points.len() < 2 {
            return;
        }

        // Travel move to contour start
        if !paths.is_empty() {
            paths.push(ToolpathSegment {
                position: apply_nonplanar_offset(contour_points[0]),
                orientation,
                extrusion: 0.0,
                feedrate: self.pattern_config.travel_speed,
            });
        }

        // Extrude along contour
        for i in 0..contour_points.len().saturating_sub(1) {
            let p1 = apply_nonplanar_offset(contour_points[i]);
            let p2 = apply_nonplanar_offset(contour_points[i + 1]);
            let distance = (p2 - p1).norm();
            let extrusion = self.calculate_extrusion_volume(distance, self.extrusion_width, self.layer_height);

            paths.push(ToolpathSegment {
                position: p1,
                orientation,
                extrusion,
                feedrate: self.pattern_config.print_speed,
            });
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
