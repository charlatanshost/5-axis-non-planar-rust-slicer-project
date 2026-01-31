use crate::geometry::{Contour, Point3D, Vector3D};
use crate::slicing::Layer;
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
}

impl ToolpathGenerator {
    pub fn new(nozzle_diameter: f64, layer_height: f64) -> Self {
        Self {
            nozzle_diameter,
            layer_height,
            extrusion_width: nozzle_diameter * 1.2,
        }
    }

    /// Generate toolpaths from sliced layers
    pub fn generate(&self, layers: &[Layer]) -> Vec<Toolpath> {
        layers
            .iter()
            .map(|layer| self.generate_layer_toolpath(layer))
            .collect()
    }

    fn generate_layer_toolpath(&self, layer: &Layer) -> Toolpath {
        let mut paths = Vec::new();

        for contour in &layer.contours {
            for i in 0..contour.points.len().saturating_sub(1) {
                let p1 = contour.points[i];
                let p2 = contour.points[i + 1];

                let distance = (p2 - p1).norm();
                let extrusion = self.calculate_extrusion(distance);

                paths.push(ToolpathSegment {
                    position: p1,
                    orientation: Vector3D::new(0.0, 0.0, 1.0),
                    extrusion,
                    feedrate: 50.0, // mm/s
                });
            }
        }

        Toolpath {
            paths,
            z: layer.z,
        }
    }

    fn calculate_extrusion(&self, distance: f64) -> f64 {
        // E = (layer_height * extrusion_width * distance) / filament_cross_section
        let filament_diameter = 1.75; // mm
        let filament_area = std::f64::consts::PI * (filament_diameter / 2.0).powi(2);
        let extrusion_volume = self.layer_height * self.extrusion_width * distance;
        extrusion_volume / filament_area
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
        let toolpath = generator.generate_layer_toolpath(&layer);

        assert_eq!(toolpath.paths.len(), 2);
    }
}
