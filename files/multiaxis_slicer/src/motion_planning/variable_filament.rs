// Variable filament calculation - Step 1 of motion planning
use crate::geometry::{Point3D, Vector3D};

#[derive(Debug, Clone)]
pub struct FilamentCalculator {
    pub filament_diameter: f64,
    pub nozzle_diameter: f64,
}

impl FilamentCalculator {
    pub fn calculate_extrusion(
        &self,
        waypoint: &Waypoint,
        prev_waypoint: Option<&Waypoint>,
        layer_height: f64,
        toolpath_width: f64,
    ) -> f64 {
        // Volume = cross_section_area × distance traveled
        let distance = match prev_waypoint {
            Some(prev) => (waypoint.position - prev.position).norm(),
            None => 0.0,
        };

        let volume = layer_height * toolpath_width * distance;
        let filament_area = std::f64::consts::PI * (self.filament_diameter / 2.0).powi(2);
        volume / filament_area
    }
}

#[derive(Debug, Clone)]
pub struct Waypoint {
    pub position: Point3D,
    pub orientation: Vector3D,
    pub layer_idx: usize,
    pub extrusion: f64,
}
