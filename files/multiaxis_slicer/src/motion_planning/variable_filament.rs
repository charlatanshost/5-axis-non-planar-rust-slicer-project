// Variable filament calculation - Step 1 of motion planning
use crate::geometry::{Point3D, Vector3D};

#[derive(Debug, Clone)]
pub struct FilamentCalculator {
    pub filament_diameter: f64,
    pub nozzle_diameter: f64,
    pub use_distance: bool,
    pub use_height: bool,
    pub use_width: bool,
}

impl FilamentCalculator {
    pub fn calculate_extrusion(
        &self,
        waypoint: &Waypoint,
        prev_waypoint: Option<&Waypoint>,
        layer_height: f64,
        toolpath_width: f64,
    ) -> f64 {
        let mut volume = 0.0;

        if self.use_distance && prev_waypoint.is_some() {
            let distance = (waypoint.position - prev_waypoint.unwrap().position).norm();
            volume += layer_height * toolpath_width * distance;
        }

        if self.use_height {
            volume += layer_height * toolpath_width * self.nozzle_diameter;
        }

        if self.use_width {
            volume += toolpath_width * layer_height * self.nozzle_diameter;
        }

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
