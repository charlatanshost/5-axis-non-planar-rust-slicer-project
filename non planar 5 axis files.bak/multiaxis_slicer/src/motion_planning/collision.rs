// Collision detection - Step 3
use crate::geometry::Point3D;
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct CollisionDetector {
    pub print_head: PrintHead,
    pub platform_bounds: (Point3D, Point3D),
    pub printed_mesh: Option<()>,
}

#[derive(Debug, Clone)]
pub struct PrintHead {
    pub nozzle_diameter: f64,
    pub body_radius: f64,
    pub body_height: f64,
}

#[derive(Debug, Clone, Default)]
pub struct CollisionResult {
    pub is_collision_free: bool,
    pub collides_with_platform: bool,
    pub collides_with_model: bool,
    pub collision_point: Option<Point3D>,
}

impl CollisionDetector {
    pub fn check_collision_at_waypoint(&self, position: &Point3D, _orientation: &Vector3<f64>) -> CollisionResult {
        let (min_bounds, max_bounds) = &self.platform_bounds;
        
        let collides_platform = position.z < min_bounds.z
            || position.x < min_bounds.x || position.x > max_bounds.x
            || position.y < min_bounds.y || position.y > max_bounds.y;

        CollisionResult {
            is_collision_free: !collides_platform,
            collides_with_platform: collides_platform,
            collides_with_model: false,
            collision_point: None,
        }
    }
}
