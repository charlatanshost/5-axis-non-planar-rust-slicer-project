// Collision detection - Step 3
use crate::geometry::{Point3D, Vector3D};

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
    pub fn check_collision_at_waypoint(&self, position: &Point3D, _orientation: &Vector3D) -> CollisionResult {
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

    /// Check if the printhead (modeled as a vertical capsule above the nozzle)
    /// intersects any triangle of `mesh`.
    ///
    /// The capsule extends from `position` (nozzle tip) upward by `body_height`,
    /// with radius `body_radius`. Uses parry3d's Capsule + Triangle shapes with
    /// an AABB pre-filter for performance.
    pub fn check_collision_with_mesh(&self, position: &Point3D, mesh: &crate::mesh::Mesh) -> bool {
        use parry3d::shape::{Capsule, Triangle as ParryTriangle};
        use parry3d::query::intersection_test;
        use nalgebra::{Isometry3, Translation3, Point3};

        let height = self.print_head.body_height as f32;
        let radius = self.print_head.body_radius as f32;

        // Capsule from nozzle tip upward by body_height (local coords: Z axis).
        let a = Point3::new(0.0_f32, 0.0, 0.0);
        let b = Point3::new(0.0_f32, 0.0, height);
        let capsule = Capsule::new(a, b, radius);

        // Translate capsule to world position (nozzle tip).
        let capsule_iso = Isometry3::from_parts(
            Translation3::new(position.x as f32, position.y as f32, position.z as f32),
            nalgebra::UnitQuaternion::identity(),
        );

        // Compute capsule AABB for fast rejection.
        let aabb = capsule.aabb(&capsule_iso);
        let tri_iso = Isometry3::identity();

        for tri in &mesh.triangles {
            let p0 = Point3::new(tri.v0.x as f32, tri.v0.y as f32, tri.v0.z as f32);
            let p1 = Point3::new(tri.v1.x as f32, tri.v1.y as f32, tri.v1.z as f32);
            let p2 = Point3::new(tri.v2.x as f32, tri.v2.y as f32, tri.v2.z as f32);

            // Fast AABB rejection per triangle.
            let tri_min_x = p0.x.min(p1.x).min(p2.x);
            let tri_max_x = p0.x.max(p1.x).max(p2.x);
            if tri_max_x < aabb.mins.x || tri_min_x > aabb.maxs.x { continue; }
            let tri_min_y = p0.y.min(p1.y).min(p2.y);
            let tri_max_y = p0.y.max(p1.y).max(p2.y);
            if tri_max_y < aabb.mins.y || tri_min_y > aabb.maxs.y { continue; }
            let tri_min_z = p0.z.min(p1.z).min(p2.z);
            let tri_max_z = p0.z.max(p1.z).max(p2.z);
            if tri_max_z < aabb.mins.z || tri_min_z > aabb.maxs.z { continue; }

            let triangle = ParryTriangle::new(p0, p1, p2);
            if intersection_test(&capsule_iso, &capsule, &tri_iso, &triangle)
                .unwrap_or(false)
            {
                return true;
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point3D;

    fn make_detector() -> CollisionDetector {
        CollisionDetector {
            print_head: PrintHead {
                nozzle_diameter: 0.4,
                body_radius: 15.0,
                body_height: 50.0,
            },
            platform_bounds: (
                Point3D::new(-150.0, -150.0, 0.0),
                Point3D::new(150.0, 150.0, 300.0),
            ),
            printed_mesh: None,
        }
    }

    #[test]
    fn test_platform_collision() {
        let det = make_detector();
        let result = det.check_collision_at_waypoint(
            &Point3D::new(0.0, 0.0, -5.0), // below bed
            &Vector3D::new(0.0, 0.0, 1.0),
        );
        assert!(!result.is_collision_free);
        assert!(result.collides_with_platform);
    }

    #[test]
    fn test_no_platform_collision() {
        let det = make_detector();
        let result = det.check_collision_at_waypoint(
            &Point3D::new(0.0, 0.0, 10.0),
            &Vector3D::new(0.0, 0.0, 1.0),
        );
        assert!(result.is_collision_free);
    }

    #[test]
    fn test_mesh_collision_hit() {
        use crate::geometry::Triangle;
        use crate::mesh::Mesh;

        // A triangle at z=5, 30x30mm — well within the capsule (radius=15, at z=0)
        let triangles = vec![
            Triangle::new(
                Point3D::new(-5.0, -5.0, 5.0),
                Point3D::new(5.0, -5.0, 5.0),
                Point3D::new(0.0, 5.0, 5.0),
            ),
        ];
        let mesh = Mesh::new(triangles).unwrap();
        let det = make_detector();

        // Nozzle at z=0, capsule extends to z=50 with radius=15 — should hit the triangle at z=5
        let hit = det.check_collision_with_mesh(&Point3D::new(0.0, 0.0, 0.0), &mesh);
        assert!(hit, "Capsule should intersect triangle at z=5");
    }

    #[test]
    fn test_mesh_collision_miss() {
        use crate::geometry::Triangle;
        use crate::mesh::Mesh;

        // Triangle far away from the capsule (x=200)
        let triangles = vec![
            Triangle::new(
                Point3D::new(200.0, 0.0, 5.0),
                Point3D::new(210.0, 0.0, 5.0),
                Point3D::new(205.0, 10.0, 5.0),
            ),
        ];
        let mesh = Mesh::new(triangles).unwrap();
        let det = make_detector();

        let hit = det.check_collision_with_mesh(&Point3D::new(0.0, 0.0, 0.0), &mesh);
        assert!(!hit, "Capsule should not intersect distant triangle");
    }
}
