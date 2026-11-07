// Collision detection - Step 3
use std::sync::Arc;

use crate::geometry::{Point3D, Vector3D};

/// BVH-backed collider built once from a mesh.
///
/// `parry3d::shape::TriMesh` carries an internal Qbvh, so a single
/// `intersection_test` against it replaces what used to be a linear scan over
/// every triangle with scalar AABB rejection. Building it is O(n), so it is
/// built once when the mesh is loaded and shared by every subsequent query.
#[derive(Clone)]
pub struct MeshCollider {
    trimesh: parry3d::shape::TriMesh,
}

impl MeshCollider {
    /// Returns `None` for an empty mesh, which has nothing to collide with.
    pub fn from_mesh(mesh: &crate::mesh::Mesh) -> Option<Self> {
        use nalgebra::Point3;

        if mesh.triangles.is_empty() {
            return None;
        }

        let mut vertices: Vec<Point3<f32>> = Vec::with_capacity(mesh.triangles.len() * 3);
        let mut indices: Vec<[u32; 3]> = Vec::with_capacity(mesh.triangles.len());

        for tri in &mesh.triangles {
            let base = vertices.len() as u32;
            vertices.push(Point3::new(tri.v0.x as f32, tri.v0.y as f32, tri.v0.z as f32));
            vertices.push(Point3::new(tri.v1.x as f32, tri.v1.y as f32, tri.v1.z as f32));
            vertices.push(Point3::new(tri.v2.x as f32, tri.v2.y as f32, tri.v2.z as f32));
            indices.push([base, base + 1, base + 2]);
        }

        Some(Self {
            trimesh: parry3d::shape::TriMesh::new(vertices, indices),
        })
    }

    pub fn triangle_count(&self) -> usize {
        self.trimesh.indices().len()
    }
}

// TriMesh does not implement Debug, and CollisionDetector derives it.
impl std::fmt::Debug for MeshCollider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MeshCollider")
            .field("triangles", &self.triangle_count())
            .finish()
    }
}

#[derive(Debug, Clone)]
pub struct CollisionDetector {
    pub print_head: PrintHead,
    pub platform_bounds: (Point3D, Point3D),
    pub printed_mesh: Option<()>,
    /// BVH-backed model collider. Build it with
    /// [`CollisionDetector::set_collision_mesh`]; while `None`, the mesh checks
    /// report no collision because there is no geometry to hit.
    pub mesh_collider: Option<Arc<MeshCollider>>,
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

    /// Build and cache the BVH-backed collider for `mesh`.
    ///
    /// Call this once when the model is loaded. Every later query reuses the
    /// Qbvh rather than rebuilding or rescanning, which is the whole point:
    /// orientation optimization calls the check up to ~10x per colliding
    /// segment.
    pub fn set_collision_mesh(&mut self, mesh: &crate::mesh::Mesh) {
        self.mesh_collider = MeshCollider::from_mesh(mesh).map(Arc::new);
    }

    /// Check if the printhead (modeled as a vertical capsule above the nozzle)
    /// intersects the cached collision mesh.
    ///
    /// The capsule extends from `position` (nozzle tip) along the tool axis by
    /// `body_height`, with radius `body_radius`.
    pub fn check_collision_with_mesh(&self, position: &Point3D) -> bool {
        self.check_collision_with_mesh_oriented(position, &Vector3D::new(0.0, 0.0, 1.0))
    }

    /// Check if the printhead capsule (oriented along `orientation` from the nozzle tip)
    /// intersects the cached collision mesh.
    ///
    /// A small clearance gap is skipped from the nozzle tip so the capsule does not
    /// self-intersect the layer surface the nozzle is printing on.
    ///
    /// `orientation` must point from the nozzle tip toward the head base.
    /// Returns `false` when no collision mesh has been set.
    pub fn check_collision_with_mesh_oriented(
        &self,
        position: &Point3D,
        orientation: &Vector3D,
    ) -> bool {
        use parry3d::query::intersection_test;
        use parry3d::shape::Capsule;
        use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3 as NaVec3};

        let Some(collider) = self.mesh_collider.as_ref() else {
            return false;
        };

        let height = self.print_head.body_height as f32;
        let radius = self.print_head.body_radius as f32;

        // Skip a small clearance from the tip so we don't count the print surface itself.
        let clearance = (radius * 0.5_f32).max(1.0_f32);

        // Capsule in local space: from clearance to height+clearance along +Z.
        let a = Point3::new(0.0_f32, 0.0, clearance);
        let b = Point3::new(0.0_f32, 0.0, height + clearance);
        let capsule = Capsule::new(a, b, radius);

        // Rotate local +Z to align with orientation.
        let local_z = NaVec3::new(0.0_f32, 0.0, 1.0);
        let target = NaVec3::new(orientation.x as f32, orientation.y as f32, orientation.z as f32);
        let target_norm = target.norm();

        let rot = if !target_norm.is_finite() || target_norm < 1e-6 {
            // Degenerate orientation carries no direction; test the untilted head.
            UnitQuaternion::identity()
        } else {
            let unit_target = target / target_norm;
            UnitQuaternion::rotation_between(&local_z, &unit_target).unwrap_or_else(|| {
                // rotation_between yields None only for an antiparallel pair,
                // where it cannot choose an axis. The tool points straight
                // down, so this is a 180 deg turn about any perpendicular axis.
                // The former unwrap_or(identity) silently left such a head
                // pointing up, testing the wrong half of space entirely.
                UnitQuaternion::from_axis_angle(&NaVec3::x_axis(), std::f32::consts::PI)
            })
        };

        let capsule_iso = Isometry3::from_parts(
            Translation3::new(position.x as f32, position.y as f32, position.z as f32),
            rot,
        );

        // One BVH-accelerated query replaces the former per-triangle loop.
        intersection_test(
            &capsule_iso,
            &capsule,
            &Isometry3::identity(),
            &collider.trimesh,
        )
        .unwrap_or(false)
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
            mesh_collider: None,
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
        let mut det = make_detector();
        det.set_collision_mesh(&mesh);

        // Nozzle at z=0, capsule extends to z=50 with radius=15 — should hit the triangle at z=5
        let hit = det.check_collision_with_mesh(&Point3D::new(0.0, 0.0, 0.0));
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
        let mut det = make_detector();
        det.set_collision_mesh(&mesh);

        let hit = det.check_collision_with_mesh(&Point3D::new(0.0, 0.0, 0.0));
        assert!(!hit, "Capsule should not intersect distant triangle");
    }

    fn triangle_at_z(z: f64) -> crate::mesh::Mesh {
        use crate::geometry::Triangle;
        crate::mesh::Mesh::new(vec![Triangle::new(
            Point3D::new(-5.0, -5.0, z),
            Point3D::new(5.0, -5.0, z),
            Point3D::new(0.0, 5.0, z),
        )])
        .unwrap()
    }

    /// Regression test for the antiparallel branch in T-07.
    ///
    /// `UnitQuaternion::rotation_between` returns `None` when the target is
    /// exactly opposite `+Z`, because no axis is determined. The former
    /// `unwrap_or(identity())` therefore modelled a downward-pointing head as
    /// pointing up, testing the wrong half of space entirely. Both probes below
    /// come out inverted under that behaviour.
    #[test]
    fn test_downward_orientation_tests_downward_space() {
        let down = Vector3D::new(0.0, 0.0, -1.0);
        let nozzle = Point3D::new(0.0, 0.0, 0.0);

        let mut det = make_detector();
        det.set_collision_mesh(&triangle_at_z(-20.0));
        assert!(
            det.check_collision_with_mesh_oriented(&nozzle, &down),
            "a downward head must collide with geometry below the nozzle"
        );

        let mut det = make_detector();
        det.set_collision_mesh(&triangle_at_z(20.0));
        assert!(
            !det.check_collision_with_mesh_oriented(&nozzle, &down),
            "a downward head must not collide with geometry above the nozzle"
        );
    }

    /// A zero-length orientation carries no direction and must not panic or
    /// produce a garbage transform.
    #[test]
    fn test_degenerate_orientation_falls_back_to_vertical() {
        let mut det = make_detector();
        det.set_collision_mesh(&triangle_at_z(5.0));
        let hit = det.check_collision_with_mesh_oriented(
            &Point3D::new(0.0, 0.0, 0.0),
            &Vector3D::new(0.0, 0.0, 0.0),
        );
        assert!(hit, "degenerate orientation should test the untilted head");
    }

    /// With no collision mesh set there is nothing to hit.
    #[test]
    fn test_no_collision_mesh_reports_clear() {
        let det = make_detector();
        assert!(det.mesh_collider.is_none());
        assert!(!det.check_collision_with_mesh(&Point3D::new(0.0, 0.0, 0.0)));
    }

    fn tiled_mesh(extra: Option<crate::geometry::Triangle>) -> crate::mesh::Mesh {
        use crate::geometry::Triangle;

        // 20k triangles tiled 500 mm away from the origin probe.
        let mut triangles = Vec::with_capacity(20_001);
        for i in 0..100 {
            for j in 0..100 {
                let x = 500.0 + i as f64;
                let y = 500.0 + j as f64;
                triangles.push(Triangle::new(
                    Point3D::new(x, y, 0.0),
                    Point3D::new(x + 1.0, y, 0.0),
                    Point3D::new(x, y + 1.0, 0.0),
                ));
                triangles.push(Triangle::new(
                    Point3D::new(x + 1.0, y, 0.0),
                    Point3D::new(x + 1.0, y + 1.0, 0.0),
                    Point3D::new(x, y + 1.0, 0.0),
                ));
            }
        }
        if let Some(t) = extra {
            triangles.push(t);
        }
        crate::mesh::Mesh::new(triangles).unwrap()
    }

    /// The BVH path must give the same answers on a mesh large enough that a
    /// linear scan would dominate runtime, and must find a single nearby
    /// triangle hidden among 20k distant ones.
    #[test]
    fn test_bvh_agrees_on_large_mesh() {
        use crate::geometry::Triangle;

        let mut det = make_detector();

        det.set_collision_mesh(&tiled_mesh(None));
        assert_eq!(
            det.mesh_collider.as_ref().unwrap().triangle_count(),
            20_000
        );
        assert!(
            !det.check_collision_with_mesh(&Point3D::new(0.0, 0.0, 0.0)),
            "probe is 500 mm from every triangle"
        );

        det.set_collision_mesh(&tiled_mesh(Some(Triangle::new(
            Point3D::new(-2.0, -2.0, 5.0),
            Point3D::new(2.0, -2.0, 5.0),
            Point3D::new(0.0, 2.0, 5.0),
        ))));
        assert!(
            det.check_collision_with_mesh(&Point3D::new(0.0, 0.0, 0.0)),
            "the one nearby triangle must still be found among 20k"
        );
    }

    /// Tripwire against regressing to a per-triangle scan. Orientation
    /// optimization issues thousands of these queries, so they must be
    /// sublinear in triangle count. The bound is deliberately loose -- a linear
    /// scan would need ~40M triangle tests here and blow straight through it.
    #[test]
    fn test_collision_queries_are_sublinear() {
        let mut det = make_detector();
        det.set_collision_mesh(&tiled_mesh(None));

        let start = std::time::Instant::now();
        for i in 0..2_000 {
            let x = (i % 50) as f64;
            det.check_collision_with_mesh(&Point3D::new(x, 0.0, 0.0));
        }
        let elapsed = start.elapsed();

        assert!(
            elapsed.as_secs_f64() < 5.0,
            "2000 queries over 20k triangles took {:.2}s; expected well under 5s with a BVH",
            elapsed.as_secs_f64()
        );
    }
}
