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
    /// Height above the nozzle tip that is excluded from collision testing, in mm.
    ///
    /// This models the nozzle cone protruding below the heater block, so the
    /// head body is not tested against the layer the nozzle is currently
    /// printing on. Typically 1-3 mm.
    ///
    /// It must not be derived from `body_radius`, as it once was: a 15 mm body
    /// produced a 7.5 mm gap, a number with nothing to do with nozzle geometry.
    pub nozzle_clearance: f64,
}

impl Default for PrintHead {
    fn default() -> Self {
        Self {
            nozzle_diameter: 0.4,
            body_radius: 20.0,
            body_height: 50.0,
            nozzle_clearance: 2.0,
        }
    }
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

    /// Whether a collision mesh has been installed.
    ///
    /// While this is false, the mesh checks are no-ops and only the platform
    /// bounds are enforced -- so callers must not report that the part
    /// geometry was checked.
    pub fn has_collision_mesh(&self) -> bool {
        self.mesh_collider.is_some()
    }

    /// Check if the printhead, held vertically above the nozzle at `position`,
    /// intersects the cached collision mesh.
    ///
    /// Shorthand for [`Self::check_collision_with_mesh_oriented`] with a
    /// straight-up tool axis; see there for the body model.
    pub fn check_collision_with_mesh(&self, position: &Point3D) -> bool {
        self.check_collision_with_mesh_oriented(position, &Vector3D::new(0.0, 0.0, 1.0))
    }

    /// Check if the printhead body (oriented along `orientation` from the nozzle
    /// tip) intersects the cached collision mesh.
    ///
    /// The body is modelled as a **cylinder** spanning exactly
    /// `[nozzle_clearance, nozzle_clearance + body_height]` along the tool axis,
    /// at full `body_radius` throughout. A capsule was wrong for this: being a
    /// swept sphere, its lower hemisphere reached `body_radius - clearance`
    /// *below* the nozzle tip (7.5 mm with the old radius-derived clearance), so
    /// it collided with the layer being printed -- precisely what the clearance
    /// is supposed to prevent. Flat caps make the excluded gap mean what it says
    /// while keeping full lateral coverage over the heater block.
    ///
    /// `orientation` must point from the nozzle tip toward the head base.
    /// Returns `false` when no collision mesh has been set.
    pub fn check_collision_with_mesh_oriented(
        &self,
        position: &Point3D,
        orientation: &Vector3D,
    ) -> bool {
        use parry3d::query::intersection_test;
        use parry3d::shape::Cylinder;
        use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3 as NaVec3};

        let Some(collider) = self.mesh_collider.as_ref() else {
            return false;
        };

        let radius = (self.print_head.body_radius as f32).max(1e-4);
        let height = (self.print_head.body_height as f32).max(1e-4);
        let clearance = (self.print_head.nozzle_clearance as f32).max(0.0);

        let half_height = height * 0.5;
        let cylinder = Cylinder::new(half_height, radius);

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

        // Tool frame: nozzle tip at `position`, +Z along the tool axis.
        let tool_iso = Isometry3::from_parts(
            Translation3::new(position.x as f32, position.y as f32, position.z as f32),
            rot,
        );

        // parry cylinders are Y-aligned and centred on the origin, so place this
        // one along the tool frame's +Z with its lower cap at `clearance`.
        let body_iso = tool_iso
            * Isometry3::from_parts(
                Translation3::new(0.0, 0.0, clearance + half_height),
                UnitQuaternion::from_axis_angle(&NaVec3::x_axis(), std::f32::consts::FRAC_PI_2),
            );

        // One BVH-accelerated query replaces the former per-triangle loop.
        intersection_test(
            &body_iso,
            &cylinder,
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
                nozzle_clearance: 2.0,
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

        // A triangle at z=5 — inside the body's footprint (radius=15, at z=0)
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

        // Nozzle at z=0, body spans z in [2, 52] at radius 15 — the triangle
        // at z=5 is inside it.
        let hit = det.check_collision_with_mesh(&Point3D::new(0.0, 0.0, 0.0));
        assert!(hit, "head body should intersect triangle at z=5");
    }

    #[test]
    fn test_mesh_collision_miss() {
        use crate::geometry::Triangle;
        use crate::mesh::Mesh;

        // Triangle well outside the body's footprint (x=200)
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
        assert!(!hit, "head body should not intersect distant triangle");
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

    /// A small flat triangle centred at `(x, 0, z)`, used to probe exactly
    /// where the head body's collision volume starts and stops.
    fn small_triangle_at(x: f64, z: f64) -> crate::mesh::Mesh {
        use crate::geometry::Triangle;
        crate::mesh::Mesh::new(vec![Triangle::new(
            Point3D::new(x - 0.2, -0.2, z),
            Point3D::new(x + 0.2, -0.2, z),
            Point3D::new(x, 0.2, z),
        )])
        .unwrap()
    }

    fn probe(det: &CollisionDetector) -> bool {
        det.check_collision_with_mesh_oriented(
            &Point3D::new(0.0, 0.0, 0.0),
            &Vector3D::new(0.0, 0.0, 1.0),
        )
    }

    /// Regression test for T-09.
    ///
    /// With body_radius 15, body_height 50 and nozzle_clearance 2, the head
    /// body occupies exactly `z in [2, 52]`. The old capsule was a swept sphere
    /// whose lower hemisphere reached `clearance - radius` = -7.5 mm, so it
    /// collided with the layer the nozzle was printing on -- exactly what the
    /// clearance is documented to prevent. The `z = -5` case fails on that
    /// geometry.
    #[test]
    fn test_body_volume_spans_exactly_the_clearance_gap() {
        let cases = [
            (-5.0, false, "below the nozzle tip: the layer being printed"),
            (1.0, false, "inside the nozzle-cone clearance gap"),
            (3.0, true, "heater block, just above the clearance"),
            (51.0, true, "top of the head body"),
            (53.0, false, "above the head body"),
        ];

        for (z, expected, why) in cases {
            let mut det = make_detector();
            det.set_collision_mesh(&small_triangle_at(0.0, z));
            assert_eq!(
                probe(&det),
                expected,
                "triangle at z={}: {} (expected hit={})",
                z,
                why,
                expected
            );
        }
    }

    /// The cylinder covers the full body radius at every height in its span.
    /// The old capsule's lower hemisphere only reached
    /// `sqrt(15^2 - 4.5^2)` = 14.31 mm at z = 3, so a graze at r = 14.5 mm was
    /// missed there -- the under-detection T-09 was reaching for, though not by
    /// the mechanism it described.
    #[test]
    fn test_full_lateral_coverage_over_the_heater_block() {
        let mut det = make_detector();
        det.set_collision_mesh(&small_triangle_at(14.5, 3.0));
        assert!(
            probe(&det),
            "a heater-block graze at r=14.5 mm, z=3 mm must be detected"
        );
    }

    /// The gap must come from `PrintHead`, not from `body_radius`.
    #[test]
    fn test_nozzle_clearance_is_configurable() {
        let mut det = make_detector();
        det.set_collision_mesh(&small_triangle_at(0.0, 5.0));

        det.print_head.nozzle_clearance = 8.0;
        assert!(
            !probe(&det),
            "z=5 mm must fall inside an 8 mm clearance gap"
        );

        det.print_head.nozzle_clearance = 2.0;
        assert!(
            probe(&det),
            "z=5 mm must be covered when the clearance is only 2 mm"
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
