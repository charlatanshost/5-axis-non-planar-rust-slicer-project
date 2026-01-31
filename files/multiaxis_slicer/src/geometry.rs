use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Serialize};

/// 3D point type
pub type Point3D = Point3<f64>;

/// 3D vector type
pub type Vector3D = Vector3<f64>;

/// Triangle defined by three vertices
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Triangle {
    pub v0: Point3D,
    pub v1: Point3D,
    pub v2: Point3D,
}

impl Triangle {
    pub fn new(v0: Point3D, v1: Point3D, v2: Point3D) -> Self {
        Self { v0, v1, v2 }
    }

    /// Calculate the normal vector of the triangle
    /// Returns a unit normal, or (0, 0, 1) for degenerate triangles
    pub fn normal(&self) -> Vector3D {
        let edge1 = self.v1 - self.v0;
        let edge2 = self.v2 - self.v0;
        let cross = edge1.cross(&edge2);
        let norm = cross.norm();

        // Handle degenerate triangles (zero area or nearly colinear)
        if norm < 1e-10 || !norm.is_finite() {
            // Return a safe default normal (pointing up)
            return Vector3D::new(0.0, 0.0, 1.0);
        }

        cross / norm  // Manual normalization to avoid NaN
    }

    /// Calculate the area of the triangle
    pub fn area(&self) -> f64 {
        let edge1 = self.v1 - self.v0;
        let edge2 = self.v2 - self.v0;
        edge1.cross(&edge2).norm() / 2.0
    }

    /// Get the bounding box (min_z, max_z)
    pub fn z_bounds(&self) -> (f64, f64) {
        let z_min = self.v0.z.min(self.v1.z).min(self.v2.z);
        let z_max = self.v0.z.max(self.v1.z).max(self.v2.z);
        (z_min, z_max)
    }

    /// Intersect triangle with a horizontal plane at height z
    /// Returns None if no intersection, or Some(line segment)
    pub fn intersect_plane(&self, z: f64) -> Option<LineSegment> {
        let (z_min, z_max) = self.z_bounds();
        
        // Early exit if plane doesn't intersect triangle
        if z < z_min || z > z_max {
            return None;
        }

        let mut intersections = Vec::new();

        // Check each edge for intersection
        let edges = [
            (self.v0, self.v1),
            (self.v1, self.v2),
            (self.v2, self.v0),
        ];

        for (p1, p2) in edges {
            if let Some(point) = intersect_edge_with_plane(p1, p2, z) {
                intersections.push(point);
            }
        }

        // Remove duplicate points
        intersections.dedup_by(|a, b| {
            (a.coords - b.coords).norm() < 1e-10
        });

        if intersections.len() == 2 {
            Some(LineSegment::new(intersections[0], intersections[1]))
        } else {
            None
        }
    }
}

/// Line segment in 3D space
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LineSegment {
    pub start: Point3D,
    pub end: Point3D,
}

impl LineSegment {
    pub fn new(start: Point3D, end: Point3D) -> Self {
        Self { start, end }
    }

    pub fn length(&self) -> f64 {
        (self.end - self.start).norm()
    }

    pub fn direction(&self) -> Vector3D {
        (self.end - self.start).normalize()
    }
}

/// Plane defined by a point and normal vector
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Plane {
    pub point: Point3D,
    pub normal: Vector3D,
}

impl Plane {
    pub fn new(point: Point3D, normal: Vector3D) -> Self {
        Self {
            point,
            normal: normal.normalize(),
        }
    }

    /// Create a horizontal plane at height z
    pub fn horizontal(z: f64) -> Self {
        Self {
            point: Point3D::new(0.0, 0.0, z),
            normal: Vector3D::new(0.0, 0.0, 1.0),
        }
    }

    /// Distance from point to plane (signed)
    pub fn signed_distance(&self, point: &Point3D) -> f64 {
        self.normal.dot(&(point - self.point))
    }
}

/// Helper function to intersect an edge with a horizontal plane
fn intersect_edge_with_plane(p1: Point3D, p2: Point3D, z: f64) -> Option<Point3D> {
    let z1 = p1.z;
    let z2 = p2.z;

    // Check if edge crosses the plane
    if (z1 - z).abs() < 1e-10 {
        return Some(p1);
    }
    if (z2 - z).abs() < 1e-10 {
        return Some(p2);
    }

    // Check if both points are on the same side
    if (z1 < z && z2 < z) || (z1 > z && z2 > z) {
        return None;
    }

    // Linear interpolation
    let t = (z - z1) / (z2 - z1);
    Some(p1 + t * (p2 - p1))
}

/// Contour - a closed loop of points
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Contour {
    pub points: Vec<Point3D>,
    pub closed: bool,
}

impl Contour {
    pub fn new(points: Vec<Point3D>, closed: bool) -> Self {
        Self { points, closed }
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Calculate the centroid of the contour
    pub fn centroid(&self) -> Point3D {
        if self.points.is_empty() {
            return Point3D::origin();
        }

        let sum: Vector3D = self.points.iter().map(|p| p.coords).sum();
        Point3D::from(sum / self.points.len() as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangle_plane_intersection() {
        let tri = Triangle::new(
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(0.0, 1.0, 1.0),
        );

        let segment = tri.intersect_plane(0.5).unwrap();
        assert!((segment.start.z - 0.5).abs() < 1e-10);
        assert!((segment.end.z - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_plane_distance() {
        let plane = Plane::horizontal(5.0);
        let point = Point3D::new(1.0, 2.0, 8.0);
        
        assert!((plane.signed_distance(&point) - 3.0).abs() < 1e-10);
    }
}
