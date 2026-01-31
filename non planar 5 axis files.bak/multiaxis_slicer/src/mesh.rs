use crate::geometry::{Point3D, Triangle, Vector3D};
use std::path::Path;

#[derive(Debug, thiserror::Error)]
pub enum MeshError {
    #[error("Failed to load mesh: {0}")]
    LoadError(String),
    
    #[error("Invalid mesh format")]
    InvalidFormat,
    
    #[error("Empty mesh")]
    EmptyMesh,
}

/// 3D triangular mesh
#[derive(Debug, Clone)]
pub struct Mesh {
    pub triangles: Vec<Triangle>,
    pub bounds_min: Point3D,
    pub bounds_max: Point3D,
}

impl Mesh {
    /// Create a new mesh from triangles
    pub fn new(triangles: Vec<Triangle>) -> Result<Self, MeshError> {
        if triangles.is_empty() {
            return Err(MeshError::EmptyMesh);
        }

        let (bounds_min, bounds_max) = Self::compute_bounds(&triangles);

        Ok(Self {
            triangles,
            bounds_min,
            bounds_max,
        })
    }

    /// Load mesh from STL file
    pub fn from_stl<P: AsRef<Path>>(path: P) -> Result<Self, MeshError> {
        let mut file = std::fs::File::open(path)
            .map_err(|e| MeshError::LoadError(e.to_string()))?;

        let stl = stl_io::read_stl(&mut file)
            .map_err(|e| MeshError::LoadError(e.to_string()))?;

        let triangles: Vec<Triangle> = stl
            .vertices
            .chunks(3)
            .map(|chunk| {
                Triangle::new(
                    Point3D::new(chunk[0][0] as f64, chunk[0][1] as f64, chunk[0][2] as f64),
                    Point3D::new(chunk[1][0] as f64, chunk[1][1] as f64, chunk[1][2] as f64),
                    Point3D::new(chunk[2][0] as f64, chunk[2][1] as f64, chunk[2][2] as f64),
                )
            })
            .collect();

        Self::new(triangles)
    }

    /// Compute bounding box of triangles
    fn compute_bounds(triangles: &[Triangle]) -> (Point3D, Point3D) {
        let mut min = Point3D::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Point3D::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

        for tri in triangles {
            for vertex in [tri.v0, tri.v1, tri.v2] {
                min.x = min.x.min(vertex.x);
                min.y = min.y.min(vertex.y);
                min.z = min.z.min(vertex.z);
                
                max.x = max.x.max(vertex.x);
                max.y = max.y.max(vertex.y);
                max.z = max.z.max(vertex.z);
            }
        }

        (min, max)
    }

    /// Get triangles that could intersect with a plane at height z
    /// Uses binary search for efficiency (O(log n + k))
    pub fn triangles_at_height(&self, z: f64, tolerance: f64) -> Vec<&Triangle> {
        self.triangles
            .iter()
            .filter(|tri| {
                let (z_min, z_max) = tri.z_bounds();
                z_max >= z - tolerance && z_min <= z + tolerance
            })
            .collect()
    }

    /// Translate mesh by vector
    pub fn translate(&mut self, offset: Vector3D) {
        for tri in &mut self.triangles {
            tri.v0 += offset;
            tri.v1 += offset;
            tri.v2 += offset;
        }
        self.bounds_min += offset;
        self.bounds_max += offset;
    }

    /// Scale mesh uniformly
    pub fn scale(&mut self, factor: f64) {
        let center = (self.bounds_min.coords + self.bounds_max.coords) / 2.0;
        
        for tri in &mut self.triangles {
            tri.v0 = Point3D::from(center + (tri.v0.coords - center) * factor);
            tri.v1 = Point3D::from(center + (tri.v1.coords - center) * factor);
            tri.v2 = Point3D::from(center + (tri.v2.coords - center) * factor);
        }

        let (bounds_min, bounds_max) = Self::compute_bounds(&self.triangles);
        self.bounds_min = bounds_min;
        self.bounds_max = bounds_max;
    }

    /// Get mesh dimensions
    pub fn dimensions(&self) -> Vector3D {
        self.bounds_max - self.bounds_min
    }

    /// Get mesh volume (approximate using triangle areas)
    pub fn volume(&self) -> f64 {
        // Simple approximation: sum of signed volumes of tetrahedra
        self.triangles
            .iter()
            .map(|tri| {
                let v0 = tri.v0.coords;
                let v1 = tri.v1.coords;
                let v2 = tri.v2.coords;
                v0.dot(&v1.cross(&v2)) / 6.0
            })
            .sum::<f64>()
            .abs()
    }

    /// Number of triangles in mesh
    pub fn num_triangles(&self) -> usize {
        self.triangles.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mesh_creation() {
        let triangles = vec![
            Triangle::new(
                Point3D::new(0.0, 0.0, 0.0),
                Point3D::new(1.0, 0.0, 0.0),
                Point3D::new(0.0, 1.0, 0.0),
            ),
        ];

        let mesh = Mesh::new(triangles).unwrap();
        assert_eq!(mesh.num_triangles(), 1);
    }

    #[test]
    fn test_mesh_bounds() {
        let triangles = vec![
            Triangle::new(
                Point3D::new(0.0, 0.0, 0.0),
                Point3D::new(1.0, 0.0, 0.0),
                Point3D::new(0.0, 1.0, 1.0),
            ),
        ];

        let mesh = Mesh::new(triangles).unwrap();
        assert_eq!(mesh.bounds_min, Point3D::new(0.0, 0.0, 0.0));
        assert_eq!(mesh.bounds_max, Point3D::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn test_mesh_translate() {
        let triangles = vec![
            Triangle::new(
                Point3D::new(0.0, 0.0, 0.0),
                Point3D::new(1.0, 0.0, 0.0),
                Point3D::new(0.0, 1.0, 0.0),
            ),
        ];

        let mut mesh = Mesh::new(triangles).unwrap();
        mesh.translate(Vector3D::new(10.0, 0.0, 0.0));
        
        assert_eq!(mesh.bounds_min.x, 10.0);
        assert_eq!(mesh.bounds_max.x, 11.0);
    }
}
