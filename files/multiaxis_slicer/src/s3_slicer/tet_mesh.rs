// Tetrahedral Mesh for Volumetric Deformation
// Provides volumetric mesh structure for improved ASAP deformation

use crate::geometry::{Point3D, Vector3D};
use crate::mesh::Mesh;
use std::collections::HashMap;

/// A single tetrahedron element
#[derive(Debug, Clone, Copy)]
pub struct Tetrahedron {
    /// Four vertex indices
    pub vertices: [usize; 4],
}

impl Tetrahedron {
    /// Create new tetrahedron
    pub fn new(v0: usize, v1: usize, v2: usize, v3: usize) -> Self {
        Self {
            vertices: [v0, v1, v2, v3],
        }
    }

    /// Compute volume of tetrahedron
    pub fn volume(&self, positions: &[Point3D]) -> f64 {
        let v0 = positions[self.vertices[0]];
        let v1 = positions[self.vertices[1]];
        let v2 = positions[self.vertices[2]];
        let v3 = positions[self.vertices[3]];

        // Volume = |det([v1-v0, v2-v0, v3-v0])| / 6
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let e3 = v3 - v0;

        let det = e1.dot(&e2.cross(&e3));
        det.abs() / 6.0
    }

    /// Get edges of tetrahedron
    pub fn edges(&self) -> [(usize, usize); 6] {
        [
            (self.vertices[0], self.vertices[1]),
            (self.vertices[0], self.vertices[2]),
            (self.vertices[0], self.vertices[3]),
            (self.vertices[1], self.vertices[2]),
            (self.vertices[1], self.vertices[3]),
            (self.vertices[2], self.vertices[3]),
        ]
    }

    /// Get faces of tetrahedron (each face is a triangle)
    pub fn faces(&self) -> [[usize; 3]; 4] {
        [
            [self.vertices[0], self.vertices[1], self.vertices[2]], // Face opposite to v3
            [self.vertices[0], self.vertices[1], self.vertices[3]], // Face opposite to v2
            [self.vertices[0], self.vertices[2], self.vertices[3]], // Face opposite to v1
            [self.vertices[1], self.vertices[2], self.vertices[3]], // Face opposite to v0
        ]
    }

    /// Check if tetrahedron is degenerate (zero or negative volume)
    pub fn is_degenerate(&self, positions: &[Point3D]) -> bool {
        self.volume(positions) < 1e-10
    }
}

/// Tetrahedral mesh for volumetric deformation
#[derive(Debug, Clone)]
pub struct TetMesh {
    /// Vertex positions
    pub vertices: Vec<Point3D>,

    /// Tetrahedra
    pub tets: Vec<Tetrahedron>,

    /// Surface triangle indices (for rendering/slicing)
    pub surface_triangles: Vec<[usize; 3]>,

    /// Bounding box
    pub bounds_min: Point3D,
    pub bounds_max: Point3D,
}

impl TetMesh {
    /// Create empty tetrahedral mesh
    pub fn new(vertices: Vec<Point3D>, tets: Vec<Tetrahedron>) -> Self {
        // Compute bounding box
        let bounds_min = vertices.iter().fold(
            Point3D::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            |min, v| Point3D::new(min.x.min(v.x), min.y.min(v.y), min.z.min(v.z)),
        );

        let bounds_max = vertices.iter().fold(
            Point3D::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
            |max, v| Point3D::new(max.x.max(v.x), max.y.max(v.y), max.z.max(v.z)),
        );

        // Extract surface triangles
        let surface_triangles = extract_surface_triangles(&tets);

        Self {
            vertices,
            tets,
            surface_triangles,
            bounds_min,
            bounds_max,
        }
    }

    /// Tetrahedralize a surface mesh (simple interior point method)
    ///
    /// **Note**: This is a simplified implementation. For production use,
    /// consider using TetGen or similar libraries.
    pub fn from_surface_mesh(surface_mesh: &Mesh, num_interior_points: usize) -> Self {
        log::info!("Tetrahedralizing surface mesh...");
        log::info!("  Surface triangles: {}", surface_mesh.triangles.len());
        log::info!("  Target interior points: {}", num_interior_points);

        // Step 1: Extract unique vertices from surface
        let mut vertices = Vec::new();
        let mut vertex_map = HashMap::new();

        for triangle in &surface_mesh.triangles {
            for &vertex in &[triangle.v0, triangle.v1, triangle.v2] {
                let hash = hash_point(&vertex);
                vertex_map.entry(hash).or_insert_with(|| {
                    let idx = vertices.len();
                    vertices.push(vertex);
                    idx
                });
            }
        }

        log::info!("  Surface vertices: {}", vertices.len());

        // Step 2: Add interior points (simple grid-based sampling)
        let initial_vertex_count = vertices.len();
        add_interior_points(
            &mut vertices,
            &surface_mesh.bounds_min,
            &surface_mesh.bounds_max,
            num_interior_points,
        );

        log::info!("  Total vertices (with interior): {}", vertices.len());

        // Step 3: Create tetrahedra (simplified - connect to centroid)
        // For better quality, use Delaunay tetrahedralization
        let tets = create_simple_tetrahedralization(
            &vertices,
            &surface_mesh.triangles,
            &vertex_map,
            initial_vertex_count,
        );

        log::info!("  Tetrahedra created: {}", tets.len());

        Self::new(vertices, tets)
    }

    /// Convert back to surface mesh
    pub fn to_surface_mesh(&self) -> Mesh {
        let triangles = self
            .surface_triangles
            .iter()
            .map(|&[i0, i1, i2]| crate::geometry::Triangle {
                v0: self.vertices[i0],
                v1: self.vertices[i1],
                v2: self.vertices[i2],
            })
            .collect();

        Mesh::new(triangles).unwrap()
    }

    /// Compute total volume
    pub fn total_volume(&self) -> f64 {
        self.tets
            .iter()
            .map(|tet| tet.volume(&self.vertices))
            .sum()
    }

    /// Get vertex positions
    pub fn vertex_positions(&self) -> &[Point3D] {
        &self.vertices
    }

    /// Get vertex positions (mutable)
    pub fn vertex_positions_mut(&mut self) -> &mut [Point3D] {
        &mut self.vertices
    }

    /// Check mesh quality
    pub fn check_quality(&self) -> MeshQuality {
        let mut min_volume = f64::INFINITY;
        let mut max_volume = f64::NEG_INFINITY;
        let mut degenerate_count = 0;

        for tet in &self.tets {
            let vol = tet.volume(&self.vertices);
            if tet.is_degenerate(&self.vertices) {
                degenerate_count += 1;
            }
            min_volume = min_volume.min(vol);
            max_volume = max_volume.max(vol);
        }

        MeshQuality {
            num_vertices: self.vertices.len(),
            num_tets: self.tets.len(),
            num_surface_triangles: self.surface_triangles.len(),
            min_tet_volume: min_volume,
            max_tet_volume: max_volume,
            avg_tet_volume: self.total_volume() / self.tets.len() as f64,
            degenerate_tets: degenerate_count,
        }
    }
}

/// Tetrahedral mesh quality metrics
#[derive(Debug, Clone)]
pub struct MeshQuality {
    pub num_vertices: usize,
    pub num_tets: usize,
    pub num_surface_triangles: usize,
    pub min_tet_volume: f64,
    pub max_tet_volume: f64,
    pub avg_tet_volume: f64,
    pub degenerate_tets: usize,
}

/// Extract surface triangles from tetrahedral mesh
fn extract_surface_triangles(tets: &[Tetrahedron]) -> Vec<[usize; 3]> {
    // Build face adjacency map
    let mut face_count: HashMap<[usize; 3], usize> = HashMap::new();

    for tet in tets {
        for face in tet.faces() {
            // Normalize face orientation (sort indices)
            let mut sorted_face = face;
            sorted_face.sort();

            *face_count.entry(sorted_face).or_insert(0) += 1;
        }
    }

    // Surface faces appear exactly once
    face_count
        .into_iter()
        .filter(|(_, count)| *count == 1)
        .map(|(face, _)| face)
        .collect()
}

/// Add interior points using grid sampling
fn add_interior_points(
    vertices: &mut Vec<Point3D>,
    bounds_min: &Point3D,
    bounds_max: &Point3D,
    num_points: usize,
) {
    // Simple grid-based sampling
    let grid_size = (num_points as f64).cbrt().ceil() as usize;

    let dx = (bounds_max.x - bounds_min.x) / (grid_size + 1) as f64;
    let dy = (bounds_max.y - bounds_min.y) / (grid_size + 1) as f64;
    let dz = (bounds_max.z - bounds_min.z) / (grid_size + 1) as f64;

    for i in 1..=grid_size {
        for j in 1..=grid_size {
            for k in 1..=grid_size {
                let point = Point3D::new(
                    bounds_min.x + i as f64 * dx,
                    bounds_min.y + j as f64 * dy,
                    bounds_min.z + k as f64 * dz,
                );
                vertices.push(point);
            }
        }
    }
}

/// Create simple tetrahedralization by connecting surface to centroid
///
/// **Note**: This creates a star-shaped tetrahedralization, not optimal quality.
/// For production, use proper Delaunay tetrahedralization.
fn create_simple_tetrahedralization(
    vertices: &[Point3D],
    surface_triangles: &[crate::geometry::Triangle],
    vertex_map: &HashMap<u64, usize>,
    surface_vertex_count: usize,
) -> Vec<Tetrahedron> {
    let mut tets = Vec::new();

    // Compute centroid
    let centroid_idx = if surface_vertex_count < vertices.len() {
        // Use first interior point as centroid approximation
        surface_vertex_count
    } else {
        // No interior points, add centroid
        log::warn!("No interior points available for tetrahedralization");
        return tets;
    };

    // Create tetrahedra by connecting each surface triangle to centroid
    for triangle in surface_triangles {
        let v0 = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
        let v1 = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
        let v2 = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

        // Create tet from triangle + centroid
        let tet = Tetrahedron::new(v0, v1, v2, centroid_idx);

        // Check if tet is valid (positive volume)
        if !tet.is_degenerate(vertices) {
            tets.push(tet);
        }
    }

    tets
}

/// Hash a point for deduplication
fn hash_point(p: &Point3D) -> u64 {
    let x = (p.x * 1000000.0).round() as i64;
    let y = (p.y * 1000000.0).round() as i64;
    let z = (p.z * 1000000.0).round() as i64;

    let mut hash = 0u64;
    hash = hash.wrapping_mul(73856093) ^ (x as u64);
    hash = hash.wrapping_mul(19349663) ^ (y as u64);
    hash = hash.wrapping_mul(83492791) ^ (z as u64);
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tetrahedron_volume() {
        // Unit tetrahedron
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(0.0, 1.0, 0.0),
            Point3D::new(0.0, 0.0, 1.0),
        ];

        let tet = Tetrahedron::new(0, 1, 2, 3);
        let volume = tet.volume(&vertices);

        // Volume of unit tetrahedron is 1/6
        assert!((volume - 1.0 / 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_tetrahedron_edges() {
        let tet = Tetrahedron::new(0, 1, 2, 3);
        let edges = tet.edges();

        assert_eq!(edges.len(), 6);
        assert!(edges.contains(&(0, 1)));
        assert!(edges.contains(&(2, 3)));
    }

    #[test]
    fn test_tetrahedron_faces() {
        let tet = Tetrahedron::new(0, 1, 2, 3);
        let faces = tet.faces();

        assert_eq!(faces.len(), 4);
        // Each face should have 3 vertices
        for face in &faces {
            assert_eq!(face.len(), 3);
        }
    }

    #[test]
    fn test_surface_extraction() {
        // Create simple mesh: 2 tets sharing a face
        let tets = vec![
            Tetrahedron::new(0, 1, 2, 3),
            Tetrahedron::new(0, 1, 2, 4), // Shares face [0,1,2]
        ];

        let surface = extract_surface_triangles(&tets);

        // Should have 6 surface faces (3 from each tet, minus 1 shared)
        assert_eq!(surface.len(), 6);
    }
}
