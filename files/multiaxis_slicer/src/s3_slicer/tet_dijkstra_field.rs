// Dijkstra path length field on tetrahedral mesh
//
// Computes shortest-path distance from the build plate (base tets) to every
// tetrahedron in the mesh, using centroid-to-centroid Euclidean distances as
// edge weights. Also computes per-tet gradient of the distance field.
//
// This is the S4-Slicer approach: the distance field determines "printing order"
// and the gradient determines which direction each tet should rotate to eliminate
// overhangs.

use crate::geometry::{Point3D, Vector3D};
use crate::s3_slicer::tet_mesh::TetMesh;
use ordered_float::OrderedFloat;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// Dijkstra path length field over a tetrahedral mesh.
///
/// Each tetrahedron gets a distance value representing how far it is from the
/// build plate (through the tet adjacency graph), and a gradient vector pointing
/// in the direction of increasing distance (roughly "upward").
pub struct TetDijkstraField {
    /// Per-tet shortest path distance from build plate
    pub distances: Vec<f64>,

    /// Per-tet gradient of the distance field (unit vector, roughly "upward")
    pub gradients: Vec<Vector3D>,

    /// Indices of base tets (those touching the build plate)
    pub base_tets: Vec<usize>,

    /// Maximum distance value in the field
    pub max_distance: f64,
}

impl TetDijkstraField {
    /// Compute Dijkstra distance field from build plate tets.
    ///
    /// 1. Identify base tets (boundary faces in bottom Z region)
    /// 2. Multi-source Dijkstra from all base tets
    /// 3. Compute per-tet gradient from neighbor distance differences
    pub fn compute(tet_mesh: &TetMesh) -> Self {
        let num_tets = tet_mesh.tets.len();
        log::info!("Computing Dijkstra field on {} tets...", num_tets);

        // Pre-compute centroids
        let centroids: Vec<Point3D> = (0..num_tets)
            .map(|ti| {
                let verts = &tet_mesh.tets[ti].vertices;
                let p0 = &tet_mesh.vertices[verts[0]];
                let p1 = &tet_mesh.vertices[verts[1]];
                let p2 = &tet_mesh.vertices[verts[2]];
                let p3 = &tet_mesh.vertices[verts[3]];
                Point3D::new(
                    (p0.x + p1.x + p2.x + p3.x) / 4.0,
                    (p0.y + p1.y + p2.y + p3.y) / 4.0,
                    (p0.z + p1.z + p2.z + p3.z) / 4.0,
                )
            })
            .collect();

        // Find base tets
        let base_tets = find_base_tets(tet_mesh);
        log::info!("  Found {} base tets (touching build plate)", base_tets.len());

        if base_tets.is_empty() {
            log::warn!("  No base tets found! Using lowest-centroid tet as seed.");
            // Fallback: use the tet with lowest centroid Z
            let mut fallback = Self {
                distances: vec![0.0; num_tets],
                gradients: vec![Vector3D::new(0.0, 0.0, 1.0); num_tets],
                base_tets: Vec::new(),
                max_distance: 0.0,
            };
            if let Some(min_ti) = (0..num_tets).min_by(|&a, &b| {
                centroids[a].z.partial_cmp(&centroids[b].z).unwrap()
            }) {
                fallback.base_tets = vec![min_ti];
                let result = run_dijkstra(tet_mesh, &centroids, &fallback.base_tets);
                fallback.distances = result.0;
                fallback.max_distance = result.1;
                fallback.gradients = compute_gradients(tet_mesh, &centroids, &fallback.distances);
            }
            return fallback;
        }

        // Run multi-source Dijkstra
        let (distances, max_distance) = run_dijkstra(tet_mesh, &centroids, &base_tets);
        log::info!("  Dijkstra complete: max_distance={:.2}", max_distance);

        // Compute gradients
        let gradients = compute_gradients(tet_mesh, &centroids, &distances);

        TetDijkstraField {
            distances,
            gradients,
            base_tets,
            max_distance,
        }
    }

    /// Get the distance for a specific tet
    pub fn distance_at(&self, tet_index: usize) -> f64 {
        self.distances[tet_index]
    }

    /// Get the gradient for a specific tet
    pub fn gradient_at(&self, tet_index: usize) -> Vector3D {
        self.gradients[tet_index]
    }

    /// Get normalized distance (0..1) for a specific tet
    pub fn normalized_distance(&self, tet_index: usize) -> f64 {
        if self.max_distance > 1e-10 {
            self.distances[tet_index] / self.max_distance
        } else {
            0.0
        }
    }
}

/// Identify base tets: those with boundary faces in the bottom Z region.
///
/// A boundary face is one where `tet_neighbors[ti][fi] == None`.
/// We check if the face vertices are in the bottom 5% of the Z range.
fn find_base_tets(tet_mesh: &TetMesh) -> Vec<usize> {
    let z_range = tet_mesh.bounds_max.z - tet_mesh.bounds_min.z;
    let z_threshold = tet_mesh.bounds_min.z + z_range * 0.05;

    let mut base_tets = Vec::new();

    for (ti, tet) in tet_mesh.tets.iter().enumerate() {
        let neighbors = &tet_mesh.tet_neighbors[ti];

        for fi in 0..4 {
            if neighbors[fi].is_some() {
                continue; // Not a boundary face
            }

            // Get the 3 face vertices (face fi is opposite vertex fi)
            let face_verts: Vec<usize> = (0..4).filter(|&vi| vi != fi).map(|vi| tet.vertices[vi]).collect();
            let all_low = face_verts.iter().all(|&vi| tet_mesh.vertices[vi].z <= z_threshold);

            if all_low {
                base_tets.push(ti);
                break; // Don't add same tet twice
            }
        }
    }

    // If too few base tets found, relax threshold
    if base_tets.len() < 2 && tet_mesh.tets.len() > 10 {
        let z_threshold_relaxed = tet_mesh.bounds_min.z + z_range * 0.10;
        base_tets.clear();

        for (ti, tet) in tet_mesh.tets.iter().enumerate() {
            let neighbors = &tet_mesh.tet_neighbors[ti];
            for fi in 0..4 {
                if neighbors[fi].is_some() {
                    continue;
                }
                let face_verts: Vec<usize> = (0..4).filter(|&vi| vi != fi).map(|vi| tet.vertices[vi]).collect();
                let all_low = face_verts.iter().all(|&vi| tet_mesh.vertices[vi].z <= z_threshold_relaxed);
                if all_low {
                    base_tets.push(ti);
                    break;
                }
            }
        }
    }

    base_tets
}

/// Run multi-source Dijkstra from base tets.
/// Returns (per-tet distances, max distance).
fn run_dijkstra(
    tet_mesh: &TetMesh,
    centroids: &[Point3D],
    base_tets: &[usize],
) -> (Vec<f64>, f64) {
    let num_tets = tet_mesh.tets.len();
    let mut distances = vec![f64::INFINITY; num_tets];

    // Min-heap: (distance, tet_index)
    let mut heap: BinaryHeap<Reverse<(OrderedFloat<f64>, usize)>> = BinaryHeap::new();

    // Initialize base tets with distance 0
    for &ti in base_tets {
        distances[ti] = 0.0;
        heap.push(Reverse((OrderedFloat(0.0), ti)));
    }

    while let Some(Reverse((OrderedFloat(dist), ti))) = heap.pop() {
        // Skip stale entries
        if dist > distances[ti] {
            continue;
        }

        // Visit face-adjacent neighbors
        for fi in 0..4 {
            if let Some(ni) = tet_mesh.tet_neighbors[ti][fi] {
                let weight = (centroids[ti] - centroids[ni]).norm();
                let new_dist = dist + weight;

                if new_dist < distances[ni] {
                    distances[ni] = new_dist;
                    heap.push(Reverse((OrderedFloat(new_dist), ni)));
                }
            }
        }
    }

    let max_distance = distances
        .iter()
        .filter(|d| d.is_finite())
        .cloned()
        .fold(0.0f64, f64::max);

    // Replace any remaining infinity with max_distance (isolated tets)
    for d in distances.iter_mut() {
        if !d.is_finite() {
            *d = max_distance;
        }
    }

    (distances, max_distance)
}

/// Compute per-tet gradient of the distance field.
///
/// For each tet, uses a weighted average of distance differences with neighbors:
///   gradient = sum_i( (dist[ni] - dist[ti]) * (centroid[ni] - centroid[ti]) / |centroid[ni] - centroid[ti]|^2 )
/// Then normalized.
fn compute_gradients(
    tet_mesh: &TetMesh,
    centroids: &[Point3D],
    distances: &[f64],
) -> Vec<Vector3D> {
    let num_tets = tet_mesh.tets.len();
    let mut gradients = Vec::with_capacity(num_tets);

    for ti in 0..num_tets {
        let mut grad = Vector3D::zeros();

        for fi in 0..4 {
            if let Some(ni) = tet_mesh.tet_neighbors[ti][fi] {
                let diff = centroids[ni] - centroids[ti];
                let dist_sq = diff.norm_squared();
                if dist_sq > 1e-20 {
                    let dist_diff = distances[ni] - distances[ti];
                    grad += diff * (dist_diff / dist_sq);
                }
            }
        }

        let norm = grad.norm();
        if norm > 1e-10 {
            gradients.push(grad / norm);
        } else {
            // Default to upward for tets with no meaningful gradient
            gradients.push(Vector3D::new(0.0, 0.0, 1.0));
        }
    }

    gradients
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s3_slicer::tet_mesh::{TetMesh, Tetrahedron};

    /// Build a simple 2-tet mesh stacked vertically.
    ///
    /// Tet 0 (bottom): [0,1,2,3] with exposed bottom face [0,1,2] at z=0
    /// Tet 1 (top):    [1,2,3,4] shares face [1,2,3] with tet 0
    ///
    /// The key is that tet 0 has a boundary face where ALL vertices are at z=0.
    fn make_two_tet_stack() -> TetMesh {
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),  // 0 - bottom corner
            Point3D::new(10.0, 0.0, 0.0), // 1 - bottom corner
            Point3D::new(5.0, 10.0, 0.0), // 2 - bottom corner
            Point3D::new(5.0, 5.0, 10.0), // 3 - middle apex
            Point3D::new(5.0, 5.0, 20.0), // 4 - top apex
        ];

        let tets = vec![
            Tetrahedron { vertices: [0, 1, 2, 3] }, // bottom tet
            Tetrahedron { vertices: [1, 2, 3, 4] }, // top tet, shares face [1,2,3]
        ];

        // Use TetMesh::new() which computes adjacency automatically
        TetMesh::new(vertices, tets)
    }

    #[test]
    fn test_find_base_tets() {
        let mesh = make_two_tet_stack();
        let base = find_base_tets(&mesh);
        // Tet 0 has boundary face [0,1,2] at z=0, should be a base tet
        assert!(!base.is_empty(), "Should find at least one base tet");
        assert!(base.contains(&0), "Tet 0 should be a base tet (bottom face at z=0)");
    }

    #[test]
    fn test_dijkstra_distances() {
        let mesh = make_two_tet_stack();
        let field = TetDijkstraField::compute(&mesh);

        // Base tet (tet 0) should have distance 0
        assert_eq!(field.distances[0], 0.0, "Base tet distance should be 0");

        // Top tet (tet 1) should have positive distance
        assert!(
            field.distances[1] > 0.0,
            "Top tet distance should be positive, got {}",
            field.distances[1]
        );

        // Distance should be monotonically increasing from base
        assert!(
            field.distances[1] > field.distances[0],
            "Top tet should be farther than base tet"
        );
    }

    #[test]
    fn test_dijkstra_gradients() {
        let mesh = make_two_tet_stack();
        let field = TetDijkstraField::compute(&mesh);

        // Gradients should generally point upward (positive Z component)
        // since the mesh is stacked vertically
        for (ti, grad) in field.gradients.iter().enumerate() {
            let norm = grad.norm();
            assert!(
                (norm - 1.0).abs() < 1e-6,
                "Gradient for tet {} should be unit vector, norm={}",
                ti, norm
            );
        }

        // The gradient of tet 0 should point upward (toward tet 1)
        assert!(
            field.gradients[0].z > 0.0,
            "Base tet gradient should point upward, got z={}",
            field.gradients[0].z
        );
    }

    #[test]
    fn test_normalized_distance() {
        let mesh = make_two_tet_stack();
        let field = TetDijkstraField::compute(&mesh);

        let d0 = field.normalized_distance(0);
        let d1 = field.normalized_distance(1);

        assert!((d0 - 0.0).abs() < 1e-6, "Base tet normalized distance should be 0");
        assert!((d1 - 1.0).abs() < 1e-6, "Farthest tet normalized distance should be 1");
    }
}
