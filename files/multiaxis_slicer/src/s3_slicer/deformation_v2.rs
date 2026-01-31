// Proper S3-Slicer deformation pipeline
// Step 1: Deform mesh using quaternion field
// Step 2: Compute scalar field on deformed mesh
// Step 3: Map scalar field back to original mesh (inverse deformation)

use crate::geometry::{Point3D, Vector3D, Triangle};
use crate::mesh::Mesh;
use crate::s3_slicer::quaternion_field::QuaternionField;
use rayon::prelude::*;
use std::collections::HashMap;
use nalgebra::UnitQuaternion;

// Import heat method for geodesic distance computation
use crate::s3_slicer::heat_method;

/// Complete S3-Slicer virtual deformation system
/// Note: This computes a VIRTUAL deformation (scalar field) without physically rotating the mesh
pub struct S3SlicerDeformation {
    /// Original mesh (NEVER modified - this is what gets sliced)
    pub original_mesh: Mesh,

    /// Deformed mesh for visualization ONLY (not used in slicing)
    pub deformed_mesh_preview: Mesh,

    /// Quaternion field used for virtual deformation
    pub quaternion_field: QuaternionField,

    /// Scalar field: virtual height values for each triangle
    /// These represent what the height WOULD be if the mesh were physically rotated
    pub scalar_field: Vec<f64>,
}

impl S3SlicerDeformation {
    /// Create deformation system from mesh and quaternion field
    /// This computes VIRTUAL deformation - a scalar field representing virtual heights
    pub fn new(mesh: Mesh, quaternion_field: QuaternionField) -> Self {
        log::info!("Computing virtual deformation (S3-Slicer)...");

        // Compute virtual height field (scalar field) WITHOUT physically deforming the mesh
        let scalar_field = compute_virtual_height_field(&mesh, &quaternion_field);

        // Create a preview mesh for visualization only (not used in slicing)
        let deformed_mesh_preview = create_deformed_preview(&mesh, &quaternion_field);

        log::info!("Virtual deformation complete:");
        log::info!("  Scalar field size: {} triangles", scalar_field.len());
        log::info!("  Height range: {:.2} to {:.2}",
            scalar_field.iter().copied().fold(f64::INFINITY, f64::min),
            scalar_field.iter().copied().fold(f64::NEG_INFINITY, f64::max));

        Self {
            original_mesh: mesh,
            deformed_mesh_preview,
            quaternion_field,
            scalar_field,
        }
    }

    /// Get the scalar field value for a triangle
    pub fn get_scalar_value(&self, triangle_index: usize) -> Option<f64> {
        self.scalar_field.get(triangle_index).copied()
    }

    /// Get the complete scalar field (virtual heights for all triangles)
    pub fn get_scalar_field(&self) -> &[f64] {
        &self.scalar_field
    }

    /// Interpolate scalar value at a point on the mesh using barycentric coordinates
    pub fn interpolate_scalar_at_point(&self, point: &Point3D) -> Option<f64> {
        // Find closest triangle in ORIGINAL mesh
        let (closest_tri_idx, barycentric) = find_closest_triangle(
            point,
            &self.original_mesh,
        )?;

        // Get scalar value for this triangle
        let scalar_value = self.scalar_field.get(closest_tri_idx)?;

        // For now, just return the triangle's scalar value
        // Could interpolate with neighboring triangles if needed
        Some(*scalar_value)
    }

    /// Get the deformed mesh PREVIEW for visualization only
    /// WARNING: This should NOT be used for slicing! Use get_original_mesh() for slicing.
    pub fn get_deformed_mesh(&self) -> &Mesh {
        &self.deformed_mesh_preview
    }

    /// Get the original mesh (this is what should be sliced using the scalar field)
    pub fn get_original_mesh(&self) -> &Mesh {
        &self.original_mesh
    }
}

/// Compute scalar field for slicing using Heat Method for geodesic distances
/// This is the KEY improvement - using proper geodesic distances instead of simple Z-heights
fn compute_virtual_height_field(
    mesh: &Mesh,
    _quaternion_field: &QuaternionField,
) -> Vec<f64> {
    log::info!("Computing scalar field for S3-Slicer:");
    log::info!("  Using Heat Method for geodesic distances");

    // Find bottom vertices as sources (lowest Z values)
    let min_z = mesh.triangles.iter()
        .flat_map(|t| [t.v0.z, t.v1.z, t.v2.z])
        .fold(f64::INFINITY, f64::min);

    // Build vertex list to find source indices
    let mut vertex_list = Vec::new();
    let mut vertex_map = HashMap::new();

    for triangle in &mesh.triangles {
        for &vertex in &[triangle.v0, triangle.v1, triangle.v2] {
            let hash = hash_point(&vertex);
            vertex_map.entry(hash).or_insert_with(|| {
                let idx = vertex_list.len();
                vertex_list.push(vertex);
                idx
            });
        }
    }

    // Find vertices near the bottom (within 1mm of minimum Z)
    let mut sources = Vec::new();
    for (idx, vertex) in vertex_list.iter().enumerate() {
        if (vertex.z - min_z).abs() < 1.0 {
            sources.push(idx);
        }
    }

    log::info!("  Found {} source vertices at bottom (Z={:.2})", sources.len(), min_z);

    // Configure and run heat method
    use crate::s3_slicer::heat_method::{compute_geodesic_distances, HeatMethodConfig};

    let config = HeatMethodConfig {
        time_step: 0.0, // Auto-compute from mean edge length
        sources,
        use_implicit: true,
        smoothing_iterations: 3,
    };

    let result = compute_geodesic_distances(mesh, config);

    log::info!("  Geodesic distance range: {:.2} to {:.2} mm",
        result.min_distance,
        result.max_distance);

    // Return per-triangle scalar values
    result.triangle_scalars
}

/// Create a properly deformed mesh using scale-controlled deformation
/// This implements the S3-Slicer deformation algorithm with Laplacian smoothing
fn create_deformed_preview(
    mesh: &Mesh,
    quaternion_field: &QuaternionField,
) -> Mesh {
    log::info!("Computing scale-controlled deformation...");

    // Configuration for deformation
    const SMOOTHING_ITERATIONS: usize = 5;
    const SMOOTHING_WEIGHT: f64 = 0.3;
    const DEFORMATION_SCALE: f64 = 1.0;

    // Step 1: Build vertex-to-triangles connectivity
    let vertex_map = build_vertex_triangle_map(mesh);
    log::info!("  Built vertex map: {} unique vertices", vertex_map.len());

    // Step 2: Compute target positions for each vertex based on quaternion field
    let target_positions = compute_vertex_target_positions(mesh, quaternion_field, &vertex_map);

    // Step 3: Apply iterative Laplacian smoothing to get smooth deformation
    let deformed_positions = apply_laplacian_deformation(
        mesh,
        &vertex_map,
        &target_positions,
        SMOOTHING_ITERATIONS,
        SMOOTHING_WEIGHT,
        DEFORMATION_SCALE,
    );

    // Step 4: Build deformed mesh from new vertex positions
    let deformed_mesh = build_deformed_mesh(mesh, &vertex_map, &deformed_positions);

    log::info!("  Scale-controlled deformation complete");
    log::info!("  Deformed bounds: ({:.2}, {:.2}, {:.2}) to ({:.2}, {:.2}, {:.2})",
        deformed_mesh.bounds_min.x, deformed_mesh.bounds_min.y, deformed_mesh.bounds_min.z,
        deformed_mesh.bounds_max.x, deformed_mesh.bounds_max.y, deformed_mesh.bounds_max.z);

    deformed_mesh
}

// ===== Scale-Controlled Deformation Helper Functions =====

/// Build map from vertex hash to list of triangle indices
fn build_vertex_triangle_map(mesh: &Mesh) -> HashMap<u64, Vec<usize>> {
    let mut vertex_map: HashMap<u64, Vec<usize>> = HashMap::new();

    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        let v0_hash = hash_point(&triangle.v0);
        let v1_hash = hash_point(&triangle.v1);
        let v2_hash = hash_point(&triangle.v2);

        vertex_map.entry(v0_hash).or_insert_with(Vec::new).push(tri_idx);
        vertex_map.entry(v1_hash).or_insert_with(Vec::new).push(tri_idx);
        vertex_map.entry(v2_hash).or_insert_with(Vec::new).push(tri_idx);
    }

    vertex_map
}

/// Compute target positions for vertices based on quaternion field
/// Each vertex's target is computed from the average rotation of its adjacent triangles
fn compute_vertex_target_positions(
    mesh: &Mesh,
    quaternion_field: &QuaternionField,
    vertex_map: &HashMap<u64, Vec<usize>>,
) -> HashMap<u64, Point3D> {
    let mesh_center = Point3D::new(
        (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0,
        (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0,
        (mesh.bounds_min.z + mesh.bounds_max.z) / 2.0,
    );

    vertex_map
        .par_iter()
        .map(|(vertex_hash, triangle_indices)| {
            // Get original vertex position
            let original_pos = get_vertex_from_triangle(mesh, triangle_indices[0], *vertex_hash)
                .unwrap_or(mesh_center);

            // Compute average quaternion from all adjacent triangles
            let quaternions: Vec<UnitQuaternion<f64>> = triangle_indices
                .iter()
                .filter_map(|&tri_idx| quaternion_field.rotation_at(tri_idx))
                .copied()
                .filter(|q| {
                    // Safety: filter out any quaternions with NaN components
                    q.coords.x.is_finite() && q.coords.y.is_finite() && q.coords.z.is_finite() && q.coords.w.is_finite()
                })
                .collect();

            let avg_quaternion = if quaternions.is_empty() {
                UnitQuaternion::identity()
            } else {
                average_quaternions(&quaternions)
            };

            // Apply rotation around mesh center to get target position
            let relative = original_pos - mesh_center;
            let rotated = avg_quaternion * relative;
            let target_pos = Point3D::from(rotated) + mesh_center.coords;

            // Safety check: if target position has NaN, use original position
            let safe_target = if target_pos.x.is_finite() && target_pos.y.is_finite() && target_pos.z.is_finite() {
                target_pos
            } else {
                log::warn!("  WARNING: NaN target position for vertex, using original position");
                original_pos
            };

            (*vertex_hash, safe_target)
        })
        .collect()
}

/// Apply Laplacian smoothing to create smooth deformation
/// This prevents mesh folding and maintains mesh quality
fn apply_laplacian_deformation(
    mesh: &Mesh,
    vertex_map: &HashMap<u64, Vec<usize>>,
    target_positions: &HashMap<u64, Point3D>,
    iterations: usize,
    smoothing_weight: f64,
    deformation_scale: f64,
) -> HashMap<u64, Point3D> {
    // Start with target positions
    let mut current_positions = target_positions.clone();

    // Build vertex neighborhood (vertices connected by edges)
    let vertex_neighbors = build_vertex_neighbors(mesh, vertex_map);

    log::info!("  Applying Laplacian smoothing: {} iterations", iterations);

    for iteration in 0..iterations {
        let new_positions: HashMap<u64, Point3D> = current_positions
            .par_iter()
            .map(|(vertex_hash, current_pos)| {
                // Get original position
                let original_pos = vertex_map
                    .get(vertex_hash)
                    .and_then(|tris| get_vertex_from_triangle(mesh, tris[0], *vertex_hash))
                    .unwrap_or(*current_pos);

                // Get target position
                let target_pos = target_positions.get(vertex_hash).copied().unwrap_or(*current_pos);

                // Compute Laplacian (average of neighbor positions)
                let neighbors = vertex_neighbors.get(vertex_hash);
                let laplacian = if let Some(neighbor_hashes) = neighbors {
                    if neighbor_hashes.is_empty() {
                        *current_pos
                    } else {
                        let neighbor_sum: Vector3D = neighbor_hashes
                            .iter()
                            .filter_map(|&nh| current_positions.get(&nh))
                            .map(|p| p.coords)
                            .sum();
                        Point3D::from(neighbor_sum / neighbor_hashes.len() as f64)
                    }
                } else {
                    *current_pos
                };

                // Blend between target, current, and Laplacian smoothed position
                let deformation_dir = target_pos - original_pos;
                let laplacian_offset = laplacian - *current_pos;
                let smoothed = Point3D::from(
                    original_pos.coords
                        + deformation_dir * deformation_scale
                        + laplacian_offset * smoothing_weight,
                );

                (*vertex_hash, smoothed)
            })
            .collect();

        current_positions = new_positions;

        if iteration % 2 == 0 {
            log::debug!("    Iteration {}/{}", iteration + 1, iterations);
        }
    }

    current_positions
}

/// Build vertex neighborhood graph (vertices connected by edges)
fn build_vertex_neighbors(
    mesh: &Mesh,
    vertex_map: &HashMap<u64, Vec<usize>>,
) -> HashMap<u64, Vec<u64>> {
    let mut neighbors: HashMap<u64, Vec<u64>> = HashMap::new();

    for triangle in &mesh.triangles {
        let v0 = hash_point(&triangle.v0);
        let v1 = hash_point(&triangle.v1);
        let v2 = hash_point(&triangle.v2);

        // Each vertex is connected to the other two in the triangle
        neighbors.entry(v0).or_insert_with(Vec::new).extend(&[v1, v2]);
        neighbors.entry(v1).or_insert_with(Vec::new).extend(&[v0, v2]);
        neighbors.entry(v2).or_insert_with(Vec::new).extend(&[v0, v1]);
    }

    // Remove duplicates and return
    for neighbor_list in neighbors.values_mut() {
        neighbor_list.sort_unstable();
        neighbor_list.dedup();
    }

    neighbors
}

/// Build deformed mesh from new vertex positions
fn build_deformed_mesh(
    original_mesh: &Mesh,
    vertex_map: &HashMap<u64, Vec<usize>>,
    deformed_positions: &HashMap<u64, Point3D>,
) -> Mesh {
    // Build deformed triangles
    let deformed_triangles: Vec<Triangle> = original_mesh
        .triangles
        .par_iter()
        .map(|triangle| {
            let v0_hash = hash_point(&triangle.v0);
            let v1_hash = hash_point(&triangle.v1);
            let v2_hash = hash_point(&triangle.v2);

            let v0_new = deformed_positions.get(&v0_hash).copied().unwrap_or(triangle.v0);
            let v1_new = deformed_positions.get(&v1_hash).copied().unwrap_or(triangle.v1);
            let v2_new = deformed_positions.get(&v2_hash).copied().unwrap_or(triangle.v2);

            Triangle::new(v0_new, v1_new, v2_new)
        })
        .collect();

    // Compute bounds
    let (bounds_min, bounds_max) = compute_bounds(&deformed_triangles);

    Mesh {
        triangles: deformed_triangles,
        bounds_min,
        bounds_max,
    }
}

/// Average multiple quaternions using spherical linear interpolation
fn average_quaternions(quaternions: &[UnitQuaternion<f64>]) -> UnitQuaternion<f64> {
    if quaternions.is_empty() {
        return UnitQuaternion::identity();
    }

    if quaternions.len() == 1 {
        return quaternions[0];
    }

    // Use iterative slerp averaging
    let mut result = quaternions[0];
    for i in 1..quaternions.len() {
        let weight = 1.0 / (i + 1) as f64;
        result = result.slerp(&quaternions[i], weight);
    }

    result
}

/// Get vertex position from a triangle by hash
fn get_vertex_from_triangle(mesh: &Mesh, tri_idx: usize, vertex_hash: u64) -> Option<Point3D> {
    let triangle = mesh.triangles.get(tri_idx)?;

    if hash_point(&triangle.v0) == vertex_hash {
        Some(triangle.v0)
    } else if hash_point(&triangle.v1) == vertex_hash {
        Some(triangle.v1)
    } else if hash_point(&triangle.v2) == vertex_hash {
        Some(triangle.v2)
    } else {
        None
    }
}

/// Find closest triangle and barycentric coordinates
fn find_closest_triangle(
    point: &Point3D,
    mesh: &Mesh,
) -> Option<(usize, (f64, f64, f64))> {
    let mut closest_idx = 0;
    let mut closest_dist = f64::INFINITY;
    let mut closest_bary = (1.0, 0.0, 0.0);

    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        // Compute closest point on triangle
        let bary = compute_barycentric_coords(triangle, point);

        // Clamp to triangle
        let (u, v, w) = clamp_barycentric(bary);

        // Interpolate point on triangle
        let projected = Point3D::new(
            u * triangle.v0.x + v * triangle.v1.x + w * triangle.v2.x,
            u * triangle.v0.y + v * triangle.v1.y + w * triangle.v2.y,
            u * triangle.v0.z + v * triangle.v1.z + w * triangle.v2.z,
        );

        let dist = (*point - projected).norm();

        if dist < closest_dist {
            closest_dist = dist;
            closest_idx = tri_idx;
            closest_bary = (u, v, w);
        }
    }

    Some((closest_idx, closest_bary))
}

/// Compute barycentric coordinates
fn compute_barycentric_coords(triangle: &Triangle, point: &Point3D) -> (f64, f64, f64) {
    let v0 = triangle.v0;
    let v1 = triangle.v1;
    let v2 = triangle.v2;

    let v0v1 = v1 - v0;
    let v0v2 = v2 - v0;
    let v0p = *point - v0;

    let d00 = v0v1.dot(&v0v1);
    let d01 = v0v1.dot(&v0v2);
    let d11 = v0v2.dot(&v0v2);
    let d20 = v0p.dot(&v0v1);
    let d21 = v0p.dot(&v0v2);

    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < 1e-10 {
        return (1.0, 0.0, 0.0);
    }

    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;

    (u, v, w)
}

/// Clamp barycentric coordinates to triangle
fn clamp_barycentric(bary: (f64, f64, f64)) -> (f64, f64, f64) {
    let (mut u, mut v, mut w) = bary;

    u = u.max(0.0);
    v = v.max(0.0);
    w = w.max(0.0);

    let sum = u + v + w;
    if sum > 1e-10 {
        u /= sum;
        v /= sum;
        w /= sum;
    }

    (u, v, w)
}

/// Hash point for vertex identification
fn hash_point(point: &Point3D) -> u64 {
    let scale = 10000.0; // Higher precision for vertex matching
    let x = (point.x * scale).round() as i64;
    let y = (point.y * scale).round() as i64;
    let z = (point.z * scale).round() as i64;
    ((x as u64) << 42) ^ ((y as u64) << 21) ^ (z as u64)
}

/// Compute mesh bounds
fn compute_bounds(triangles: &[Triangle]) -> (Point3D, Point3D) {
    if triangles.is_empty() {
        return (Point3D::origin(), Point3D::origin());
    }

    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut max_z = f64::NEG_INFINITY;

    for triangle in triangles {
        for vertex in &[triangle.v0, triangle.v1, triangle.v2] {
            min_x = min_x.min(vertex.x);
            min_y = min_y.min(vertex.y);
            min_z = min_z.min(vertex.z);
            max_x = max_x.max(vertex.x);
            max_y = max_y.max(vertex.y);
            max_z = max_z.max(vertex.z);
        }
    }

    (
        Point3D::new(min_x, min_y, min_z),
        Point3D::new(max_x, max_y, max_z),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s3_slicer::quaternion_field::{QuaternionFieldConfig, FabricationObjective};

    #[test]
    fn test_deformation_system() {
        let triangle = Triangle::new(
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(5.0, 10.0, 0.0),
        );

        let mesh = Mesh {
            triangles: vec![triangle],
            bounds_min: Point3D::new(0.0, 0.0, 0.0),
            bounds_max: Point3D::new(10.0, 10.0, 0.0),
        };

        let config = QuaternionFieldConfig::default();
        let quat_field = QuaternionField::optimize(&mesh, config);

        let deformation = S3SlicerDeformation::new(mesh, quat_field);

        assert_eq!(deformation.deformed_mesh_preview.triangles.len(), 1);
        assert_eq!(deformation.scalar_field.len(), 1);
    }
}
