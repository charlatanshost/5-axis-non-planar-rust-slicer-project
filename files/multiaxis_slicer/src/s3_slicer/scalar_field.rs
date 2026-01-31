// Scalar field computation for S3-Slicer
// Defines curved layers as isosurfaces of a scalar field

use crate::geometry::{Point3D, Vector3D};
use crate::mesh::Mesh;
use std::collections::HashMap;
use rayon::prelude::*;

/// Type of scalar field to generate
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FieldType {
    /// Simple height-based field (Z-coordinate)
    Height,

    /// Geodesic distance from base surface
    Geodesic,

    /// Deformation-based field following surface geometry
    Deformation,

    /// Custom field defined by user function
    Custom,
}

/// Configuration for scalar field generation
#[derive(Debug, Clone)]
pub struct ScalarFieldConfig {
    /// Type of field to generate
    pub field_type: FieldType,

    /// Number of samples per mesh triangle
    pub samples_per_triangle: usize,

    /// Build direction vector (typically [0, 0, 1] for vertical)
    pub build_direction: Vector3D,

    /// Base surface Z-coordinate (platform height)
    pub base_z: f64,

    /// Smoothing iterations for field regularization
    pub smoothing_iterations: usize,

    /// Smoothing factor (0.0 to 1.0)
    pub smoothing_factor: f64,
}

impl Default for ScalarFieldConfig {
    fn default() -> Self {
        Self {
            field_type: FieldType::Height,
            samples_per_triangle: 3,  // Sample at vertices
            build_direction: Vector3D::new(0.0, 0.0, 1.0),
            base_z: 0.0,
            smoothing_iterations: 5,
            smoothing_factor: 0.5,
        }
    }
}

/// Scalar field defined over a mesh
#[derive(Clone)]
pub struct ScalarField {
    /// Scalar value at each mesh vertex
    pub values: Vec<f64>,

    /// Minimum scalar value
    pub min_value: f64,

    /// Maximum scalar value
    pub max_value: f64,

    /// Configuration used to generate this field
    pub config: ScalarFieldConfig,
}

impl ScalarField {
    /// Compute scalar field from mesh
    pub fn compute(mesh: &Mesh, config: ScalarFieldConfig) -> Self {
        let values = match config.field_type {
            FieldType::Height => compute_height_field(mesh, &config),
            FieldType::Geodesic => compute_geodesic_field(mesh, &config),
            FieldType::Deformation => compute_deformation_field(mesh, &config),
            FieldType::Custom => compute_height_field(mesh, &config),  // Fallback
        };

        // Smooth the field if requested
        let smoothed_values = if config.smoothing_iterations > 0 {
            smooth_field(&values, mesh, &config)
        } else {
            values
        };

        let min_value = smoothed_values.iter().copied().fold(f64::INFINITY, f64::min);
        let max_value = smoothed_values.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        Self {
            values: smoothed_values,
            min_value,
            max_value,
            config,
        }
    }

    /// Get scalar value at a specific vertex
    pub fn value_at_vertex(&self, vertex_idx: usize) -> Option<f64> {
        self.values.get(vertex_idx).copied()
    }

    /// Interpolate scalar value within a triangle
    pub fn interpolate_in_triangle(
        &self,
        tri_idx: usize,
        mesh: &Mesh,
        point: &Point3D,
    ) -> Option<f64> {
        if tri_idx >= mesh.triangles.len() {
            return None;
        }

        let triangle = &mesh.triangles[tri_idx];

        // Get scalar values at triangle vertices
        // Note: This assumes we have a way to get vertex indices from triangle
        // For now, we'll compute barycentric coordinates and interpolate

        // Compute barycentric coordinates of point in triangle
        let bary = compute_barycentric_coords(triangle, point)?;

        // We need vertex indices to look up scalar values
        // For simplicity, we'll return None for now
        // In a full implementation, mesh would track vertex indices per triangle
        None
    }

    /// Generate isosurface contours at a given scalar value
    pub fn extract_isosurface(&self, iso_value: f64, mesh: &Mesh) -> Vec<Point3D> {
        let mut contour_points = Vec::new();

        // For each triangle, check if isosurface passes through it
        for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
            // Get vertex positions (we need vertex indexing for this to work properly)
            // For now, this is a placeholder
            // In full implementation, we'd use marching triangles algorithm

            // Skip for now - requires proper vertex indexing
        }

        contour_points
    }

    /// Get the range of scalar values
    pub fn range(&self) -> (f64, f64) {
        (self.min_value, self.max_value)
    }

    /// Normalize field values to [0, 1] range
    pub fn normalize(&mut self) {
        let range = self.max_value - self.min_value;
        if range > 1e-10 {
            for value in &mut self.values {
                *value = (*value - self.min_value) / range;
            }
            self.min_value = 0.0;
            self.max_value = 1.0;
        }
    }
}

/// Compute simple height-based scalar field (Z-coordinate)
fn compute_height_field(mesh: &Mesh, config: &ScalarFieldConfig) -> Vec<f64> {
    // Parallel computation of Z-values for triangle centroids
    let values: Vec<f64> = mesh.triangles
        .par_iter()
        .map(|tri| {
            // Average Z of triangle vertices
            let avg_z = (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0;
            avg_z - config.base_z
        })
        .collect();

    // Debug: Check for non-finite values
    let nan_count = values.iter().filter(|&&v| !v.is_finite()).count();
    if nan_count > 0 {
        log::warn!("  WARNING: {} height field values are non-finite! base_z = {}", nan_count, config.base_z);
        // Check mesh vertices
        let nan_verts = mesh.triangles.iter().filter(|tri| {
            !tri.v0.z.is_finite() || !tri.v1.z.is_finite() || !tri.v2.z.is_finite()
        }).count();
        log::warn!("  Triangles with non-finite Z coordinates: {}", nan_verts);
    }

    values
}

/// Compute geodesic distance field from base surface
fn compute_geodesic_field(mesh: &Mesh, config: &ScalarFieldConfig) -> Vec<f64> {
    // Placeholder: Use Dijkstra's algorithm on mesh graph
    // For now, return height-based approximation
    compute_height_field(mesh, config)
}

/// Compute deformation-based field following surface geometry
/// This is the core S3-Slicer algorithm
fn compute_deformation_field(mesh: &Mesh, config: &ScalarFieldConfig) -> Vec<f64> {
    // Parallel computation of deformation field
    // Algorithm:
    // 1. Start with height field as initial guess
    // 2. Compute surface normals
    // 3. Deform field to align with build direction
    // 4. Optimize for smoothness and printability

    mesh.triangles
        .par_iter()
        .map(|triangle| {
            let normal = triangle.normal();
            let center_z = (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0;

            // Compute alignment with build direction
            let alignment = normal.dot(&config.build_direction).abs();

            // Deform based on alignment (more aligned = larger field value offset)
            let deformation = (1.0 - alignment) * 2.0;  // Adjust multiplier as needed

            center_z - config.base_z + deformation
        })
        .collect()
}

/// Smooth scalar field using Laplacian smoothing
fn smooth_field(values: &[f64], mesh: &Mesh, config: &ScalarFieldConfig) -> Vec<f64> {
    let mut smoothed = values.to_vec();

    // Build adjacency information
    // For each triangle, find neighbors (triangles sharing an edge)
    let adjacency = build_triangle_adjacency(mesh);

    for _iteration in 0..config.smoothing_iterations {
        let mut new_values = smoothed.clone();

        for (i, neighbors) in adjacency.iter().enumerate() {
            if neighbors.is_empty() {
                continue;
            }

            // Average with neighbors
            let neighbor_avg: f64 = neighbors.iter().map(|&j| smoothed[j]).sum::<f64>() / neighbors.len() as f64;

            // Blend with smoothing factor
            new_values[i] = (1.0 - config.smoothing_factor) * smoothed[i]
                + config.smoothing_factor * neighbor_avg;
        }

        smoothed = new_values;
    }

    smoothed
}

/// Build triangle adjacency graph
fn build_triangle_adjacency(mesh: &Mesh) -> Vec<Vec<usize>> {
    let mut adjacency = vec![Vec::new(); mesh.triangles.len()];

    // Build edge map: edge -> triangles sharing that edge
    let mut edge_map: HashMap<(u64, u64), Vec<usize>> = HashMap::new();

    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        // Get three edges of triangle
        // We need to hash vertices somehow - use position as key
        let v0_hash = hash_point(&triangle.v0);
        let v1_hash = hash_point(&triangle.v1);
        let v2_hash = hash_point(&triangle.v2);

        let edges = [
            (v0_hash.min(v1_hash), v0_hash.max(v1_hash)),
            (v1_hash.min(v2_hash), v1_hash.max(v2_hash)),
            (v2_hash.min(v0_hash), v2_hash.max(v0_hash)),
        ];

        for edge in &edges {
            edge_map.entry(*edge).or_insert_with(Vec::new).push(tri_idx);
        }
    }

    // Build adjacency from edge map
    for triangles in edge_map.values() {
        if triangles.len() == 2 {
            // Two triangles share this edge
            adjacency[triangles[0]].push(triangles[1]);
            adjacency[triangles[1]].push(triangles[0]);
        }
    }

    adjacency
}

/// Hash a 3D point to u64 for use as map key
fn hash_point(point: &Point3D) -> u64 {
    // Simple hash: quantize to grid and combine coordinates
    let scale = 1000.0;  // 1mm precision
    let x = (point.x * scale).round() as i64;
    let y = (point.y * scale).round() as i64;
    let z = (point.z * scale).round() as i64;

    // Combine into u64 (simple but effective)
    ((x as u64) << 42) ^ ((y as u64) << 21) ^ (z as u64)
}

/// Compute barycentric coordinates of point in triangle
fn compute_barycentric_coords(
    triangle: &crate::geometry::Triangle,
    point: &Point3D,
) -> Option<(f64, f64, f64)> {
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
        return None;  // Degenerate triangle
    }

    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;

    Some((u, v, w))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_height_field() {
        // Create simple test mesh (single triangle)
        use crate::geometry::Triangle;

        let triangle = Triangle::new(
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 5.0),
            Point3D::new(0.0, 10.0, 5.0),
        );

        let mesh = Mesh {
            triangles: vec![triangle],
            bounds_min: Point3D::new(0.0, 0.0, 0.0),
            bounds_max: Point3D::new(10.0, 10.0, 5.0),
        };

        let config = ScalarFieldConfig::default();
        let field = ScalarField::compute(&mesh, config);

        // Should have one value (one triangle)
        assert_eq!(field.values.len(), 1);

        // Value should be approximately (0 + 5 + 5) / 3 = 3.33
        assert!((field.values[0] - 3.33).abs() < 0.1);
    }

    #[test]
    fn test_field_normalization() {
        let mut field = ScalarField {
            values: vec![10.0, 20.0, 30.0, 40.0],
            min_value: 10.0,
            max_value: 40.0,
            config: ScalarFieldConfig::default(),
        };

        field.normalize();

        assert_eq!(field.min_value, 0.0);
        assert_eq!(field.max_value, 1.0);
        assert_eq!(field.values[0], 0.0);
        assert_eq!(field.values[3], 1.0);
    }
}
