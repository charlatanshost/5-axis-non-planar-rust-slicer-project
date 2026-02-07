// Volume-based Scalar Field for Tetrahedral Meshes
//
// Computes a smooth scalar field on the tet mesh for layer extraction:
// 1. Initial field: Z-coordinates from deformed mesh (height after deformation)
// 2. Laplacian smoothing to remove noise
// 3. The scalar field defines iso-values for layer extraction
//
// The paper uses a more complex Poisson refinement with volume matrix gradients,
// but Laplacian smoothing achieves similar results for most geometries.

use crate::geometry::{Point3D, Vector3D};
use crate::s3_slicer::tet_mesh::TetMesh;
use sprs::TriMat;
use std::collections::HashMap;

/// Scalar field defined on tet mesh vertices
#[derive(Debug, Clone)]
pub struct TetScalarField {
    /// Scalar value at each vertex
    pub values: Vec<f64>,

    /// Min/max values
    pub min_value: f64,
    pub max_value: f64,
}

/// Configuration for scalar field computation
#[derive(Debug, Clone)]
pub struct TetScalarFieldConfig {
    /// Number of Laplacian smoothing iterations
    pub smoothing_iterations: usize,

    /// Smoothing weight (0 = no smoothing, 1 = full Laplacian)
    pub smoothing_weight: f64,

    /// Use Z-coordinates from deformed mesh as initial field
    pub use_deformed_z: bool,
}

impl Default for TetScalarFieldConfig {
    fn default() -> Self {
        Self {
            smoothing_iterations: 5,
            smoothing_weight: 0.3,
            use_deformed_z: true,
        }
    }
}

impl TetScalarField {
    /// Compute scalar field from deformed tet mesh
    ///
    /// The scalar field is the Z-coordinate of the deformed mesh,
    /// smoothed to remove noise. This gives us virtual layer heights.
    pub fn from_deformed_mesh(
        original_mesh: &TetMesh,
        deformed_mesh: &TetMesh,
        config: &TetScalarFieldConfig,
    ) -> Self {
        log::info!("Computing tet scalar field...");

        let n = original_mesh.vertices.len();

        // Step 1: Initial field from deformed Z-coordinates
        let mut values: Vec<f64> = if config.use_deformed_z {
            deformed_mesh.vertices.iter().map(|v| v.z).collect()
        } else {
            original_mesh.vertices.iter().map(|v| v.z).collect()
        };

        log::info!("  Initial field range: {:.2} to {:.2}",
            values.iter().copied().fold(f64::INFINITY, f64::min),
            values.iter().copied().fold(f64::NEG_INFINITY, f64::max));

        // Step 2: Laplacian smoothing
        if config.smoothing_iterations > 0 && config.smoothing_weight > 0.0 {
            log::info!("  Applying {} Laplacian smoothing iterations (weight={:.2})...",
                config.smoothing_iterations, config.smoothing_weight);

            // Build vertex adjacency from tet edges
            let adjacency = build_vertex_adjacency(original_mesh);

            for iter in 0..config.smoothing_iterations {
                let mut new_values = values.clone();

                for vi in 0..n {
                    let neighbors = &adjacency[vi];
                    if neighbors.is_empty() {
                        continue;
                    }

                    // Compute Laplacian: average of neighbors minus current value
                    let avg: f64 = neighbors.iter()
                        .map(|&ni| values[ni])
                        .sum::<f64>() / neighbors.len() as f64;

                    // Blend current value with Laplacian average
                    new_values[vi] = values[vi] * (1.0 - config.smoothing_weight)
                        + avg * config.smoothing_weight;
                }

                values = new_values;

                if iter % 2 == 0 {
                    log::debug!("    Smoothing iteration {}", iter);
                }
            }
        }

        // Step 3: Normalize to start from 0
        let min_value = values.iter().copied().fold(f64::INFINITY, f64::min);
        for v in &mut values {
            *v -= min_value;
        }

        let min_value = 0.0;
        let max_value = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        log::info!("  Final scalar field range: {:.2} to {:.2}", min_value, max_value);

        Self { values, min_value, max_value }
    }

    /// Compute scalar field directly from quaternion field (virtual method)
    /// This avoids mesh deformation entirely - computes what Z WOULD BE if deformed
    pub fn from_quaternion_field(
        tet_mesh: &TetMesh,
        quaternion_field: &crate::s3_slicer::tet_quaternion_field::TetQuaternionField,
        config: &TetScalarFieldConfig,
    ) -> Self {
        log::info!("Computing virtual tet scalar field from quaternion field...");

        let n = tet_mesh.vertices.len();
        let min_z = tet_mesh.bounds_min.z;
        let z_range = (tet_mesh.bounds_max.z - tet_mesh.bounds_min.z).max(1.0);

        // Anchor point (bottom center)
        let anchor = Point3D::new(
            (tet_mesh.bounds_min.x + tet_mesh.bounds_max.x) / 2.0,
            (tet_mesh.bounds_min.y + tet_mesh.bounds_max.y) / 2.0,
            min_z,
        );

        // For each vertex, compute virtual deformed Z
        let mut values: Vec<f64> = Vec::with_capacity(n);

        for vi in 0..n {
            let pos = tet_mesh.vertices[vi];

            // Height parameter (0 at bottom, 1 at top)
            let height_param = ((pos.z - min_z) / z_range).clamp(0.0, 1.0);

            // Get rotation (averaged from surrounding tets)
            let rotation = quaternion_field.interpolate_at_vertex(tet_mesh, vi);

            // Scale rotation by height (progressive unfolding)
            let scaled_rotation = nalgebra::UnitQuaternion::identity()
                .slerp(&rotation, height_param);

            // Apply rotation around anchor
            let relative = Vector3D::new(
                pos.x - anchor.x,
                pos.y - anchor.y,
                pos.z - anchor.z,
            );

            let rotated = scaled_rotation * relative;
            let virtual_z = rotated.z + anchor.z;

            values.push(virtual_z);
        }

        // Laplacian smoothing
        if config.smoothing_iterations > 0 && config.smoothing_weight > 0.0 {
            let adjacency = build_vertex_adjacency(tet_mesh);

            for _ in 0..config.smoothing_iterations {
                let mut new_values = values.clone();
                for vi in 0..n {
                    let neighbors = &adjacency[vi];
                    if neighbors.is_empty() {
                        continue;
                    }
                    let avg: f64 = neighbors.iter()
                        .map(|&ni| values[ni])
                        .sum::<f64>() / neighbors.len() as f64;
                    new_values[vi] = values[vi] * (1.0 - config.smoothing_weight)
                        + avg * config.smoothing_weight;
                }
                values = new_values;
            }
        }

        // Normalize
        let min_value = values.iter().copied().fold(f64::INFINITY, f64::min);
        for v in &mut values {
            *v -= min_value;
        }

        let min_value = 0.0;
        let max_value = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        log::info!("  Virtual scalar field range: {:.2} to {:.2}", min_value, max_value);

        Self { values, min_value, max_value }
    }

    /// Get scalar value at a vertex
    pub fn value_at(&self, vertex_idx: usize) -> f64 {
        self.values.get(vertex_idx).copied().unwrap_or(0.0)
    }

    /// Interpolate scalar value within a tetrahedron using barycentric coordinates
    pub fn interpolate_in_tet(
        &self,
        tet_mesh: &TetMesh,
        tet_idx: usize,
        point: &Point3D,
    ) -> f64 {
        let tet = &tet_mesh.tets[tet_idx];
        let bary = compute_barycentric_tet(
            point,
            &tet_mesh.vertices[tet.vertices[0]],
            &tet_mesh.vertices[tet.vertices[1]],
            &tet_mesh.vertices[tet.vertices[2]],
            &tet_mesh.vertices[tet.vertices[3]],
        );

        bary[0] * self.values[tet.vertices[0]]
            + bary[1] * self.values[tet.vertices[1]]
            + bary[2] * self.values[tet.vertices[2]]
            + bary[3] * self.values[tet.vertices[3]]
    }
}

/// Build vertex adjacency from tet mesh edges
fn build_vertex_adjacency(tet_mesh: &TetMesh) -> Vec<Vec<usize>> {
    let n = tet_mesh.vertices.len();
    let mut adj = vec![Vec::new(); n];
    let mut seen: Vec<HashMap<usize, ()>> = vec![HashMap::new(); n];

    for tet in &tet_mesh.tets {
        for (a, b) in tet.edges() {
            if seen[a].insert(b, ()).is_none() {
                adj[a].push(b);
            }
            if seen[b].insert(a, ()).is_none() {
                adj[b].push(a);
            }
        }
    }

    adj
}

/// Compute barycentric coordinates of a point within a tetrahedron
fn compute_barycentric_tet(
    p: &Point3D,
    v0: &Point3D,
    v1: &Point3D,
    v2: &Point3D,
    v3: &Point3D,
) -> [f64; 4] {
    // Using the standard method: solve the system
    // p = w0*v0 + w1*v1 + w2*v2 + w3*v3, where w0+w1+w2+w3 = 1
    // Equivalent to: p - v3 = w0*(v0-v3) + w1*(v1-v3) + w2*(v2-v3)

    let e0 = *v0 - *v3;
    let e1 = *v1 - *v3;
    let e2 = *v2 - *v3;
    let ep = *p - *v3;

    let mat = nalgebra::Matrix3::new(
        e0.x, e1.x, e2.x,
        e0.y, e1.y, e2.y,
        e0.z, e1.z, e2.z,
    );

    let rhs = nalgebra::Vector3::new(ep.x, ep.y, ep.z);

    if let Some(inv) = mat.try_inverse() {
        let w = inv * rhs;
        let w3 = 1.0 - w[0] - w[1] - w[2];
        [w[0], w[1], w[2], w3]
    } else {
        // Degenerate tet - equal weights
        [0.25, 0.25, 0.25, 0.25]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s3_slicer::tet_mesh::Tetrahedron;
    use crate::s3_slicer::tet_quaternion_field::TetQuaternionField;
    use crate::s3_slicer::quaternion_field::QuaternionFieldConfig;

    fn make_test_mesh() -> TetMesh {
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(5.0, 10.0, 0.0),
            Point3D::new(5.0, 5.0, 10.0),
            Point3D::new(5.0, 5.0, -5.0),
        ];

        let tets = vec![
            Tetrahedron::new(0, 1, 2, 3),
            Tetrahedron::new(0, 1, 2, 4),
        ];

        TetMesh::new(vertices, tets)
    }

    #[test]
    fn test_scalar_field_from_deformed() {
        let mesh = make_test_mesh();
        let config = TetScalarFieldConfig::default();
        // Use same mesh as "deformed" (identity deformation)
        let field = TetScalarField::from_deformed_mesh(&mesh, &mesh, &config);

        assert_eq!(field.values.len(), mesh.vertices.len());
        assert!(field.min_value >= 0.0);
        assert!(field.max_value > 0.0);
    }

    #[test]
    fn test_scalar_field_from_quaternion() {
        let mesh = make_test_mesh();
        let qconfig = QuaternionFieldConfig::default();
        let qfield = TetQuaternionField::optimize(&mesh, qconfig);
        let sconfig = TetScalarFieldConfig::default();

        let field = TetScalarField::from_quaternion_field(&mesh, &qfield, &sconfig);

        assert_eq!(field.values.len(), mesh.vertices.len());
        assert!(field.max_value > 0.0);
    }

    #[test]
    fn test_barycentric_tet() {
        let v0 = Point3D::new(0.0, 0.0, 0.0);
        let v1 = Point3D::new(1.0, 0.0, 0.0);
        let v2 = Point3D::new(0.0, 1.0, 0.0);
        let v3 = Point3D::new(0.0, 0.0, 1.0);

        // Centroid should have equal bary coords
        let centroid = Point3D::new(0.25, 0.25, 0.25);
        let bary = compute_barycentric_tet(&centroid, &v0, &v1, &v2, &v3);

        for &w in &bary {
            assert!((w - 0.25).abs() < 1e-10);
        }

        // v0 should have bary [1,0,0,0]
        let bary0 = compute_barycentric_tet(&v0, &v0, &v1, &v2, &v3);
        assert!((bary0[0] - 1.0).abs() < 1e-10);
        assert!(bary0[1].abs() < 1e-10);
    }

    #[test]
    fn test_interpolate_in_tet() {
        let mesh = make_test_mesh();
        let config = TetScalarFieldConfig {
            smoothing_iterations: 0,
            ..Default::default()
        };
        let field = TetScalarField::from_deformed_mesh(&mesh, &mesh, &config);

        // Centroid of first tet
        let centroid = mesh.tets[0].centroid(&mesh.vertices);
        let val = field.interpolate_in_tet(&mesh, 0, &centroid);

        // Should be between min and max of tet vertex values
        let tet = &mesh.tets[0];
        let tet_vals: Vec<f64> = tet.vertices.iter().map(|&vi| field.values[vi]).collect();
        let tet_min = tet_vals.iter().copied().fold(f64::INFINITY, f64::min);
        let tet_max = tet_vals.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        assert!(val >= tet_min - 1e-6 && val <= tet_max + 1e-6,
            "Interpolated value {} should be in [{}, {}]", val, tet_min, tet_max);
    }
}
