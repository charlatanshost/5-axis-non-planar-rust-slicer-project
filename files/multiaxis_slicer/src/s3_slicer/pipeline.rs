// Complete S3-Slicer pipeline implementation
// Follows the correct algorithm from the SIGGRAPH Asia 2022 paper

use crate::mesh::Mesh;
use crate::slicing::Layer;
use crate::geometry::{Point3D, Vector3D};
use crate::s3_slicer::{
    QuaternionField, QuaternionFieldConfig, FabricationObjective,
    S3SlicerDeformation, ScalarField, ScalarFieldConfig, FieldType,
    IsosurfaceConfig, extract_curved_layers, CurvedLayer,
    AsapSolver, AsapConfig,
    TetMesh, TetQuaternionField, TetAsapSolver, TetAsapConfig,
    TetScalarField, TetScalarFieldConfig, extract_tet_layers,
};
use std::collections::HashMap;

/// Deformation method for S3-Slicer
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeformationMethod {
    /// Virtual scalar field - computes scalar field directly without mesh deformation
    /// This is the RECOMMENDED method - avoids mesh collapse issues
    /// The mesh preview won't show deformation, but slicing works correctly
    VirtualScalarField,

    /// ASAP deformation - full mesh deformation using As-Rigid-As-Possible solver
    /// Can cause mesh collapse on complex models (like upside-down benchy)
    AsapDeformation,

    /// Scale-controlled deformation - per-vertex local deformation
    /// Faster than ASAP but less accurate
    ScaleControlled,

    /// Tetrahedral volumetric pipeline - full volume mesh deformation
    /// Uses TetGen for tetrahedralization, per-tet ASAP deformation with scaling,
    /// and marching tetrahedra for isosurface extraction.
    /// Most accurate method from the original S3-Slicer paper.
    TetVolumetric,
}

/// Configuration for the complete S3-Slicer pipeline
#[derive(Debug, Clone)]
pub struct S3PipelineConfig {
    /// Fabrication objective for quaternion optimization
    pub objective: FabricationObjective,

    /// Layer height for slicing (mm)
    pub layer_height: f64,

    /// Quaternion field optimization iterations
    pub optimization_iterations: usize,

    /// Overhang threshold for support-free objective (degrees)
    pub overhang_threshold: f64,

    /// Smoothness weight for quaternion field
    pub smoothness_weight: f64,

    /// Maximum rotation angle per triangle (degrees)
    /// Limits how much each triangle can rotate to prevent extreme deformations
    /// Typical range: 10-30 degrees
    pub max_rotation_degrees: f64,

    /// Deformation method to use
    pub deformation_method: DeformationMethod,

    /// Use ASAP deformation (true) or scale-controlled (false)
    /// DEPRECATED: Use deformation_method instead
    pub use_asap_deformation: bool,

    /// ASAP solver configuration (only used if use_asap_deformation = true)
    pub asap_max_iterations: usize,

    /// ASAP convergence threshold
    pub asap_convergence_threshold: f64,
}

impl Default for S3PipelineConfig {
    fn default() -> Self {
        Self {
            objective: FabricationObjective::SupportFree,
            layer_height: 0.2,
            optimization_iterations: 50,
            overhang_threshold: 45.0,
            smoothness_weight: 0.5,
            max_rotation_degrees: 15.0,  // Conservative default to prevent extreme deformations
            deformation_method: DeformationMethod::VirtualScalarField,  // Default to virtual - no mesh collapse
            use_asap_deformation: false,  // Deprecated - use deformation_method
            asap_max_iterations: 3,      // 3 iterations is usually enough for ARAP convergence
            asap_convergence_threshold: 1e-3,  // Higher threshold for faster termination
        }
    }
}

/// Result of the S3-Slicer pipeline
pub struct S3PipelineResult {
    /// Original mesh
    pub original_mesh: Mesh,

    /// Deformed mesh
    pub deformed_mesh: Mesh,

    /// Optimized quaternion field
    pub quaternion_field: QuaternionField,

    /// Deformation system (with inverse mapping)
    pub deformation: S3SlicerDeformation,

    /// Scalar field on deformed mesh
    pub deformed_scalar_field: ScalarField,

    /// Scalar field mapped back to original mesh
    pub original_scalar_field: Vec<f64>,

    /// Extracted curved layers
    pub curved_layers: Vec<CurvedLayer>,

    /// Converted to standard layers for compatibility
    pub layers: Vec<Layer>,
}

impl S3PipelineResult {
    /// Get the final layers ready for toolpath generation
    pub fn get_layers(&self) -> &[Layer] {
        &self.layers
    }

    /// Get statistics about the slicing
    pub fn stats(&self) -> PipelineStats {
        PipelineStats {
            num_layers: self.layers.len(),
            quaternion_field_energy: self.quaternion_field.energy,
            original_mesh_triangles: self.original_mesh.triangles.len(),
            deformed_mesh_triangles: self.deformed_mesh.triangles.len(),
        }
    }
}

/// Statistics from the pipeline
#[derive(Debug, Clone)]
pub struct PipelineStats {
    pub num_layers: usize,
    pub quaternion_field_energy: f64,
    pub original_mesh_triangles: usize,
    pub deformed_mesh_triangles: usize,
}

/// Compute virtual scalar field directly from quaternion field
///
/// This is the key function that avoids mesh collapse. Instead of actually deforming
/// the mesh (which can cause collapse with surface meshes), we compute what the
/// Z-heights WOULD BE if the mesh were deformed.
///
/// Algorithm:
/// 1. Find the anchor point (bottom center of mesh)
/// 2. For each triangle, apply the quaternion rotation to its centroid
/// 3. The virtual Z-height is the Z-coordinate of the rotated centroid
/// 4. This gives us a scalar field that represents "virtual deformed heights"
fn compute_virtual_scalar_field(mesh: &Mesh, quaternion_field: &QuaternionField) -> Vec<f64> {
    let num_triangles = mesh.triangles.len();
    log::info!("  Computing virtual scalar field for {} triangles...", num_triangles);

    // Find anchor point: bottom center of mesh
    let min_z = mesh.bounds_min.z;
    let anchor = Point3D::new(
        (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0,
        (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0,
        min_z,
    );

    // Compute virtual Z-height for each triangle
    // This represents what the Z would be IF the mesh were deformed
    let mut scalar_values: Vec<f64> = Vec::with_capacity(num_triangles);

    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        // Get triangle centroid
        let centroid = Point3D::new(
            (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0,
            (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0,
            (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0,
        );

        // Get quaternion rotation for this triangle
        let quaternion = quaternion_field.rotations.get(tri_idx)
            .copied()
            .unwrap_or(nalgebra::UnitQuaternion::identity());

        // Compute height parameter (0 at bottom, 1 at top)
        let height_range = (mesh.bounds_max.z - mesh.bounds_min.z).max(1.0);
        let height_param = ((centroid.z - min_z) / height_range).clamp(0.0, 1.0);

        // Scale rotation by height - bottom triangles move less, top triangles move more
        // This creates a progressive unfolding effect from bottom to top
        let scaled_quaternion = nalgebra::UnitQuaternion::identity()
            .slerp(&quaternion, height_param);

        // Apply rotation around anchor point to get virtual position
        let relative = Vector3D::new(
            centroid.x - anchor.x,
            centroid.y - anchor.y,
            centroid.z - anchor.z,
        );

        let rotated = scaled_quaternion * relative;

        // The virtual Z-height is the Z-coordinate after rotation
        // Add anchor.z back to get absolute height
        let virtual_z = rotated.z + anchor.z;

        scalar_values.push(virtual_z);
    }

    // Normalize scalar field to start from 0
    let min_scalar = scalar_values.iter().copied().fold(f64::INFINITY, f64::min);
    for value in &mut scalar_values {
        *value -= min_scalar;
    }

    let max_scalar = scalar_values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    log::info!("  Virtual scalar field range: 0.0 to {:.2} mm", max_scalar);

    scalar_values
}

/// Execute the complete S3-Slicer pipeline
pub fn execute_s3_pipeline(
    original_mesh: Mesh,
    config: S3PipelineConfig,
) -> S3PipelineResult {
    log::info!("=== S3-Slicer Pipeline Started ===");

    // Step 1: Optimize quaternion field
    log::info!("Step 1/5: Optimizing quaternion field...");
    let quat_config = QuaternionFieldConfig {
        objective: config.objective,
        build_direction: crate::geometry::Vector3D::new(0.0, 0.0, 1.0),
        optimization_iterations: config.optimization_iterations,
        smoothness_weight: config.smoothness_weight,
        objective_weight: 1.0,
        overhang_threshold: config.overhang_threshold,
        max_rotation_degrees: config.max_rotation_degrees,
    };

    // For TetVolumetric, use the dedicated tet pipeline (completely different flow)
    if matches!(config.deformation_method, DeformationMethod::TetVolumetric) {
        return execute_tet_pipeline(original_mesh, config, &quat_config);
    }

    let quaternion_field = QuaternionField::optimize(&original_mesh, quat_config);
    log::info!("  → Quaternion field energy: {:.4}", quaternion_field.energy);

    // Step 2: Compute scalar field using selected deformation method
    // VirtualScalarField = compute scalar field directly without mesh deformation (RECOMMENDED)
    // ASAP = global deformation (whole sections move together) - can cause mesh collapse
    // ScaleControlled = local per-vertex deformation (faster but less accurate)
    let (deformed_mesh, scalar_field_from_deformation, deformation) = match config.deformation_method {
        DeformationMethod::VirtualScalarField => {
            log::info!("Step 2/5: Computing VIRTUAL scalar field (no mesh deformation)...");
            log::info!("  This method computes the scalar field directly from quaternion rotations");
            log::info!("  The mesh preview won't show deformation, but slicing works correctly");

            // Compute virtual scalar field directly from quaternion field
            // For each vertex, we compute what its Z would be IF the mesh were deformed
            let scalar_field = compute_virtual_scalar_field(&original_mesh, &quaternion_field);

            log::info!("  → Virtual scalar field computed for {} triangles", scalar_field.len());

            // Create deformation object for API compatibility (no actual deformation)
            let deformation = S3SlicerDeformation::new(original_mesh.clone(), quaternion_field.clone());

            // Use original mesh as "deformed" mesh (for preview - it won't show deformation)
            (original_mesh.clone(), scalar_field, deformation)
        }
        DeformationMethod::AsapDeformation => {
            log::info!("Step 2/5: Applying ASAP deformation (global, high quality)...");
            log::info!("  WARNING: ASAP can cause mesh collapse on complex models");

            let asap_config = AsapConfig {
                max_iterations: config.asap_max_iterations,
                convergence_threshold: config.asap_convergence_threshold,
                quaternion_weight: 0.3,
                constraint_weight: 500.0,
                use_cotangent_weights: true,
            };

            let solver = AsapSolver::new(original_mesh.clone(), quaternion_field.clone(), asap_config);
            let asap_deformed = solver.solve();

            log::info!("  → ASAP deformed mesh created: {} triangles", asap_deformed.triangles.len());

            let scalar_field: Vec<f64> = asap_deformed.triangles.iter()
                .map(|tri| (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0)
                .collect();

            let min_z = scalar_field.iter().copied().fold(f64::INFINITY, f64::min);
            let normalized_field: Vec<f64> = scalar_field.iter().map(|z| z - min_z).collect();

            let deformation = S3SlicerDeformation::new(original_mesh.clone(), quaternion_field.clone());

            (asap_deformed, normalized_field, deformation)
        }
        DeformationMethod::ScaleControlled => {
            log::info!("Step 2/5: Applying scale-controlled deformation (local, fast)...");

            let deformation = S3SlicerDeformation::new(original_mesh.clone(), quaternion_field.clone());
            let deformed_mesh = deformation.get_deformed_mesh().clone();
            let scalar_field = deformation.get_scalar_field().to_vec();

            log::info!("  → Deformed mesh created: {} triangles", deformed_mesh.triangles.len());

            (deformed_mesh, scalar_field, deformation)
        }
        DeformationMethod::TetVolumetric => {
            // This arm is unreachable — TetVolumetric is handled by the early return above.
            unreachable!("TetVolumetric is handled by execute_tet_pipeline()");
        }
    };

    // Step 3: Use virtual scalar field computed from deformation
    // The scalar field represents VIRTUAL heights (deformed Z mapped back to original)
    log::info!("Step 3/5: Using virtual scalar field from deformation...");

    let scalar_field_values = &scalar_field_from_deformation;
    let min_value = scalar_field_values.iter().copied().fold(f64::INFINITY, f64::min);
    let max_value = scalar_field_values.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    log::info!("  → Virtual scalar field range: {:.2} to {:.2}", min_value, max_value);

    // Create ScalarField wrapper for compatibility
    let virtual_scalar_field = ScalarField {
        values: scalar_field_values.to_vec(),
        min_value,
        max_value,
        config: ScalarFieldConfig::default(),
    };

    // Step 4: Extract isosurfaces from ORIGINAL mesh using VIRTUAL scalar field
    // This is the correct S3-Slicer approach: slice original mesh with virtual heights
    log::info!("Step 4/5: Extracting curved layers from ORIGINAL mesh using virtual scalar field...");

    // Calculate number of layers based on VIRTUAL height range
    let height_range = max_value - min_value;
    let num_layers = (height_range / config.layer_height).ceil() as usize;

    let iso_config = IsosurfaceConfig {
        num_layers,
        layer_height: config.layer_height,
        min_contour_length: 1.0,
        simplification_tolerance: 0.01,
        ..Default::default()
    };

    // Extract from ORIGINAL mesh using VIRTUAL scalar field
    // Toolpaths will already be in original mesh space (no inverse mapping needed!)
    let curved_layers = extract_curved_layers(&original_mesh, &virtual_scalar_field, &iso_config);
    log::info!("  → Extracted {} curved layers from original mesh", curved_layers.len());

    // Debug: Log info about curved layers
    for (i, cl) in curved_layers.iter().enumerate().take(5) {
        log::info!("    Layer {}: z={:.2}, {} segments, iso_value={:.2}",
            i, cl.z, cl.segments.len(), cl.iso_value);
    }
    if curved_layers.len() > 5 {
        log::info!("    ... and {} more layers", curved_layers.len() - 5);
    }

    // Step 5: Convert layers for toolpath generation
    log::info!("Step 5/5: Preparing layers for toolpath generation...");

    // Convert to standard Layer format
    let layers: Vec<Layer> = curved_layers.iter()
        .map(|cl| cl.to_layer())
        .collect();

    // Debug: Log info about converted layers
    let total_contours: usize = layers.iter().map(|l| l.contours.len()).sum();
    let total_points: usize = layers.iter()
        .flat_map(|l| l.contours.iter())
        .map(|c| c.points.len())
        .sum();
    log::info!("  → Converted to {} standard layers with {} total contours, {} total points",
        layers.len(), total_contours, total_points);

    log::info!("=== S3-Slicer Pipeline Complete ===");
    let deformation_type = if config.use_asap_deformation {
        "ASAP global deformation"
    } else {
        "scale-controlled local deformation"
    };
    log::info!("Generated {} layers using {} (virtual deformation)", layers.len(), deformation_type);
    log::info!("Toolpaths are in ORIGINAL mesh space (no inverse mapping needed)");

    // Use our computed scalar field (from ASAP or scale-controlled, depending on config)
    let original_scalar_field_values = scalar_field_from_deformation;

    S3PipelineResult {
        original_mesh,
        deformed_mesh,
        quaternion_field,
        deformation,
        deformed_scalar_field: virtual_scalar_field,
        original_scalar_field: original_scalar_field_values,
        curved_layers,
        layers,
    }
}

/// Execute the tetrahedral volumetric pipeline
///
/// This follows the original S3-Slicer paper more closely:
/// 1. Surface mesh → Tetrahedral volume mesh (TetGen)
/// 2. Per-tet quaternion field optimization
/// 3. Volumetric ASAP deformation (with per-tet scaling)
/// 4. Scalar field on tet mesh (deformed Z + Laplacian smoothing)
/// 5. Marching tetrahedra for isosurface extraction
/// 6. Convert curved layers to standard layers
fn execute_tet_pipeline(
    original_mesh: Mesh,
    config: S3PipelineConfig,
    quat_config: &QuaternionFieldConfig,
) -> S3PipelineResult {
    log::info!("=== Tetrahedral Volumetric Pipeline ===");

    // Step 1: Tetrahedralize the surface mesh
    log::info!("Step 1/6: Tetrahedralizing surface mesh...");
    let tet_mesh = match TetMesh::from_surface_mesh(&original_mesh) {
        Ok(mesh) => {
            let quality = mesh.check_quality();
            log::info!("  → TetMesh: {} vertices, {} tets, {} surface tris",
                quality.num_vertices, quality.num_tets, quality.num_surface_triangles);
            log::info!("  → Volume range: {:.6} to {:.6}", quality.min_tet_volume, quality.max_tet_volume);
            if quality.degenerate_tets > 0 {
                log::warn!("  → {} degenerate tets", quality.degenerate_tets);
            }
            mesh
        }
        Err(e) => {
            log::error!("Tetrahedralization failed: {}", e);
            log::warn!("Falling back to VirtualScalarField pipeline (naive tet fallback is too low quality)");
            let fallback_config = S3PipelineConfig {
                deformation_method: DeformationMethod::VirtualScalarField,
                ..config
            };
            return execute_s3_pipeline(original_mesh, fallback_config);
        }
    };

    // Step 2: Optimize per-tet quaternion field
    log::info!("Step 2/6: Optimizing per-tet quaternion field...");
    let tet_qfield = TetQuaternionField::optimize(&tet_mesh, quat_config.clone());
    log::info!("  → Tet quaternion field energy: {:.4}", tet_qfield.energy);
    let non_identity = tet_qfield.rotations.iter().filter(|q| q.angle() > 0.01).count();
    log::info!("  → Tets with rotation: {}/{}", non_identity, tet_mesh.tets.len());

    // Step 3: Volumetric ASAP deformation
    log::info!("Step 3/6: Running volumetric ASAP deformation...");
    let tet_asap_config = TetAsapConfig {
        max_iterations: config.asap_max_iterations,
        convergence_threshold: config.asap_convergence_threshold,
        quaternion_weight: 0.3,
        constraint_weight: 500.0,
        allow_scaling: true, // ASAP (with scaling) - key difference from surface ARAP
    };

    let solver = TetAsapSolver::new(tet_mesh.clone(), tet_qfield.clone(), tet_asap_config);
    let deformed_tet_mesh = solver.solve();

    let deformed_quality = deformed_tet_mesh.check_quality();
    log::info!("  → Deformed: {} inverted tets, {} degenerate tets",
        deformed_quality.inverted_tets, deformed_quality.degenerate_tets);

    // Quality check: detect deformation blowup
    let orig_extent = (tet_mesh.bounds_max - tet_mesh.bounds_min).norm();
    let deformed_extent = (deformed_tet_mesh.bounds_max - deformed_tet_mesh.bounds_min).norm();
    let extent_ratio = deformed_extent / orig_extent.max(1e-10);
    let inverted_ratio = deformed_quality.inverted_tets as f64 / tet_mesh.tets.len().max(1) as f64;

    log::info!("  → Extent ratio (deformed/original): {:.2}x", extent_ratio);
    log::info!("  → Inverted tet ratio: {:.1}%", inverted_ratio * 100.0);

    if extent_ratio > 5.0 || inverted_ratio > 0.3 {
        log::error!("Deformation quality too poor (extent {:.1}x, {:.1}% inverted). Falling back to VirtualScalarField.",
            extent_ratio, inverted_ratio * 100.0);
        let fallback_config = S3PipelineConfig {
            deformation_method: DeformationMethod::VirtualScalarField,
            ..config
        };
        return execute_s3_pipeline(original_mesh, fallback_config);
    }

    // Step 4: Compute scalar field on tet mesh
    log::info!("Step 4/6: Computing volume-based scalar field...");
    let scalar_config = TetScalarFieldConfig {
        smoothing_iterations: 5,
        smoothing_weight: 0.3,
        use_deformed_z: true,
    };

    let tet_scalar_field = TetScalarField::from_deformed_mesh(
        &tet_mesh, &deformed_tet_mesh, &scalar_config,
    );
    log::info!("  → Scalar field range: {:.2} to {:.2}",
        tet_scalar_field.min_value, tet_scalar_field.max_value);

    // Step 5: Extract curved layers via marching tetrahedra
    log::info!("Step 5/6: Extracting layers via marching tetrahedra...");
    let curved_layers = extract_tet_layers(
        &tet_mesh,
        &tet_scalar_field,
        config.layer_height,
        1.0, // min_contour_length
    );
    log::info!("  → Extracted {} curved layers", curved_layers.len());

    for (i, cl) in curved_layers.iter().enumerate().take(5) {
        log::info!("    Layer {}: z={:.2}, {} segments, iso_value={:.2}",
            i, cl.z, cl.segments.len(), cl.iso_value);
    }
    if curved_layers.len() > 5 {
        log::info!("    ... and {} more layers", curved_layers.len() - 5);
    }

    // Step 6: Convert to standard layers
    log::info!("Step 6/6: Converting to standard layers...");
    let layers: Vec<Layer> = curved_layers.iter()
        .map(|cl| cl.to_layer())
        .collect();

    let total_contours: usize = layers.iter().map(|l| l.contours.len()).sum();
    let total_points: usize = layers.iter()
        .flat_map(|l| l.contours.iter())
        .map(|c| c.points.len())
        .sum();
    log::info!("  → {} standard layers, {} contours, {} points",
        layers.len(), total_contours, total_points);

    // Build result with compatibility fields
    let deformed_surface_mesh = deformed_tet_mesh.to_surface_mesh();

    // Create placeholder surface quaternion field for API compatibility
    let num_triangles = original_mesh.triangles.len();
    let placeholder_qfield = QuaternionField {
        rotations: vec![nalgebra::UnitQuaternion::identity(); num_triangles],
        energy: tet_qfield.energy,
        config: quat_config.clone(),
    };

    let deformation = S3SlicerDeformation::new(
        original_mesh.clone(),
        placeholder_qfield.clone(),
    );

    // Build per-triangle scalar field for compatibility
    let height_range = tet_scalar_field.max_value - tet_scalar_field.min_value;
    let per_tri_scalar: Vec<f64> = original_mesh.triangles.iter()
        .map(|tri| {
            let centroid_z = (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0;
            let mesh_z_range = (original_mesh.bounds_max.z - original_mesh.bounds_min.z).max(1.0);
            let normalized = (centroid_z - original_mesh.bounds_min.z) / mesh_z_range;
            normalized * height_range
        })
        .collect();

    let virtual_scalar_field = ScalarField {
        values: per_tri_scalar.clone(),
        min_value: 0.0,
        max_value: height_range,
        config: ScalarFieldConfig::default(),
    };

    log::info!("=== Tetrahedral Volumetric Pipeline Complete ===");
    log::info!("Generated {} layers via marching tetrahedra", layers.len());

    S3PipelineResult {
        original_mesh,
        deformed_mesh: deformed_surface_mesh,
        quaternion_field: placeholder_qfield,
        deformation,
        deformed_scalar_field: virtual_scalar_field,
        original_scalar_field: per_tri_scalar,
        curved_layers,
        layers,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point3D, Triangle};

    #[test]
    fn test_s3_pipeline() {
        let triangle = Triangle::new(
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(5.0, 10.0, 5.0),
        );

        let mesh = Mesh {
            triangles: vec![triangle],
            bounds_min: Point3D::new(0.0, 0.0, 0.0),
            bounds_max: Point3D::new(10.0, 10.0, 5.0),
        };

        let config = S3PipelineConfig::default();
        let result = execute_s3_pipeline(mesh, config);

        assert!(!result.layers.is_empty());
        assert_eq!(result.original_mesh.triangles.len(), result.deformed_mesh.triangles.len());
    }
}
