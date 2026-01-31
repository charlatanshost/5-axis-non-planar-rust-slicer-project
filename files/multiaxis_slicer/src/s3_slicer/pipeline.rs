// Complete S3-Slicer pipeline implementation
// Follows the correct algorithm from the SIGGRAPH Asia 2022 paper

use crate::mesh::Mesh;
use crate::slicing::Layer;
use crate::s3_slicer::{
    QuaternionField, QuaternionFieldConfig, FabricationObjective,
    S3SlicerDeformation, ScalarField, ScalarFieldConfig, FieldType,
    IsosurfaceConfig, extract_curved_layers, CurvedLayer,
    AsapSolver, AsapConfig,
};

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

    /// Use ASAP deformation (true) or scale-controlled (false)
    /// ASAP provides higher quality but is slower
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
            use_asap_deformation: false, // Default to scale-controlled (faster)
            asap_max_iterations: 10,
            asap_convergence_threshold: 1e-4,
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

    let quaternion_field = QuaternionField::optimize(&original_mesh, quat_config);
    log::info!("  → Quaternion field energy: {:.4}", quaternion_field.energy);

    // Step 2: Apply deformation to create deformed mesh
    let (deformation, deformed_mesh) = if config.use_asap_deformation {
        log::info!("Step 2/5: Applying ASAP deformation (high quality)...");

        // Use ASAP solver
        let asap_config = AsapConfig {
            max_iterations: config.asap_max_iterations,
            convergence_threshold: config.asap_convergence_threshold,
            quaternion_weight: 1.0,
            constraint_weight: 100.0,
            use_cotangent_weights: true,
        };

        let solver = AsapSolver::new(original_mesh.clone(), quaternion_field.clone(), asap_config);
        let asap_deformed = solver.solve();

        log::info!("  → ASAP deformed mesh created: {} triangles", asap_deformed.triangles.len());

        // Still create S3SlicerDeformation for compatibility
        let deformation = S3SlicerDeformation::new(original_mesh.clone(), quaternion_field.clone());

        (deformation, asap_deformed)
    } else {
        log::info!("Step 2/5: Applying scale-controlled deformation (fast)...");

        // Use existing scale-controlled deformation
        let deformation = S3SlicerDeformation::new(original_mesh.clone(), quaternion_field.clone());
        let deformed_mesh = deformation.get_deformed_mesh().clone();

        log::info!("  → Deformed mesh created: {} triangles", deformed_mesh.triangles.len());

        (deformation, deformed_mesh)
    };

    // Step 3: Get virtual scalar field from deformation system
    // IMPORTANT: The scalar field is already computed by S3SlicerDeformation
    // It represents VIRTUAL heights (what heights would be if mesh were rotated)
    log::info!("Step 3/5: Using virtual scalar field from deformation...");

    let scalar_field_values = deformation.get_scalar_field();
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

    // Step 5: Convert layers for toolpath generation
    log::info!("Step 5/5: Preparing layers for toolpath generation...");

    // Convert to standard Layer format
    let layers: Vec<Layer> = curved_layers.iter()
        .map(|cl| cl.to_layer())
        .collect();

    log::info!("=== S3-Slicer Pipeline Complete ===");
    let deformation_type = if config.use_asap_deformation {
        "ASAP deformation"
    } else {
        "scale-controlled deformation"
    };
    log::info!("Generated {} layers using {} (virtual deformation)", layers.len(), deformation_type);
    log::info!("Toolpaths are in ORIGINAL mesh space (no inverse mapping needed)");

    // Store scalar field values for compatibility
    let original_scalar_field_values = deformation.get_scalar_field().to_vec();

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
