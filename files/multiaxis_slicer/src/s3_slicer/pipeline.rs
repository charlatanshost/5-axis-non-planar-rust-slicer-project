// Complete S3-Slicer pipeline implementation
// Follows the correct algorithm from the SIGGRAPH Asia 2022 paper

use crate::mesh::Mesh;
use crate::slicing::{Layer, Slicer, SlicingConfig};
use crate::geometry::{Point3D, Vector3D, LineSegment};
use crate::s3_slicer::{
    QuaternionField, QuaternionFieldConfig, FabricationObjective,
    S3SlicerDeformation, ScalarField, ScalarFieldConfig, FieldType,
    IsosurfaceConfig, extract_curved_layers, CurvedLayer,
    AsapSolver, AsapConfig,
    TetMesh, TetQuaternionField, TetAsapSolver, TetAsapConfig,
    TetScalarField, TetScalarFieldConfig, extract_tet_layers,
};
use crate::s3_slicer::tet_dijkstra_field::TetDijkstraField;
use crate::s3_slicer::s4_rotation_field::{S4RotationField, S4RotationConfig};
use crate::s3_slicer::tet_point_location::{TetSpatialGrid, interpolate_point};
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

    /// S4-style pipeline: Dijkstra distance from build plate, gradient-based
    /// rotation field, ASAP deformation, then planar slice of deformed mesh
    /// with barycentric untransform of toolpath points back to original space.
    /// Simpler than TetVolumetric (no marching tetrahedra), reuses standard slicer.
    S4Deform,
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

    /// ASAP solver max iterations
    pub asap_max_iterations: usize,

    /// ASAP convergence threshold
    pub asap_convergence_threshold: f64,

    /// Z-bias for Dijkstra edge weights in the S4 pipeline (0 = Euclidean, 1 = pure |ΔZ|).
    /// Only used by DeformationMethod::S4Deform.  Default 0.8 gives height-tracking layers.
    pub z_bias: f64,
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
            asap_max_iterations: 3,      // 3 iterations is usually enough for ARAP convergence
            asap_convergence_threshold: 1e-3,  // Higher threshold for faster termination
            z_bias: 0.8,  // Height-tracking Dijkstra by default
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

    // For S4Deform, use the S4-style pipeline (Dijkstra + deform + slice + untransform)
    if matches!(config.deformation_method, DeformationMethod::S4Deform) {
        return execute_s4_pipeline(original_mesh, config);
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
        DeformationMethod::S4Deform => {
            unreachable!("S4Deform is handled by execute_s4_pipeline()");
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
    log::info!("Generated {} layers using {:?} deformation method", layers.len(), config.deformation_method);
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

/// Deform a tet mesh by applying per-tet rotations directly to vertices.
///
/// For each vertex, computes a weighted average of rotations from all
/// tetrahedra containing that vertex (via incremental SLERP), then
/// applies the rotation relative to the mesh's bottom center (build plate anchor).
///
/// This is much simpler and more robust than the full ASAP solver — it won't
/// produce mesh collapse or massive numbers of inverted tets for small rotations.
/// The trade-off is that individual tets may not be perfectly rigid, but for
/// the S4 pipeline (deform → slice → untransform) this is fine.
pub fn deform_tet_mesh_direct(
    tet_mesh: &TetMesh,
    rotations: &[nalgebra::UnitQuaternion<f64>],
) -> TetMesh {
    let num_verts = tet_mesh.vertices.len();
    let mut new_vertices = Vec::with_capacity(num_verts);

    // Anchor at bottom center of mesh (build plate)
    let anchor = Point3D::new(
        (tet_mesh.bounds_min.x + tet_mesh.bounds_max.x) / 2.0,
        (tet_mesh.bounds_min.y + tet_mesh.bounds_max.y) / 2.0,
        tet_mesh.bounds_min.z,
    );

    // Height range for blending: bottom vertices get no rotation, top gets full rotation.
    // This keeps the build plate contact fixed and progressively deforms upward.
    let height_range = (tet_mesh.bounds_max.z - tet_mesh.bounds_min.z).max(1e-6);

    for vi in 0..num_verts {
        let tets = &tet_mesh.vertex_tets[vi];
        let v = &tet_mesh.vertices[vi];

        if tets.is_empty() {
            new_vertices.push(*v);
            continue;
        }

        // Height-based blend: 0 at bottom (no rotation) → 1 at top (full rotation)
        let height_param = ((v.z - tet_mesh.bounds_min.z) / height_range).clamp(0.0, 1.0);

        // Compute average rotation using incremental SLERP:
        // After k rotations, avg = correct uniform average of all k
        let mut avg_rot = nalgebra::UnitQuaternion::identity();
        let mut count = 0usize;
        for &ti in tets {
            if ti < rotations.len() {
                count += 1;
                let t = 1.0 / count as f64;
                avg_rot = avg_rot.slerp(&rotations[ti], t);
            }
        }

        // Scale rotation by height: bottom stays fixed, top gets full deformation
        let scaled_rot = nalgebra::UnitQuaternion::identity().slerp(&avg_rot, height_param);

        // Apply rotation relative to anchor
        let relative = Vector3D::new(v.x - anchor.x, v.y - anchor.y, v.z - anchor.z);
        let rotated = scaled_rot * relative;
        new_vertices.push(Point3D::new(
            anchor.x + rotated.x,
            anchor.y + rotated.y,
            anchor.z + rotated.z,
        ));
    }

    log::info!("  Direct rotation deform: {} vertices, anchor=({:.1}, {:.1}, {:.1}), height_range={:.1}",
        num_verts, anchor.x, anchor.y, anchor.z, height_range);

    TetMesh::new(new_vertices, tet_mesh.tets.clone())
}

/// Execute the S4-style pipeline
///
/// Uses Dijkstra distance from build plate for rotation direction, direct vertex
/// rotation deformation, then slices the deformed surface with the standard planar
/// slicer and untransforms each toolpath point back to original space via barycentric
/// interpolation.
///
/// Pipeline:
/// 1. Surface mesh → TetMesh (TetGen)
/// 2. TetDijkstraField (path lengths + gradients)
/// 3. S4RotationField (overhang-based rotations from gradients)
/// 4. Direct vertex rotation (apply per-tet rotations to vertices)
/// 5. Extract deformed surface → planar slice
/// 6. Untransform each contour point via barycentric coords
/// 7. Build CurvedLayers → standard Layers
pub fn execute_s4_pipeline(
    original_mesh: Mesh,
    config: S3PipelineConfig,
) -> S3PipelineResult {
    log::info!("=== S4-Style Pipeline (Dijkstra + Deform + Untransform) ===");

    // Step 1: Tetrahedralize
    log::info!("Step 1/7: Tetrahedralizing surface mesh...");
    let tet_mesh = match TetMesh::from_surface_mesh(&original_mesh) {
        Ok(mesh) => {
            let quality = mesh.check_quality();
            log::info!("  → TetMesh: {} vertices, {} tets, {} surface tris",
                quality.num_vertices, quality.num_tets, quality.num_surface_triangles);
            mesh
        }
        Err(e) => {
            log::error!("Tetrahedralization failed: {}", e);
            log::warn!("Falling back to VirtualScalarField pipeline");
            let fallback_config = S3PipelineConfig {
                deformation_method: DeformationMethod::VirtualScalarField,
                ..config
            };
            return execute_s3_pipeline(original_mesh, fallback_config);
        }
    };

    // Step 2: Dijkstra distance field
    log::info!("Step 2/7: Computing Dijkstra distance field (z_bias={:.2})...", config.z_bias);
    let dijkstra_field = TetDijkstraField::compute(&tet_mesh, config.z_bias);
    log::info!("  → Max distance: {:.2}, {} base tets",
        dijkstra_field.max_distance, dijkstra_field.base_tets.len());

    // Step 3: S4 rotation field
    log::info!("Step 3/7: Computing S4 rotation field...");
    let s4_config = S4RotationConfig {
        build_direction: Vector3D::new(0.0, 0.0, 1.0),
        overhang_threshold: config.overhang_threshold,
        max_rotation_degrees: config.max_rotation_degrees,
        z_bias: config.z_bias,
        smoothing_iterations: config.optimization_iterations / 2,
        smoothness_weight: config.smoothness_weight,
    };
    let rotation_field = S4RotationField::compute(&tet_mesh, &dijkstra_field, &s4_config);

    // Step 4: Direct vertex rotation (apply per-tet rotations to vertices)
    log::info!("Step 4/7: Applying direct vertex rotation deformation...");
    let deformed_tet_mesh = deform_tet_mesh_direct(&tet_mesh, &rotation_field.rotations);

    // Quality check
    let deformed_quality = deformed_tet_mesh.check_quality();
    let orig_extent = (tet_mesh.bounds_max - tet_mesh.bounds_min).norm();
    let deformed_extent = (deformed_tet_mesh.bounds_max - deformed_tet_mesh.bounds_min).norm();
    let extent_ratio = deformed_extent / orig_extent.max(1e-10);
    let inverted_ratio = deformed_quality.inverted_tets as f64 / tet_mesh.tets.len().max(1) as f64;

    log::info!("  → Extent ratio: {:.2}x, inverted: {:.1}%",
        extent_ratio, inverted_ratio * 100.0);

    if inverted_ratio > 0.5 {
        log::error!("Deformation quality too poor ({:.1}% inverted). Falling back to VirtualScalarField.",
            inverted_ratio * 100.0);
        let fallback_config = S3PipelineConfig {
            deformation_method: DeformationMethod::VirtualScalarField,
            ..config
        };
        return execute_s3_pipeline(original_mesh, fallback_config);
    }

    // Step 5: Deform original mesh surface through tet field, then slice planarly
    log::info!("Step 5/7: Deforming original surface through tet field and slicing...");
    let deformed_surface = crate::s3_slicer::tet_point_location::deform_surface_through_tets(
        &original_mesh,
        &tet_mesh,
        &deformed_tet_mesh,
    );
    log::info!("  → Deformed surface: {} triangles (original mesh topology)", deformed_surface.triangles.len());

    // Filter out spurious triangles (bad STL geometry with very long edges) that
    // create stray intersection lines during slicing. Compute median edge length
    // and remove triangles with any edge > 15× median.
    let deformed_surface = {
        let mut edge_lengths: Vec<f64> = deformed_surface.triangles.iter()
            .flat_map(|tri| {
                let e0 = ((tri.v1.x - tri.v0.x).powi(2) + (tri.v1.y - tri.v0.y).powi(2) + (tri.v1.z - tri.v0.z).powi(2)).sqrt();
                let e1 = ((tri.v2.x - tri.v1.x).powi(2) + (tri.v2.y - tri.v1.y).powi(2) + (tri.v2.z - tri.v1.z).powi(2)).sqrt();
                let e2 = ((tri.v0.x - tri.v2.x).powi(2) + (tri.v0.y - tri.v2.y).powi(2) + (tri.v0.z - tri.v2.z).powi(2)).sqrt();
                vec![e0, e1, e2]
            })
            .collect();
        edge_lengths.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median_edge = edge_lengths[edge_lengths.len() / 2];
        let max_allowed_edge = (median_edge * 15.0).max(5.0); // At least 5mm threshold

        let filtered: Vec<_> = deformed_surface.triangles.iter()
            .filter(|tri| {
                let e0 = ((tri.v1.x - tri.v0.x).powi(2) + (tri.v1.y - tri.v0.y).powi(2) + (tri.v1.z - tri.v0.z).powi(2)).sqrt();
                let e1 = ((tri.v2.x - tri.v1.x).powi(2) + (tri.v2.y - tri.v1.y).powi(2) + (tri.v2.z - tri.v1.z).powi(2)).sqrt();
                let e2 = ((tri.v0.x - tri.v2.x).powi(2) + (tri.v0.y - tri.v2.y).powi(2) + (tri.v0.z - tri.v2.z).powi(2)).sqrt();
                e0 <= max_allowed_edge && e1 <= max_allowed_edge && e2 <= max_allowed_edge
            })
            .cloned()
            .collect();
        let removed = deformed_surface.triangles.len() - filtered.len();
        if removed > 0 {
            log::info!("  → Filtered {} spurious triangles (median edge={:.2}mm, max allowed={:.2}mm)",
                removed, median_edge, max_allowed_edge);
        }
        Mesh::new(filtered).unwrap()
    };

    let slicing_config = SlicingConfig {
        layer_height: config.layer_height,
        ..SlicingConfig::default()
    };
    let slicer = Slicer::new(slicing_config);
    let planar_layers = match slicer.slice(&deformed_surface) {
        Ok(layers) => layers,
        Err(e) => {
            log::error!("Planar slicer failed: {}. Falling back to VirtualScalarField.", e);
            let fallback_config = S3PipelineConfig {
                deformation_method: DeformationMethod::VirtualScalarField,
                ..config
            };
            return execute_s3_pipeline(original_mesh, fallback_config);
        }
    };

    log::info!("  → Planar slicer produced {} layers", planar_layers.len());

    if planar_layers.is_empty() {
        log::error!("Planar slicer produced no layers. Falling back to VirtualScalarField.");
        let fallback_config = S3PipelineConfig {
            deformation_method: DeformationMethod::VirtualScalarField,
            ..config
        };
        return execute_s3_pipeline(original_mesh, fallback_config);
    }

    // Step 6: Untransform each contour point via barycentric interpolation
    // IMPORTANT: Preserve contour structure from the planar slicer! Do NOT dump all
    // segments into one Vec and re-chain — that causes cross-contour mixing where
    // segments from different contours connect because endpoints are within tolerance.
    log::info!("Step 6/7: Untransforming toolpath points to original space...");
    let deformed_grid = TetSpatialGrid::build(&deformed_tet_mesh);

    let mut layers: Vec<Layer> = Vec::new();
    let mut curved_layers: Vec<CurvedLayer> = Vec::new();
    let mut total_points = 0usize;
    let mut points_found = 0usize;
    let mut points_nearest = 0usize;

    // Bounds of original mesh for sanity checking untransformed points
    let orig_diag = (original_mesh.bounds_max - original_mesh.bounds_min).norm();
    let bounds_margin = orig_diag * 0.5; // Allow 50% outside original bounds

    for layer in &planar_layers {
        let mut untransformed_contours: Vec<crate::geometry::Contour> = Vec::new();
        let mut all_segments = Vec::new(); // for CurvedLayer compat
        let mut all_normals = Vec::new();

        for contour in &layer.contours {
            if contour.points.len() < 2 {
                continue;
            }

            let mut untransformed_points: Vec<Point3D> = Vec::new();

            for point in &contour.points {
                total_points += 1;

                // Find containing tet in DEFORMED mesh
                let loc = match deformed_grid.find_containing_tet(point, &deformed_tet_mesh) {
                    Some(r) => { points_found += 1; r }
                    None => { points_nearest += 1; deformed_grid.find_nearest_tet(point, &deformed_tet_mesh) }
                };

                // Apply barycentric coords in ORIGINAL tet mesh
                let tet = &tet_mesh.tets[loc.tet_index];
                let original_point = interpolate_point(
                    &loc.bary_coords,
                    &tet_mesh.vertices[tet.vertices[0]],
                    &tet_mesh.vertices[tet.vertices[1]],
                    &tet_mesh.vertices[tet.vertices[2]],
                    &tet_mesh.vertices[tet.vertices[3]],
                );

                // Sanity check: skip wildly out-of-bounds points (from inverted tets)
                let dist_from_center = (original_point - Point3D::new(
                    (original_mesh.bounds_min.x + original_mesh.bounds_max.x) / 2.0,
                    (original_mesh.bounds_min.y + original_mesh.bounds_max.y) / 2.0,
                    (original_mesh.bounds_min.z + original_mesh.bounds_max.z) / 2.0,
                )).norm();
                if dist_from_center > orig_diag + bounds_margin {
                    continue; // Skip this point — barycentric interpolation went bad
                }

                untransformed_points.push(original_point);
            }

            // Split contour at stray jumps: if consecutive points are too far apart,
            // break into separate sub-contours to avoid long stray lines
            let max_seg_len = config.layer_height * 20.0; // Allow generous moves but catch strays
            let max_seg_len_sq = max_seg_len * max_seg_len;

            let mut sub_contours: Vec<Vec<Point3D>> = Vec::new();
            let mut current_sub: Vec<Point3D> = Vec::new();

            for pt in &untransformed_points {
                if let Some(last) = current_sub.last() {
                    let dx = pt.x - last.x;
                    let dy = pt.y - last.y;
                    let dz = pt.z - last.z;
                    let dist_sq = dx * dx + dy * dy + dz * dz;
                    if dist_sq > max_seg_len_sq {
                        // Break: save current sub-contour and start new one
                        if current_sub.len() >= 2 {
                            sub_contours.push(std::mem::take(&mut current_sub));
                        } else {
                            current_sub.clear();
                        }
                    }
                }
                current_sub.push(*pt);
            }
            if current_sub.len() >= 2 {
                sub_contours.push(current_sub);
            }

            for sub in &sub_contours {
                for window in sub.windows(2) {
                    all_segments.push(LineSegment::new(window[0], window[1]));
                    all_normals.push(Vector3D::new(0.0, 0.0, 1.0));
                }
                untransformed_contours.push(crate::geometry::Contour {
                    points: sub.clone(),
                    closed: false, // Sub-contours from splits are open
                });
            }
        }

        if !untransformed_contours.is_empty() {
            let point_count: usize = untransformed_contours.iter().map(|c| c.points.len()).sum();
            let avg_z = untransformed_contours.iter()
                .flat_map(|c| c.points.iter())
                .map(|p| p.z)
                .sum::<f64>() / point_count.max(1) as f64;

            layers.push(Layer {
                z: avg_z,
                contours: untransformed_contours,
                layer_height: layer.layer_height,
            });

            curved_layers.push(CurvedLayer {
                z: avg_z,
                segments: all_segments,
                iso_value: layer.z,
                normals: all_normals,
            });
        }
    }

    log::info!("  → {} points: {} found in tet, {} nearest fallback",
        total_points, points_found, points_nearest);

    // Step 7: Summary (layers already built directly above)
    log::info!("Step 7/7: Finalizing layers...");
    let total_contours: usize = layers.iter().map(|l| l.contours.len()).sum();
    let total_layer_points: usize = layers.iter()
        .flat_map(|l| l.contours.iter())
        .map(|c| c.points.len())
        .sum();
    log::info!("  → {} layers, {} contours, {} points",
        layers.len(), total_contours, total_layer_points);

    // Build result with compatibility fields
    let deformed_surface_for_result = deformed_tet_mesh.to_surface_mesh();

    let num_triangles = original_mesh.triangles.len();
    let placeholder_qfield = QuaternionField {
        rotations: vec![nalgebra::UnitQuaternion::identity(); num_triangles],
        energy: rotation_field.energy,
        config: QuaternionFieldConfig {
            objective: config.objective,
            build_direction: Vector3D::new(0.0, 0.0, 1.0),
            optimization_iterations: config.optimization_iterations,
            smoothness_weight: config.smoothness_weight,
            objective_weight: 1.0,
            overhang_threshold: config.overhang_threshold,
            max_rotation_degrees: config.max_rotation_degrees,
        },
    };

    // Use lightweight placeholder — S4 has its own deformation, no need to run S3's
    let deformation = S3SlicerDeformation::placeholder(
        original_mesh.clone(),
        placeholder_qfield.clone(),
    );

    let height_range = if !curved_layers.is_empty() {
        let min_z = curved_layers.iter().map(|cl| cl.z).fold(f64::INFINITY, f64::min);
        let max_z = curved_layers.iter().map(|cl| cl.z).fold(f64::NEG_INFINITY, f64::max);
        max_z - min_z
    } else {
        original_mesh.bounds_max.z - original_mesh.bounds_min.z
    };

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

    log::info!("=== S4-Style Pipeline Complete ===");
    log::info!("Generated {} curved layers via deform+slice+untransform", layers.len());

    S3PipelineResult {
        original_mesh,
        deformed_mesh: deformed_surface_for_result,
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
