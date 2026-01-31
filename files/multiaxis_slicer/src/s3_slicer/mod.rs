// S3-Slicer module
// Curved layer slicing based on SIGGRAPH Asia 2022 Best Paper
// "S3-Slicer: A General Slicing Framework for Multi-Axis 3D Printing"
//
// PROPER ALGORITHM FLOW:
// 1. Optimize quaternion field for fabrication objectives
// 2. Deform mesh using quaternion field
// 3. Compute scalar field (height) on deformed mesh
// 4. Extract isosurfaces (curved layers) from DEFORMED mesh
// 5. Curved layers represent the actual deformed geometry
// 6. Generate toolpaths from curved layers

pub mod scalar_field;
pub mod deformation;
pub mod deformation_v2;
pub mod quaternion_field;
pub mod isosurface;
pub mod pipeline;
pub mod heat_method;
pub mod asap_deformation;
pub mod tet_mesh;

pub use scalar_field::{ScalarField, ScalarFieldConfig, FieldType};
pub use deformation::{DeformationConfig, DeformedMesh, DeformationTransform, deform_mesh};
pub use deformation_v2::S3SlicerDeformation;
pub use quaternion_field::{QuaternionField, QuaternionFieldConfig, FabricationObjective};
pub use isosurface::{IsosurfaceExtractor, IsosurfaceConfig, CurvedLayer, extract_curved_layers};
pub use pipeline::{execute_s3_pipeline, S3PipelineConfig, S3PipelineResult, PipelineStats};
pub use heat_method::{compute_geodesic_distances, HeatMethodConfig, HeatMethodResult};
pub use asap_deformation::{AsapSolver, AsapConfig};
pub use tet_mesh::{TetMesh, Tetrahedron, MeshQuality};
