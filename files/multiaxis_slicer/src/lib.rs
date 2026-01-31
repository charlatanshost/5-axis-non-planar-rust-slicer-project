// Core modules for 5-axis non-planar slicing
pub mod geometry;
pub mod mesh;
pub mod slicing;
pub mod toolpath;
pub mod toolpath_patterns;
pub mod collision;
pub mod gcode;
pub mod centroidal_axis;
pub mod ruled_surface;
pub mod singularity;
pub mod motion_planning;
pub mod support_generation;
pub mod s3_slicer;
pub mod gui;

// Re-export commonly used types
pub use geometry::{Point3D, Vector3D, Plane, Triangle};
pub use mesh::{Mesh, MeshError};
pub use slicing::{Slicer, SlicingConfig, Layer};
pub use toolpath::{Toolpath, ToolpathGenerator};

/// Main result type for the slicer
pub type Result<T> = std::result::Result<T, Error>;

/// Error types for the slicer
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Mesh error: {0}")]
    Mesh(#[from] MeshError),
    
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    
    #[error("Slicing error: {0}")]
    Slicing(String),
    
    #[error("Collision detected at waypoint {0}")]
    Collision(usize),
    
    #[error("Singularity detected at configuration: {0:?}")]
    Singularity(Vec<f64>),
}
