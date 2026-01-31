// Support generation module
// Automatic support structure generation for 5-axis printing

pub mod overhang_detection;
pub mod tree_skeleton;
pub mod support_toolpath;

pub use overhang_detection::{OverhangConfig, OverhangFace, OverhangAnalysis, detect_overhangs, cluster_overhangs};
pub use tree_skeleton::{SupportNode, SupportTree, TreeConfig, generate_tree_skeleton};
pub use support_toolpath::{SupportToolpathConfig, generate_support_toolpaths};

use crate::mesh::Mesh;
use crate::centroidal_axis::CentroidalAxis;
use crate::geometry::Point3D;

/// Complete support generation pipeline
pub struct SupportGenerator {
    pub overhang_config: OverhangConfig,
    pub tree_config: TreeConfig,
}

impl SupportGenerator {
    pub fn new() -> Self {
        Self {
            overhang_config: OverhangConfig::default(),
            tree_config: TreeConfig::default(),
        }
    }

    pub fn with_overhang_angle(mut self, angle: f64) -> Self {
        self.overhang_config.overhang_angle = angle;
        self
    }

    /// Generate support structures for a mesh
    pub fn generate(&self, mesh: &Mesh, centroidal_axis: Option<&CentroidalAxis>) -> SupportResult {
        // Step 1: Detect overhangs
        let overhang_analysis = detect_overhangs(mesh, &self.overhang_config, centroidal_axis);

        if !overhang_analysis.requires_support {
            return SupportResult {
                requires_support: false,
                tree: None,
                overhang_analysis,
                toolpaths: Vec::new(),
            };
        }

        // Step 2: Generate tree skeleton
        let platform_z = mesh.bounds_min.z;
        let tree = generate_tree_skeleton(
            &overhang_analysis.overhang_faces,
            platform_z,
            &self.tree_config,
        );

        SupportResult {
            requires_support: true,
            tree: Some(tree),
            overhang_analysis,
            toolpaths: Vec::new(),  // Toolpaths generated separately
        }
    }

    /// Generate toolpaths for existing support tree
    pub fn generate_toolpaths(&self, support_tree: &SupportTree) -> Vec<crate::toolpath::Toolpath> {
        let config = SupportToolpathConfig::default();
        generate_support_toolpaths(support_tree, &config)
    }
}

impl Default for SupportGenerator {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of support generation
#[derive(Debug, Clone)]
pub struct SupportResult {
    pub requires_support: bool,
    pub tree: Option<SupportTree>,
    pub overhang_analysis: OverhangAnalysis,
    pub toolpaths: Vec<crate::toolpath::Toolpath>,
}

impl SupportResult {
    /// Get statistics about generated supports
    pub fn stats(&self) -> SupportStats {
        SupportStats {
            num_overhangs: self.overhang_analysis.overhang_faces.len(),
            overhang_area: self.overhang_analysis.total_overhang_area,
            num_support_nodes: self.tree.as_ref().map(|t| t.nodes.len()).unwrap_or(0),
            num_contact_points: self.tree.as_ref().map(|t| t.contact_nodes.len()).unwrap_or(0),
        }
    }
}

/// Statistics about support structures
#[derive(Debug, Clone)]
pub struct SupportStats {
    pub num_overhangs: usize,
    pub overhang_area: f64,
    pub num_support_nodes: usize,
    pub num_contact_points: usize,
}
