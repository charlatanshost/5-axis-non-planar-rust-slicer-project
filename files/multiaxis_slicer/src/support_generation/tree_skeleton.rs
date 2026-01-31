// Tree skeleton generation for minimal support structures
// Based on SIGGRAPH Asia 2022: Support Generation for Curved RoboFDM

use crate::geometry::{Point3D, Vector3D};
use crate::support_generation::overhang_detection::OverhangFace;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// Node in the support tree
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupportNode {
    pub id: usize,
    pub position: Point3D,
    pub radius: f64,        // Support column radius
    pub parent: Option<usize>, // Parent node (closer to platform)
    pub children: Vec<usize>,  // Child nodes (closer to model)
}

/// Tree-based support skeleton
#[derive(Debug, Clone)]
pub struct SupportTree {
    pub nodes: Vec<SupportNode>,
    pub root_nodes: Vec<usize>, // Nodes touching the platform
    pub contact_nodes: Vec<usize>, // Nodes touching the model
}

/// Configuration for tree generation
#[derive(Debug, Clone)]
pub struct TreeConfig {
    /// Minimum support radius (mm)
    pub min_radius: f64,

    /// Maximum support radius (mm)
    pub max_radius: f64,

    /// Maximum branch angle from vertical (degrees)
    pub max_branch_angle: f64,

    /// Distance to merge nearby branches (mm)
    pub merge_distance: f64,

    /// Step size for tree growth (mm)
    pub growth_step: f64,
}

impl Default for TreeConfig {
    fn default() -> Self {
        Self {
            min_radius: 0.8,
            max_radius: 3.0,
            max_branch_angle: 60.0,
            merge_distance: 5.0,
            growth_step: 2.0,
        }
    }
}

/// Generate support tree skeleton from overhang faces
pub fn generate_tree_skeleton(
    overhang_faces: &[OverhangFace],
    platform_z: f64,
    config: &TreeConfig,
) -> SupportTree {
    let mut nodes = Vec::new();
    let mut node_id_counter = 0;

    // Group overhangs by clusters
    let clusters = cluster_by_proximity(overhang_faces, config.merge_distance);

    let mut root_nodes = Vec::new();
    let mut contact_nodes = Vec::new();

    for cluster in clusters {
        // Find center of cluster
        let cluster_center = calculate_cluster_center(&cluster);

        // Create contact node at overhang location
        let contact_node = SupportNode {
            id: node_id_counter,
            position: cluster_center,
            radius: config.min_radius,
            parent: None,
            children: Vec::new(),
        };

        contact_nodes.push(node_id_counter);
        nodes.push(contact_node);
        node_id_counter += 1;

        // Grow tree downward to platform
        let tree_path = grow_branch_to_platform(
            cluster_center,
            platform_z,
            config,
            &mut node_id_counter,
        );

        // Add nodes from this branch
        for (i, mut node) in tree_path.into_iter().enumerate() {
            if i > 0 {
                // Link to parent (previous node in path)
                let parent_idx = nodes.len() - 1;
                node.parent = Some(parent_idx);
                nodes[parent_idx].children.push(node.id);
            } else {
                // First node connects to contact point
                let contact_idx = contact_nodes[contact_nodes.len() - 1];
                node.parent = Some(contact_idx);
                nodes[contact_idx].children.push(node.id);
            }

            // Check if this is a root node (touches platform)
            if node.position.z <= platform_z + 0.1 {
                root_nodes.push(node.id);
            }

            nodes.push(node);
        }
    }

    // Merge nearby branches for efficiency
    merge_close_branches(&mut nodes, config.merge_distance);

    SupportTree {
        nodes,
        root_nodes,
        contact_nodes,
    }
}

/// Grow a single branch from overhang point to platform
fn grow_branch_to_platform(
    start: Point3D,
    platform_z: f64,
    config: &TreeConfig,
    node_id: &mut usize,
) -> Vec<SupportNode> {
    let mut branch = Vec::new();
    let mut current_pos = start;

    // Primary direction is downward
    let down = Vector3D::new(0.0, 0.0, -1.0);

    while current_pos.z > platform_z {
        // Move downward with slight random offset for natural look
        let offset_x = 0.0; // Could add small random offset
        let offset_y = 0.0;

        let next_z = (current_pos.z - config.growth_step).max(platform_z);
        let next_pos = Point3D::new(
            current_pos.x + offset_x,
            current_pos.y + offset_y,
            next_z,
        );

        // Calculate radius based on height (thicker at bottom)
        let height_ratio = (platform_z - next_pos.z).abs() / (platform_z - start.z).abs();
        let radius = config.min_radius + (config.max_radius - config.min_radius) * height_ratio;

        let node = SupportNode {
            id: *node_id,
            position: next_pos,
            radius,
            parent: None, // Will be set by caller
            children: Vec::new(),
        };

        *node_id += 1;
        branch.push(node);

        current_pos = next_pos;
    }

    branch
}

/// Cluster overhang faces by proximity
fn cluster_by_proximity(faces: &[OverhangFace], distance: f64) -> Vec<Vec<OverhangFace>> {
    if faces.is_empty() {
        return Vec::new();
    }

    let mut clusters = Vec::new();
    let mut assigned = vec![false; faces.len()];

    for i in 0..faces.len() {
        if assigned[i] {
            continue;
        }

        let mut cluster = vec![faces[i].clone()];
        assigned[i] = true;

        // Find nearby faces
        for j in (i + 1)..faces.len() {
            if assigned[j] {
                continue;
            }

            let is_close = cluster.iter().any(|face| {
                (face.center - faces[j].center).norm() < distance
            });

            if is_close {
                cluster.push(faces[j].clone());
                assigned[j] = true;
            }
        }

        clusters.push(cluster);
    }

    clusters
}

/// Calculate center point of a cluster
fn calculate_cluster_center(cluster: &[OverhangFace]) -> Point3D {
    if cluster.is_empty() {
        return Point3D::new(0.0, 0.0, 0.0);
    }

    let sum = cluster.iter().fold(
        Point3D::new(0.0, 0.0, 0.0),
        |acc, face| Point3D::new(
            acc.x + face.center.x,
            acc.y + face.center.y,
            acc.z + face.center.z,
        ),
    );

    Point3D::new(
        sum.x / cluster.len() as f64,
        sum.y / cluster.len() as f64,
        sum.z / cluster.len() as f64,
    )
}

/// Merge branches that are close together
fn merge_close_branches(nodes: &mut Vec<SupportNode>, merge_distance: f64) {
    // Two-pass approach to avoid borrow checker issues
    // Pass 1: Identify merge operations
    // Pass 2: Apply merge operations

    let mut merged_ids: HashSet<usize> = HashSet::new();
    let mut merge_operations: Vec<(usize, usize)> = Vec::new(); // (target_id, source_id) pairs

    // Pass 1: Identify which nodes should be merged
    for i in 0..nodes.len() {
        if merged_ids.contains(&i) {
            continue;
        }

        for j in (i + 1)..nodes.len() {
            if merged_ids.contains(&j) {
                continue;
            }

            // Check if nodes are at similar height and close in XY
            let z_diff = (nodes[i].position.z - nodes[j].position.z).abs();
            if z_diff > 1.0 {
                continue;
            }

            let xy_dist = ((nodes[i].position.x - nodes[j].position.x).powi(2)
                + (nodes[i].position.y - nodes[j].position.y).powi(2))
            .sqrt();

            if xy_dist < merge_distance {
                // Mark this merge operation
                merge_operations.push((i, j));
                merged_ids.insert(j);
            }
        }
    }

    // Pass 2: Apply merge operations
    for (target_id, source_id) in merge_operations {
        // Clone the children to avoid borrow issues
        let source_children = nodes[source_id].children.clone();
        let source_radius = nodes[source_id].radius;

        // Redirect children of source to target
        for child_id in source_children.iter() {
            if *child_id < nodes.len() {
                nodes[*child_id].parent = Some(target_id);
            }
        }

        // Merge children lists
        nodes[target_id].children.extend(source_children);

        // Update radius (take maximum)
        nodes[target_id].radius = nodes[target_id].radius.max(source_radius);
    }

    // Note: Merged nodes are left in the tree but orphaned
    // A more sophisticated implementation would remove them and reindex
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cluster_center() {
        let faces = vec![
            OverhangFace {
                triangle_idx: 0,
                center: Point3D::new(0.0, 0.0, 10.0),
                normal: Vector3D::new(0.0, 0.0, -1.0),
                overhang_angle: 135.0,
                severity: 0.5,
                layer_idx: 0,
            },
            OverhangFace {
                triangle_idx: 1,
                center: Point3D::new(2.0, 0.0, 10.0),
                normal: Vector3D::new(0.0, 0.0, -1.0),
                overhang_angle: 135.0,
                severity: 0.5,
                layer_idx: 0,
            },
        ];

        let center = calculate_cluster_center(&faces);
        assert!((center.x - 1.0).abs() < 0.001);
        assert!((center.z - 10.0).abs() < 0.001);
    }
}
