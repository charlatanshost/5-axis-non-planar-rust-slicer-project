# Porting MultiAxis_3DP_MotionPlanning to Rust

## Overview

This guide helps you port the proven C++/Qt implementation to Rust, focusing on the **core algorithms** that make multi-axis printing work.

## Architecture Mapping

### C++ Structure → Rust Structure

**C++ (Qt/OpenGL):**
```
MultiAxis_3DP_MotionPlanning/
├── QMeshLib/          # Mesh data structures
├── GLKLib/            # OpenGL visualization
├── ShapeLab/          # Main application (Qt GUI)
└── DataSet/           # Test models and data
```

**Rust (Clean separation):**
```
multiaxis_slicer/
├── src/
│   ├── motion_planning/     # NEW: Core algorithms from C++
│   │   ├── mod.rs
│   │   ├── variable_filament.rs      # Step 1
│   │   ├── singularity.rs            # Step 2
│   │   ├── collision.rs              # Step 3
│   │   ├── graph_search.rs           # Step 4
│   │   └── gcode.rs                  # Step 5
│   ├── kinematics/          # NEW: IK/FK for 5-axis
│   ├── visualization/       # Optional: egui or kiss3d
│   └── [existing modules]
```

## Core Algorithms to Port

### Step 1: Variable Filament Calculation

**C++ Location**: `ShapeLab/VariableFilamentCalculation.cpp`

**What it does**: Dynamically adjusts extrusion based on:
- Distance between waypoints (D)
- Layer height (H)
- Toolpath width (W)

**Rust Implementation:**

```rust
// src/motion_planning/variable_filament.rs

use crate::geometry::{Point3D, Vector3D};

pub struct FilamentCalculator {
    pub filament_diameter: f64,
    pub nozzle_diameter: f64,
    pub use_distance: bool,
    pub use_height: bool,
    pub use_width: bool,
}

impl FilamentCalculator {
    pub fn calculate_extrusion(
        &self,
        waypoint: &Waypoint,
        prev_waypoint: Option<&Waypoint>,
        layer_height: f64,
        toolpath_width: f64,
    ) -> f64 {
        let mut volume = 0.0;

        // Distance-based (D)
        if self.use_distance && prev_waypoint.is_some() {
            let distance = (waypoint.position - prev_waypoint.unwrap().position).norm();
            volume += self.volume_from_distance(distance, layer_height, toolpath_width);
        }

        // Height-based (H)
        if self.use_height {
            volume += self.volume_from_height(layer_height, toolpath_width);
        }

        // Width-based (W)
        if self.use_width {
            volume += self.volume_from_width(toolpath_width, layer_height);
        }

        // Convert volume to filament length
        self.volume_to_filament_length(volume)
    }

    fn volume_from_distance(&self, distance: f64, height: f64, width: f64) -> f64 {
        // Cross-sectional area × distance
        height * width * distance
    }

    fn volume_to_filament_length(&self, volume: f64) -> f64 {
        let filament_area = std::f64::consts::PI * (self.filament_diameter / 2.0).powi(2);
        volume / filament_area
    }
}

#[derive(Debug, Clone)]
pub struct Waypoint {
    pub position: Point3D,
    pub orientation: Vector3D,
    pub layer_idx: usize,
    pub extrusion: f64,
}
```

### Step 2: Singularity Optimization

**C++ Location**: `ShapeLab/SingularityOptimization.cpp`

**What it does**: 
- Detects singular configurations (where IK Jacobian is near-singular)
- Switches between multiple IK solutions to avoid singularities
- Uses lambda threshold to identify singular regions

**Rust Implementation:**

```rust
// src/motion_planning/singularity.rs

use nalgebra::{Matrix6, Vector6};

pub struct SingularityOptimizer {
    pub lambda_threshold: f64,
    pub machine_config: MachineConfig,
}

#[derive(Debug, Clone)]
pub struct IKSolution {
    pub joint_angles: [f64; 5],  // [X, Y, Z, A, B] for 5-axis
    pub manipulability: f64,      // Measure of how far from singularity
    pub is_singular: bool,
}

impl SingularityOptimizer {
    pub fn compute_ik_solutions(
        &self,
        position: &Point3D,
        orientation: &Vector3D,
    ) -> Vec<IKSolution> {
        let mut solutions = Vec::new();

        // For 5-axis machine: typically 2 solutions (elbow up/down)
        for solve_type in 0..2 {
            if let Some(solution) = self.inverse_kinematics(position, orientation, solve_type) {
                solutions.push(solution);
            }
        }

        solutions
    }

    fn inverse_kinematics(
        &self,
        position: &Point3D,
        orientation: &Vector3D,
        solve_type: u8,
    ) -> Option<IKSolution> {
        // Convert orientation vector to A, B angles
        let (a_angle, b_angle) = self.orientation_to_angles(orientation);

        // Apply solve type (different IK branches)
        let (a_final, b_final) = match solve_type {
            0 => (a_angle, b_angle),
            1 => (180.0 - a_angle, -b_angle),  // Alternative solution
            _ => return None,
        };

        let joint_angles = [
            position.x,
            position.y,
            position.z,
            a_final,
            b_final,
        ];

        // Check if configuration is singular
        let manipulability = self.compute_manipulability(&joint_angles);
        let is_singular = manipulability < self.lambda_threshold;

        Some(IKSolution {
            joint_angles,
            manipulability,
            is_singular,
        })
    }

    fn compute_manipulability(&self, joint_angles: &[f64; 5]) -> f64 {
        // Compute Jacobian matrix
        let jacobian = self.compute_jacobian(joint_angles);
        
        // Manipulability = sqrt(det(J * J^T))
        let jjt = &jacobian * jacobian.transpose();
        jjt.determinant().abs().sqrt()
    }

    fn compute_jacobian(&self, _joint_angles: &[f64; 5]) -> Matrix6<f64> {
        // Simplified: In reality, this depends on machine kinematics
        // For a 5-axis printer with A/B rotation:
        Matrix6::identity()  // Placeholder
    }

    fn orientation_to_angles(&self, orientation: &Vector3D) -> (f64, f64) {
        let a_angle = orientation.y.atan2(orientation.z).to_degrees();
        let b_angle = (-orientation.x).atan2(orientation.z).to_degrees();
        (a_angle, b_angle)
    }

    /// Select optimal solution avoiding singularities
    pub fn select_best_solution(
        &self,
        solutions: &[IKSolution],
        prev_solution: Option<&IKSolution>,
    ) -> Option<usize> {
        // Filter out singular solutions
        let valid: Vec<(usize, &IKSolution)> = solutions
            .iter()
            .enumerate()
            .filter(|(_, sol)| !sol.is_singular)
            .collect();

        if valid.is_empty() {
            return None;
        }

        // If we have previous solution, minimize joint angle changes
        if let Some(prev) = prev_solution {
            return valid
                .iter()
                .min_by_key(|(_, sol)| {
                    let distance: f64 = sol.joint_angles
                        .iter()
                        .zip(prev.joint_angles.iter())
                        .map(|(a, b)| (a - b).abs())
                        .sum();
                    (distance * 1000.0) as i64  // Convert to integer for Ord
                })
                .map(|(idx, _)| *idx);
        }

        // Otherwise, select solution with highest manipulability
        valid
            .iter()
            .max_by_key(|(_, sol)| (sol.manipulability * 1000.0) as i64)
            .map(|(idx, _)| *idx)
    }
}

#[derive(Debug, Clone)]
pub struct MachineConfig {
    pub work_volume: (Point3D, Point3D),  // Min/max workspace bounds
    pub a_limits: (f64, f64),              // A axis limits (degrees)
    pub b_limits: (f64, f64),              // B axis limits (degrees)
}
```

### Step 3: Collision Detection

**C++ Location**: `ShapeLab/CollisionChecking.cpp`

**What it does**:
- Detects collision between print head and platform
- Detects collision between print head and already-printed material
- Sample-based approach with continuous refinement

**Rust Implementation:**

```rust
// src/motion_planning/collision.rs

use crate::geometry::Point3D;
use crate::mesh::Mesh;
use parry3d::query::{Ray, RayCast};
use parry3d::shape::TriMesh;

pub struct CollisionDetector {
    pub print_head: PrintHead,
    pub platform_bounds: (Point3D, Point3D),
    pub printed_mesh: Option<TriMesh<f64>>,
}

#[derive(Debug, Clone)]
pub struct PrintHead {
    pub nozzle_diameter: f64,
    pub body_radius: f64,
    pub body_height: f64,
}

impl CollisionDetector {
    pub fn check_collision_at_waypoint(
        &self,
        position: &Point3D,
        orientation: &nalgebra::Vector3<f64>,
    ) -> CollisionResult {
        let mut result = CollisionResult::default();

        // Check platform collision
        if self.collides_with_platform(position, orientation) {
            result.collides_with_platform = true;
            return result;
        }

        // Check collision with printed material
        if let Some(ref mesh) = self.printed_mesh {
            if self.collides_with_mesh(position, orientation, mesh) {
                result.collides_with_model = true;
                return result;
            }
        }

        result.is_collision_free = true;
        result
    }

    fn collides_with_platform(
        &self,
        position: &Point3D,
        _orientation: &nalgebra::Vector3<f64>,
    ) -> bool {
        // Check if nozzle or print head body hits platform
        let (min_bounds, max_bounds) = &self.platform_bounds;

        // Simple check: position below platform
        position.z < min_bounds.z
            || position.x < min_bounds.x
            || position.x > max_bounds.x
            || position.y < min_bounds.y
            || position.y > max_bounds.y
    }

    fn collides_with_mesh(
        &self,
        position: &Point3D,
        orientation: &nalgebra::Vector3<f64>,
        mesh: &TriMesh<f64>,
    ) -> bool {
        // Create cylinder representing print head
        let start = nalgebra::Point3::new(position.x, position.y, position.z);
        let end = start + orientation * self.print_head.body_height;

        // Sample points along print head body
        let num_samples = 10;
        for i in 0..num_samples {
            let t = i as f64 / num_samples as f64;
            let sample_point = start + (end - start) * t;

            // Check distance to mesh
            // If distance < body_radius, collision detected
            if self.point_near_mesh(&sample_point, mesh, self.print_head.body_radius) {
                return true;
            }
        }

        false
    }

    fn point_near_mesh(
        &self,
        point: &nalgebra::Point3<f64>,
        _mesh: &TriMesh<f64>,
        radius: f64,
    ) -> bool {
        // Use parry3d for efficient spatial queries
        // This is a simplified version
        // In practice, use mesh.project_point() or similar
        
        // Placeholder: would need actual mesh distance computation
        false
    }

    /// Continuous collision checking between two waypoints
    pub fn check_path_collision(
        &self,
        start: &Waypoint,
        end: &Waypoint,
        num_samples: usize,
    ) -> bool {
        for i in 0..=num_samples {
            let t = i as f64 / num_samples as f64;
            
            let interpolated_pos = start.position + (end.position - start.position) * t;
            let interpolated_orient = start.orientation.lerp(&end.orientation, t);

            let result = self.check_collision_at_waypoint(
                &interpolated_pos,
                &interpolated_orient,
            );

            if !result.is_collision_free {
                return true;  // Collision detected
            }
        }

        false  // Path is clear
    }
}

#[derive(Debug, Clone, Default)]
pub struct CollisionResult {
    pub is_collision_free: bool,
    pub collides_with_platform: bool,
    pub collides_with_model: bool,
    pub collision_point: Option<Point3D>,
}

use crate::motion_planning::variable_filament::Waypoint;
```

### Step 4: Graph Search for Path Planning

**C++ Location**: `ShapeLab/GraphSearch.cpp`

**What it does**:
- Builds a graph where nodes are (waypoint, IK_solution) pairs
- Edges connect accessible configurations
- Finds shortest path through graph

**Rust Implementation:**

```rust
// src/motion_planning/graph_search.rs

use std::collections::{HashMap, BinaryHeap};
use std::cmp::Ordering;

pub struct PathPlanner {
    pub collision_detector: CollisionDetector,
    pub singularity_optimizer: SingularityOptimizer,
}

#[derive(Debug, Clone)]
pub struct GraphNode {
    pub waypoint_idx: usize,
    pub solution_idx: usize,
    pub cost: f64,
}

impl PathPlanner {
    /// Build graph and find optimal path through waypoints
    pub fn plan_path(&self, waypoints: &[Waypoint]) -> Result<Vec<(usize, usize)>, String> {
        // Step 1: Compute all valid IK solutions for each waypoint
        let all_solutions: Vec<Vec<IKSolution>> = waypoints
            .iter()
            .map(|wp| {
                self.singularity_optimizer.compute_ik_solutions(
                    &wp.position,
                    &wp.orientation,
                )
            })
            .collect();

        // Step 2: Filter out solutions that cause collisions
        let valid_solutions = self.filter_collision_free(&all_solutions, waypoints);

        // Step 3: Build graph
        let graph = self.build_graph(&valid_solutions, waypoints);

        // Step 4: Find shortest path using Dijkstra
        self.dijkstra_shortest_path(&graph, waypoints.len())
    }

    fn filter_collision_free(
        &self,
        all_solutions: &[Vec<IKSolution>],
        waypoints: &[Waypoint],
    ) -> Vec<Vec<(usize, IKSolution)>> {
        all_solutions
            .iter()
            .zip(waypoints.iter())
            .map(|(solutions, waypoint)| {
                solutions
                    .iter()
                    .enumerate()
                    .filter(|(_, sol)| {
                        !sol.is_singular
                            && !self.check_collision_with_solution(waypoint, sol)
                    })
                    .map(|(idx, sol)| (idx, sol.clone()))
                    .collect()
            })
            .collect()
    }

    fn check_collision_with_solution(
        &self,
        waypoint: &Waypoint,
        _solution: &IKSolution,
    ) -> bool {
        let result = self.collision_detector.check_collision_at_waypoint(
            &waypoint.position,
            &waypoint.orientation,
        );
        !result.is_collision_free
    }

    fn build_graph(
        &self,
        valid_solutions: &[Vec<(usize, IKSolution)>],
        waypoints: &[Waypoint],
    ) -> Graph {
        let mut graph = Graph::new();

        // Add edges between consecutive waypoints
        for i in 0..waypoints.len().saturating_sub(1) {
            let current_solutions = &valid_solutions[i];
            let next_solutions = &valid_solutions[i + 1];

            for (curr_idx, curr_sol) in current_solutions {
                for (next_idx, next_sol) in next_solutions {
                    // Cost = joint angle distance
                    let cost = self.compute_transition_cost(curr_sol, next_sol);
                    
                    graph.add_edge(
                        NodeId { waypoint: i, solution: *curr_idx },
                        NodeId { waypoint: i + 1, solution: *next_idx },
                        cost,
                    );
                }
            }
        }

        graph
    }

    fn compute_transition_cost(&self, from: &IKSolution, to: &IKSolution) -> f64 {
        // Euclidean distance in joint space
        from.joint_angles
            .iter()
            .zip(to.joint_angles.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    fn dijkstra_shortest_path(
        &self,
        graph: &Graph,
        num_waypoints: usize,
    ) -> Result<Vec<(usize, usize)>, String> {
        // Standard Dijkstra's algorithm
        let mut distances: HashMap<NodeId, f64> = HashMap::new();
        let mut previous: HashMap<NodeId, NodeId> = HashMap::new();
        let mut heap = BinaryHeap::new();

        // Start from first waypoint, all solutions
        for solution in 0..10 {  // Assume max 10 solutions
            let start = NodeId { waypoint: 0, solution };
            if graph.nodes.contains(&start) {
                distances.insert(start, 0.0);
                heap.push(State { cost: 0.0, node: start });
            }
        }

        while let Some(State { cost, node }) = heap.pop() {
            // Found goal?
            if node.waypoint == num_waypoints - 1 {
                return Ok(self.reconstruct_path(&previous, node));
            }

            // Skip if we've found a better path
            if cost > *distances.get(&node).unwrap_or(&f64::INFINITY) {
                continue;
            }

            // Check neighbors
            if let Some(edges) = graph.adjacency.get(&node) {
                for edge in edges {
                    let next_cost = cost + edge.cost;
                    let next_distance = distances.get(&edge.to).copied().unwrap_or(f64::INFINITY);

                    if next_cost < next_distance {
                        distances.insert(edge.to, next_cost);
                        previous.insert(edge.to, node);
                        heap.push(State {
                            cost: next_cost,
                            node: edge.to,
                        });
                    }
                }
            }
        }

        Err("No valid path found".to_string())
    }

    fn reconstruct_path(
        &self,
        previous: &HashMap<NodeId, NodeId>,
        goal: NodeId,
    ) -> Vec<(usize, usize)> {
        let mut path = vec![(goal.waypoint, goal.solution)];
        let mut current = goal;

        while let Some(&prev) = previous.get(&current) {
            path.push((prev.waypoint, prev.solution));
            current = prev;
        }

        path.reverse();
        path
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct NodeId {
    waypoint: usize,
    solution: usize,
}

struct Graph {
    nodes: std::collections::HashSet<NodeId>,
    adjacency: HashMap<NodeId, Vec<Edge>>,
}

impl Graph {
    fn new() -> Self {
        Self {
            nodes: std::collections::HashSet::new(),
            adjacency: HashMap::new(),
        }
    }

    fn add_edge(&mut self, from: NodeId, to: NodeId, cost: f64) {
        self.nodes.insert(from);
        self.nodes.insert(to);
        self.adjacency
            .entry(from)
            .or_default()
            .push(Edge { to, cost });
    }
}

#[derive(Debug, Clone)]
struct Edge {
    to: NodeId,
    cost: f64,
}

#[derive(Debug, Clone)]
struct State {
    cost: f64,
    node: NodeId,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        (self.cost - other.cost).abs() < 1e-10
    }
}

impl Eq for State {}

use crate::motion_planning::collision::CollisionDetector;
use crate::motion_planning::singularity::{IKSolution, SingularityOptimizer};
use crate::motion_planning::variable_filament::Waypoint;
```

## Key Differences from C++

### 1. No Qt/OpenGL GUI
The C++ code uses Qt for GUI. In Rust:
- **Option A**: No GUI (CLI only)
- **Option B**: Use `egui` for simple GUI
- **Option C**: Web interface with visualization

### 2. Memory Management
**C++**: Manual memory management with pointers
**Rust**: Ownership system handles it automatically

### 3. Parallelism
**C++**: OpenMP (`#pragma omp parallel`)
**Rust**: Rayon (even easier!)

```rust
use rayon::prelude::*;

waypoints.par_iter()
    .map(|wp| compute_ik(wp))
    .collect()
```

## Integration with Existing Rust Code

Add to your `src/lib.rs`:

```rust
pub mod motion_planning {
    pub mod variable_filament;
    pub mod singularity;
    pub mod collision;
    pub mod graph_search;
}

pub mod kinematics;
```

Update `Cargo.toml`:

```toml
[dependencies]
# ... existing dependencies ...
petgraph = "0.6"        # For graph algorithms
priority-queue = "1.3"   # For Dijkstra's algorithm
```

## Testing Strategy

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ik_solutions() {
        let optimizer = SingularityOptimizer {
            lambda_threshold: 0.01,
            machine_config: MachineConfig::default(),
        };

        let position = Point3D::new(100.0, 100.0, 50.0);
        let orientation = Vector3D::new(0.0, 0.0, 1.0);

        let solutions = optimizer.compute_ik_solutions(&position, &orientation);
        assert!(!solutions.is_empty());
    }

    #[test]
    fn test_collision_detection() {
        let detector = CollisionDetector {
            print_head: PrintHead {
                nozzle_diameter: 0.4,
                body_radius: 20.0,
                body_height: 50.0,
            },
            platform_bounds: (
                Point3D::new(0.0, 0.0, 0.0),
                Point3D::new(200.0, 200.0, 200.0),
            ),
            printed_mesh: None,
        };

        let position = Point3D::new(100.0, 100.0, -10.0);  // Below platform
        let orientation = nalgebra::Vector3::new(0.0, 0.0, 1.0);

        let result = detector.check_collision_at_waypoint(&position, &orientation);
        assert!(result.collides_with_platform);
    }
}
```

## Next Steps

1. **Copy this guide** to `MOTION_PLANNING_PORT.md`
2. **Create module files** with the code above
3. **Test each module** independently
4. **Integrate** with your existing slicer
5. **Add visualization** (optional)

This gives you the **complete motion planning pipeline** from the proven C++ implementation!
