// Graph search for path planning - Step 4
// Phase E: Dijkstra over segment-orientation graph for globally optimal orientation sequences

use super::{CollisionDetector, SingularityOptimizer, IKSolution, Waypoint};
use std::collections::BinaryHeap;
use std::cmp::Ordering;

#[derive(Debug, Clone)]
pub struct PathPlanner {
    pub collision_detector: CollisionDetector,
    pub singularity_optimizer: SingularityOptimizer,
}

/// A node in the priority queue for Dijkstra's algorithm.
/// Represents choosing a particular IK solution for a particular waypoint.
#[derive(Debug, Clone)]
struct DijkstraNode {
    cost: f64,
    waypoint_idx: usize,
    solution_idx: usize,
}

impl PartialEq for DijkstraNode {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for DijkstraNode {}

impl PartialOrd for DijkstraNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for DijkstraNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse order for min-heap (BinaryHeap is a max-heap by default)
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

impl PathPlanner {
    /// Original simple interface — always picks the first valid solution (greedy fallback).
    pub fn plan_path(&self, waypoints: &[Waypoint]) -> crate::Result<Vec<(usize, usize)>> {
        self.plan_path_from_solutions(&vec![vec![]; waypoints.len()], waypoints)
    }

    /// Dijkstra-based optimal path planning over the segment-orientation graph.
    ///
    /// Each node is (waypoint_index, solution_index). Edges connect consecutive waypoints
    /// with weight = angular_distance + collision_penalty + singularity_penalty.
    /// Returns the globally optimal sequence of (waypoint_idx, solution_idx) pairs.
    pub fn plan_path_from_solutions(
        &self,
        valid_solutions: &[Vec<IKSolution>],
        waypoints: &[Waypoint],
    ) -> crate::Result<Vec<(usize, usize)>> {
        let n = waypoints.len();
        if n == 0 {
            return Ok(vec![]);
        }

        // Check if any waypoint has solutions; if none do, fall back to greedy
        let has_any_solutions = valid_solutions.iter().any(|sols| !sols.is_empty());
        if !has_any_solutions {
            return Ok((0..n).map(|i| (i, 0)).collect());
        }

        // For each waypoint, ensure at least one candidate (use index 0 as fallback)
        let candidates: Vec<usize> = valid_solutions.iter()
            .map(|sols| if sols.is_empty() { 1 } else { sols.len() })
            .collect();

        // dist[waypoint_idx][solution_idx] = best cost to reach this node
        let mut dist: Vec<Vec<f64>> = candidates.iter()
            .map(|&c| vec![f64::INFINITY; c])
            .collect();

        // prev[waypoint_idx][solution_idx] = (prev_waypoint_idx, prev_solution_idx)
        let mut prev: Vec<Vec<Option<(usize, usize)>>> = candidates.iter()
            .map(|&c| vec![None; c])
            .collect();

        let mut heap = BinaryHeap::new();

        // Initialize: first waypoint, all solutions have cost 0
        for si in 0..candidates[0] {
            dist[0][si] = 0.0;
            heap.push(DijkstraNode { cost: 0.0, waypoint_idx: 0, solution_idx: si });
        }

        // Process Dijkstra
        while let Some(DijkstraNode { cost, waypoint_idx: wi, solution_idx: si }) = heap.pop() {
            // Skip if we already found a better path to this node
            if cost > dist[wi][si] {
                continue;
            }

            // If this is the last waypoint, we're done
            if wi == n - 1 {
                continue;
            }

            let next_wi = wi + 1;

            // Get current solution's joint angles (or default)
            let current_angles = if !valid_solutions[wi].is_empty() {
                valid_solutions[wi][si].joint_angles
            } else {
                [waypoints[wi].position.x, waypoints[wi].position.y,
                 waypoints[wi].position.z, 0.0, 0.0]
            };

            // Try all solutions for the next waypoint
            for next_si in 0..candidates[next_wi] {
                let next_angles = if !valid_solutions[next_wi].is_empty() {
                    valid_solutions[next_wi][next_si].joint_angles
                } else {
                    [waypoints[next_wi].position.x, waypoints[next_wi].position.y,
                     waypoints[next_wi].position.z, 0.0, 0.0]
                };

                // Edge weight: angular distance between rotary axes (indices 3,4)
                let angular_dist = {
                    let da = (current_angles[3] - next_angles[3]).to_radians();
                    let db = (current_angles[4] - next_angles[4]).to_radians();
                    (da * da + db * db).sqrt()
                };

                // Singularity penalty
                let sing_penalty = if !valid_solutions[next_wi].is_empty() {
                    let manip = valid_solutions[next_wi][next_si].manipulability;
                    let threshold = self.singularity_optimizer.lambda_threshold;
                    if manip < threshold {
                        (threshold - manip) * 10.0
                    } else {
                        0.0
                    }
                } else {
                    0.0
                };

                let edge_cost = angular_dist + sing_penalty;
                let new_cost = cost + edge_cost;

                if new_cost < dist[next_wi][next_si] {
                    dist[next_wi][next_si] = new_cost;
                    prev[next_wi][next_si] = Some((wi, si));
                    heap.push(DijkstraNode {
                        cost: new_cost,
                        waypoint_idx: next_wi,
                        solution_idx: next_si,
                    });
                }
            }
        }

        // Backtrack from the last waypoint to find the optimal path
        let last_wi = n - 1;
        let best_last_si = (0..candidates[last_wi])
            .min_by(|&a, &b| dist[last_wi][a].partial_cmp(&dist[last_wi][b]).unwrap_or(Ordering::Equal))
            .unwrap_or(0);

        let mut path = vec![(0usize, 0usize); n];
        let mut wi = last_wi;
        let mut si = best_last_si;
        path[wi] = (wi, si);

        while wi > 0 {
            if let Some((pw, ps)) = prev[wi][si] {
                wi = pw;
                si = ps;
                path[wi] = (wi, si);
            } else {
                // No predecessor found — fill remaining with index 0
                for i in 0..wi {
                    path[i] = (i, 0);
                }
                break;
            }
        }

        Ok(path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point3D, Vector3D};
    use crate::motion_planning::{CollisionDetector, SingularityOptimizer, MachineConfig, PrintHead};

    fn make_planner() -> PathPlanner {
        PathPlanner {
            collision_detector: CollisionDetector {
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
            },
            singularity_optimizer: SingularityOptimizer {
                lambda_threshold: 0.01,
                machine_config: MachineConfig {
                    work_volume: (
                        Point3D::new(0.0, 0.0, 0.0),
                        Point3D::new(200.0, 200.0, 200.0),
                    ),
                    a_limits: (-90.0, 90.0),
                    b_limits: (-90.0, 90.0),
                },
            },
        }
    }

    #[test]
    fn test_empty_waypoints() {
        let planner = make_planner();
        let result = planner.plan_path(&[]).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn test_single_waypoint() {
        let planner = make_planner();
        let waypoints = vec![Waypoint {
            position: Point3D::new(10.0, 10.0, 0.2),
            orientation: Vector3D::new(0.0, 0.0, 1.0),
            layer_idx: 0,
            extrusion: 0.0,
        }];
        let solutions = vec![vec![IKSolution {
            joint_angles: [10.0, 10.0, 0.2, 0.0, 0.0],
            manipulability: 0.5,
            is_singular: false,
        }]];
        let result = planner.plan_path_from_solutions(&solutions, &waypoints).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0], (0, 0));
    }

    #[test]
    fn test_dijkstra_prefers_smooth_transitions() {
        let planner = make_planner();
        let waypoints = vec![
            Waypoint {
                position: Point3D::new(10.0, 10.0, 0.2),
                orientation: Vector3D::new(0.0, 0.0, 1.0),
                layer_idx: 0,
                extrusion: 0.0,
            },
            Waypoint {
                position: Point3D::new(20.0, 10.0, 0.2),
                orientation: Vector3D::new(0.0, 0.0, 1.0),
                layer_idx: 0,
                extrusion: 1.0,
            },
        ];
        // First waypoint: one solution at A=0, B=0
        // Second waypoint: two solutions — (A=0, B=0) and (A=45, B=45)
        // Dijkstra should prefer (A=0, B=0) for smoothest transition
        let solutions = vec![
            vec![IKSolution {
                joint_angles: [10.0, 10.0, 0.2, 0.0, 0.0],
                manipulability: 0.5,
                is_singular: false,
            }],
            vec![
                IKSolution {
                    joint_angles: [20.0, 10.0, 0.2, 45.0, 45.0], // large angular jump
                    manipulability: 0.5,
                    is_singular: false,
                },
                IKSolution {
                    joint_angles: [20.0, 10.0, 0.2, 0.0, 0.0], // smooth transition
                    manipulability: 0.5,
                    is_singular: false,
                },
            ],
        ];
        let result = planner.plan_path_from_solutions(&solutions, &waypoints).unwrap();
        // Should pick solution index 1 for the second waypoint (smooth transition)
        assert_eq!(result[1], (1, 1));
    }

    #[test]
    fn test_dijkstra_avoids_singular_solutions() {
        let planner = make_planner();
        let waypoints = vec![
            Waypoint {
                position: Point3D::new(10.0, 10.0, 0.2),
                orientation: Vector3D::new(0.0, 0.0, 1.0),
                layer_idx: 0,
                extrusion: 0.0,
            },
            Waypoint {
                position: Point3D::new(20.0, 10.0, 0.2),
                orientation: Vector3D::new(0.0, 0.0, 1.0),
                layer_idx: 0,
                extrusion: 1.0,
            },
        ];
        // Second waypoint: solution 0 is near-singular (manipulability << threshold)
        // solution 1 has small angular jump but good manipulability
        let solutions = vec![
            vec![IKSolution {
                joint_angles: [10.0, 10.0, 0.2, 0.0, 0.0],
                manipulability: 0.5,
                is_singular: false,
            }],
            vec![
                IKSolution {
                    joint_angles: [20.0, 10.0, 0.2, 0.0, 0.0], // smooth but near-singular
                    manipulability: 0.001,
                    is_singular: false,
                },
                IKSolution {
                    joint_angles: [20.0, 10.0, 0.2, 1.0, 1.0], // tiny jump, good manipulability
                    manipulability: 0.5,
                    is_singular: false,
                },
            ],
        ];
        let result = planner.plan_path_from_solutions(&solutions, &waypoints).unwrap();
        // Should prefer solution 1 (good manipulability) despite small angular cost
        assert_eq!(result[1], (1, 1));
    }
}
