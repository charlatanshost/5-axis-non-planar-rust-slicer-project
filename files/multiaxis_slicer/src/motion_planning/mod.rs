// Motion planning module - algorithms from MultiAxis_3DP_MotionPlanning

pub mod variable_filament;
pub mod singularity;
pub mod collision;
pub mod graph_search;

pub use variable_filament::{FilamentCalculator, Waypoint};
pub use singularity::{SingularityOptimizer, IKSolution, MachineConfig};
pub use collision::{CollisionDetector, CollisionResult, PrintHead};
pub use graph_search::PathPlanner;

/// Complete motion planning pipeline
pub struct MotionPlanner {
    pub filament_calculator: FilamentCalculator,
    pub singularity_optimizer: SingularityOptimizer,
    pub collision_detector: CollisionDetector,
    pub path_planner: PathPlanner,
}

impl MotionPlanner {
    pub fn new(config: MotionPlanningConfig) -> Self {
        let collision_detector = CollisionDetector {
            print_head: config.print_head,
            platform_bounds: config.platform_bounds,
            printed_mesh: None,
        };

        let singularity_optimizer = SingularityOptimizer {
            lambda_threshold: config.lambda_threshold,
            machine_config: config.machine_config,
        };

        let path_planner = PathPlanner {
            collision_detector: collision_detector.clone(),
            singularity_optimizer: singularity_optimizer.clone(),
        };

        Self {
            filament_calculator: config.filament_calculator,
            singularity_optimizer,
            collision_detector,
            path_planner,
        }
    }

    /// Run complete 6-step pipeline
    pub fn plan_complete_path(&self, mut waypoints: Vec<Waypoint>) -> crate::Result<MotionPlan> {
        log::info!("Step 1: Variable filament calculation");
        self.calculate_all_extrusions(&mut waypoints);

        log::info!("Step 2: Singularity optimization");
        let solutions = self.compute_all_ik_solutions(&waypoints);

        log::info!("Step 3: Collision checking");
        let valid_solutions = self.filter_collision_free(&solutions, &waypoints);

        log::info!("Step 4: Graph search");
        let path = self.path_planner.plan_path_from_solutions(&valid_solutions, &waypoints)?;

        log::info!("Step 5: Motion plan complete");
        Ok(MotionPlan { waypoints, path })
    }

    fn calculate_all_extrusions(&self, waypoints: &mut [Waypoint]) {
        for i in 0..waypoints.len() {
            let prev = if i > 0 { Some(&waypoints[i - 1]) } else { None };

            let extrusion = self.filament_calculator.calculate_extrusion(
                &waypoints[i],
                prev,
                0.2,  // layer_height
                0.4,  // toolpath_width
            );

            waypoints[i].extrusion = extrusion;
        }
    }

    fn compute_all_ik_solutions(&self, waypoints: &[Waypoint]) -> Vec<Vec<IKSolution>> {
        waypoints
            .iter()
            .map(|wp| {
                self.singularity_optimizer.compute_ik_solutions(
                    &wp.position,
                    &wp.orientation,
                )
            })
            .collect()
    }

    fn filter_collision_free(
        &self,
        all_solutions: &[Vec<IKSolution>],
        waypoints: &[Waypoint],
    ) -> Vec<Vec<IKSolution>> {
        all_solutions
            .iter()
            .zip(waypoints.iter())
            .map(|(solutions, waypoint)| {
                solutions
                    .iter()
                    .filter(|sol| {
                        !sol.is_singular
                            && self.is_collision_free(waypoint, sol)
                    })
                    .cloned()
                    .collect()
            })
            .collect()
    }

    fn is_collision_free(&self, waypoint: &Waypoint, _solution: &IKSolution) -> bool {
        let result = self.collision_detector.check_collision_at_waypoint(
            &waypoint.position,
            &waypoint.orientation,
        );
        result.is_collision_free
    }
}

#[derive(Debug, Clone)]
pub struct MotionPlanningConfig {
    pub filament_calculator: FilamentCalculator,
    pub machine_config: MachineConfig,
    pub lambda_threshold: f64,
    pub print_head: PrintHead,
    pub platform_bounds: (crate::geometry::Point3D, crate::geometry::Point3D),
}

impl Default for MotionPlanningConfig {
    fn default() -> Self {
        use crate::geometry::Point3D;

        Self {
            filament_calculator: FilamentCalculator {
                filament_diameter: 1.75,
                nozzle_diameter: 0.4,
            },
            machine_config: MachineConfig {
                work_volume: (
                    Point3D::new(0.0, 0.0, 0.0),
                    Point3D::new(200.0, 200.0, 200.0),
                ),
                a_limits: (-90.0, 90.0),
                b_limits: (-90.0, 90.0),
            },
            lambda_threshold: 0.01,
            print_head: PrintHead {
                nozzle_diameter: 0.4,
                body_radius: 20.0,
                body_height: 50.0,
            },
            platform_bounds: (
                Point3D::new(0.0, 0.0, 0.0),
                Point3D::new(200.0, 200.0, 200.0),
            ),
        }
    }
}

#[derive(Debug, Clone)]
pub struct MotionPlan {
    pub waypoints: Vec<Waypoint>,
    pub path: Vec<(usize, usize)>,  // (waypoint_idx, solution_idx)
}

impl MotionPlan {
    /// Get the selected IK solution for each waypoint
    pub fn get_joint_angles(&self) -> Vec<[f64; 5]> {
        // This would need to store the actual solutions
        // Placeholder for now
        vec![[0.0; 5]; self.waypoints.len()]
    }
}
