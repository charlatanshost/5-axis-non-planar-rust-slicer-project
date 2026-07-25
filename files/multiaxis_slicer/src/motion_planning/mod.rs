// Motion planning module - algorithms from MultiAxis_3DP_MotionPlanning

pub mod variable_filament;
pub mod singularity;
pub mod collision;
pub mod graph_search;

pub use variable_filament::{FilamentCalculator, Waypoint};
pub use singularity::{SingularityOptimizer, IKSolution, MachineConfig};
pub use collision::{CollisionDetector, CollisionResult, PrintHead};
pub use graph_search::PathPlanner;

/// Real print geometry for recomputing waypoint extrusion.
///
/// Supplying this opts into overwriting the extrusion values produced by
/// `ToolpathGenerator`. Only do so with the actual settings in force -- the
/// point of the type is that these can no longer be guessed.
#[derive(Debug, Clone, Copy)]
pub struct ExtrusionRecompute {
    pub layer_height: f64,
    pub line_width: f64,
}

/// Complete motion planning pipeline
pub struct MotionPlanner {
    pub filament_calculator: FilamentCalculator,
    pub singularity_optimizer: SingularityOptimizer,
    pub collision_detector: CollisionDetector,
    pub path_planner: PathPlanner,
    /// `None` (the default) leaves incoming extrusion values untouched.
    pub extrusion_recompute: Option<ExtrusionRecompute>,
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
            extrusion_recompute: config.extrusion_recompute,
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

    /// Step 1 of the pipeline.
    ///
    /// Waypoint extrusion values arrive already computed by `ToolpathGenerator`,
    /// which accounts for the real per-layer height and the configured line
    /// width. This used to overwrite them unconditionally using hardcoded
    /// 0.2 mm / 0.4 mm literals, silently rescaling flow on every segment --
    /// roughly a 50% error on a 0.3 mm layer, with no warning.
    ///
    /// Recomputation is now opt-in and requires the caller to supply the real
    /// geometry through [`MotionPlanningConfig::extrusion_recompute`]. With the
    /// default of `None`, incoming values pass through untouched.
    fn calculate_all_extrusions(&self, waypoints: &mut [Waypoint]) {
        let Some(params) = self.extrusion_recompute else {
            return;
        };

        for i in 0..waypoints.len() {
            let prev = if i > 0 { Some(&waypoints[i - 1]) } else { None };

            let extrusion = self.filament_calculator.calculate_extrusion(
                &waypoints[i],
                prev,
                params.layer_height,
                params.line_width,
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
    /// Opt into recomputing waypoint extrusion. Defaults to `None`, which
    /// preserves the values computed during toolpath generation.
    pub extrusion_recompute: Option<ExtrusionRecompute>,
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
            extrusion_recompute: None,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point3D, Vector3D};

    const FILAMENT_DIAMETER: f64 = 1.75;

    fn sample_waypoints() -> Vec<Waypoint> {
        let up = Vector3D::new(0.0, 0.0, 1.0);
        vec![
            Waypoint { position: Point3D::new(0.0, 0.0, 0.3),  orientation: up, layer_idx: 0, extrusion: 0.0 },
            Waypoint { position: Point3D::new(10.0, 0.0, 0.3), orientation: up, layer_idx: 0, extrusion: 1.234_5 },
            Waypoint { position: Point3D::new(10.0, 10.0, 0.3), orientation: up, layer_idx: 0, extrusion: 2.5 },
        ]
    }

    fn filament_area() -> f64 {
        std::f64::consts::PI * (FILAMENT_DIAMETER / 2.0).powi(2)
    }

    /// Regression test for T-04. The planner used to overwrite every waypoint's
    /// extrusion using hardcoded 0.2 mm / 0.4 mm geometry, corrupting flow that
    /// `ToolpathGenerator` had already computed correctly.
    #[test]
    fn test_extrusion_untouched_by_default() {
        let planner = MotionPlanner::new(MotionPlanningConfig::default());
        let mut waypoints = sample_waypoints();
        let before: Vec<f64> = waypoints.iter().map(|w| w.extrusion).collect();

        planner.calculate_all_extrusions(&mut waypoints);

        for (i, (wp, original)) in waypoints.iter().zip(before.iter()).enumerate() {
            assert_eq!(
                wp.extrusion.to_bits(),
                original.to_bits(),
                "waypoint {} extrusion changed from {} to {}",
                i,
                original,
                wp.extrusion
            );
        }
    }

    #[test]
    fn test_extrusion_recompute_uses_supplied_geometry() {
        let config = MotionPlanningConfig {
            extrusion_recompute: Some(ExtrusionRecompute { layer_height: 0.3, line_width: 0.5 }),
            ..Default::default()
        };
        let planner = MotionPlanner::new(config);
        let mut waypoints = sample_waypoints();

        planner.calculate_all_extrusions(&mut waypoints);

        // Second waypoint is a 10 mm move at a 0.3 x 0.5 mm cross-section.
        let expected = 0.3 * 0.5 * 10.0 / filament_area();
        assert!(
            (waypoints[1].extrusion - expected).abs() < 1e-12,
            "expected {}, got {}",
            expected,
            waypoints[1].extrusion
        );

        // It must not fall back to the old hardcoded 0.2 x 0.4 geometry.
        let hardcoded = 0.2 * 0.4 * 10.0 / filament_area();
        assert!(
            (waypoints[1].extrusion - hardcoded).abs() > 1e-6,
            "recompute still used the hardcoded 0.2/0.4 geometry"
        );
    }
}
