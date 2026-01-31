// Singularity detection and optimization - Step 2
use crate::geometry::{Point3D, Vector3D};

#[derive(Debug, Clone)]
pub struct SingularityOptimizer {
    pub lambda_threshold: f64,
    pub machine_config: MachineConfig,
}

#[derive(Debug, Clone)]
pub struct IKSolution {
    pub joint_angles: [f64; 5],
    pub manipulability: f64,
    pub is_singular: bool,
}

#[derive(Debug, Clone)]
pub struct MachineConfig {
    pub work_volume: (Point3D, Point3D),
    pub a_limits: (f64, f64),
    pub b_limits: (f64, f64),
}

impl SingularityOptimizer {
    pub fn compute_ik_solutions(&self, position: &Point3D, orientation: &Vector3D) -> Vec<IKSolution> {
        let (a_angle, b_angle) = self.orientation_to_angles(orientation);

        // Two IK solutions (elbow up/down)
        vec![
            IKSolution {
                joint_angles: [position.x, position.y, position.z, a_angle, b_angle],
                manipulability: 0.5,
                is_singular: false,
            },
            IKSolution {
                joint_angles: [position.x, position.y, position.z, 180.0 - a_angle, -b_angle],
                manipulability: 0.3,
                is_singular: false,
            },
        ]
    }

    fn orientation_to_angles(&self, orientation: &Vector3D) -> (f64, f64) {
        let a_angle = orientation.y.atan2(orientation.z).to_degrees();
        let b_angle = (-orientation.x).atan2(orientation.z).to_degrees();
        (a_angle, b_angle)
    }
}
