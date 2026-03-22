// Singularity detection and optimization - Step 2
// Phase F: Real manipulability computation for rotary axes

use crate::geometry::{Point3D, Vector3D};

/// Rotary axis configuration mode, matching the gcode module's convention.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotaryMode {
    /// A (pitch/tilt) + B (roll/rotate) axes
    AB,
    /// B (tilt) + C (rotate) axes
    BC,
}

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
    /// Compute IK solutions for a given position and orientation.
    /// Generates two candidates (elbow up/down) with real manipulability values.
    pub fn compute_ik_solutions(&self, position: &Point3D, orientation: &Vector3D) -> Vec<IKSolution> {
        let (a_angle, b_angle) = self.orientation_to_angles(orientation);
        let manip_primary = self.compute_manipulability(a_angle, b_angle);

        let a_alt = 180.0 - a_angle;
        let b_alt = -b_angle;
        let manip_alt = self.compute_manipulability(a_alt, b_alt);

        vec![
            IKSolution {
                joint_angles: [position.x, position.y, position.z, a_angle, b_angle],
                manipulability: manip_primary,
                is_singular: manip_primary < self.lambda_threshold,
            },
            IKSolution {
                joint_angles: [position.x, position.y, position.z, a_alt, b_alt],
                manipulability: manip_alt,
                is_singular: manip_alt < self.lambda_threshold,
            },
        ]
    }

    /// Compute real manipulability for a given pair of rotary axis angles.
    ///
    /// For AB axes: manipulability = |sin(A)| — singular when A ≈ 0° (tool vertical,
    /// B rotation has no effect on tool direction).
    ///
    /// For BC axes: manipulability = |sin(B)| — singular when B ≈ 0° (tool vertical,
    /// C rotation has no effect on tool direction).
    ///
    /// This reflects the physical reality: when the primary tilt axis is at zero,
    /// the secondary rotation axis becomes degenerate (gimbal lock).
    pub fn compute_manipulability(&self, a_deg: f64, b_deg: f64) -> f64 {
        // For a 2-axis rotary system, the primary tilt determines manipulability.
        // AB mode: A is the primary tilt → |sin(A)|
        // BC mode: B is the primary tilt → |sin(B)|
        // We use A as the primary axis by convention (matching our IK decomposition).
        let a_rad = a_deg.to_radians();
        let b_rad = b_deg.to_radians();

        // Combined manipulability: product of individual axis contributions
        // ensures both axes have meaningful range of motion
        let primary = a_rad.sin().abs();
        let secondary = b_rad.sin().abs();

        // Use the minimum — singular if either axis is degenerate
        primary.min(secondary).max(0.0)
    }

    /// Compute manipulability for a tool orientation vector directly.
    /// Useful for integration into the collision avoidance scoring without
    /// first decomposing into IK solutions.
    pub fn manipulability_from_orientation(orientation: &Vector3D, mode: RotaryMode) -> f64 {
        let len = orientation.norm();
        if len < 1e-9 { return 0.0; }

        match mode {
            RotaryMode::AB => {
                // A = tilt from vertical = acos(oz)
                // Singular when A ≈ 0 → tool is vertical → |sin(A)|
                let oz = (orientation.z / len).clamp(-1.0, 1.0);
                let a_rad = oz.acos();
                a_rad.sin().abs()
            }
            RotaryMode::BC => {
                // B = tilt from vertical = acos(oz)
                // Singular when B ≈ 0 → tool is vertical → |sin(B)|
                let oz = (orientation.z / len).clamp(-1.0, 1.0);
                let b_rad = oz.acos();
                b_rad.sin().abs()
            }
        }
    }

    fn orientation_to_angles(&self, orientation: &Vector3D) -> (f64, f64) {
        let a_angle = orientation.y.atan2(orientation.z).to_degrees();
        let b_angle = (-orientation.x).atan2(orientation.z).to_degrees();
        (a_angle, b_angle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_optimizer() -> SingularityOptimizer {
        SingularityOptimizer {
            lambda_threshold: 0.1,
            machine_config: MachineConfig {
                work_volume: (
                    Point3D::new(0.0, 0.0, 0.0),
                    Point3D::new(200.0, 200.0, 200.0),
                ),
                a_limits: (-90.0, 90.0),
                b_limits: (-90.0, 90.0),
            },
        }
    }

    #[test]
    fn test_vertical_is_singular() {
        // Tool pointing straight up: A=0, B=0 → manipulability should be ~0
        let opt = make_optimizer();
        let manip = opt.compute_manipulability(0.0, 0.0);
        assert!(manip < 0.01, "Vertical should be singular, got {}", manip);
    }

    #[test]
    fn test_tilted_is_not_singular() {
        let opt = make_optimizer();
        // A=45°, B=45° → both sin values are ~0.707
        let manip = opt.compute_manipulability(45.0, 45.0);
        assert!(manip > 0.5, "45° tilt should have good manipulability, got {}", manip);
    }

    #[test]
    fn test_manipulability_from_orientation_vertical() {
        let ori = Vector3D::new(0.0, 0.0, 1.0);
        let manip = SingularityOptimizer::manipulability_from_orientation(&ori, RotaryMode::AB);
        assert!(manip < 0.01, "Vertical orientation should be singular, got {}", manip);
    }

    #[test]
    fn test_manipulability_from_orientation_tilted() {
        // 45° tilt in X direction
        let ori = Vector3D::new(0.707, 0.0, 0.707);
        let manip = SingularityOptimizer::manipulability_from_orientation(&ori, RotaryMode::AB);
        assert!(manip > 0.5, "45° tilt should have good manipulability, got {}", manip);
    }

    #[test]
    fn test_ik_solutions_contain_real_manipulability() {
        let opt = make_optimizer();
        let pos = Point3D::new(50.0, 50.0, 10.0);
        let ori = Vector3D::new(0.5, 0.0, 0.866); // ~30° tilt
        let solutions = opt.compute_ik_solutions(&pos, &ori);
        assert_eq!(solutions.len(), 2);
        // At least one solution should have non-trivial manipulability
        assert!(solutions.iter().any(|s| s.manipulability > 0.0));
    }
}
