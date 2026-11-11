// Main GUI application state and update logic

use crate::Mesh;
use crate::geometry::Vector3D;
use crate::slicing::{Slicer, SlicingConfig, Layer};
use crate::toolpath::{Toolpath, ToolpathGenerator};
use crate::gcode::GCodeGenerator;
use crate::gui::{viewport_3d::Viewport3D, theme};
use crate::support_generation::{SupportGenerator, SupportResult, OverhangConfig};
use crate::s3_slicer::{
    execute_s3_pipeline, execute_s4_pipeline, S3PipelineConfig, FabricationObjective,
    S4DeformData, execute_s4_deform, execute_s4_slice, execute_s4_untransform,
    quaternion_field::{QuaternionField, QuaternionFieldConfig},
    deformation_v2::S3SlicerDeformation,
};

use std::sync::{Arc, Mutex, mpsc};
use std::path::PathBuf;

/// Progress tracking for background operations
#[derive(Debug, Clone)]
pub struct SlicingProgress {
    pub operation: String,
    pub percentage: f32,
    pub layers_completed: usize,
    pub total_layers: usize,
    pub message: String,
}

impl Default for SlicingProgress {
    fn default() -> Self {
        Self {
            operation: "Idle".to_string(),
            percentage: 0.0,
            layers_completed: 0,
            total_layers: 0,
            message: String::new(),
        }
    }
}

/// Machine configuration for multi-axis printing
#[derive(Debug, Clone)]
pub struct MachineConfig {
    pub name: String,
    pub workspace_x: (f64, f64), // (min, max) in mm
    pub workspace_y: (f64, f64),
    pub workspace_z: (f64, f64),
    pub a_axis_range: (f64, f64), // Rotation range in degrees
    pub b_axis_range: (f64, f64),
    pub max_feedrate: f64, // mm/s
    pub has_heated_bed: bool,
}

impl Default for MachineConfig {
    fn default() -> Self {
        Self {
            name: "Generic 5-Axis".to_string(),
            workspace_x: (0.0, 200.0),
            workspace_y: (0.0, 200.0),
            workspace_z: (0.0, 200.0),
            a_axis_range: (-180.0, 180.0),
            b_axis_range: (-180.0, 180.0),
            max_feedrate: 150.0,
            has_heated_bed: true,
        }
    }
}

/// Slicing mode selection
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SlicingMode {
    /// Traditional planar slicing
    Planar,
    /// Curved layer slicing (S3-Slicer)
    Curved,
    /// S4 non-planar slicing (Deform → Slice → Un-Deform)
    S4,
    /// Conical slicing (RotBot/Transform — cone-based Z shift)
    Conical,
    /// Geodesic slicing (Heat Method distance field)
    Geodesic,
    /// Cylindrical coordinate-transform slicing (concentric radial shells)
    CoordTransformCylindrical,
    /// Spherical coordinate-transform slicing (concentric spherical shells)
    CoordTransformSpherical,
}

/// Tracks the current stage of the S4 interactive step-by-step workflow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum S4Stage {
    #[default]
    Idle,      // No deformation stored
    Deformed,  // S4DeformData stored, deformed mesh visible in viewport
    Sliced,    // Deformed layers computed (flat layers on deformed mesh)
    Final,     // Layers untransformed back to original space
}

// ─── Printer Profile Data Model ──────────────────────────────────────────────

/// Whether a rotary axis moves the head or the bed
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum MovingPart { Head, Bed }

impl MovingPart {
    pub fn label(&self) -> &'static str { match self { Self::Head => "Head", Self::Bed => "Bed" } }
    pub fn all() -> &'static [Self] { &[Self::Head, Self::Bed] }
}

/// Whether an axis is linear or rotary
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum AxisType { Linear, Rotary }

impl AxisType {
    pub fn label(&self) -> &'static str { match self { Self::Linear => "Linear", Self::Rotary => "Rotary" } }
    pub fn all() -> &'static [Self] { &[Self::Linear, Self::Rotary] }
}

/// Physical axis a rotary joint revolves around (X, Y, or Z in machine frame)
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub enum RotationAxis {
    X,
    #[default]
    Y,
    Z,
}

impl RotationAxis {
    pub fn label(&self) -> &'static str { match self { Self::X => "X", Self::Y => "Y", Self::Z => "Z" } }
    pub fn all() -> &'static [Self] { &[Self::X, Self::Y, Self::Z] }
    /// Sensible default rotation axis inferred from conventional axis letter.
    pub fn default_for_name(name: &str) -> Self {
        match name { "A" | "a" => Self::X, "B" | "b" => Self::Y, _ => Self::Z }
    }
    pub fn as_vec3_f64(&self) -> [f64; 3] {
        match self { Self::X => [1.,0.,0.], Self::Y => [0.,1.,0.], Self::Z => [0.,0.,1.] }
    }
    pub fn as_vec3_f32(&self) -> [f32; 3] {
        match self { Self::X => [1.,0.,0.], Self::Y => [0.,1.,0.], Self::Z => [0.,0.,1.] }
    }
}

/// Configuration for a single machine axis
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AxisConfig {
    pub name: String,           // "A", "B", "C", "U", "V", "W", …
    pub axis_type: AxisType,
    pub moving_part: MovingPart,
    pub min: f64,
    pub max: f64,
    /// Which physical axis this joint revolves around (default Y)
    #[serde(default)]
    pub rotation_axis: RotationAxis,
    /// Maximum angular speed in degrees/second. G-code feedrate is capped so
    /// this axis never exceeds this speed. 0 = unlimited.
    #[serde(default = "default_axis_angular_speed")]
    pub max_angular_speed: f64,
    /// Sign convention: `true` when a positive commanded value produces a
    /// right-handed rotation about `rotation_axis`, `false` when the machine
    /// defines positive the other way.
    ///
    /// Machines genuinely differ, so this has to be configuration. Set it to
    /// match the real hardware and confirm with the round-trip check on the
    /// profile page, which reports whether the commanded angles actually point
    /// the tool where the slicer intended.
    #[serde(default = "default_axis_right_handed")]
    pub right_handed: bool,
}

fn default_axis_angular_speed() -> f64 { 30.0 }
fn default_axis_right_handed() -> bool { true }

impl Default for AxisConfig {
    fn default() -> Self {
        Self {
            name: "A".to_string(),
            axis_type: AxisType::Rotary,
            moving_part: MovingPart::Head,
            min: -180.0,
            max: 180.0,
            rotation_axis: RotationAxis::X,
            max_angular_speed: 30.0,
            right_handed: true,
        }
    }
}

/// High-level kinematics category
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum KinematicsType {
    Cartesian,
    CoreXY,
    Delta,
    FiveAxis,
    SixAxis,
    SevenAxis,
    RoboticArm,
}

impl KinematicsType {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Cartesian  => "Cartesian (3-axis)",
            Self::CoreXY     => "CoreXY",
            Self::Delta      => "Delta",
            Self::FiveAxis   => "5-Axis CNC",
            Self::SixAxis    => "6-Axis Robotic Arm",
            Self::SevenAxis  => "7-Axis (Trunnion + Rotary Head)",
            Self::RoboticArm => "Robotic Arm (custom)",
        }
    }
    pub fn all() -> &'static [Self] {
        use KinematicsType::*;
        &[Cartesian, CoreXY, Delta, FiveAxis, SixAxis, SevenAxis, RoboticArm]
    }
}

/// A complete printer hardware profile that can be saved/loaded as JSON
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PrinterProfile {
    pub name: String,
    // Build volume
    pub workspace_x: (f64, f64),
    pub workspace_y: (f64, f64),
    pub workspace_z: (f64, f64),
    // Kinematics
    pub kinematics_type: KinematicsType,
    /// Flexible rotary axis list — up to 7 axes
    pub rotary_axes: Vec<AxisConfig>,
    // 5-axis G-code output
    pub tcp_offset: f64,
    pub rotary_axis_mode: crate::gcode::RotaryAxisMode,
    // Toolhead
    pub nozzle_diameter: f64,
    pub max_feedrate: f64,
    pub has_heated_bed: bool,
    // Machine simulation geometry  (all have serde defaults for backward compatibility)
    #[serde(default = "profile_default_show_machine")]
    pub show_machine: bool,
    /// Bed bounding box [width_mm, depth_mm, height_mm]
    #[serde(default = "profile_default_bed_dims")]
    pub bed_dims: [f64; 3],
    /// World-space pivot point the bed axes rotate around (usually origin)
    #[serde(default)]
    pub bed_pivot: [f64; 3],
    /// Optional path to an STL file that replaces the parametric bed box
    #[serde(default)]
    pub bed_stl_path: Option<String>,
    /// Printhead carriage bounding box [width_mm, depth_mm, height_mm]
    #[serde(default = "profile_default_head_dims")]
    pub head_dims: [f64; 3],
    /// Optional path to an STL file that replaces the parametric head box
    #[serde(default)]
    pub head_stl_path: Option<String>,
    /// The point inside the head STL file that corresponds to the nozzle tip [x, y, z] in mm.
    /// When the STL is placed in the simulation, this local point is pinned to the actual
    /// nozzle-tip world position.  Default (0,0,0) means the STL origin IS the nozzle tip.
    #[serde(default)]
    pub head_stl_tip_offset: [f64; 3],
    /// Radius of the nozzle cylinder that hangs below the head pivot (mm)
    #[serde(default = "profile_default_nozzle_radius")]
    pub nozzle_radius: f64,
    /// Visual length of the nozzle cylinder hanging below the head body pivot (mm).
    /// Independent of tcp_offset; set this to match the real nozzle-tip distance from
    /// the carriage so the machine simulation looks correct.
    #[serde(default = "profile_default_nozzle_length")]
    pub nozzle_length: f64,
    /// Height above the nozzle tip excluded from collision testing (mm).
    ///
    /// Models the nozzle cone protruding below the heater block, so the head
    /// body is not tested against the layer currently being printed. Typically
    /// 1-3 mm; unrelated to `body_radius`, which is what it used to be derived
    /// from.
    #[serde(default = "profile_default_nozzle_clearance")]
    pub nozzle_clearance: f64,
    /// How far the tool axis may lean away from vertical, in degrees.
    ///
    /// This is **not** the same quantity as a rotary axis's travel range, and
    /// conflating the two is what let the orientation passes emit upside-down
    /// tool vectors: the default profile allows A and B to travel +/-180
    /// degrees, so deriving the tilt budget from axis travel permitted tilts up
    /// to 177 degrees, i.e. the nozzle pointing into the bed.
    ///
    /// What actually bounds it is the head's physical envelope -- gantry
    /// clearance, hot-end geometry, filament path -- so it is a separate
    /// property of the machine. Both limits apply; the effective budget is
    /// whichever is tighter.
    #[serde(default = "profile_default_max_tool_tilt")]
    pub max_tool_tilt_deg: f64,
    /// Bed surface shape for machine simulation
    #[serde(default)]
    pub bed_shape: BedShape,
    /// Maximum symmetric linear travel of the bed on each axis [X, Y, Z] in mm.
    /// A value of 0 means the bed does not travel on that axis.
    /// Z travel means the bed rises/falls to meet the head (bed-slinger Z motion).
    #[serde(default)]
    pub bed_travel: [f64; 3],
    /// Maximum symmetric linear travel of the toolhead on each axis [X, Y, Z] in mm.
    /// A value of 0 means the toolhead does not travel on that axis.
    #[serde(default)]
    pub head_travel: [f64; 3],
}

fn profile_default_show_machine() -> bool { true }
fn profile_default_bed_dims() -> [f64; 3] { [200.0, 200.0, 10.0] }
fn profile_default_head_dims() -> [f64; 3] { [60.0, 60.0, 80.0] }
fn profile_default_nozzle_radius() -> f64 { 5.0 }
fn profile_default_nozzle_length() -> f64 { 20.0 }
fn profile_default_nozzle_clearance() -> f64 { 2.0 }
fn profile_default_max_tool_tilt() -> f64 { 45.0 }

/// Bed surface shape used in machine simulation
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub enum BedShape {
    #[default]
    Rectangle,
    Circle,
}

impl Default for PrinterProfile {
    fn default() -> Self {
        Self {
            name: "Generic 5-Axis".to_string(),
            workspace_x: (0.0, 200.0),
            workspace_y: (0.0, 200.0),
            workspace_z: (0.0, 200.0),
            kinematics_type: KinematicsType::FiveAxis,
            rotary_axes: vec![
                // Both rotaries on the head: a tilting-head machine, matching
                // MachineKinematics::default(). The two defaults used to
                // disagree -- this one put B on the bed -- and since
                // build_gcode_axes reads the profile, the profile won. Bed
                // rotation also needs the part branch to transform the XYZ
                // path, which is not implemented; see
                // docs/KINEMATICS_DESIGN_NOTE.md.
                AxisConfig { name: "A".to_string(), axis_type: AxisType::Rotary, moving_part: MovingPart::Head, min: -180.0, max: 180.0, rotation_axis: RotationAxis::X, max_angular_speed: 30.0, right_handed: true },
                AxisConfig { name: "B".to_string(), axis_type: AxisType::Rotary, moving_part: MovingPart::Head, min: -180.0, max: 180.0, rotation_axis: RotationAxis::Y, max_angular_speed: 30.0, right_handed: true },
            ],
            tcp_offset: 0.0,
            rotary_axis_mode: crate::gcode::RotaryAxisMode::AB,
            nozzle_diameter: 0.4,
            max_feedrate: 150.0,
            has_heated_bed: true,
            show_machine: true,
            bed_dims: [200.0, 200.0, 10.0],
            bed_pivot: [0.0, 0.0, 0.0],
            bed_stl_path: None,
            head_dims: [60.0, 60.0, 80.0],
            head_stl_path: None,
            head_stl_tip_offset: [0.0, 0.0, 0.0],
            bed_shape: BedShape::Rectangle,
            nozzle_radius: 5.0,
            nozzle_length: 20.0,
            nozzle_clearance: 2.0,
            max_tool_tilt_deg: 45.0,
            bed_travel:  [0.0, 0.0, 200.0], // bed moves up to 200 mm in Z by default
            head_travel: [300.0, 300.0, 0.0], // head moves 300 mm in X/Y by default
        }
    }
}

/// Top-level tab navigation (browser-tab style)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum AppTab {
    #[default]
    Main,
    PrinterProfiles,
}

// ─────────────────────────────────────────────────────────────────────────────

// ─── HeightMap ────────────────────────────────────────────────────────────────
/// 2-D grid that tracks the maximum Z deposited at each XY cell.
/// Used by `apply_tilt_collision_avoidance` to detect head-body collisions
/// with previously printed material.
struct HeightMap {
    cells: Vec<f64>,
    nx: usize,
    ny: usize,
    x0: f64,
    y0: f64,
    cell: f64, // cell side length (mm)
}

impl HeightMap {
    fn new(x0: f64, y0: f64, x1: f64, y1: f64, cell: f64) -> Self {
        let nx = (((x1 - x0) / cell).ceil() as usize).max(1);
        let ny = (((y1 - y0) / cell).ceil() as usize).max(1);
        Self {
            cells: vec![f64::NEG_INFINITY; nx * ny],
            nx, ny, x0, y0, cell,
        }
    }

    /// Mark a circle of radius `r` around `(cx, cy)` as having height `z`.
    fn update_circle(&mut self, cx: f64, cy: f64, z: f64, r: f64) {
        let ix0 = (((cx - r - self.x0) / self.cell).floor() as isize).max(0);
        let ix1 = (((cx + r - self.x0) / self.cell).ceil() as isize).min(self.nx as isize - 1);
        let iy0 = (((cy - r - self.y0) / self.cell).floor() as isize).max(0);
        let iy1 = (((cy + r - self.y0) / self.cell).ceil() as isize).min(self.ny as isize - 1);
        let r2 = r * r;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let gx = self.x0 + (ix as f64 + 0.5) * self.cell;
                let gy = self.y0 + (iy as f64 + 0.5) * self.cell;
                if (gx - cx) * (gx - cx) + (gy - cy) * (gy - cy) <= r2 {
                    let idx = iy as usize * self.nx + ix as usize;
                    if z > self.cells[idx] { self.cells[idx] = z; }
                }
            }
        }
    }

    /// Height at a single point `(x, y)`.  Returns `f64::NEG_INFINITY` if outside the map.
    fn get(&self, x: f64, y: f64) -> f64 {
        let ix = (((x - self.x0) / self.cell).floor() as isize).clamp(0, self.nx as isize - 1) as usize;
        let iy = (((y - self.y0) / self.cell).floor() as isize).clamp(0, self.ny as isize - 1) as usize;
        self.cells[iy * self.nx + ix]
    }

    /// Maximum height in a circle of radius `r` around `(cx, cy)`.
    /// Returns `f64::NEG_INFINITY` if no material has been deposited yet in that region.
    fn max_z_in_circle(&self, cx: f64, cy: f64, r: f64) -> f64 {
        let ix0 = (((cx - r - self.x0) / self.cell).floor() as isize).max(0);
        let ix1 = (((cx + r - self.x0) / self.cell).ceil() as isize).min(self.nx as isize - 1);
        let iy0 = (((cy - r - self.y0) / self.cell).floor() as isize).max(0);
        let iy1 = (((cy + r - self.y0) / self.cell).ceil() as isize).min(self.ny as isize - 1);
        let r2 = r * r;
        let mut best = f64::NEG_INFINITY;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let gx = self.x0 + (ix as f64 + 0.5) * self.cell;
                let gy = self.y0 + (iy as f64 + 0.5) * self.cell;
                if (gx - cx) * (gx - cx) + (gy - cy) * (gy - cy) <= r2 {
                    let v = self.cells[iy as usize * self.nx + ix as usize];
                    if v > best { best = v; }
                }
            }
        }
        best
    }

    /// Maximum height along a line from `(x0,y0)` to `(x1,y1)`.
    /// Samples every `cell`-sized step and returns the highest Z encountered.
    fn max_z_along_line(&self, x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
        let dx = x1 - x0;
        let dy = y1 - y0;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-6 { return self.get(x0, y0); }
        let steps = ((len / self.cell).ceil() as usize).max(1);
        let mut best = f64::NEG_INFINITY;
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let v = self.get(x0 + dx * t, y0 + dy * t);
            if v > best { best = v; }
        }
        best
    }
}

// ─────────────────────────────────────────────────────────────────────────────

/// Main application state
pub struct SlicerApp {
    // Model data
    pub mesh: Option<Mesh>,
    pub mesh_path: Option<PathBuf>,
    pub deformed_mesh: Option<Mesh>, // S3-Slicer deformed mesh preview
    pub show_deformed_mesh: bool,     // Toggle between original and deformed view
    pub is_deforming: bool,           // Background deformation in progress
    pub layers: Vec<Layer>,
    pub toolpaths: Vec<Toolpath>,
    pub gcode_lines: Vec<String>,

    // Slicing configuration
    pub config: SlicingConfig,
    pub slicing_mode: SlicingMode,

    // Machine configuration
    pub machine: MachineConfig,

    // Toolpath configuration
    pub nozzle_diameter: f64,
    pub feedrate: f64,
    pub toolpath_pattern: crate::toolpath_patterns::ToolpathPattern,
    pub toolpath_line_width: f64,
    pub toolpath_infill_density: f64,
    pub wall_count: usize,
    pub infill_pattern: crate::toolpath_patterns::InfillPattern,
    pub force_nonplanar_infill: bool,  // Override skip_infill for geodesic/cylindrical/spherical
    pub wall_seam_transitions: bool,   // Insert ruled-surface seam paths between consecutive curved layers
    pub optimize_print_order: bool,    // Reorder contours per-layer to minimize travel (nearest-neighbor)
    pub enable_orientation_smoothing: bool,  // Phase C: bidirectional SLERP smoothing
    pub orientation_smoothing_rate: f64,     // Phase C: max degrees per mm of travel
    pub look_ahead_window: usize,            // Phase D: segments to look ahead in collision scoring
    pub enable_singularity_avoidance: bool,  // Phase F: penalize near-singular orientations
    pub singularity_threshold: f64,          // Phase F: manipulability threshold

    // Support generation
    pub support_config: OverhangConfig,
    pub support_result: Option<SupportResult>,
    pub has_supports: bool,
    pub show_supports: bool,
    pub enable_supports: bool,  // auto-generate supports when generating toolpaths

    // UI state
    pub show_mesh: bool,
    pub show_layers: bool,
    pub show_toolpaths: bool,
    pub show_wireframe: bool,
    pub show_collision_overlay: bool,
    pub selected_layer: Option<usize>,
    pub show_gcode_terminal: bool,
    pub gcode_scroll_to_bottom: bool,
    pub toolpath_playback_enabled: bool,
    pub toolpath_playback_position: usize, // Current segment index
    pub toolpath_playback_playing: bool,
    pub toolpath_playback_speed: f32, // Segments per second
    pub gcode_highlight_line: Option<usize>, // Line to highlight in terminal
    pub last_playback_time: std::time::Instant,

    // Section view
    pub section_enabled: bool,
    pub section_axis: u8,      // 0=X, 1=Y, 2=Z
    pub section_depth: f32,    // Normalized 0.0-1.0 within mesh bounds

    // Toolpath display options
    pub show_travel_moves: bool,

    // Background task state
    pub slicing_progress: Arc<Mutex<SlicingProgress>>,
    pub is_slicing: bool,
    pub has_sliced: bool,
    pub has_toolpaths: bool,
    pub has_motion_plan: bool,
    pub is_motion_planning: bool,

    // Channel for receiving sliced layers from background thread
    layers_receiver: Option<mpsc::Receiver<Vec<Layer>>>,

    // Channel for receiving deformed mesh from background thread
    deformed_mesh_receiver: Option<mpsc::Receiver<Mesh>>,

    // S4 interactive step-by-step workflow state
    pub s4_stage: S4Stage,
    pub s4_deform_data: Option<Box<S4DeformData>>,
    pub s4_deformed_layers: Vec<Layer>,
    s4_deform_result_receiver: Option<mpsc::Receiver<S4DeformData>>,
    s4_deformed_layers_receiver: Option<mpsc::Receiver<Vec<Layer>>>,

    // 3D viewport
    pub viewport_3d: Option<Viewport3D>,

    // S3-Slicer deformation configuration
    pub s3_fabrication_objective: FabricationObjective,
    pub s3_overhang_threshold: f64,        // degrees (30-60)
    pub s3_smoothness_weight: f64,         // 0.0-1.0
    pub s3_optimization_iterations: usize, // 10-100

    // Deformation method (Phase 3)
    pub s3_deformation_method: crate::s3_slicer::DeformationMethod,
    pub s3_asap_max_iterations: usize,     // 5-20
    pub s3_asap_convergence: f64,          // 1e-5 to 1e-3

    // S4-Slicer configuration (separate from S3)
    pub s4_overhang_threshold: f64,        // degrees (30-60)
    pub s4_max_rotation_degrees: f64,      // degrees (5-45)
    pub s4_smoothing_iterations: usize,    // 5-50
    pub s4_smoothness_weight: f64,         // 0.0-1.0
    pub s4_asap_max_iterations: usize,     // 3-20
    pub s4_asap_convergence: f64,          // 1e-5 to 1e-3
    pub s4_z_bias: f64,                    // Dijkstra Z-bias (0.0 = Euclidean, 1.0 = pure |ΔZ|)
    pub s4_auto_z_bias: bool,              // Auto-compute z_bias from mesh geometry
    pub s4_use_asap: bool,                 // Use full ASAP deformation instead of direct vertex rotation
    pub s4_base_detection_threshold: f64,  // Z-threshold percentage for base tet detection (0.01-0.20)
    pub s4_edge_filter_multiplier: f64,    // Edge length filter multiplier (5-30)
    pub s4_jump_split_multiplier: f64,     // Jump-split threshold as multiple of layer_height (5-30)
    pub s4_multi_axis: bool,               // Multi-axis overhang detection

    // Conical slicing configuration
    pub conical_angle_degrees: f64,        // Cone half-angle (5-60)
    pub conical_direction: crate::conical::ConicalDirection,
    pub conical_auto_center: bool,         // Auto-center on mesh centroid
    pub conical_center_x: f64,
    pub conical_center_y: f64,
    pub conical_use_artifact_filter: bool, // Split contours at long bed-level edges

    // Cylindrical/Spherical coordinate transform configuration
    pub coord_transform_auto_center: bool,
    pub coord_transform_center_x: f64,
    pub coord_transform_center_y: f64,
    pub coord_transform_center_z: f64,

    // Geodesic slicing configuration
    pub geodesic_source_mode: u8,          // 0=BottomBoundary, 1=PointSource
    pub geodesic_source_point: [f64; 3],   // For point source mode
    pub geodesic_heat_factor: f64,         // Base timestep factor (finest scale in multi-scale)
    pub geodesic_bottom_tolerance: f64,    // Z tolerance for bottom boundary
    pub geodesic_use_multiscale: bool,     // Enable multi-scale heat method
    pub geodesic_num_scales: usize,        // Number of doubling scales (default 6)
    /// Diffusion mode: 0=Isotropic, 1=AdaptiveScalar, 2=Anisotropic, 3=PrintDirectionBiased
    pub geodesic_diffusion_mode: u8,
    pub geodesic_adaptive_kappa_base: f64,  // Per-face κ = kappa_base × (avg_edge)² (mode 1)
    pub geodesic_anisotropy_ratio: f64,     // Curvature-direction stretch ratio (mode 2)
    pub geodesic_aniso_smooth_iters: usize, // Smoothing passes for curvature dirs (mode 2)
    pub geodesic_print_dir_axis: u8,        // 0=X, 1=Y, 2=Z preferred print direction (mode 3)
    pub geodesic_print_dir_ratio: f64,      // Diffusivity ratio along print direction (mode 3)
    pub geodesic_z_weight: f64,             // Z-regularization weight (0=pure geodesic, 1=pure planar)

    // G-code / machine kinematics configuration
    pub tcp_offset: f64,                               // pivot-to-nozzle distance (mm); 0 = disabled
    pub rotary_axis_mode: crate::gcode::RotaryAxisMode, // A/B vs B/C axis labels

    // Face orientation mode
    pub face_orientation_mode: bool,       // Whether we're in "select face to orient" mode
    pub selected_face_index: Option<usize>, // Index of the selected triangle

    // Logs
    pub log_messages: Vec<String>,

    // Toolpath post-processing
    /// Apply mesh surface normals as tool orientation for all non-conical slicing modes.
    /// Makes the tool tilt perpendicular to the printed surface, maximising rotary-axis use.
    pub use_surface_normals: bool,
    /// Z clearance (mm) added to non-extruding travel moves so the nozzle clears the print.
    pub travel_z_lift: f64,
    /// If true, after assigning surface-normal orientations, check whether the tilted head body
    /// would intersect previously deposited extrusions and adjust the tilt azimuth (and
    /// magnitude if necessary) to maintain `head_clearance_mm` of clearance.
    pub avoid_head_collision: bool,
    /// Minimum clearance (mm) between the head body and already-printed material.
    pub head_clearance_mm: f64,

    // Machine simulation
    /// Whether to show the machine bed + head in the viewport (mirrors active profile flag)
    pub show_machine_simulation: bool,
    /// One entry per flat toolpath segment (layer0_seg0, layer0_seg1, …, layer1_seg0, …).
    /// `true` = the head would physically collide with the bed at that move.
    pub collision_segments: Vec<bool>,

    // Tab navigation
    pub active_tab: AppTab,

    // Profile page I/O status
    pub profile_io_error: Option<String>,   // last load/save error to show inline

    // Printer profiles
    pub profiles: Vec<PrinterProfile>,
    pub active_profile_index: usize,   // which profile is loaded into the main session
    pub editing_profile_index: usize,  // which profile is open in the profiles page
}

impl Default for SlicerApp {
    fn default() -> Self {
        Self {
            mesh: None,
            mesh_path: None,
            deformed_mesh: None,
            show_deformed_mesh: false,
            is_deforming: false,
            layers: Vec::new(),
            toolpaths: Vec::new(),
            gcode_lines: Vec::new(),

            config: SlicingConfig::default(),
            slicing_mode: SlicingMode::Planar,

            machine: MachineConfig::default(),

            nozzle_diameter: 0.4,
            feedrate: 50.0,
            toolpath_pattern: crate::toolpath_patterns::ToolpathPattern::Contour,
            toolpath_line_width: 0.4,
            toolpath_infill_density: 0.2,
            wall_count: 2,
            infill_pattern: crate::toolpath_patterns::InfillPattern::Rectilinear,
            force_nonplanar_infill: false,
            wall_seam_transitions: false,
            optimize_print_order: true,
            enable_orientation_smoothing: true,
            orientation_smoothing_rate: 15.0,
            look_ahead_window: 3,
            enable_singularity_avoidance: false,
            singularity_threshold: 0.1,

            support_config: OverhangConfig::default(),
            support_result: None,
            has_supports: false,
            show_supports: true,
            enable_supports: false,

            show_mesh: true,
            show_layers: true,
            show_toolpaths: true,
            show_wireframe: false,
            show_collision_overlay: true,
            selected_layer: None,
            show_gcode_terminal: false,
            gcode_scroll_to_bottom: false,
            toolpath_playback_enabled: false,
            toolpath_playback_position: 0,
            toolpath_playback_playing: false,
            toolpath_playback_speed: 10.0, // 10 segments per second
            gcode_highlight_line: None,
            last_playback_time: std::time::Instant::now(),

            section_enabled: false,
            section_axis: 2,    // Z axis
            section_depth: 1.0, // Show all

            show_travel_moves: true,

            slicing_progress: Arc::new(Mutex::new(SlicingProgress::default())),
            is_slicing: false,
            has_sliced: false,
            has_toolpaths: false,
            has_motion_plan: false,
            is_motion_planning: false,

            layers_receiver: None,
            deformed_mesh_receiver: None,

            s4_stage: S4Stage::Idle,
            s4_deform_data: None,
            s4_deformed_layers: Vec::new(),
            s4_deform_result_receiver: None,
            s4_deformed_layers_receiver: None,

            viewport_3d: None,

            // S3-Slicer defaults
            s3_fabrication_objective: FabricationObjective::SupportFree,
            s3_overhang_threshold: 45.0,
            s3_smoothness_weight: 0.5,
            s3_optimization_iterations: 50,

            // Deformation method defaults
            s3_deformation_method: crate::s3_slicer::DeformationMethod::TetVolumetric, // Default to tet volumetric (best quality)
            s3_asap_max_iterations: 10,
            s3_asap_convergence: 1e-4,

            // S4-Slicer defaults
            s4_overhang_threshold: 45.0,
            s4_max_rotation_degrees: 15.0,
            s4_smoothing_iterations: 25,
            s4_smoothness_weight: 0.5,
            s4_asap_max_iterations: 10,
            s4_asap_convergence: 1e-4,
            s4_z_bias: 0.8,
            s4_auto_z_bias: true,
            // The alternative, rotating every vertex about one anchor on the
            // bed, turns any change in rotation between neighbouring tets into
            // shear in proportion to the distance from that anchor. Measured
            // on a Benchy: layer normals out to 121 degrees and 0.6 % of tets
            // inverted, against 10 degrees and none with ASAP.
            s4_use_asap: true,
            s4_base_detection_threshold: 0.05,
            s4_edge_filter_multiplier: 15.0,
            s4_jump_split_multiplier: 10.0,
            s4_multi_axis: false,

            // Conical slicing defaults
            conical_angle_degrees: 45.0,
            conical_direction: crate::conical::ConicalDirection::Outward,
            conical_auto_center: true,
            conical_center_x: 0.0,
            conical_center_y: 0.0,
            conical_use_artifact_filter: true,

            // Cylindrical/Spherical coordinate transform defaults
            coord_transform_auto_center: true,
            coord_transform_center_x: 0.0,
            coord_transform_center_y: 0.0,
            coord_transform_center_z: 0.0,

            // Geodesic slicing defaults
            geodesic_source_mode: 0,
            geodesic_source_point: [0.0, 0.0, 0.0],
            geodesic_heat_factor: 1.0,
            geodesic_bottom_tolerance: 0.1,
            geodesic_use_multiscale: true,
            geodesic_num_scales: 6,
            geodesic_diffusion_mode: 0,     // Isotropic
            geodesic_adaptive_kappa_base: 6.0,
            geodesic_anisotropy_ratio: 0.3,
            geodesic_aniso_smooth_iters: 2,
            geodesic_print_dir_axis: 2,     // Z-axis
            geodesic_print_dir_ratio: 10.0,
            geodesic_z_weight: 0.0,

            // G-code kinematics defaults
            tcp_offset: 0.0,
            rotary_axis_mode: crate::gcode::RotaryAxisMode::AB,

            // Face orientation defaults
            face_orientation_mode: false,
            selected_face_index: None,

            log_messages: vec!["Welcome to MultiAxis Slicer!".to_string()],

            // Toolpath post-processing
            use_surface_normals: true,
            travel_z_lift: 2.0,
            avoid_head_collision: true,
            head_clearance_mm: 1.0,

            // Machine simulation
            show_machine_simulation: false,
            collision_segments: Vec::new(),

            // Tab navigation
            active_tab: AppTab::Main,

            profile_io_error: None,

            // Printer profiles — start with one default profile
            profiles: vec![PrinterProfile::default()],
            active_profile_index: 0,
            editing_profile_index: 0,
        }
    }
}

impl SlicerApp {
    /// Create a new slicer application
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Apply custom theme
        theme::apply_theme(&cc.egui_ctx);

        let mut app = Self::default();

        // Restore persisted printer profiles and active index
        if let Some(storage) = cc.storage {
            if let Some(json) = storage.get_string("printer_profiles") {
                if let Ok(profiles) = serde_json::from_str::<Vec<PrinterProfile>>(&json) {
                    if !profiles.is_empty() {
                        app.profiles = profiles;
                    }
                }
            }
            if let Some(idx_str) = storage.get_string("active_profile_index") {
                if let Ok(idx) = idx_str.parse::<usize>() {
                    app.active_profile_index = idx.min(app.profiles.len().saturating_sub(1));
                    app.editing_profile_index = app.active_profile_index;
                }
            }
        }

        // Apply the loaded profile so machine simulation and settings are live from the start
        app.apply_active_profile();

        // Initialize 3D viewport
        app.viewport_3d = Some(Viewport3D::new(cc));

        app
    }

    /// Add a log message
    pub fn log(&mut self, message: String) {
        log::info!("{}", message);
        self.log_messages.push(message);

        // Keep only last 100 messages
        if self.log_messages.len() > 100 {
            self.log_messages.remove(0);
        }
    }

    /// Apply the active profile's settings to the current session
    pub fn apply_active_profile(&mut self) {
        if let Some(p) = self.profiles.get(self.active_profile_index).cloned() {
            self.machine.name = p.name.clone();
            self.machine.workspace_x = p.workspace_x;
            self.machine.workspace_y = p.workspace_y;
            self.machine.workspace_z = p.workspace_z;
            // Map first two rotary axes to legacy a/b ranges for machine config
            if let Some(ax) = p.rotary_axes.get(0) { self.machine.a_axis_range = (ax.min, ax.max); }
            if let Some(ax) = p.rotary_axes.get(1) { self.machine.b_axis_range = (ax.min, ax.max); }
            self.machine.max_feedrate = p.max_feedrate;
            self.machine.has_heated_bed = p.has_heated_bed;
            self.tcp_offset = p.tcp_offset;
            self.rotary_axis_mode = p.rotary_axis_mode.clone();
            self.nozzle_diameter = p.nozzle_diameter;
            self.feedrate = p.max_feedrate;
            self.show_machine_simulation = p.show_machine;
        }
    }

    /// Compute axis angles for a single tool orientation using the active profile's axis list.
    pub fn compute_segment_axis_angles(
        &self,
        orientation: &crate::geometry::Vector3D,
    ) -> Vec<(String, f64)> {
        let mut gen = crate::gcode::GCodeGenerator::new();
        gen.kinematics.axes = self.build_gcode_axes();
        gen.compute_axis_angles(orientation)
    }

    /// Assign mesh surface normals as tool orientation for every toolpath segment.
    ///
    /// Builds a fast 2-D XY bin grid over all triangles.  For each segment the nearest
    /// triangle centroid is found and its face normal used as the orientation vector.
    /// The normal is always flipped to the upper hemisphere (Z ≥ 0) and clamped to the
    /// maximum tilt angle of the head rotary axes defined in the active printer profile.
    ///
    /// Skipped for `Conical` mode (which has its own analytical normal formula).
    /// How far the tool axis may lean from vertical, in degrees.
    ///
    /// Two separate limits apply and the tighter one wins:
    ///
    /// * the rotary axes' travel range, since the machine cannot command an
    ///   angle it cannot reach, and
    /// * the head's physical tilt envelope, `PrinterProfile::max_tool_tilt_deg`.
    ///
    /// Every orientation pass must use this rather than deriving a budget from
    /// axis travel alone. Doing the latter is what produced G-code with tilts
    /// up to 177.98 degrees -- the nozzle pointing into the bed -- because the
    /// default profile lets A and B travel +/-180 degrees.
    ///
    /// `head_axes_only` restricts the axis-travel part to axes that move the
    /// head, for passes that only reorient the tool rather than the part.
    pub fn tool_tilt_budget_deg(&self, head_axes_only: bool) -> f64 {
        let profile = self.profiles.get(self.active_profile_index);

        // Tightest relevant rotary axis. IK assigns the same angle magnitude to
        // each axis, so the most restricted one is the bottleneck.
        let axis_budget = profile
            .map(|p| {
                p.rotary_axes
                    .iter()
                    .filter(|ax| {
                        ax.axis_type == AxisType::Rotary
                            && (!head_axes_only || ax.moving_part == MovingPart::Head)
                    })
                    .map(|ax| ax.max.abs().max(ax.min.abs()))
                    .fold(f64::MAX, f64::min)
            })
            .unwrap_or(f64::MAX);
        let axis_budget = if axis_budget == f64::MAX || axis_budget < 1.0 {
            // No usable axis definition: fall back to the head envelope alone.
            f64::MAX
        } else {
            axis_budget
        };

        let head_budget = profile
            .map(|p| p.max_tool_tilt_deg)
            .filter(|t| t.is_finite() && *t > 0.0)
            .unwrap_or_else(profile_default_max_tool_tilt);

        let budget = axis_budget.min(head_budget);
        // Never return something unusable; a zero budget would pin every
        // segment vertical and silently disable non-planar printing.
        if budget.is_finite() && budget >= 1.0 {
            budget.min(90.0)
        } else {
            profile_default_max_tool_tilt()
        }
    }

    pub fn apply_surface_normal_orientations(&mut self) {
        if matches!(self.slicing_mode, SlicingMode::Conical) { return; }
        if !self.use_surface_normals { return; }

        let mesh = match &self.mesh { Some(m) => m, None => return };
        if self.toolpaths.is_empty() { return; }

        // ── Where the orientations come from ──────────────────────────────
        //
        // The tool should point perpendicular to the surface being PRINTED,
        // which is the layer. When the slicer knows the layer normals --
        // marching tetrahedra computes them from the scalar field gradient,
        // which is exactly the isosurface normal -- use those.
        //
        // Otherwise fall back to the nearest model triangle. That fallback is
        // only an approximation and it is wrong wherever the layer cuts
        // across the model rather than along it: at the base of a vertical
        // wall the nearest triangle IS the wall, whose normal is horizontal,
        // so a flat bottom layer came out tilted to the limit. On a Benchy
        // the first extrusion was commanded at 44.9 degrees with nothing
        // below it but the bed.
        let mut centroids: Vec<[f64; 3]> = Vec::new();
        let mut face_normals: Vec<crate::geometry::Vector3D> = Vec::new();

        let layer_points: usize = self
            .layers
            .iter()
            .flat_map(|l| l.contours.iter())
            .filter(|c| c.normals.is_some())
            .map(|c| c.points.len())
            .sum();

        let from_layers = layer_points > 0;
        if from_layers {
            centroids.reserve(layer_points);
            face_normals.reserve(layer_points);
            for layer in &self.layers {
                for contour in &layer.contours {
                    let Some(ns) = contour.normals.as_ref() else { continue };
                    for (p, n) in contour.points.iter().zip(ns.iter()) {
                        centroids.push([p.x, p.y, p.z]);
                        let mut n = *n;
                        if n.z < 0.0 { n = -n; } // always upper hemisphere
                        face_normals.push(n);
                    }
                }
            }
        } else {
            let n_tris = mesh.triangles.len();
            centroids.reserve(n_tris);
            face_normals.reserve(n_tris);
            for tri in &mesh.triangles {
                let cx = (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0;
                let cy = (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0;
                let cz = (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0;
                centroids.push([cx, cy, cz]);

                let mut n = tri.normal();
                if n.z < 0.0 { n = -n; } // always upper hemisphere
                face_normals.push(n);
            }
        }
        if centroids.is_empty() { return; }

        let x_min = mesh.bounds_min.x;
        let y_min = mesh.bounds_min.y;
        let x_range = (mesh.bounds_max.x - x_min).max(1.0);
        let y_range = (mesh.bounds_max.y - y_min).max(1.0);
        let nx: usize = 48;
        let ny: usize = 48;
        let bsx = x_range / nx as f64;
        let bsy = y_range / ny as f64;

        let mut bins: Vec<Vec<usize>> = vec![Vec::new(); nx * ny];
        for (i, c) in centroids.iter().enumerate() {
            let bx = ((c[0] - x_min) / bsx).floor() as isize;
            let by = ((c[1] - y_min) / bsy).floor() as isize;
            let bx = bx.clamp(0, nx as isize - 1) as usize;
            let by = by.clamp(0, ny as isize - 1) as usize;
            bins[by * nx + bx].push(i);
        }

        // Helper: find nearest-triangle normal for a query point
        let nearest_normal = |qx: f64, qy: f64, qz: f64| -> crate::geometry::Vector3D {
            let bx0 = ((qx - x_min) / bsx).floor() as isize;
            let by0 = ((qy - y_min) / bsy).floor() as isize;
            let mut best_d2 = f64::MAX;
            let mut best_n = crate::geometry::Vector3D::new(0.0, 0.0, 1.0);
            // Expand search ring until we find at least one candidate
            for radius in 0..=4isize {
                for dy in -radius..=radius {
                    for dx in -radius..=radius {
                        if radius > 0 && dx.abs() != radius && dy.abs() != radius { continue; }
                        let bx = (bx0 + dx).clamp(0, nx as isize - 1) as usize;
                        let by = (by0 + dy).clamp(0, ny as isize - 1) as usize;
                        for &ti in &bins[by * nx + bx] {
                            let c = centroids[ti];
                            let d2 = (c[0]-qx)*(c[0]-qx) + (c[1]-qy)*(c[1]-qy) + (c[2]-qz)*(c[2]-qz);
                            if d2 < best_d2 { best_d2 = d2; best_n = face_normals[ti]; }
                        }
                    }
                }
                if best_d2 < f64::MAX { break; }
            }
            best_n
        };

        // Axis travel AND the head's physical tilt envelope, whichever binds.
        let max_tilt_deg = self.tool_tilt_budget_deg(true);
        let max_tilt_rad = max_tilt_deg.to_radians();

        // ── Assign orientations ───────────────────────────────────────────
        let mut count = 0usize;
        for toolpath in &mut self.toolpaths {
            for segment in &mut toolpath.paths {
                let mut n = nearest_normal(
                    segment.position.x,
                    segment.position.y,
                    segment.position.z,
                );

                // Normalise (already unit from Triangle::normal, but be safe)
                let len = n.norm();
                if len < 1e-6 { continue; }
                n /= len;

                // Clamp tilt to max rotary axis range
                let tilt = n.z.clamp(-1.0, 1.0).acos(); // radians from vertical
                let orientation = if tilt > max_tilt_rad {
                    let xy = (n.x * n.x + n.y * n.y).sqrt();
                    if xy > 1e-6 {
                        let s = max_tilt_rad.sin() / xy;
                        crate::geometry::Vector3D::new(n.x * s, n.y * s, max_tilt_rad.cos())
                    } else {
                        crate::geometry::Vector3D::new(0.0, 0.0, 1.0)
                    }
                } else {
                    n
                };

                segment.orientation = orientation;
                count += 1;
            }
        }

        self.log(format!(
            "Oriented {} segments from {} {} (max tilt {:.0}°)",
            count,
            centroids.len(),
            if from_layers { "layer normals" } else { "model triangles" },
            max_tilt_deg
        ));
    }

    /// Lift non-extruding (travel) moves by `travel_z_lift` mm above the last extrusion Z.
    ///
    /// This ensures the nozzle clears already-printed material when traversing between
    /// islands, preventing collisions and surface scarring.
    ///
    /// Only travels with XY displacement ≥ 1 mm are lifted (short hops are left as-is
    /// to avoid excessive Z oscillation within a dense infill region).
    pub fn apply_travel_lifts(&mut self) {
        let lift = self.travel_z_lift;
        if lift < 0.01 { return; }

        const MIN_XY: f64 = 1.0; // mm — ignore micro-travels within a region

        // Build a HeightMap to track deposited material so travel moves can
        // clear tall features along the XY corridor, not just the last extrusion Z.
        let bead_r = (self.nozzle_diameter / 2.0).max(0.2);
        let cell_size = (self.nozzle_diameter * 3.0).clamp(2.0, 10.0);
        let (x0, y0, x1, y1) = if let Some(mesh) = &self.mesh {
            let margin = 20.0;
            (mesh.bounds_min.x - margin, mesh.bounds_min.y - margin,
             mesh.bounds_max.x + margin, mesh.bounds_max.y + margin)
        } else {
            (-220.0, -220.0, 220.0, 220.0)
        };
        let mut hmap = HeightMap::new(x0, y0, x1, y1, cell_size);

        for toolpath in &mut self.toolpaths {
            let mut last_extrude_z = toolpath.z;
            let mut last_extrude_x = 0.0_f64;
            let mut last_extrude_y = 0.0_f64;

            let old_paths = std::mem::take(&mut toolpath.paths);
            let mut new_paths: Vec<crate::toolpath::ToolpathSegment> =
                Vec::with_capacity(old_paths.len() + 4);

            let mut prev_was_lifted = false;

            for mut seg in old_paths {
                if seg.extrusion > 1e-8 {
                    // First extrusion after a lifted travel: insert a non-extruding descent
                    // so the nozzle reaches print height before extruding begins.
                    if prev_was_lifted {
                        let mut descend = seg.clone();
                        descend.extrusion = 0.0;
                        new_paths.push(descend);
                    }
                    // Update HeightMap with this deposited extrusion
                    hmap.update_circle(seg.position.x, seg.position.y, seg.position.z, bead_r);
                    last_extrude_z = seg.position.z;
                    last_extrude_x = seg.position.x;
                    last_extrude_y = seg.position.y;
                    prev_was_lifted = false;
                } else {
                    // Travel move: lift Z to clear all deposited material along the XY corridor
                    let dx = seg.position.x - last_extrude_x;
                    let dy = seg.position.y - last_extrude_y;
                    if (dx * dx + dy * dy).sqrt() >= MIN_XY {
                        // Sample HeightMap along the travel corridor
                        let corridor_max_z = hmap.max_z_along_line(
                            last_extrude_x, last_extrude_y,
                            seg.position.x, seg.position.y,
                        );
                        // Lift above the higher of: last extrusion Z or corridor max Z
                        let reference_z = if corridor_max_z > f64::NEG_INFINITY {
                            last_extrude_z.max(corridor_max_z)
                        } else {
                            last_extrude_z
                        };
                        let lifted_z = seg.position.z.max(reference_z + lift);
                        if lifted_z > seg.position.z + 1e-6 {
                            seg.position.z = lifted_z;
                            prev_was_lifted = true;
                        } else {
                            prev_was_lifted = false;
                        }
                    } else {
                        prev_was_lifted = false;
                    }
                }
                new_paths.push(seg);
            }

            toolpath.paths = new_paths;
        }

        self.log(format!("Applied {:.1} mm HeightMap-aware travel lift to all toolpaths", lift));
    }

    /// After surface-normal (or conical) orientations have been assigned, check whether the
    /// tilted printhead body would physically intersect previously deposited extrusions and
    /// adjust the tilt azimuth — or reduce the tilt magnitude — until `head_clearance_mm` of
    /// clearance is maintained.
    ///
    /// ## Algorithm
    /// Iterates all toolpath segments **in printing order** (bottom layer first, within each
    /// layer in path order).  For each segment:
    ///
    /// 1. **Collision check** — Given the current orientation (θ, φ) where θ = tilt from
    ///    vertical and φ = azimuth in XY, compute the head body's XY footprint and query the
    ///    incremental height map for maximum previously-deposited Z in that region.  The head
    ///    body bottom is at `nozzle_z + (tcp − head_h/2) × cos(θ)`.  If the height map max >
    ///    head_body_bottom − clearance, a collision is predicted.
    ///
    /// 2. **Azimuth search** — Try 16 candidate azimuths (φ + k × 22.5°, k = 1..15) with the
    ///    same tilt magnitude, pick the one with the smallest collision depth.  If any is
    ///    collision-free, use it.
    ///
    /// 3. **Tilt reduction** — If no azimuth is clear, reduce θ in 5° steps down to 0°
    ///    (vertical), each time trying all 16 azimuths.  Vertical is always safe.
    ///
    /// 4. **Height map update** — After assigning the final orientation, update the
    ///    height map with a circle of radius = line_width/2 at the segment's XY position.
    ///    Only extrusion segments (e > 0) are added; travel moves are not deposited material.
    pub fn apply_tilt_collision_avoidance(&mut self) {
        if !self.avoid_head_collision { return; }

        let profile = match self.profiles.get(self.active_profile_index) {
            Some(p) => p.clone(),
            None => return,
        };

        let tcp   = profile.tcp_offset.max(1.0);
        let head_w = profile.head_dims[0];
        let head_d = profile.head_dims[1];
        let head_h = profile.head_dims[2];
        let clearance = self.head_clearance_mm;

        // Guard: if TCP offset is smaller than half the head body height the head body
        // extends below the nozzle tip — physically invalid geometry that makes every
        // segment appear to collide, causing the algorithm to run for trillions of
        // iterations.  Skip with a clear log message so the user can fix their profile.
        let min_valid_tcp = head_h / 2.0 + clearance + 5.0;
        if tcp < min_valid_tcp {
            self.log(format!(
                "Head-body collision avoidance skipped: TCP offset ({:.1} mm) must be \
                 ≥ {:.1} mm (head_height/2 + clearance + 5 mm safety margin). \
                 Increase the TCP offset in the active printer profile.",
                tcp, min_valid_tcp
            ));
            return;
        }

        // Bounding radius of head body in XY (conservative for any yaw orientation)
        let head_half_diag = ((head_w / 2.0).powi(2) + (head_d / 2.0).powi(2)).sqrt();
        // Bead deposit radius (half the line width)
        let bead_r = (self.nozzle_diameter / 2.0).max(0.2);

        // Build height map covering the mesh footprint + generous margin
        let margin = (head_half_diag + tcp).max(80.0);
        let (x0, y0, x1, y1) = if let Some(mesh) = &self.mesh {
            (mesh.bounds_min.x - margin, mesh.bounds_min.y - margin,
             mesh.bounds_max.x + margin, mesh.bounds_max.y + margin)
        } else {
            (-200.0 - margin, -200.0 - margin, 200.0 + margin, 200.0 + margin)
        };
        // Cell size must be proportional to the head body so that max_z_in_circle only
        // iterates a handful of cells. ~3 cells across the head footprint diameter is
        // plenty accurate (head body is 60mm wide → cell ≈14mm → 3 cells across).
        // Keeping it small relative to nozzle diameter gives no benefit here and would
        // make the grid enormous, turning every query into thousands of cell lookups.
        let cell_size = (head_half_diag / 3.0).max(8.0).min(25.0);
        let mut hmap = HeightMap::new(x0, y0, x1, y1, cell_size);

        // Axis travel AND the head's physical tilt envelope, whichever binds.
        let max_tilt_rad = {
            self.tool_tilt_budget_deg(true).to_radians()
        };

        // Pre-compute 16 azimuth offsets (0°, 22.5°, 45°, …)
        const N_PHI: usize = 16;
        let phi_offsets: [f64; N_PHI] = {
            let mut arr = [0.0f64; N_PHI];
            for k in 0..N_PHI {
                arr[k] = (k as f64) * (2.0 * std::f64::consts::PI / N_PHI as f64);
            }
            arr
        };

        let mut adjusted_count = 0usize;

        // Pre-extract head STL vertices from the active printer profile.
        // The profile stores a file path; we load it once here and flatten to a Vec so the
        // hot collision loop only does simple array indexing.
        // head_stl_tip_offset is the LOCAL position of the nozzle tip inside that STL — we
        // subtract it so the nozzle tip is at the local origin before applying the rotation.
        // Falls back to the bounding-box cylinder approximation if no STL is configured.
        // Pre-extract head STL vertices — capped at MAX_HEAD_VERTS so the per-segment
        // collision check stays O(1) regardless of mesh complexity.  Representative
        // downsampling (every Nth vertex) is sufficient; the head body is tens of mm
        // wide so fine surface detail adds no collision-detection value.
        const MAX_HEAD_VERTS: usize = 300;
        let tip = profile.head_stl_tip_offset;
        let head_verts: Option<Vec<[f64; 3]>> = profile.head_stl_path.as_deref()
            .and_then(|p| Mesh::from_stl(std::path::Path::new(p)).ok())
            .map(|mesh| {
                // Collect all vertices shifted to nozzle-tip-at-origin frame
                let all: Vec<[f64; 3]> = mesh.triangles.iter()
                    .flat_map(|t| [t.v0, t.v1, t.v2])
                    .map(|p| [p.x - tip[0], p.y - tip[1], p.z - tip[2]])
                    .collect();
                // Downsample if necessary
                if all.len() <= MAX_HEAD_VERTS {
                    all
                } else {
                    let step = all.len() / MAX_HEAD_VERTS;
                    all.into_iter().step_by(step).take(MAX_HEAD_VERTS).collect()
                }
            });

        self.log(format!(
            "Head-body collision avoidance: {} segments, {} (cell {:.1}mm, tcp {:.1}mm)",
            self.toolpaths.iter().map(|t| t.paths.len()).sum::<usize>(),
            match &head_verts {
                Some(v) => format!("head STL ({} probe points)", v.len()),
                None    => format!("bounding box {:.0}×{:.0}×{:.0}mm",
                                   head_w, head_d, head_h),
            },
            cell_size, tcp,
        ));

        // Closure: collision depth at a specific (theta, phi).
        // Returns > 0 if there is a collision, ≤ 0 if clear.
        //
        // When a head STL is loaded, each vertex is rotated from the tool's local frame
        // to world space and tested against the height map individually.  The rotation maps
        // local +Z (nozzle-to-body axis) onto the world tool direction:
        //   ex = (cos θ·cos φ,  cos θ·sin φ, −sin θ)   ← local X
        //   ey = (     −sin φ,       cos φ,       0 )   ← local Y
        //   ez = (sin θ·cos φ,  sin θ·sin φ,  cos θ)   ← local Z (tool direction)
        //
        // Without a head STL, the original bounding-box cylinder approximation is used.
        let collision_depth = |hmap: &HeightMap, nox: f64, noy: f64, noz: f64,
                                theta: f64, phi: f64| -> f64 {
            if let Some(ref verts) = head_verts {
                let ct = theta.cos(); let st = theta.sin();
                let cp = phi.cos();   let sp = phi.sin();
                let mut max_depth = f64::NEG_INFINITY;
                for v in verts {
                    let (vx, vy, vz) = (v[0], v[1], v[2]);
                    // Rotate local vertex into world space (relative to nozzle tip)
                    let wx = nox + vx * (ct * cp) - vy * sp + vz * (st * cp);
                    let wy = noy + vx * (ct * sp) + vy * cp + vz * (st * sp);
                    let wz = noz - vx * st                  + vz * ct;
                    let depth = hmap.get(wx, wy) - (wz - clearance);
                    if depth > max_depth { max_depth = depth; }
                }
                max_depth
            } else {
                // Bounding-box cylinder approximation
                let sin_t = theta.sin();
                let pivot_x = nox + sin_t * phi.cos() * tcp;
                let pivot_y = noy + sin_t * phi.sin() * tcp;
                let head_bottom_z = noz + (tcp - head_h / 2.0) * theta.cos();
                let max_h = hmap.max_z_in_circle(pivot_x, pivot_y, head_half_diag);
                max_h - (head_bottom_z - clearance)
            }
        };

        // Phase F: singularity avoidance config
        let sing_enabled = self.enable_singularity_avoidance;
        let sing_threshold = self.singularity_threshold;

        // Score function (Nishat et al. 2024, Heuristic 1) with Phase D look-ahead
        // and Phase F singularity penalty:
        // - Collision-free candidates scored by 3D angular distance from the previous
        //   orientation (0…π), ensuring smooth tool-vector transitions between segments.
        // - Look-ahead: also penalizes large angular jumps to upcoming segments.
        // - Singularity: penalizes orientations near gimbal-lock configurations.
        // - Colliding candidates always score ≥ 100, so any collision-free option beats
        //   any colliding one regardless of angular distance.
        let score = |hmap: &HeightMap, nox: f64, noy: f64, noz: f64,
                      t: f64, p: f64, pt: f64, pp: f64,
                      future_orientations: &[(f64, f64, f64)]| -> f64 {
            let d = collision_depth(hmap, nox, noy, noz, t, p);
            if d <= 0.0 {
                let dot = (pt.sin() * t.sin() * (p - pp).cos()
                         + pt.cos() * t.cos()).clamp(-1.0, 1.0);
                let base = dot.acos(); // [0, π] — lower = smaller angular change

                // Look-ahead penalty: penalize if candidate creates large jumps to future segments
                let mut penalty = 0.0;
                for &(ft, fp, dist_to) in future_orientations {
                    let f_dot = (ft.sin() * t.sin() * (p - fp).cos()
                               + ft.cos() * t.cos()).clamp(-1.0, 1.0);
                    let angle_to_future = f_dot.acos();
                    // Allow 15°/mm of distance to the future segment
                    let allowed = 15.0f64.to_radians() * dist_to;
                    if angle_to_future > allowed {
                        penalty += (angle_to_future - allowed) * 0.5;
                    }
                }

                // Singularity penalty: |sin(theta)| measures manipulability for tilt-rotate systems
                if sing_enabled {
                    let manipulability = t.sin().abs();
                    if manipulability < sing_threshold {
                        penalty += (sing_threshold - manipulability) * 10.0;
                    }
                }

                base + penalty
            } else {
                100.0 + d
            }
        };

        let look_ahead = self.look_ahead_window;

        // Pre-collect all segment orientations and positions for look-ahead access.
        // Structure: Vec<(toolpath_idx, seg_idx, theta, phi, x, y, z)>
        let mut all_seg_info: Vec<(usize, usize, f64, f64, f64, f64, f64)> = Vec::new();
        for (ti, tp) in self.toolpaths.iter().enumerate() {
            for (si, seg) in tp.paths.iter().enumerate() {
                let ori = seg.orientation;
                let len = ori.norm();
                if len > 1e-6 {
                    let theta = (ori.z / len).clamp(-1.0, 1.0).acos();
                    let phi = ori.y.atan2(ori.x);
                    all_seg_info.push((ti, si, theta, phi, seg.position.x, seg.position.y, seg.position.z));
                }
            }
        }

        // Track previous segment orientation for angular continuity.
        let mut prev_theta = 0.0f64;
        let mut prev_phi   = 0.0f64;

        // Flat index into all_seg_info for look-ahead
        let mut flat_idx = 0usize;

        for toolpath in &mut self.toolpaths {
            for segment in &mut toolpath.paths {
                let nx = segment.position.x;
                let ny = segment.position.y;
                let nz = segment.position.z;

                // Decompose current orientation into (theta, phi)
                let ori = segment.orientation;
                let len = ori.norm();
                if len < 1e-6 {
                    if segment.extrusion > 1e-8 {
                        hmap.update_circle(nx, ny, nz, bead_r);
                    }
                    continue;
                }
                let oz_unit = (ori.z / len).clamp(-1.0, 1.0);
                let theta = oz_unit.acos();
                let phi   = ori.y.atan2(ori.x);

                // Build look-ahead data: orientations of the next `look_ahead` segments
                let mut future_oris: Vec<(f64, f64, f64)> = Vec::new();
                if look_ahead > 0 && flat_idx < all_seg_info.len() {
                    for fi in (flat_idx + 1)..all_seg_info.len().min(flat_idx + 1 + look_ahead) {
                        let (_, _, ft, fp, fx, fy, fz) = all_seg_info[fi];
                        let dx = fx - nx;
                        let dy = fy - ny;
                        let dz = fz - nz;
                        let dist = (dx * dx + dy * dy + dz * dz).sqrt().max(0.1);
                        future_oris.push((ft, fp, dist));
                    }
                }
                // Advance flat index (it tracks segments with valid orientations)
                if flat_idx < all_seg_info.len() {
                    flat_idx += 1;
                }

                // --- 1. Check collision at current orientation ---
                let current_depth = collision_depth(&hmap, nx, ny, nz, theta, phi);

                if current_depth > 0.0 {
                    let mut best_score = f64::MAX;
                    let mut best_theta = theta;
                    let mut best_phi   = phi;

                    // Quick escape: check whether the vertical orientation (θ=0) also
                    // collides.  If it does, no tilt adjustment can possibly clear the
                    // head body — this almost always means an out-of-order floating
                    // contour is being printed beneath already-deposited material (the
                    // height map shows z=80mm while the nozzle is at z=5mm, for
                    // example).  Running the full 160-candidate search would waste time
                    // and still fail, so skip it and fall through to the vertical
                    // last-resort path below.
                    let vertical_also_collides =
                        collision_depth(&hmap, nx, ny, nz, 0.0, prev_phi) > 0.0;

                    if !vertical_also_collides {
                        // --- 2. Try all 16 azimuths at the same tilt magnitude.
                        // Score combines collision clearance + angular continuity from prev.
                        // Collision-free options (score < 100) always beat colliding ones.
                        for k in 0..N_PHI {
                            let p_try = phi + phi_offsets[k];
                            let s = score(&hmap, nx, ny, nz, theta, p_try, prev_theta, prev_phi, &future_oris);
                            if s < best_score {
                                best_score = s;
                                best_phi   = p_try;
                                best_theta = theta;
                            }
                        }

                        // --- 3. Reduce tilt in 5° steps if still colliding (score ≥ 100) ---
                        if best_score >= 100.0 {
                            let step = 5.0f64.to_radians();
                            let mut t = theta;
                            'reduce: while t > step {
                                t = (t - step).max(0.0);
                                let t_clamped = t.min(max_tilt_rad);
                                for k in 0..N_PHI {
                                    let p_try = phi + phi_offsets[k];
                                    let s = score(&hmap, nx, ny, nz, t_clamped, p_try,
                                                 prev_theta, prev_phi, &future_oris);
                                    if s < best_score {
                                        best_score = s;
                                        best_phi   = p_try;
                                        best_theta = t_clamped;
                                    }
                                    if s < 100.0 { break 'reduce; } // found collision-free
                                }
                                if t == 0.0 { break; }
                            }
                        }
                    }

                    // Vertical is always the last resort — keep azimuth continuous.
                    // Triggers when either: (a) the full search found no clear orientation,
                    // or (b) vertical was also colliding (floating contour fast-path).
                    if best_score >= 100.0 || vertical_also_collides {
                        best_theta = 0.0;
                        best_phi   = prev_phi;
                    }

                    // Apply the adjusted orientation
                    let st = best_theta.sin();
                    segment.orientation = crate::geometry::Vector3D::new(
                        st * best_phi.cos(),
                        st * best_phi.sin(),
                        best_theta.cos(),
                    );
                    adjusted_count += 1;
                }

                // --- 4. Update angular-continuity tracking ---
                let final_ori = segment.orientation;
                let flen = final_ori.norm();
                if flen > 1e-6 {
                    prev_theta = (final_ori.z / flen).clamp(-1.0, 1.0).acos();
                    prev_phi   = final_ori.y.atan2(final_ori.x);
                }

                // --- 5. Update height map with this bead ---
                if segment.extrusion > 1e-8 {
                    hmap.update_circle(nx, ny, nz, bead_r);
                }
            }
        }

        self.log(format!(
            "Head-body collision avoidance: adjusted {} segment(s) ({:.1} mm clearance, {:.0}×{:.0}×{:.0} mm head)",
            adjusted_count, clearance, head_w, head_d, head_h
        ));
    }

    /// Smooth toolpath orientations using a bidirectional angular-velocity-limited pass.
    /// Prevents sharp orientation jumps between consecutive extrusion segments that would
    /// cause jerky axis motion and mechanical wear.
    pub fn smooth_toolpath_orientations(&mut self) {
        if !self.enable_orientation_smoothing {
            return;
        }
        let rate_rad = self.orientation_smoothing_rate.to_radians(); // max rad per mm

        for tp in &mut self.toolpaths {
            let segs = &mut tp.paths;
            if segs.len() < 2 {
                continue;
            }

            // Collect indices of extrusion segments only
            let ext_indices: Vec<usize> = (0..segs.len())
                .filter(|&i| segs[i].extrusion > 0.0)
                .collect();
            if ext_indices.len() < 2 {
                continue;
            }

            // --- Forward pass ---
            let mut forward: Vec<Vector3D> = ext_indices.iter().map(|&i| segs[i].orientation).collect();
            for k in 1..forward.len() {
                let i0 = ext_indices[k - 1];
                let i1 = ext_indices[k];
                let dx = segs[i1].position.x - segs[i0].position.x;
                let dy = segs[i1].position.y - segs[i0].position.y;
                let dz = segs[i1].position.z - segs[i0].position.z;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-6);
                let max_angle = rate_rad * dist;

                let prev = forward[k - 1];
                let cur = forward[k];
                let dot = prev.dot(&cur).clamp(-1.0, 1.0);
                let angle = dot.acos();

                if angle > max_angle && angle > 1e-9 {
                    let t = max_angle / angle;
                    // SLERP: interpolate cur toward prev
                    forward[k] = slerp_vec3(prev, cur, 1.0 - t);
                }
            }

            // --- Backward pass ---
            let mut backward: Vec<Vector3D> = ext_indices.iter().map(|&i| segs[i].orientation).collect();
            let n = backward.len();
            for k in (0..n - 1).rev() {
                let i0 = ext_indices[k];
                let i1 = ext_indices[k + 1];
                let dx = segs[i1].position.x - segs[i0].position.x;
                let dy = segs[i1].position.y - segs[i0].position.y;
                let dz = segs[i1].position.z - segs[i0].position.z;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-6);
                let max_angle = rate_rad * dist;

                let next = backward[k + 1];
                let cur = backward[k];
                let dot = next.dot(&cur).clamp(-1.0, 1.0);
                let angle = dot.acos();

                if angle > max_angle && angle > 1e-9 {
                    let t = max_angle / angle;
                    backward[k] = slerp_vec3(next, cur, 1.0 - t);
                }
            }

            // --- Merge: midpoint SLERP of forward and backward ---
            for k in 0..ext_indices.len() {
                let merged = slerp_vec3(forward[k], backward[k], 0.5);
                let norm = merged.norm();
                if norm > 1e-9 {
                    segs[ext_indices[k]].orientation = merged / norm;
                }
            }
        }
    }

    /// Post-process toolpath segments generated by conical slicing to assign the correct
    /// cone-surface normal as each segment's orientation.  This drives the rotary axes so
    /// the tool tilts perpendicular to the cone, avoiding collisions with the print surface.
    ///
    /// Formula derivation:
    ///   Outward (sign = -1 in conical.rs): world-space z = z_slice + r·tan α
    ///     ∂z/∂r = +tan α  →  unnorm normal = (−sin α·dx/r, −sin α·dy/r, cos α)
    ///   Inward  (sign = +1 in conical.rs): world-space z = z_slice − r·tan α
    ///     ∂z/∂r = −tan α  →  unnorm normal = (+sin α·dx/r, +sin α·dy/r, cos α)
    pub fn apply_conical_orientations(&mut self) {
        if !matches!(self.slicing_mode, SlicingMode::Conical) {
            return;
        }

        // Max tilt the machine can actually execute: the tightest rotary axis AND
        // the head's physical tilt envelope, whichever binds.
        let max_tilt_rad = self.tool_tilt_budget_deg(false).to_radians();

        // Clamp the cone half-angle to the machine limit.
        let alpha_rad = self.conical_angle_degrees.to_radians().min(max_tilt_rad);
        let sin_a = alpha_rad.sin();
        let cos_a = alpha_rad.cos();

        // Outward: world z increases with r → radial normal component points inward (−)
        // Inward:  world z decreases with r → radial normal component points outward (+)
        let radial_sign = match self.conical_direction {
            crate::conical::ConicalDirection::Outward => -1.0_f64,
            crate::conical::ConicalDirection::Inward  =>  1.0_f64,
        };

        let cx = if self.conical_auto_center {
            self.mesh.as_ref()
                .map(|m| (m.bounds_min.x + m.bounds_max.x) / 2.0)
                .unwrap_or(0.0)
        } else {
            self.conical_center_x
        };
        let cy = if self.conical_auto_center {
            self.mesh.as_ref()
                .map(|m| (m.bounds_min.y + m.bounds_max.y) / 2.0)
                .unwrap_or(0.0)
        } else {
            self.conical_center_y
        };

        let mut count = 0usize;
        for toolpath in &mut self.toolpaths {
            for segment in &mut toolpath.paths {
                let dx = segment.position.x - cx;
                let dy = segment.position.y - cy;
                let r = (dx * dx + dy * dy).sqrt();

                if r < 0.5 {
                    // Near the axis: keep vertical to avoid numerical instability
                    segment.orientation = crate::geometry::Vector3D::new(0.0, 0.0, 1.0);
                } else {
                    let nx = radial_sign * sin_a * dx / r;
                    let ny = radial_sign * sin_a * dy / r;
                    // Normal is already unit-length: |n|² = sin²α + cos²α = 1
                    segment.orientation = crate::geometry::Vector3D::new(nx, ny, cos_a);
                }
                count += 1;
            }
        }

        self.log(format!(
            "Applied conical orientations: {:.1}° half-angle, {} total segments",
            self.conical_angle_degrees, count
        ));
    }

    /// Compute head↔bed AABB collision for every toolpath segment and store the result.
    ///
    /// Uses the AABB-of-OBB formula so each box's axis-aligned bounding box is computed
    /// after applying the kinematic rotation, then the two AABBs are checked for overlap.
    pub fn compute_all_collisions(&mut self) {
        let profile = match self.profiles.get(self.active_profile_index) {
            Some(p) => p.clone(),
            None => { self.collision_segments.clear(); return; }
        };

        if !profile.show_machine {
            self.collision_segments.clear();
            return;
        }

        // Bottom Z of the print — needed to place the bed in world space.
        let grid_z: f32 = self.toolpaths.iter()
            .flat_map(|tp| tp.paths.iter())
            .map(|s| s.position.z as f32)
            .fold(f32::INFINITY, f32::min);
        let grid_z = if grid_z.is_infinite() { 0.0 } else { grid_z };

        let bed_full_h = profile.bed_dims[2] as f32;
        let bed_travel_z = profile.bed_travel[2] as f32;
        let bed_pivot_xy = [profile.bed_pivot[0] as f32, profile.bed_pivot[1] as f32];

        // Nozzle length = distance from head pivot to nozzle tip (same as viewport).
        let nl = profile.nozzle_length as f32;
        let head_h = profile.head_dims[2] as f32;

        // OBB half-extents (head body only — not including the nozzle cylinder below pivot).
        let head_he = [
            profile.head_dims[0] as f32 / 2.0,
            profile.head_dims[1] as f32 / 2.0,
            head_h / 2.0,
        ];
        let bed_he = [
            profile.bed_dims[0] as f32 / 2.0,
            profile.bed_dims[1] as f32 / 2.0,
            bed_full_h / 2.0,
        ];

        let gcode_axes = self.build_gcode_axes();

        let total: usize = self.toolpaths.iter().map(|tp| tp.paths.len()).sum();
        let mut results = vec![false; total];

        let mut flat_idx = 0usize;
        for toolpath in &self.toolpaths {
            for segment in &toolpath.paths {
                let o = &segment.orientation;
                let angles = self.compute_segment_axis_angles(o);

                // Per-segment bed Z: matches viewport logic (bed lowers as print height increases).
                let seg_z = segment.position.z as f32;
                let bed_pivot_z = if bed_travel_z > 0.0 {
                    let dz = (seg_z - grid_z).max(0.0);
                    grid_z - bed_full_h - dz.min(bed_travel_z)
                } else {
                    grid_z - bed_full_h
                };
                let bed_world_pivot = [bed_pivot_xy[0], bed_pivot_xy[1], bed_pivot_z];

                // ── Bed rotation (chained Rodrigues over bed axes) ──────────
                let bed_joints: Vec<([f32; 3], f32)> = gcode_axes.iter()
                    .zip(angles.iter())
                    .filter(|(ax, _)| !ax.is_head)
                    .map(|(ax, (_, deg))| {
                        let ra = ax.rotation_axis;
                        let clamped = deg.clamp(ax.min_deg, ax.max_deg);
                        ([ra[0] as f32, ra[1] as f32, ra[2] as f32], clamped.to_radians() as f32)
                    })
                    .collect();
                let rb = chained_rotation_f32(&bed_joints);

                // ── Nozzle world position (print point after bed tilt) ──────
                let seg_pos = [
                    segment.position.x as f32,
                    segment.position.y as f32,
                    segment.position.z as f32,
                ];
                let rel = [
                    seg_pos[0] - bed_world_pivot[0],
                    seg_pos[1] - bed_world_pivot[1],
                    seg_pos[2] - bed_world_pivot[2],
                ];
                let rotated = rot3_apply(rb, rel);
                let nozzle_world = [
                    rotated[0] + bed_world_pivot[0],
                    rotated[1] + bed_world_pivot[1],
                    rotated[2] + bed_world_pivot[2],
                ];

                // ── Residual orientation the head must achieve ──────────────
                let rb_t = mat3_transpose_f32(rb);
                let o_vec = [o.x as f32, o.y as f32, o.z as f32];
                let residual = rot3_apply(rb_t, o_vec);

                // ── Head rotation (same IK as viewport) ─────────────────────
                let head_joints: Vec<([f32; 3], f32)> = gcode_axes.iter()
                    .filter(|ax| ax.is_head)
                    .map(|ax| {
                        let [rx, ry, _rz] = ax.rotation_axis;
                        let angle_deg = if rx.abs() > 0.7 {
                            (residual[1] as f64).atan2(residual[2] as f64).to_degrees()
                        } else if ry.abs() > 0.7 {
                            (-(residual[0] as f64)).atan2(residual[2] as f64).to_degrees()
                        } else {
                            (residual[1] as f64).atan2(residual[0] as f64).to_degrees()
                        };
                        let clamped = angle_deg.clamp(ax.min_deg, ax.max_deg).to_radians() as f32;
                        ([rx as f32, ry as f32, _rz as f32], clamped)
                    })
                    .collect();
                let rh = chained_rotation_f32(&head_joints);

                // ── Head pivot and OBB center ───────────────────────────────
                // pivot = nozzle_world + rh * (0, 0, nl)  [nozzle tip is at rh*(0,0,-nl)+pivot]
                let pivot_offset = rot3_apply(rh, [0.0, 0.0, nl]);
                let pivot_head = [
                    nozzle_world[0] + pivot_offset[0],
                    nozzle_world[1] + pivot_offset[1],
                    nozzle_world[2] + pivot_offset[2],
                ];
                // Head OBB center = pivot + rh*(0, 0, head_h/2)  [box bottom at pivot, top at pivot+head_h]
                let head_box_offset = rot3_apply(rh, [0.0, 0.0, head_h / 2.0]);
                let head_center = [
                    pivot_head[0] + head_box_offset[0],
                    pivot_head[1] + head_box_offset[1],
                    pivot_head[2] + head_box_offset[2],
                ];

                // ── Bed OBB center ──────────────────────────────────────────
                // Bed box bottom at bed_world_pivot, top at +bed_full_h in local Z.
                let bed_box_offset = rot3_apply(rb, [0.0, 0.0, bed_full_h / 2.0]);
                let bed_center = [
                    bed_world_pivot[0] + bed_box_offset[0],
                    bed_world_pivot[1] + bed_box_offset[1],
                    bed_world_pivot[2] + bed_box_offset[2],
                ];

                // ── OBB overlap: check in bed's local frame ─────────────────
                // Express the head box in the bed's coordinate frame.
                // This avoids the large world-AABB that a tilted bed produces (a
                // 200 mm bed at 45° has ±74 mm Z AABB, causing massive false positives).
                // Head center in bed frame
                let hc_rel = [
                    head_center[0] - bed_world_pivot[0],
                    head_center[1] - bed_world_pivot[1],
                    head_center[2] - bed_world_pivot[2],
                ];
                let hc_bed = rot3_apply(rb_t, hc_rel);
                // Head OBB rotation in bed frame: rb_t * rh
                let rh_in_bed = mat3_mul_f32(rb_t, rh);
                // AABB half-extents of head OBB expressed in bed frame
                let head_aabb_bed = aabb_of_obb(head_he, rh_in_bed);
                // Bed occupies [−bw/2 .. +bw/2, −bd/2 .. +bd/2, 0 .. bh] in its own frame
                let collides = (hc_bed[0].abs() <= head_aabb_bed[0] + bed_he[0])
                    && (hc_bed[1].abs() <= head_aabb_bed[1] + bed_he[1])
                    && (hc_bed[2] - bed_full_h / 2.0).abs() <= head_aabb_bed[2] + bed_he[2];
                results[flat_idx] = collides;
                flat_idx += 1;
            }
        }

        let collision_count = results.iter().filter(|&&c| c).count();
        if collision_count > 0 {
            self.log(format!("⚠️ Machine simulation: {} collision segments detected", collision_count));
        }
        self.collision_segments = results;
    }

    /// For every toolpath segment where the tool body (capsule from nozzle tip along the
    /// tool orientation) would intersect the mesh, search for a collision-free orientation
    /// by sweeping the tilt angle radially outward from the cone/mesh center.
    ///
    /// This is the active avoidance counterpart to `compute_all_collisions` (which only marks).
    /// Called after toolpaths are generated when the mesh is available.
    pub fn optimize_toolpath_orientations_for_mesh(&mut self) {
        let mesh = match self.mesh.as_ref() {
            Some(m) => m.clone(),
            None => return,
        };
        let profile = match self.profiles.get(self.active_profile_index) {
            Some(p) => p.clone(),
            None => return,
        };

        // Must have at least one rotary head axis to benefit from orientation changes.
        let has_head_rotary = profile.rotary_axes.iter().any(|ax| {
            ax.axis_type == AxisType::Rotary && ax.moving_part == MovingPart::Head
        });
        if !has_head_rotary { return; }

        // Approximate head geometry from profile dimensions.
        let body_radius = (profile.head_dims[0].min(profile.head_dims[1]) / 2.0)
            .max(profile.nozzle_radius * 2.0)
            .max(5.0);
        let body_height = self.tcp_offset + profile.head_dims[2];

        let mut detector = crate::motion_planning::collision::CollisionDetector::new(
            crate::motion_planning::collision::PrintHead {
                nozzle_diameter: 0.4,
                body_radius,
                body_height,
                nozzle_clearance: profile.nozzle_clearance,
            },
            (
                crate::geometry::Point3D::new(-500.0, -500.0, -50.0),
                crate::geometry::Point3D::new(500.0, 500.0, 1000.0),
            ),
        );
        // Build the BVH ladder once; the loops below query it thousands of
        // times. Banded by height so a segment is only tested against
        // geometry that exists by the time it is printed -- testing against
        // the whole model made the head swerve around the part it was in the
        // middle of printing.
        // 4 bands rather than 8: the ladder is cumulative, so band i holds
        // i/n of the mesh and the whole ladder costs about n/2 copies to
        // build. On a Benchy 8 bands added ~57s; 4 keeps the benefit at half
        // the build cost, and the granularity only needs to be fine enough
        // that the head is not tested against material far above it.
        detector.set_collision_mesh_banded(&mesh, 4);

        // How far the tool may lean: the tightest rotary axis AND the head's
        // physical tilt envelope, whichever binds.
        //
        // This used to use the axis travel range alone. The default profile
        // lets A and B travel +/-180 degrees, so the escape search below was
        // free to rotate the tool all the way over -- measured G-code contained
        // tilts up to 177.98 degrees, i.e. the nozzle pointing into the bed,
        // on every single segment.
        let max_tilt_deg = self.tool_tilt_budget_deg(false);

        // Radial center for escape direction: use conical center or mesh centroid.
        let (cone_cx, cone_cy) = if matches!(self.slicing_mode, SlicingMode::Conical) {
            if self.conical_auto_center {
                let cx = (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0;
                let cy = (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0;
                (cx, cy)
            } else {
                (self.conical_center_x, self.conical_center_y)
            }
        } else {
            let cx = (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0;
            let cy = (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0;
            (cx, cy)
        };

        let tilt_step_deg = 5.0_f64;
        let steps = ((max_tilt_deg / tilt_step_deg).ceil() as usize + 1).max(1);

        // One BVH query per segment, plus up to `steps` more for each segment
        // that collides. Curved layers produce orders of magnitude more
        // segments than planar ones -- a 30 mm sphere gives ~146k, a Benchy far
        // more -- and this ran serially, which is why the pass took longer than
        // the slice it was processing. Every segment is independent and the BVH
        // is read-only, so the work parallelises exactly.
        use rayon::prelude::*;
        let n_fixed: usize = self
            .toolpaths
            .par_iter_mut()
            .map(|toolpath| {
                toolpath
                    .paths
                    .par_iter_mut()
                    .map(|seg| {
                        // Only geometry already deposited can be hit. The
                        // nozzle tip is at the surface being printed, so
                        // anything at or below it exists; anything above does
                        // not yet. Testing the whole model instead made the
                        // first extrusion on a Benchy come out at 44.9 degrees
                        // of tilt, dodging the hull it was printing.
                        let printed_to_z = seg.position.z;

                        // Skip if the current orientation already clears it.
                        if !detector.check_collision_with_printed(
                            &seg.position,
                            &seg.orientation,
                            printed_to_z,
                        ) {
                            return 0usize;
                        }

                        // Radial escape direction (away from cone/mesh center in XY).
                        let dx = seg.position.x - cone_cx;
                        let dy = seg.position.y - cone_cy;
                        let rxy = (dx * dx + dy * dy).sqrt();
                        let (ex, ey) = if rxy > 1e-6 {
                            (dx / rxy, dy / rxy)
                        } else {
                            (1.0_f64, 0.0_f64)
                        };

                        // Search outward from the CURRENT orientation in order of
                        // increasing angular deviation, accepting the first candidate
                        // that clears. This nudges a good surface-normal orientation
                        // minimally instead of snapping it back to vertical, which is
                        // what sweeping up from 0° used to do.
                        for trial in collision_escape_candidates(
                            seg.orientation,
                            (ex, ey),
                            tilt_step_deg,
                            max_tilt_deg,
                            steps,
                        ) {
                            if !detector.check_collision_with_printed(
                                &seg.position,
                                &trial,
                                printed_to_z,
                            ) {
                                // Only override when a clear angle was actually
                                // found. If every tilt still collides (e.g. we are
                                // inside the mesh at the bottom of the print), keep
                                // the original -- do NOT force max tilt.
                                seg.orientation = trial;
                                return 1usize;
                            }
                        }
                        0usize
                    })
                    .sum::<usize>()
            })
            .sum();

        if n_fixed > 0 {
            self.log(format!(
                "Collision avoidance: adjusted {} segment orientations to avoid mesh",
                n_fixed
            ));
        }
    }

    /// Build `Vec<GCodeAxis>` from the active profile for G-code generation.
    ///
    /// Only rotary axes are included (linear axes X/Y/Z are handled by the
    /// generator directly).  The profile's `AxisConfig` fields are converted to
    /// the leaner `GCodeAxis` form expected by `GCodeGenerator`.
    pub fn build_gcode_axes(&self) -> Vec<crate::gcode::GCodeAxis> {
        self.profiles
            .get(self.active_profile_index)
            .map(|p| {
                p.rotary_axes
                    .iter()
                    .filter(|ax| ax.axis_type == AxisType::Rotary)
                    .map(|ax| crate::gcode::GCodeAxis {
                        name: ax.name.clone(),
                        is_head: ax.moving_part == MovingPart::Head,
                        min_deg: ax.min,
                        max_deg: ax.max,
                        rotation_axis: ax.rotation_axis.as_vec3_f64(),
                        direction: if ax.right_handed { 1 } else { -1 },
                        max_deg_per_s: ax.max_angular_speed,
                    })
                    .collect()
            })
            .unwrap_or_else(|| vec![
                crate::gcode::GCodeAxis { name: "A".to_string(), is_head: true, min_deg: -180.0, max_deg: 180.0, rotation_axis: [1.,0.,0.], max_deg_per_s: 30.0, direction: 1 },
                crate::gcode::GCodeAxis { name: "B".to_string(), is_head: true, min_deg: -180.0, max_deg: 180.0, rotation_axis: [0.,1.,0.], max_deg_per_s: 30.0, direction: 1 },
            ])
    }

    /// Render the browser-style tab bar at the top of the window
    fn render_tab_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("tab_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.active_tab, AppTab::Main, "  Main  ");
                ui.selectable_value(&mut self.active_tab, AppTab::PrinterProfiles, "  Printer Profiles  ");
            });
        });
    }

    /// Load a mesh from file
    pub fn load_mesh(&mut self, path: PathBuf) {
        self.log(format!("Loading mesh from {:?}", path));

        match Mesh::from_stl(&path) {
            Ok(mesh) => {
                self.log(format!("Mesh loaded: {} triangles", mesh.num_triangles()));
                self.log(format!("Dimensions: {:?}", mesh.dimensions()));
                self.log(format!("Volume: {:.2} mm³", mesh.volume()));

                self.mesh = Some(mesh);
                self.mesh_path = Some(path);

                // Reset slicing state
                self.layers.clear();
                self.toolpaths.clear();
                self.gcode_lines.clear();
                self.has_sliced = false;
                self.has_toolpaths = false;

                // Clear viewport's cached mesh so it reloads
                if let Some(viewport) = &mut self.viewport_3d {
                    viewport.clear_mesh();
                }
            }
            Err(e) => {
                self.log(format!("Error loading mesh: {}", e));
            }
        }
    }

    /// Start slicing in background thread
    pub fn start_slicing(&mut self) {
        if self.mesh.is_none() {
            self.log("No mesh loaded!".to_string());
            return;
        }

        if self.is_slicing {
            self.log("Already slicing!".to_string());
            return;
        }

        self.is_slicing = true;

        let mode = self.slicing_mode;
        match mode {
            SlicingMode::Planar => {
                self.log("Starting planar slicing process...".to_string());
                self.start_planar_slicing();
            }
            SlicingMode::Curved => {
                self.log("Starting curved layer slicing (S3-Slicer)...".to_string());
                self.start_curved_slicing();
            }
            SlicingMode::S4 => {
                self.log("Starting S4 non-planar slicing (Deform + Slice + Un-Deform)...".to_string());
                self.start_s4_slicing();
            }
            SlicingMode::Conical => {
                self.log("Starting conical slicing (RotBot/Transform)...".to_string());
                self.start_conical_slicing();
            }
            SlicingMode::Geodesic => {
                self.log("Starting geodesic slicing (Heat Method)...".to_string());
                self.start_geodesic_slicing();
            }
            SlicingMode::CoordTransformCylindrical => {
                self.log("Starting cylindrical coordinate-transform slicing...".to_string());
                self.start_cylindrical_slicing();
            }
            SlicingMode::CoordTransformSpherical => {
                self.log("Starting spherical coordinate-transform slicing...".to_string());
                self.start_spherical_slicing();
            }
        }
    }

    /// Start planar slicing in background thread
    fn start_planar_slicing(&mut self) {
        let mesh = self.mesh.clone().unwrap();
        let config = self.config.clone();
        let progress = self.slicing_progress.clone();

        // Create channel for sending layers back to main thread
        let (tx, rx) = mpsc::channel();
        self.layers_receiver = Some(rx);

        // Spawn background thread for slicing
        std::thread::spawn(move || {
            // Update progress
            {
                let mut p = progress.lock().unwrap();
                p.operation = "Planar Slicing".to_string();
                p.percentage = 0.0;
                p.message = "Initializing...".to_string();
            }

            let slicer = Slicer::new(config);

            match slicer.slice(&mesh) {
                Ok(layers) => {
                    let layer_count = layers.len();

                    // Send layers back to main thread
                    let _ = tx.send(layers);

                    // Update progress
                    {
                        let mut p = progress.lock().unwrap();
                        p.operation = "Complete".to_string();
                        p.percentage = 100.0;
                        p.layers_completed = layer_count;
                        p.total_layers = layer_count;
                        p.message = format!("Generated {} layers", layer_count);
                    }
                }
                Err(e) => {
                    let mut p = progress.lock().unwrap();
                    p.operation = "Error".to_string();
                    p.message = format!("Slicing failed: {}", e);
                }
            }
        });
    }

    /// Start curved layer slicing using S3-Slicer
    fn start_curved_slicing(&mut self) {
        let mesh = self.mesh.clone().unwrap();
        let layer_height = self.config.layer_height;
        let max_rotation_degrees = self.config.max_rotation_degrees;
        let progress = self.slicing_progress.clone();

        // Get S3-Slicer configuration from GUI
        let fabrication_objective = self.s3_fabrication_objective;
        let overhang_threshold = self.s3_overhang_threshold;
        let smoothness_weight = self.s3_smoothness_weight;
        let optimization_iterations = self.s3_optimization_iterations;

        // Get deformation method settings
        let deformation_method = self.s3_deformation_method;
        let asap_max_iterations = self.s3_asap_max_iterations;
        let asap_convergence = self.s3_asap_convergence;

        // Create channel for sending layers back to main thread
        let (tx, rx) = mpsc::channel();
        self.layers_receiver = Some(rx);

        // Spawn background thread for curved slicing
        std::thread::spawn(move || {
            // Update progress
            {
                let mut p = progress.lock().unwrap();
                p.operation = "Curved Slicing (S3-Slicer)".to_string();
                p.percentage = 5.0;
                p.message = "Initializing S3-Slicer pipeline...".to_string();
            }

            // Configure S3-Slicer pipeline with GUI parameters
            let pipeline_config = S3PipelineConfig {
                objective: fabrication_objective,
                layer_height,
                optimization_iterations,
                overhang_threshold,
                smoothness_weight,
                max_rotation_degrees,
                // Deformation method settings
                deformation_method,
                asap_max_iterations,
                asap_convergence_threshold: asap_convergence,
                asap_quaternion_weight: 0.3,
                ..S3PipelineConfig::default()
            };

            // Update progress
            {
                let mut p = progress.lock().unwrap();
                p.percentage = 10.0;
                p.message = "Optimizing quaternion field...".to_string();
            }

            // Execute complete S3-Slicer pipeline with error handling
            // Steps: optimize quaternion field → scale-controlled deformation → slice deformed mesh
            //        → extract curved layers → toolpath generation
            let result = std::panic::catch_unwind(|| {
                execute_s3_pipeline(mesh, pipeline_config)
            });

            match result {
                Ok(pipeline_result) => {
                    // Update progress
                    {
                        let mut p = progress.lock().unwrap();
                        p.percentage = 90.0;
                        p.message = "Finalizing layers...".to_string();
                    }

                    // Get layers from pipeline result
                    let layers = pipeline_result.layers;
                    let layer_count = layers.len();

                    log::info!("DEBUG: Pipeline completed with {} layers", layer_count);

                    // Send layers back to main thread
                    if let Err(e) = tx.send(layers) {
                        log::error!("Failed to send layers to main thread: {:?}", e);
                        let mut p = progress.lock().unwrap();
                        p.operation = "Error".to_string();
                        p.message = "Failed to send layers to main thread".to_string();
                        return;
                    }

                    // Update progress
                    {
                        let mut p = progress.lock().unwrap();
                        p.operation = "Complete".to_string();
                        p.percentage = 100.0;
                        p.layers_completed = layer_count;
                        p.total_layers = layer_count;
                        p.message = format!("Generated {} curved layers (S3-Slicer)", layer_count);
                        log::info!("DEBUG: Progress set to Complete with {} layers", layer_count);
                    }
                }
                Err(e) => {
                    log::error!("S3-Slicer pipeline panicked: {:?}", e);
                    let mut p = progress.lock().unwrap();
                    p.operation = "Error".to_string();
                    p.message = "S3-Slicer pipeline failed (panic)".to_string();
                }
            }
        });
    }

    /// Start S4 non-planar slicing (Deform → Slice → Un-Deform)
    fn start_s4_slicing(&mut self) {
        let mesh = self.mesh.clone().unwrap();
        let layer_height = self.config.layer_height;
        let progress = self.slicing_progress.clone();

        // Get S4-specific configuration
        let overhang_threshold = self.s4_overhang_threshold;
        let max_rotation_degrees = self.s4_max_rotation_degrees;
        let smoothing_iterations = self.s4_smoothing_iterations;
        let smoothness_weight = self.s4_smoothness_weight;
        let asap_max_iterations = self.s4_asap_max_iterations;
        let asap_convergence = self.s4_asap_convergence;
        let z_bias = self.s4_z_bias;
        let auto_z_bias = self.s4_auto_z_bias;
        let use_asap = self.s4_use_asap;
        let base_detection_threshold = self.s4_base_detection_threshold;
        let edge_filter_multiplier = self.s4_edge_filter_multiplier;
        let jump_split_multiplier = self.s4_jump_split_multiplier;
        let multi_axis = self.s4_multi_axis;

        let (tx, rx) = mpsc::channel();
        self.layers_receiver = Some(rx);

        std::thread::spawn(move || {
            {
                let mut p = progress.lock().unwrap();
                p.operation = "S4 Non-Planar Slicing".to_string();
                p.percentage = 5.0;
                p.message = "Initializing S4 pipeline...".to_string();
            }

            // S4 pipeline config
            let pipeline_config = S3PipelineConfig {
                objective: FabricationObjective::SupportFree,
                layer_height,
                optimization_iterations: smoothing_iterations * 2,
                overhang_threshold,
                smoothness_weight,
                max_rotation_degrees,
                deformation_method: crate::s3_slicer::DeformationMethod::S4Deform,
                asap_max_iterations,
                asap_convergence_threshold: asap_convergence,
                asap_quaternion_weight: 0.3,
                z_bias,
                auto_z_bias,
                use_asap,
                base_detection_threshold,
                edge_filter_multiplier,
                jump_split_multiplier,
                multi_axis,
            };

            {
                let mut p = progress.lock().unwrap();
                p.percentage = 10.0;
                p.message = "Running S4 pipeline (Deform + Slice + Un-Deform)...".to_string();
            }

            // Call S4 pipeline directly (not through execute_s3_pipeline)
            let result = std::panic::catch_unwind(|| {
                execute_s4_pipeline(mesh, pipeline_config)
            });

            match result {
                Ok(pipeline_result) => {
                    {
                        let mut p = progress.lock().unwrap();
                        p.percentage = 90.0;
                        p.message = "Finalizing layers...".to_string();
                    }

                    let layers = pipeline_result.layers;
                    let layer_count = layers.len();

                    log::info!("S4 pipeline completed with {} layers", layer_count);

                    if let Err(e) = tx.send(layers) {
                        log::error!("Failed to send layers to main thread: {:?}", e);
                        let mut p = progress.lock().unwrap();
                        p.operation = "Error".to_string();
                        p.message = "Failed to send layers to main thread".to_string();
                        return;
                    }

                    {
                        let mut p = progress.lock().unwrap();
                        p.operation = "Complete".to_string();
                        p.percentage = 100.0;
                        p.layers_completed = layer_count;
                        p.total_layers = layer_count;
                        p.message = format!("Generated {} curved layers (S4 Non-Planar)", layer_count);
                    }
                }
                Err(e) => {
                    log::error!("S4 pipeline panicked: {:?}", e);
                    let mut p = progress.lock().unwrap();
                    p.operation = "Error".to_string();
                    p.message = "S4 pipeline failed (panic)".to_string();
                }
            }
        });
    }

    /// Step 2 of S4 interactive workflow: slice the deformed mesh with planar slicer.
    /// Requires `s4_stage == Deformed` (i.e. `s4_deform_data` is set).
    pub fn start_s4_slice_deformed(&mut self) {
        let Some(data) = &self.s4_deform_data else {
            self.log("S4: No deform data — run Step 1 first.".to_string());
            return;
        };
        let deformed_surface = data.deformed_surface.clone();
        let layer_height = self.config.layer_height;
        let progress = self.slicing_progress.clone();

        let (tx, rx) = mpsc::channel();
        self.s4_deformed_layers_receiver = Some(rx);

        std::thread::spawn(move || {
            {
                let mut p = progress.lock().unwrap();
                p.operation = "S4 Slicing Deformed Mesh".to_string();
                p.percentage = 10.0;
                p.message = "Slicing deformed mesh (planar)...".to_string();
            }
            match execute_s4_slice(&deformed_surface, layer_height) {
                Ok(layers) => {
                    let n = layers.len();
                    log::info!("S4 slice deformed: {} layers", n);
                    let _ = tx.send(layers);
                    let mut p = progress.lock().unwrap();
                    p.operation = "Complete".to_string();
                    p.percentage = 100.0;
                    p.message = format!("Sliced deformed mesh: {} layers", n);
                }
                Err(e) => {
                    log::error!("S4 slice deformed failed: {}", e);
                    let mut p = progress.lock().unwrap();
                    p.operation = "Error".to_string();
                    p.message = format!("S4 slice failed: {}", e);
                }
            }
        });
    }

    /// Step 3 of S4 interactive workflow: untransform the deformed layers back to original space.
    /// Requires `s4_stage == Sliced` (i.e. `s4_deformed_layers` is populated).
    pub fn start_s4_untransform(&mut self) {
        let Some(data) = &self.s4_deform_data else {
            self.log("S4: No deform data — run Steps 1 and 2 first.".to_string());
            return;
        };
        let Some(mesh) = &self.mesh else { return; };
        let planar_layers = self.s4_deformed_layers.clone();
        let data = (**data).clone();
        let original_mesh = mesh.clone();
        let layer_height = self.config.layer_height;
        let jump_split_multiplier = self.s4_jump_split_multiplier;
        let progress = self.slicing_progress.clone();

        let (tx, rx) = mpsc::channel();
        self.layers_receiver = Some(rx);
        self.is_slicing = true;

        std::thread::spawn(move || {
            {
                let mut p = progress.lock().unwrap();
                p.operation = "S4 Untransform".to_string();
                p.percentage = 10.0;
                p.message = "Untransforming layers to original mesh space...".to_string();
            }
            let layers = execute_s4_untransform(planar_layers, &data, &original_mesh, layer_height, jump_split_multiplier);
            let n = layers.len();
            log::info!("S4 untransform complete: {} layers", n);
            let _ = tx.send(layers);
            let mut p = progress.lock().unwrap();
            p.operation = "Complete".to_string();
            p.percentage = 100.0;
            p.layers_completed = n;
            p.total_layers = n;
            p.message = format!("S4 final layers: {}", n);
        });
    }

    /// Start conical slicing in background thread
    fn start_conical_slicing(&mut self) {
        let mesh = self.mesh.clone().unwrap();
        let config = self.config.clone();
        let progress = self.slicing_progress.clone();
        let cone_angle = self.conical_angle_degrees;
        let direction = self.conical_direction;
        let use_artifact_filter = self.conical_use_artifact_filter;

        // Determine center: auto from mesh centroid or manual
        let (center_x, center_y) = if self.conical_auto_center {
            let cx = (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0;
            let cy = (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0;
            (cx, cy)
        } else {
            (self.conical_center_x, self.conical_center_y)
        };

        let (tx, rx) = mpsc::channel();
        self.layers_receiver = Some(rx);

        std::thread::spawn(move || {
            {
                let mut p = progress.lock().unwrap();
                p.operation = "Conical Slicing".to_string();
                p.percentage = 5.0;
                p.message = "Initializing conical pipeline...".to_string();
            }

            let result = std::panic::catch_unwind(|| {
                crate::conical::execute_conical_pipeline(
                    &mesh, &config, cone_angle, center_x, center_y, direction,
                    use_artifact_filter,
                )
            });

            match result {
                Ok(layers) => {
                    let layer_count = layers.len();
                    log::info!("Conical pipeline completed with {} layers", layer_count);

                    if let Err(e) = tx.send(layers) {
                        log::error!("Failed to send layers to main thread: {:?}", e);
                        let mut p = progress.lock().unwrap();
                        p.operation = "Error".to_string();
                        p.message = "Failed to send layers to main thread".to_string();
                        return;
                    }

                    {
                        let mut p = progress.lock().unwrap();
                        p.operation = "Complete".to_string();
                        p.percentage = 100.0;
                        p.layers_completed = layer_count;
                        p.total_layers = layer_count;
                        p.message = format!("Generated {} conical layers", layer_count);
                    }
                }
                Err(e) => {
                    log::error!("Conical pipeline panicked: {:?}", e);
                    let mut p = progress.lock().unwrap();
                    p.operation = "Error".to_string();
                    p.message = "Conical pipeline failed (panic)".to_string();
                }
            }
        });
    }

    /// Start geodesic slicing in background thread
    fn start_geodesic_slicing(&mut self) {
        let mesh = self.mesh.clone().unwrap();
        let layer_height = self.config.layer_height;
        let progress = self.slicing_progress.clone();

        let source = if self.geodesic_source_mode == 1 {
            let p = self.geodesic_source_point;
            crate::geodesic::GeodesicSource::Point(crate::geometry::Point3D::new(p[0], p[1], p[2]))
        } else {
            crate::geodesic::GeodesicSource::BottomBoundary
        };
        let heat_factor = self.geodesic_heat_factor;
        let bottom_tol = self.geodesic_bottom_tolerance;
        let use_multiscale = self.geodesic_use_multiscale;
        let num_scales = self.geodesic_num_scales;
        let diffusion_mode_idx = self.geodesic_diffusion_mode;
        let adaptive_kappa_base = self.geodesic_adaptive_kappa_base;
        let anisotropy_ratio = self.geodesic_anisotropy_ratio;
        let aniso_smooth_iters = self.geodesic_aniso_smooth_iters;
        let print_dir_axis = self.geodesic_print_dir_axis;
        let print_dir_ratio = self.geodesic_print_dir_ratio;
        let z_weight_val = self.geodesic_z_weight;

        let (tx, rx) = mpsc::channel();
        self.layers_receiver = Some(rx);

        std::thread::spawn(move || {
            {
                let mut p = progress.lock().unwrap();
                p.operation = "Geodesic Slicing".to_string();
                p.percentage = 5.0;
                p.message = "Initializing geodesic pipeline (Heat Method)...".to_string();
            }

            let diffusion_mode = match diffusion_mode_idx {
                1 => crate::geodesic::GeodesicDiffusionMode::AdaptiveScalar {
                    kappa_base: adaptive_kappa_base,
                },
                2 => crate::geodesic::GeodesicDiffusionMode::Anisotropic {
                    anisotropy_ratio,
                    smoothing_iters: aniso_smooth_iters,
                },
                3 => {
                    let preferred_dir = match print_dir_axis {
                        0 => crate::geometry::Vector3D::new(1.0, 0.0, 0.0),
                        1 => crate::geometry::Vector3D::new(0.0, 1.0, 0.0),
                        _ => crate::geometry::Vector3D::new(0.0, 0.0, 1.0),
                    };
                    crate::geodesic::GeodesicDiffusionMode::PrintDirectionBiased {
                        preferred_dir,
                        ratio: print_dir_ratio,
                    }
                }
                _ => crate::geodesic::GeodesicDiffusionMode::Isotropic,
            };

            let config = crate::geodesic::GeodesicSlicerConfig {
                source,
                layer_height,
                heat_timestep_factor: heat_factor,
                bottom_tolerance: bottom_tol,
                use_multiscale,
                num_scales,
                diffusion_mode,
                z_weight: z_weight_val,
            };

            let result = std::panic::catch_unwind(|| {
                crate::geodesic::geodesic_slice(&mesh, &config)
            });

            match result {
                Ok(layers) => {
                    let layer_count = layers.len();
                    log::info!("Geodesic pipeline completed with {} layers", layer_count);

                    if let Err(e) = tx.send(layers) {
                        log::error!("Failed to send layers to main thread: {:?}", e);
                        let mut p = progress.lock().unwrap();
                        p.operation = "Error".to_string();
                        p.message = "Failed to send layers to main thread".to_string();
                        return;
                    }

                    {
                        let mut p = progress.lock().unwrap();
                        p.operation = "Complete".to_string();
                        p.percentage = 100.0;
                        p.layers_completed = layer_count;
                        p.total_layers = layer_count;
                        p.message = format!("Generated {} geodesic layers (Heat Method)", layer_count);
                    }
                }
                Err(e) => {
                    log::error!("Geodesic pipeline panicked: {:?}", e);
                    let mut p = progress.lock().unwrap();
                    p.operation = "Error".to_string();
                    p.message = "Geodesic pipeline failed (panic)".to_string();
                }
            }
        });
    }

    /// Start cylindrical coordinate-transform slicing in background thread
    fn start_cylindrical_slicing(&mut self) {
        let mesh = self.mesh.clone().unwrap();
        let config = self.config.clone();
        let progress = self.slicing_progress.clone();

        let (center_x, center_y) = if self.coord_transform_auto_center {
            let cx = (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0;
            let cy = (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0;
            (cx, cy)
        } else {
            (self.coord_transform_center_x, self.coord_transform_center_y)
        };

        let (tx, rx) = mpsc::channel();
        self.layers_receiver = Some(rx);

        std::thread::spawn(move || {
            {
                let mut p = progress.lock().unwrap();
                p.operation = "Cylindrical Slicing".to_string();
                p.percentage = 5.0;
                p.message = "Initializing cylindrical pipeline...".to_string();
            }

            let result = std::panic::catch_unwind(|| {
                crate::coordinate_transform::execute_cylindrical_pipeline(
                    &mesh, &config, center_x, center_y,
                )
            });

            match result {
                Ok(layers) => {
                    let layer_count = layers.len();
                    log::info!("Cylindrical pipeline completed with {} layers", layer_count);

                    if let Err(e) = tx.send(layers) {
                        log::error!("Failed to send layers to main thread: {:?}", e);
                        let mut p = progress.lock().unwrap();
                        p.operation = "Error".to_string();
                        p.message = "Failed to send layers to main thread".to_string();
                        return;
                    }

                    {
                        let mut p = progress.lock().unwrap();
                        p.operation = "Complete".to_string();
                        p.percentage = 100.0;
                        p.layers_completed = layer_count;
                        p.total_layers = layer_count;
                        p.message = format!("Generated {} cylindrical layers", layer_count);
                    }
                }
                Err(e) => {
                    log::error!("Cylindrical pipeline panicked: {:?}", e);
                    let mut p = progress.lock().unwrap();
                    p.operation = "Error".to_string();
                    p.message = "Cylindrical pipeline failed (panic)".to_string();
                }
            }
        });
    }

    /// Start spherical coordinate-transform slicing in background thread
    fn start_spherical_slicing(&mut self) {
        let mesh = self.mesh.clone().unwrap();
        let config = self.config.clone();
        let progress = self.slicing_progress.clone();

        let (center_x, center_y, center_z) = if self.coord_transform_auto_center {
            let cx = (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0;
            let cy = (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0;
            let cz = mesh.bounds_min.z; // Use bottom of mesh as sphere center Z
            (cx, cy, cz)
        } else {
            (self.coord_transform_center_x, self.coord_transform_center_y, self.coord_transform_center_z)
        };

        let (tx, rx) = mpsc::channel();
        self.layers_receiver = Some(rx);

        std::thread::spawn(move || {
            {
                let mut p = progress.lock().unwrap();
                p.operation = "Spherical Slicing".to_string();
                p.percentage = 5.0;
                p.message = "Initializing spherical pipeline...".to_string();
            }

            let result = std::panic::catch_unwind(|| {
                crate::coordinate_transform::execute_spherical_pipeline(
                    &mesh, &config, center_x, center_y, center_z,
                )
            });

            match result {
                Ok(layers) => {
                    let layer_count = layers.len();
                    log::info!("Spherical pipeline completed with {} layers", layer_count);

                    if let Err(e) = tx.send(layers) {
                        log::error!("Failed to send layers to main thread: {:?}", e);
                        let mut p = progress.lock().unwrap();
                        p.operation = "Error".to_string();
                        p.message = "Failed to send layers to main thread".to_string();
                        return;
                    }

                    {
                        let mut p = progress.lock().unwrap();
                        p.operation = "Complete".to_string();
                        p.percentage = 100.0;
                        p.layers_completed = layer_count;
                        p.total_layers = layer_count;
                        p.message = format!("Generated {} spherical layers", layer_count);
                    }
                }
                Err(e) => {
                    log::error!("Spherical pipeline panicked: {:?}", e);
                    let mut p = progress.lock().unwrap();
                    p.operation = "Error".to_string();
                    p.message = "Spherical pipeline failed (panic)".to_string();
                }
            }
        });
    }

    /// Deform mesh using S3-Slicer algorithm (for preview before slicing)
    pub fn deform_mesh_preview(&mut self) {
        let mesh = match &self.mesh {
            Some(m) => m.clone(),
            None => {
                self.log("No mesh loaded!".to_string());
                return;
            }
        };

        // For Virtual method, no actual deformation happens - explain this to user
        if matches!(self.s3_deformation_method, crate::s3_slicer::DeformationMethod::VirtualScalarField) {
            self.log("Virtual mode: Scalar field computed directly, no mesh deformation preview.".to_string());
            self.log("Slicing will work correctly - curved layers are computed from virtual heights.".to_string());
            self.deformed_mesh = None;
            self.show_deformed_mesh = false;
            return;
        }

        // For S4 mode or S4Deform method, run the S4-style deformation pipeline for preview
        let is_s4 = self.slicing_mode == SlicingMode::S4
            || matches!(self.s3_deformation_method, crate::s3_slicer::DeformationMethod::S4Deform);
        if is_s4 {
            self.log("S4: Running Dijkstra-based deformation preview...".to_string());
            self.is_deforming = true;
            let mesh_clone = mesh.clone();
            // Use S4-specific config when in S4 mode, fall back to S3 config for legacy S4Deform
            let pipeline_config = if self.slicing_mode == SlicingMode::S4 {
                S3PipelineConfig {
                    objective: FabricationObjective::SupportFree,
                    layer_height: self.config.layer_height,
                    optimization_iterations: self.s4_smoothing_iterations * 2,
                    overhang_threshold: self.s4_overhang_threshold,
                    smoothness_weight: self.s4_smoothness_weight,
                    max_rotation_degrees: self.s4_max_rotation_degrees,
                    deformation_method: crate::s3_slicer::DeformationMethod::S4Deform,
                    asap_max_iterations: self.s4_asap_max_iterations,
                    asap_convergence_threshold: self.s4_asap_convergence,
                asap_quaternion_weight: 0.3,
                    z_bias: self.s4_z_bias,
                    auto_z_bias: self.s4_auto_z_bias,
                    use_asap: self.s4_use_asap,
                    base_detection_threshold: self.s4_base_detection_threshold,
                    edge_filter_multiplier: self.s4_edge_filter_multiplier,
                    jump_split_multiplier: self.s4_jump_split_multiplier,
                    multi_axis: self.s4_multi_axis,
                }
            } else {
                S3PipelineConfig {
                    objective: self.s3_fabrication_objective,
                    layer_height: self.config.layer_height,
                    optimization_iterations: self.s3_optimization_iterations,
                    overhang_threshold: self.s3_overhang_threshold,
                    smoothness_weight: self.s3_smoothness_weight,
                    max_rotation_degrees: self.config.max_rotation_degrees,
                    deformation_method: crate::s3_slicer::DeformationMethod::S4Deform,
                    asap_max_iterations: self.s3_asap_max_iterations,
                    asap_convergence_threshold: self.s3_asap_convergence,
                asap_quaternion_weight: 0.3,
                    z_bias: 0.8,
                    ..S3PipelineConfig::default()
                }
            };

            // Two channels: one for the deformed surface (mesh display), one for full S4DeformData
            let (mesh_tx, mesh_rx) = mpsc::channel::<Mesh>();
            let (data_tx, data_rx) = mpsc::channel::<S4DeformData>();
            self.deformed_mesh_receiver = Some(mesh_rx);
            self.s4_deform_result_receiver = Some(data_rx);

            std::thread::spawn(move || {
                log::info!("=== S4 Deform Preview (step 1 of 3) ===");
                match execute_s4_deform(&mesh_clone, &pipeline_config) {
                    Ok(data) => {
                        log::info!("S4 deform complete: {} tris on deformed surface",
                            data.deformed_surface.triangles.len());
                        let surface = data.deformed_surface.clone();
                        let _ = data_tx.send(data);
                        let _ = mesh_tx.send(surface);
                    }
                    Err(e) => {
                        log::error!("S4 deform preview failed: {}", e);
                        // Send original mesh so the viewport doesn't freeze
                        let _ = mesh_tx.send(mesh_clone);
                    }
                }
            });
            return;
        }

        // For Tet Volumetric, run the tet deformation pipeline for preview
        if matches!(self.s3_deformation_method, crate::s3_slicer::DeformationMethod::TetVolumetric) {
            self.log("Tet Volumetric: Running volumetric deformation preview...".to_string());
            self.is_deforming = true;
            let mesh_clone = mesh.clone();
            let fabrication_objective = self.s3_fabrication_objective;
            let overhang_threshold = self.s3_overhang_threshold;
            let smoothness_weight = self.s3_smoothness_weight;
            let optimization_iterations = self.s3_optimization_iterations;
            let max_rotation_degrees = self.config.max_rotation_degrees;
            let asap_max_iterations = self.s3_asap_max_iterations;
            let asap_convergence = self.s3_asap_convergence;

            let (tx, rx) = mpsc::channel();
            self.deformed_mesh_receiver = Some(rx);

            std::thread::spawn(move || {
                use crate::s3_slicer::{
                    TetMesh, TetQuaternionField, TetAsapSolver, TetAsapConfig,
                };

                log::info!("=== Tet Volumetric Deformation Preview ===");

                // Step 1: Tetrahedralize
                log::info!("Step 1/3: Tetrahedralizing...");
                let tet_mesh = match TetMesh::from_surface_mesh(&mesh_clone) {
                    Ok(m) => m,
                    Err(e) => {
                        log::error!("Tet preview failed: {}. No deformation preview available.", e);
                        let _ = tx.send(mesh_clone);
                        return;
                    }
                };

                log::info!("  → {} vertices, {} tets", tet_mesh.vertices.len(), tet_mesh.tets.len());

                // Step 2: Optimize tet quaternion field
                log::info!("Step 2/3: Optimizing per-tet quaternion field...");
                let quat_config = crate::s3_slicer::QuaternionFieldConfig {
                    objective: fabrication_objective,
                    build_direction: crate::geometry::Vector3D::new(0.0, 0.0, 1.0),
                    optimization_iterations,
                    smoothness_weight,
                    objective_weight: 1.0,
                    overhang_threshold,
                    max_rotation_degrees,
                };
                let tet_qfield = TetQuaternionField::optimize(&tet_mesh, quat_config);

                // Step 3: ASAP deformation
                log::info!("Step 3/3: Running volumetric ASAP deformation...");
                let tet_asap_config = TetAsapConfig {
                    max_iterations: asap_max_iterations,
                    convergence_threshold: asap_convergence,
                    quaternion_weight: 0.3,
                    constraint_weight: 500.0,
                    allow_scaling: true,
                };

                let solver = TetAsapSolver::new(tet_mesh, tet_qfield, tet_asap_config);
                let deformed_tet = solver.solve();

                // Convert deformed tet surface to Mesh for preview
                let deformed_surface = deformed_tet.to_surface_mesh();
                log::info!("  → Deformed surface: {} triangles", deformed_surface.triangles.len());
                log::info!("✓ Tet volumetric deformation preview complete");

                let _ = tx.send(deformed_surface);
            });
            return;
        }

        self.log("Deforming mesh using S3-Slicer algorithm...".to_string());
        self.is_deforming = true;
        let fabrication_objective = self.s3_fabrication_objective;
        let overhang_threshold = self.s3_overhang_threshold;
        let smoothness_weight = self.s3_smoothness_weight;
        let optimization_iterations = self.s3_optimization_iterations;
        let max_rotation_degrees = self.config.max_rotation_degrees;
        let deformation_method = self.s3_deformation_method;
        let asap_max_iterations = self.s3_asap_max_iterations;
        let asap_convergence = self.s3_asap_convergence;

        // Create channel for sending deformed mesh back to main thread
        let (tx, rx) = mpsc::channel();
        self.deformed_mesh_receiver = Some(rx);

        // Spawn background thread for deformation
        std::thread::spawn(move || {
            log::info!("=== S3-Slicer Mesh Deformation (Preview) ===");

            // Step 1: Optimize quaternion field
            log::info!("Step 1/2: Optimizing quaternion field...");
            let quat_config = QuaternionFieldConfig {
                objective: fabrication_objective,
                build_direction: crate::geometry::Vector3D::new(0.0, 0.0, 1.0),
                optimization_iterations,
                smoothness_weight,
                objective_weight: 1.0,
                overhang_threshold,
                max_rotation_degrees,
            };

            let quaternion_field = QuaternionField::optimize(&mesh, quat_config);
            log::info!("  → Quaternion field energy: {:.4}", quaternion_field.energy);

            // Step 2: Deform mesh using quaternion field
            let deformed = match deformation_method {
                crate::s3_slicer::DeformationMethod::AsapDeformation => {
                    log::info!("Step 2/2: Deforming mesh using ASAP (global)...");
                    use crate::s3_slicer::{AsapSolver, AsapConfig};

                    let asap_config = AsapConfig {
                        max_iterations: asap_max_iterations,
                        convergence_threshold: asap_convergence,
                        quaternion_weight: 0.3,  // Match slicing pipeline
                        constraint_weight: 500.0,  // Match slicing pipeline
                        use_cotangent_weights: true,
                    };

                    let solver = AsapSolver::new(mesh.clone(), quaternion_field, asap_config);
                    solver.solve()
                }
                crate::s3_slicer::DeformationMethod::ScaleControlled => {
                    log::info!("Step 2/2: Deforming mesh using scale-controlled (local)...");
                    let deformation = S3SlicerDeformation::new(mesh.clone(), quaternion_field);
                    deformation.get_deformed_mesh().clone()
                }
                crate::s3_slicer::DeformationMethod::VirtualScalarField |
                crate::s3_slicer::DeformationMethod::TetVolumetric |
                crate::s3_slicer::DeformationMethod::S4Deform => {
                    // These shouldn't happen (handled above), but just in case
                    log::info!("Step 2/2: No surface mesh deformation for this method");
                    mesh.clone()
                }
            };

            log::info!("  → Deformed mesh: {} triangles", deformed.triangles.len());
            log::info!("✓ Mesh deformation complete");

            // Send deformed mesh back to main thread
            let _ = tx.send(deformed);
        });
    }

    /// Generate toolpaths from layers
    pub fn generate_toolpaths(&mut self) {
        if self.layers.is_empty() {
            self.log("No layers to generate toolpaths from!".to_string());
            return;
        }

        // Debug: Log layer info
        let total_contours: usize = self.layers.iter().map(|l| l.contours.len()).sum();
        let total_points: usize = self.layers.iter()
            .flat_map(|l| l.contours.iter())
            .map(|c| c.points.len())
            .sum();
        log::info!("generate_toolpaths: {} layers, {} contours, {} total points",
            self.layers.len(), total_contours, total_points);

        // Debug: Log first few layers
        for (i, layer) in self.layers.iter().enumerate().take(3) {
            log::info!("  Layer {}: z={:.2}, {} contours, {} points",
                i, layer.z,
                layer.contours.len(),
                layer.contours.iter().map(|c| c.points.len()).sum::<usize>());
        }

        self.log(format!("Generating {} toolpaths from {} layers ({} contours)...",
            self.toolpath_pattern.name(), self.layers.len(), total_contours));

        use crate::toolpath_patterns::ToolpathConfig;
        // Geodesic mode: infill is enabled by default with mesh raycaster for accurate Z.
        // Coordinate-transform modes still skip infill unless forced (no analytic back-transform
        // and raycaster Z would corrupt their coordinate-transformed paths).
        let skip_infill = matches!(
            self.slicing_mode,
            SlicingMode::CoordTransformCylindrical
            | SlicingMode::CoordTransformSpherical
        ) && !self.force_nonplanar_infill;
        // Mesh ray-cast Z projection is always used for geodesic slicing — the toolpath should
        // lie on the mesh surface and there's no analytic formula.  Must NOT be enabled for
        // transform-based methods (conical, S4, cylindrical, spherical) whose Z comes from
        // back-transforms.
        let use_mesh_raycaster = matches!(self.slicing_mode, SlicingMode::Geodesic);

        // For conical mode, provide the analytic back-transform formula so the toolpath
        // generator can compute exact Z values for every infill and wall-loop point.
        // IDW from perimeter points gives badly wrong Z for interior points (e.g. the
        // bunny body centre gets z≈50mm instead of the correct z≈20mm at deformed_z=20).
        let conical_params: Option<(f64, f64, f64, f64)> = if matches!(self.slicing_mode, SlicingMode::Conical) {
            use crate::conical::ConicalDirection;
            let tan_angle = self.conical_angle_degrees.to_radians().tan();
            // sign=-1 for Outward (outer regions pulled down), +1 for Inward
            let sign = match self.conical_direction {
                ConicalDirection::Outward => -1.0_f64,
                ConicalDirection::Inward =>   1.0_f64,
            };
            let (cx, cy, z_min) = if self.conical_auto_center {
                if let Some(mesh) = &self.mesh {
                    let cx = (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0;
                    let cy = (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0;
                    (cx, cy, mesh.bounds_min.z)
                } else {
                    (self.conical_center_x, self.conical_center_y, 0.0)
                }
            } else {
                let z_min = self.mesh.as_ref().map(|m| m.bounds_min.z).unwrap_or(0.0);
                (self.conical_center_x, self.conical_center_y, z_min)
            };
            Some((cx, cy, sign * tan_angle, z_min))
        } else {
            None
        };

        let pattern_config = ToolpathConfig {
            pattern: self.toolpath_pattern,
            line_width: self.toolpath_line_width,
            node_distance: 1.0,
            infill_density: self.toolpath_infill_density,
            wall_count: self.wall_count,
            infill_pattern: self.infill_pattern,
            skip_infill,
            wall_transitions: self.wall_seam_transitions,
            use_mesh_raycaster,
            conical_params,
            optimize_print_order: self.optimize_print_order,
            ..ToolpathConfig::default()
        };

        let generator = ToolpathGenerator::new(self.nozzle_diameter, self.config.layer_height)
            .with_config(pattern_config);

        // Pass the mesh so the generator can ray-cast accurate surface Z values onto infill/wall
        // points, and to provide coverage gap fill on steep faces.
        self.toolpaths = generator.generate(&self.layers, self.mesh.as_ref());
        self.has_toolpaths = true;

        let total_moves: usize = self.toolpaths.iter().map(|tp| tp.paths.len()).sum();
        self.log(format!("Generated {} toolpath moves using {} pattern", total_moves, self.toolpath_pattern.name()));
        self.log("💡 Tip: Run Motion Planning to optimize for singularities and collisions".to_string());

        // ── Per-segment tool orientations ─────────────────────────────────
        // Conical: analytical cone-surface normal.
        // All other modes: nearest mesh-surface normal (clamped by profile axis limits).
        self.apply_conical_orientations();
        self.apply_surface_normal_orientations();

        // ── Static mesh body avoidance ────────────────────────────────────
        // Check each segment's tool capsule against the original mesh and tilt
        // outward to avoid passing through unprinted model geometry (e.g., bunny nose).
        // This runs before the height-map pass so the improved orientations feed into it.
        self.optimize_toolpath_orientations_for_mesh();

        // ── Head-body collision avoidance (deposited material) ────────────
        // Rotate the tilt azimuth (and reduce magnitude if needed) so the
        // printhead body clears previously deposited extrusions.
        self.apply_tilt_collision_avoidance();

        // ── Orientation smoothing (angular velocity limiter) ──────────────
        // Bidirectional SLERP pass that limits how fast the tool orientation
        // can change per mm of travel, preventing jerky axis motion.
        self.smooth_toolpath_orientations();

        // ── Travel lift (Z clearance on non-extrusion moves) ──────────────
        // Raise non-extruding moves above the last printed Z so the nozzle
        // clears already-deposited material during traversals.
        self.apply_travel_lifts();

        // ── Support generation (if enabled) ──────────────────────────────
        // Runs after main toolpaths are ready so supports are appended at the end.
        if self.enable_supports && self.mesh.is_some() {
            self.generate_supports();
            self.generate_support_toolpaths();
            // Merge support toolpaths into the main toolpath list.
            if let Some(result) = &self.support_result {
                if !result.toolpaths.is_empty() {
                    let support_tps = result.toolpaths.clone();
                    let n = support_tps.len();
                    self.toolpaths.extend(support_tps);
                    self.log(format!("Merged {} support toolpath layers into main toolpaths", n));
                }
            }
        }

        // Check machine simulation collisions if enabled
        if self.profiles.get(self.active_profile_index).map(|p| p.show_machine).unwrap_or(false) {
            self.compute_all_collisions();
        }
    }

    /// Generate support structures
    pub fn generate_supports(&mut self) {
        if self.mesh.is_none() {
            self.log("No mesh loaded!".to_string());
            return;
        }

        let overhang_angle = self.support_config.overhang_angle;
        self.log(format!("Detecting overhangs (threshold: {}°)...", overhang_angle));

        let mut generator = SupportGenerator::new();
        generator.overhang_config = self.support_config.clone();

        let result = generator.generate(self.mesh.as_ref().unwrap(), None);

        let stats = result.stats();

        self.log(format!("✓ Support generation complete:"));
        self.log(format!("  - Overhang faces: {}", stats.num_overhangs));
        self.log(format!("  - Overhang area: {:.2} mm²", stats.overhang_area));
        self.log(format!("  - Support nodes: {}", stats.num_support_nodes));
        self.log(format!("  - Contact points: {}", stats.num_contact_points));

        self.support_result = Some(result);
        self.has_supports = true;
    }

    /// Generate toolpaths for support structures and merge with model toolpaths
    pub fn generate_support_toolpaths(&mut self) {
        if self.support_result.is_none() {
            self.log("No supports generated yet!".to_string());
            return;
        }

        self.log("Generating support toolpaths...".to_string());

        let generator = SupportGenerator::new();

        if let Some(result) = &self.support_result {
            if let Some(tree) = &result.tree {
                let support_toolpaths = generator.generate_toolpaths(tree);

                let num_layers = support_toolpaths.len();
                let total_segments: usize = support_toolpaths.iter().map(|tp| tp.paths.len()).sum();

                // Store support toolpaths in the result
                if let Some(result_mut) = &mut self.support_result {
                    result_mut.toolpaths = support_toolpaths;
                }

                self.log(format!("✓ Support toolpaths generated:"));
                self.log(format!("  - Layers: {}", num_layers));
                self.log(format!("  - Segments: {}", total_segments));
            }
        }
    }

    /// Run motion planning pipeline
    pub fn run_motion_planning(&mut self) {
        if self.toolpaths.is_empty() {
            self.log("No toolpaths to optimize!".to_string());
            return;
        }

        self.log("Running motion planning pipeline...".to_string());
        self.is_motion_planning = true;

        // Convert toolpaths to waypoints
        use crate::motion_planning::Waypoint;
        let mut waypoints = Vec::new();

        for (layer_idx, toolpath) in self.toolpaths.iter().enumerate() {
            for segment in &toolpath.paths {
                waypoints.push(Waypoint {
                    position: segment.position,
                    orientation: segment.orientation,
                    layer_idx,
                    extrusion: segment.extrusion,
                });
            }
        }

        self.log(format!("Created {} waypoints", waypoints.len()));

        // Create motion planning configuration
        use crate::motion_planning::{MotionPlanner, MotionPlanningConfig};
        use crate::geometry::Point3D;

        let config = MotionPlanningConfig {
            platform_bounds: (
                Point3D::new(self.machine.workspace_x.0, self.machine.workspace_y.0, self.machine.workspace_z.0),
                Point3D::new(self.machine.workspace_x.1, self.machine.workspace_y.1, self.machine.workspace_z.1),
            ),
            ..Default::default()
        };

        let planner = MotionPlanner::new(config);

        // Run the 6-step pipeline
        match planner.plan_complete_path(waypoints) {
            Ok(motion_plan) => {
                self.has_motion_plan = true;
                self.log("✓ Motion planning complete!".to_string());
                self.log(format!("  - Waypoints: {}", motion_plan.waypoints.len()));
                self.log(format!("  - Path segments: {}", motion_plan.path.len()));

                // Report what the planner actually achieved. These two lines
                // used to be unconditional claims; both are false in the
                // common case -- a vertical tool is scored as singular, so a
                // planar print leaves no waypoint with a usable solution, and
                // no collision mesh is installed on the detector at all.
                let diag = motion_plan.diagnostics;
                if diag.fell_back_to_greedy {
                    self.log(
                        "  - ⚠ No waypoint had a non-singular, collision-free solution; \
                         orientations were left as sliced (no search performed)"
                            .to_string(),
                    );
                } else {
                    self.log(format!(
                        "  - Singularity check: {} of {} waypoints had a usable solution",
                        diag.waypoints_with_solutions,
                        diag.waypoints_with_solutions + diag.waypoints_without_solutions
                    ));
                    if diag.waypoints_without_solutions > 0 {
                        self.log(format!(
                            "  - ⚠ {} waypoints kept their sliced orientation (no alternative found)",
                            diag.waypoints_without_solutions
                        ));
                    }
                }
                self.log(if diag.mesh_collision_checked {
                    "  - Collisions checked against platform bounds and part mesh".to_string()
                } else {
                    "  - Collisions checked against platform bounds only (no part mesh installed)"
                        .to_string()
                });
                self.log("💡 Ready to generate G-code".to_string());
            }
            Err(e) => {
                self.log(format!("Motion planning failed: {}", e));
            }
        }

        self.is_motion_planning = false;
    }

    /// Generate G-code from current toolpaths
    pub fn generate_gcode_from_toolpaths(&mut self) {
        if self.toolpaths.is_empty() {
            self.log("No toolpaths to generate G-code from!".to_string());
            return;
        }

        self.log("Generating G-code...".to_string());

        if !self.has_motion_plan {
            self.log("⚠️ Warning: Generating G-code without motion planning optimization".to_string());
        }

        let mut gcode_gen = GCodeGenerator::new();
        gcode_gen.kinematics.tcp_offset = self.tcp_offset;
        gcode_gen.kinematics.axes = self.build_gcode_axes();
        self.gcode_lines = gcode_gen.generate_to_string(&self.toolpaths);
        self.show_gcode_terminal = true;
        self.gcode_scroll_to_bottom = true;
        self.log(format!("✓ Generated {} lines of G-code", self.gcode_lines.len()));
    }

    /// Export G-code
    pub fn export_gcode(&mut self, path: PathBuf) {
        if self.toolpaths.is_empty() {
            self.log("No toolpaths to export!".to_string());
            return;
        }

        self.log(format!("Exporting G-code to {:?}", path));

        let mut gcode_gen = GCodeGenerator::new();
        gcode_gen.kinematics.tcp_offset = self.tcp_offset;
        gcode_gen.kinematics.axes = self.build_gcode_axes();
        match gcode_gen.generate(&self.toolpaths, &path) {
            Ok(_) => {
                self.log(format!("G-code exported successfully"));

                // Load G-code for preview
                if let Ok(content) = std::fs::read_to_string(&path) {
                    self.gcode_lines = content.lines().map(|s| s.to_string()).collect();
                }
            }
            Err(e) => {
                self.log(format!("Error exporting G-code: {}", e));
            }
        }
    }

    /// Get mesh statistics
    pub fn mesh_stats(&self) -> Option<MeshStats> {
        self.mesh.as_ref().map(|mesh| {
            let dims = mesh.dimensions();
            MeshStats {
                triangles: mesh.num_triangles(),
                dimensions: (dims.x, dims.y, dims.z),
                volume: mesh.volume(),
            }
        })
    }

    /// Get slicing statistics
    pub fn slicing_stats(&self) -> SlicingStats {
        let total_moves = self.toolpaths.iter().map(|tp| tp.paths.len()).sum();

        SlicingStats {
            layers: self.layers.len(),
            total_moves,
            has_sliced: self.has_sliced,
            has_toolpaths: self.has_toolpaths,
        }
    }

    /// Enable face orientation mode
    pub fn enable_face_orientation_mode(&mut self) {
        self.face_orientation_mode = true;
        self.selected_face_index = None;
        self.log("Face orientation mode enabled. Click on a face to orient the model.".to_string());
    }

    /// Apply face orientation - rotate mesh so selected face is flat on build plate
    pub fn apply_face_orientation(&mut self, face_index: usize) {
        if let Some(mesh) = &mut self.mesh {
            if face_index < mesh.triangles.len() {
                // Get the selected triangle
                let triangle = &mesh.triangles[face_index];
                let face_normal = triangle.normal();

                // Log before mutating mesh
                let normal_msg = format!("Selected face with normal: ({:.3}, {:.3}, {:.3})",
                    face_normal.x, face_normal.y, face_normal.z);

                // Target direction is -Z (pointing down to build plate)
                // We want the face to point DOWN so it sits flat
                use crate::geometry::Vector3D;
                let target_direction = Vector3D::new(0.0, 0.0, -1.0);

                // Orient the mesh
                mesh.orient_face_to_direction(face_normal, target_direction);

                // Now we can log (mesh borrow released)
                self.log(normal_msg);
                self.log("Model oriented. Face is now flat on the build plate.".to_string());

                // Clear viewport's cached mesh so it reloads
                if let Some(viewport) = &mut self.viewport_3d {
                    viewport.clear_mesh();
                }

                // Reset slicing state since mesh has changed
                self.layers.clear();
                self.toolpaths.clear();
                self.gcode_lines.clear();
                self.has_sliced = false;
                self.has_toolpaths = false;
                self.has_motion_plan = false;

                // Exit face orientation mode
                self.face_orientation_mode = false;
                self.selected_face_index = None;
            }
        }
    }
}

impl eframe::App for SlicerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Check for completed slicing results
        if let Some(ref receiver) = self.layers_receiver {
            if let Ok(layers) = receiver.try_recv() {
                self.layers = layers;
                self.log(format!("Received {} layers from slicing", self.layers.len()));
                log::info!("DEBUG: Received layers, self.layers.len() = {}", self.layers.len());
                self.layers_receiver = None; // Clear the receiver
                // If we were in the S4 Sliced stage, receiving via layers_receiver means untransform is done
                if self.s4_stage == S4Stage::Sliced {
                    self.s4_stage = S4Stage::Final;
                    self.show_deformed_mesh = false;
                    self.log("S4 Step 3 complete: layers untransformed to original mesh space.".to_string());
                }
            }
        }

        // Check for completed deformation results (mesh display)
        if let Some(ref receiver) = self.deformed_mesh_receiver {
            if let Ok(deformed_mesh) = receiver.try_recv() {
                self.deformed_mesh = Some(deformed_mesh);
                self.is_deforming = false;
                self.show_deformed_mesh = true; // Automatically show the deformed mesh
                self.log("Mesh deformation complete - showing deformed mesh".to_string());
                self.log("Tip: Use the toggle button to switch between original and deformed views".to_string());
                self.deformed_mesh_receiver = None; // Clear the receiver

                // Viewport will pick up the deformed mesh on next render frame
                // (via show_deformed_mesh flag in viewport_3d render logic)
                if let Some(viewport) = &mut self.viewport_3d {
                    viewport.clear_mesh(); // Force reload on next frame
                }
            }
        }

        // S4 step 1: receive full S4DeformData (after deform_mesh_preview for S4 mode)
        if let Some(ref receiver) = self.s4_deform_result_receiver {
            if let Ok(data) = receiver.try_recv() {
                self.s4_deform_data = Some(Box::new(data));
                self.s4_stage = S4Stage::Deformed;
                self.s4_deform_result_receiver = None;
                self.log("S4 Step 1 complete: deformed mesh ready. Click '2: Slice Deformed'.".to_string());
            }
        }

        // S4 step 2: receive planar layers sliced on deformed mesh
        if let Some(ref receiver) = self.s4_deformed_layers_receiver {
            if let Ok(layers) = receiver.try_recv() {
                let n = layers.len();
                // Show these layers ON the deformed mesh by putting them in self.layers
                self.s4_deformed_layers = layers.clone();
                self.layers = layers;
                self.s4_stage = S4Stage::Sliced;
                self.show_deformed_mesh = true;
                self.s4_deformed_layers_receiver = None;
                self.has_sliced = true;
                self.is_slicing = false;
                self.log(format!("S4 Step 2 complete: {} layers on deformed mesh. Click '3: Untransform'.", n));
            }
        }

        // Check slicing progress
        if self.is_slicing {
            let progress = self.slicing_progress.lock().unwrap().clone();

            if progress.operation == "Complete" {
                self.is_slicing = false;
                self.has_sliced = true;
                self.log(progress.message.clone());
                log::info!("DEBUG: Slicing complete, has_sliced = true, layers.len() = {}", self.layers.len());
            } else if progress.operation == "Error" {
                self.is_slicing = false;
                self.log(progress.message.clone());
            }

            // Request repaint for smooth progress updates
            ctx.request_repaint();
        }

        // Keep repainting while deformation is running so receivers get polled
        if self.is_deforming
            || self.deformed_mesh_receiver.is_some()
            || self.s4_deform_result_receiver.is_some()
            || self.s4_deformed_layers_receiver.is_some()
        {
            ctx.request_repaint();
        }

        // Handle automatic playback
        if self.toolpath_playback_playing && self.toolpath_playback_enabled {
            // Use windows(2) count to match viewport tube rendering
            let total_segments: usize = self.toolpaths.iter()
                .map(|tp| tp.paths.len().saturating_sub(1))
                .sum();

            if total_segments > 0 {
                let now = std::time::Instant::now();
                let elapsed = now.duration_since(self.last_playback_time).as_secs_f32();

                // Calculate how many segments to advance based on speed
                let segments_to_advance = (elapsed * self.toolpath_playback_speed) as usize;

                if segments_to_advance > 0 {
                    self.toolpath_playback_position =
                        (self.toolpath_playback_position + segments_to_advance).min(total_segments - 1);
                    self.last_playback_time = now;

                    // Update highlighted G-code line (skip header lines)
                    let header_lines = 11; // Number of header lines in G-code
                    self.gcode_highlight_line = Some(header_lines + self.toolpath_playback_position);

                    // Stop playing if we reached the end
                    if self.toolpath_playback_position >= total_segments - 1 {
                        self.toolpath_playback_playing = false;
                    }
                }

                // Request repaint for smooth playback
                ctx.request_repaint();
            }
        }

        // Render UI panels
        // NOTE: In egui, all Side/Top/Bottom panels must be added BEFORE CentralPanel.
        // CentralPanel takes whatever space remains after other panels are allocated.
        self.render_tab_bar(ctx);
        // The slicing controls and the slicing statistics belong to the Main
        // tab. They were drawn on every tab, so the Printer Profiles page
        // opened with a third of the window given over to layer heights and
        // infill patterns that have nothing to do with configuring a machine.
        if self.active_tab == AppTab::Main {
            self.render_control_panel(ctx);
            self.render_stats_panel(ctx);
        }
        self.render_gcode_terminal(ctx);
        self.render_central_panel(ctx);
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        // Persist printer profiles so they survive across sessions
        if let Ok(json) = serde_json::to_string(&self.profiles) {
            storage.set_string("printer_profiles", json);
        }
        storage.set_string("active_profile_index", self.active_profile_index.to_string());
    }
}

impl SlicerApp {
    fn render_control_panel(&mut self, ctx: &egui::Context) {
        use crate::gui::control_panel;
        control_panel::render(self, ctx);
    }

    fn render_gcode_terminal(&mut self, ctx: &egui::Context) {
        if !self.show_gcode_terminal {
            return;
        }

        egui::TopBottomPanel::bottom("gcode_terminal")
            .resizable(true)
            .default_height(150.0)
            .min_height(80.0)
            .max_height(300.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("G-code Output");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("✖").clicked() {
                            self.show_gcode_terminal = false;
                        }
                        if ui.button("📋 Copy All").clicked() {
                            let gcode_text = self.gcode_lines.join("\n");
                            ui.output_mut(|o| o.copied_text = gcode_text);
                        }
                    });
                });

                ui.separator();

                if self.gcode_lines.is_empty() {
                    ui.centered_and_justified(|ui| {
                        ui.label(
                            egui::RichText::new("No G-code generated yet")
                                .italics()
                                .weak()
                        );
                    });
                } else {
                    let scroll = egui::ScrollArea::vertical()
                        .stick_to_bottom(self.gcode_scroll_to_bottom)
                        .auto_shrink([false, false]);

                    scroll.show(ui, |ui| {
                        ui.with_layout(egui::Layout::top_down(egui::Align::LEFT), |ui| {
                            ui.style_mut().override_font_id = Some(egui::FontId::monospace(10.0));

                            for (i, line) in self.gcode_lines.iter().enumerate() {
                                let is_highlighted = self.gcode_highlight_line == Some(i);
                                let line_num = format!("{:5} │ ", i + 1);

                                let response = ui.horizontal(|ui| {
                                    // Highlight background if this is the current line
                                    if is_highlighted {
                                        let rect = ui.available_rect_before_wrap();
                                        ui.painter().rect_filled(
                                            rect,
                                            0.0,
                                            egui::Color32::from_rgb(60, 80, 40), // Dark green highlight
                                        );
                                    }

                                    ui.label(
                                        egui::RichText::new(line_num)
                                            .color(if is_highlighted {
                                                egui::Color32::from_rgb(150, 255, 150)
                                            } else {
                                                egui::Color32::GRAY
                                            })
                                    );
                                    ui.label(
                                        egui::RichText::new(line)
                                            .color(if is_highlighted {
                                                egui::Color32::from_rgb(200, 255, 200)
                                            } else {
                                                egui::Color32::WHITE
                                            })
                                    );
                                });

                                // Scroll to the highlighted line
                                if is_highlighted {
                                    response.response.scroll_to_me(Some(egui::Align::Center));
                                }
                            }
                        });
                    });

                    // Show stats
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label(format!("Total lines: {}", self.gcode_lines.len()));
                        ui.separator();

                        // Count moves and extrusions
                        let g1_moves = self.gcode_lines.iter().filter(|l| l.starts_with("G1 ")).count();
                        ui.label(format!("G1 moves: {}", g1_moves));
                    });
                }
            });

        // Reset scroll flag
        self.gcode_scroll_to_bottom = false;
    }

    fn render_central_panel(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            match self.active_tab {
                AppTab::PrinterProfiles => {
                    crate::gui::printer_profiles_page::render(self, ui);
                }
                AppTab::Main => {
                    ui.heading("3D Viewport");

                    if let Some(mut viewport) = self.viewport_3d.take() {
                        let clicked_face = viewport.render(ui, self);
                        self.viewport_3d = Some(viewport);

                        // Handle clicked face in face orientation mode
                        if let Some(face_index) = clicked_face {
                            self.apply_face_orientation(face_index);
                        }
                    } else {
                        ui.label("3D viewport not initialized");
                    }
                }
            }
        });
    }

    fn render_stats_panel(&mut self, ctx: &egui::Context) {
        use crate::gui::stats_panel;
        stats_panel::render(self, ctx);
    }
}

// ─── Orientation smoothing helpers ─────────────────────────────────────────────

/// SLERP between two unit vectors `a` and `b` by parameter `t` (0=a, 1=b).
/// Falls back to linear interpolation for nearly-parallel vectors.
fn slerp_vec3(a: Vector3D, b: Vector3D, t: f64) -> Vector3D {
    let dot = a.dot(&b).clamp(-1.0, 1.0);
    let theta = dot.acos();
    if theta.abs() < 1e-9 {
        return a; // Nearly identical — no interpolation needed
    }
    let sin_theta = theta.sin();
    let wa = ((1.0 - t) * theta).sin() / sin_theta;
    let wb = (t * theta).sin() / sin_theta;
    let result = a * wa + b * wb;
    let norm = result.norm();
    if norm > 1e-9 { result / norm } else { a }
}

// ─── Machine simulation helpers ───────────────────────────────────────────────

/// Rodrigues rotation matrix around a unit axis by `angle` radians.
fn rodrigues_f32(axis: [f32; 3], angle: f32) -> [[f32; 3]; 3] {
    let (s, c) = (angle.sin(), angle.cos());
    let t = 1.0 - c;
    let [x, y, z] = axis;
    [
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ]
}

/// Multiply two 3×3 matrices.
fn mat3_mul_f32(a: [[f32; 3]; 3], b: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut r = [[0.0f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { for k in 0..3 { r[i][j] += a[i][k] * b[k][j]; } } }
    r
}

/// Apply a 3×3 rotation matrix to a [f32; 3] vector.
fn rot3_apply(r: [[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        r[0][0]*v[0] + r[0][1]*v[1] + r[0][2]*v[2],
        r[1][0]*v[0] + r[1][1]*v[1] + r[1][2]*v[2],
        r[2][0]*v[0] + r[2][1]*v[1] + r[2][2]*v[2],
    ]
}

/// Transpose a 3×3 matrix (= inverse for rotation matrices).
fn mat3_transpose_f32(m: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    [[m[0][0], m[1][0], m[2][0]],
     [m[0][1], m[1][1], m[2][1]],
     [m[0][2], m[1][2], m[2][2]]]
}

/// Build a chained rotation from `(unit_axis, angle_rad)` pairs applied in order.
fn chained_rotation_f32(joints: &[([f32; 3], f32)]) -> [[f32; 3]; 3] {
    let mut r = [[1.0f32,0.,0.],[0.,1.,0.],[0.,0.,1.]];
    for &(axis, angle) in joints {
        r = mat3_mul_f32(rodrigues_f32(axis, angle), r);
    }
    r
}

/// Given a box with half-extents `he` and rotation matrix `r`,
/// compute the world-space AABB half-extents using the standard AABB-of-OBB formula.
fn aabb_of_obb(he: [f32; 3], r: [[f32; 3]; 3]) -> [f32; 3] {
    [
        r[0][0].abs() * he[0] + r[0][1].abs() * he[1] + r[0][2].abs() * he[2],
        r[1][0].abs() * he[0] + r[1][1].abs() * he[1] + r[1][2].abs() * he[2],
        r[2][0].abs() * he[0] + r[2][1].abs() * he[1] + r[2][2].abs() * he[2],
    ]
}

// ─────────────────────────────────────────────────────────────────────────────

/// Mesh statistics for display
#[derive(Debug, Clone)]
pub struct MeshStats {
    pub triangles: usize,
    pub dimensions: (f64, f64, f64),
    pub volume: f64,
}

/// Slicing statistics for display
#[derive(Debug, Clone)]
pub struct SlicingStats {
    pub layers: usize,
    pub total_moves: usize,
    pub has_sliced: bool,
    pub has_toolpaths: bool,
}

/// Candidate tool orientations for escaping a collision, ordered by increasing
/// angular deviation from `current`.
///
/// Each candidate tilts `current` toward the radially-outward direction
/// `escape_xy` by a multiple of `tilt_step_deg`. Searching outward from the
/// *current* orientation is the point: the previous sweep ran from vertical
/// upward, so its first candidate was always `(0, 0, 1)` and the first
/// collision-free hit usually threw away whatever
/// `apply_surface_normal_orientations` had computed.
///
/// Candidates whose tilt away from vertical would exceed `max_tilt_deg` are
/// skipped, because that -- not the deviation from `current` -- is what the
/// machine's rotary axes have to achieve. `current` itself is never returned:
/// the caller has already established that it collides.
///
/// Lives here next to its only caller; T-16 should move it into the library
/// alongside the rest of the orientation logic.
pub(crate) fn collision_escape_candidates(
    current: crate::geometry::Vector3D,
    escape_xy: (f64, f64),
    tilt_step_deg: f64,
    max_tilt_deg: f64,
    steps: usize,
) -> Vec<crate::geometry::Vector3D> {
    use crate::geometry::Vector3D;

    let cur_len = current.norm();
    let cur = if cur_len > 1e-9 {
        current / cur_len
    } else {
        Vector3D::new(0.0, 0.0, 1.0)
    };

    let escape_raw = Vector3D::new(escape_xy.0, escape_xy.1, 0.0);
    let esc_len = escape_raw.norm();
    let escape = if esc_len > 1e-9 {
        escape_raw / esc_len
    } else {
        Vector3D::new(1.0, 0.0, 0.0)
    };

    // Axis that rotates `cur` toward `escape`.
    let cross = cur.cross(&escape);
    let axis = if cross.norm() > 1e-9 {
        nalgebra::Unit::new_normalize(cross)
    } else {
        // `cur` already lies along the escape direction (or exactly opposite),
        // so the cross product is degenerate. Any axis perpendicular to the
        // escape direction still tilts it; take one in the XY plane.
        let alt = Vector3D::new(-escape.y, escape.x, 0.0);
        if alt.norm() > 1e-9 {
            nalgebra::Unit::new_normalize(alt)
        } else {
            nalgebra::Unit::new_normalize(Vector3D::new(1.0, 0.0, 0.0))
        }
    };

    let max_tilt_rad = max_tilt_deg.to_radians();
    let mut out = Vec::with_capacity(steps);

    for step in 1..=steps {
        let deviation = (step as f64) * tilt_step_deg.to_radians();
        let rot = nalgebra::UnitQuaternion::from_axis_angle(&axis, deviation);
        let trial = rot * cur;

        // Tilt from vertical is the quantity the rotary axes must reach, and it
        // is not monotonic in `deviation` when `current` leans away from the
        // escape direction -- so skip rather than stop.
        if trial.z.clamp(-1.0, 1.0).acos() > max_tilt_rad + 1e-9 {
            continue;
        }
        out.push(trial);
    }

    out
}

#[cfg(test)]
mod tests {
    use super::collision_escape_candidates;
    use super::{AxisType, MovingPart, PrinterProfile, SlicerApp};
    use crate::geometry::Vector3D;

    /// Build an app with one profile whose rotary axes travel +/-`axis_deg` and
    /// whose head envelope allows `head_deg` of tilt.
    fn app_with_limits(axis_deg: f64, head_deg: f64) -> SlicerApp {
        let mut profile = PrinterProfile::default();
        for ax in &mut profile.rotary_axes {
            ax.min = -axis_deg;
            ax.max = axis_deg;
        }
        profile.max_tool_tilt_deg = head_deg;

        let mut app = SlicerApp::default();
        app.profiles = vec![profile];
        app.active_profile_index = 0;
        app
    }

    /// Regression test for D-12.
    ///
    /// Every orientation pass used to derive its tilt budget from the rotary
    /// axes' travel range alone. The default profile lets A and B travel
    /// +/-180 degrees, so the collision escape search was free to rotate the
    /// tool all the way over: measured G-code for a sphere contained tilts up
    /// to 177.98 degrees -- the nozzle pointing into the bed -- on every one of
    /// 146216 segments, with A and B spanning the full +/-180.
    ///
    /// How far an axis can travel and how far the nozzle may lean are different
    /// quantities. Both must bind.
    #[test]
    fn test_tilt_budget_is_limited_by_the_head_not_just_the_axes() {
        // The shipped default: axes can go all the way round, the head cannot.
        let app = app_with_limits(180.0, 45.0);
        assert_eq!(app.tool_tilt_budget_deg(false), 45.0);
        assert_eq!(app.tool_tilt_budget_deg(true), 45.0);
    }

    #[test]
    fn test_tilt_budget_takes_whichever_limit_is_tighter() {
        // Axis-limited: a 20 degree axis beats a 45 degree head envelope.
        let app = app_with_limits(20.0, 45.0);
        assert_eq!(app.tool_tilt_budget_deg(false), 20.0);

        // Head-limited: a 30 degree head envelope beats 90 degree axes.
        let app = app_with_limits(90.0, 30.0);
        assert_eq!(app.tool_tilt_budget_deg(false), 30.0);
    }

    #[test]
    fn test_tilt_budget_never_exceeds_ninety_degrees() {
        // Past 90 degrees the tool is pointing below horizontal, which is never
        // a legitimate printing orientation however the profile is configured.
        let app = app_with_limits(180.0, 180.0);
        assert!(
            app.tool_tilt_budget_deg(false) <= 90.0,
            "budget was {}",
            app.tool_tilt_budget_deg(false)
        );
    }

    #[test]
    fn test_tilt_budget_survives_a_degenerate_profile() {
        // No rotary axes at all: fall back to the head envelope.
        let mut app = app_with_limits(180.0, 45.0);
        app.profiles[0].rotary_axes.clear();
        assert_eq!(app.tool_tilt_budget_deg(false), 45.0);

        // A nonsensical head limit must not pin every segment vertical, which
        // would silently disable non-planar printing altogether.
        let app = app_with_limits(180.0, 0.0);
        assert!(app.tool_tilt_budget_deg(false) >= 1.0);

        // No profile at all.
        let mut app = SlicerApp::default();
        app.profiles.clear();
        assert!(app.tool_tilt_budget_deg(false) >= 1.0);
    }

    #[test]
    fn test_tilt_budget_head_axes_only_ignores_bed_axes() {
        let mut profile = PrinterProfile::default();
        // Head axis is the restrictive one; the bed axis is wide open.
        for ax in &mut profile.rotary_axes {
            let (min, max) = if ax.moving_part == MovingPart::Head {
                (-15.0, 15.0)
            } else {
                (-180.0, 180.0)
            };
            ax.min = min;
            ax.max = max;
            assert_eq!(ax.axis_type, AxisType::Rotary);
        }
        profile.max_tool_tilt_deg = 60.0;

        let mut app = SlicerApp::default();
        app.profiles = vec![profile];
        app.active_profile_index = 0;

        // Head-only passes see the 15 degree head axis.
        assert_eq!(app.tool_tilt_budget_deg(true), 15.0);
        // Passes that may also turn the part still see the head axis as the
        // tightest of all rotary axes.
        assert_eq!(app.tool_tilt_budget_deg(false), 15.0);
    }

    fn angle_deg(a: &Vector3D, b: &Vector3D) -> f64 {
        (a.dot(b) / (a.norm() * b.norm()))
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees()
    }

    /// Regression test for T-08.
    ///
    /// The sweep used to run `0..=steps` from vertical, so its first candidate
    /// was always `(0, 0, 1)`; a colliding segment carrying a good
    /// surface-normal orientation was therefore snapped back to vertical rather
    /// than nudged. Candidates must now begin adjacent to the current
    /// orientation and work outward.
    #[test]
    fn test_escape_candidates_start_near_current_orientation() {
        let tilt = 30_f64.to_radians();
        let current = Vector3D::new(tilt.sin(), 0.0, tilt.cos());
        let vertical = Vector3D::new(0.0, 0.0, 1.0);

        let candidates = collision_escape_candidates(current, (1.0, 0.0), 5.0, 45.0, 10);
        assert!(!candidates.is_empty(), "expected candidates within the tilt limit");

        let first = candidates[0];
        assert!(
            (angle_deg(&first, &current) - 5.0).abs() < 1e-6,
            "first candidate should deviate by exactly one tilt step, got {:.4} deg",
            angle_deg(&first, &current)
        );
        assert!(
            angle_deg(&first, &current) < angle_deg(&first, &vertical),
            "first candidate sits nearer vertical ({:.2} deg) than the current orientation ({:.2} deg)",
            angle_deg(&first, &vertical),
            angle_deg(&first, &current)
        );
        for c in &candidates {
            assert!(
                angle_deg(c, &vertical) > 1e-6,
                "vertical must never be offered as an escape for an already-tilted segment"
            );
        }
    }

    #[test]
    fn test_escape_candidates_are_unit_and_monotonic() {
        let tilt = 10_f64.to_radians();
        let current = Vector3D::new(0.0, tilt.sin(), tilt.cos());

        let candidates = collision_escape_candidates(current, (0.0, 1.0), 5.0, 60.0, 8);
        assert!(!candidates.is_empty());

        let mut previous = 0.0;
        for c in &candidates {
            assert!(
                (c.norm() - 1.0).abs() < 1e-9,
                "candidate must be a unit vector, got norm {}",
                c.norm()
            );
            let deviation = angle_deg(c, &current);
            assert!(
                deviation > previous - 1e-9,
                "deviation must not decrease: {:.4} then {:.4}",
                previous,
                deviation
            );
            previous = deviation;
        }
    }

    #[test]
    fn test_escape_candidates_respect_max_tilt() {
        let vertical = Vector3D::new(0.0, 0.0, 1.0);
        let max_tilt = 20.0;

        let candidates = collision_escape_candidates(vertical, (1.0, 0.0), 5.0, max_tilt, 20);
        assert!(!candidates.is_empty());

        for c in &candidates {
            assert!(
                angle_deg(c, &vertical) <= max_tilt + 1e-6,
                "candidate tilted {:.4} deg from vertical, past the {} deg axis limit",
                angle_deg(c, &vertical),
                max_tilt
            );
        }
    }

    /// A vertical segment still gets the plain outward sweep, so existing
    /// behaviour for untilted toolpaths is unchanged apart from skipping the
    /// redundant zero-tilt probe.
    #[test]
    fn test_vertical_current_tilts_along_escape_direction() {
        let vertical = Vector3D::new(0.0, 0.0, 1.0);
        let candidates = collision_escape_candidates(vertical, (1.0, 0.0), 5.0, 45.0, 9);

        let first = candidates[0];
        let expected = 5_f64.to_radians();
        assert!(
            (first.x - expected.sin()).abs() < 1e-9,
            "expected a tilt toward +X, got x={}",
            first.x
        );
        assert!(first.y.abs() < 1e-9, "no Y component expected, got {}", first.y);
        assert!(
            (first.z - expected.cos()).abs() < 1e-9,
            "expected z=cos(5 deg), got {}",
            first.z
        );
    }
}
