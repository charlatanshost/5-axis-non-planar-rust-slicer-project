// Main GUI application state and update logic

use crate::{Mesh, Result};
use crate::slicing::{Slicer, SlicingConfig, Layer};
use crate::toolpath::{Toolpath, ToolpathGenerator};
use crate::gcode::GCodeGenerator;
use crate::gui::{viewport_3d::Viewport3D, theme};
use crate::support_generation::{SupportGenerator, SupportResult, OverhangConfig};
use crate::s3_slicer::{
    execute_s3_pipeline, S3PipelineConfig, FabricationObjective,
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
}

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

    // Support generation
    pub support_config: OverhangConfig,
    pub support_result: Option<SupportResult>,
    pub has_supports: bool,
    pub show_supports: bool,

    // UI state
    pub show_mesh: bool,
    pub show_layers: bool,
    pub show_toolpaths: bool,
    pub show_wireframe: bool,
    pub selected_layer: Option<usize>,
    pub show_gcode_terminal: bool,
    pub gcode_scroll_to_bottom: bool,
    pub toolpath_playback_enabled: bool,
    pub toolpath_playback_position: usize, // Current segment index
    pub toolpath_playback_playing: bool,
    pub toolpath_playback_speed: f32, // Segments per second
    pub gcode_highlight_line: Option<usize>, // Line to highlight in terminal
    pub last_playback_time: std::time::Instant,

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

    // 3D viewport
    pub viewport_3d: Option<Viewport3D>,

    // S3-Slicer deformation configuration
    pub s3_fabrication_objective: FabricationObjective,
    pub s3_overhang_threshold: f64,        // degrees (30-60)
    pub s3_smoothness_weight: f64,         // 0.0-1.0
    pub s3_optimization_iterations: usize, // 10-100

    // ASAP deformation settings (Phase 3)
    pub s3_use_asap_deformation: bool,     // true = ASAP (quality), false = scale-controlled (fast)
    pub s3_asap_max_iterations: usize,     // 5-20
    pub s3_asap_convergence: f64,          // 1e-5 to 1e-3

    // Face orientation mode
    pub face_orientation_mode: bool,       // Whether we're in "select face to orient" mode
    pub selected_face_index: Option<usize>, // Index of the selected triangle

    // Logs
    pub log_messages: Vec<String>,
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

            support_config: OverhangConfig::default(),
            support_result: None,
            has_supports: false,
            show_supports: true,

            show_mesh: true,
            show_layers: true,
            show_toolpaths: true,
            show_wireframe: false,
            selected_layer: None,
            show_gcode_terminal: true,
            gcode_scroll_to_bottom: false,
            toolpath_playback_enabled: false,
            toolpath_playback_position: 0,
            toolpath_playback_playing: false,
            toolpath_playback_speed: 10.0, // 10 segments per second
            gcode_highlight_line: None,
            last_playback_time: std::time::Instant::now(),

            slicing_progress: Arc::new(Mutex::new(SlicingProgress::default())),
            is_slicing: false,
            has_sliced: false,
            has_toolpaths: false,
            has_motion_plan: false,
            is_motion_planning: false,

            layers_receiver: None,
            deformed_mesh_receiver: None,

            viewport_3d: None,

            // S3-Slicer defaults
            s3_fabrication_objective: FabricationObjective::SupportFree,
            s3_overhang_threshold: 45.0,
            s3_smoothness_weight: 0.5,
            s3_optimization_iterations: 50,

            // ASAP deformation defaults
            s3_use_asap_deformation: false, // Default to fast mode
            s3_asap_max_iterations: 10,
            s3_asap_convergence: 1e-4,

            // Face orientation defaults
            face_orientation_mode: false,
            selected_face_index: None,

            log_messages: vec!["Welcome to MultiAxis Slicer!".to_string()],
        }
    }
}

impl SlicerApp {
    /// Create a new slicer application
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Apply custom theme
        theme::apply_theme(&cc.egui_ctx);

        let mut app = Self::default();

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

        // Get ASAP deformation settings
        let use_asap_deformation = self.s3_use_asap_deformation;
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
                // ASAP deformation settings
                use_asap_deformation,
                asap_max_iterations,
                asap_convergence_threshold: asap_convergence,
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

    /// Deform mesh using S3-Slicer algorithm (for preview before slicing)
    pub fn deform_mesh_preview(&mut self) {
        let mesh = match &self.mesh {
            Some(m) => m.clone(),
            None => {
                self.log("No mesh loaded!".to_string());
                return;
            }
        };

        self.log("Deforming mesh using S3-Slicer algorithm...".to_string());
        self.is_deforming = true;
        let fabrication_objective = self.s3_fabrication_objective;
        let overhang_threshold = self.s3_overhang_threshold;
        let smoothness_weight = self.s3_smoothness_weight;
        let optimization_iterations = self.s3_optimization_iterations;
        let max_rotation_degrees = self.config.max_rotation_degrees;

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
            log::info!("Step 2/2: Deforming mesh...");
            let deformation = S3SlicerDeformation::new(mesh.clone(), quaternion_field);
            let deformed = deformation.get_deformed_mesh().clone();
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

        self.log(format!("Generating {} toolpaths...", self.toolpath_pattern.name()));

        use crate::toolpath_patterns::ToolpathConfig;
        let pattern_config = ToolpathConfig {
            pattern: self.toolpath_pattern,
            line_width: self.toolpath_line_width,
            node_distance: 1.0,
            infill_density: self.toolpath_infill_density,
        };

        let generator = ToolpathGenerator::new(self.nozzle_diameter, self.config.layer_height)
            .with_config(pattern_config);

        self.toolpaths = generator.generate(&self.layers);
        self.has_toolpaths = true;

        let total_moves: usize = self.toolpaths.iter().map(|tp| tp.paths.len()).sum();
        self.log(format!("Generated {} toolpath moves using {} pattern", total_moves, self.toolpath_pattern.name()));
        self.log("💡 Tip: Run Motion Planning to optimize for singularities and collisions".to_string());
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
                self.log("  - Singularities avoided".to_string());
                self.log("  - Collisions checked".to_string());
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

        let gcode_gen = GCodeGenerator::new();
        self.gcode_lines = gcode_gen.generate_to_string(&self.toolpaths);
        self.log(format!("✓ Generated {} lines of G-code", self.gcode_lines.len()));
    }

    /// Export G-code
    pub fn export_gcode(&mut self, path: PathBuf) {
        if self.toolpaths.is_empty() {
            self.log("No toolpaths to export!".to_string());
            return;
        }

        self.log(format!("Exporting G-code to {:?}", path));

        let gcode_gen = GCodeGenerator::new();
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
            }
        }

        // Check for completed deformation results
        if let Some(ref receiver) = self.deformed_mesh_receiver {
            if let Ok(deformed_mesh) = receiver.try_recv() {
                self.deformed_mesh = Some(deformed_mesh);
                self.is_deforming = false;
                self.show_deformed_mesh = true; // Automatically show the deformed mesh
                self.log("✓ Mesh deformation complete - showing deformed mesh".to_string());
                self.log("💡 Tip: Use the toggle button to switch between original and deformed views".to_string());
                self.deformed_mesh_receiver = None; // Clear the receiver

                // Update viewport to show deformed mesh
                if let Some(viewport) = &mut self.viewport_3d {
                    if let Some(ref deformed) = self.deformed_mesh {
                        viewport.set_mesh(deformed);
                    }
                }
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

        // Handle automatic playback
        if self.toolpath_playback_playing && self.toolpath_playback_enabled {
            let total_segments: usize = self.toolpaths.iter()
                .map(|tp| tp.paths.len())
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
        self.render_control_panel(ctx);
        self.render_gcode_terminal(ctx);
        self.render_central_panel(ctx);
        self.render_stats_panel(ctx);
    }

    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
        // Save application state if needed
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
            .min_height(100.0)
            .max_height(600.0)
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
        });
    }

    fn render_stats_panel(&mut self, ctx: &egui::Context) {
        use crate::gui::stats_panel;
        stats_panel::render(self, ctx);
    }
}

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
