// 3D visualization viewport with hardware-accelerated rendering using three-d

use crate::gui::app::{SlicerApp, BedShape};
use crate::Mesh as SlicerMesh;
use egui;
use eframe::glow;
use std::sync::Arc;
use three_d::*;

/// 3D viewport for visualizing mesh, layers, and toolpaths
pub struct Viewport3D {
    // Three-d context
    context: Context,

    // Camera state
    pub camera_yaw: f32,
    pub camera_pitch: f32,
    pub camera_distance: f32,
    pub camera_target: [f32; 3],

    // Default camera state for reset (H key)
    default_camera_yaw: f32,
    default_camera_pitch: f32,
    default_camera_distance: f32,
    default_camera_target: [f32; 3],

    // Mouse interaction state
    last_mouse_pos: Option<egui::Pos2>,
    is_rotating: bool,
    is_panning: bool,

    // Cached mesh data for rendering
    mesh_object: Option<Arc<Gm<Mesh, PhysicalMaterial>>>,
    mesh_bounds: Option<([f32; 3], [f32; 3])>,

    // Build plate as 3D mesh
    build_plate_mesh: Option<Arc<Gm<Mesh, ColorMaterial>>>,

    // Toolpath visualization as instanced spheres
    toolpath_spheres: Option<Arc<Gm<InstancedMesh, ColorMaterial>>>,
    // Toolpath line connections between points
    toolpath_lines: Option<Arc<Gm<Mesh, ColorMaterial>>>,
    last_toolpath_count: usize,

    // Coordinate axes (X=red, Y=green, Z=blue)
    axes_mesh: Option<Arc<Gm<Mesh, ColorMaterial>>>,

    // Layer contour visualization
    layer_lines: Option<Arc<Gm<Mesh, ColorMaterial>>>,
    last_layer_count: usize,

    // Wireframe overlay
    wireframe_mesh: Option<Arc<Gm<Mesh, ColorMaterial>>>,
    last_wireframe_triangle_count: usize,

    // Playback tracking
    last_playback_position: usize,

    // Section view (mesh cut)
    section_mesh: Option<Arc<Gm<Mesh, PhysicalMaterial>>>,
    last_section_axis: u8,
    last_section_depth: f32,
    last_section_enabled: bool,

    // Section view tracking for layer/toolpath content rebuild
    last_section_axis_for_content: u8,
    last_section_depth_for_content: f32,
    last_section_enabled_for_content: bool,

    // Travel moves visibility tracking
    last_show_travel_moves: bool,

    // Face selection
    last_mesh_triangle_count: usize,

    // Machine simulation geometry
    machine_head_gm: Option<Arc<Gm<Mesh, ColorMaterial>>>,
    machine_bed_gm: Option<Arc<Gm<Mesh, ColorMaterial>>>,
    machine_profile_sig: u64,  // simple hash to detect profile or playback changes
    collision_lines_gm: Option<Arc<Gm<Mesh, ColorMaterial>>>,
    collision_lines_sig: usize, // collision_segments.len() at last rebuild
}

impl Viewport3D {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Get the glow context from eframe
        let gl = cc.gl.as_ref().expect("Failed to get GL context");

        // Create three-d context from the glow context
        let context = Context::from_gl_context(gl.clone()).expect("Failed to create three-d context");

        Self {
            context,
            camera_yaw: 45.0,
            camera_pitch: 30.0,
            camera_distance: 100.0,
            camera_target: [0.0, 0.0, 0.0],

            // Initialize default camera to same as current
            default_camera_yaw: 45.0,
            default_camera_pitch: 30.0,
            default_camera_distance: 100.0,
            default_camera_target: [0.0, 0.0, 0.0],

            last_mouse_pos: None,
            is_rotating: false,
            is_panning: false,

            mesh_object: None,
            mesh_bounds: None,
            build_plate_mesh: None,
            toolpath_spheres: None,
            toolpath_lines: None,
            last_toolpath_count: 0,
            axes_mesh: None,
            layer_lines: None,
            last_layer_count: 0,
            wireframe_mesh: None,
            last_wireframe_triangle_count: 0,
            last_playback_position: usize::MAX,
            section_mesh: None,
            last_section_axis: 255,
            last_section_depth: 1.0,
            last_section_enabled: false,
            last_section_axis_for_content: 255,
            last_section_depth_for_content: 1.0,
            last_section_enabled_for_content: false,
            last_show_travel_moves: true,
            last_mesh_triangle_count: 0,

            machine_head_gm: None,
            machine_bed_gm: None,
            machine_profile_sig: u64::MAX,
            collision_lines_gm: None,
            collision_lines_sig: usize::MAX,
        }
    }

    /// Set mesh bounds and prepare for rendering
    pub fn set_mesh(&mut self, mesh: &SlicerMesh) {
        log::warn!("========================================");
        log::warn!("VIEWPORT: set_mesh() CALLED - Loading mesh to viewport!");
        log::warn!("========================================");

        let bounds_min = [
            mesh.bounds_min.x as f32,
            mesh.bounds_min.y as f32,
            mesh.bounds_min.z as f32,
        ];
        let bounds_max = [
            mesh.bounds_max.x as f32,
            mesh.bounds_max.y as f32,
            mesh.bounds_max.z as f32,
        ];

        // Calculate mesh center
        let mesh_center = [
            (bounds_min[0] + bounds_max[0]) / 2.0,
            (bounds_min[1] + bounds_max[1]) / 2.0,
            (bounds_min[2] + bounds_max[2]) / 2.0,
        ];

        // Calculate ideal camera distance
        let size = [
            bounds_max[0] - bounds_min[0],
            bounds_max[1] - bounds_min[1],
            bounds_max[2] - bounds_min[2],
        ];
        let max_dim = size[0].max(size[1]).max(size[2]);

        // Set camera to look at mesh center
        self.camera_distance = max_dim * 2.0;
        self.camera_target = mesh_center;

        // Save default camera position for reset (H key)
        self.default_camera_yaw = 45.0;
        self.default_camera_pitch = 30.0;
        self.default_camera_distance = self.camera_distance;
        self.default_camera_target = self.camera_target;

        // Calculate bounding box diagonal for diagnostics
        let dx = bounds_max[0] - bounds_min[0];
        let dy = bounds_max[1] - bounds_min[1];
        let dz = bounds_max[2] - bounds_min[2];
        let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();

        log::info!("Viewport: Setting mesh bounds");
        log::info!("  Bounds min: ({:.2}, {:.2}, {:.2})", bounds_min[0], bounds_min[1], bounds_min[2]);
        log::info!("  Bounds max: ({:.2}, {:.2}, {:.2})", bounds_max[0], bounds_max[1], bounds_max[2]);
        log::info!("  Mesh center: ({:.2}, {:.2}, {:.2})", mesh_center[0], mesh_center[1], mesh_center[2]);
        log::info!("  Bounding box diagonal: {:.2}mm", diagonal);
        log::info!("  Max dimension: {:.2}mm", max_dim);
        log::info!("  Camera distance: {:.2}mm ({:.1}× max dim)", self.camera_distance, self.camera_distance / max_dim);
        log::info!("  Triangles: {}", mesh.triangles.len());

        self.mesh_bounds = Some((bounds_min, bounds_max));

        // Convert mesh to three-d format
        self.load_mesh_to_gpu(mesh);

        // Create build plate grid at the minimum Z
        self.create_build_plate(bounds_min[2]);

        // Create coordinate axes at origin
        self.create_coordinate_axes();
    }

    /// Convert SlicerMesh to three-d mesh and upload to GPU with FLAT shading (per-face normals)
    fn load_mesh_to_gpu(&mut self, mesh: &SlicerMesh) {
        log::warn!("========================================");
        log::warn!("VIEWPORT: load_mesh_to_gpu() STARTED");
        log::warn!("  Input triangles: {}", mesh.triangles.len());
        log::warn!("========================================");

        // Collect all positions and compute per-face normals
        // Using FLAT shading to avoid scrambled appearance with multi-part meshes
        let mut positions = Vec::new();
        let mut normals = Vec::new();

        for triangle in mesh.triangles.iter() {
            let v0 = vec3(triangle.v0.x as f32, triangle.v0.y as f32, triangle.v0.z as f32);
            let v1 = vec3(triangle.v1.x as f32, triangle.v1.y as f32, triangle.v1.z as f32);
            let v2 = vec3(triangle.v2.x as f32, triangle.v2.y as f32, triangle.v2.z as f32);

            positions.push(v0);
            positions.push(v1);
            positions.push(v2);

            // Compute face normal using right-hand rule
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let mut normal = edge1.cross(edge2);
            let len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
            if len > 1e-6 {
                normal = vec3(normal.x / len, normal.y / len, normal.z / len);
            } else {
                normal = vec3(0.0, 1.0, 0.0); // Fallback for degenerate triangles
            }

            // NOTE: We DON'T flip normals even if they point inward, because:
            // 1. We disabled backface culling (Cull::None)
            // 2. Both sides of triangles render anyway
            // 3. Flipping could make inconsistently wound STLs look worse

            // Flat shading: all 3 vertices of triangle get same normal
            normals.push(normal);
            normals.push(normal);
            normals.push(normal);
        }

        // Create indices (simple sequential indexing for flat shading)
        let mut indices = Vec::new();
        for i in 0..mesh.triangles.len() {
            let base = (i * 3) as u32;
            indices.push(base);
            indices.push(base + 1);
            indices.push(base + 2);
        }

        log::info!("Converting mesh to GPU format (with FLAT shading per-face normals):");
        log::info!("  Triangles: {}", mesh.triangles.len());
        log::info!("  Vertices: {}", positions.len());
        log::info!("  Flat normals: {}", normals.len());
        log::info!("  Indices: {}", indices.len());

        // DIAGNOSTIC: Detect spurious long triangles that might connect separate parts
        let edge_threshold = 20.0; // mm - triangles with edges longer than this are suspicious
        let mut long_triangle_count = 0;
        let mut long_triangles = Vec::new();

        for i in 0..mesh.triangles.len() {
            let base = i * 3;
            let v0 = positions[base];
            let v1 = positions[base + 1];
            let v2 = positions[base + 2];

            // Calculate edge lengths
            let edge1_len = ((v1.x - v0.x).powi(2) + (v1.y - v0.y).powi(2) + (v1.z - v0.z).powi(2)).sqrt();
            let edge2_len = ((v2.x - v1.x).powi(2) + (v2.y - v1.y).powi(2) + (v2.z - v1.z).powi(2)).sqrt();
            let edge3_len = ((v0.x - v2.x).powi(2) + (v0.y - v2.y).powi(2) + (v0.z - v2.z).powi(2)).sqrt();

            let max_edge = edge1_len.max(edge2_len).max(edge3_len);

            if max_edge > edge_threshold {
                long_triangle_count += 1;
                if long_triangles.len() < 10 {
                    long_triangles.push((i, max_edge, v0, v1, v2));
                }
            }
        }

        if long_triangle_count > 0 {
            log::warn!("========================================");
            log::warn!("SPURIOUS TRIANGLE DETECTION:");
            log::warn!("  Found {} triangles with edges longer than {:.1}mm", long_triangle_count, edge_threshold);
            log::warn!("  This likely indicates BAD STL GEOMETRY connecting separate parts!");
            log::warn!("========================================");

            for (idx, max_edge, v0, v1, v2) in long_triangles.iter() {
                log::warn!("  Triangle {} (max edge: {:.1}mm):", idx, max_edge);
                log::warn!("    v0: ({:.2}, {:.2}, {:.2})", v0.x, v0.y, v0.z);
                log::warn!("    v1: ({:.2}, {:.2}, {:.2})", v1.x, v1.y, v1.z);
                log::warn!("    v2: ({:.2}, {:.2}, {:.2})", v2.x, v2.y, v2.z);
            }
        } else {
            log::info!("  No spurious long triangles detected (all edges < {:.1}mm)", edge_threshold);
        }

        // Log first few triangles to check geometry
        if positions.len() >= 15 {
            log::info!("  First 5 triangles:");
            for i in 0..5.min(mesh.triangles.len()) {
                let base = i * 3;
                log::info!("    Triangle {}: ({:.1}, {:.1}, {:.1}) ({:.1}, {:.1}, {:.1}) ({:.1}, {:.1}, {:.1})",
                    i,
                    positions[base].x, positions[base].y, positions[base].z,
                    positions[base+1].x, positions[base+1].y, positions[base+1].z,
                    positions[base+2].x, positions[base+2].y, positions[base+2].z
                );
            }
        }

        // Create vertex colors (all same color) to force solid rendering
        let colors = vec![Srgba::new(220, 220, 230, 255); positions.len()];

        // Create CPU mesh with explicit normals and colors
        let cpu_mesh = CpuMesh {
            positions: Positions::F32(positions),
            normals: Some(normals),
            colors: Some(colors),
            indices: Indices::U32(indices),
            ..Default::default()
        };

        log::info!("Created CpuMesh with positions, normals, colors, and indices");

        // Create GPU mesh
        let gpu_mesh = Mesh::new(&self.context, &cpu_mesh);

        log::info!("Created GPU mesh successfully");

        // Create PhysicalMaterial with smooth shading + PBR lighting + NO backface culling
        let material = PhysicalMaterial::new_opaque(
            &self.context,
            &CpuMaterial {
                albedo: Srgba::new(220, 220, 230, 255), // Neutral light gray
                roughness: 0.7,  // Matte but not fully diffuse (like PLA plastic)
                metallic: 0.05,  // Slight metallic sheen
                ..Default::default()
            },
        );

        // CRITICAL FIX: Override render states to DISABLE backface culling
        // STL files have inconsistent triangle winding, causing 50%+ faces to disappear with culling
        let mut material = material;
        material.render_states.cull = Cull::None;
        material.render_states.depth_test = DepthTest::Less;

        log::info!("Created PhysicalMaterial with Cull::None (FLAT shading + PBR + no culling)");
        log::info!("Material render_states: cull={:?}, depth_test={:?}",
            material.render_states.cull,
            material.render_states.depth_test);

        // Create renderable object wrapped in Arc for sharing with render callback
        self.mesh_object = Some(Arc::new(Gm::new(gpu_mesh, material)));
        self.last_mesh_triangle_count = mesh.triangles.len();

        log::info!("✓ Successfully loaded mesh to GPU: {} triangles", mesh.triangles.len());
    }

    /// Clear the cached mesh data
    pub fn clear_mesh(&mut self) {
        log::info!("Clearing viewport mesh cache - will reload on next frame");
        self.mesh_bounds = None;
        self.mesh_object = None;
        self.build_plate_mesh = None;
        self.toolpath_spheres = None;
        self.toolpath_lines = None;
        self.axes_mesh = None;
        self.layer_lines = None;
        self.wireframe_mesh = None;
        self.last_toolpath_count = 0;
        self.last_layer_count = 0;
        self.last_wireframe_triangle_count = 0;
        self.last_mesh_triangle_count = 0;
        self.section_mesh = None;
        self.last_section_enabled = false;
        self.last_section_axis_for_content = 255;
        self.last_section_depth_for_content = 1.0;
        self.last_section_enabled_for_content = false;
        self.last_show_travel_moves = true;
        self.last_playback_position = usize::MAX;
    }

    /// Create build plate grid as 3D mesh
    fn create_build_plate(&mut self, z_height: f32) {
        let size = 300.0; // 300mm build plate
        let grid_spacing = 10.0; // 10mm grid lines
        let num_lines = (size / grid_spacing) as i32;

        let mut positions = Vec::new();
        let mut indices = Vec::new();

        // Create grid lines as thin quads (rendered as line segments)
        let line_width = 0.2; // Very thin

        // Lines parallel to X axis (along Y direction)
        for i in -num_lines..=num_lines {
            let y = i as f32 * grid_spacing;
            let x_start = -size / 2.0;
            let x_end = size / 2.0;

            let base_idx = positions.len() as u32;

            // Create a thin quad for this line
            positions.push(vec3(x_start, y - line_width, z_height));
            positions.push(vec3(x_end, y - line_width, z_height));
            positions.push(vec3(x_end, y + line_width, z_height));
            positions.push(vec3(x_start, y + line_width, z_height));

            // Two triangles for the quad
            indices.push(base_idx);
            indices.push(base_idx + 1);
            indices.push(base_idx + 2);

            indices.push(base_idx);
            indices.push(base_idx + 2);
            indices.push(base_idx + 3);
        }

        // Lines parallel to Y axis (along X direction)
        for i in -num_lines..=num_lines {
            let x = i as f32 * grid_spacing;
            let y_start = -size / 2.0;
            let y_end = size / 2.0;

            let base_idx = positions.len() as u32;

            // Create a thin quad for this line
            positions.push(vec3(x - line_width, y_start, z_height));
            positions.push(vec3(x + line_width, y_start, z_height));
            positions.push(vec3(x + line_width, y_end, z_height));
            positions.push(vec3(x - line_width, y_end, z_height));

            // Two triangles for the quad
            indices.push(base_idx);
            indices.push(base_idx + 1);
            indices.push(base_idx + 2);

            indices.push(base_idx);
            indices.push(base_idx + 2);
            indices.push(base_idx + 3);
        }

        log::info!("Created build plate grid: {} lines, {} vertices, {} indices",
            num_lines * 2 + 2, positions.len(), indices.len());

        // Create CPU mesh
        let cpu_mesh = CpuMesh {
            positions: Positions::F32(positions),
            indices: Indices::U32(indices),
            ..Default::default()
        };

        // Create GPU mesh
        let gpu_mesh = Mesh::new(&self.context, &cpu_mesh);

        // Create material - gray grid
        let material = ColorMaterial {
            color: Srgba::new(80, 80, 80, 255),
            ..Default::default()
        };

        self.build_plate_mesh = Some(Arc::new(Gm::new(gpu_mesh, material)));

        log::info!("✓ Build plate mesh created at Z = {:.2}", z_height);
    }

    /// Create XYZ coordinate axes at origin (X=red, Y=green, Z=blue)
    fn create_coordinate_axes(&mut self) {
        let mut positions = Vec::new();
        let mut colors = Vec::new();
        let mut indices = Vec::new();

        let axis_length = 50.0; // 50mm long axes
        let axis_width = 0.5;   // Thickness of axis lines

        // Helper to create a thick line (as a thin box) between two points
        let mut add_axis_line = |start: [f32; 3], end: [f32; 3], color: Srgba| {
            // Create 6 vertices for a thin rectangular prism (box)
            let base_idx = positions.len() as u32;

            // For simplicity, create thin quads perpendicular to the axis
            let dir = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];

            // Determine which axis we're drawing
            let is_x = dir[0].abs() > 0.1;
            let is_y = dir[1].abs() > 0.1;
            let is_z = dir[2].abs() > 0.1;

            // Create a thin quad perpendicular to the axis
            if is_x {
                // X axis: create quad in YZ plane
                positions.push(vec3(start[0], start[1] - axis_width, start[2] - axis_width));
                positions.push(vec3(end[0], end[1] - axis_width, end[2] - axis_width));
                positions.push(vec3(end[0], end[1] + axis_width, end[2] + axis_width));
                positions.push(vec3(start[0], start[1] + axis_width, start[2] + axis_width));
            } else if is_y {
                // Y axis: create quad in XZ plane
                positions.push(vec3(start[0] - axis_width, start[1], start[2] - axis_width));
                positions.push(vec3(end[0] - axis_width, end[1], end[2] - axis_width));
                positions.push(vec3(end[0] + axis_width, end[1], end[2] + axis_width));
                positions.push(vec3(start[0] + axis_width, start[1], start[2] + axis_width));
            } else if is_z {
                // Z axis: create quad in XY plane
                positions.push(vec3(start[0] - axis_width, start[1] - axis_width, start[2]));
                positions.push(vec3(end[0] - axis_width, end[1] - axis_width, end[2]));
                positions.push(vec3(end[0] + axis_width, end[1] + axis_width, end[2]));
                positions.push(vec3(start[0] + axis_width, start[1] + axis_width, start[2]));
            }

            // Add color for all 4 vertices
            for _ in 0..4 {
                colors.push(color);
            }

            // Two triangles for the quad
            indices.push(base_idx);
            indices.push(base_idx + 1);
            indices.push(base_idx + 2);

            indices.push(base_idx);
            indices.push(base_idx + 2);
            indices.push(base_idx + 3);
        };

        // X axis - Red
        add_axis_line([0.0, 0.0, 0.0], [axis_length, 0.0, 0.0], Srgba::new(255, 0, 0, 255));

        // Y axis - Green
        add_axis_line([0.0, 0.0, 0.0], [0.0, axis_length, 0.0], Srgba::new(0, 255, 0, 255));

        // Z axis - Blue
        add_axis_line([0.0, 0.0, 0.0], [0.0, 0.0, axis_length], Srgba::new(0, 0, 255, 255));

        log::info!("Created coordinate axes: {} vertices, {} indices", positions.len(), indices.len());

        // Create CPU mesh
        let cpu_mesh = CpuMesh {
            positions: Positions::F32(positions),
            colors: Some(colors),
            indices: Indices::U32(indices),
            ..Default::default()
        };

        // Create GPU mesh
        let gpu_mesh = Mesh::new(&self.context, &cpu_mesh);

        // Material uses vertex colors
        let material = ColorMaterial::default();

        self.axes_mesh = Some(Arc::new(Gm::new(gpu_mesh, material)));

        log::info!("✓ Created XYZ coordinate axes (X=red, Y=green, Z=blue)");
    }

    /// Create 3D visualization of layer contours as colored tube lines
    fn create_layer_lines(&mut self, app: &SlicerApp) {
        if app.layers.is_empty() {
            self.layer_lines = None;
            self.last_layer_count = 0;
            return;
        }

        // Compute section clip plane (if active)
        let mesh_bounds_copy = self.mesh_bounds;
        let section_filter: Option<(usize, f32)> = if app.section_enabled {
            mesh_bounds_copy.map(|(bounds_min, bounds_max)| {
                let ax = app.section_axis.min(2) as usize;
                let cut_pos = bounds_min[ax] + app.section_depth * (bounds_max[ax] - bounds_min[ax]);
                (ax, cut_pos)
            })
        } else {
            None
        };

        let mut positions: Vec<Vec3> = Vec::new();
        let mut normals: Vec<Vec3> = Vec::new();
        let mut colors: Vec<Srgba> = Vec::new();
        let mut indices: Vec<u32> = Vec::new();

        let tube_radius = 0.15;
        let tube_segments = 3; // Triangular cross-section for performance

        // Find min/max Z for color gradient
        let min_z = app.layers.iter().map(|l| l.z).fold(f64::INFINITY, f64::min) as f32;
        let max_z = app.layers.iter().map(|l| l.z).fold(f64::NEG_INFINITY, f64::max) as f32;
        let z_range = (max_z - min_z).max(0.1);

        let mut total_segments = 0usize;

        for layer in &app.layers {
            // Color gradient: blue (240°) at bottom → magenta (300°) at top
            let t = ((layer.z as f32) - min_z) / z_range;
            let hue = 240.0 + 60.0 * t; // 240° = blue, 300° = magenta
            let (r, g, b) = hsl_to_rgb(hue, 0.9, 0.55);
            let color = Srgba::new((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8, 255);

            for contour in &layer.contours {
                if contour.points.len() < 2 {
                    continue;
                }

                for window in contour.points.windows(2) {
                    let p1 = vec3(window[0].x as f32, window[0].y as f32, window[0].z as f32);
                    let p2 = vec3(window[1].x as f32, window[1].y as f32, window[1].z as f32);

                    // Section clip: skip if either endpoint is beyond the cut plane
                    if let Some((ax, cut_pos)) = section_filter {
                        let p1c = match ax { 0 => p1.x, 1 => p1.y, _ => p1.z };
                        let p2c = match ax { 0 => p2.x, 1 => p2.y, _ => p2.z };
                        if p1c > cut_pos || p2c > cut_pos { continue; }
                    }

                    create_tube_segment(
                        p1, p2, tube_radius, tube_segments, color,
                        &mut positions, &mut normals, &mut colors, &mut indices,
                    );
                    total_segments += 1;
                }

                // Close the contour if it's closed
                if contour.closed && contour.points.len() >= 3 {
                    let first = contour.points.first().unwrap();
                    let last = contour.points.last().unwrap();
                    let p1 = vec3(last.x as f32, last.y as f32, last.z as f32);
                    let p2 = vec3(first.x as f32, first.y as f32, first.z as f32);

                    if let Some((ax, cut_pos)) = section_filter {
                        let p1c = match ax { 0 => p1.x, 1 => p1.y, _ => p1.z };
                        let p2c = match ax { 0 => p2.x, 1 => p2.y, _ => p2.z };
                        if p1c > cut_pos || p2c > cut_pos { continue; }
                    }

                    create_tube_segment(
                        p1, p2, tube_radius, tube_segments, color,
                        &mut positions, &mut normals, &mut colors, &mut indices,
                    );
                    total_segments += 1;
                }
            }
        }

        self.last_layer_count = app.layers.len();

        if positions.is_empty() {
            self.layer_lines = None;
            return;
        }

        let cpu_mesh = CpuMesh {
            positions: Positions::F32(positions),
            normals: Some(normals),
            colors: Some(colors),
            indices: Indices::U32(indices),
            ..Default::default()
        };

        let gpu_mesh = Mesh::new(&self.context, &cpu_mesh);
        self.layer_lines = Some(Arc::new(Gm::new(gpu_mesh, ColorMaterial::default())));

        log::info!("✓ Layer visualization: {} layers, {} segments (BLUE→MAGENTA by height)",
            app.layers.len(), total_segments);
    }

    /// Create wireframe overlay showing mesh triangle edges as thin tubes
    fn create_wireframe_mesh(&mut self, mesh: &SlicerMesh) {
        use std::collections::HashSet;

        let mut positions: Vec<Vec3> = Vec::new();
        let mut normals: Vec<Vec3> = Vec::new();
        let mut colors: Vec<Srgba> = Vec::new();
        let mut indices: Vec<u32> = Vec::new();

        let tube_radius = 0.05;
        let tube_segments = 3;
        let wire_color = Srgba::new(255, 255, 255, 200);

        // Quantize vertex positions to weld STL duplicate vertices
        let quantize = |v: f64| -> i64 { (v * 1e3).round() as i64 };

        // Collect unique edges by quantized vertex positions
        let mut seen_edges: HashSet<((i64, i64, i64), (i64, i64, i64))> = HashSet::new();
        let mut edge_endpoints: Vec<(Vec3, Vec3)> = Vec::new();

        for tri in &mesh.triangles {
            let verts = [
                (tri.v0.x, tri.v0.y, tri.v0.z),
                (tri.v1.x, tri.v1.y, tri.v1.z),
                (tri.v2.x, tri.v2.y, tri.v2.z),
            ];
            let keys: Vec<(i64, i64, i64)> = verts.iter()
                .map(|v| (quantize(v.0), quantize(v.1), quantize(v.2)))
                .collect();

            for i in 0..3 {
                let j = (i + 1) % 3;
                let (a, b) = if keys[i] < keys[j] { (keys[i], keys[j]) } else { (keys[j], keys[i]) };
                if seen_edges.insert((a, b)) {
                    edge_endpoints.push((
                        vec3(verts[i].0 as f32, verts[i].1 as f32, verts[i].2 as f32),
                        vec3(verts[j].0 as f32, verts[j].1 as f32, verts[j].2 as f32),
                    ));
                }
            }
        }

        // Limit wireframe to avoid massive GPU uploads on complex meshes
        let max_edges = 200_000;
        let edges_to_render = if edge_endpoints.len() > max_edges {
            log::warn!("  Wireframe: capping at {} edges (mesh has {})", max_edges, edge_endpoints.len());
            &edge_endpoints[..max_edges]
        } else {
            &edge_endpoints[..]
        };

        for (p1, p2) in edges_to_render {
            create_tube_segment(
                *p1, *p2, tube_radius, tube_segments, wire_color,
                &mut positions, &mut normals, &mut colors, &mut indices,
            );
        }

        self.last_wireframe_triangle_count = mesh.triangles.len();

        if positions.is_empty() {
            self.wireframe_mesh = None;
            return;
        }

        let cpu_mesh = CpuMesh {
            positions: Positions::F32(positions),
            normals: Some(normals),
            colors: Some(colors),
            indices: Indices::U32(indices),
            ..Default::default()
        };

        let gpu_mesh = Mesh::new(&self.context, &cpu_mesh);
        self.wireframe_mesh = Some(Arc::new(Gm::new(gpu_mesh, ColorMaterial::default())));

        log::info!("✓ Wireframe: {} unique edges rendered", edges_to_render.len());
    }

    /// Create section view mesh by filtering triangles on one side of a clipping plane
    fn create_section_mesh(&mut self, mesh: &SlicerMesh, axis: u8, depth: f32) {
        let bounds_min = [mesh.bounds_min.x as f32, mesh.bounds_min.y as f32, mesh.bounds_min.z as f32];
        let bounds_max = [mesh.bounds_max.x as f32, mesh.bounds_max.y as f32, mesh.bounds_max.z as f32];
        let ax = axis.min(2) as usize;
        let cut_pos = bounds_min[ax] + depth * (bounds_max[ax] - bounds_min[ax]);

        let mut positions: Vec<Vec3> = Vec::new();
        let mut normals: Vec<Vec3> = Vec::new();

        for triangle in mesh.triangles.iter() {
            let v0 = vec3(triangle.v0.x as f32, triangle.v0.y as f32, triangle.v0.z as f32);
            let v1 = vec3(triangle.v1.x as f32, triangle.v1.y as f32, triangle.v1.z as f32);
            let v2 = vec3(triangle.v2.x as f32, triangle.v2.y as f32, triangle.v2.z as f32);

            // Centroid-based filtering
            let centroid_val = match ax {
                0 => (v0.x + v1.x + v2.x) / 3.0,
                1 => (v0.y + v1.y + v2.y) / 3.0,
                _ => (v0.z + v1.z + v2.z) / 3.0,
            };

            if centroid_val > cut_pos {
                continue;
            }

            positions.push(v0);
            positions.push(v1);
            positions.push(v2);

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let mut normal = edge1.cross(edge2);
            let len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
            if len > 1e-6 {
                normal = vec3(normal.x / len, normal.y / len, normal.z / len);
            } else {
                normal = vec3(0.0, 1.0, 0.0);
            }
            normals.push(normal);
            normals.push(normal);
            normals.push(normal);
        }

        if positions.is_empty() {
            self.section_mesh = None;
            return;
        }

        let num_tris = positions.len() / 3;
        let indices: Vec<u32> = (0..positions.len() as u32).collect();
        let colors = vec![Srgba::new(220, 220, 230, 255); positions.len()];

        let cpu_mesh = CpuMesh {
            positions: Positions::F32(positions),
            normals: Some(normals),
            colors: Some(colors),
            indices: Indices::U32(indices),
            ..Default::default()
        };

        let gpu_mesh = Mesh::new(&self.context, &cpu_mesh);
        let material = PhysicalMaterial::new_opaque(
            &self.context,
            &CpuMaterial {
                albedo: Srgba::new(220, 220, 230, 255),
                roughness: 0.7,
                metallic: 0.05,
                ..Default::default()
            },
        );
        let mut material = material;
        material.render_states.cull = Cull::None;
        material.render_states.depth_test = DepthTest::Less;

        self.section_mesh = Some(Arc::new(Gm::new(gpu_mesh, material)));
        self.last_section_axis = axis;
        self.last_section_depth = depth;

        log::info!("✓ Section view: {}/{} triangles visible (axis={}, depth={:.2})",
            num_tris, mesh.triangles.len(), ["X", "Y", "Z"][ax], depth);
    }

    /// Create 3D visualization of ALL toolpaths as thin tube geometry
    /// Color scheme:
    /// - EXTRUSION moves: Gradient from GREEN (start) → YELLOW → RED (end) based on queue position
    /// - TRAVEL moves: Bright CYAN - clearly different from extrusion colors
    fn create_toolpath_lines(&mut self, app: &SlicerApp) {
        if app.toolpaths.is_empty() {
            self.toolpath_spheres = None;
            self.toolpath_lines = None;
            self.last_toolpath_count = 0;
            return;
        }

        // Compute section clip plane (if active)
        let mesh_bounds_copy = self.mesh_bounds;
        let section_filter: Option<(usize, f32)> = if app.section_enabled {
            mesh_bounds_copy.map(|(bounds_min, bounds_max)| {
                let ax = app.section_axis.min(2) as usize;
                let cut_pos = bounds_min[ax] + app.section_depth * (bounds_max[ax] - bounds_min[ax]);
                (ax, cut_pos)
            })
        } else {
            None
        };

        // For tube segments (proper 3D geometry)
        let mut tube_positions: Vec<Vec3> = Vec::new();
        let mut tube_normals: Vec<Vec3> = Vec::new();
        let mut tube_colors: Vec<Srgba> = Vec::new();
        let mut tube_indices: Vec<u32> = Vec::new();

        let tube_segments = 4;     // Square cross-section for performance
        // Travel moves are always drawn thin so they stand out from extrusion.
        let travel_radius: f32 = 0.05;

        // Count total segments for gradient calculation
        let total_segments: usize = app.toolpaths.iter()
            .map(|tp| tp.paths.len().saturating_sub(1))
            .sum();

        log::info!("Creating toolpath visualization: {} layers, {} total segments",
            app.toolpaths.len(), total_segments);

        // Track global segment index for gradient color (motion planning queue position)
        let mut global_segment_idx = 0usize;

        // Process ALL toolpaths - no skipping!
        for toolpath in app.toolpaths.iter() {
            // Tube radius for extrusion = half the layer height, so the tube diameter
            // matches the deposited bead height.  Clamped to [0.04, 0.30] mm so even
            // very thin or very thick adaptive layers remain visible.
            let extrusion_radius: f32 = (toolpath.layer_height as f32 / 2.0).clamp(0.04, 0.30);

            // Create tube segments connecting consecutive points
            for window in toolpath.paths.windows(2) {
                let p1 = vec3(window[0].position.x as f32, window[0].position.y as f32, window[0].position.z as f32);
                let p2 = vec3(window[1].position.x as f32, window[1].position.y as f32, window[1].position.z as f32);

                // Determine if this is extrusion or travel move
                let is_extrusion = window[1].extrusion > 1e-6;

                // Skip travel moves if hidden
                if !app.show_travel_moves && !is_extrusion {
                    global_segment_idx += 1;
                    continue;
                }

                // Section clip: skip if either endpoint is beyond the cut plane
                if let Some((ax, cut_pos)) = section_filter {
                    let p1c = match ax { 0 => p1.x, 1 => p1.y, _ => p1.z };
                    let p2c = match ax { 0 => p2.x, 1 => p2.y, _ => p2.z };
                    if p1c > cut_pos || p2c > cut_pos {
                        global_segment_idx += 1;
                        continue;
                    }
                }

                let (segment_color, tube_radius) = if is_extrusion {
                    // EXTRUSION: Gradient GREEN (120°) -> YELLOW (60°) -> RED (0°)
                    // Based on position in motion planning queue
                    let t = global_segment_idx as f32 / total_segments.max(1) as f32;
                    let hue = 120.0 * (1.0 - t); // 120° = green, 0° = red
                    let (r, g, b) = hsl_to_rgb(hue, 1.0, 0.5);
                    (Srgba::new((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8, 255),
                     extrusion_radius)
                } else {
                    // TRAVEL: Bright cyan, thin tube
                    (Srgba::new(0, 220, 255, 220), travel_radius)
                };

                // Create tube geometry for this segment
                create_tube_segment(
                    p1, p2, tube_radius, tube_segments, segment_color,
                    &mut tube_positions, &mut tube_normals, &mut tube_colors, &mut tube_indices
                );

                global_segment_idx += 1;
            }
        }

        log::info!("  → {} tube vertices, {} indices", tube_positions.len(), tube_indices.len());

        self.last_toolpath_count = app.toolpaths.len();

        // No spheres - just tubes for cleaner visualization
        self.toolpath_spheres = None;

        // Create tube mesh for lines
        if tube_positions.is_empty() || tube_indices.is_empty() {
            self.toolpath_lines = None;
            return;
        }

        let tube_cpu_mesh = CpuMesh {
            positions: Positions::F32(tube_positions),
            normals: Some(tube_normals),
            colors: Some(tube_colors),
            indices: Indices::U32(tube_indices),
            ..Default::default()
        };

        let tube_gpu_mesh = Mesh::new(&self.context, &tube_cpu_mesh);
        let tube_material = ColorMaterial::default();

        self.toolpath_lines = Some(Arc::new(Gm::new(tube_gpu_mesh, tube_material)));

        log::info!("✓ Toolpath visualization: {} segments (GREEN→RED = extrusion queue, CYAN = travel)",
            total_segments);
    }

    /// Create 3D visualization of toolpaths up to a specific segment index (for playback)
    /// Shows a bright nozzle marker at the current position
    fn create_toolpath_lines_up_to(&mut self, app: &SlicerApp, max_segment: usize) {
        if app.toolpaths.is_empty() {
            self.toolpath_spheres = None;
            self.toolpath_lines = None;
            self.last_toolpath_count = 0;
            return;
        }

        // Compute section clip plane (if active)
        let mesh_bounds_copy = self.mesh_bounds;
        let section_filter: Option<(usize, f32)> = if app.section_enabled {
            mesh_bounds_copy.map(|(bounds_min, bounds_max)| {
                let ax = app.section_axis.min(2) as usize;
                let cut_pos = bounds_min[ax] + app.section_depth * (bounds_max[ax] - bounds_min[ax]);
                (ax, cut_pos)
            })
        } else {
            None
        };

        let mut tube_positions: Vec<Vec3> = Vec::new();
        let mut tube_normals: Vec<Vec3> = Vec::new();
        let mut tube_colors: Vec<Srgba> = Vec::new();
        let mut tube_indices: Vec<u32> = Vec::new();

        let tube_segments = 4;
        let travel_radius: f32 = 0.05;

        // Use visible count for gradient so color range covers what's shown
        let total_visible = max_segment + 1;
        let mut global_segment_idx = 0usize;

        'outer: for toolpath in app.toolpaths.iter() {
            // Extrusion tube radius scales with layer height (diameter ≈ bead height).
            let extrusion_radius: f32 = (toolpath.layer_height as f32 / 2.0).clamp(0.04, 0.30);

            for window in toolpath.paths.windows(2) {
                if global_segment_idx > max_segment {
                    break 'outer;
                }

                let p1 = vec3(window[0].position.x as f32, window[0].position.y as f32, window[0].position.z as f32);
                let p2 = vec3(window[1].position.x as f32, window[1].position.y as f32, window[1].position.z as f32);

                let is_extrusion = window[1].extrusion > 1e-6;
                let is_last = global_segment_idx == max_segment;

                // Skip travel moves if hidden (still advance index to keep playback position correct)
                if !app.show_travel_moves && !is_extrusion && !is_last {
                    global_segment_idx += 1;
                    continue;
                }

                // Section clip: skip if either endpoint is beyond the cut plane
                if let Some((ax, cut_pos)) = section_filter {
                    let p1c = match ax { 0 => p1.x, 1 => p1.y, _ => p1.z };
                    let p2c = match ax { 0 => p2.x, 1 => p2.y, _ => p2.z };
                    if (p1c > cut_pos || p2c > cut_pos) && !is_last {
                        global_segment_idx += 1;
                        continue;
                    }
                }

                let (segment_color, base_radius) = if is_last {
                    // Nozzle position: bright white
                    (Srgba::new(255, 255, 255, 255), extrusion_radius)
                } else if is_extrusion {
                    let t = global_segment_idx as f32 / total_visible.max(1) as f32;
                    let hue = 120.0 * (1.0 - t);
                    let (r, g, b) = hsl_to_rgb(hue, 1.0, 0.5);
                    (Srgba::new((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8, 255),
                     extrusion_radius)
                } else {
                    (Srgba::new(0, 220, 255, 220), travel_radius)
                };

                // Nozzle marker is 2.5× larger so it's easy to spot during playback
                let radius = if is_last { base_radius * 2.5 } else { base_radius };

                create_tube_segment(
                    p1, p2, radius, tube_segments, segment_color,
                    &mut tube_positions, &mut tube_normals, &mut tube_colors, &mut tube_indices
                );

                global_segment_idx += 1;
            }
        }

        self.last_toolpath_count = app.toolpaths.len();
        self.toolpath_spheres = None;

        if tube_positions.is_empty() || tube_indices.is_empty() {
            self.toolpath_lines = None;
            return;
        }

        let tube_cpu_mesh = CpuMesh {
            positions: Positions::F32(tube_positions),
            normals: Some(tube_normals),
            colors: Some(tube_colors),
            indices: Indices::U32(tube_indices),
            ..Default::default()
        };

        let tube_gpu_mesh = Mesh::new(&self.context, &tube_cpu_mesh);
        let tube_material = ColorMaterial::default();
        self.toolpath_lines = Some(Arc::new(Gm::new(tube_gpu_mesh, tube_material)));
    }

    /// Render build plate grid at specified Z height by projecting to 2D screen space
    fn render_build_plate(&self, rect: egui::Rect, painter: &egui::Painter, z_height: f32) {
        let size = 300.0; // 300mm build plate
        let grid_spacing = 10.0; // 10mm grid lines

        // Calculate camera matrices
        let yaw_rad = self.camera_yaw.to_radians();
        let pitch_rad = self.camera_pitch.to_radians();

        let camera_x = self.camera_target[0] + self.camera_distance * pitch_rad.cos() * yaw_rad.sin();
        let camera_y = self.camera_target[1] + self.camera_distance * pitch_rad.sin();
        let camera_z = self.camera_target[2] + self.camera_distance * pitch_rad.cos() * yaw_rad.cos();

        // Camera vectors
        let forward = [
            self.camera_target[0] - camera_x,
            self.camera_target[1] - camera_y,
            self.camera_target[2] - camera_z,
        ];
        let forward_len = (forward[0] * forward[0] + forward[1] * forward[1] + forward[2] * forward[2]).sqrt();
        let forward_norm = [forward[0] / forward_len, forward[1] / forward_len, forward[2] / forward_len];

        let right = [
            -forward_norm[2],
            0.0,
            forward_norm[0],
        ];
        let right_len = (right[0] * right[0] + right[2] * right[2]).sqrt();
        let right_norm = if right_len > 0.001 {
            [right[0] / right_len, 0.0, right[2] / right_len]
        } else {
            [1.0, 0.0, 0.0]
        };

        let up = [
            forward_norm[1] * right_norm[2] - forward_norm[2] * right_norm[1],
            forward_norm[2] * right_norm[0] - forward_norm[0] * right_norm[2],
            forward_norm[0] * right_norm[1] - forward_norm[1] * right_norm[0],
        ];

        let fov_factor = (45.0f32.to_radians() / 2.0).tan();
        let aspect = rect.width() / rect.height();

        // Project 3D point at build plate Z height to 2D screen space
        let project = |x: f32, y: f32| -> Option<egui::Pos2> {
            let dx = x - camera_x;
            let dy = y - camera_y;
            let dz = z_height - camera_z; // Build plate at mesh minimum Z

            let view_x = dx * right_norm[0] + dy * right_norm[1] + dz * right_norm[2];
            let view_y = dx * up[0] + dy * up[1] + dz * up[2];
            let view_z = dx * forward_norm[0] + dy * forward_norm[1] + dz * forward_norm[2];

            if view_z <= 0.0 {
                return None;
            }

            let ndc_x = (view_x / view_z) / (aspect * fov_factor);
            let ndc_y = -(view_y / view_z) / fov_factor;

            if ndc_x.abs() > 1.5 || ndc_y.abs() > 1.5 {
                return None;
            }

            let screen_x = rect.center().x + ndc_x * rect.width() * 0.5;
            let screen_y = rect.center().y + ndc_y * rect.height() * 0.5;

            Some(egui::pos2(screen_x, screen_y))
        };

        let num_lines = (size / grid_spacing) as i32;

        // Center the grid at the camera target's X-Y position
        let center_x = self.camera_target[0];
        let center_y = self.camera_target[1];

        // Draw grid lines parallel to X axis
        for i in -num_lines..=num_lines {
            let y = center_y + i as f32 * grid_spacing;
            if let (Some(p0), Some(p1)) = (
                project(center_x - size / 2.0, y),
                project(center_x + size / 2.0, y)
            ) {
                let color = if (y - center_y).abs() < 0.1 {
                    egui::Color32::from_rgb(150, 50, 50) // Red line at center Y
                } else {
                    egui::Color32::from_rgb(80, 80, 80)
                };
                painter.line_segment([p0, p1], egui::Stroke::new(1.0, color));
            }
        }

        // Draw grid lines parallel to Y axis
        for i in -num_lines..=num_lines {
            let x = center_x + i as f32 * grid_spacing;
            if let (Some(p0), Some(p1)) = (
                project(x, center_y - size / 2.0),
                project(x, center_y + size / 2.0)
            ) {
                let color = if (x - center_x).abs() < 0.1 {
                    egui::Color32::from_rgb(50, 150, 50) // Green line at center X
                } else {
                    egui::Color32::from_rgb(80, 80, 80)
                };
                painter.line_segment([p0, p1], egui::Stroke::new(1.0, color));
            }
        }
    }

    pub fn render(&mut self, ui: &mut egui::Ui, app: &SlicerApp) -> Option<usize> {
        let available_size = ui.available_size();

        // Log once when mesh is present
        if app.mesh.is_some() && self.mesh_bounds.is_none() {
            log::warn!("VIEWPORT: render() called with mesh present but not loaded yet!");
        }

        // Update mesh if it changed
        // When showing deformed mesh, display that instead of the original
        let display_mesh = if app.show_deformed_mesh {
            app.deformed_mesh.as_ref().or(app.mesh.as_ref())
        } else {
            app.mesh.as_ref()
        };
        if let Some(mesh) = display_mesh {
            if self.mesh_bounds.is_none() || mesh.triangles.len() != self.last_mesh_triangle_count {
                log::info!("Reloading mesh to viewport (bounds_none={}, count_changed={}, deformed={})",
                    self.mesh_bounds.is_none(),
                    mesh.triangles.len() != self.last_mesh_triangle_count,
                    app.show_deformed_mesh && app.deformed_mesh.is_some());
                self.set_mesh(mesh);
            }
        } else {
            if self.mesh_bounds.is_some() {
                self.clear_mesh();
            }
        }

        // Detect section/travel state changes (affects layer lines and toolpath lines)
        let section_content_changed = app.section_enabled != self.last_section_enabled_for_content
            || (app.section_enabled && (
                app.section_axis != self.last_section_axis_for_content
                || (app.section_depth - self.last_section_depth_for_content).abs() > 0.001
            ));
        let travel_changed = app.show_travel_moves != self.last_show_travel_moves;

        // Update toolpath lines — playback-aware rebuild
        if app.toolpath_playback_enabled {
            // Rebuild when playback position, toolpaths, section, or travel visibility changed
            if app.toolpath_playback_position != self.last_playback_position
                || app.toolpaths.len() != self.last_toolpath_count
                || section_content_changed
                || travel_changed
            {
                self.create_toolpath_lines_up_to(app, app.toolpath_playback_position);
                self.last_playback_position = app.toolpath_playback_position;
            }
        } else if app.toolpaths.len() != self.last_toolpath_count
            || self.last_playback_position != usize::MAX
            || section_content_changed
            || travel_changed
        {
            // Playback disabled or toolpaths/section/travel changed — show all
            log::info!("Updating toolpath visualization ({} layers)", app.toolpaths.len());
            self.create_toolpath_lines(app);
            self.last_playback_position = usize::MAX;
        }

        // Update machine simulation geometry
        {
            let profile_sig = app.profiles.get(app.active_profile_index)
                .map(|p| {
                    let show = p.show_machine as u64;
                    let bdims = ((p.bed_dims[0] as u32) as u64) << 32
                              | ((p.bed_dims[1] as u32) as u64) << 16
                              | (p.bed_dims[2] as u32) as u64;
                    let hdims = ((p.head_dims[0] as u32) as u64) << 32
                              | ((p.head_dims[1] as u32) as u64) << 16
                              | (p.head_dims[2] as u32) as u64;
                    let stl_h = p.bed_stl_path.as_ref()
                        .map(|s| s.bytes().fold(0u64, |a, b| a.wrapping_mul(31).wrapping_add(b as u64)))
                        .unwrap_or(0)
                        ^ p.head_stl_path.as_ref()
                        .map(|s| s.bytes().fold(0u64, |a, b| a.wrapping_mul(37).wrapping_add(b as u64)))
                        .unwrap_or(0);
                    let shape_h = match p.bed_shape { BedShape::Rectangle => 0u64, BedShape::Circle => 1 };
                    let grid_z_h = self.mesh_bounds.map(|(mn, _)| (mn[2] * 100.0) as u64).unwrap_or(0);
                    let nozzle_h = ((p.nozzle_radius * 10.0) as u64)
                        ^ (((p.nozzle_length * 10.0) as u64) << 20)
                        ^ (((p.head_stl_tip_offset[0] * 10.0) as u64).wrapping_mul(1000003))
                        ^ (((p.head_stl_tip_offset[1] * 10.0) as u64).wrapping_mul(999983))
                        ^ (((p.head_stl_tip_offset[2] * 10.0) as u64).wrapping_mul(999979));
                    show ^ bdims.wrapping_add(hdims)
                        ^ (app.toolpath_playback_position as u64)
                        ^ (app.active_profile_index as u64) << 48
                        ^ stl_h ^ (shape_h << 56) ^ (grid_z_h << 16) ^ nozzle_h
                })
                .unwrap_or(0);

            if profile_sig != self.machine_profile_sig {
                self.update_machine_geometry(app);
                self.machine_profile_sig = profile_sig;
            }

            // Collision overlay: rebuild when collision_segments changes
            let col_sig = app.collision_segments.len();
            if col_sig != self.collision_lines_sig {
                self.update_collision_lines(app);
                self.collision_lines_sig = col_sig;
            }
        }

        // Update layer contour lines if they changed or section changed
        if app.layers.len() != self.last_layer_count || section_content_changed {
            log::info!("Updating layer visualization ({} layers)", app.layers.len());
            self.create_layer_lines(app);
        }

        // Persist section/travel tracking state
        if section_content_changed {
            self.last_section_enabled_for_content = app.section_enabled;
            self.last_section_axis_for_content = app.section_axis;
            self.last_section_depth_for_content = app.section_depth;
        }
        if travel_changed {
            self.last_show_travel_moves = app.show_travel_moves;
        }

        // Update wireframe if mesh changed
        if let Some(mesh) = display_mesh {
            if app.show_wireframe && mesh.triangles.len() != self.last_wireframe_triangle_count {
                log::info!("Creating wireframe visualization ({} triangles)", mesh.triangles.len());
                self.create_wireframe_mesh(mesh);
            }
        }

        // Update section view mesh
        if let Some(mesh) = display_mesh {
            if app.section_enabled {
                let depth_changed = (app.section_depth - self.last_section_depth).abs() > 0.001;
                let axis_changed = app.section_axis != self.last_section_axis;
                let mesh_changed = mesh.triangles.len() != self.last_mesh_triangle_count;
                let just_enabled = !self.last_section_enabled;
                if depth_changed || axis_changed || mesh_changed || just_enabled {
                    self.create_section_mesh(mesh, app.section_axis, app.section_depth);
                }
                self.last_section_enabled = true;
            } else if self.last_section_enabled {
                self.section_mesh = None;
                self.last_section_enabled = false;
            }
        }

        // Allocate space for the 3D viewport
        let (rect, response) = ui.allocate_exact_size(
            available_size,
            egui::Sense::click_and_drag(),
        );

        // Check for keyboard input (H key for camera reset)
        if ui.input(|i| i.key_pressed(egui::Key::H)) {
            log::info!("Camera reset (H key pressed) - restoring default view");
            self.camera_yaw = self.default_camera_yaw;
            self.camera_pitch = self.default_camera_pitch;
            self.camera_distance = self.default_camera_distance;
            self.camera_target = self.default_camera_target;
        }

        // Handle input
        let clicked_face = self.handle_input(&response, app);

        // Render using three-d — always render if there is a mesh OR machine geometry to show.
        let has_machine = self.machine_head_gm.is_some() || self.machine_bed_gm.is_some();
        if self.mesh_object.is_some() || has_machine {
            self.render_3d(ui, rect, app);

            // Overlay "No mesh loaded" hint when machine is showing but no mesh yet
            if self.mesh_object.is_none() {
                let painter = ui.painter();
                painter.text(
                    egui::pos2(rect.center().x, rect.top() + 22.0),
                    egui::Align2::CENTER_TOP,
                    "No mesh loaded",
                    egui::FontId::proportional(13.0),
                    egui::Color32::from_rgba_unmultiplied(200, 200, 200, 120),
                );
            }

            // Show camera controls hint
            let painter = ui.painter();
            let hint_pos = egui::pos2(rect.left() + 10.0, rect.bottom() - 20.0);
            painter.text(
                hint_pos,
                egui::Align2::LEFT_BOTTOM,
                "H: Reset camera | Drag: Rotate | Shift+Drag: Pan | Scroll: Zoom",
                egui::FontId::proportional(11.0),
                egui::Color32::from_rgba_unmultiplied(200, 200, 200, 180),
            );
        } else {
            // No mesh and no machine geometry — show empty placeholder
            let painter = ui.painter();
            painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(15, 15, 20));
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No mesh loaded",
                egui::FontId::proportional(16.0),
                egui::Color32::GRAY,
            );
        }

        clicked_face
    }

    /// Rebuild machine head + bed geometry for the current playback position.
    fn update_machine_geometry(&mut self, app: &SlicerApp) {
        let profile = match app.profiles.get(app.active_profile_index) {
            Some(p) => p.clone(),
            None => {
                self.machine_head_gm = None;
                self.machine_bed_gm = None;
                return;
            }
        };

        if !profile.show_machine {
            self.machine_head_gm = None;
            self.machine_bed_gm = None;
            return;
        }

        // Find current segment's position and orientation
        let (seg_pos, seg_orient) = {
            let mut found = None;
            let mut idx = 0usize;
            'outer: for tp in &app.toolpaths {
                for seg in &tp.paths {
                    if idx == app.toolpath_playback_position {
                        found = Some((
                            [seg.position.x as f32, seg.position.y as f32, seg.position.z as f32],
                            seg.orientation,
                        ));
                        break 'outer;
                    }
                    idx += 1;
                }
            }
            found.unwrap_or_else(|| (
                [0.0f32, 0.0, 0.0],
                crate::geometry::Vector3D::new(0.0, 0.0, 1.0),
            ))
        };

        // Compute axis angles for this orientation
        let angles = app.compute_segment_axis_angles(&seg_orient);
        let gcode_axes = app.build_gcode_axes();

        let head_angles: Vec<f64> = gcode_axes.iter().zip(angles.iter())
            .filter(|(ax, _)| ax.is_head).take(2)
            .map(|(_, (_, d))| *d).collect();
        let bed_angles: Vec<f64> = gcode_axes.iter().zip(angles.iter())
            .filter(|(ax, _)| !ax.is_head).take(2)
            .map(|(_, (_, d))| *d).collect();

        let ha = head_angles.first().copied().unwrap_or(0.0).to_radians() as f32;
        let hb = head_angles.get(1).copied().unwrap_or(0.0).to_radians() as f32;
        let ba = bed_angles.first().copied().unwrap_or(0.0).to_radians() as f32;
        let bb = bed_angles.get(1).copied().unwrap_or(0.0).to_radians() as f32;

        let tcp = app.tcp_offset as f32;
        let olen = ((seg_orient.x * seg_orient.x + seg_orient.y * seg_orient.y + seg_orient.z * seg_orient.z) as f32).sqrt().max(1e-6);

        // Head pivot = nozzle_pos minus tcp along orientation
        let pivot_head = Vec3::new(
            seg_pos[0] - tcp * seg_orient.x as f32 / olen,
            seg_pos[1] - tcp * seg_orient.y as f32 / olen,
            seg_pos[2] - tcp * seg_orient.z as f32 / olen,
        );

        // ── Head ─────────────────────────────────────────────────────────
        let rh = machine_rot_xy(ha, hb);
        let (w, d, h) = (profile.head_dims[0] as f32, profile.head_dims[1] as f32, profile.head_dims[2] as f32);
        let nr = profile.nozzle_radius as f32;
        // nozzle_length drives the visual cylinder; tcp drives G-code compensation
        let nl = profile.nozzle_length as f32;
        let head_color   = Srgba::new(80, 140, 220, 200);
        let nozzle_color = Srgba::new(200, 200, 200, 220);
        let tip_color    = Srgba::new(255, 200, 60, 255); // bright gold tip marker

        // Nozzle tip in world space (the actual contact point)
        let seg_pos_vec3 = Vec3::new(seg_pos[0], seg_pos[1], seg_pos[2]);

        self.machine_head_gm = {
            // Try STL override first
            let cpu_opt: Option<CpuMesh> = profile.head_stl_path.as_ref().and_then(|path_str| {
                match crate::Mesh::from_stl(std::path::Path::new(path_str)) {
                    Ok(sm) => {
                        // The user-specified tip offset tells us which point in the STL is the
                        // nozzle tip.  We pin that local point to seg_pos_vec3 in world space:
                        //   world_origin = seg_pos - rh * tip_local
                        let tip_local = Vec3::new(
                            profile.head_stl_tip_offset[0] as f32,
                            profile.head_stl_tip_offset[1] as f32,
                            profile.head_stl_tip_offset[2] as f32,
                        );
                        let world_origin = seg_pos_vec3 - machine_apply_rot(rh, tip_local);
                        Some(machine_slicer_mesh_to_cpu_mesh(&sm, head_color, world_origin, rh))
                    }
                    Err(e) => { log::warn!("Head STL load failed ({}): {}", path_str, e); None }
                }
            });

            if let Some(cpu) = cpu_opt {
                // STL mesh loaded
                Some(Arc::new(Gm::new(Mesh::new(&self.context, &cpu), ColorMaterial::default())))
            } else {
                // Parametric box + nozzle cylinder + nozzle-tip sphere
                let mut pos: Vec<Vec3> = Vec::new();
                let mut nor: Vec<Vec3> = Vec::new();
                let mut col: Vec<Srgba> = Vec::new();
                let mut idx: Vec<u32> = Vec::new();

                // Head carriage box: local bottom at z=0 (pivot plane), extends up
                machine_append_box(&mut pos, &mut nor, &mut col, &mut idx,
                    w, d, h, head_color,
                    Vec3::new(0.0, 0.0, h / 2.0),
                    pivot_head, rh);

                // Nozzle cylinder hanging below pivot — uses nozzle_length, not tcp
                if nl > 0.5 {
                    machine_append_cylinder(&mut pos, &mut nor, &mut col, &mut idx,
                        nr, nl, 12, nozzle_color,
                        Vec3::new(0.0, 0.0, -nl),
                        pivot_head, rh);

                    // Bright tip sphere at the nozzle tip so it's easy to identify
                    let tip_r = (nr * 1.5).max(2.5);
                    machine_append_sphere(&mut pos, &mut nor, &mut col, &mut idx,
                        tip_r, 8, tip_color,
                        Vec3::new(0.0, 0.0, -nl),
                        pivot_head, rh);
                }

                if pos.is_empty() { None } else {
                    let cpu = CpuMesh {
                        positions: Positions::F32(pos),
                        normals: Some(nor), colors: Some(col), indices: Indices::U32(idx),
                        ..Default::default()
                    };
                    Some(Arc::new(Gm::new(Mesh::new(&self.context, &cpu), ColorMaterial::default())))
                }
            }
        };

        // ── Bed ──────────────────────────────────────────────────────────
        let rb = machine_rot_xy(ba, bb);
        let (bw, bd, bh) = (profile.bed_dims[0] as f32, profile.bed_dims[1] as f32, profile.bed_dims[2] as f32);

        // Snap bed's top face to just below the grid (mesh bottom Z), falling back to z=0
        let grid_z = self.mesh_bounds.map(|(mn, _)| mn[2]).unwrap_or(0.0f32);
        let bed_world_pivot = Vec3::new(
            profile.bed_pivot[0] as f32,
            profile.bed_pivot[1] as f32,
            grid_z - bh,  // top face of bed aligns with the build-plate grid
        );
        let bed_color = Srgba::new(180, 100, 60, 200);

        self.machine_bed_gm = {
            // Try STL override first
            let cpu_opt: Option<CpuMesh> = profile.bed_stl_path.as_ref().and_then(|path_str| {
                match crate::Mesh::from_stl(std::path::Path::new(path_str)) {
                    Ok(sm) => Some(machine_slicer_mesh_to_cpu_mesh(&sm, bed_color, bed_world_pivot, rb)),
                    Err(e) => { log::warn!("Bed STL load failed ({}): {}", path_str, e); None }
                }
            });

            if let Some(cpu) = cpu_opt {
                Some(Arc::new(Gm::new(Mesh::new(&self.context, &cpu), ColorMaterial::default())))
            } else {
                // Parametric bed shape
                let mut pos: Vec<Vec3> = Vec::new();
                let mut nor: Vec<Vec3> = Vec::new();
                let mut col: Vec<Srgba> = Vec::new();
                let mut idx: Vec<u32> = Vec::new();

                match profile.bed_shape {
                    BedShape::Rectangle => {
                        // Box with bottom at z=0, top at z=bh
                        machine_append_box(&mut pos, &mut nor, &mut col, &mut idx,
                            bw, bd, bh, bed_color,
                            Vec3::new(0.0, 0.0, bh / 2.0),
                            bed_world_pivot, rb);
                    }
                    BedShape::Circle => {
                        // Cylinder: radius = min(bw, bd)/2, along +Z
                        let radius = bw.min(bd) / 2.0;
                        machine_append_cylinder(&mut pos, &mut nor, &mut col, &mut idx,
                            radius, bh, 32, bed_color,
                            Vec3::new(0.0, 0.0, 0.0),
                            bed_world_pivot, rb);
                    }
                }

                if pos.is_empty() { None } else {
                    let cpu = CpuMesh {
                        positions: Positions::F32(pos),
                        normals: Some(nor), colors: Some(col), indices: Indices::U32(idx),
                        ..Default::default()
                    };
                    Some(Arc::new(Gm::new(Mesh::new(&self.context, &cpu), ColorMaterial::default())))
                }
            }
        };
    }

    /// Rebuild the red collision overlay tubes from `app.collision_segments`.
    fn update_collision_lines(&mut self, app: &SlicerApp) {
        if app.collision_segments.is_empty() || !app.collision_segments.iter().any(|&c| c) {
            self.collision_lines_gm = None;
            return;
        }

        let mut positions: Vec<Vec3> = Vec::new();
        let mut normals: Vec<Vec3> = Vec::new();
        let mut colors: Vec<Srgba> = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        let red = Srgba::new(255, 30, 30, 255);
        let tube_segments = 4u32;

        let mut flat_idx = 0usize;
        for toolpath in &app.toolpaths {
            for window in toolpath.paths.windows(2) {
                let is_collision = app.collision_segments.get(flat_idx).copied().unwrap_or(false);
                flat_idx += 1;
                if !is_collision { continue; }

                let p1 = vec3(window[0].position.x as f32, window[0].position.y as f32, window[0].position.z as f32);
                let p2 = vec3(window[1].position.x as f32, window[1].position.y as f32, window[1].position.z as f32);
                let r = (toolpath.layer_height as f32 / 2.0).clamp(0.08, 0.35) * 1.5;
                create_tube_segment(p1, p2, r, tube_segments, red,
                    &mut positions, &mut normals, &mut colors, &mut indices);
            }
        }

        self.collision_lines_gm = if positions.is_empty() {
            None
        } else {
            let cpu = CpuMesh {
                positions: Positions::F32(positions),
                normals: Some(normals),
                colors: Some(colors),
                indices: Indices::U32(indices),
                ..Default::default()
            };
            Some(Arc::new(Gm::new(Mesh::new(&self.context, &cpu), ColorMaterial::default())))
        };
    }

    fn render_3d(&mut self, ui: &mut egui::Ui, rect: egui::Rect, app: &SlicerApp) {
        // Render directly to screen framebuffer using paint callback.
        // Works both with and without a loaded mesh (machine geometry is visible even with no model).
        {
            static mut RENDER_COUNT: u32 = 0;
            unsafe {
                RENDER_COUNT += 1;
                if RENDER_COUNT % 60 == 0 {
                    log::info!("VIEWPORT: render_3d() executing (frame {})", RENDER_COUNT);
                }
            }
            let context = self.context.clone();
            // Use section mesh when section view is active, otherwise full mesh if available
            let mesh_arc = if app.show_mesh {
                if app.section_enabled {
                    self.section_mesh.clone()
                } else {
                    self.mesh_object.clone()
                }
            } else {
                None
            };
            let build_plate_arc = self.build_plate_mesh.clone();
            // Only include toolpath objects if show_toolpaths is enabled
            let toolpath_spheres_arc = if app.show_toolpaths { self.toolpath_spheres.clone() } else { None };
            let toolpath_lines_arc = if app.show_toolpaths { self.toolpath_lines.clone() } else { None };
            let layer_lines_arc = if app.show_layers { self.layer_lines.clone() } else { None };
            let wireframe_arc = if app.show_wireframe { self.wireframe_mesh.clone() } else { None };
            let axes_arc = self.axes_mesh.clone();
            let show_machine = app.profiles.get(app.active_profile_index)
                .map(|p| p.show_machine)
                .unwrap_or(false);
            let machine_head_arc = if show_machine { self.machine_head_gm.clone() } else { None };
            let machine_bed_arc = if show_machine { self.machine_bed_gm.clone() } else { None };
            let collision_lines_arc = self.collision_lines_gm.clone();
            let camera_yaw = self.camera_yaw;
            let camera_pitch = self.camera_pitch;
            let camera_distance = self.camera_distance;
            let camera_target = self.camera_target;

            let callback = egui::PaintCallback {
                rect,
                callback: Arc::new(eframe::egui_glow::CallbackFn::new(move |_info, painter| {
                    use eframe::glow::HasContext as _;
                    let gl = painter.gl();

                    unsafe {
                        // Save current OpenGL state
                        let prev_depth_test = gl.is_enabled(eframe::glow::DEPTH_TEST);
                        let prev_cull_face = gl.is_enabled(eframe::glow::CULL_FACE);

                        // Enable depth test
                        gl.enable(eframe::glow::DEPTH_TEST);
                        gl.depth_func(eframe::glow::LEQUAL);

                        // DISABLE backface culling - STL files often have inconsistent winding order
                        // Many STL exporters don't enforce consistent normals/winding, causing 30-70%
                        // of faces to disappear with culling enabled. This is the #1 cause of
                        // "incomplete mesh" rendering issues.
                        gl.disable(eframe::glow::CULL_FACE);

                        // Set viewport to callback rectangle
                        let screen_rect = _info.viewport_in_pixels();
                        gl.viewport(
                            screen_rect.left_px as i32,
                            screen_rect.from_bottom_px as i32,
                            screen_rect.width_px as i32,
                            screen_rect.height_px as i32,
                        );

                        // Clear depth buffer
                        gl.clear(eframe::glow::DEPTH_BUFFER_BIT);

                        // Calculate camera position
                        let yaw_rad = camera_yaw.to_radians();
                        let pitch_rad = camera_pitch.to_radians();

                        let camera_x = camera_target[0] + camera_distance * pitch_rad.cos() * yaw_rad.sin();
                        let camera_y = camera_target[1] + camera_distance * pitch_rad.sin();
                        let camera_z = camera_target[2] + camera_distance * pitch_rad.cos() * yaw_rad.cos();

                        // Create three-d viewport
                        let viewport = Viewport {
                            x: screen_rect.left_px as i32,
                            y: screen_rect.from_bottom_px as i32,
                            width: screen_rect.width_px as u32,
                            height: screen_rect.height_px as u32,
                        };

                        // Create camera with optimized depth precision
                        let camera = Camera::new_perspective(
                            viewport,
                            vec3(camera_x, camera_y, camera_z),
                            vec3(camera_target[0], camera_target[1], camera_target[2]),
                            vec3(0.0, 1.0, 0.0),
                            degrees(45.0),
                            5.0,      // Near plane: Increased from 0.1 to 5.0 for better depth precision
                            10000.0,  // Far plane: Sufficient for 300mm build volume
                        );

                        // Dramatic lighting for flat reliefs (like coins/medals/seals)
                        // Low ambient + strong raking lights reveal embossed details
                        let ambient = AmbientLight::new(&context, 0.2, Srgba::new(180, 180, 200, 255));

                        // Strong raking light from low angle (grazes surface to show relief)
                        let key_light = DirectionalLight::new(
                            &context,
                            2.0,  // Very bright!
                            Srgba::new(255, 250, 240, 255), // Warm white
                            vec3(1.0, 0.2, 0.1),  // Almost horizontal (raking angle)
                        );

                        // Opposite raking light to show the other side
                        let fill_light = DirectionalLight::new(
                            &context,
                            1.2,
                            Srgba::new(200, 220, 255, 255), // Cool blue-white
                            vec3(-0.8, 0.2, -0.1),  // Almost horizontal, opposite side
                        );

                        // Top-down light to provide overall illumination
                        let top_light = DirectionalLight::new(
                            &context,
                            0.6,
                            Srgba::new(255, 255, 255, 255),
                            vec3(0.0, 1.0, 0.5),
                        );

                        // Collect objects to render
                        let mut objects: Vec<&dyn three_d::Object> = Vec::new();

                        // Add mesh if show_mesh is enabled
                        let mesh_ref;
                        if let Some(ref m) = mesh_arc {
                            mesh_ref = m.clone();
                            objects.push(&*mesh_ref);
                        }

                        // Add build plate if available
                        let build_plate_ref;
                        if let Some(ref plate) = build_plate_arc {
                            build_plate_ref = plate.clone();
                            objects.push(&*build_plate_ref);
                        }

                        // Add toolpath spheres if available
                        let toolpath_spheres_ref;
                        if let Some(ref tp_spheres) = toolpath_spheres_arc {
                            toolpath_spheres_ref = tp_spheres.clone();
                            objects.push(&*toolpath_spheres_ref);
                        }

                        // Add toolpath lines if available
                        let toolpath_lines_ref;
                        if let Some(ref tp_lines) = toolpath_lines_arc {
                            toolpath_lines_ref = tp_lines.clone();
                            objects.push(&*toolpath_lines_ref);
                        }

                        // Add layer contour lines if available
                        let layer_lines_ref;
                        if let Some(ref ll) = layer_lines_arc {
                            layer_lines_ref = ll.clone();
                            objects.push(&*layer_lines_ref);
                        }

                        // Add wireframe overlay if available
                        let wireframe_ref;
                        if let Some(ref wf) = wireframe_arc {
                            wireframe_ref = wf.clone();
                            objects.push(&*wireframe_ref);
                        }

                        // Add coordinate axes
                        let axes_ref;
                        if let Some(ref axes) = axes_arc {
                            axes_ref = axes.clone();
                            objects.push(&*axes_ref);
                        }

                        // Add machine simulation geometry (bed and head)
                        let machine_head_ref;
                        if let Some(ref mh) = machine_head_arc {
                            machine_head_ref = mh.clone();
                            objects.push(&*machine_head_ref);
                        }
                        let machine_bed_ref;
                        if let Some(ref mb) = machine_bed_arc {
                            machine_bed_ref = mb.clone();
                            objects.push(&*machine_bed_ref);
                        }

                        // Add collision overlay (red segments on top of normal toolpaths)
                        let collision_lines_ref;
                        if let Some(ref cl) = collision_lines_arc {
                            collision_lines_ref = cl.clone();
                            objects.push(&*collision_lines_ref);
                        }

                        // CRITICAL: Set three-d context render states to disable culling globally
                        // This ensures ALL objects render with culling disabled
                        context.set_render_states(RenderStates {
                            cull: Cull::None,
                            depth_test: DepthTest::Less,
                            write_mask: WriteMask::COLOR_AND_DEPTH,
                            ..Default::default()
                        });

                        // Also force disable at GL level (belt and suspenders approach)
                        gl.disable(eframe::glow::CULL_FACE);

                        // Render with all lights (including top light for flat reliefs)
                        RenderTarget::screen(&context, viewport.width, viewport.height)
                            .render(&camera, objects.as_slice(), &[&key_light, &fill_light, &top_light, &ambient]);

                        // Restore OpenGL state
                        if !prev_depth_test {
                            gl.disable(eframe::glow::DEPTH_TEST);
                        }
                        if prev_cull_face {
                            gl.enable(eframe::glow::CULL_FACE);
                        }
                    }
                })),
            };

            ui.painter().add(callback);
        }

        let painter = ui.painter();

        // Build plate is now rendered as 3D mesh in the render callback above
        // Toolpaths are now rendered as 3D lines in the render callback above (no more 2D projection)

        // Draw debug info overlay using egui
        if let Some((bounds_min, bounds_max)) = self.mesh_bounds {
            let size = [
                bounds_max[0] - bounds_min[0],
                bounds_max[1] - bounds_min[1],
                bounds_max[2] - bounds_min[2],
            ];

            painter.text(
                rect.left_top() + egui::vec2(10.0, 10.0),
                egui::Align2::LEFT_TOP,
                format!("Camera: Yaw={:.1}° Pitch={:.1}° Dist={:.1}",
                    self.camera_yaw, self.camera_pitch, self.camera_distance),
                egui::FontId::monospace(12.0),
                egui::Color32::WHITE,
            );

            if let Some(mesh) = &app.mesh {
                painter.text(
                    rect.left_top() + egui::vec2(10.0, 30.0),
                    egui::Align2::LEFT_TOP,
                    format!("Triangles: {}", mesh.triangles.len()),
                    egui::FontId::monospace(12.0),
                    egui::Color32::WHITE,
                );
            }

            painter.text(
                rect.left_top() + egui::vec2(10.0, 45.0),
                egui::Align2::LEFT_TOP,
                format!("Size: {:.1}x{:.1}x{:.1} mm", size[0], size[1], size[2]),
                egui::FontId::monospace(12.0),
                egui::Color32::WHITE,
            );

            painter.text(
                rect.left_top() + egui::vec2(10.0, 60.0),
                egui::Align2::LEFT_TOP,
                format!("Center: ({:.1}, {:.1}, {:.1})",
                    self.camera_target[0], self.camera_target[1], self.camera_target[2]),
                egui::FontId::monospace(12.0),
                egui::Color32::WHITE,
            );
        }

        // Show controls help
        painter.text(
            rect.left_bottom() + egui::vec2(10.0, -10.0),
            egui::Align2::LEFT_BOTTOM,
            "Right-drag: Rotate | Middle-drag: Pan | Scroll: Zoom",
            egui::FontId::monospace(11.0),
            egui::Color32::from_rgb(150, 150, 150),
        );
    }

    fn handle_input(&mut self, response: &egui::Response, app: &SlicerApp) -> Option<usize> {
        let mut clicked_face: Option<usize> = None;

        // Handle left click for face selection (if in face orientation mode)
        if response.clicked() {
            log::info!("Viewport clicked! Face orientation mode: {}", app.face_orientation_mode);

            if app.face_orientation_mode {
                if let Some(mesh) = &app.mesh {
                    if let Some((bounds_min, bounds_max)) = self.mesh_bounds {
                        if let Some(click_pos) = response.interact_pointer_pos() {
                            log::info!("Click position: {:?}, Rect: {:?}", click_pos, response.rect);
                            clicked_face = self.find_clicked_face(click_pos, mesh, bounds_min, bounds_max, response.rect);

                            if let Some(face_idx) = clicked_face {
                                log::info!("Found clicked face: {}", face_idx);
                            } else {
                                log::info!("No face found at click position");
                            }
                        } else {
                            log::info!("No interact_pointer_pos available");
                        }
                    } else {
                        log::info!("No mesh bounds available");
                    }
                } else {
                    log::info!("No mesh available");
                }
            }
        }

        // Mouse buttons
        if response.drag_started_by(egui::PointerButton::Secondary) {
            self.is_rotating = true;
        }
        if response.drag_started_by(egui::PointerButton::Middle) {
            self.is_panning = true;
        }

        if response.drag_stopped() {
            self.is_rotating = false;
            self.is_panning = false;
        }

        // Mouse delta
        if let Some(pos) = response.interact_pointer_pos() {
            if let Some(last_pos) = self.last_mouse_pos {
                let delta = pos - last_pos;

                if self.is_rotating {
                    // Rotate camera
                    self.camera_yaw += delta.x * 0.5;
                    self.camera_pitch = (self.camera_pitch - delta.y * 0.5).clamp(-89.0, 89.0);
                } else if self.is_panning {
                    // Pan camera
                    let pan_speed = self.camera_distance * 0.001;
                    let yaw_rad = self.camera_yaw.to_radians();

                    // Move camera target
                    self.camera_target[0] -= delta.x * pan_speed * yaw_rad.cos();
                    self.camera_target[2] -= delta.x * pan_speed * yaw_rad.sin();
                    self.camera_target[1] += delta.y * pan_speed;
                }
            }
            self.last_mouse_pos = Some(pos);
        } else {
            self.last_mouse_pos = None;
        }

        // Mouse wheel for zoom
        let scroll = response.ctx.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.1 {
            self.camera_distance = (self.camera_distance - scroll * 0.1).max(10.0).min(1000.0);
        }

        clicked_face
    }

    /// Find which face was clicked using ray-triangle intersection
    fn find_clicked_face(
        &self,
        click_pos: egui::Pos2,
        mesh: &SlicerMesh,
        bounds_min: [f32; 3],
        bounds_max: [f32; 3],
        rect: egui::Rect,
    ) -> Option<usize> {
        // Calculate camera position
        let yaw_rad = self.camera_yaw.to_radians();
        let pitch_rad = self.camera_pitch.to_radians();

        let camera_x = self.camera_target[0] + self.camera_distance * pitch_rad.cos() * yaw_rad.sin();
        let camera_y = self.camera_target[1] + self.camera_distance * pitch_rad.sin();
        let camera_z = self.camera_target[2] + self.camera_distance * pitch_rad.cos() * yaw_rad.cos();

        let camera_pos = [camera_x, camera_y, camera_z];

        log::info!("  Camera position: ({:.2}, {:.2}, {:.2})", camera_x, camera_y, camera_z);
        log::info!("  Camera target: ({:.2}, {:.2}, {:.2})", self.camera_target[0], self.camera_target[1], self.camera_target[2]);

        // Convert click position to normalized device coordinates
        let ndc_x = (click_pos.x - rect.center().x) / (rect.width() * 0.5);
        let ndc_y = -(click_pos.y - rect.center().y) / (rect.height() * 0.5);

        log::info!("  NDC coordinates: ({:.4}, {:.4})", ndc_x, ndc_y);

        // Create ray direction (simplified - assumes 45° FOV)
        let aspect = rect.width() / rect.height();
        let fov_factor = (45.0f32.to_radians() / 2.0).tan();

        // Ray in view space
        let ray_view_x = ndc_x * aspect * fov_factor;
        let ray_view_y = ndc_y * fov_factor;
        let ray_view_z = 1.0; // Positive Z points forward in view space

        // Transform ray to world space
        let forward = [
            (self.camera_target[0] - camera_x),
            (self.camera_target[1] - camera_y),
            (self.camera_target[2] - camera_z),
        ];
        let forward_len = (forward[0] * forward[0] + forward[1] * forward[1] + forward[2] * forward[2]).sqrt();
        let forward_norm = [forward[0] / forward_len, forward[1] / forward_len, forward[2] / forward_len];

        let right = [
            -forward_norm[2],
            0.0,
            forward_norm[0],
        ];
        let right_len = (right[0] * right[0] + right[2] * right[2]).sqrt();
        let right_norm = if right_len > 0.001 {
            [right[0] / right_len, 0.0, right[2] / right_len]
        } else {
            [1.0, 0.0, 0.0]
        };

        let up = [
            forward_norm[1] * right_norm[2] - forward_norm[2] * right_norm[1],
            forward_norm[2] * right_norm[0] - forward_norm[0] * right_norm[2],
            forward_norm[0] * right_norm[1] - forward_norm[1] * right_norm[0],
        ];

        let ray_direction = [
            right_norm[0] * ray_view_x + up[0] * ray_view_y + forward_norm[0] * ray_view_z,
            right_norm[1] * ray_view_x + up[1] * ray_view_y + forward_norm[1] * ray_view_z,
            right_norm[2] * ray_view_x + up[2] * ray_view_y + forward_norm[2] * ray_view_z,
        ];

        // Normalize ray direction
        let ray_len = (ray_direction[0] * ray_direction[0] +
                       ray_direction[1] * ray_direction[1] +
                       ray_direction[2] * ray_direction[2]).sqrt();
        let ray_dir_norm = [
            ray_direction[0] / ray_len,
            ray_direction[1] / ray_len,
            ray_direction[2] / ray_len,
        ];

        log::info!("  Ray direction: ({:.4}, {:.4}, {:.4})", ray_dir_norm[0], ray_dir_norm[1], ray_dir_norm[2]);

        // Test all triangles for intersection
        let mut closest_distance = f32::MAX;
        let mut closest_face = None;
        let mut tested_count = 0;
        let mut hit_count = 0;

        for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
            tested_count += 1;
            let v0 = [triangle.v0.x as f32, triangle.v0.y as f32, triangle.v0.z as f32];
            let v1 = [triangle.v1.x as f32, triangle.v1.y as f32, triangle.v1.z as f32];
            let v2 = [triangle.v2.x as f32, triangle.v2.y as f32, triangle.v2.z as f32];

            // Log first triangle for debugging
            if tri_idx == 0 {
                log::info!("  First triangle v0: ({:.2}, {:.2}, {:.2})", v0[0], v0[1], v0[2]);
            }

            if let Some(distance) = Self::ray_triangle_intersection(camera_pos, ray_dir_norm, v0, v1, v2) {
                hit_count += 1;
                if distance < closest_distance {
                    closest_distance = distance;
                    closest_face = Some(tri_idx);
                }
            }
        }

        log::info!("  Tested {} triangles, {} hits, closest_distance: {:.2}",
            tested_count, hit_count, if hit_count > 0 { closest_distance } else { -1.0 });

        closest_face
    }

    /// Ray-triangle intersection using Möller–Trumbore algorithm
    fn ray_triangle_intersection(
        ray_origin: [f32; 3],
        ray_direction: [f32; 3],
        v0: [f32; 3],
        v1: [f32; 3],
        v2: [f32; 3],
    ) -> Option<f32> {
        const EPSILON: f32 = 1e-8;

        let edge1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
        let edge2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

        let h = [
            ray_direction[1] * edge2[2] - ray_direction[2] * edge2[1],
            ray_direction[2] * edge2[0] - ray_direction[0] * edge2[2],
            ray_direction[0] * edge2[1] - ray_direction[1] * edge2[0],
        ];

        let a = edge1[0] * h[0] + edge1[1] * h[1] + edge1[2] * h[2];

        if a.abs() < EPSILON {
            return None;
        }

        let f = 1.0 / a;
        let s = [
            ray_origin[0] - v0[0],
            ray_origin[1] - v0[1],
            ray_origin[2] - v0[2],
        ];

        let u = f * (s[0] * h[0] + s[1] * h[1] + s[2] * h[2]);
        if u < 0.0 || u > 1.0 {
            return None;
        }

        let q = [
            s[1] * edge1[2] - s[2] * edge1[1],
            s[2] * edge1[0] - s[0] * edge1[2],
            s[0] * edge1[1] - s[1] * edge1[0],
        ];

        let v = f * (ray_direction[0] * q[0] + ray_direction[1] * q[1] + ray_direction[2] * q[2]);
        if v < 0.0 || u + v > 1.0 {
            return None;
        }

        let t = f * (edge2[0] * q[0] + edge2[1] * q[1] + edge2[2] * q[2]);

        if t > EPSILON {
            Some(t)
        } else {
            None
        }
    }

    /// Render toolpaths by projecting them to 2D screen space
    fn render_toolpaths(&self, rect: egui::Rect, app: &SlicerApp, painter: &egui::Painter) {
        // Calculate camera matrices
        let yaw_rad = self.camera_yaw.to_radians();
        let pitch_rad = self.camera_pitch.to_radians();

        let camera_x = self.camera_target[0] + self.camera_distance * pitch_rad.cos() * yaw_rad.sin();
        let camera_y = self.camera_target[1] + self.camera_distance * pitch_rad.sin();
        let camera_z = self.camera_target[2] + self.camera_distance * pitch_rad.cos() * yaw_rad.cos();

        // Camera vectors
        let forward = [
            self.camera_target[0] - camera_x,
            self.camera_target[1] - camera_y,
            self.camera_target[2] - camera_z,
        ];
        let forward_len = (forward[0] * forward[0] + forward[1] * forward[1] + forward[2] * forward[2]).sqrt();
        let forward_norm = [forward[0] / forward_len, forward[1] / forward_len, forward[2] / forward_len];

        let right = [
            -forward_norm[2],
            0.0,
            forward_norm[0],
        ];
        let right_len = (right[0] * right[0] + right[2] * right[2]).sqrt();
        let right_norm = if right_len > 0.001 {
            [right[0] / right_len, 0.0, right[2] / right_len]
        } else {
            [1.0, 0.0, 0.0]
        };

        let up = [
            forward_norm[1] * right_norm[2] - forward_norm[2] * right_norm[1],
            forward_norm[2] * right_norm[0] - forward_norm[0] * right_norm[2],
            forward_norm[0] * right_norm[1] - forward_norm[1] * right_norm[0],
        ];

        let fov_factor = (45.0f32.to_radians() / 2.0).tan();
        let aspect = rect.width() / rect.height();

        // Project 3D point to 2D screen space
        let project = |pos: &crate::geometry::Point3D| -> Option<egui::Pos2> {
            // Vector from camera to point
            let dx = pos.x as f32 - camera_x;
            let dy = pos.y as f32 - camera_y;
            let dz = pos.z as f32 - camera_z;

            // Project onto camera space
            let view_x = dx * right_norm[0] + dy * right_norm[1] + dz * right_norm[2];
            let view_y = dx * up[0] + dy * up[1] + dz * up[2];
            let view_z = dx * forward_norm[0] + dy * forward_norm[1] + dz * forward_norm[2];

            // Check if behind camera
            if view_z <= 0.0 {
                return None;
            }

            // Perspective projection
            let ndc_x = (view_x / view_z) / (aspect * fov_factor);
            let ndc_y = -(view_y / view_z) / fov_factor;

            // Check if outside view frustum
            if ndc_x.abs() > 1.2 || ndc_y.abs() > 1.2 {
                return None;
            }

            // Convert NDC to screen space
            let screen_x = rect.center().x + ndc_x * rect.width() * 0.5;
            let screen_y = rect.center().y + ndc_y * rect.height() * 0.5;

            Some(egui::pos2(screen_x, screen_y))
        };

        // Render toolpaths
        for (layer_idx, toolpath) in app.toolpaths.iter().enumerate() {
            // Color based on layer height
            let t = layer_idx as f32 / app.toolpaths.len().max(1) as f32;
            let color = egui::Color32::from_rgb(
                (255.0 * (1.0 - t * 0.5)) as u8,
                (100.0 + 155.0 * t) as u8,
                255,
            );

            // Draw lines between consecutive points
            for window in toolpath.paths.windows(2) {
                if let (Some(p0), Some(p1)) = (project(&window[0].position), project(&window[1].position)) {
                    painter.line_segment([p0, p1], egui::Stroke::new(2.0, color));
                }
            }
        }
    }
}

/// Create tube geometry for a single line segment (proper 3D instead of degenerate triangles)
fn create_tube_segment(
    p1: Vec3,
    p2: Vec3,
    radius: f32,
    segments: u32,
    color: Srgba,
    positions: &mut Vec<Vec3>,
    normals: &mut Vec<Vec3>,
    colors: &mut Vec<Srgba>,
    indices: &mut Vec<u32>,
) {
    let direction = p2 - p1;
    let length = direction.magnitude();

    // Skip zero-length segments
    if length < 0.001 {
        return;
    }

    let dir_normalized = direction / length;

    // Find perpendicular vectors for the tube cross-section
    let up = if dir_normalized.y.abs() < 0.9 {
        vec3(0.0, 1.0, 0.0)
    } else {
        vec3(1.0, 0.0, 0.0)
    };

    let perp1 = dir_normalized.cross(up).normalize();
    let perp2 = dir_normalized.cross(perp1).normalize();

    let base_idx = positions.len() as u32;

    // Create vertices for tube (ring at p1 and ring at p2)
    for ring in 0..2 {
        let center = if ring == 0 { p1 } else { p2 };

        for i in 0..segments {
            let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            let offset = perp1 * cos_a * radius + perp2 * sin_a * radius;
            let position = center + offset;
            let normal = (perp1 * cos_a + perp2 * sin_a).normalize();

            positions.push(position);
            normals.push(normal);
            colors.push(color);
        }
    }

    // Create indices for tube faces (quads made of triangles)
    for i in 0..segments {
        let i_next = (i + 1) % segments;

        // Ring 0 vertex indices
        let v0 = base_idx + i;
        let v1 = base_idx + i_next;

        // Ring 1 vertex indices
        let v2 = base_idx + segments + i;
        let v3 = base_idx + segments + i_next;

        // Two triangles per quad
        indices.push(v0);
        indices.push(v2);
        indices.push(v1);

        indices.push(v1);
        indices.push(v2);
        indices.push(v3);
    }
}

// ─── Machine simulation geometry helpers ─────────────────────────────────────

/// Convert a `SlicerMesh` (from `crate::Mesh::from_stl`) into a `CpuMesh`
/// with the given rotation + translation baked into each vertex position.
fn machine_slicer_mesh_to_cpu_mesh(
    slicer_mesh: &crate::Mesh,
    color: Srgba,
    world_pivot: Vec3,
    rot: [[f32; 3]; 3],
) -> CpuMesh {
    let mut positions: Vec<Vec3> = Vec::new();
    let mut normals:   Vec<Vec3> = Vec::new();
    let mut colors:    Vec<Srgba> = Vec::new();
    let mut indices:   Vec<u32>  = Vec::new();

    for tri in &slicer_mesh.triangles {
        let lv0 = Vec3::new(tri.v0.x as f32, tri.v0.y as f32, tri.v0.z as f32);
        let lv1 = Vec3::new(tri.v1.x as f32, tri.v1.y as f32, tri.v1.z as f32);
        let lv2 = Vec3::new(tri.v2.x as f32, tri.v2.y as f32, tri.v2.z as f32);

        let e1 = lv1 - lv0;
        let e2 = lv2 - lv0;
        let ln = {
            let c = e1.cross(e2);
            let len = (c.x*c.x + c.y*c.y + c.z*c.z).sqrt();
            if len > 1e-9 { c / len } else { Vec3::new(0.0, 1.0, 0.0) }
        };
        let wn = machine_apply_rot(rot, ln).normalize();

        let base = positions.len() as u32;
        for lv in &[lv0, lv1, lv2] {
            positions.push(machine_apply_rot(rot, *lv) + world_pivot);
            normals.push(wn);
            colors.push(color);
        }
        indices.extend_from_slice(&[base, base + 1, base + 2]);
    }

    CpuMesh {
        positions: Positions::F32(positions),
        normals: Some(normals),
        colors: Some(colors),
        indices: Indices::U32(indices),
        ..Default::default()
    }
}

/// Build rotation matrix Ry(b) * Rx(a) for machine simulation.
fn machine_rot_xy(a: f32, b: f32) -> [[f32; 3]; 3] {
    let (sa, ca) = (a.sin(), a.cos());
    let (sb, cb) = (b.sin(), b.cos());
    [
        [cb,   sb * sa,  sb * ca],
        [0.0,  ca,      -sa     ],
        [-sb,  cb * sa,  cb * ca],
    ]
}

/// Apply a 3×3 rotation matrix to a Vec3.
fn machine_apply_rot(r: [[f32; 3]; 3], v: Vec3) -> Vec3 {
    Vec3::new(
        r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z,
        r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
        r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z,
    )
}

/// Append a flat-shaded axis-aligned box (in local space) to the geometry buffers.
///
/// `local_center` — centre of the box before rotation (in local frame)
/// `world_pivot`  — world-space translation applied after rotation
/// `rot`          — rotation matrix applied before translation
fn machine_append_box(
    positions: &mut Vec<Vec3>,
    normals:   &mut Vec<Vec3>,
    colors:    &mut Vec<Srgba>,
    indices:   &mut Vec<u32>,
    w: f32, d: f32, h: f32,
    color: Srgba,
    local_center: Vec3,
    world_pivot: Vec3,
    rot: [[f32; 3]; 3],
) {
    let hw = w / 2.0;
    let hd = d / 2.0;
    let hh = h / 2.0;
    let cx = local_center.x;
    let cy = local_center.y;
    let cz = local_center.z;

    // Each face: (local_normal, [4 corner positions in local space])
    let face_data: [([f32; 3], [[f32; 3]; 4]); 6] = [
        ([1.0, 0.0, 0.0], [[cx+hw, cy-hd, cz-hh], [cx+hw, cy+hd, cz-hh], [cx+hw, cy+hd, cz+hh], [cx+hw, cy-hd, cz+hh]]),
        ([-1.0, 0.0, 0.0], [[cx-hw, cy+hd, cz-hh], [cx-hw, cy-hd, cz-hh], [cx-hw, cy-hd, cz+hh], [cx-hw, cy+hd, cz+hh]]),
        ([0.0, 1.0, 0.0], [[cx+hw, cy+hd, cz-hh], [cx-hw, cy+hd, cz-hh], [cx-hw, cy+hd, cz+hh], [cx+hw, cy+hd, cz+hh]]),
        ([0.0, -1.0, 0.0], [[cx-hw, cy-hd, cz-hh], [cx+hw, cy-hd, cz-hh], [cx+hw, cy-hd, cz+hh], [cx-hw, cy-hd, cz+hh]]),
        ([0.0, 0.0, 1.0], [[cx-hw, cy-hd, cz+hh], [cx+hw, cy-hd, cz+hh], [cx+hw, cy+hd, cz+hh], [cx-hw, cy+hd, cz+hh]]),
        ([0.0, 0.0, -1.0], [[cx+hw, cy-hd, cz-hh], [cx-hw, cy-hd, cz-hh], [cx-hw, cy+hd, cz-hh], [cx+hw, cy+hd, cz-hh]]),
    ];

    for (fn_local, verts) in &face_data {
        let local_n = Vec3::new(fn_local[0], fn_local[1], fn_local[2]);
        let world_n = machine_apply_rot(rot, local_n).normalize();
        let base = positions.len() as u32;
        for v in verts {
            let local_p = Vec3::new(v[0], v[1], v[2]);
            positions.push(machine_apply_rot(rot, local_p) + world_pivot);
            normals.push(world_n);
            colors.push(color);
        }
        indices.extend_from_slice(&[base, base+1, base+2, base, base+2, base+3]);
    }
}

/// Append a flat-shaded cylinder along the local +Z axis to the geometry buffers.
///
/// `local_bottom` — local-space position of the cylinder's bottom centre
/// `height`       — length of the cylinder along local Z
/// `n`            — number of side segments
fn machine_append_cylinder(
    positions: &mut Vec<Vec3>,
    normals:   &mut Vec<Vec3>,
    colors:    &mut Vec<Srgba>,
    indices:   &mut Vec<u32>,
    radius: f32,
    height: f32,
    n: u32,
    color: Srgba,
    local_bottom: Vec3,
    world_pivot: Vec3,
    rot: [[f32; 3]; 3],
) {
    let cx = local_bottom.x;
    let cy = local_bottom.y;
    let bot_z = local_bottom.z;
    let top_z = local_bottom.z + height;

    // Side quads
    for i in 0..n {
        let a0 = (i as f32 / n as f32) * std::f32::consts::TAU;
        let a1 = ((i + 1) as f32 / n as f32) * std::f32::consts::TAU;
        let (s0, c0) = (a0.sin(), a0.cos());
        let (s1, c1) = (a1.sin(), a1.cos());

        let lverts = [
            Vec3::new(cx + c0 * radius, cy + s0 * radius, bot_z),
            Vec3::new(cx + c1 * radius, cy + s1 * radius, bot_z),
            Vec3::new(cx + c1 * radius, cy + s1 * radius, top_z),
            Vec3::new(cx + c0 * radius, cy + s0 * radius, top_z),
        ];
        let ln = Vec3::new((c0 + c1) * 0.5, (s0 + s1) * 0.5, 0.0).normalize();
        let wn = machine_apply_rot(rot, ln).normalize();

        let vi = positions.len() as u32;
        for lv in &lverts {
            positions.push(machine_apply_rot(rot, *lv) + world_pivot);
            normals.push(wn);
            colors.push(color);
        }
        indices.extend_from_slice(&[vi, vi+1, vi+2, vi, vi+2, vi+3]);
    }

    // Bottom cap
    let bot_n = machine_apply_rot(rot, Vec3::new(0.0, 0.0, -1.0)).normalize();
    let bc_idx = positions.len() as u32;
    positions.push(machine_apply_rot(rot, Vec3::new(cx, cy, bot_z)) + world_pivot);
    normals.push(bot_n); colors.push(color);
    for i in 0..n {
        let a = (i as f32 / n as f32) * std::f32::consts::TAU;
        positions.push(machine_apply_rot(rot, Vec3::new(cx + a.cos() * radius, cy + a.sin() * radius, bot_z)) + world_pivot);
        normals.push(bot_n); colors.push(color);
    }
    for i in 0..n {
        let i0 = bc_idx + 1 + i;
        let i1 = bc_idx + 1 + (i + 1) % n;
        indices.extend_from_slice(&[bc_idx, i1, i0]);
    }

    // Top cap
    let top_n = machine_apply_rot(rot, Vec3::new(0.0, 0.0, 1.0)).normalize();
    let tc_idx = positions.len() as u32;
    positions.push(machine_apply_rot(rot, Vec3::new(cx, cy, top_z)) + world_pivot);
    normals.push(top_n); colors.push(color);
    for i in 0..n {
        let a = (i as f32 / n as f32) * std::f32::consts::TAU;
        positions.push(machine_apply_rot(rot, Vec3::new(cx + a.cos() * radius, cy + a.sin() * radius, top_z)) + world_pivot);
        normals.push(top_n); colors.push(color);
    }
    for i in 0..n {
        let i0 = tc_idx + 1 + i;
        let i1 = tc_idx + 1 + (i + 1) % n;
        indices.extend_from_slice(&[tc_idx, i0, i1]);
    }
}

/// Append a UV-sphere of given radius at a local-space centre point.
///
/// `rings`  — number of latitude rings (equator included); `2*rings` longitude segments used.
fn machine_append_sphere(
    positions: &mut Vec<Vec3>,
    normals:   &mut Vec<Vec3>,
    colors:    &mut Vec<Srgba>,
    indices:   &mut Vec<u32>,
    radius: f32,
    rings: u32,
    color: Srgba,
    local_centre: Vec3,
    world_pivot: Vec3,
    rot: [[f32; 3]; 3],
) {
    let segs = rings * 2;
    let base = positions.len() as u32;

    for ring in 0..=rings {
        let phi = std::f32::consts::PI * ring as f32 / rings as f32; // 0 = top, π = bottom
        let (sp, cp) = (phi.sin(), phi.cos());
        for seg in 0..=segs {
            let theta = std::f32::consts::TAU * seg as f32 / segs as f32;
            let (st, ct) = (theta.sin(), theta.cos());
            let lv = Vec3::new(
                local_centre.x + radius * sp * ct,
                local_centre.y + radius * sp * st,
                local_centre.z + radius * cp,
            );
            let ln = Vec3::new(sp * ct, sp * st, cp);
            positions.push(machine_apply_rot(rot, lv) + world_pivot);
            normals.push(machine_apply_rot(rot, ln).normalize());
            colors.push(color);
        }
    }

    let stride = segs + 1;
    for ring in 0..rings {
        for seg in 0..segs {
            let i0 = base + ring * stride + seg;
            let i1 = i0 + 1;
            let i2 = i0 + stride;
            let i3 = i2 + 1;
            indices.extend_from_slice(&[i0, i2, i1, i1, i2, i3]);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────

/// Convert HSL color to RGB (h: 0-360, s: 0-1, l: 0-1)
fn hsl_to_rgb(h: f32, s: f32, l: f32) -> (f32, f32, f32) {
    let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = l - c / 2.0;

    let (r, g, b) = if h < 60.0 {
        (c, x, 0.0)
    } else if h < 120.0 {
        (x, c, 0.0)
    } else if h < 180.0 {
        (0.0, c, x)
    } else if h < 240.0 {
        (0.0, x, c)
    } else if h < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };

    (r + m, g + m, b + m)
}
