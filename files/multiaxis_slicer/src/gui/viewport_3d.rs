// 3D visualization viewport with hardware-accelerated rendering using three-d

use crate::gui::app::SlicerApp;
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

    // Face selection
    last_mesh_triangle_count: usize,
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
            last_mesh_triangle_count: 0,
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
        self.last_toolpath_count = 0;
        self.last_mesh_triangle_count = 0;
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

    /// Create 3D visualization of toolpaths as instanced spheres + connecting lines (much better than 2D projection for non-planar)
    fn create_toolpath_lines(&mut self, app: &SlicerApp) {
        if app.toolpaths.is_empty() {
            self.toolpath_spheres = None;
            self.toolpath_lines = None;
            self.last_toolpath_count = 0;
            return;
        }

        let mut transforms = Vec::new();
        let mut sphere_colors = Vec::new();

        // For line segments
        let mut line_positions = Vec::new();
        let mut line_colors = Vec::new();
        let mut line_indices = Vec::new();

        let sphere_radius = 0.25; // Small spheres for toolpath points
        let line_width = 0.15;    // Thin lines connecting points
        let num_layers = app.toolpaths.len();

        // PERFORMANCE: For large toolpath sets (>50 layers), reduce detail to prevent GPU overload
        let layer_step = if num_layers > 100 { 5 } else if num_layers > 50 { 3 } else { 1 };
        let point_step = if num_layers > 100 { 5 } else if num_layers > 50 { 3 } else { 1 };

        if layer_step > 1 || point_step > 1 {
            log::warn!("PERFORMANCE: Reducing toolpath detail - showing every {}th layer, every {}th point", layer_step, point_step);
            log::warn!("  Total layers: {}, will show ~{} layers", num_layers, num_layers / layer_step);
        }

        for (layer_idx, toolpath) in app.toolpaths.iter().enumerate() {
            // PERFORMANCE: Skip layers if we're reducing detail
            if layer_idx % layer_step != 0 {
                continue;
            }

            // Color gradient from blue (bottom) to red (top) for extrusion
            let t = layer_idx as f32 / num_layers.max(1) as f32;

            // HSL color: hue from 240° (blue) to 0° (red)
            let hue = 240.0 * (1.0 - t);
            let saturation = 0.9;
            let lightness = 0.6;

            // Convert HSL to RGB for extrusion moves
            let (r, g, b) = hsl_to_rgb(hue, saturation, lightness);
            let extrusion_color = Srgba::new((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8, 255);

            // Travel move color (light cyan/gray)
            let travel_color = Srgba::new(100, 200, 220, 180);

            // PERFORMANCE: Skip spheres entirely for large toolpath sets (lines are enough)
            if num_layers <= 50 {
                // Create sphere instance for each toolpath point
                for (point_idx, segment) in toolpath.paths.iter().enumerate() {
                    // Skip points if we're reducing detail
                    if point_idx % point_step != 0 {
                        continue;
                    }

                    let position = vec3(
                        segment.position.x as f32,
                        segment.position.y as f32,
                        segment.position.z as f32,
                    );

                    // Color based on whether this is travel or extrusion
                    let is_extrusion = segment.extrusion > 1e-6;
                    let color = if is_extrusion { extrusion_color } else { travel_color };

                    // Create transformation matrix for this sphere instance
                    let transform = Mat4::from_translation(position) * Mat4::from_scale(sphere_radius);

                    transforms.push(transform);
                    sphere_colors.push(color);
                }
            }

            // Create line segments connecting consecutive points
            for (seg_idx, window) in toolpath.paths.windows(2).enumerate() {
                // Skip segments if we're reducing detail
                if seg_idx % point_step != 0 {
                    continue;
                }

                let p1 = vec3(
                    window[0].position.x as f32,
                    window[0].position.y as f32,
                    window[0].position.z as f32,
                );
                let p2 = vec3(
                    window[1].position.x as f32,
                    window[1].position.y as f32,
                    window[1].position.z as f32,
                );

                // Determine if this segment is travel or extrusion
                // Use the second point's extrusion value (where we're moving TO)
                let is_extrusion = window[1].extrusion > 1e-6;
                let line_color = if is_extrusion { extrusion_color } else { travel_color };

                // Create a thin box connecting p1 and p2
                let base_idx = line_positions.len() as u32;

                // Simple quad representation (two triangles forming a line)
                line_positions.push(p1);
                line_positions.push(p2);
                line_colors.push(line_color);
                line_colors.push(line_color);

                // We'll render these as degenerate triangles that appear as lines
                line_indices.push(base_idx);
                line_indices.push(base_idx + 1);
                line_indices.push(base_idx + 1);
            }
        }

        log::info!("Creating toolpath visualization: {} sphere instances, {} line segments",
            transforms.len(), line_indices.len() / 3);

        // CRITICAL FIX: Always update last_toolpath_count to prevent infinite loop
        // Even if transforms is empty (due to performance skip), we still processed these toolpaths
        self.last_toolpath_count = app.toolpaths.len();

        if transforms.is_empty() {
            self.toolpath_spheres = None;
            // Don't return yet - we still need to create lines!
        } else {
            // Create spheres for points
            let sphere_cpu = CpuMesh::sphere(8); // Low poly sphere for performance

            let instances = Instances {
                transformations: transforms,
                colors: Some(sphere_colors),
                ..Default::default()
            };

            let instanced_mesh = InstancedMesh::new(&self.context, &instances, &sphere_cpu);

            let sphere_material = ColorMaterial {
                color: Srgba::WHITE, // Will be modulated by instance colors
                ..Default::default()
            };

            self.toolpath_spheres = Some(Arc::new(Gm::new(instanced_mesh, sphere_material)));
        }

        // Create lines connecting points (if we have any)
        if line_positions.is_empty() || line_indices.is_empty() {
            self.toolpath_lines = None;
            return;
        }

        let line_cpu_mesh = CpuMesh {
            positions: Positions::F32(line_positions),
            colors: Some(line_colors),
            indices: Indices::U32(line_indices),
            ..Default::default()
        };

        let line_gpu_mesh = Mesh::new(&self.context, &line_cpu_mesh);

        let line_material = ColorMaterial::default(); // Uses vertex colors

        self.toolpath_lines = Some(Arc::new(Gm::new(line_gpu_mesh, line_material)));

        log::info!("✓ Created 3D toolpath visualization for {} layers", num_layers);
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
        if let Some(mesh) = &app.mesh {
            if self.mesh_bounds.is_none() || mesh.triangles.len() != self.last_mesh_triangle_count {
                log::info!("Reloading mesh to viewport (bounds_none={}, count_changed={})",
                    self.mesh_bounds.is_none(),
                    mesh.triangles.len() != self.last_mesh_triangle_count);
                self.set_mesh(mesh);
            }
        } else {
            if self.mesh_bounds.is_some() {
                self.clear_mesh();
            }
        }

        // Update toolpath lines if they changed
        if app.toolpaths.len() != self.last_toolpath_count {
            log::info!("Updating toolpath visualization ({} layers)", app.toolpaths.len());
            self.create_toolpath_lines(app);
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

        // Render using three-d
        if self.mesh_object.is_some() {
            self.render_3d(ui, rect, app);

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
            // Show empty viewport
            let painter = ui.painter();
            painter.rect_filled(
                rect,
                0.0,
                egui::Color32::from_rgb(15, 15, 20),
            );

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

    fn render_3d(&mut self, ui: &mut egui::Ui, rect: egui::Rect, app: &SlicerApp) {
        // Render directly to screen framebuffer using paint callback
        if let Some(mesh_obj) = &self.mesh_object {
            // Log occasionally to verify rendering is happening
            static mut RENDER_COUNT: u32 = 0;
            unsafe {
                RENDER_COUNT += 1;
                if RENDER_COUNT % 60 == 0 {
                    log::info!("VIEWPORT: render_3d() executing (frame {})", RENDER_COUNT);
                }
            }
            let context = self.context.clone();
            let mesh_arc = Arc::clone(mesh_obj);
            let build_plate_arc = self.build_plate_mesh.clone();
            let toolpath_spheres_arc = self.toolpath_spheres.clone();
            let toolpath_lines_arc = self.toolpath_lines.clone();
            let axes_arc = self.axes_mesh.clone();
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
                        let mut objects: Vec<&dyn three_d::Object> = vec![&*mesh_arc];

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

                        // Add coordinate axes
                        let axes_ref;
                        if let Some(ref axes) = axes_arc {
                            axes_ref = axes.clone();
                            objects.push(&*axes_ref);
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
