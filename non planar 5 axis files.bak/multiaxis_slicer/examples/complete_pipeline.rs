// Complete 5-axis motion planning pipeline example
// Combines slicing with motion planning algorithms from MultiAxis_3DP_MotionPlanning

use multiaxis_slicer::*;
use motion_planning::*;
use std::path::PathBuf;

fn main() -> Result<()> {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    println!("\n╔═══════════════════════════════════════════════════════╗");
    println!("║   5-Axis Motion Planning & Slicing Pipeline          ║");
    println!("║   Based on MultiAxis_3DP_MotionPlanning (RAL 2021)   ║");
    println!("╚═══════════════════════════════════════════════════════╝\n");

    // ============================================================
    // PART 1: SLICING (from your existing code)
    // ============================================================
    
    println!("┌─ PART 1: Mesh Slicing ─────────────────────────────┐");
    
    let mesh_path = PathBuf::from("examples/test_model.stl");
    let mesh = if mesh_path.exists() {
        Mesh::from_stl(&mesh_path)?
    } else {
        println!("│ Creating test cube (20mm)...");
        create_test_cube()
    };

    println!("│ Mesh loaded:");
    println!("│   - Triangles: {}", mesh.num_triangles());
    println!("│   - Dimensions: {:.1?} mm", mesh.dimensions());
    
    // Slice the mesh
    let config = slicing::SlicingConfig {
        layer_height: 0.2,
        tolerance: 1e-6,
        ..Default::default()
    };

    let slicer = slicing::Slicer::new(config);
    let layers = slicer.slice(&mesh)?;
    println!("│ Generated {} layers", layers.len());

    // Compute centroidal axis
    let axis = centroidal_axis::CentroidalAxis::compute(&layers, 15.0);
    println!("│ Centroidal axis: {} centroids, {} breaks", 
             axis.centroids.len(), axis.break_links.len());
    println!("└─────────────────────────────────────────────────────┘\n");

    // ============================================================
    // PART 2: GENERATE WAYPOINTS FROM LAYERS
    // ============================================================
    
    println!("┌─ PART 2: Waypoint Generation ──────────────────────┐");
    
    let waypoints = generate_waypoints_from_layers(&layers, &axis);
    println!("│ Generated {} waypoints from {} layers", 
             waypoints.len(), layers.len());
    println!("└─────────────────────────────────────────────────────┘\n");

    // ============================================================
    // PART 3: MOTION PLANNING (MultiAxis_3DP_MotionPlanning)
    // ============================================================
    
    println!("┌─ PART 3: Motion Planning Pipeline ─────────────────┐");
    
    let motion_config = MotionPlanningConfig::default();
    let planner = MotionPlanner::new(motion_config);

    // Run the complete 6-step pipeline
    println!("│");
    println!("│ Step 1: Variable Filament Calculation");
    println!("│   └─ Computing extrusion for each waypoint...");
    
    println!("│ Step 2: Singularity Optimization");
    println!("│   └─ Computing IK solutions and detecting singularities...");
    
    println!("│ Step 3: Collision Checking");  
    println!("│   └─ Checking print head vs platform/model...");
    
    println!("│ Step 4: Graph Search");
    println!("│   └─ Finding optimal collision-free path...");

    let motion_plan = match planner.plan_complete_path(waypoints) {
        Ok(plan) => {
            println!("│ Step 5: ✓ Motion plan complete!");
            println!("│   └─ Found valid path through {} waypoints", plan.waypoints.len());
            plan
        }
        Err(e) => {
            println!("│ Step 5: ✗ Motion planning failed");
            println!("│   └─ Error: {}", e);
            return Err(e);
        }
    };
    
    println!("└─────────────────────────────────────────────────────┘\n");

    // ============================================================
    // PART 4: G-CODE GENERATION
    // ============================================================
    
    println!("┌─ PART 4: G-Code Generation ────────────────────────┐");
    
    let toolpaths = convert_motion_plan_to_toolpaths(&motion_plan);
    println!("│ Converted to {} toolpath segments", toolpaths.len());

    let gcode_gen = gcode::GCodeGenerator::new();
    let output_path = PathBuf::from("5axis_output.gcode");
    gcode_gen.generate(&toolpaths, &output_path)?;
    
    println!("│ ✓ G-code written to: {:?}", output_path);
    println!("└─────────────────────────────────────────────────────┘\n");

    // ============================================================
    // SUMMARY
    // ============================================================
    
    println!("┌─ SUMMARY ───────────────────────────────────────────┐");
    println!("│ Input:  Test cube (20mm)");
    println!("│ Layers: {}", layers.len());
    println!("│ Waypoints: {}", motion_plan.waypoints.len());
    println!("│ Output: {:?}", output_path);
    println!("│");
    println!("│ Estimated print time: {:.1} min", 
             estimate_print_time(&toolpaths));
    println!("│ Material used: {:.2} m", 
             total_extrusion(&motion_plan.waypoints));
    println!("│");
    println!("│ ✓ Pipeline complete! Ready for 5-axis printing.");
    println!("└─────────────────────────────────────────────────────┘\n");

    Ok(())
}

/// Generate waypoints from sliced layers with orientation
fn generate_waypoints_from_layers(
    layers: &[slicing::Layer],
    axis: &centroidal_axis::CentroidalAxis,
) -> Vec<Waypoint> {
    let mut waypoints = Vec::new();

    for (layer_idx, layer) in layers.iter().enumerate() {
        // Get slicing direction from centroidal axis
        let orientation = if layer_idx < axis.links.len() {
            axis.links[layer_idx].direction
        } else {
            geometry::Vector3D::new(0.0, 0.0, 1.0)
        };

        // Generate waypoints for each contour
        for contour in &layer.contours {
            for point in &contour.points {
                waypoints.push(Waypoint {
                    position: *point,
                    orientation,
                    layer_idx,
                    extrusion: 0.0,  // Will be calculated in Step 1
                });
            }
        }
    }

    waypoints
}

/// Convert motion plan to toolpaths for G-code generation
fn convert_motion_plan_to_toolpaths(plan: &MotionPlan) -> Vec<toolpath::Toolpath> {
    let mut toolpaths = Vec::new();
    let mut current_z = 0.0;
    let mut layer_segments = Vec::new();

    for waypoint in &plan.waypoints {
        if waypoint.position.z != current_z {
            // New layer
            if !layer_segments.is_empty() {
                toolpaths.push(toolpath::Toolpath {
                    paths: layer_segments.clone(),
                    z: current_z,
                });
                layer_segments.clear();
            }
            current_z = waypoint.position.z;
        }

        layer_segments.push(toolpath::ToolpathSegment {
            position: waypoint.position,
            orientation: waypoint.orientation,
            extrusion: waypoint.extrusion,
            feedrate: 50.0,
        });
    }

    // Add final layer
    if !layer_segments.is_empty() {
        toolpaths.push(toolpath::Toolpath {
            paths: layer_segments,
            z: current_z,
        });
    }

    toolpaths
}

/// Create simple test cube
fn create_test_cube() -> Mesh {
    use geometry::{Point3D, Triangle};
    
    let size = 20.0;
    let v = [
        Point3D::new(0.0, 0.0, 0.0),
        Point3D::new(size, 0.0, 0.0),
        Point3D::new(size, size, 0.0),
        Point3D::new(0.0, size, 0.0),
        Point3D::new(0.0, 0.0, size),
        Point3D::new(size, 0.0, size),
        Point3D::new(size, size, size),
        Point3D::new(0.0, size, size),
    ];

    let triangles = vec![
        Triangle::new(v[0], v[1], v[2]), Triangle::new(v[0], v[2], v[3]),
        Triangle::new(v[4], v[6], v[5]), Triangle::new(v[4], v[7], v[6]),
        Triangle::new(v[0], v[5], v[1]), Triangle::new(v[0], v[4], v[5]),
        Triangle::new(v[3], v[2], v[6]), Triangle::new(v[3], v[6], v[7]),
        Triangle::new(v[0], v[3], v[7]), Triangle::new(v[0], v[7], v[4]),
        Triangle::new(v[1], v[5], v[6]), Triangle::new(v[1], v[6], v[2]),
    ];

    Mesh::new(triangles).unwrap()
}

/// Estimate print time
fn estimate_print_time(toolpaths: &[toolpath::Toolpath]) -> f64 {
    let mut total_time = 0.0;
    
    for toolpath in toolpaths {
        for segment in &toolpath.paths {
            let distance = segment.extrusion * 10.0;
            total_time += distance / segment.feedrate;
        }
    }
    
    total_time / 60.0
}

/// Calculate total filament used
fn total_extrusion(waypoints: &[Waypoint]) -> f64 {
    waypoints.iter().map(|wp| wp.extrusion).sum()
}
