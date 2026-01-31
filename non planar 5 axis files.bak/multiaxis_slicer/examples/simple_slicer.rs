// Example CLI application for the slicer

use multiaxis_slicer::*;
use std::path::PathBuf;

fn main() -> Result<()> {
    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    log::info!("MultiAxis Slicer v0.1.0");
    log::info!("========================\n");

    // Configuration
    let mesh_path = PathBuf::from("examples/cube.stl");
    let output_path = PathBuf::from("output.gcode");

    // Step 1: Load mesh
    log::info!("Step 1: Loading mesh from {:?}", mesh_path);
    let mesh = if mesh_path.exists() {
        Mesh::from_stl(&mesh_path)?
    } else {
        log::warn!("Mesh file not found, creating test cube");
        create_test_cube()
    };

    log::info!("  Triangles: {}", mesh.num_triangles());
    log::info!("  Dimensions: {:?}", mesh.dimensions());
    log::info!("  Volume: {:.2} mm³\n", mesh.volume());

    // Step 2: Configure slicing
    log::info!("Step 2: Configuring slicer");
    let config = slicing::SlicingConfig {
        layer_height: 0.2,
        min_layer_height: 0.1,
        max_layer_height: 0.3,
        adaptive: false,
        tolerance: 1e-6,
    };
    log::info!("  Layer height: {} mm", config.layer_height);
    log::info!("  Tolerance: {}\n", config.tolerance);

    // Step 3: Slice mesh
    log::info!("Step 3: Slicing mesh");
    let slicer = slicing::Slicer::new(config);
    let layers = slicer.slice(&mesh)?;
    log::info!("  Generated {} layers\n", layers.len());

    // Step 4: Compute centroidal axis
    log::info!("Step 4: Computing centroidal axis");
    let centroidal_axis = centroidal_axis::CentroidalAxis::compute(&layers, 15.0);
    log::info!("  Centroids: {}", centroidal_axis.centroids.len());
    log::info!("  Break links: {}\n", centroidal_axis.break_links.len());

    // Step 5: Generate toolpaths
    log::info!("Step 5: Generating toolpaths");
    let toolpath_gen = toolpath::ToolpathGenerator::new(0.4, 0.2);
    let toolpaths = toolpath_gen.generate(&layers);
    
    let total_moves: usize = toolpaths.iter().map(|tp| tp.paths.len()).sum();
    log::info!("  Total toolpath moves: {}\n", total_moves);

    // Step 6: Generate G-code
    log::info!("Step 6: Generating G-code");
    let gcode_gen = gcode::GCodeGenerator::new();
    gcode_gen.generate(&toolpaths, &output_path)?;
    log::info!("  G-code written to: {:?}\n", output_path);

    // Summary
    log::info!("✓ Slicing complete!");
    log::info!("  Input: {:?}", mesh_path);
    log::info!("  Output: {:?}", output_path);
    log::info!("  Layers: {}", layers.len());
    log::info!("  Print time estimate: {:.1} min", estimate_print_time(&toolpaths));

    Ok(())
}

/// Create a simple test cube mesh
fn create_test_cube() -> Mesh {
    use geometry::{Point3D, Triangle};
    
    let size = 20.0;
    
    // Define 8 vertices of a cube
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

    // Define 12 triangles (2 per face, 6 faces)
    let triangles = vec![
        // Bottom face (z=0)
        Triangle::new(v[0], v[1], v[2]),
        Triangle::new(v[0], v[2], v[3]),
        // Top face (z=size)
        Triangle::new(v[4], v[6], v[5]),
        Triangle::new(v[4], v[7], v[6]),
        // Front face (y=0)
        Triangle::new(v[0], v[5], v[1]),
        Triangle::new(v[0], v[4], v[5]),
        // Back face (y=size)
        Triangle::new(v[3], v[2], v[6]),
        Triangle::new(v[3], v[6], v[7]),
        // Left face (x=0)
        Triangle::new(v[0], v[3], v[7]),
        Triangle::new(v[0], v[7], v[4]),
        // Right face (x=size)
        Triangle::new(v[1], v[5], v[6]),
        Triangle::new(v[1], v[6], v[2]),
    ];

    Mesh::new(triangles).unwrap()
}

/// Estimate print time in minutes
fn estimate_print_time(toolpaths: &[toolpath::Toolpath]) -> f64 {
    let mut total_time = 0.0;
    
    for toolpath in toolpaths {
        for segment in &toolpath.paths {
            let distance = segment.extrusion * 10.0; // Rough approximation
            total_time += distance / segment.feedrate;
        }
    }
    
    total_time / 60.0 // Convert to minutes
}
