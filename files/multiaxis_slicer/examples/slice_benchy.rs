// Example: Slice the 3DBenchy model

use multiaxis_slicer::*;
use std::path::PathBuf;

fn main() -> Result<()> {
    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    log::info!("MultiAxis Slicer - 3DBenchy Test");
    log::info!("==================================\n");

    // Configuration
    let mesh_path = PathBuf::from("../3dbenchy.stl");
    let output_path = PathBuf::from("benchy_output.gcode");

    // Step 1: Load mesh
    log::info!("Step 1: Loading 3DBenchy from {:?}", mesh_path);
    let mesh = Mesh::from_stl(&mesh_path)?;

    log::info!("  Triangles: {}", mesh.num_triangles());
    log::info!("  Dimensions: {:?}", mesh.dimensions());
    log::info!("  Volume: {:.2} mm³\n", mesh.volume());

    // Step 2: Configure slicing
    log::info!("Step 2: Configuring slicer");
    let config = slicing::SlicingConfig {
        layer_height: 0.3, // Larger for faster test
        min_layer_height: 0.2,
        max_layer_height: 0.4,
        adaptive: false,
        tolerance: 1e-6,
    };
    log::info!("  Layer height: {} mm", config.layer_height);
    log::info!("  Tolerance: {}\n", config.tolerance);

    // Step 3: Slice mesh
    log::info!("Step 3: Slicing mesh (this may take a moment for complex models)...");
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
    let toolpath_gen = toolpath::ToolpathGenerator::new(0.4, 0.3);
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
    log::info!("  Triangles processed: {}", mesh.num_triangles());

    Ok(())
}
