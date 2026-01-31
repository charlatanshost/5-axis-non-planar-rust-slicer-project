// GUI application entry point for MultiAxis Slicer

use multiaxis_slicer::gui::SlicerApp;

fn main() -> eframe::Result<()> {
    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    log::info!("Starting MultiAxis Slicer GUI...");

    // Configure window options
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 600.0])
            .with_icon(
                // Default icon
                eframe::icon_data::from_png_bytes(&[])
                    .unwrap_or_default()
            ),

        // CRITICAL: Enable depth buffer for 3D rendering
        // Without this, the GPU cannot determine which triangles are in front,
        // causing the mesh to look transparent/ghostly with random triangle ordering
        depth_buffer: 24,

        // RECOMMENDED: Enable 4x multisampling for smooth edges (OrcaSlicer quality)
        multisampling: 4,

        ..Default::default()
    };

    // Run the application
    eframe::run_native(
        "MultiAxis Slicer - 5-Axis Non-Planar Slicing",
        options,
        Box::new(|cc| Ok(Box::new(SlicerApp::new(cc)))),
    )
}
