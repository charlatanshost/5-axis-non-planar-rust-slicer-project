// Statistics and progress panel

use crate::gui::app::SlicerApp;
use egui;

pub fn render(app: &SlicerApp, ctx: &egui::Context) {
    egui::TopBottomPanel::bottom("stats_panel")
        .default_height(150.0)
        .show(ctx, |ui| {
            ui.heading("Statistics & Progress");
            ui.separator();

            ui.horizontal(|ui| {
                // Left column: Operation status and progress
                ui.vertical(|ui| {
                    ui.set_min_width(400.0);

                    // Get current progress
                    let progress = app.slicing_progress.lock().unwrap();

                    ui.horizontal(|ui| {
                        ui.label("Status:");
                        let color = match progress.operation.as_str() {
                            "Idle" => egui::Color32::GRAY,
                            "Slicing" => egui::Color32::LIGHT_BLUE,
                            "Complete" => egui::Color32::GREEN,
                            "Error" => egui::Color32::RED,
                            _ => egui::Color32::WHITE,
                        };
                        ui.colored_label(color, &progress.operation);
                    });

                    if progress.percentage > 0.0 && progress.percentage < 100.0 {
                        ui.add(egui::ProgressBar::new(progress.percentage / 100.0)
                            .text(format!("{:.0}%", progress.percentage))
                            .animate(true));
                    }

                    if !progress.message.is_empty() {
                        ui.label(&progress.message);
                    }
                });

                ui.separator();

                // Middle column: Slicing statistics
                ui.vertical(|ui| {
                    ui.set_min_width(250.0);

                    ui.label(egui::RichText::new("Slicing Stats").strong());

                    let stats = app.slicing_stats();

                    ui.horizontal(|ui| {
                        ui.label("Layers:");
                        if stats.layers > 0 {
                            ui.colored_label(egui::Color32::GREEN, format!("{}", stats.layers));
                        } else {
                            ui.label("0");
                        }
                    });

                    ui.horizontal(|ui| {
                        ui.label("Toolpath moves:");
                        if stats.total_moves > 0 {
                            ui.colored_label(egui::Color32::GREEN, format!("{}", stats.total_moves));
                        } else {
                            ui.label("0");
                        }
                    });

                    if stats.has_toolpaths && stats.total_moves > 0 {
                        let estimated_time = estimate_print_time(&app.toolpaths);
                        ui.horizontal(|ui| {
                            ui.label("Est. print time:");
                            ui.label(format!("{:.1} min", estimated_time));
                        });
                    }
                });

                ui.separator();

                // Right column: Logs
                ui.vertical(|ui| {
                    ui.label(egui::RichText::new("Recent Logs").strong());

                    egui::ScrollArea::vertical()
                        .max_height(80.0)
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            for msg in app.log_messages.iter().rev().take(5).rev() {
                                ui.label(egui::RichText::new(msg).small().weak());
                            }
                        });
                });
            });
        });
}

/// Estimate print time in minutes
fn estimate_print_time(toolpaths: &[crate::toolpath::Toolpath]) -> f64 {
    let mut total_time = 0.0;

    for toolpath in toolpaths {
        for segment in &toolpath.paths {
            let distance = segment.extrusion * 10.0; // Rough approximation
            total_time += distance / segment.feedrate;
        }
    }

    total_time / 60.0 // Convert to minutes
}
