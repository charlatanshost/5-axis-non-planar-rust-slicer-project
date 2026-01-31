// G-code preview panel with syntax highlighting

use egui;

/// Render G-code preview panel (can be shown as a window or panel)
pub fn render_window(ctx: &egui::Context, gcode_lines: &[String], is_open: &mut bool) {
    egui::Window::new("G-code Preview")
        .open(is_open)
        .default_size([600.0, 400.0])
        .show(ctx, |ui| {
            render_content(ui, gcode_lines);
        });
}

pub fn render_content(ui: &mut egui::Ui, gcode_lines: &[String]) {
    ui.heading("G-code Preview");
    ui.separator();

    if gcode_lines.is_empty() {
        ui.label(egui::RichText::new("No G-code generated yet").italics().weak());
        return;
    }

    ui.label(format!("Total lines: {}", gcode_lines.len()));
    ui.separator();

    // Scrollable G-code display
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            // Use monospace font for code
            let font_id = egui::FontId::monospace(12.0);

            for (i, line) in gcode_lines.iter().enumerate() {
                ui.horizontal(|ui| {
                    // Line number
                    ui.label(
                        egui::RichText::new(format!("{:5}", i + 1))
                            .font(font_id.clone())
                            .weak()
                    );

                    // G-code line with basic syntax coloring
                    let colored_text = if line.trim().starts_with(';') {
                        // Comment
                        egui::RichText::new(line).color(egui::Color32::DARK_GRAY)
                    } else if line.contains('G') || line.contains('M') {
                        // G-code or M-code command
                        egui::RichText::new(line).color(egui::Color32::LIGHT_BLUE)
                    } else {
                        // Default
                        egui::RichText::new(line).color(egui::Color32::WHITE)
                    };

                    ui.label(colored_text.font(font_id.clone()));
                });
            }
        });
}
