// Control panel for file operations and slicing parameters

use crate::gui::app::SlicerApp;
use egui;
use std::path::PathBuf;

pub fn render(app: &mut SlicerApp, ctx: &egui::Context) {
    egui::SidePanel::left("control_panel")
        .default_width(300.0)
        .show(ctx, |ui| {
            ui.heading("Control Panel");
            ui.separator();

            // Wrap everything in a scroll area
            egui::ScrollArea::vertical()
                .auto_shrink([false; 2])
                .show(ui, |ui| {

            // File Operations Section
            ui.group(|ui| {
                ui.label(egui::RichText::new("File Operations").strong());
                ui.separator();

                if ui.button("📁 Load STL").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STL Files", &["stl"])
                        .pick_file()
                    {
                        app.load_mesh(path);
                    }
                }

                // Orient by Face button
                let can_orient = app.mesh.is_some() && !app.face_orientation_mode;
                if ui.add_enabled(can_orient, egui::Button::new("🔄 Orient by Face")).clicked() {
                    app.enable_face_orientation_mode();
                }

                if app.face_orientation_mode {
                    ui.label(egui::RichText::new("👆 Click a face to orient").color(egui::Color32::from_rgb(100, 255, 100)));
                }

                if ui.add_enabled(
                    app.has_toolpaths,
                    egui::Button::new("💾 Export G-code")
                ).clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("G-code Files", &["gcode", "nc"])
                        .set_file_name("output.gcode")
                        .save_file()
                    {
                        app.export_gcode(path);
                    }
                }

                ui.separator();

                // Display mesh info if available
                if let Some(stats) = app.mesh_stats() {
                    ui.label(format!("Triangles: {}", stats.triangles));
                    ui.label(format!(
                        "Dimensions: {:.1} × {:.1} × {:.1} mm",
                        stats.dimensions.0, stats.dimensions.1, stats.dimensions.2
                    ));
                    ui.label(format!("Volume: {:.2} mm³", stats.volume));
                } else {
                    ui.label(egui::RichText::new("No mesh loaded").italics().weak());
                }
            });

            ui.add_space(10.0);

            // Machine Configuration Section
            ui.group(|ui| {
                ui.label(egui::RichText::new("Machine Profile").strong());
                ui.separator();

                ui.horizontal(|ui| {
                    ui.label("Profile:");
                    ui.text_edit_singleline(&mut app.machine.name);
                });

                ui.collapsing("Workspace Limits", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("X:");
                        ui.add(egui::DragValue::new(&mut app.machine.workspace_x.0).suffix(" mm"));
                        ui.label("to");
                        ui.add(egui::DragValue::new(&mut app.machine.workspace_x.1).suffix(" mm"));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Y:");
                        ui.add(egui::DragValue::new(&mut app.machine.workspace_y.0).suffix(" mm"));
                        ui.label("to");
                        ui.add(egui::DragValue::new(&mut app.machine.workspace_y.1).suffix(" mm"));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Z:");
                        ui.add(egui::DragValue::new(&mut app.machine.workspace_z.0).suffix(" mm"));
                        ui.label("to");
                        ui.add(egui::DragValue::new(&mut app.machine.workspace_z.1).suffix(" mm"));
                    });
                });

                ui.collapsing("Rotation Axes", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("A-axis:");
                        ui.add(egui::DragValue::new(&mut app.machine.a_axis_range.0).suffix(" °"));
                        ui.label("to");
                        ui.add(egui::DragValue::new(&mut app.machine.a_axis_range.1).suffix(" °"));
                    });

                    ui.horizontal(|ui| {
                        ui.label("B-axis:");
                        ui.add(egui::DragValue::new(&mut app.machine.b_axis_range.0).suffix(" °"));
                        ui.label("to");
                        ui.add(egui::DragValue::new(&mut app.machine.b_axis_range.1).suffix(" °"));
                    });
                });

                ui.horizontal(|ui| {
                    ui.label("Max feedrate:");
                    ui.add(egui::Slider::new(&mut app.machine.max_feedrate, 10.0..=300.0)
                        .suffix(" mm/s"));
                });

                ui.checkbox(&mut app.machine.has_heated_bed, "Heated bed");
            });

            ui.add_space(10.0);

            // Slicing Parameters Section
            ui.group(|ui| {
                ui.label(egui::RichText::new("Slicing Parameters").strong());
                ui.separator();

                // Slicing mode selector
                ui.horizontal(|ui| {
                    ui.label("Slicing mode:");
                    egui::ComboBox::from_id_salt("slicing_mode")
                        .selected_text(match app.slicing_mode {
                            crate::gui::app::SlicingMode::Planar => "Planar (Traditional)",
                            crate::gui::app::SlicingMode::Curved => "Curved (S3-Slicer)",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::Planar, "Planar (Traditional)");
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::Curved, "Curved (S3-Slicer)");
                        });
                });

                ui.add_space(5.0);

                // S3-Slicer Configuration (only shown in Curved mode)
                if app.slicing_mode == crate::gui::app::SlicingMode::Curved {
                    ui.collapsing("S3-Slicer Configuration", |ui| {
                        ui.label(egui::RichText::new("Deformation Parameters").weak());

                        ui.horizontal(|ui| {
                            ui.label("Objective:");
                            use crate::s3_slicer::FabricationObjective;
                            egui::ComboBox::from_id_salt("s3_objective")
                                .selected_text(match app.s3_fabrication_objective {
                                    FabricationObjective::SupportFree => "Support-Free",
                                    FabricationObjective::Strength => "Strength",
                                    FabricationObjective::SurfaceQuality => "Surface Quality",
                                    FabricationObjective::Balanced => "Balanced",
                                })
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(&mut app.s3_fabrication_objective, FabricationObjective::SupportFree, "Support-Free")
                                        .on_hover_text("Minimize support material by optimizing overhang angles");
                                    ui.selectable_value(&mut app.s3_fabrication_objective, FabricationObjective::Strength, "Strength")
                                        .on_hover_text("Maximize part strength along build direction");
                                    ui.selectable_value(&mut app.s3_fabrication_objective, FabricationObjective::SurfaceQuality, "Surface Quality")
                                        .on_hover_text("Minimize deformation for best surface finish");
                                    ui.selectable_value(&mut app.s3_fabrication_objective, FabricationObjective::Balanced, "Balanced")
                                        .on_hover_text("Balance between support-free and strength");
                                });
                        });

                        ui.horizontal(|ui| {
                            ui.label("Overhang threshold:");
                            ui.add(egui::Slider::new(&mut app.s3_overhang_threshold, 30.0..=60.0)
                                .suffix("°"))
                                .on_hover_text("Maximum overhang angle before requiring support (lower = more aggressive)");
                        });

                        ui.horizontal(|ui| {
                            ui.label("Smoothness weight:");
                            ui.add(egui::Slider::new(&mut app.s3_smoothness_weight, 0.0..=1.0))
                                .on_hover_text("0.0 = sharp transitions, 1.0 = very smooth gradual changes");
                        });

                        ui.horizontal(|ui| {
                            ui.label("Optimization iterations:");
                            ui.add(egui::Slider::new(&mut app.s3_optimization_iterations, 10..=100))
                                .on_hover_text("More iterations = better quality but slower (50 is recommended)");
                        });

                        ui.separator();
                        ui.label(egui::RichText::new("Deformation Quality (Phase 3)").weak());

                        ui.checkbox(&mut app.s3_use_asap_deformation, "Use ASAP Deformation")
                            .on_hover_text("Enable high-quality ASAP deformation (slower but better geometry preservation)\nDisabled = Fast scale-controlled deformation");

                        // Show ASAP settings only when enabled
                        if app.s3_use_asap_deformation {
                            ui.indent("asap_settings", |ui| {
                                ui.horizontal(|ui| {
                                    ui.label("ASAP iterations:");
                                    ui.add(egui::Slider::new(&mut app.s3_asap_max_iterations, 5..=20))
                                        .on_hover_text("More iterations = better quality (10 recommended)");
                                });

                                ui.horizontal(|ui| {
                                    ui.label("Convergence:");
                                    ui.add(egui::Slider::new(&mut app.s3_asap_convergence, 1e-5..=1e-3)
                                        .logarithmic(true)
                                        .custom_formatter(|n, _| format!("{:.1e}", n)))
                                        .on_hover_text("Convergence threshold (lower = more precise)");
                                });

                                ui.label(egui::RichText::new("⚠ ASAP is 10-30x slower but produces better results")
                                    .small()
                                    .color(egui::Color32::from_rgb(255, 200, 100)));
                            });
                        }
                    });

                    ui.add_space(5.0);
                }

                ui.horizontal(|ui| {
                    ui.label("Layer height:");
                    ui.add(egui::Slider::new(&mut app.config.layer_height, 0.05..=0.5)
                        .suffix(" mm"));
                });

                ui.horizontal(|ui| {
                    ui.label("Min height:");
                    ui.add(egui::Slider::new(&mut app.config.min_layer_height, 0.05..=0.3)
                        .suffix(" mm"));
                });

                ui.horizontal(|ui| {
                    ui.label("Max height:");
                    ui.add(egui::Slider::new(&mut app.config.max_layer_height, 0.1..=1.0)
                        .suffix(" mm"));
                });

                ui.checkbox(&mut app.config.adaptive, "Adaptive slicing");

                // S3-Slicer specific parameter (only shown/used in Curved mode)
                if app.slicing_mode == crate::gui::app::SlicingMode::Curved {
                    ui.horizontal(|ui| {
                        ui.label("Max rotation:");
                        ui.add(egui::Slider::new(&mut app.config.max_rotation_degrees, 5.0..=45.0)
                            .suffix("°")
                            .step_by(1.0));
                    });
                    if ui.small_button("?").on_hover_text(
                        "Limits how much each triangle can rotate during deformation.\n\
                         Lower values = gentler deformation, more recognizable mesh\n\
                         Higher values = more aggressive optimization, potential distortion"
                    ).clicked() {}
                }

                ui.separator();

                ui.horizontal(|ui| {
                    ui.label("Nozzle diameter:");
                    ui.add(egui::Slider::new(&mut app.nozzle_diameter, 0.2..=1.2)
                        .suffix(" mm"));
                });

                ui.horizontal(|ui| {
                    ui.label("Feedrate:");
                    ui.add(egui::Slider::new(&mut app.feedrate, 10.0..=150.0)
                        .suffix(" mm/s"));
                });

                ui.separator();

                // Toolpath Pattern Selection
                ui.label(egui::RichText::new("Toolpath Pattern").strong());

                ui.horizontal(|ui| {
                    ui.label("Pattern:");
                    use crate::toolpath_patterns::ToolpathPattern;
                    egui::ComboBox::from_id_salt("toolpath_pattern")
                        .selected_text(app.toolpath_pattern.name())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut app.toolpath_pattern, ToolpathPattern::Contour, "Contour");
                            ui.selectable_value(&mut app.toolpath_pattern, ToolpathPattern::Spiral, "Spiral");
                            ui.selectable_value(&mut app.toolpath_pattern, ToolpathPattern::Zigzag, "Zigzag");
                        });
                });

                ui.horizontal(|ui| {
                    ui.label("Line width:");
                    ui.add(egui::Slider::new(&mut app.toolpath_line_width, 0.2..=2.0)
                        .suffix(" mm"));
                });

                if app.toolpath_pattern == crate::toolpath_patterns::ToolpathPattern::Zigzag {
                    ui.horizontal(|ui| {
                        ui.label("Infill density:");
                        ui.add(egui::Slider::new(&mut app.toolpath_infill_density, 0.0..=1.0)
                            .suffix("%")
                            .custom_formatter(|v, _| format!("{:.0}", v * 100.0)));
                    });
                }

                ui.separator();

                ui.horizontal(|ui| {
                    ui.label("Tolerance:");
                    ui.add(egui::DragValue::new(&mut app.config.tolerance)
                        .speed(1e-7)
                        .clamp_range(1e-10..=1e-3));
                });
            });

            ui.add_space(10.0);

            // Visualization Controls Section
            ui.group(|ui| {
                ui.label(egui::RichText::new("Visualization").strong());
                ui.separator();

                ui.checkbox(&mut app.show_mesh, "Show mesh");
                ui.checkbox(&mut app.show_layers, "Show layers");
                ui.checkbox(&mut app.show_toolpaths, "Show toolpaths");
                ui.checkbox(&mut app.show_supports, "Show supports");
                ui.checkbox(&mut app.show_wireframe, "Wireframe mode");
                ui.checkbox(&mut app.show_gcode_terminal, "Show G-code terminal");

                ui.separator();

                // Toolpath playback controls
                if !app.toolpaths.is_empty() {
                    ui.label("Toolpath Playback:");
                    ui.checkbox(&mut app.toolpath_playback_enabled, "Enable playback");

                    if app.toolpath_playback_enabled {
                        // Calculate total segments
                        let total_segments: usize = app.toolpaths.iter()
                            .map(|tp| tp.paths.len())
                            .sum();

                        if total_segments > 0 {
                            ui.add(egui::Slider::new(&mut app.toolpath_playback_position, 0..=total_segments.saturating_sub(1))
                                .text("Position"));

                            // Playback controls
                            ui.horizontal(|ui| {
                                if ui.button("⏮").clicked() {
                                    app.toolpath_playback_position = 0;
                                    app.toolpath_playback_playing = false;
                                }

                                if ui.button("◀").clicked() {
                                    if app.toolpath_playback_position > 0 {
                                        app.toolpath_playback_position -= 1;
                                    }
                                    app.toolpath_playback_playing = false;
                                    // Update highlighted line
                                    let header_lines = 11;
                                    app.gcode_highlight_line = Some(header_lines + app.toolpath_playback_position);
                                }

                                let play_button_text = if app.toolpath_playback_playing { "⏸" } else { "▶" };
                                if ui.button(play_button_text).clicked() {
                                    app.toolpath_playback_playing = !app.toolpath_playback_playing;
                                    app.last_playback_time = std::time::Instant::now();
                                }

                                if ui.button("▶").clicked() {
                                    if app.toolpath_playback_position < total_segments.saturating_sub(1) {
                                        app.toolpath_playback_position += 1;
                                    }
                                    app.toolpath_playback_playing = false;
                                    // Update highlighted line
                                    let header_lines = 11;
                                    app.gcode_highlight_line = Some(header_lines + app.toolpath_playback_position);
                                }

                                if ui.button("⏭").clicked() {
                                    app.toolpath_playback_position = total_segments.saturating_sub(1);
                                    app.toolpath_playback_playing = false;
                                }
                            });

                            // Speed control
                            ui.horizontal(|ui| {
                                ui.label("Speed:");
                                ui.add(egui::Slider::new(&mut app.toolpath_playback_speed, 1.0..=100.0)
                                    .suffix(" seg/s")
                                    .logarithmic(true));
                            });

                            ui.label(format!("Segment {}/{}", app.toolpath_playback_position + 1, total_segments));
                        }
                    }
                }

                ui.separator();

                // Layer selector
                if !app.layers.is_empty() {
                    ui.label("Layer preview:");
                    let mut layer_idx = app.selected_layer.unwrap_or(0);
                    ui.add(egui::Slider::new(&mut layer_idx, 0..=app.layers.len().saturating_sub(1))
                        .text("Layer"));

                    if layer_idx < app.layers.len() {
                        app.selected_layer = Some(layer_idx);
                        let z = app.layers[layer_idx].z;
                        ui.label(format!("Z = {:.2} mm", z));
                    }
                } else {
                    ui.label(egui::RichText::new("No layers yet").italics().weak());
                }
            });

            ui.add_space(10.0);

            // Actions Section
            ui.group(|ui| {
                ui.label(egui::RichText::new("Actions").strong());
                ui.separator();

                let can_slice = app.mesh.is_some() && !app.is_slicing;
                let can_deform = app.mesh.is_some() && !app.is_deforming && app.slicing_mode == crate::gui::app::SlicingMode::Curved;

                // Compute S3-Slicer quaternion field (only for Curved mode)
                if app.slicing_mode == crate::gui::app::SlicingMode::Curved {
                    if ui.add_enabled(can_deform, egui::Button::new("🧮 Compute Quaternion Field").min_size(egui::vec2(ui.available_width(), 30.0))).clicked() {
                        app.deform_mesh_preview();
                    }

                    if app.is_deforming {
                        ui.label(egui::RichText::new("⏳ Computing quaternion field...").italics());
                    }

                    if app.deformed_mesh.is_some() {
                        ui.label(egui::RichText::new("✓ Quaternion field ready").color(egui::Color32::from_rgb(100, 200, 100)));
                    }

                    ui.add_space(5.0);
                }

                if ui.add_enabled(can_slice, egui::Button::new("🔪 Slice!").min_size(egui::vec2(ui.available_width(), 40.0))).clicked() {
                    app.start_slicing();
                }

                if app.is_slicing {
                    ui.label(egui::RichText::new("⏳ Slicing in progress...").italics());
                }

                ui.add_space(5.0);

                let can_generate = app.has_sliced && !app.layers.is_empty();
                log::debug!("DEBUG Button state: has_sliced={}, layers.len()={}, can_generate={}",
                    app.has_sliced, app.layers.len(), can_generate);

                if ui.add_enabled(can_generate, egui::Button::new("⚙ Generate Toolpaths").min_size(egui::vec2(ui.available_width(), 30.0))).clicked() {
                    app.generate_toolpaths();
                }

                ui.add_space(5.0);

                // Support Generation (Optional)
                ui.collapsing("🏗 Support Generation (Optional)", |ui| {
                    ui.label(egui::RichText::new("Supports are optional for most prints").weak().italics());

                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        ui.label("Overhang angle:");
                        ui.add(egui::Slider::new(&mut app.support_config.overhang_angle, 0.0..=89.0)
                            .suffix("°"));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Min area:");
                        ui.add(egui::DragValue::new(&mut app.support_config.min_area_threshold)
                            .speed(0.1)
                            .suffix(" mm²"));
                    });

                    ui.checkbox(&mut app.support_config.use_curved_layers, "Use curved layer analysis");

                    ui.add_space(5.0);

                    let can_generate_supports = app.mesh.is_some() && app.has_sliced;

                    if ui.add_enabled(can_generate_supports, egui::Button::new("Generate Supports").min_size(egui::vec2(ui.available_width(), 30.0))).clicked() {
                        app.generate_supports();
                    }

                    if app.has_supports {
                        ui.label(egui::RichText::new("✓ Supports generated").color(egui::Color32::from_rgb(100, 255, 100)));

                        if ui.add_enabled(true, egui::Button::new("Generate Support Toolpaths").min_size(egui::vec2(ui.available_width(), 25.0))).clicked() {
                            app.generate_support_toolpaths();
                        }

                        if let Some(result) = &app.support_result {
                            if !result.toolpaths.is_empty() {
                                ui.label(egui::RichText::new("✓ Support toolpaths ready").color(egui::Color32::from_rgb(100, 255, 100)));
                            }
                        }
                    }
                });

                ui.add_space(5.0);

                let can_motion_plan = app.has_toolpaths && !app.toolpaths.is_empty();

                if app.is_motion_planning {
                    ui.label(egui::RichText::new("⏳ Running motion planning...").italics());
                }

                if ui.add_enabled(can_motion_plan && !app.is_motion_planning, egui::Button::new("🤖 Run Motion Planning").min_size(egui::vec2(ui.available_width(), 30.0))).clicked() {
                    app.run_motion_planning();
                }

                if app.has_motion_plan {
                    ui.label(egui::RichText::new("✓ Motion plan optimized").color(egui::Color32::from_rgb(100, 255, 100)));
                }

                ui.add_space(5.0);

                let can_generate_gcode = app.has_motion_plan || (app.has_toolpaths && !app.toolpaths.is_empty());

                if ui.add_enabled(can_generate_gcode, egui::Button::new("📄 Generate G-code").min_size(egui::vec2(ui.available_width(), 30.0))).clicked() {
                    app.generate_gcode_from_toolpaths();
                }

                if !app.gcode_lines.is_empty() {
                    ui.label(egui::RichText::new("✓ G-code ready").color(egui::Color32::from_rgb(100, 255, 100)));
                }
            });

            ui.add_space(10.0);

            // G-code Preview Section
            if !app.gcode_lines.is_empty() {
                ui.group(|ui| {
                    ui.label(egui::RichText::new("G-code Preview").strong());
                    ui.separator();

                    egui::ScrollArea::vertical()
                        .max_height(200.0)
                        .show(ui, |ui| {
                            for (i, line) in app.gcode_lines.iter().take(50).enumerate() {
                                ui.label(format!("{:4}: {}", i + 1, line));
                            }

                            if app.gcode_lines.len() > 50 {
                                ui.label(format!("... ({} more lines)", app.gcode_lines.len() - 50));
                            }
                        });
                });
            }

            }); // End ScrollArea
        });
}
