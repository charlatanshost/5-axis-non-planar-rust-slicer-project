// Printer Profiles page — browser-tab-style editor for PrinterProfile entries.
// Renders inside CentralPanel when AppTab::PrinterProfiles is active.

use crate::gui::app::{SlicerApp, AppTab, PrinterProfile, AxisConfig, AxisType, MovingPart, KinematicsType, BedShape};

pub fn render(app: &mut SlicerApp, ui: &mut egui::Ui) {
    ui.heading("Printer Profiles");
    ui.separator();

    // ── Profile selector toolbar ──────────────────────────────────────────
    ui.horizontal(|ui| {
        ui.label("Profile:");

        // ComboBox to pick which profile is being edited
        let editing_name = app.profiles
            .get(app.editing_profile_index)
            .map(|p| p.name.clone())
            .unwrap_or_else(|| "—".to_string());

        egui::ComboBox::from_id_source("edit_profile_selector")
            .selected_text(&editing_name)
            .show_ui(ui, |ui| {
                for i in 0..app.profiles.len() {
                    let name = app.profiles[i].name.clone();
                    ui.selectable_value(&mut app.editing_profile_index, i, &name);
                }
            });

        if ui.button("+ New").clicked() {
            let mut p = PrinterProfile::default();
            p.name = format!("Printer {}", app.profiles.len() + 1);
            app.profiles.push(p);
            app.editing_profile_index = app.profiles.len() - 1;
        }

        if ui.button("Duplicate").clicked() {
            if let Some(p) = app.profiles.get(app.editing_profile_index).cloned() {
                let mut copy = p;
                copy.name = format!("{} (copy)", copy.name);
                app.profiles.push(copy);
                app.editing_profile_index = app.profiles.len() - 1;
            }
        }

        // Keep at least one profile
        let can_delete = app.profiles.len() > 1;
        if ui.add_enabled(can_delete, egui::Button::new("Delete")).clicked() {
            app.profiles.remove(app.editing_profile_index);
            app.editing_profile_index = app.editing_profile_index.saturating_sub(1);
            // Also clamp active index
            if app.active_profile_index >= app.profiles.len() {
                app.active_profile_index = app.profiles.len() - 1;
            }
        }
    });

    // File I/O row
    ui.horizontal(|ui| {
        if ui.button("📁 Load Profile...").clicked() {
            app.profile_io_error = None;
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("Printer Profile", &["json"])
                .pick_file()
            {
                match std::fs::read_to_string(&path) {
                    Ok(json) => match serde_json::from_str::<PrinterProfile>(&json) {
                        Ok(mut profile) => {
                            // Use the filename stem as the profile name so the dropdown
                            // shows something meaningful (e.g. "My_Printer" from "My_Printer.json").
                            if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                                profile.name = stem.replace('_', " ");
                            }
                            app.profiles.push(profile);
                            app.editing_profile_index = app.profiles.len() - 1;
                            // Immediately make the newly-loaded profile active so it shows
                            // in the viewport without needing to click "Apply".
                            app.active_profile_index = app.editing_profile_index;
                            app.apply_active_profile();
                        }
                        Err(e) => {
                            let msg = format!("Failed to parse profile: {}", e);
                            log::error!("{}", msg);
                            app.profile_io_error = Some(msg);
                        }
                    },
                    Err(e) => {
                        let msg = format!("Failed to read file: {}", e);
                        log::error!("{}", msg);
                        app.profile_io_error = Some(msg);
                    }
                }
            }
        }

        if ui.button("💾 Save Profile As...").clicked() {
            app.profile_io_error = None;
            if let Some(profile) = app.profiles.get(app.editing_profile_index).cloned() {
                let default_name = format!("{}.json", profile.name.replace(' ', "_"));
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("Printer Profile", &["json"])
                    .set_file_name(&default_name)
                    .save_file()
                {
                    match serde_json::to_string_pretty(&profile) {
                        Ok(json) => {
                            if let Err(e) = std::fs::write(&path, &json) {
                                let msg = format!("Failed to write file: {}", e);
                                log::error!("{}", msg);
                                app.profile_io_error = Some(msg);
                            }
                        }
                        Err(e) => {
                            let msg = format!("Failed to serialize profile: {}", e);
                            log::error!("{}", msg);
                            app.profile_io_error = Some(msg);
                        }
                    }
                }
            }
        }
    });

    // Show I/O error inline if present
    if let Some(ref err) = app.profile_io_error.clone() {
        ui.horizontal(|ui| {
            ui.colored_label(egui::Color32::RED, format!("⚠ {}", err));
            if ui.small_button("✕").clicked() {
                app.profile_io_error = None;
            }
        });
    }

    ui.separator();

    // ── Profile editor ────────────────────────────────────────────────────
    // Guard: make sure editing index is valid
    if app.editing_profile_index >= app.profiles.len() {
        app.editing_profile_index = app.profiles.len().saturating_sub(1);
    }

    // We need to work around the borrow checker: clone the profile,
    // edit it, then write it back.
    let mut profile = app.profiles[app.editing_profile_index].clone();
    let changed = render_profile_editor(ui, &mut profile);
    if changed {
        app.profiles[app.editing_profile_index] = profile;
    }

    ui.separator();

    // ── Apply button ──────────────────────────────────────────────────────
    ui.horizontal(|ui| {
        let is_active = app.active_profile_index == app.editing_profile_index;
        let label = if is_active {
            "✓ Active Profile"
        } else {
            "Apply to Current Session"
        };

        if ui.add_enabled(!is_active, egui::Button::new(label)).clicked() {
            app.active_profile_index = app.editing_profile_index;
            app.apply_active_profile();
            app.active_tab = AppTab::Main;
        }

        if is_active {
            ui.label(
                egui::RichText::new("(this profile is currently active)")
                    .weak()
                    .italics(),
            );
        }
    });
}

/// Returns `true` if any field was changed.
fn render_profile_editor(ui: &mut egui::Ui, profile: &mut PrinterProfile) -> bool {
    let mut changed = false;

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {

        // ── Name ─────────────────────────────────────────────────────────
        ui.horizontal(|ui| {
            ui.label("Name:");
            if ui.text_edit_singleline(&mut profile.name).changed() { changed = true; }
        });

        ui.add_space(4.0);

        // ── Kinematics type ───────────────────────────────────────────────
        ui.horizontal(|ui| {
            ui.label("Kinematics:");
            let current_label = profile.kinematics_type.label();
            egui::ComboBox::from_id_source("kinematics_type")
                .selected_text(current_label)
                .show_ui(ui, |ui| {
                    for kt in KinematicsType::all() {
                        if ui.selectable_label(&profile.kinematics_type == kt, kt.label()).clicked() {
                            profile.kinematics_type = kt.clone();
                            changed = true;
                        }
                    }
                });
        });

        ui.add_space(8.0);

        // ── Build Volume ──────────────────────────────────────────────────
        ui.collapsing("Build Volume", |ui| {
            ui.horizontal(|ui| {
                ui.label("X:");
                if ui.add(egui::DragValue::new(&mut profile.workspace_x.0).suffix(" mm").speed(1.0)).changed() { changed = true; }
                ui.label("to");
                if ui.add(egui::DragValue::new(&mut profile.workspace_x.1).suffix(" mm").speed(1.0)).changed() { changed = true; }
            });
            ui.horizontal(|ui| {
                ui.label("Y:");
                if ui.add(egui::DragValue::new(&mut profile.workspace_y.0).suffix(" mm").speed(1.0)).changed() { changed = true; }
                ui.label("to");
                if ui.add(egui::DragValue::new(&mut profile.workspace_y.1).suffix(" mm").speed(1.0)).changed() { changed = true; }
            });
            ui.horizontal(|ui| {
                ui.label("Z:");
                if ui.add(egui::DragValue::new(&mut profile.workspace_z.0).suffix(" mm").speed(1.0)).changed() { changed = true; }
                ui.label("to");
                if ui.add(egui::DragValue::new(&mut profile.workspace_z.1).suffix(" mm").speed(1.0)).changed() { changed = true; }
            });
        });

        ui.add_space(4.0);

        // ── Rotary Axes ───────────────────────────────────────────────────
        ui.collapsing("Rotary Axes", |ui| {
            ui.label(
                egui::RichText::new("Up to 7 axes. Use for A/B/C/trunnion/robotic arm joints.")
                    .weak()
                    .small(),
            );
            ui.add_space(4.0);

            // Column headers
            ui.horizontal(|ui| {
                ui.add_sized([40.0, 0.0], egui::Label::new(egui::RichText::new("Name").weak()));
                ui.add_sized([60.0, 0.0], egui::Label::new(egui::RichText::new("Type").weak()));
                ui.add_sized([60.0, 0.0], egui::Label::new(egui::RichText::new("Moves").weak()));
                ui.add_sized([60.0, 0.0], egui::Label::new(egui::RichText::new("Min").weak()));
                ui.add_sized([60.0, 0.0], egui::Label::new(egui::RichText::new("Max").weak()));
            });
            ui.separator();

            let mut to_remove: Option<usize> = None;
            for i in 0..profile.rotary_axes.len() {
                let ax = &mut profile.rotary_axes[i];
                ui.horizontal(|ui| {
                    // Name
                    let mut name = ax.name.clone();
                    let name_resp = ui.add_sized([40.0, 20.0], egui::TextEdit::singleline(&mut name));
                    if name_resp.changed() { ax.name = name; changed = true; }

                    // Type
                    let type_label = ax.axis_type.label();
                    egui::ComboBox::from_id_source(format!("axis_type_{}", i))
                        .selected_text(type_label)
                        .width(65.0)
                        .show_ui(ui, |ui| {
                            for at in AxisType::all() {
                                if ui.selectable_label(&ax.axis_type == at, at.label()).clicked() {
                                    ax.axis_type = at.clone();
                                    changed = true;
                                }
                            }
                        });

                    // Moving part
                    let part_label = ax.moving_part.label();
                    egui::ComboBox::from_id_source(format!("axis_part_{}", i))
                        .selected_text(part_label)
                        .width(60.0)
                        .show_ui(ui, |ui| {
                            for mp in MovingPart::all() {
                                if ui.selectable_label(&ax.moving_part == mp, mp.label()).clicked() {
                                    ax.moving_part = mp.clone();
                                    changed = true;
                                }
                            }
                        });

                    // Min / Max
                    let suffix = if ax.axis_type == AxisType::Rotary { " °" } else { " mm" };
                    if ui.add(egui::DragValue::new(&mut ax.min).suffix(suffix).speed(1.0)).changed() { changed = true; }
                    ui.label("to");
                    if ui.add(egui::DragValue::new(&mut ax.max).suffix(suffix).speed(1.0)).changed() { changed = true; }

                    // Remove button
                    if ui.small_button("✕").clicked() {
                        to_remove = Some(i);
                    }
                });
            }

            if let Some(idx) = to_remove {
                profile.rotary_axes.remove(idx);
                changed = true;
            }

            ui.add_space(4.0);
            if ui.add_enabled(
                profile.rotary_axes.len() < 7,
                egui::Button::new("+ Add Axis"),
            ).clicked() {
                profile.rotary_axes.push(AxisConfig::default());
                changed = true;
            }
            if profile.rotary_axes.len() >= 7 {
                ui.label(egui::RichText::new("Maximum 7 axes reached").weak().small());
            }
        });

        ui.add_space(4.0);

        // ── 5-Axis G-code Output ──────────────────────────────────────────
        ui.collapsing("5-Axis G-code Output", |ui| {
            ui.horizontal(|ui| {
                ui.label("Rotary axis labels:");
                egui::ComboBox::from_id_source("rotary_axis_mode_profile")
                    .selected_text(match profile.rotary_axis_mode {
                        crate::gcode::RotaryAxisMode::AB => "A/B (pitch + roll)",
                        crate::gcode::RotaryAxisMode::BC => "B/C (tilt + rotate)",
                    })
                    .show_ui(ui, |ui| {
                        if ui.selectable_label(
                            profile.rotary_axis_mode == crate::gcode::RotaryAxisMode::AB,
                            "A/B (pitch + roll)",
                        ).clicked() {
                            profile.rotary_axis_mode = crate::gcode::RotaryAxisMode::AB;
                            changed = true;
                        }
                        if ui.selectable_label(
                            profile.rotary_axis_mode == crate::gcode::RotaryAxisMode::BC,
                            "B/C (tilt + rotate)",
                        ).clicked() {
                            profile.rotary_axis_mode = crate::gcode::RotaryAxisMode::BC;
                            changed = true;
                        }
                    });
            });

            ui.horizontal(|ui| {
                ui.label("TCP offset (pivot → nozzle):");
                if ui.add(
                    egui::DragValue::new(&mut profile.tcp_offset)
                        .range(0.0..=500.0)
                        .speed(0.5)
                        .suffix(" mm"),
                ).changed() {
                    changed = true;
                }
            });
            ui.label(
                egui::RichText::new("Set to 0 to disable TCP compensation.")
                    .weak()
                    .small(),
            );
        });

        ui.add_space(4.0);

        // ── Toolhead ──────────────────────────────────────────────────────
        ui.collapsing("Toolhead", |ui| {
            ui.horizontal(|ui| {
                ui.label("Nozzle diameter:");
                if ui.add(
                    egui::DragValue::new(&mut profile.nozzle_diameter)
                        .range(0.1..=2.0)
                        .speed(0.01)
                        .suffix(" mm"),
                ).changed() {
                    changed = true;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Max feedrate:");
                if ui.add(
                    egui::DragValue::new(&mut profile.max_feedrate)
                        .range(1.0..=1000.0)
                        .speed(1.0)
                        .suffix(" mm/s"),
                ).changed() {
                    changed = true;
                }
            });

            if ui.checkbox(&mut profile.has_heated_bed, "Heated bed").changed() {
                changed = true;
            }
        });

        ui.add_space(4.0);

        // ── Machine Simulation ────────────────────────────────────────────
        ui.collapsing("Machine Simulation", |ui| {
            ui.label(
                egui::RichText::new("Simple geometric shapes shown in the 3D viewport. \
                    Segments where the head and bed overlap are highlighted red.")
                    .weak()
                    .small(),
            );
            ui.add_space(4.0);

            if ui.checkbox(&mut profile.show_machine, "Show machine in viewport").changed() {
                changed = true;
            }

            ui.add_space(6.0);

            // Bed
            ui.label(egui::RichText::new("Bed").strong());
            ui.horizontal(|ui| {
                ui.label("Shape:");
                let shape_label = match profile.bed_shape {
                    BedShape::Rectangle => "Rectangle",
                    BedShape::Circle    => "Circle",
                };
                egui::ComboBox::from_id_source("bed_shape_selector")
                    .selected_text(shape_label)
                    .show_ui(ui, |ui| {
                        if ui.selectable_label(profile.bed_shape == BedShape::Rectangle, "Rectangle").clicked() {
                            profile.bed_shape = BedShape::Rectangle; changed = true;
                        }
                        if ui.selectable_label(profile.bed_shape == BedShape::Circle, "Circle").clicked() {
                            profile.bed_shape = BedShape::Circle; changed = true;
                        }
                    });
            });
            ui.horizontal(|ui| {
                match profile.bed_shape {
                    BedShape::Rectangle => {
                        ui.label("Size:");
                        if ui.add(egui::DragValue::new(&mut profile.bed_dims[0]).suffix(" mm").speed(1.0).prefix("W ")).changed() { changed = true; }
                        if ui.add(egui::DragValue::new(&mut profile.bed_dims[1]).suffix(" mm").speed(1.0).prefix("D ")).changed() { changed = true; }
                        if ui.add(egui::DragValue::new(&mut profile.bed_dims[2]).suffix(" mm").speed(1.0).prefix("H ")).changed() { changed = true; }
                    }
                    BedShape::Circle => {
                        ui.label("Diameter:");
                        if ui.add(egui::DragValue::new(&mut profile.bed_dims[0]).suffix(" mm").speed(1.0).prefix("Ø ")).changed() {
                            profile.bed_dims[1] = profile.bed_dims[0]; // keep diameter square
                            changed = true;
                        }
                        ui.label("H:");
                        if ui.add(egui::DragValue::new(&mut profile.bed_dims[2]).suffix(" mm").speed(1.0)).changed() { changed = true; }
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.label("Pivot:");
                if ui.add(egui::DragValue::new(&mut profile.bed_pivot[0]).suffix(" mm").speed(0.5).prefix("X ")).changed() { changed = true; }
                if ui.add(egui::DragValue::new(&mut profile.bed_pivot[1]).suffix(" mm").speed(0.5).prefix("Y ")).changed() { changed = true; }
                if ui.add(egui::DragValue::new(&mut profile.bed_pivot[2]).suffix(" mm").speed(0.5).prefix("Z ")).changed() { changed = true; }
            });
            ui.horizontal(|ui| {
                if ui.button("Load Bed STL...").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STL", &["stl"])
                        .pick_file()
                    {
                        profile.bed_stl_path = Some(path.to_string_lossy().into_owned());
                        changed = true;
                    }
                }
                match &profile.bed_stl_path {
                    Some(p) => {
                        let short = std::path::Path::new(p)
                            .file_name()
                            .map(|n| n.to_string_lossy().into_owned())
                            .unwrap_or_else(|| p.clone());
                        ui.label(&short);
                        if ui.small_button("✕").clicked() {
                            profile.bed_stl_path = None;
                            changed = true;
                        }
                    }
                    None => { ui.label(egui::RichText::new("none").weak()); }
                }
            });

            ui.add_space(6.0);

            // Head body
            ui.label(egui::RichText::new("Head body").strong());
            ui.horizontal(|ui| {
                ui.label("Size:");
                if ui.add(egui::DragValue::new(&mut profile.head_dims[0]).suffix(" mm").speed(1.0).prefix("W ")).changed() { changed = true; }
                if ui.add(egui::DragValue::new(&mut profile.head_dims[1]).suffix(" mm").speed(1.0).prefix("D ")).changed() { changed = true; }
                if ui.add(egui::DragValue::new(&mut profile.head_dims[2]).suffix(" mm").speed(1.0).prefix("H ")).changed() { changed = true; }
            });
            ui.horizontal(|ui| {
                ui.label("Nozzle radius:");
                if ui.add(
                    egui::DragValue::new(&mut profile.nozzle_radius)
                        .range(0.5..=20.0)
                        .speed(0.1)
                        .suffix(" mm"),
                ).changed() { changed = true; }
            });
            ui.horizontal(|ui| {
                ui.label("Nozzle length:")
                    .on_hover_text("Visual length of the nozzle below the head carriage (mm). \
                        Set this to match the real distance from the carriage bottom to the nozzle tip.");
                if ui.add(
                    egui::DragValue::new(&mut profile.nozzle_length)
                        .range(1.0..=200.0)
                        .speed(0.5)
                        .suffix(" mm"),
                ).changed() { changed = true; }
            });
            ui.horizontal(|ui| {
                if ui.button("Load Head STL...").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("STL", &["stl"])
                        .pick_file()
                    {
                        profile.head_stl_path = Some(path.to_string_lossy().into_owned());
                        changed = true;
                    }
                }
                match &profile.head_stl_path {
                    Some(p) => {
                        let short = std::path::Path::new(p)
                            .file_name()
                            .map(|n| n.to_string_lossy().into_owned())
                            .unwrap_or_else(|| p.clone());
                        ui.label(&short);
                        if ui.small_button("✕").clicked() {
                            profile.head_stl_path = None;
                            changed = true;
                        }
                    }
                    None => { ui.label(egui::RichText::new("none").weak()); }
                }
            });

            // STL tip offset — only shown when an STL is loaded
            if profile.head_stl_path.is_some() {
                ui.horizontal(|ui| {
                    ui.label("STL tip offset:")
                        .on_hover_text("Position of the nozzle tip inside the STL file's local coordinate system (mm).\n\
                            The simulation pins this point to the actual nozzle contact position.\n\
                            Default (0,0,0) means the STL origin is the nozzle tip.");
                    if ui.add(egui::DragValue::new(&mut profile.head_stl_tip_offset[0])
                        .prefix("X ").suffix(" mm").speed(0.5)).changed() { changed = true; }
                    if ui.add(egui::DragValue::new(&mut profile.head_stl_tip_offset[1])
                        .prefix("Y ").suffix(" mm").speed(0.5)).changed() { changed = true; }
                    if ui.add(egui::DragValue::new(&mut profile.head_stl_tip_offset[2])
                        .prefix("Z ").suffix(" mm").speed(0.5)).changed() { changed = true; }
                    if ui.small_button("Reset").clicked() {
                        profile.head_stl_tip_offset = [0.0, 0.0, 0.0];
                        changed = true;
                    }
                });
            }
        });

    }); // end ScrollArea

    changed
}
