// Printer Profiles page — browser-tab-style editor for PrinterProfile entries.
// Renders inside CentralPanel when AppTab::PrinterProfiles is active.

use crate::gui::app::{SlicerApp, AppTab, PrinterProfile, AxisConfig, AxisType, MovingPart, KinematicsType, BedShape, RotationAxis};

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

                    // Rotation axis (only for rotary joints)
                    if ax.axis_type == AxisType::Rotary {
                        egui::ComboBox::from_id_source(format!("axis_rot_{}", i))
                            .selected_text(format!("↺{}", ax.rotation_axis.label()))
                            .width(52.0)
                            .show_ui(ui, |ui| {
                                for ra in RotationAxis::all() {
                                    if ui.selectable_label(&ax.rotation_axis == ra, ra.label()).clicked() {
                                        ax.rotation_axis = ra.clone();
                                        changed = true;
                                    }
                                }
                            });
                    }

                    // Min / Max
                    let suffix = if ax.axis_type == AxisType::Rotary { " °" } else { " mm" };
                    if ui.add(egui::DragValue::new(&mut ax.min).suffix(suffix).speed(1.0)).changed() { changed = true; }
                    ui.label("to");
                    if ui.add(egui::DragValue::new(&mut ax.max).suffix(suffix).speed(1.0)).changed() { changed = true; }

                    // Max angular speed (rotary only)
                    if ax.axis_type == AxisType::Rotary {
                        ui.label("≤");
                        if ui.add(egui::DragValue::new(&mut ax.max_angular_speed)
                            .suffix(" °/s")
                            .range(0.0..=3600.0)
                            .speed(1.0))
                            .on_hover_text("Maximum angular speed (deg/s). G-code feedrate is capped so this axis never moves faster than this.")
                            .changed() { changed = true; }
                    }

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
                ui.label("Linear travel (0 = fixed):");
                if ui.add(egui::DragValue::new(&mut profile.bed_travel[0]).suffix(" mm").speed(1.0).prefix("X ")).changed() { changed = true; }
                if ui.add(egui::DragValue::new(&mut profile.bed_travel[1]).suffix(" mm").speed(1.0).prefix("Y ")).changed() { changed = true; }
                if ui.add(egui::DragValue::new(&mut profile.bed_travel[2]).suffix(" mm").speed(1.0).prefix("Z ")).changed() { changed = true; }
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
                ui.label("Linear travel (0 = fixed):");
                if ui.add(egui::DragValue::new(&mut profile.head_travel[0]).suffix(" mm").speed(1.0).prefix("X ")).changed() { changed = true; }
                if ui.add(egui::DragValue::new(&mut profile.head_travel[1]).suffix(" mm").speed(1.0).prefix("Y ")).changed() { changed = true; }
                if ui.add(egui::DragValue::new(&mut profile.head_travel[2]).suffix(" mm").speed(1.0).prefix("Z ")).changed() { changed = true; }
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

        ui.add_space(4.0);

        // ── Head Geometry Preview ─────────────────────────────────────────
        ui.collapsing("Head Geometry Preview", |ui| {
            ui.label(
                egui::RichText::new(
                    "Side-view diagram: head body, nozzle, TCP pivot, and tilt sweep \
                     from the rotary axes defined above.",
                )
                .weak()
                .small(),
            );
            ui.add_space(4.0);
            draw_head_diagram(ui, profile);
        });

    }); // end ScrollArea

    changed
}

/// Cached set of XZ-projected edge pairs for the side-view STL wireframe.
/// Each entry is [[x1, z1], [x2, z2]] in tip-offset-corrected coordinates.
type StlEdgeCache = Vec<[[f64; 2]; 2]>;

/// Load an STL, translate so the nozzle tip is at the origin, and project all
/// triangle edges onto the XZ plane (X = left/right, Z = up).  At most ~700
/// triangles are kept so the diagram stays fast to render.
fn load_head_stl_edges(path: &str, tip_offset: [f64; 3]) -> StlEdgeCache {
    use crate::Mesh;
    let mesh = match Mesh::from_stl(std::path::Path::new(path)) {
        Ok(m) => m,
        Err(_) => return Vec::new(),
    };
    let step = (mesh.triangles.len() / 700).max(1);
    let mut edges = Vec::with_capacity(mesh.triangles.len().min(700) * 3);
    for tri in mesh.triangles.iter().step_by(step) {
        let verts = [tri.v0, tri.v1, tri.v2];
        for k in 0..3 {
            let a = verts[k];
            let b = verts[(k + 1) % 3];
            edges.push([
                [a.x - tip_offset[0], a.z - tip_offset[2]],
                [b.x - tip_offset[0], b.z - tip_offset[2]],
            ]);
        }
    }
    edges
}

/// 2-D side-view diagram of the tool head geometry.
/// Shows the head body (box or STL wireframe), nozzle, TCP pivot point, and the
/// arc the nozzle tip sweeps when the machine tilts through its full rotary range.
fn draw_head_diagram(ui: &mut egui::Ui, profile: &PrinterProfile) {
    let canvas_size = egui::vec2(280.0, 360.0);
    let (response, painter) = ui.allocate_painter(canvas_size, egui::Sense::hover());
    let rect = response.rect;
    painter.rect_filled(rect, 4.0, egui::Color32::from_gray(28));

    // ── Profile values ────────────────────────────────────────────────────
    let tcp      = profile.tcp_offset.max(1.0);
    let head_w   = profile.head_dims[0].max(profile.head_dims[1]).max(1.0);
    let head_h   = profile.head_dims[2].max(1.0);
    let nozzle_l = profile.nozzle_length.max(1.0);
    let nozzle_r = profile.nozzle_radius.max(0.5);

    let max_tilt_deg: f64 = {
        let t = profile.rotary_axes.iter()
            .filter(|ax| ax.axis_type == AxisType::Rotary)
            .map(|ax| ax.max.abs().max(ax.min.abs()))
            .fold(0.0_f64, f64::max);
        if t < 1.0 { 45.0 } else { t.min(89.0) }
    };
    let max_tilt_rad = max_tilt_deg.to_radians();

    // ── Load / cache STL wireframe ────────────────────────────────────────
    // Cache key includes the path and tip offset so any change triggers a reload.
    let stl_edges: Option<StlEdgeCache> =
        if let Some(path) = profile.head_stl_path.as_deref() {
            let tip = profile.head_stl_tip_offset;
            let cache_id = egui::Id::new("head_stl_edges")
                .with(path)
                .with(format!("{:.3},{:.3},{:.3}", tip[0], tip[1], tip[2]));
            let cached = ui.ctx().data(|d| d.get_temp::<StlEdgeCache>(cache_id));
            Some(if let Some(edges) = cached {
                edges
            } else {
                let edges = load_head_stl_edges(path, tip);
                ui.ctx().data_mut(|d| d.insert_temp(cache_id, edges.clone()));
                edges
            })
        } else {
            None
        };

    // ── Bounding box for world-space scaling ──────────────────────────────
    // When an STL is loaded use its actual extents; otherwise use head_dims.
    let (world_y_top, world_y_bot, world_x_ext) =
        if let Some(ref edges) = stl_edges {
            let (mut xlo, mut xhi, mut zlo, mut zhi) =
                (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for e in edges {
                for pt in e {
                    xlo = xlo.min(pt[0]); xhi = xhi.max(pt[0]);
                    zlo = zlo.min(pt[1]); zhi = zhi.max(pt[1]);
                }
            }
            let y_top = (zhi + 8.0).max(tcp + 8.0) as f32;
            let y_bot = (zlo - 4.0).min(-8.0) as f32;
            let x_ext = ((xlo.abs().max(xhi.abs()) + 8.0)
                .max(tcp * max_tilt_rad.sin() + 8.0)) as f32;
            (y_top, y_bot, x_ext)
        } else {
            let arc_x  = tcp * max_tilt_rad.sin();
            let y_top  = (nozzle_l + head_h + 8.0).max(tcp + 8.0) as f32;
            let x_ext  = ((head_w / 2.0 + 8.0).max(arc_x + 8.0)) as f32;
            (y_top, -8.0_f32, x_ext)
        };

    // ── Scale ─────────────────────────────────────────────────────────────
    let margin       = 6.0_f32;
    let right_margin = 56.0_f32;
    let draw_h = canvas_size.y - margin * 2.0;
    let draw_w = canvas_size.x - margin * 2.0 - right_margin;
    let scale  = (draw_h / (world_y_top - world_y_bot))
        .min(draw_w / (world_x_ext * 2.0));

    // Screen origin = nozzle tip (world 0, 0)
    let ox = rect.left() + margin + world_x_ext * scale;
    let oy = rect.top()  + margin + world_y_top * scale;

    let w2s = |wx: f64, wy: f64| -> egui::Pos2 {
        egui::pos2(ox + wx as f32 * scale, oy - wy as f32 * scale)
    };

    // ── Colours ───────────────────────────────────────────────────────────
    let head_fill   = egui::Color32::from_rgba_unmultiplied(140, 150, 175, 100);
    let head_stroke = egui::Color32::from_rgb(185, 190, 205);
    let nozzle_col  = egui::Color32::from_rgb(215, 165, 70);
    let pivot_col   = egui::Color32::from_rgb(255, 140, 0);
    let tip_col     = egui::Color32::from_rgb(70, 200, 70);
    let arc_col     = egui::Color32::from_rgb(60, 120, 200);
    let ghost_col   = egui::Color32::from_rgba_unmultiplied(60, 120, 200, 55);
    let faint_arc   = egui::Color32::from_rgba_unmultiplied(60, 120, 200, 80);
    let dim_col     = egui::Color32::from_gray(135);
    let axis_col    = egui::Color32::from_rgba_unmultiplied(255, 255, 255, 35);

    // ── Tilt sweep arc ────────────────────────────────────────────────────
    // When the head tilts by angle t around the fixed pivot, the nozzle traces:
    //   nozzle_world = (tcp·sin t,  tcp·(1 − cos t))
    // (pivot stays at (0, tcp); nozzle moves right for positive tilt)
    let n_arc = 40_usize;
    let arc_pts: Vec<egui::Pos2> = (0..=n_arc).map(|i| {
        let t = -max_tilt_rad + 2.0 * max_tilt_rad * i as f64 / n_arc as f64;
        w2s(tcp * t.sin(), tcp * (1.0 - t.cos()))
    }).collect();

    for i in 0..arc_pts.len().saturating_sub(1) {
        painter.line_segment([arc_pts[i], arc_pts[i + 1]], egui::Stroke::new(1.5, arc_col));
    }
    let pivot_s = w2s(0.0, tcp);
    if let Some(&p0) = arc_pts.first() {
        painter.line_segment([pivot_s, p0], egui::Stroke::new(1.0, faint_arc));
    }
    if let Some(&p1) = arc_pts.last() {
        painter.line_segment([pivot_s, p1], egui::Stroke::new(1.0, faint_arc));
    }

    // ── Ghost head outlines at ±max tilt ─────────────────────────────────
    // Correct kinematics: pivot is fixed at (0, tcp); a point (lx, ly) in
    // tool-frame (lx = right, ly = up from nozzle tip) maps to world via:
    //   tool_X_world = (cos t,  sin t)
    //   tool_Z_world = (−sin t, cos t)   [points toward pivot, i.e. tool "up"]
    //   nozzle_world = (tcp·sin t,  tcp·(1−cos t))
    //   world = nozzle_world + lx·tool_X + ly·tool_Z
    for &tilt in &[-max_tilt_rad, max_tilt_rad] {
        let ct = tilt.cos();
        let st = tilt.sin();
        let tg = |lx: f64, ly: f64| -> egui::Pos2 {
            let wx = tcp * st + lx * ct - ly * st;
            let wy = tcp * (1.0 - ct) + lx * st + ly * ct;
            egui::pos2(ox + wx as f32 * scale, oy - wy as f32 * scale)
        };
        if let Some(ref edges) = stl_edges {
            for e in edges {
                let p0 = tg(e[0][0], e[0][1]);
                let p1 = tg(e[1][0], e[1][1]);
                painter.line_segment([p0, p1], egui::Stroke::new(0.5, ghost_col));
            }
        } else {
            // Parametric box ghost
            let corners = [
                tg(-head_w / 2.0, nozzle_l),
                tg( head_w / 2.0, nozzle_l),
                tg( head_w / 2.0, nozzle_l + head_h),
                tg(-head_w / 2.0, nozzle_l + head_h),
            ];
            for i in 0..4 {
                painter.line_segment(
                    [corners[i], corners[(i + 1) % 4]],
                    egui::Stroke::new(1.0, ghost_col),
                );
            }
        }
        painter.circle_filled(tg(0.0, 0.0), 2.0, ghost_col);
    }

    // ── Main (upright) head ───────────────────────────────────────────────
    if let Some(ref edges) = stl_edges {
        // STL wireframe — XZ projection, tip offset already applied
        for e in edges {
            painter.line_segment(
                [w2s(e[0][0], e[0][1]), w2s(e[1][0], e[1][1])],
                egui::Stroke::new(0.8, head_stroke),
            );
        }
    } else {
        // Parametric head: nozzle trapezoid + body box
        let tip_r = (nozzle_r * 0.25).max(0.25);
        let nv = [
            w2s(-tip_r,    0.0),
            w2s( tip_r,    0.0),
            w2s( nozzle_r, nozzle_l),
            w2s(-nozzle_r, nozzle_l),
        ];
        painter.line_segment([nv[0], nv[1]], egui::Stroke::new(2.0,  nozzle_col));
        painter.line_segment([nv[1], nv[2]], egui::Stroke::new(1.5,  nozzle_col));
        painter.line_segment([nv[2], nv[3]], egui::Stroke::new(1.5,  nozzle_col));
        painter.line_segment([nv[3], nv[0]], egui::Stroke::new(1.5,  nozzle_col));

        let head_rect = egui::Rect::from_two_pos(
            w2s(-head_w / 2.0, nozzle_l),
            w2s( head_w / 2.0, nozzle_l + head_h),
        );
        painter.rect_filled(head_rect, 2.0, head_fill);
        painter.rect_stroke(head_rect, 2.0, egui::Stroke::new(2.0, head_stroke));
    }

    // Faint tool axis line
    painter.line_segment([w2s(0.0, 0.0), pivot_s], egui::Stroke::new(1.0, axis_col));

    // ── TCP pivot (orange) and nozzle tip (green) ─────────────────────────
    painter.circle_filled(pivot_s, 5.5, pivot_col);
    painter.circle_stroke(pivot_s, 5.5, egui::Stroke::new(1.5, egui::Color32::WHITE));
    let tip_s = w2s(0.0, 0.0);
    painter.circle_filled(tip_s, 4.0, tip_col);

    // ── Dimension: TCP offset ─────────────────────────────────────────────
    let dim_x = rect.right() - right_margin + 4.0;
    let d_tip  = egui::pos2(dim_x, oy);
    let d_piv  = egui::pos2(dim_x, oy - tcp as f32 * scale);
    let tick   = 3.5_f32;
    painter.line_segment([d_tip, d_piv], egui::Stroke::new(1.0, dim_col));
    for &py in &[d_tip.y, d_piv.y] {
        painter.line_segment(
            [egui::pos2(dim_x - tick, py), egui::pos2(dim_x + tick, py)],
            egui::Stroke::new(1.0, dim_col),
        );
    }
    painter.text(
        egui::pos2(dim_x + 5.0, (d_tip.y + d_piv.y) / 2.0),
        egui::Align2::LEFT_CENTER,
        format!("TCP\n{:.0}mm", tcp),
        egui::FontId::proportional(9.0),
        dim_col,
    );

    // ── Labels ────────────────────────────────────────────────────────────
    painter.text(
        egui::pos2(pivot_s.x + 8.0, pivot_s.y),
        egui::Align2::LEFT_CENTER,
        "pivot",
        egui::FontId::proportional(10.0),
        pivot_col,
    );
    painter.text(
        egui::pos2(tip_s.x + 6.0, tip_s.y - 1.0),
        egui::Align2::LEFT_CENTER,
        "tip",
        egui::FontId::proportional(10.0),
        tip_col,
    );
    // Source label (STL or parametric)
    let src_label = if stl_edges.is_some() { "STL" } else { "parametric" };
    painter.text(
        egui::pos2(rect.left() + 4.0, rect.bottom() - 4.0),
        egui::Align2::LEFT_BOTTOM,
        src_label,
        egui::FontId::proportional(9.0),
        egui::Color32::from_gray(100),
    );
    // Tilt label
    if let Some(&arc_pt) = arc_pts.get(n_arc * 3 / 4) {
        painter.text(
            egui::pos2(arc_pt.x + 3.0, arc_pt.y),
            egui::Align2::LEFT_CENTER,
            format!("±{:.0}°", max_tilt_deg),
            egui::FontId::proportional(9.0),
            arc_col,
        );
    }
}
