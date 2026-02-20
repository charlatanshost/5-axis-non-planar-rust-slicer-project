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
                            crate::gui::app::SlicingMode::S4 => "S4 Non-Planar",
                            crate::gui::app::SlicingMode::Conical => "Conical (RotBot)",
                            crate::gui::app::SlicingMode::Curved => "S3 Curved Layer",
                            crate::gui::app::SlicingMode::Geodesic => "Geodesic (Heat Method)",
                            crate::gui::app::SlicingMode::CoordTransformCylindrical => "Cylindrical",
                            crate::gui::app::SlicingMode::CoordTransformSpherical => "Spherical",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::Planar, "Planar (Traditional)")
                                .on_hover_text("Standard planar slicing — flat layers at constant Z height");
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::Conical, "Conical (RotBot)")
                                .on_hover_text("Conical slicing: Shift Z by r*tan(angle) → Planar slice → Reverse shift.\nSimple & fast, great for radially symmetric overhangs.");
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::S4, "S4 Non-Planar")
                                .on_hover_text("S4-style: Deform mesh → Planar slice → Un-deform toolpaths.\nSimpler approach, uses Dijkstra distance field for rotation direction.");
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::Curved, "S3 Curved Layer")
                                .on_hover_text("S3-Slicer: Quaternion field optimization → Deformation → Curved layers.\nMore complex, multiple deformation methods available.");
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::Geodesic, "Geodesic (Heat Method)")
                                .on_hover_text("Geodesic slicing: layers follow surface curvature via Heat Method distance field.\nLayers are equidistant along the mesh surface, not through air.");
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::CoordTransformCylindrical, "Cylindrical")
                                .on_hover_text("Cylindrical slicing: maps mesh to (theta, z, r) space.\nPlanar slices become concentric radial shells around the Z axis.");
                            ui.selectable_value(&mut app.slicing_mode, crate::gui::app::SlicingMode::CoordTransformSpherical, "Spherical")
                                .on_hover_text("Spherical slicing: maps mesh to (theta, phi, r) space.\nPlanar slices become concentric spherical shells around a center point.");
                        });
                });

                ui.add_space(5.0);

                // S4 Configuration (only shown in S4 mode)
                if app.slicing_mode == crate::gui::app::SlicingMode::S4 {
                    ui.collapsing("S4 Non-Planar Configuration", |ui| {
                        ui.label(egui::RichText::new("Pipeline: Deform → Slice → Un-Deform").weak());
                        ui.add_space(3.0);

                        ui.horizontal(|ui| {
                            ui.label("Overhang threshold:");
                            ui.add(egui::Slider::new(&mut app.s4_overhang_threshold, 30.0..=60.0)
                                .suffix("°"))
                                .on_hover_text("Surface faces steeper than this angle get rotated to reduce overhangs");
                        });

                        ui.horizontal(|ui| {
                            ui.label("Max rotation:");
                            ui.add(egui::Slider::new(&mut app.s4_max_rotation_degrees, 5.0..=45.0)
                                .suffix("°")
                                .step_by(1.0))
                                .on_hover_text("Maximum rotation per tet to fix overhangs.\nLower = gentler deformation, Higher = more aggressive.");
                        });

                        ui.separator();
                        ui.label(egui::RichText::new("Smoothing").weak());

                        ui.horizontal(|ui| {
                            ui.label("Smoothing iterations:");
                            ui.add(egui::Slider::new(&mut app.s4_smoothing_iterations, 5..=50))
                                .on_hover_text("SLERP smoothing passes over the rotation field.\nMore = smoother transitions between tets (25 recommended).");
                        });

                        ui.horizontal(|ui| {
                            ui.label("Smoothness weight:");
                            ui.add(egui::Slider::new(&mut app.s4_smoothness_weight, 0.0..=1.0))
                                .on_hover_text("How much to blend with neighbors per iteration.\n0.0 = no smoothing, 1.0 = heavy smoothing.");
                        });

                        ui.separator();
                        ui.label(egui::RichText::new("ASAP Deformation").weak());

                        ui.horizontal(|ui| {
                            ui.label("ASAP iterations:");
                            ui.add(egui::Slider::new(&mut app.s4_asap_max_iterations, 3..=20))
                                .on_hover_text("Iterations for the As-Rigid-As-Possible deformation solver.\nMore = better shape preservation (10 recommended).");
                        });

                        ui.horizontal(|ui| {
                            ui.label("Convergence:");
                            ui.add(egui::Slider::new(&mut app.s4_asap_convergence, 1e-5..=1e-3)
                                .logarithmic(true)
                                .custom_formatter(|n, _| format!("{:.1e}", n)))
                                .on_hover_text("ASAP solver convergence threshold (lower = more precise)");
                        });
                    });

                    ui.add_space(5.0);
                }

                // Conical Configuration (only shown in Conical mode)
                if app.slicing_mode == crate::gui::app::SlicingMode::Conical {
                    ui.collapsing("Conical Slicing Configuration", |ui| {
                        ui.label(egui::RichText::new("Pipeline: Z-shift → Planar slice → Reverse shift").weak());
                        ui.add_space(3.0);

                        ui.horizontal(|ui| {
                            ui.label("Cone angle:");
                            ui.add(egui::Slider::new(&mut app.conical_angle_degrees, 5.0..=60.0)
                                .suffix("°")
                                .step_by(1.0))
                                .on_hover_text("Half-angle of the slicing cone.\n45° is most common. Lower = gentler cones, higher = steeper.");
                        });

                        ui.horizontal(|ui| {
                            ui.label("Direction:");
                            use crate::conical::ConicalDirection;
                            egui::ComboBox::from_id_salt("conical_direction")
                                .selected_text(match app.conical_direction {
                                    ConicalDirection::Outward => "Outward",
                                    ConicalDirection::Inward => "Inward",
                                })
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(&mut app.conical_direction, ConicalDirection::Outward, "Outward")
                                        .on_hover_text("Layers cone outward — outer regions pushed down.\nBest for overhangs radiating away from center.");
                                    ui.selectable_value(&mut app.conical_direction, ConicalDirection::Inward, "Inward")
                                        .on_hover_text("Layers cone inward — outer regions pushed up.\nBest for central peaks or spires.");
                                });
                        });

                        ui.separator();
                        ui.label(egui::RichText::new("Cone Center").weak());

                        ui.checkbox(&mut app.conical_auto_center, "Auto-center on mesh")
                            .on_hover_text("Automatically use the mesh centroid as cone center");

                        if !app.conical_auto_center {
                            ui.horizontal(|ui| {
                                ui.label("Center X:");
                                ui.add(egui::DragValue::new(&mut app.conical_center_x).speed(0.5).suffix(" mm"));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Center Y:");
                                ui.add(egui::DragValue::new(&mut app.conical_center_y).speed(0.5).suffix(" mm"));
                            });
                        }
                    });

                    ui.add_space(5.0);
                }

                // Geodesic Configuration (only shown in Geodesic mode)
                if app.slicing_mode == crate::gui::app::SlicingMode::Geodesic {
                    ui.collapsing("Geodesic Slicing Configuration", |ui| {
                        ui.label(egui::RichText::new("Pipeline: Heat Method → Distance Field → Level Sets").weak());
                        ui.add_space(3.0);

                        ui.horizontal(|ui| {
                            ui.label("Source mode:");
                            egui::ComboBox::from_id_salt("geodesic_source")
                                .selected_text(if app.geodesic_source_mode == 0 { "Bottom Boundary" } else { "Point Source" })
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(&mut app.geodesic_source_mode, 0, "Bottom Boundary")
                                        .on_hover_text("Layers grow from the bottom of the mesh, conforming to surface curvature.");
                                    ui.selectable_value(&mut app.geodesic_source_mode, 1, "Point Source")
                                        .on_hover_text("Layers radiate outward from a specified point on the mesh.");
                                });
                        });

                        if app.geodesic_source_mode == 0 {
                            ui.horizontal(|ui| {
                                ui.label("Bottom tolerance:");
                                ui.add(egui::Slider::new(&mut app.geodesic_bottom_tolerance, 0.01..=5.0)
                                    .suffix(" mm")
                                    .step_by(0.1))
                                    .on_hover_text("Z tolerance for detecting bottom boundary vertices.\nLarger = more source vertices at the base.");
                            });
                        } else {
                            ui.horizontal(|ui| {
                                ui.label("Source X:");
                                ui.add(egui::DragValue::new(&mut app.geodesic_source_point[0]).speed(0.5).suffix(" mm"));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Source Y:");
                                ui.add(egui::DragValue::new(&mut app.geodesic_source_point[1]).speed(0.5).suffix(" mm"));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Source Z:");
                                ui.add(egui::DragValue::new(&mut app.geodesic_source_point[2]).speed(0.5).suffix(" mm"));
                            });
                        }

                        ui.separator();
                        ui.label(egui::RichText::new("Advanced").weak());

                        ui.horizontal(|ui| {
                            ui.checkbox(&mut app.geodesic_use_multiscale, "Multi-scale")
                                .on_hover_text("Run several heat solves at doubling timesteps and fuse per-vertex.\nGives fine detail in thin features AND full-mesh coverage simultaneously.\nRecommended: ON.");
                        });

                        if app.geodesic_use_multiscale {
                            ui.horizontal(|ui| {
                                ui.label("  Base factor:");
                                ui.add(egui::Slider::new(&mut app.geodesic_heat_factor, 0.1..=20.0)
                                    .step_by(0.5))
                                    .on_hover_text("Finest (most detailed) heat timestep = factor × avg_edge².\nMulti-scale adds larger scales automatically for full coverage.\nSmaller = more local detail in thin features (feet, ears).");
                            });
                            ui.horizontal(|ui| {
                                ui.label("  Scales:");
                                ui.add(egui::Slider::new(&mut app.geodesic_num_scales, 3..=8)
                                    .step_by(1.0))
                                    .on_hover_text("Number of doubling timestep scales.\n6 covers factor × [1,2,4,8,16,32] — enough for most meshes.\nIncrease if the mesh is very tall relative to its fine features.");
                            });
                        } else {
                            ui.horizontal(|ui| {
                                ui.label("Heat timestep:");
                                ui.add(egui::Slider::new(&mut app.geodesic_heat_factor, 0.1..=100.0)
                                    .step_by(0.5))
                                    .on_hover_text("Multiplier on avg_edge_length² for heat diffusion timestep.\nLarger = smoother but less accurate. Needs ~15+ for full mesh coverage on most objects.");
                            });
                        }

                        ui.horizontal(|ui| {
                            ui.checkbox(&mut app.geodesic_use_adaptive, "Adaptive κ")
                                .on_hover_text("Variable-diffusivity heat method (Grok/Crane adaptive extension).\nPer-face κ = kappa_base × (avg_edge)².\nSmall triangles (thin features, toes) → small κ → slow diffusion → sharp contours.\nLarge triangles (body, ears) → large κ → fast spread → full coverage.\nCombines well with multi-scale for best results.");
                        });

                        if app.geodesic_use_adaptive {
                            ui.horizontal(|ui| {
                                ui.label("  κ base:");
                                ui.add(egui::Slider::new(&mut app.geodesic_adaptive_kappa_base, 0.5..=20.0)
                                    .step_by(0.5))
                                    .on_hover_text("Scaling factor for per-face diffusivity κ.\nκ(face) = base × (avg_edge_length)², clamped to [0.05, 50].\nTypical range: 4–8. Higher = more aggressive spread in large regions.\nTry 6.0 as a starting point.");
                            });
                        }
                    });

                    ui.add_space(5.0);
                }

                // Cylindrical / Spherical Configuration
                let is_coord_transform = matches!(
                    app.slicing_mode,
                    crate::gui::app::SlicingMode::CoordTransformCylindrical
                    | crate::gui::app::SlicingMode::CoordTransformSpherical
                );
                if is_coord_transform {
                    let title = if app.slicing_mode == crate::gui::app::SlicingMode::CoordTransformCylindrical {
                        "Cylindrical Slicing Configuration"
                    } else {
                        "Spherical Slicing Configuration"
                    };
                    ui.collapsing(title, |ui| {
                        if app.slicing_mode == crate::gui::app::SlicingMode::CoordTransformCylindrical {
                            ui.label(egui::RichText::new("Pipeline: (x,y,z) → (θ,z,r) → Planar slice → Inverse").weak());
                            ui.label(egui::RichText::new("⚠ Branch-cut artifact possible at θ = ±180° for thin regions").italics().weak());
                        } else {
                            ui.label(egui::RichText::new("Pipeline: (x,y,z) → (θ,φ,r) → Planar slice → Inverse").weak());
                            ui.label(egui::RichText::new("⚠ Branch-cut artifact possible at θ = ±180° for thin regions").italics().weak());
                        }
                        ui.add_space(3.0);

                        ui.separator();
                        ui.label(egui::RichText::new("Transform Center").weak());

                        ui.checkbox(&mut app.coord_transform_auto_center, "Auto-center on mesh")
                            .on_hover_text("Automatically use the mesh centroid as the transform center");

                        if !app.coord_transform_auto_center {
                            ui.horizontal(|ui| {
                                ui.label("Center X:");
                                ui.add(egui::DragValue::new(&mut app.coord_transform_center_x).speed(0.5).suffix(" mm"));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Center Y:");
                                ui.add(egui::DragValue::new(&mut app.coord_transform_center_y).speed(0.5).suffix(" mm"));
                            });
                            if app.slicing_mode == crate::gui::app::SlicingMode::CoordTransformSpherical {
                                ui.horizontal(|ui| {
                                    ui.label("Center Z:");
                                    ui.add(egui::DragValue::new(&mut app.coord_transform_center_z).speed(0.5).suffix(" mm"));
                                });
                            }
                        }
                    });

                    ui.add_space(5.0);
                }

                // S3-Slicer Configuration (only shown in Curved/S3 mode)
                if app.slicing_mode == crate::gui::app::SlicingMode::Curved {
                    ui.collapsing("S3 Curved Layer Configuration", |ui| {
                        ui.label(egui::RichText::new("Quaternion Field Parameters").weak());

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
                        ui.label(egui::RichText::new("Deformation Method").weak());

                        // Deformation method combo box (S4Deform removed - use S4 mode instead)
                        egui::ComboBox::from_label("")
                            .selected_text(match app.s3_deformation_method {
                                crate::s3_slicer::DeformationMethod::TetVolumetric => "Tet Volumetric (Best)",
                                crate::s3_slicer::DeformationMethod::VirtualScalarField => "Virtual",
                                crate::s3_slicer::DeformationMethod::AsapDeformation => "ASAP Deformation",
                                crate::s3_slicer::DeformationMethod::ScaleControlled => "Scale-Controlled",
                                crate::s3_slicer::DeformationMethod::S4Deform => "S4 (use S4 mode)",
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut app.s3_deformation_method,
                                    crate::s3_slicer::DeformationMethod::TetVolumetric,
                                    "Tet Volumetric (Best)"
                                ).on_hover_text("Full volumetric pipeline from the original S3-Slicer paper.\nUses tetrahedral mesh, per-tet ASAP deformation with scaling,\nand marching tetrahedra for layer extraction.");

                                ui.selectable_value(
                                    &mut app.s3_deformation_method,
                                    crate::s3_slicer::DeformationMethod::VirtualScalarField,
                                    "Virtual"
                                ).on_hover_text("Computes scalar field directly without mesh deformation.\nAvoids mesh collapse issues. Good for complex models.");

                                ui.selectable_value(
                                    &mut app.s3_deformation_method,
                                    crate::s3_slicer::DeformationMethod::AsapDeformation,
                                    "ASAP Deformation"
                                ).on_hover_text("Full mesh deformation using ASAP solver.\n⚠ Can cause mesh collapse on complex models!");

                                ui.selectable_value(
                                    &mut app.s3_deformation_method,
                                    crate::s3_slicer::DeformationMethod::ScaleControlled,
                                    "Scale-Controlled"
                                ).on_hover_text("Per-vertex local deformation. Faster but less accurate.");
                            });

                        // Show ASAP settings only when ASAP is selected
                        if matches!(app.s3_deformation_method, crate::s3_slicer::DeformationMethod::AsapDeformation) {
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

                                ui.label(egui::RichText::new("⚠ ASAP can cause mesh collapse on complex models!")
                                    .small()
                                    .color(egui::Color32::from_rgb(255, 100, 100)));
                            });
                        }

                        // Show hints for each method
                        if matches!(app.s3_deformation_method, crate::s3_slicer::DeformationMethod::TetVolumetric) {
                            ui.label(egui::RichText::new("Tet Volumetric: Full volume mesh pipeline (original paper algorithm)")
                                .small()
                                .color(egui::Color32::from_rgb(100, 200, 255)));
                        }

                        if matches!(app.s3_deformation_method, crate::s3_slicer::DeformationMethod::VirtualScalarField) {
                            ui.label(egui::RichText::new("Virtual mode: No mesh deformation preview, but slicing works correctly")
                                .small()
                                .color(egui::Color32::from_rgb(100, 200, 100)));
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

                // S3-Slicer specific parameter (only shown/used in S3 Curved mode — S4 has its own)
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

                // Wall loops (shown for Contour pattern)
                if app.toolpath_pattern == crate::toolpath_patterns::ToolpathPattern::Contour {
                    ui.horizontal(|ui| {
                        ui.label("Wall loops:");
                        let mut wall_count_f = app.wall_count as f64;
                        if ui.add(egui::Slider::new(&mut wall_count_f, 1.0..=10.0)
                            .step_by(1.0))
                            .changed() {
                            app.wall_count = wall_count_f as usize;
                        }
                    });
                }

                // Infill density (always visible)
                ui.horizontal(|ui| {
                    ui.label("Infill density:");
                    ui.add(egui::Slider::new(&mut app.toolpath_infill_density, 0.0..=1.0)
                        .suffix("%")
                        .custom_formatter(|v, _| format!("{:.0}", v * 100.0)))
                        .on_hover_text("0% = no infill (hollow), 100% = solid fill");
                });

                // Infill pattern selector (shown when Contour pattern and density > 0)
                if app.toolpath_pattern == crate::toolpath_patterns::ToolpathPattern::Contour
                    && app.toolpath_infill_density > 0.01
                {
                    ui.horizontal(|ui| {
                        ui.label("Infill pattern:");
                        use crate::toolpath_patterns::InfillPattern;
                        egui::ComboBox::from_id_salt("infill_pattern")
                            .selected_text(match app.infill_pattern {
                                InfillPattern::Rectilinear => "Rectilinear",
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut app.infill_pattern, InfillPattern::Rectilinear, "Rectilinear")
                                    .on_hover_text("Alternating-direction horizontal lines");
                            });
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

                // Section view controls
                ui.checkbox(&mut app.section_enabled, "Section view");
                if app.section_enabled {
                    ui.horizontal(|ui| {
                        ui.label("Axis:");
                        ui.selectable_value(&mut app.section_axis, 0, "X");
                        ui.selectable_value(&mut app.section_axis, 1, "Y");
                        ui.selectable_value(&mut app.section_axis, 2, "Z");
                    });
                    ui.add(egui::Slider::new(&mut app.section_depth, 0.0..=1.0)
                        .text("Depth"));
                }

                ui.checkbox(&mut app.show_travel_moves, "Show travel moves")
                    .on_hover_text("Toggle visibility of rapid travel moves (cyan lines) in toolpath view");

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
                let needs_deform_preview = app.slicing_mode == crate::gui::app::SlicingMode::Curved
                    || app.slicing_mode == crate::gui::app::SlicingMode::S4;
                let can_deform = app.mesh.is_some() && !app.is_deforming && needs_deform_preview;

                // Deformation preview (for S3 Curved and S4 modes only — Conical doesn't need it)
                if needs_deform_preview {
                    let button_label = if app.slicing_mode == crate::gui::app::SlicingMode::S4 {
                        "🔄 Preview Deformation (S4)"
                    } else {
                        "🧮 Compute Quaternion Field"
                    };

                    if ui.add_enabled(can_deform, egui::Button::new(button_label).min_size(egui::vec2(ui.available_width(), 30.0))).clicked() {
                        app.deform_mesh_preview();
                    }

                    if app.is_deforming {
                        let msg = if app.slicing_mode == crate::gui::app::SlicingMode::S4 {
                            "⏳ Running S4 deformation preview..."
                        } else {
                            "⏳ Computing quaternion field..."
                        };
                        ui.label(egui::RichText::new(msg).italics());
                    }

                    if app.deformed_mesh.is_some() {
                        let msg = if app.slicing_mode == crate::gui::app::SlicingMode::S4 {
                            "✓ S4 deformation preview ready"
                        } else {
                            "✓ Quaternion field ready"
                        };
                        ui.label(egui::RichText::new(msg).color(egui::Color32::from_rgb(100, 200, 100)));
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

                // 5-axis G-code settings (TCP compensation + rotary axis labels)
                ui.collapsing("5-Axis G-code Settings", |ui| {
                    ui.label(egui::RichText::new("Rotary axis label format:").weak());
                    use crate::gcode::RotaryAxisMode;
                    egui::ComboBox::from_id_salt("rotary_axis_mode")
                        .selected_text(match app.rotary_axis_mode {
                            RotaryAxisMode::AB => "A/B (pitch + roll)",
                            RotaryAxisMode::BC => "B/C (tilt + rotate)",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut app.rotary_axis_mode, RotaryAxisMode::AB, "A/B (pitch + roll)")
                                .on_hover_text("A = rotation around X (pitch), B = rotation around Y (roll).\nDefault for most 5-axis head machines.");
                            ui.selectable_value(&mut app.rotary_axis_mode, RotaryAxisMode::BC, "B/C (tilt + rotate)")
                                .on_hover_text("B = tilt, C = rotate.\nCommon on table-tilt / table-rotate machine configurations.");
                        });

                    ui.add_space(4.0);
                    ui.label(egui::RichText::new("TCP offset (pivot → nozzle tip):").weak());
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut app.tcp_offset)
                            .speed(0.5)
                            .range(0.0..=500.0)
                            .suffix(" mm"));
                        if app.tcp_offset < 1e-6 {
                            ui.label(egui::RichText::new("(disabled)").weak());
                        } else {
                            ui.label(egui::RichText::new("✓ TCP active").color(egui::Color32::from_rgb(100, 200, 255)));
                        }
                    });
                    ui.label(egui::RichText::new("Set to 0 to disable. Shifts XYZ so nozzle tip lands at target when axes are tilted.").italics().weak());
                });

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
