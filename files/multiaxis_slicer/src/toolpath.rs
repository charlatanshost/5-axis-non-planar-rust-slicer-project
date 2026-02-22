use crate::geometry::{Point3D, Vector3D};
use crate::mesh::Mesh;
use crate::slicing::Layer;
use crate::centroidal_axis::CentroidalAxis;
use crate::toolpath_patterns::{ToolpathPattern, ToolpathConfig, generate_spiral_pattern, generate_zigzag_pattern, generate_contour_pattern, generate_clipped_infill, resample_contour, MeshRayCaster, is_point_in_contour};
use serde::{Deserialize, Serialize};

/// Toolpath for a single layer
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Toolpath {
    pub paths: Vec<ToolpathSegment>,
    pub z: f64,
    /// Layer height for this toolpath (used by the viewport to scale tube radius).
    #[serde(default = "default_layer_height")]
    pub layer_height: f64,
}

fn default_layer_height() -> f64 { 0.2 }

/// A segment of a toolpath with position and orientation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolpathSegment {
    pub position: Point3D,
    pub orientation: Vector3D, // Tool orientation (for 5-axis)
    pub extrusion: f64,        // Amount to extrude
    pub feedrate: f64,         // Movement speed
}

pub struct ToolpathGenerator {
    pub nozzle_diameter: f64,
    pub layer_height: f64,
    pub extrusion_width: f64,
    pub pattern_config: ToolpathConfig,
}

impl ToolpathGenerator {
    pub fn new(nozzle_diameter: f64, layer_height: f64) -> Self {
        Self {
            nozzle_diameter,
            layer_height,
            extrusion_width: nozzle_diameter * 1.2,
            pattern_config: ToolpathConfig::default(),
        }
    }

    pub fn with_pattern(mut self, pattern: ToolpathPattern) -> Self {
        self.pattern_config.pattern = pattern;
        self
    }

    pub fn with_config(mut self, config: ToolpathConfig) -> Self {
        self.pattern_config = config;
        self
    }

    /// Generate toolpaths from sliced layers.
    /// `mesh` is used (when Some) for accurate surface Z projection via ray-casting, and for
    /// coverage gap fill on steep faces. Also needed for wall seam transitions.
    pub fn generate(&self, layers: &[Layer], mesh: Option<&Mesh>) -> Vec<Toolpath> {
        // Calculate centroidal axis for non-planar orientation
        let centroidal_axis = CentroidalAxis::compute(layers, 15.0);

        let mut toolpaths: Vec<Toolpath> = layers
            .iter()
            .enumerate()
            .map(|(layer_idx, layer)| self.generate_layer_toolpath(layer, layer_idx, &centroidal_axis, mesh))
            .collect();

        // Insert ruled-surface seam transitions between consecutive curved layers if enabled
        if self.pattern_config.wall_transitions {
            self.insert_wall_transitions(layers, &mut toolpaths);
        }

        toolpaths
    }

    fn generate_layer_toolpath(&self, layer: &Layer, layer_idx: usize, centroidal_axis: &CentroidalAxis, mesh: Option<&Mesh>) -> Toolpath {
        let mut paths = Vec::new();

        // Get the slicing direction from centroidal axis
        let slicing_direction = centroidal_axis.slicing_direction_at(layer_idx);

        // Get the centroid for this layer if available
        let layer_centroid = if layer_idx < centroidal_axis.centroids.len() {
            centroidal_axis.centroids[layer_idx]
        } else {
            Point3D::new(0.0, 0.0, layer.z)
        };

        // Check if this layer already has curved Z (e.g. from S4 untransform).
        // If contour points have varying Z within a contour, skip the centroidal axis offset.
        let has_curved_z = layer.contours.iter().any(|c| {
            if c.points.len() < 2 { return false; }
            let z_min = c.points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
            let z_max = c.points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
            (z_max - z_min) > 0.01 // More than 10 microns of Z variation
        });

        // For conical mode every layer's Z is already set correctly by the back-transform and
        // the analytic conical_z_fn.  Fully-clamped bottom layers (all points at bed_z) have
        // has_curved_z=false, but applying the centroidal-axis tilt to bed-level points can
        // push them BELOW bed_z (Z < 0), creating stray sub-bed extrusions.
        // Skip the centroidal-axis offset for all conical layers, regardless of curved-Z status.
        let is_conical = self.pattern_config.conical_params.is_some();

        // Non-planar offset: only apply for planar layers that need centroidal axis adjustment
        let apply_nonplanar_offset = |p: Point3D| -> Point3D {
            if has_curved_z || is_conical {
                return p; // Z already correct from back-transform / conical_z_fn
            }
            if slicing_direction.norm() > 0.001 {
                let slicing_dir_norm = slicing_direction.normalize();
                let dx = p.x - layer_centroid.x;
                let dy = p.y - layer_centroid.y;
                let radial_dist = (dx * dx + dy * dy).sqrt();
                let z_offset = radial_dist * (slicing_dir_norm.x * dx / (radial_dist + 1e-6)
                                             + slicing_dir_norm.y * dy / (radial_dist + 1e-6));
                Point3D::new(p.x, p.y, p.z + z_offset * 0.5)
            } else {
                p
            }
        };

        let orientation = if !has_curved_z && !is_conical && slicing_direction.norm() > 0.001 {
            slicing_direction.normalize()
        } else {
            Vector3D::new(0.0, 0.0, 1.0)
        };

        // For Contour pattern, generate wall loops + infill
        if matches!(self.pattern_config.pattern, ToolpathPattern::Contour) {
            self.generate_walls_and_infill(layer, &apply_nonplanar_offset, orientation, &mut paths, mesh);
        } else {
            // For Spiral/Zigzag patterns, use existing merged-point approach
            let pattern_points = match self.pattern_config.pattern {
                ToolpathPattern::Spiral => generate_spiral_pattern(layer, &self.pattern_config),
                ToolpathPattern::Zigzag => generate_zigzag_pattern(layer, &self.pattern_config),
                ToolpathPattern::Contour => unreachable!(),
            };

            for i in 0..pattern_points.len().saturating_sub(1) {
                let p1 = apply_nonplanar_offset(pattern_points[i]);
                let p2 = apply_nonplanar_offset(pattern_points[i + 1]);
                let distance = (p2 - p1).norm();
                let extrusion = self.calculate_extrusion_volume(distance, self.extrusion_width, layer.layer_height);

                paths.push(ToolpathSegment {
                    position: p1,
                    orientation,
                    extrusion,
                    feedrate: self.pattern_config.print_speed,
                });
            }
        }

        Toolpath {
            paths,
            z: layer.z,
            layer_height: layer.layer_height,
        }
    }

    /// Generate wall loops (perimeters) + infill for a layer.
    /// For each contour: offset inward N times for walls, then fill interior with infill pattern.
    /// When `mesh` is provided and the layer has curved Z, a MeshRayCaster is built once and used
    /// to project wall loop Z values and infill Z values onto the actual mesh surface.
    fn generate_walls_and_infill(
        &self,
        layer: &Layer,
        apply_nonplanar_offset: &impl Fn(Point3D) -> Point3D,
        orientation: Vector3D,
        paths: &mut Vec<ToolpathSegment>,
        mesh: Option<&Mesh>,
    ) {
        use crate::contour_offset::generate_wall_loops;

        // For geodesic / coordinate-transform modes (skip_infill=true), 2D inward offset doesn't
        // make sense for 3D surface curves: the XY projection of a surface contour is often
        // concave or non-convex, and the offset can push vertices outside the actual mesh boundary,
        // creating stray extrusion paths visible in the toolpath view.
        // In these modes just extrude the original contours directly.
        if self.pattern_config.skip_infill {
            for contour in &layer.contours {
                if contour.points.len() >= 2 {
                    self.extrude_contour(contour, apply_nonplanar_offset, orientation, paths, layer.layer_height);
                }
            }
            return;
        }

        // Detect whether this layer has varying Z (curved surface).
        // Only build the raycaster for curved layers — planar layers don't need it.
        let has_curved_z_layer = layer.contours.iter().any(|c| {
            if c.points.len() < 2 { return false; }
            let z_min = c.points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
            let z_max = c.points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
            (z_max - z_min) > 0.01
        });

        // Build raycaster once per layer (64×64 bin grid, ~2–5 ms on typical meshes).
        // ONLY when use_mesh_raycaster is explicitly enabled — this flag is correct only for
        // geodesic force-infill.  Conical / S4 / cylindrical / spherical paths have their own
        // geometrically-correct Z from back-transforms; applying mesh surface Z here would corrupt them.
        let raycaster_storage: Option<MeshRayCaster> = if has_curved_z_layer && self.pattern_config.use_mesh_raycaster {
            mesh.map(|m| MeshRayCaster::new(m, 64))
        } else {
            None
        };
        let raycaster_opt: Option<(&MeshRayCaster, &Mesh)> = match (&raycaster_storage, mesh) {
            (Some(r), Some(m)) => Some((r, m)),
            _ => None,
        };

        let wall_count = self.pattern_config.wall_count;

        for contour in &layer.contours {
            if contour.points.len() < 3 {
                continue;
            }

            // Open arcs (e.g. from artifact-segment splitting at the base) must be extruded
            // directly — the offset_contour algorithm uses (i+1)%n wrap-around and treats every
            // contour as closed, so it creates a ghost closing edge for open arcs, producing
            // endpoint vertices that can land outside the mesh (stray extrusions).
            if !contour.closed {
                self.extrude_contour(contour, apply_nonplanar_offset, orientation, paths, layer.layer_height);
                continue;
            }

            // Generate wall loops (outermost to innermost)
            let mut wall_loops = generate_wall_loops(contour, wall_count, self.pattern_config.line_width);

            // If no wall loops could be generated (contour too small), just trace the original
            if wall_loops.is_empty() {
                self.extrude_contour(contour, apply_nonplanar_offset, orientation, paths, layer.layer_height);
                continue;
            }

            // ── Wall loop boundary validation ──────────────────────────────────────────
            // The 2D inward offset algorithm can place vertices inside the bounding box
            // of the original contour but *outside* the actual polygon for concave shapes.
            // This is especially common with adaptive layers which creates more irregular
            // cross-sections. Remove any wall loop where a point falls outside the original
            // contour boundary; fall back to tracing the original if all loops are invalid.
            wall_loops.retain(|wl| {
                wl.points.iter().all(|pt| is_point_in_contour(pt, contour))
            });
            if wall_loops.is_empty() {
                self.extrude_contour(contour, apply_nonplanar_offset, orientation, paths, layer.layer_height);
                continue;
            }
            // ──────────────────────────────────────────────────────────────────────────

            // ── Conical Z correction ───────────────────────────────────────────────────
            // For conical slicing the correct Z for any (x,y) in this layer is:
            //   z(x,y) = deformed_z - signed_tan * r,  r = sqrt((x-cx)² + (y-cy)²)
            // where signed_tan = sign × tan(angle) (-1×tan for Outward, +1×tan for Inward).
            // IDW from perimeter wall-loop points is wrong because the Z function is radial,
            // not linear: IDW gives z≈50mm at the center when the correct value is z=20mm.
            // We recover deformed_z from any contour point (all points share the same
            // deformed Z — that is what the planar slicer cut at).
            let conical_z_fn: Option<Box<dyn Fn(f64, f64) -> f64>> =
                if let Some((cx, cy, signed_tan, z_min)) = self.pattern_config.conical_params {
                    // Recover deformed_z from contour points using the minimum.
                    // For each point: D_computed = p.z + signed_tan × r.
                    // Clamped points (at bed_z) give D_computed > D (too high).
                    // Unclamped points give D_computed = D exactly.
                    // Therefore min() gives the correct true deformed_z.
                    let deformed_z = if !contour.points.is_empty() {
                        contour.points.iter()
                            .map(|p| {
                                let r = ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt();
                                p.z + signed_tan * r  // deformed_z = z_back + signed_tan × r
                            })
                            .fold(f64::INFINITY, f64::min)
                    } else {
                        layer.z
                    };
                    Some(Box::new(move |x: f64, y: f64| {
                        let r = ((x - cx).powi(2) + (y - cy).powi(2)).sqrt();
                        // Clamp to z_min (build plate) to prevent infill from going below the bed.
                        (deformed_z - signed_tan * r).max(z_min)
                    }))
                } else {
                    None
                };

            // Apply conical Z correction to wall loop points (the 2D offset left them
            // at the layer's nominal Z; we need the per-point analytic value).
            if let Some(ref z_fn) = conical_z_fn {
                for wall_contour in &mut wall_loops {
                    for pt in &mut wall_contour.points {
                        pt.z = z_fn(pt.x, pt.y);
                    }
                }
            }
            // ──────────────────────────────────────────────────────────────────────────

            // Post-project wall loop Z values onto the mesh surface for accurate non-planar
            // positioning. The 2D wall offset operates purely in XY, so the Z is still the
            // layer's nominal z — raycasting gives the correct surface Z for each point.
            if let Some((caster, mesh_ref)) = raycaster_opt {
                for wall_contour in &mut wall_loops {
                    wall_contour.points = wall_contour.points.iter()
                        .map(|pt| {
                            let z = caster.project_z(pt.x, pt.y, pt.z, mesh_ref).unwrap_or(pt.z);
                            Point3D::new(pt.x, pt.y, z)
                        })
                        .collect();
                }
            }

            // Extrude each wall loop (outer first for surface quality)
            for wall_contour in &wall_loops {
                self.extrude_contour(wall_contour, apply_nonplanar_offset, orientation, paths, layer.layer_height);
            }

            // Generate infill inside the innermost wall
            if self.pattern_config.infill_density > 0.01 && !self.pattern_config.skip_infill {
                let innermost = wall_loops.last().unwrap();

                // Collect all wall loop points for IDW Z fallback on curved layers.
                let z_ref_points: Vec<crate::geometry::Point3D> = wall_loops.iter()
                    .flat_map(|c| c.points.iter().copied())
                    .collect();

                let z_override: Option<&dyn Fn(f64, f64) -> f64> =
                    conical_z_fn.as_deref();

                let infill_lines = generate_clipped_infill(
                    innermost,
                    &self.pattern_config,
                    layer.z,
                    Some(contour), // Validate against original contour
                    Some(&z_ref_points),
                    raycaster_opt, // Accurate mesh Z + coverage gap fill
                    z_override,    // Analytic conical formula (overrides IDW)
                );

                for line in &infill_lines {
                    if line.len() < 2 {
                        continue;
                    }

                    // Travel to infill line start
                    paths.push(ToolpathSegment {
                        position: apply_nonplanar_offset(line[0]),
                        orientation,
                        extrusion: 0.0,
                        feedrate: self.pattern_config.travel_speed,
                    });

                    // Extrude along infill line
                    for i in 0..line.len().saturating_sub(1) {
                        let p1 = apply_nonplanar_offset(line[i]);
                        let p2 = apply_nonplanar_offset(line[i + 1]);
                        let distance = (p2 - p1).norm();
                        let extrusion = self.calculate_extrusion_volume(distance, self.extrusion_width, layer.layer_height);

                        paths.push(ToolpathSegment {
                            position: p1,
                            orientation,
                            extrusion,
                            feedrate: self.pattern_config.infill_speed,
                        });
                    }
                }
            }
        }
    }

    /// Extrude along a single contour with travel move to start.
    fn extrude_contour(
        &self,
        contour: &crate::geometry::Contour,
        apply_nonplanar_offset: &impl Fn(Point3D) -> Point3D,
        orientation: Vector3D,
        paths: &mut Vec<ToolpathSegment>,
        layer_height: f64,
    ) {
        // For closed contours append the starting point so the loop visually closes.
        // resample_contour only covers i..i+1 pairs so without this the closing edge
        // (last_point → first_point) is never resampled or extruded.
        let pts_for_resample: Vec<Point3D> = if contour.closed && !contour.points.is_empty() {
            let mut p = contour.points.clone();
            p.push(contour.points[0]);
            p
        } else {
            contour.points.clone()
        };

        let contour_points = resample_contour(&pts_for_resample, self.pattern_config.node_distance);
        if contour_points.len() < 2 {
            return;
        }

        // Always emit a travel move to the contour start.
        // This ensures that the first contour of every layer also has a proper
        // positioning move, so the toolpath visualization never draws a spurious
        // extrusion line from the previous layer's endpoint to this contour.
        paths.push(ToolpathSegment {
            position: apply_nonplanar_offset(contour_points[0]),
            orientation,
            extrusion: 0.0,
            feedrate: self.pattern_config.travel_speed,
        });

        // Extrude along contour
        for i in 0..contour_points.len().saturating_sub(1) {
            let p1 = apply_nonplanar_offset(contour_points[i]);
            let p2 = apply_nonplanar_offset(contour_points[i + 1]);
            let distance = (p2 - p1).norm();
            let extrusion = self.calculate_extrusion_volume(distance, self.extrusion_width, layer_height);

            paths.push(ToolpathSegment {
                position: p1,
                orientation,
                extrusion,
                feedrate: self.pattern_config.print_speed,
            });
        }
    }

    /// Insert ruled-surface transition paths between consecutive curved layer outer walls.
    /// The transition bridges the staircase gap between layer n's top surface and layer n+1's
    /// bottom surface by extruding a zigzag along ruling lines of the two outer contours.
    fn insert_wall_transitions(&self, layers: &[Layer], toolpaths: &mut Vec<Toolpath>) {
        // Wall seam transitions are only meaningful for geodesic/S4 slicing modes where
        // consecutive layer surfaces have genuine 3D gaps between their walls.
        // For conical slicing the back-transform produces analytically smooth layer
        // connectivity — no staircase gap exists — and the ruling-surface zigzag sweeps
        // across the full interior of the (wide, ring-like) conical contours, producing
        // large stray extrusions that cover the model base.  Skip in conical mode.
        if self.pattern_config.conical_params.is_some() {
            return;
        }

        use crate::ruled_surface::RuledSurface;

        for i in 0..layers.len().saturating_sub(1) {
            // Only add transitions for curved layers (flat layers are already contiguous)
            let has_curved = layers[i].contours.first()
                .map(|c| {
                    if c.points.len() < 2 { return false; }
                    let z_min = c.points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
                    let z_max = c.points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
                    (z_max - z_min) > 0.01
                })
                .unwrap_or(false);
            if !has_curved { continue; }

            let c1 = match layers[i].contours.first() {
                Some(c) if !c.points.is_empty() => c,
                _ => continue,
            };
            let c2 = match layers[i + 1].contours.first() {
                Some(c) if !c.points.is_empty() => c,
                _ => continue,
            };

            // Only bridge when there is a meaningful Z gap to fill
            let z1_max = c1.points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
            let z2_min = c2.points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
            if z2_min - z1_max < self.layer_height * 0.3 { continue; }

            if let Some(ruled) = RuledSurface::detect_from_contours(&c1.points, &c2.points) {
                let transition_pts = ruled.generate_transition_path();
                if transition_pts.len() < 2 { continue; }

                let orientation = Vector3D::new(0.0, 0.0, 1.0);

                // Travel move to transition start
                toolpaths[i].paths.push(ToolpathSegment {
                    position: transition_pts[0],
                    orientation,
                    extrusion: 0.0,
                    feedrate: self.pattern_config.travel_speed,
                });

                // Extrude zigzag along ruling lines
                for j in 0..transition_pts.len().saturating_sub(1) {
                    let p1 = transition_pts[j];
                    let p2 = transition_pts[j + 1];
                    let dist = (p2 - p1).norm();
                    let extrusion = self.calculate_extrusion_volume(dist, self.extrusion_width, self.layer_height);
                    toolpaths[i].paths.push(ToolpathSegment {
                        position: p1,
                        orientation,
                        extrusion,
                        feedrate: self.pattern_config.print_speed,
                    });
                }
            }
        }
    }

    /// Calculate extrusion based on volume (SIGGRAPH Asia 2022 method)
    /// Takes into account waypoint width, height, and distance
    fn calculate_extrusion_volume(&self, distance: f64, width: f64, height: f64) -> f64 {
        // Volume of material extruded = cross_section_area * distance
        // Cross section is approximately rectangular: width * height
        let extrusion_volume = width * height * distance;

        // Convert volume to filament length
        let filament_diameter = 1.75_f64; // mm (standard)
        let filament_area = std::f64::consts::PI * (filament_diameter / 2.0).powi(2);

        extrusion_volume / filament_area
    }

    /// Legacy method for compatibility
    fn calculate_extrusion(&self, distance: f64) -> f64 {
        self.calculate_extrusion_volume(distance, self.extrusion_width, self.layer_height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Contour;

    #[test]
    fn test_toolpath_generation() {
        let points = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(10.0, 10.0, 0.0),
        ];
        let contour = Contour::new(points, false);
        let layer = Layer::new(0.2, vec![contour], 0.2);

        let generator = ToolpathGenerator::new(0.4, 0.2);
        let centroidal_axis = CentroidalAxis::compute(&[layer.clone()], 45.0);
        let toolpath = generator.generate_layer_toolpath(&layer, 0, &centroidal_axis, None);

        assert!(!toolpath.paths.is_empty());
    }
}
