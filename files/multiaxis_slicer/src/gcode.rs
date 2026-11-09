// G-code generation module — supports up to N rotary axes driven by printer profile.

use crate::toolpath::{Toolpath, ToolpathSegment};
use std::fs::File;
use std::io::{self, Write};
use std::path::Path;

// ─── Axis types ───────────────────────────────────────────────────────────────

/// Which rotary-axis labels to emit in G-code moves (legacy 2-axis enum, kept for
/// profile serialization and UI dropdowns).
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum RotaryAxisMode {
    /// A = pitch (rotation around X), B = roll (rotation around Y).
    AB,
    /// B = tilt, C = rotate.
    BC,
}

impl Default for RotaryAxisMode {
    fn default() -> Self { Self::AB }
}

/// A single rotary axis as seen by the G-code generator.
///
/// The slicer converts `AxisConfig` entries from the active `PrinterProfile` into
/// this leaner form before constructing the generator.
#[derive(Debug, Clone)]
pub struct GCodeAxis {
    /// G-code letter: "A", "B", "C", "U", "V", "W", etc.
    pub name: String,
    /// `true`  → this axis tilts/rotates the head (nozzle).
    /// `false` → this axis tilts/rotates the bed/table (part).
    pub is_head: bool,
    /// Allowed range in degrees (inclusive). Used for clamping only — the
    /// controller will fault if these are exceeded, so we clamp pre-emptively.
    pub min_deg: f64,
    pub max_deg: f64,
    /// Unit vector in machine frame that this joint rotates around.
    /// X=[1,0,0], Y=[0,1,0], Z=[0,0,1].
    pub rotation_axis: [f64; 3],
    /// Maximum angular speed in degrees/second. The linear feedrate is capped
    /// each move so this axis never exceeds this speed. 0 = unlimited.
    pub max_deg_per_s: f64,
    /// Sign convention: `+1` when a positive commanded value produces a
    /// right-handed rotation about `rotation_axis`, `-1` when the machine
    /// defines positive the other way.
    ///
    /// Machines genuinely differ here, so it has to be configuration rather
    /// than a constant. Before this field existed the convention was
    /// hard-coded, and inverted relative to the right-hand rule: a requested
    /// 45 degree tilt toward +X came out as 45 degrees toward -X. That is a 90
    /// degree orientation error, and nothing recorded which convention was
    /// intended, so there was no way to check it except against hardware.
    ///
    /// Use [`GCodeGenerator::verify_round_trip`] to confirm a configuration
    /// reproduces the orientations it is asked for.
    pub direction: i8,
}

// ─── Kinematics ──────────────────────────────────────────────────────────────

/// Machine kinematics parameters — passed to `GCodeGenerator` at construction.
#[derive(Debug, Clone)]
pub struct MachineKinematics {
    /// Pivot-to-nozzle-tip distance in mm.
    /// Set to 0.0 to disable TCP compensation entirely.
    pub tcp_offset: f64,

    /// All configured rotary axes, in the order they should appear in G-code.
    ///
    /// Default: two head axes ["A", "B"] for a standard 5-axis machine.
    /// For a 7-axis trunnion+head machine you might have:
    ///   ["A"(bed), "B"(bed), "C"(head), …]
    pub axes: Vec<GCodeAxis>,
}

impl Default for MachineKinematics {
    fn default() -> Self {
        Self {
            tcp_offset: 0.0,
            axes: vec![
                GCodeAxis { name: "A".to_string(), is_head: true,  min_deg: -180.0, max_deg: 180.0, rotation_axis: [1.,0.,0.], max_deg_per_s: 30.0, direction: 1 },
                GCodeAxis { name: "B".to_string(), is_head: true,  min_deg: -180.0, max_deg: 180.0, rotation_axis: [0.,1.,0.], max_deg_per_s: 30.0, direction: 1 },
            ],
        }
    }
}

// ─── Generator ───────────────────────────────────────────────────────────────

pub struct GCodeGenerator {
    pub filament_diameter: f64,
    pub nozzle_temp: f64,
    pub bed_temp: f64,
    pub retraction_distance: f64,
    pub retraction_speed: f64,
    pub kinematics: MachineKinematics,
}

impl Default for GCodeGenerator {
    fn default() -> Self {
        Self {
            filament_diameter: 1.75,
            nozzle_temp: 210.0,
            bed_temp: 60.0,
            retraction_distance: 6.5,
            retraction_speed: 25.0,
            kinematics: MachineKinematics::default(),
        }
    }
}

impl GCodeGenerator {
    pub fn new() -> Self { Self::default() }

    // ── Public generation methods ─────────────────────────────────────────

    /// Generate G-code from toolpaths and write to file.
    pub fn generate<P: AsRef<Path>>(
        &self,
        toolpaths: &[Toolpath],
        output_path: P,
    ) -> io::Result<()> {
        let mut file = File::create(output_path)?;
        self.write_header(&mut file)?;

        let mut total_extrusion = 0.0;
        let mut prev_pos: Option<crate::geometry::Point3D> = None;
        let mut prev_angles: Vec<(String, f64)> = Vec::new();
        for toolpath in toolpaths {
            for segment in &toolpath.paths {
                let (new_pos, new_angles) = self.write_move(
                    &mut file, segment, &mut total_extrusion,
                    prev_pos.as_ref(), &prev_angles)?;
                prev_pos = Some(new_pos);
                prev_angles = new_angles;
            }
        }

        self.write_footer(&mut file)?;
        Ok(())
    }

    /// Generate G-code from toolpaths and return as a vector of strings.
    pub fn generate_to_string(&self, toolpaths: &[Toolpath]) -> Vec<String> {
        let total_layers = toolpaths.len();
        let total_segments: usize = toolpaths.iter().map(|tp| tp.paths.len()).sum();

        log::info!("G-code generation started:");
        log::info!("  Layers:           {}", total_layers);
        log::info!("  Total segments:   {}", total_segments);
        log::info!("  Rotary axes:      {}", self.kinematics.axes.iter()
            .map(|a| a.name.as_str()).collect::<Vec<_>>().join(", "));

        let mut lines = Vec::with_capacity(total_segments + 40);

        // ── Header ─────────────────────────────────────────────────────
        lines.push("; Generated by multiaxis_slicer".to_string());
        lines.push(format!("; Filament diameter: {} mm", self.filament_diameter));
        let axis_desc = self.kinematics.axes.iter()
            .map(|a| format!("{} ({})", a.name, if a.is_head { "head" } else { "bed" }))
            .collect::<Vec<_>>().join(", ");
        lines.push(format!("; Rotary axes: {}", axis_desc));
        if self.kinematics.tcp_offset.abs() > 1e-6 {
            lines.push(format!("; TCP offset: {:.2} mm", self.kinematics.tcp_offset));
        }
        lines.push(String::new());
        lines.push("G21 ; Set units to millimeters".to_string());
        lines.push("G90 ; Use absolute coordinates".to_string());
        lines.push("M82 ; Use absolute extrusion".to_string());
        lines.push(String::new());
        lines.push(format!("M104 S{} ; Set nozzle temp", self.nozzle_temp));
        lines.push(format!("M140 S{} ; Set bed temp",   self.bed_temp));
        lines.push(format!("M109 S{} ; Wait for nozzle temp", self.nozzle_temp));
        lines.push(format!("M190 S{} ; Wait for bed temp",    self.bed_temp));
        lines.push(String::new());
        lines.push("G28 ; Home all axes".to_string());
        lines.push("G92 E0 ; Reset extruder".to_string());
        lines.push(String::new());

        // ── Toolpaths ──────────────────────────────────────────────────
        let mut total_extrusion = 0.0_f64;
        let mut processed = 0usize;
        let report_every = (total_segments / 10).max(1000);
        // Track previous move's position and axis angles for feedrate limiting.
        let mut prev_pos: Option<crate::geometry::Point3D> = None;
        let mut prev_angles: Vec<(String, f64)> = Vec::new();

        for (layer_idx, toolpath) in toolpaths.iter().enumerate() {
            lines.push(format!("; LAYER:{}", layer_idx));
            lines.push(format!("; Z:{:.3}", toolpath.z));

            for segment in &toolpath.paths {
                let axis_angles = self.compute_axis_angles(&segment.orientation);
                let pos = self.tcp_compensate(&segment.position, &segment.orientation);
                let rot_str = self.format_all_axes(&axis_angles);

                if segment.extrusion < 1e-8 {
                    // Travel: retract → rapid → re-prime  (no angular speed limiting on rapids)
                    lines.push(format!("G1 E{:.5} F{:.0} ; Retract",
                        total_extrusion - self.retraction_distance,
                        self.retraction_speed * 60.0));
                    lines.push(format!("G0 X{:.3} Y{:.3} Z{:.3} {} F6000 ; Travel",
                        pos.x, pos.y, pos.z, rot_str));
                    lines.push(format!("G1 E{:.5} F{:.0} ; Prime",
                        total_extrusion,
                        self.retraction_speed * 60.0));
                } else {
                    // Extrusion move — cap feedrate by rotary axis angular speed limits.
                    let dist_mm = prev_pos.as_ref().map(|pp| {
                        let dx = pos.x - pp.x; let dy = pos.y - pp.y; let dz = pos.z - pp.z;
                        (dx*dx + dy*dy + dz*dz).sqrt()
                    }).unwrap_or(0.0);
                    let capped_f = if prev_angles.is_empty() {
                        segment.feedrate
                    } else {
                        self.limit_feedrate_for_angular_speed(
                            segment.feedrate, dist_mm, &axis_angles, &prev_angles)
                    };
                    total_extrusion += segment.extrusion;
                    lines.push(format!("G1 X{:.3} Y{:.3} Z{:.3} {} E{:.5} F{:.0}",
                        pos.x, pos.y, pos.z, rot_str, total_extrusion,
                        capped_f * 60.0));
                }

                // Update tracking state for next move.
                prev_pos = Some(pos);
                prev_angles = axis_angles;

                processed += 1;
                if processed % report_every == 0 {
                    let pct = (processed as f32 / total_segments as f32 * 100.0) as u32;
                    log::info!("  G-code: {}% ({}/{} segs, layer {}/{})",
                        pct, processed, total_segments, layer_idx + 1, total_layers);
                }
            }
        }

        log::info!("✓ G-code complete: {} lines", lines.len());

        // ── Footer ─────────────────────────────────────────────────────
        lines.push(String::new());
        lines.push("; End G-code".to_string());
        lines.push(format!("G1 E-{:.1} F{:.0} ; Retract",
            self.retraction_distance, self.retraction_speed * 60.0));
        lines.push("M104 S0 ; Turn off nozzle".to_string());
        lines.push("M140 S0 ; Turn off bed".to_string());
        lines.push("G28 X Y ; Home X and Y".to_string());
        lines.push("M84 ; Disable motors".to_string());

        lines
    }

    // ── Private I/O helpers ───────────────────────────────────────────────

    fn write_header(&self, file: &mut File) -> io::Result<()> {
        writeln!(file, "; Generated by multiaxis_slicer")?;
        writeln!(file, "; Filament diameter: {} mm", self.filament_diameter)?;
        let axis_desc = self.kinematics.axes.iter()
            .map(|a| format!("{} ({})", a.name, if a.is_head { "head" } else { "bed" }))
            .collect::<Vec<_>>().join(", ");
        writeln!(file, "; Rotary axes: {}", axis_desc)?;
        if self.kinematics.tcp_offset.abs() > 1e-6 {
            writeln!(file, "; TCP offset: {:.2} mm", self.kinematics.tcp_offset)?;
        }
        writeln!(file)?;
        writeln!(file, "G21 ; Set units to millimeters")?;
        writeln!(file, "G90 ; Use absolute coordinates")?;
        writeln!(file, "M82 ; Use absolute extrusion")?;
        writeln!(file)?;
        writeln!(file, "M104 S{} ; Set nozzle temp", self.nozzle_temp)?;
        writeln!(file, "M140 S{} ; Set bed temp",   self.bed_temp)?;
        writeln!(file, "M109 S{} ; Wait for nozzle temp", self.nozzle_temp)?;
        writeln!(file, "M190 S{} ; Wait for bed temp",    self.bed_temp)?;
        writeln!(file)?;
        writeln!(file, "G28 ; Home all axes")?;
        writeln!(file, "G92 E0 ; Reset extruder")?;
        writeln!(file)?;
        Ok(())
    }

    /// Returns `(commanded_pos, axis_angles)` so the caller can pass them as
    /// `prev_pos`/`prev_angles` to the next call for feedrate limiting.
    fn write_move(
        &self,
        file: &mut File,
        segment: &ToolpathSegment,
        total_extrusion: &mut f64,
        prev_pos: Option<&crate::geometry::Point3D>,
        prev_angles: &[(String, f64)],
    ) -> io::Result<(crate::geometry::Point3D, Vec<(String, f64)>)> {
        let axis_angles = self.compute_axis_angles(&segment.orientation);
        let pos = self.tcp_compensate(&segment.position, &segment.orientation);
        let rot_str = self.format_all_axes(&axis_angles);

        if segment.extrusion < 1e-8 {
            // Travel: no angular speed limiting on rapids
            writeln!(file, "G1 E{:.5} F{:.0} ; Retract",
                *total_extrusion - self.retraction_distance,
                self.retraction_speed * 60.0)?;
            writeln!(file, "G0 X{:.3} Y{:.3} Z{:.3} {} F6000 ; Travel",
                pos.x, pos.y, pos.z, rot_str)?;
            writeln!(file, "G1 E{:.5} F{:.0} ; Prime",
                *total_extrusion,
                self.retraction_speed * 60.0)?;
        } else {
            let dist_mm = prev_pos.map(|pp| {
                let dx = pos.x - pp.x; let dy = pos.y - pp.y; let dz = pos.z - pp.z;
                (dx*dx + dy*dy + dz*dz).sqrt()
            }).unwrap_or(0.0);
            let capped_f = if prev_angles.is_empty() {
                segment.feedrate
            } else {
                self.limit_feedrate_for_angular_speed(
                    segment.feedrate, dist_mm, &axis_angles, prev_angles)
            };
            *total_extrusion += segment.extrusion;
            writeln!(file, "G1 X{:.3} Y{:.3} Z{:.3} {} E{:.5} F{:.0}",
                pos.x, pos.y, pos.z, rot_str, *total_extrusion,
                capped_f * 60.0)?;
        }
        Ok((pos, axis_angles))
    }

    fn write_footer(&self, file: &mut File) -> io::Result<()> {
        writeln!(file)?;
        writeln!(file, "; End G-code")?;
        writeln!(file, "G1 E-{:.1} F{:.0} ; Retract",
            self.retraction_distance, self.retraction_speed * 60.0)?;
        writeln!(file, "M104 S0 ; Turn off nozzle")?;
        writeln!(file, "M140 S0 ; Turn off bed")?;
        writeln!(file, "G28 X Y ; Home X and Y")?;
        writeln!(file, "M84 ; Disable motors")?;
        Ok(())
    }

    // ── Kinematics ────────────────────────────────────────────────────────

    /// Forward kinematics: the tool direction produced by a set of rotary axis
    /// commands.
    ///
    /// `MachineKinematics::axes` is an **ordered serial chain**, base first.
    /// Each axis is carried by the ones before it, so earlier axes also rotate
    /// the axes that follow:
    ///
    /// ```text
    /// tool = R(axis[0], q[0]) · R(axis[1], q[1]) · … · ẑ
    /// ```
    ///
    /// This ordering is a physical property of the machine, so it has to come
    /// from configuration. Each axis's `direction` selects whether a positive
    /// command is a right-handed rotation about `rotation_axis` or the reverse.
    pub fn tool_direction(&self, angles_deg: &[f64]) -> crate::geometry::Vector3D {
        use nalgebra::{Rotation3, Unit, Vector3};

        let mut rot = Rotation3::identity();
        for (ax, &q) in self.kinematics.axes.iter().zip(angles_deg.iter()) {
            let [x, y, z] = ax.rotation_axis;
            let v = Vector3::new(x, y, z);
            if v.norm() < 1e-12 {
                continue;
            }
            let sign = if ax.direction < 0 { -1.0 } else { 1.0 };
            rot *= Rotation3::from_axis_angle(&Unit::new_normalize(v), sign * q.to_radians());
        }
        let d = rot * Vector3::new(0.0, 0.0, 1.0);
        crate::geometry::Vector3D::new(d.x, d.y, d.z)
    }

    /// Inverse kinematics: rotary axis commands that point the tool along
    /// `orientation`.
    ///
    /// A tool direction is 2 degrees of freedom — the nozzle is rotationally
    /// symmetric about its own axis — so at most the first two rotary axes are
    /// solved and any beyond them are left at zero.
    ///
    /// ## Why this is not a pair of `atan2`s
    ///
    /// This used to compute each axis independently:
    ///
    /// ```text
    /// A (about X) = atan2(oy, oz)
    /// B (about Y) = atan2(-ox, oz)
    /// ```
    ///
    /// Each formula solves its own axis as though every other axis were at
    /// zero. On a serial chain the joints compose, so that is only valid when
    /// the tilt happens to lie in one joint's plane. Sweeping a 45 degree tilt
    /// around the azimuth and composing the resulting commands gave a worst
    /// case orientation error of **92.4 degrees**; the hard-coded handedness
    /// added a further 90 degree error even in the in-plane cases, where the
    /// independent form should have been exact.
    ///
    /// Solved properly instead: minimise the angle between the commanded tool
    /// direction and the target over the two joint angles. The objective is
    /// smooth and two-dimensional, so a coarse seed plus Gauss-Newton
    /// refinement converges in a few iterations, and it works for whatever
    /// axis vectors and sign conventions the machine is configured with rather
    /// than only for the axis-aligned cases.
    ///
    /// Angles are clamped to each axis's configured range afterwards. A clamp
    /// means the orientation is unreachable; that should be caught during
    /// motion planning, and this is a safety net.
    pub fn compute_axis_angles(
        &self,
        orientation: &crate::geometry::Vector3D,
    ) -> Vec<(String, f64)> {
        let n = self.kinematics.axes.len();
        let mut angles = vec![0.0_f64; n];

        let target = {
            let len = orientation.norm();
            if !len.is_finite() || len < 1e-9 {
                // Degenerate request: leave every axis at zero rather than
                // emitting NaN into the G-code.
                return self
                    .kinematics
                    .axes
                    .iter()
                    .map(|ax| (ax.name.clone(), 0.0_f64.clamp(ax.min_deg, ax.max_deg)))
                    .collect();
            }
            crate::geometry::Vector3D::new(
                orientation.x / len,
                orientation.y / len,
                orientation.z / len,
            )
        };

        // Closed form for the overwhelmingly common case: the first two rotary
        // axes are distinct coordinate axes. Exact, and cheap enough to run per
        // toolpath segment -- a Benchy emits over 600k of them, so the
        // numerical search below (hundreds of FK evaluations each) is not
        // affordable as the primary path, quite apart from converging to local
        // minima 7 degrees out.
        //
        // Derivations, writing the chain as tool = R(u0, q0) · R(u1, q1) · ẑ
        // with q0 the outer (base-side) joint:
        //
        //   (X, Y): n = ( sin q1, -sin q0 cos q1,  cos q0 cos q1)
        //           q1 = asin(nx),          q0 = atan2(-ny, nz)
        //   (Y, X): n = ( sin q0 cos q1, -sin q1,  cos q0 cos q1)
        //           q1 = -asin(ny),         q0 = atan2(nx, nz)
        //   (Z, X): n = ( sin q0 sin q1, -cos q0 sin q1, cos q1)
        //           q1 = acos(nz),          q0 = atan2(nx, -ny)
        //   (Z, Y): n = ( cos q0 sin q1,  sin q0 sin q1, cos q1)
        //           q1 = acos(nz),          q0 = atan2(ny, nx)
        //
        // (X, Z) and (Y, Z) are degenerate: an inner Z rotation leaves ẑ fixed,
        // so the chain only reaches one plane. Those fall through to the
        // numerical path, which will report the shortfall honestly via
        // `verify_round_trip`.
        if n >= 2 {
            let axis_of = |v: &[f64; 3]| -> Option<usize> {
                let mag = [v[0].abs(), v[1].abs(), v[2].abs()];
                let k = (0..3).max_by(|&i, &j| mag[i].partial_cmp(&mag[j]).unwrap())?;
                // Only treat it as a coordinate axis if it really is one.
                if mag[k] > 0.999 && mag[(k + 1) % 3] < 1e-6 && mag[(k + 2) % 3] < 1e-6 {
                    Some(k)
                } else {
                    None
                }
            };
            let sign_of = |v: &[f64; 3], k: usize| if v[k] < 0.0 { -1.0 } else { 1.0 };

            let a0 = &self.kinematics.axes[0];
            let a1 = &self.kinematics.axes[1];
            if let (Some(k0), Some(k1)) = (axis_of(&a0.rotation_axis), axis_of(&a1.rotation_axis)) {
                let (nx, ny, nz) = (target.x, target.y, target.z);
                let solution = match (k0, k1) {
                    (0, 1) => Some((
                        (-ny).atan2(nz),
                        nx.clamp(-1.0, 1.0).asin(),
                    )),
                    (1, 0) => Some((
                        nx.atan2(nz),
                        -ny.clamp(-1.0, 1.0).asin(),
                    )),
                    (2, 0) => Some((
                        nx.atan2(-ny),
                        nz.clamp(-1.0, 1.0).acos(),
                    )),
                    (2, 1) => Some((
                        ny.atan2(nx),
                        nz.clamp(-1.0, 1.0).acos(),
                    )),
                    _ => None,
                };

                if let Some((q0, q1)) = solution {
                    // Undo the axis vector's own sign and the machine's
                    // positive-direction convention, so the commanded value
                    // matches the hardware.
                    let s0 = sign_of(&a0.rotation_axis, k0)
                        * if a0.direction < 0 { -1.0 } else { 1.0 };
                    let s1 = sign_of(&a1.rotation_axis, k1)
                        * if a1.direction < 0 { -1.0 } else { 1.0 };

                    let mut out = vec![0.0_f64; n];
                    out[0] = q0.to_degrees() / s0;
                    out[1] = q1.to_degrees() / s1;

                    return self
                        .kinematics
                        .axes
                        .iter()
                        .enumerate()
                        .map(|(i, ax)| {
                            let a = out.get(i).copied().unwrap_or(0.0);
                            let a = if a.is_finite() { a } else { 0.0 };
                            (ax.name.clone(), a.clamp(ax.min_deg, ax.max_deg))
                        })
                        .collect();
                }
            }
        }

        let solved = n.min(2);
        if solved > 0 {
            // Search inside each axis's configured range rather than clamping
            // afterwards. Clamping a solved angle can destroy the solution: on
            // a redundant chain (two axes about the same vector) the search
            // legitimately returned a=-195, b=165, whose sum is the required
            // -30 degrees, and independently folding then clamping those to
            // +/-90 gave 90 and 90 -- a sum of 180, pointing the tool straight
            // down. Joint limits are part of the problem, not a post-process.
            let lo: Vec<f64> = self
                .kinematics
                .axes
                .iter()
                .map(|ax| ax.min_deg.max(-360.0))
                .collect();
            let hi: Vec<f64> = self
                .kinematics
                .axes
                .iter()
                .map(|ax| ax.max_deg.min(360.0))
                .collect();
            let project = |q: &mut Vec<f64>| {
                for k in 0..q.len() {
                    q[k] = q[k].clamp(lo[k], hi[k]);
                }
            };

            // Cost: angular error between the achieved and requested direction.
            let cost = |q: &[f64]| -> f64 {
                let d = self.tool_direction(q);
                let dot = (d.dot(&target) / d.norm().max(1e-12)).clamp(-1.0, 1.0);
                dot.acos()
            };

            // Seed from a coarse sweep so refinement starts in the right basin;
            // the objective can have a local minimum on the far side of the
            // sphere.
            let mut best = vec![0.0_f64; n];
            project(&mut best);
            let mut best_cost = cost(&best);
            const SEEDS: i32 = 16;
            for i in 0..=SEEDS {
                for j in 0..=if solved > 1 { SEEDS } else { 0 } {
                    let mut q = vec![0.0_f64; n];
                    q[0] = lo[0] + (hi[0] - lo[0]) * i as f64 / SEEDS as f64;
                    if solved > 1 {
                        q[1] = lo[1] + (hi[1] - lo[1]) * j as f64 / SEEDS as f64;
                    }
                    project(&mut q);
                    let c = cost(&q);
                    if c < best_cost {
                        best_cost = c;
                        best = q;
                    }
                }
            }

            // Pattern search with a shrinking step, including the diagonals.
            //
            // The diagonals matter: the two joint angles are coupled, so the
            // objective has valleys that run diagonally, and axis-aligned
            // moves alone stall on them well short of the minimum. With only
            // the four axis directions this left a 14.5 degree residual on the
            // sweep in `verify_round_trip`.
            const DIRS: [(f64, f64); 8] = [
                (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
                (1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0),
            ];
            let mut step = 30.0_f64;
            while step > 1e-9 {
                let mut improved = false;
                for (d0, d1) in DIRS {
                    if solved < 2 && d1 != 0.0 {
                        continue;
                    }
                    let mut trial = best.clone();
                    trial[0] += d0 * step;
                    if solved > 1 {
                        trial[1] += d1 * step;
                    }
                    project(&mut trial);
                    let c = cost(&trial);
                    if c < best_cost - 1e-15 {
                        best_cost = c;
                        best = trial;
                        improved = true;
                    }
                }
                if !improved {
                    step *= 0.5;
                }
            }

            angles = best;
        }

        self.kinematics
            .axes
            .iter()
            .enumerate()
            .map(|(i, ax)| {
                let a = angles.get(i).copied().unwrap_or(0.0);
                let a = if a.is_finite() { a } else { 0.0 };
                // The search already works inside each axis's range, so this
                // clamp is only a guard against a degenerate configuration.
                (ax.name.clone(), a.clamp(ax.min_deg, ax.max_deg))
            })
            .collect()
    }

    /// Check that this machine configuration can actually reproduce the
    /// orientations it is asked for: solve IK, run FK on the result, and report
    /// the worst angular discrepancy in degrees over a sweep of tool
    /// directions.
    ///
    /// This is the invariant whose absence let a hard-coded, inverted sign
    /// convention and an incorrect serial decomposition both go unnoticed. Run
    /// it after configuring a machine; a healthy configuration returns
    /// essentially zero, and anything above a small fraction of a degree means
    /// the emitted A/B values do not point the tool where the slicer intended.
    ///
    /// `max_tilt_deg` should be the machine's usable tilt envelope. Directions
    /// outside the axes' configured ranges are necessarily unreachable and are
    /// reported as such, so this also doubles as a reachability check.
    pub fn verify_round_trip(&self, max_tilt_deg: f64) -> f64 {
        let mut worst = 0.0_f64;
        let tilt_steps = 6;
        let az_steps = 24;

        for ti in 0..=tilt_steps {
            let tilt = max_tilt_deg * ti as f64 / tilt_steps as f64;
            let t = tilt.to_radians();
            for ai in 0..az_steps {
                let az = 2.0 * std::f64::consts::PI * ai as f64 / az_steps as f64;
                let want = crate::geometry::Vector3D::new(
                    t.sin() * az.cos(),
                    t.sin() * az.sin(),
                    t.cos(),
                );

                let angles = self.compute_axis_angles(&want);
                let q: Vec<f64> = angles.iter().map(|(_, v)| *v).collect();
                let got = self.tool_direction(&q);

                let dot = (got.dot(&want) / got.norm().max(1e-12)).clamp(-1.0, 1.0);
                worst = worst.max(dot.acos().to_degrees());

                if tilt < 1e-9 {
                    break; // vertical has no distinct azimuths
                }
            }
        }
        worst
    }

    /// Cap the linear feedrate (mm/s) so that no rotary axis exceeds its maximum
    /// angular speed (deg/s) over this move.
    ///
    /// `dist_mm` — Euclidean distance of the linear XYZ move.
    /// `cur`     — Axis angles at the end of this move (from `compute_axis_angles`).
    /// `prev`    — Axis angles at the start of this move (end of previous move).
    ///
    /// Returns the minimum of `requested_feedrate` and the axis-constrained limit.
    pub fn limit_feedrate_for_angular_speed(
        &self,
        requested: f64,
        dist_mm: f64,
        cur: &[(String, f64)],
        prev: &[(String, f64)],
    ) -> f64 {
        if dist_mm < 1e-6 { return requested; }
        let mut max_f = requested;
        for ax in &self.kinematics.axes {
            if ax.max_deg_per_s <= 0.0 { continue; }
            let c = cur .iter().find(|(n,_)| *n == ax.name).map(|(_,v)| *v).unwrap_or(0.0);
            let p = prev.iter().find(|(n,_)| *n == ax.name).map(|(_,v)| *v).unwrap_or(0.0);
            let delta = (c - p).abs();
            if delta < 0.01 { continue; }
            // feedrate_limit = max_deg_per_s * dist_mm / delta_deg
            let f_limit = ax.max_deg_per_s * dist_mm / delta;
            if f_limit < max_f { max_f = f_limit; }
        }
        max_f
    }

    /// Apply TCP (tool-centre-point) compensation for multi-axis moves.
    ///
    /// The standard TCP formula assumes the nozzle tip is `tcp_offset` mm below
    /// the head's pivot point.  When the head tilts, the pivot must shift so that
    /// the nozzle tip still lands at `position`.
    ///
    /// The nozzle tip sits a distance `L` = `tcp_offset` from the pivot, along
    /// the tool axis.  Working from the tool direction directly avoids routing
    /// through joint angles, which cannot express this correctly: `a` and `b`
    /// are independent pitch and roll joints, not a polar/azimuth pair.
    ///
    /// With `n̂` the unit tool direction, the tip displacement relative to the
    /// untilted case is exactly `L · (n̂ − ẑ)`:
    ///
    /// ```text
    ///   Δx =  L · n.x
    ///   Δy =  L · n.y
    ///   Δz =  L · (n.z − 1)
    /// ```
    ///
    /// # Sign convention
    ///
    /// This assumes:
    /// - `orientation` points from the nozzle tip toward the pivot, so it is
    ///   `+Z` when the head is vertical (matching the rest of the crate);
    /// - the controller has no built-in RTCP, so G-code XYZ commands the
    ///   **pivot**, and the compensation is *added*;
    /// - machine zero is set such that with the head vertical, commanded XYZ
    ///   already coincides with the nozzle tip.
    ///
    /// A machine that compensates internally, or whose tool vector points the
    /// other way, needs the sign flipped.  Verify on a real or simulated move
    /// before trusting this on hardware.
    ///
    /// If `tcp_offset ≈ 0`, the position is returned unchanged.
    pub fn tcp_compensate(
        &self,
        position: &crate::geometry::Point3D,
        orientation: &crate::geometry::Vector3D,
    ) -> crate::geometry::Point3D {
        let offset = self.kinematics.tcp_offset;
        if offset.abs() < 1e-6 { return *position; }

        let len = orientation.norm();
        if !len.is_finite() || len < 1e-9 { return *position; }
        let n = orientation / len;

        crate::geometry::Point3D::new(
            position.x + offset * n.x,
            position.y + offset * n.y,
            position.z + offset * (n.z - 1.0),
        )
    }

    /// Format all rotary axis angles as a single space-separated G-code fragment.
    ///
    /// Example output for a 3-axis machine:  `"A12.345 B-3.500 C0.000"`
    ///
    /// All configured axes are always emitted so the controller can synchronise
    /// them in one interpolated move.
    fn format_all_axes(&self, axis_angles: &[(String, f64)]) -> String {
        axis_angles.iter()
            .map(|(name, deg)| format!("{}{:.3}", name, deg))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point3D, Vector3D};
    use crate::toolpath::ToolpathSegment;

    /// Regression test for D-14, and the invariant whose absence hid two
    /// separate kinematics defects.
    ///
    /// Solving IK and then running FK on the result must reproduce the
    /// requested tool direction. Before this, `compute_axis_angles` solved each
    /// axis with an independent `atan2`, which (a) hard-coded a handedness
    /// inverted relative to the right-hand rule and (b) treated serial joints
    /// as independent. The same sweep this test performs reported a worst-case
    /// error of 92.4 degrees, including 90 degrees in the in-plane cases where
    /// the independent form should have been exact.
    #[test]
    fn test_ik_round_trips_through_fk() {
        let gen = GCodeGenerator::new();
        let worst = gen.verify_round_trip(45.0);
        assert!(
            worst < 0.01,
            "IK/FK round trip is off by {:.4} deg; the commanded angles do not \
             point the tool where it was asked to go",
            worst
        );
    }

    #[test]
    fn test_ik_round_trips_for_a_reversed_axis() {
        // A machine whose B axis calls the other direction positive must still
        // round-trip: that is the whole point of making the convention
        // configuration rather than a constant.
        let mut gen = GCodeGenerator::new();
        gen.kinematics.axes[1].direction = -1;
        let worst = gen.verify_round_trip(45.0);
        assert!(worst < 0.01, "reversed-axis round trip off by {:.4} deg", worst);
    }

    #[test]
    fn test_reversed_axis_actually_reverses_the_command() {
        // Same requested orientation, opposite sign convention => opposite
        // commanded angle. If this passes trivially the field is being ignored.
        let want = Vector3D::new(0.5, 0.0, 0.866);

        let right = GCodeGenerator::new();
        let a_right = right
            .compute_axis_angles(&want)
            .into_iter()
            .find(|(n, _)| n == "B")
            .map(|(_, v)| v)
            .expect("B axis");

        let mut left = GCodeGenerator::new();
        left.kinematics.axes[1].direction = -1;
        let a_left = left
            .compute_axis_angles(&want)
            .into_iter()
            .find(|(n, _)| n == "B")
            .map(|(_, v)| v)
            .expect("B axis");

        assert!(
            (a_right + a_left).abs() < 0.05,
            "expected opposite commands, got {:.3} and {:.3}",
            a_right,
            a_left
        );
    }

    #[test]
    fn test_vertical_tool_needs_no_rotation() {
        let gen = GCodeGenerator::new();
        for (name, angle) in gen.compute_axis_angles(&Vector3D::new(0.0, 0.0, 1.0)) {
            assert!(
                angle.abs() < 0.01,
                "axis {} commanded {:.4} deg for a vertical tool",
                name,
                angle
            );
        }
    }

    #[test]
    fn test_degenerate_orientation_does_not_emit_nan() {
        let gen = GCodeGenerator::new();
        for bad in [
            Vector3D::new(0.0, 0.0, 0.0),
            Vector3D::new(f64::NAN, 0.0, 1.0),
            Vector3D::new(f64::INFINITY, 0.0, 1.0),
        ] {
            for (name, angle) in gen.compute_axis_angles(&bad) {
                assert!(
                    angle.is_finite(),
                    "axis {} got a non-finite command from {:?}",
                    name,
                    bad
                );
            }
        }
    }

    #[test]
    fn test_fk_respects_the_configured_rotation_axis() {
        // A single axis about X tilts the tool in the YZ plane, not XZ.
        let mut gen = GCodeGenerator::new();
        gen.kinematics.axes.truncate(1);
        gen.kinematics.axes[0].rotation_axis = [1.0, 0.0, 0.0];

        let d = gen.tool_direction(&[30.0]);
        assert!(d.x.abs() < 1e-9, "rotation about X moved the tool in X: {:?}", d);
        assert!(d.y.abs() > 0.1, "rotation about X did not tilt the tool in Y: {:?}", d);
    }

    fn default_toolpath_segment(pos: Point3D, orientation: Vector3D, extrusion: f64) -> ToolpathSegment {
        ToolpathSegment { position: pos, orientation, extrusion, feedrate: 50.0 }
    }

    #[test]
    fn test_vertical_orientation_gives_zero_angles() {
        let gen = GCodeGenerator::new(); // default: A(head) + B(head)
        let angles = gen.compute_axis_angles(&Vector3D::new(0.0, 0.0, 1.0));
        let a = angles.iter().find(|(n, _)| n == "A").map(|(_, v)| *v).unwrap();
        let b = angles.iter().find(|(n, _)| n == "B").map(|(_, v)| *v).unwrap();
        assert!(a.abs() < 1e-9, "A should be 0° for vertical orientation");
        assert!(b.abs() < 1e-9, "B should be 0° for vertical orientation");
    }

    #[test]
    fn test_tilted_orientation_angles() {
        let gen = GCodeGenerator::new();
        // Tilt 45° toward +Y: oy = sin(45°), oz = cos(45°)
        let angle = 45_f64.to_radians();
        let orientation = Vector3D::new(0.0, angle.sin(), angle.cos());
        let angles = gen.compute_axis_angles(&orientation);
        let a = angles.iter().find(|(n, _)| n == "A").map(|(_, v)| *v).unwrap();

        // A revolves around X. Under the right-hand rule a positive rotation
        // about +X carries +Z toward -Y, so reaching +Y needs a NEGATIVE
        // command. This assertion previously expected +45, which is what the
        // hard-coded `atan2(oy, oz)` produced -- and which pointed the tool
        // 90 degrees away from the requested direction. The sign convention is
        // now configurable per axis via `GCodeAxis::direction`; this default
        // generator is right-handed.
        assert!(
            (a + 45.0).abs() < 0.1,
            "A should be ~-45° to tilt the tool toward +Y about a right-handed X axis, got {}",
            a
        );

        // The assertion that actually matters: the commanded angles must point
        // the tool where it was asked to go.
        let q: Vec<f64> = angles.iter().map(|(_, v)| *v).collect();
        let got = gen.tool_direction(&q);
        let err = (got.dot(&orientation) / (got.norm() * orientation.norm()))
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        assert!(err < 0.01, "round trip off by {:.4} deg", err);
    }

    #[test]
    fn test_seven_axis_all_appear_in_output() {
        let gen = GCodeGenerator {
            kinematics: MachineKinematics {
                tcp_offset: 0.0,
                axes: vec![
                    GCodeAxis { name: "A".to_string(), is_head: false, min_deg: -90.0,  max_deg: 90.0,  rotation_axis: [1.,0.,0.], max_deg_per_s: 30.0, direction: 1 },
                    GCodeAxis { name: "B".to_string(), is_head: false, min_deg: -90.0,  max_deg: 90.0,  rotation_axis: [0.,1.,0.], max_deg_per_s: 30.0, direction: 1 },
                    GCodeAxis { name: "C".to_string(), is_head: true,  min_deg: -180.0, max_deg: 180.0, rotation_axis: [0.,0.,1.], max_deg_per_s: 30.0, direction: 1 },
                    GCodeAxis { name: "U".to_string(), is_head: true,  min_deg: -180.0, max_deg: 180.0, rotation_axis: [1.,0.,0.], max_deg_per_s: 30.0, direction: 1 },
                    GCodeAxis { name: "V".to_string(), is_head: false, min_deg: -45.0,  max_deg: 45.0,  rotation_axis: [1.,0.,0.], max_deg_per_s: 30.0, direction: 1 },
                    GCodeAxis { name: "W".to_string(), is_head: false, min_deg: -45.0,  max_deg: 45.0,  rotation_axis: [0.,1.,0.], max_deg_per_s: 30.0, direction: 1 },
                    GCodeAxis { name: "Q".to_string(), is_head: true,  min_deg: -180.0, max_deg: 180.0, rotation_axis: [0.,0.,1.], max_deg_per_s: 30.0, direction: 1 },
                ],
            },
            ..GCodeGenerator::default()
        };
        let angles = gen.compute_axis_angles(&Vector3D::new(0.0, 0.0, 1.0));
        assert_eq!(angles.len(), 7, "All 7 axes must appear in output");
        let formatted = gen.format_all_axes(&angles);
        for letter in &["A", "B", "C", "U", "V", "W", "Q"] {
            assert!(formatted.contains(letter), "Expected {} in '{}'", letter, formatted);
        }
    }

    /// Mixed head/bed machines are **not** correctly supported, and this test
    /// documents that rather than asserting a behaviour that looked right.
    ///
    /// It used to assert that a bed axis received the negation of a head axis,
    /// so the two cancelled. That negation -- `if ax.is_head { angle } else
    /// { -angle }` -- was the entire bed-axis model, and it is not sufficient:
    /// rotating the *part* rotates the whole workpiece frame, so the XYZ path
    /// has to be transformed too, not just an angle flipped. The old code
    /// emitted identical XYZ for a tilting head and a tilting bed, which
    /// cannot both be right.
    ///
    /// The kinematics now treat every configured rotary axis as part of one
    /// serial *tool* chain, which is correct for a tilting head and wrong for
    /// a bed axis. Proper support needs the part branch described in
    /// docs/KINEMATICS_DESIGN_NOTE.md. Until then, two axes about the same
    /// vector compose additively rather than cancelling, and the round trip
    /// still closes for directions the chain can reach -- which is what this
    /// test now pins, so the behaviour cannot change unnoticed.
    #[test]
    fn test_bed_axes_compose_additively_pending_part_branch() {
        let gen = GCodeGenerator {
            kinematics: MachineKinematics {
                tcp_offset: 0.0,
                axes: vec![
                    GCodeAxis { name: "A".to_string(), is_head: false, min_deg: -90.0, max_deg: 90.0, rotation_axis: [1.,0.,0.], max_deg_per_s: 30.0, direction: 1 },
                    GCodeAxis { name: "B".to_string(), is_head: true,  min_deg: -90.0, max_deg: 90.0, rotation_axis: [1.,0.,0.], max_deg_per_s: 30.0, direction: 1 },
                ],
            },
            ..GCodeGenerator::default()
        };
        let angle = 30_f64.to_radians();
        // Orientation tilted 30° toward +Y: achieved by rotating [0,0,1] around X by 30°
        let orientation = Vector3D::new(0.0, angle.sin(), angle.cos());
        let angles = gen.compute_axis_angles(&orientation);
        let a = angles.iter().find(|(n, _)| n == "A").map(|(_, v)| *v).unwrap();
        let b = angles.iter().find(|(n, _)| n == "B").map(|(_, v)| *v).unwrap();

        // Both axes revolve about X, so as a serial chain they sum. Reaching
        // +Y about a right-handed X axis needs -30 degrees in total.
        assert!(
            (a + b + 30.0).abs() < 0.1,
            "the two X axes should sum to -30 deg, got a={:.3} b={:.3}",
            a,
            b
        );

        // And the commanded angles must still achieve the requested direction.
        let q: Vec<f64> = angles.iter().map(|(_, v)| *v).collect();
        let got = gen.tool_direction(&q);
        let err = (got.dot(&orientation) / (got.norm() * orientation.norm()))
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        assert!(err < 0.01, "round trip off by {:.4} deg", err);
    }

    #[test]
    fn test_tcp_compensation_disabled_at_zero_offset() {
        let gen = GCodeGenerator::new();
        let pos = Point3D::new(10.0, 20.0, 5.0);
        let compensated = gen.tcp_compensate(&pos, &Vector3D::new(0.0, 0.0, 1.0));
        assert!((compensated.x - pos.x).abs() < 1e-9);
        assert!((compensated.y - pos.y).abs() < 1e-9);
        assert!((compensated.z - pos.z).abs() < 1e-9);
    }

    /// Machine with a 40 mm pivot-to-tip offset.
    fn tcp_generator(offset: f64) -> GCodeGenerator {
        GCodeGenerator {
            kinematics: MachineKinematics { tcp_offset: offset, ..MachineKinematics::default() },
            ..GCodeGenerator::default()
        }
    }

    #[test]
    fn test_tcp_vertical_tool_needs_no_compensation() {
        let gen = tcp_generator(40.0);
        let pos = Point3D::new(10.0, 20.0, 5.0);
        let out = gen.tcp_compensate(&pos, &Vector3D::new(0.0, 0.0, 1.0));
        assert!((out.x - pos.x).abs() < 1e-9, "x moved: {}", out.x);
        assert!((out.y - pos.y).abs() < 1e-9, "y moved: {}", out.y);
        assert!((out.z - pos.z).abs() < 1e-9, "z moved: {}", out.z);
    }

    /// Regression test for T-03. The previous formula treated the two head
    /// joint angles as a polar/azimuth pair, so a pure tilt toward +Y produced
    /// `Δx = L·sin(a), Δy = 0` -- the whole displacement landed on the wrong
    /// axis, with the Z term inverted as well.
    #[test]
    fn test_tcp_tilt_toward_y_displaces_along_y() {
        let l = 40.0;
        let gen = tcp_generator(l);
        let tilt = 30_f64.to_radians();
        let n = Vector3D::new(0.0, tilt.sin(), tilt.cos());

        let pos = Point3D::new(10.0, 20.0, 5.0);
        let out = gen.tcp_compensate(&pos, &n);

        assert!(
            (out.x - pos.x).abs() < 1e-9,
            "a tilt in the Y-Z plane must not displace X, got {}",
            out.x - pos.x
        );
        assert!(
            ((out.y - pos.y) - l * tilt.sin()).abs() < 1e-9,
            "expected Y displacement of {}, got {}",
            l * tilt.sin(),
            out.y - pos.y
        );
        assert!(
            ((out.z - pos.z) - l * (tilt.cos() - 1.0)).abs() < 1e-9,
            "expected Z displacement of {}, got {}",
            l * (tilt.cos() - 1.0),
            out.z - pos.z
        );
    }

    /// Applying the compensation and then the forward relation must put the
    /// nozzle tip back on the originally commanded point.
    #[test]
    fn test_tcp_round_trip_recovers_tip() {
        let l = 40.0;
        let gen = tcp_generator(l);
        let target = Point3D::new(12.0, -7.0, 3.5);

        for n in [
            Vector3D::new(0.0, 0.0, 1.0),
            Vector3D::new(0.0, 0.5, 3_f64.sqrt() / 2.0),
            Vector3D::new(0.5, 0.0, 3_f64.sqrt() / 2.0),
            Vector3D::new(-0.3, 0.4, 0.866_025_403_784_44),
        ] {
            let n = n / n.norm();
            let commanded = gen.tcp_compensate(&target, &n);
            // Forward: tip = commanded - L·(n̂ - ẑ)
            let tip = Point3D::new(
                commanded.x - l * n.x,
                commanded.y - l * n.y,
                commanded.z - l * (n.z - 1.0),
            );
            assert!(
                (tip - target).norm() < 1e-9,
                "tip {:?} did not return to target {:?} for n {:?}",
                tip,
                target,
                n
            );
        }
    }

    #[test]
    fn test_gcode_file_generation() {
        use crate::toolpath::Toolpath;
        let gen = GCodeGenerator::new();
        let toolpath = Toolpath {
            z: 0.2,
            layer_height: 0.2,
            paths: vec![
                default_toolpath_segment(
                    Point3D::new(0.0, 0.0, 0.2),
                    Vector3D::new(0.0, 0.0, 1.0),
                    0.0,
                ),
                default_toolpath_segment(
                    Point3D::new(10.0, 0.0, 0.2),
                    Vector3D::new(0.0, 0.0, 1.0),
                    0.2,
                ),
            ],
        };
        let result = gen.generate(&[toolpath], "/tmp/test_multiaxis.gcode");
        assert!(result.is_ok());
    }
}
