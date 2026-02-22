// Advanced toolpath pattern generation
// Based on algorithms from SIGGRAPH Asia 2022 paper

use crate::geometry::{Point3D, Vector3D, Contour, Triangle};
use crate::mesh::Mesh;
use crate::slicing::Layer;
use serde::{Deserialize, Serialize};

/// Available toolpath patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ToolpathPattern {
    /// Wall loops + infill (standard slicer output)
    Contour,
    /// Spiral pattern from outside to inside
    Spiral,
    /// Zigzag back-and-forth pattern
    Zigzag,
}

/// Infill pattern for interior fill
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InfillPattern {
    /// Alternating-direction parallel lines (rectilinear)
    Rectilinear,
}

impl ToolpathPattern {
    pub fn name(&self) -> &'static str {
        match self {
            ToolpathPattern::Contour => "Contour",
            ToolpathPattern::Spiral => "Spiral",
            ToolpathPattern::Zigzag => "Zigzag",
        }
    }
}

impl Default for ToolpathPattern {
    fn default() -> Self {
        ToolpathPattern::Contour
    }
}

/// Configuration for toolpath generation
#[derive(Debug, Clone)]
pub struct ToolpathConfig {
    pub pattern: ToolpathPattern,
    pub line_width: f64,         // Spacing between adjacent paths (mm)
    pub node_distance: f64,      // Spacing between nodes along path (mm)
    pub infill_density: f64,     // 0.0 to 1.0 for infill patterns
    pub wall_count: usize,       // Number of perimeter loops (1-10)
    pub infill_pattern: InfillPattern, // Pattern for interior fill
    pub print_speed: f64,        // Wall/perimeter speed (mm/s)
    pub infill_speed: f64,       // Infill extrusion speed (mm/s)
    pub travel_speed: f64,       // Non-extrusion travel speed (mm/s)
    pub skip_infill: bool,       // Skip infill generation (for geodesic/coord-transform modes)
    pub wall_transitions: bool,  // Insert ruled-surface seam paths between consecutive curved layers
    /// Apply mesh ray-cast Z projection to wall and infill points.
    /// Only correct for geodesic mode where the toolpath should lie on the mesh surface.
    /// Must NOT be enabled for conical / S4 / cylindrical / spherical modes because those
    /// methods compute their own geometrically-correct Z values (back-transform / un-deform /
    /// coordinate-transform) — overwriting them with raw mesh surface Z corrupts the paths.
    pub use_mesh_raycaster: bool,
    /// Conical back-transform parameters: (center_x, center_y, signed_tan, z_min).
    /// `signed_tan = sign × tan(angle)`, where sign = -1 for Outward, +1 for Inward.
    /// `z_min` = mesh.bounds_min.z (build plate Z) — prevents infill from going below the bed.
    /// When Some, infill and wall-loop Z values are computed analytically:
    ///   z(x,y) = max(deformed_z - signed_tan × sqrt((x−cx)² + (y−cy)²), z_min)
    /// replacing IDW which gives badly wrong Z for interior points of large cross-sections.
    pub conical_params: Option<(f64, f64, f64, f64)>,
}

impl Default for ToolpathConfig {
    fn default() -> Self {
        Self {
            pattern: ToolpathPattern::Contour,
            line_width: 0.4,
            node_distance: 1.0,
            infill_density: 0.2,
            wall_count: 2,
            infill_pattern: InfillPattern::Rectilinear,
            print_speed: 50.0,
            infill_speed: 40.0,
            travel_speed: 80.0,
            skip_infill: false,
            wall_transitions: false,
            use_mesh_raycaster: false,
            conical_params: None,
        }
    }
}

impl Default for InfillPattern {
    fn default() -> Self {
        InfillPattern::Rectilinear
    }
}

/// Generate spiral toolpath pattern
pub fn generate_spiral_pattern(
    layer: &Layer,
    config: &ToolpathConfig,
) -> Vec<Point3D> {
    let mut points = Vec::new();

    if layer.contours.is_empty() {
        return points;
    }

    // Get the outermost contour
    let outer_contour = &layer.contours[0];
    if outer_contour.points.is_empty() {
        return points;
    }

    // Calculate bounding box
    let (min_x, max_x, min_y, max_y) = calculate_bounds(&outer_contour.points);
    let center_x = (min_x + max_x) / 2.0;
    let center_y = (min_y + max_y) / 2.0;

    // Calculate max radius from center
    let max_radius = outer_contour.points.iter()
        .map(|p| {
            let dx = p.x - center_x;
            let dy = p.y - center_y;
            (dx * dx + dy * dy).sqrt()
        })
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap_or(10.0);

    // Generate spiral from outside to inside
    let num_loops = (max_radius / config.line_width).ceil() as usize;

    for loop_idx in 0..num_loops {
        let radius = max_radius - (loop_idx as f64 * config.line_width);
        if radius < config.line_width / 2.0 {
            break;
        }

        // Number of points around this loop
        let circumference = 2.0 * std::f64::consts::PI * radius;
        let num_points = (circumference / config.node_distance).max(12.0) as usize;

        for i in 0..num_points {
            let angle = (i as f64 / num_points as f64) * 2.0 * std::f64::consts::PI;

            // Spiral gradually inward
            let current_radius = radius - (i as f64 / num_points as f64) * config.line_width;

            let x = center_x + current_radius * angle.cos();
            let y = center_y + current_radius * angle.sin();

            points.push(Point3D::new(x, y, layer.z));
        }
    }

    // If no points were generated (shouldn't happen), fall back to contour
    if points.is_empty() {
        points = outer_contour.points.clone();
    }

    points
}

/// Generate zigzag toolpath pattern
pub fn generate_zigzag_pattern(
    layer: &Layer,
    config: &ToolpathConfig,
) -> Vec<Point3D> {
    let mut points = Vec::new();

    if layer.contours.is_empty() {
        return points;
    }

    let outer_contour = &layer.contours[0];
    if outer_contour.points.is_empty() {
        return points;
    }

    // Calculate bounding box
    let (min_x, max_x, min_y, max_y) = calculate_bounds(&outer_contour.points);

    // Calculate line spacing based on infill density
    // Lower density = wider spacing
    let line_spacing = if config.infill_density > 0.01 {
        config.line_width / config.infill_density
    } else {
        config.line_width * 2.0
    };

    // Generate horizontal zigzag lines
    let mut y = min_y;
    let mut direction = 1.0; // 1.0 for left-to-right, -1.0 for right-to-left

    while y <= max_y {
        // Generate points along this Y line
        let mut x = min_x;
        let mut line_points = Vec::new();

        while x <= max_x {
            line_points.push(Point3D::new(x, y, layer.z));
            x += config.node_distance;
        }

        // Add final point at max_x if we didn't reach it
        if !line_points.is_empty() {
            let last_x = line_points.last().unwrap().x;
            if (max_x - last_x) > config.node_distance * 0.5 {
                line_points.push(Point3D::new(max_x, y, layer.z));
            }
        }

        // Add points in zigzag order
        if !line_points.is_empty() {
            if direction > 0.0 {
                // Left to right
                points.extend(line_points);
            } else {
                // Right to left
                points.extend(line_points.into_iter().rev());
            }
        }

        y += line_spacing;
        direction *= -1.0; // Reverse direction for zigzag
    }

    // If no points were generated (shouldn't happen), fall back to contour
    if points.is_empty() {
        points = outer_contour.points.clone();
    }

    points
}

/// Generate contour-based toolpath (existing method, improved)
pub fn generate_contour_pattern(
    layer: &Layer,
    config: &ToolpathConfig,
) -> Vec<Point3D> {
    let mut points = Vec::new();

    for contour in &layer.contours {
        // Sample points along contour based on node_distance
        let contour_points = resample_contour(&contour.points, config.node_distance);
        points.extend(contour_points);
    }

    points
}

/// Resample contour with specific spacing
pub fn resample_contour(points: &[Point3D], spacing: f64) -> Vec<Point3D> {
    if points.len() < 2 {
        return points.to_vec();
    }

    let mut resampled = Vec::new();
    resampled.push(points[0]);

    let mut accumulated_distance = 0.0;

    for i in 0..points.len().saturating_sub(1) {
        let p1 = points[i];
        let p2 = points[i + 1];
        let segment_length = (p2 - p1).norm();

        accumulated_distance += segment_length;

        while accumulated_distance >= spacing {
            let t = (accumulated_distance - spacing) / segment_length;
            let interpolated = Point3D::new(
                p1.x + (p2.x - p1.x) * (1.0 - t),
                p1.y + (p2.y - p1.y) * (1.0 - t),
                p1.z + (p2.z - p1.z) * (1.0 - t),
            );
            resampled.push(interpolated);
            accumulated_distance -= spacing;
        }
    }

    resampled
}

/// Calculate bounding box of points
fn calculate_bounds(points: &[Point3D]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;

    for p in points {
        min_x = min_x.min(p.x);
        max_x = max_x.max(p.x);
        min_y = min_y.min(p.y);
        max_y = max_y.max(p.y);
    }

    (min_x, max_x, min_y, max_y)
}

// ─── MeshRayCaster ────────────────────────────────────────────────────────────

/// 2D spatial bin grid for fast vertical ray–triangle intersection.
/// For any XY query, finds all triangles covering that position and returns
/// the surface Z nearest to a reference Z (handles multi-shell objects correctly).
pub struct MeshRayCaster {
    bins: Vec<Vec<usize>>,
    x_min: f64,
    y_min: f64,
    bin_size: f64,
    nx: usize,
    ny: usize,
}

impl MeshRayCaster {
    /// Build the caster from a mesh with a `grid_res × grid_res` bin grid.
    pub fn new(mesh: &Mesh, grid_res: usize) -> Self {
        let grid_res = grid_res.max(1);
        let x_min = mesh.bounds_min.x;
        let y_min = mesh.bounds_min.y;
        let x_max = mesh.bounds_max.x;
        let y_max = mesh.bounds_max.y;
        let span_x = (x_max - x_min).max(1e-6);
        let span_y = (y_max - y_min).max(1e-6);
        // Slightly oversize bins to avoid boundary off-by-one
        let bin_size = (span_x.max(span_y) / grid_res as f64) * 1.001;
        let nx = ((span_x / bin_size).ceil() as usize).max(1);
        let ny = ((span_y / bin_size).ceil() as usize).max(1);

        let mut bins: Vec<Vec<usize>> = vec![Vec::new(); nx * ny];
        for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
            let tx0 = tri.v0.x.min(tri.v1.x).min(tri.v2.x);
            let tx1 = tri.v0.x.max(tri.v1.x).max(tri.v2.x);
            let ty0 = tri.v0.y.min(tri.v1.y).min(tri.v2.y);
            let ty1 = tri.v0.y.max(tri.v1.y).max(tri.v2.y);

            let bx0 = ((tx0 - x_min) / bin_size).floor() as isize;
            let bx1 = ((tx1 - x_min) / bin_size).ceil() as isize;
            let by0 = ((ty0 - y_min) / bin_size).floor() as isize;
            let by1 = ((ty1 - y_min) / bin_size).ceil() as isize;

            for bx in bx0.max(0)..(bx1 + 1).min(nx as isize) {
                for by in by0.max(0)..(by1 + 1).min(ny as isize) {
                    bins[bx as usize + by as usize * nx].push(tri_idx);
                }
            }
        }

        Self { bins, x_min, y_min, bin_size, nx, ny }
    }

    /// Find mesh surface Z nearest to `near_z` at (x, y).
    /// Returns `None` if no triangle covers this XY position.
    pub fn project_z(&self, x: f64, y: f64, near_z: f64, mesh: &Mesh) -> Option<f64> {
        let bx = ((x - self.x_min) / self.bin_size) as isize;
        let by = ((y - self.y_min) / self.bin_size) as isize;
        if bx < 0 || bx >= self.nx as isize || by < 0 || by >= self.ny as isize {
            return None;
        }
        let bin = &self.bins[bx as usize + by as usize * self.nx];
        let mut best: Option<f64> = None;
        for &tri_idx in bin {
            let tri = &mesh.triangles[tri_idx];
            if let Some(z_hit) = vertical_ray_triangle(x, y, tri) {
                best = Some(match best {
                    None => z_hit,
                    Some(prev) => {
                        if (z_hit - near_z).abs() < (prev - near_z).abs() { z_hit } else { prev }
                    }
                });
            }
        }
        best
    }
}

/// Vertical ray–triangle intersection using XY barycentric coordinates.
/// Returns the Z at which a vertical ray through (x, y) hits the triangle, or None.
fn vertical_ray_triangle(x: f64, y: f64, tri: &Triangle) -> Option<f64> {
    let (v0x, v0y, v0z) = (tri.v0.x, tri.v0.y, tri.v0.z);
    let (v1x, v1y, v1z) = (tri.v1.x, tri.v1.y, tri.v1.z);
    let (v2x, v2y, v2z) = (tri.v2.x, tri.v2.y, tri.v2.z);

    let denom = (v1y - v2y) * (v0x - v2x) + (v2x - v1x) * (v0y - v2y);
    if denom.abs() < 1e-12 {
        return None; // Degenerate triangle
    }
    let a = ((v1y - v2y) * (x - v2x) + (v2x - v1x) * (y - v2y)) / denom;
    let b = ((v2y - v0y) * (x - v2x) + (v0x - v2x) * (y - v2y)) / denom;
    let c = 1.0 - a - b;

    const EPS: f64 = -1e-6;
    if a >= EPS && b >= EPS && c >= EPS {
        Some(a * v0z + b * v1z + c * v2z)
    } else {
        None
    }
}

// ─── Gap-fill helpers ─────────────────────────────────────────────────────────

/// Generate clipped infill scanlines at a single Y level with raycaster Z projection.
/// Used by coverage_gap_fill to insert extra scanlines on steep surfaces.
fn scanlines_at_y(
    y: f64,
    boundary_pts: &[Point3D],
    config: &ToolpathConfig,
    layer_z: f64,
    caster: &MeshRayCaster,
    mesh_ref: &Mesh,
) -> Vec<Vec<Point3D>> {
    let n = boundary_pts.len();
    if n < 3 { return Vec::new(); }
    let margin = config.line_width * 0.1;
    let mut intersections: Vec<f64> = Vec::new();

    for i in 0..n {
        let j = (i + 1) % n;
        let yi = boundary_pts[i].y;
        let yj = boundary_pts[j].y;
        if (yi <= y && yj > y) || (yj <= y && yi > y) {
            let t = (y - yi) / (yj - yi);
            let x = boundary_pts[i].x + t * (boundary_pts[j].x - boundary_pts[i].x);
            intersections.push(x);
        }
    }
    intersections.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let mut lines: Vec<Vec<Point3D>> = Vec::new();
    let mut i = 0;
    while i + 1 < intersections.len() {
        let x_start = intersections[i] + margin;
        let x_end = intersections[i + 1] - margin;
        if x_end > x_start + config.line_width {
            let mut pts = Vec::new();
            let mut x = x_start;
            while x <= x_end {
                let z = caster.project_z(x, y, layer_z, mesh_ref).unwrap_or(layer_z);
                pts.push(Point3D::new(x, y, z));
                x += config.node_distance;
            }
            if pts.last().map(|p| (x_end - p.x).abs() > config.node_distance * 0.3).unwrap_or(true) {
                let z = caster.project_z(x_end, y, layer_z, mesh_ref).unwrap_or(layer_z);
                pts.push(Point3D::new(x_end, y, z));
            }
            if pts.len() >= 2 {
                lines.push(pts);
            }
        }
        i += 2;
    }
    lines
}

/// One-pass coverage gap fill: for each consecutive pair of scanlines, check if their
/// 3D separation (accounting for slope-driven Z difference) exceeds 2× the target spacing.
/// If so, insert a midpoint scanline. Only runs when raycaster is available.
fn coverage_gap_fill(
    infill_lines: &[Vec<Point3D>],
    boundary_pts: &[Point3D],
    config: &ToolpathConfig,
    layer_z: f64,
    nominal_spacing: f64,
    caster: &MeshRayCaster,
    mesh_ref: &Mesh,
) -> Vec<Vec<Point3D>> {
    if infill_lines.len() < 2 {
        return Vec::new();
    }
    let target_3d = nominal_spacing * 2.0;
    let mut extra: Vec<Vec<Point3D>> = Vec::new();

    for i in 0..infill_lines.len() - 1 {
        let y_i = match infill_lines[i].first() { Some(p) => p.y, None => continue };
        let y_j = match infill_lines[i + 1].first() { Some(p) => p.y, None => continue };

        let dy = (y_j - y_i).abs();
        // Only check pairs that are nominally adjacent (within 50% of expected spacing)
        if dy < nominal_spacing * 0.5 || dy > nominal_spacing * 1.5 {
            continue;
        }

        // Sample mesh Z at the horizontal midpoint of scanline[i] to estimate slope
        let mid_x = infill_lines[i]
            .get(infill_lines[i].len() / 2)
            .map(|p| p.x)
            .unwrap_or(0.0);
        let z_i = caster.project_z(mid_x, y_i, layer_z, mesh_ref).unwrap_or(layer_z);
        let z_j = caster.project_z(mid_x, y_j, layer_z, mesh_ref).unwrap_or(layer_z);

        let dz = (z_j - z_i).abs();
        let gap_3d = (dy * dy + dz * dz).sqrt();

        if gap_3d > target_3d {
            let y_mid = (y_i + y_j) / 2.0;
            extra.extend(scanlines_at_y(y_mid, boundary_pts, config, layer_z, caster, mesh_ref));
        }
    }

    extra
}

// ─── Infill generation ────────────────────────────────────────────────────────

/// Collect all X intersections of a closed/open polygon at scan-line Y.
/// Returns a sorted vec of X coordinates (even count → inside/outside pairs).
fn scanline_x_at_y(pts: &[Point3D], closed: bool, y: f64) -> Vec<f64> {
    let n = pts.len();
    let mut xs = Vec::new();
    for i in 0..n {
        let j = if closed {
            (i + 1) % n
        } else if i + 1 < n {
            i + 1
        } else {
            break;
        };
        let yi = pts[i].y;
        let yj = pts[j].y;
        if (yi <= y && yj > y) || (yj <= y && yi > y) {
            let t = (y - yi) / (yj - yi);
            xs.push(pts[i].x + t * (pts[j].x - pts[i].x));
        }
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    xs
}

/// Generate infill lines clipped to the interior of a boundary contour.
/// Uses scanline approach: horizontal lines at spacing, clipped to boundary edges.
///
/// Dual-clip: each scanline segment is clipped to BOTH `boundary` (the innermost wall
/// loop) AND `outer_boundary` (the original contour).  This ensures infill never
/// escapes the model shell even when the 2D inward-offset algorithm places a wall-loop
/// vertex slightly outside the original contour for non-convex geometries (e.g. conical
/// slicing with complex cross-sections).
///
/// `raycaster` (when Some) gives accurate surface Z from the mesh; falls back to IDW /
/// boundary interpolation otherwise.
/// `z_override` (when Some) overrides ALL other Z methods — used for conical mode where
/// the exact analytic formula is known.
/// Returns individual line segments (each is a vec of points to extrude along).
pub fn generate_clipped_infill(
    boundary: &Contour,
    config: &ToolpathConfig,
    layer_z: f64,
    outer_boundary: Option<&Contour>,
    z_ref_points: Option<&[Point3D]>,
    raycaster: Option<(&MeshRayCaster, &Mesh)>,
    z_override: Option<&dyn Fn(f64, f64) -> f64>,
) -> Vec<Vec<Point3D>> {
    let pts = &boundary.points;
    if pts.len() < 3 || config.infill_density < 0.01 {
        return Vec::new();
    }

    let (_min_x, _max_x, min_y, max_y) = calculate_bounds(pts);

    // Line spacing based on density
    let line_spacing = config.line_width / config.infill_density;
    let margin = config.line_width * 0.1; // Small margin to avoid edge artifacts

    // Unified Z lookup priority:
    //   1. z_override (analytic formula — conical mode)
    //   2. raycaster  (mesh surface — geodesic mode)
    //   3. IDW from reference points
    //   4. nearest-edge boundary interpolation
    let compute_z = |x: f64, y: f64| -> f64 {
        if let Some(zfn) = z_override {
            return zfn(x, y);
        }
        if let Some((caster, mesh_ref)) = raycaster {
            if let Some(z) = caster.project_z(x, y, layer_z, mesh_ref) {
                return z;
            }
        }
        if let Some(refs) = z_ref_points {
            return interpolate_z_idw(x, y, refs, layer_z);
        }
        interpolate_z_for_infill(x, y, boundary, layer_z)
    };

    // Helper: emit a line from x_start..x_end at fixed y in the current direction.
    let emit_line = |x_start: f64, x_end: f64, y: f64, forward: bool, compute_z: &dyn Fn(f64,f64)->f64| -> Option<Vec<Point3D>> {
        if x_end <= x_start + config.line_width {
            return None;
        }
        let mut pts_out = Vec::new();
        if forward {
            let mut x = x_start;
            while x <= x_end {
                pts_out.push(Point3D::new(x, y, compute_z(x, y)));
                x += config.node_distance;
            }
            if pts_out.last().map(|p| (x_end - p.x).abs() > config.node_distance * 0.3).unwrap_or(true) {
                pts_out.push(Point3D::new(x_end, y, compute_z(x_end, y)));
            }
        } else {
            let mut x = x_end;
            while x >= x_start {
                pts_out.push(Point3D::new(x, y, compute_z(x, y)));
                x -= config.node_distance;
            }
            if pts_out.last().map(|p| (p.x - x_start).abs() > config.node_distance * 0.3).unwrap_or(true) {
                pts_out.push(Point3D::new(x_start, y, compute_z(x_start, y)));
            }
        }
        if pts_out.len() >= 2 { Some(pts_out) } else { None }
    };

    let mut infill_lines: Vec<Vec<Point3D>> = Vec::new();
    let mut direction_forward = true;

    // Generate scanlines
    let mut y = min_y + line_spacing / 2.0;
    while y < max_y {
        // Intersections with inner boundary (innermost wall loop)
        let inner_xs = scanline_x_at_y(pts, true, y);

        // Intersections with outer boundary (original contour) — used for dual-clipping.
        // If no outer boundary, fall back to inner only.
        let outer_xs: Vec<f64> = match outer_boundary {
            Some(outer) => scanline_x_at_y(&outer.points, outer.closed, y),
            None => Vec::new(),
        };

        let mut ii = 0;
        while ii + 1 < inner_xs.len() {
            let xi_s = inner_xs[ii] + margin;
            let xi_e = inner_xs[ii + 1] - margin;

            if outer_xs.len() >= 2 {
                // Dual-clip: intersect the inner pair against every outer pair.
                // For convex outer boundaries there is only one pair; non-convex boundaries
                // may have multiple pairs (holes, concavities).
                let mut oi = 0;
                while oi + 1 < outer_xs.len() {
                    let xo_s = outer_xs[oi] + margin;
                    let xo_e = outer_xs[oi + 1] - margin;
                    let x_start = xi_s.max(xo_s);
                    let x_end   = xi_e.min(xo_e);
                    if let Some(line) = emit_line(x_start, x_end, y, direction_forward, &compute_z) {
                        infill_lines.push(line);
                    }
                    oi += 2;
                }
            } else {
                // No outer boundary — clip to inner only (original behaviour)
                if let Some(line) = emit_line(xi_s, xi_e, y, direction_forward, &compute_z) {
                    infill_lines.push(line);
                }
            }

            ii += 2;
        }

        direction_forward = !direction_forward;
        y += line_spacing;
    }

    // Coverage gap fill: insert midpoint scanlines where 3D slope creates gaps > 2× target.
    // Only runs when the mesh raycaster is available (curved layers).
    if let Some((caster, mesh_ref)) = raycaster {
        let extra = coverage_gap_fill(
            &infill_lines, pts, config, layer_z, line_spacing, caster, mesh_ref,
        );
        if !extra.is_empty() {
            infill_lines.extend(extra);
            infill_lines.sort_by(|a, b| {
                let ya = a.first().map(|p| p.y).unwrap_or(0.0);
                let yb = b.first().map(|p| p.y).unwrap_or(0.0);
                ya.partial_cmp(&yb).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
    }

    // Final safety filter: reject any infill line whose every point is not inside
    // the outer boundary.  The dual-clip above handles the common case; this catches
    // any remaining escapes from floating-point rounding or concave degenerate cases.
    if let Some(outer) = outer_boundary {
        infill_lines.retain(|line| {
            line.len() >= 2 && line.iter().all(|pt| is_point_in_contour(pt, outer))
        });
    }

    infill_lines
}

/// Interpolate Z for infill point — uses boundary contour for non-planar layers,
/// falls back to layer_z for planar layers.
fn interpolate_z_for_infill(x: f64, y: f64, boundary: &Contour, layer_z: f64) -> f64 {
    // Check if this is a non-planar layer (varying Z in contour)
    let pts = &boundary.points;
    if pts.len() < 2 {
        return layer_z;
    }

    let z_min = pts.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    let z_max = pts.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);

    if (z_max - z_min) < 0.01 {
        // Planar layer
        return layer_z;
    }

    // Non-planar: interpolate from nearest boundary edge
    crate::contour_offset::interpolate_z_from_contour_pub(x, y, boundary)
}

/// Inverse-distance weighted Z interpolation from a set of reference points.
///
/// Uses the K=8 nearest points (by XY distance) with 1/d² weighting.
/// Falls back to `layer_z` for planar layers (Z range < 0.01mm).
fn interpolate_z_idw(x: f64, y: f64, ref_points: &[Point3D], layer_z: f64) -> f64 {
    if ref_points.is_empty() {
        return layer_z;
    }
    let z_min = ref_points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    let z_max = ref_points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
    if z_max - z_min < 0.01 {
        return layer_z; // planar layer — skip expensive IDW
    }

    const K: usize = 8;
    let mut heap: Vec<(f64, f64)> = ref_points.iter()
        .map(|p| ((p.x - x).powi(2) + (p.y - y).powi(2), p.z))
        .collect();
    heap.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    heap.truncate(K);

    let (mut wz, mut tw) = (0.0f64, 0.0f64);
    for (d2, z) in &heap {
        let w = 1.0 / (d2 + 1e-9);
        wz += w * z;
        tw += w;
    }
    if tw > 1e-12 { wz / tw } else { layer_z }
}

/// Check if a point is inside a contour using ray casting (XY only, ignores Z).
pub fn is_point_in_contour(point: &Point3D, contour: &Contour) -> bool {
    if contour.points.len() < 3 {
        return false;
    }

    let mut inside = false;
    let n = contour.points.len();

    for i in 0..n {
        let j = (i + 1) % n;
        let pi = &contour.points[i];
        let pj = &contour.points[j];

        if ((pi.y > point.y) != (pj.y > point.y)) &&
           (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x) {
            inside = !inside;
        }
    }

    inside
}
