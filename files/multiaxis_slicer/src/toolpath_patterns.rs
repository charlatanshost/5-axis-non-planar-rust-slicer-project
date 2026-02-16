// Advanced toolpath pattern generation
// Based on algorithms from SIGGRAPH Asia 2022 paper

use crate::geometry::{Point3D, Vector3D, Contour};
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

/// Generate infill lines clipped to the interior of a boundary contour.
/// Uses scanline approach: horizontal lines at spacing, clipped to boundary edges.
/// `outer_boundary` is used to validate that infill lines stay inside the original contour.
/// Returns individual line segments (each is a vec of points to extrude along).
pub fn generate_clipped_infill(
    boundary: &Contour,
    config: &ToolpathConfig,
    layer_z: f64,
    outer_boundary: Option<&Contour>,
) -> Vec<Vec<Point3D>> {
    let pts = &boundary.points;
    if pts.len() < 3 || config.infill_density < 0.01 {
        return Vec::new();
    }

    let (min_x, max_x, min_y, max_y) = calculate_bounds(pts);

    // Line spacing based on density
    let line_spacing = config.line_width / config.infill_density;
    let margin = config.line_width * 0.1; // Small margin to avoid edge artifacts

    let mut infill_lines: Vec<Vec<Point3D>> = Vec::new();
    let mut direction_forward = true;

    // Generate scanlines
    let mut y = min_y + line_spacing / 2.0;
    while y < max_y {
        // Find all intersections of horizontal line y with boundary edges
        let mut intersections: Vec<f64> = Vec::new();
        let n = pts.len();

        for i in 0..n {
            let j = (i + 1) % n;
            let yi = pts[i].y;
            let yj = pts[j].y;

            // Check if this edge crosses the scanline
            if (yi <= y && yj > y) || (yj <= y && yi > y) {
                // Compute x intersection
                let t = (y - yi) / (yj - yi);
                let x = pts[i].x + t * (pts[j].x - pts[i].x);
                intersections.push(x);
            }
        }

        // Sort intersections by x
        intersections.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Pair up entry/exit: intersections[0..1] is inside, [1..2] is outside, etc.
        let mut i = 0;
        while i + 1 < intersections.len() {
            let x_start = intersections[i] + margin;
            let x_end = intersections[i + 1] - margin;

            if x_end > x_start + config.line_width {
                // Generate points along this infill segment
                let mut line_points = Vec::new();

                if direction_forward {
                    let mut x = x_start;
                    while x <= x_end {
                        let z = interpolate_z_for_infill(x, y, boundary, layer_z);
                        line_points.push(Point3D::new(x, y, z));
                        x += config.node_distance;
                    }
                    // Add final point
                    if line_points.last().map(|p| (x_end - p.x).abs() > config.node_distance * 0.3).unwrap_or(true) {
                        let z = interpolate_z_for_infill(x_end, y, boundary, layer_z);
                        line_points.push(Point3D::new(x_end, y, z));
                    }
                } else {
                    let mut x = x_end;
                    while x >= x_start {
                        let z = interpolate_z_for_infill(x, y, boundary, layer_z);
                        line_points.push(Point3D::new(x, y, z));
                        x -= config.node_distance;
                    }
                    if line_points.last().map(|p| (p.x - x_start).abs() > config.node_distance * 0.3).unwrap_or(true) {
                        let z = interpolate_z_for_infill(x_start, y, boundary, layer_z);
                        line_points.push(Point3D::new(x_start, y, z));
                    }
                }

                if line_points.len() >= 2 {
                    infill_lines.push(line_points);
                }
            }

            i += 2; // Skip to next entry/exit pair
        }

        direction_forward = !direction_forward; // Zigzag alternation
        y += line_spacing;
    }

    // Validate infill lines against the outer boundary (original contour) if provided.
    // This catches cases where the offset contour is malformed for complex shapes.
    if let Some(outer) = outer_boundary {
        infill_lines.retain(|line| {
            if line.len() < 2 {
                return false;
            }
            // Check midpoint of the line — if it's outside the original contour, discard
            let mid_idx = line.len() / 2;
            let mid = &line[mid_idx];
            if !is_point_in_contour(mid, outer) {
                return false;
            }
            // Also check endpoints
            is_point_in_contour(&line[0], outer) && is_point_in_contour(line.last().unwrap(), outer)
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

/// Check if a point is inside a contour using ray casting
fn is_point_in_contour(point: &Point3D, contour: &Contour) -> bool {
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
