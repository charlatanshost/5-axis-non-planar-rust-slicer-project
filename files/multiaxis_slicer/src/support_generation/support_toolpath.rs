// Support toolpath generation
// Converts support tree skeleton into printable toolpaths

use crate::geometry::{Point3D, Vector3D};
use crate::support_generation::tree_skeleton::{SupportTree, SupportNode};
use crate::toolpath::{Toolpath, ToolpathSegment};
use std::collections::HashMap;

/// Configuration for support toolpath generation
#[derive(Debug, Clone)]
pub struct SupportToolpathConfig {
    /// Layer height for support structures (mm)
    pub layer_height: f64,

    /// Extrusion width for support material (mm)
    pub extrusion_width: f64,

    /// Infill density for support (0.0 to 1.0)
    pub infill_density: f64,

    /// Number of perimeters around support structures
    pub num_perimeters: usize,

    /// Feedrate for support printing (mm/s)
    pub feedrate: f64,

    /// Filament diameter (mm)
    pub filament_diameter: f64,
}

impl Default for SupportToolpathConfig {
    fn default() -> Self {
        Self {
            layer_height: 0.2,
            extrusion_width: 0.4,
            infill_density: 0.15,  // 15% infill typical for supports
            num_perimeters: 1,
            feedrate: 50.0,
            filament_diameter: 1.75,
        }
    }
}

/// Generate toolpaths from support tree skeleton
pub fn generate_support_toolpaths(
    support_tree: &SupportTree,
    config: &SupportToolpathConfig,
) -> Vec<Toolpath> {
    // Group nodes by Z-height (layer)
    let layers = group_nodes_by_layer(&support_tree.nodes, config.layer_height);

    // Generate toolpath for each layer
    layers
        .into_iter()
        .map(|(z, nodes)| generate_layer_toolpath(&nodes, z, config))
        .collect()
}

/// Group support nodes by layer height
fn group_nodes_by_layer(
    nodes: &[SupportNode],
    layer_height: f64,
) -> Vec<(f64, Vec<SupportNode>)> {
    let mut layer_map: HashMap<i32, Vec<SupportNode>> = HashMap::new();

    for node in nodes {
        // Round Z position to nearest layer
        let layer_index = (node.position.z / layer_height).round() as i32;
        layer_map.entry(layer_index).or_insert_with(Vec::new).push(node.clone());
    }

    // Convert to sorted vector
    let mut layers: Vec<(f64, Vec<SupportNode>)> = layer_map
        .into_iter()
        .map(|(idx, nodes)| (idx as f64 * layer_height, nodes))
        .collect();

    layers.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    layers
}

/// Generate toolpath for a single support layer
fn generate_layer_toolpath(
    nodes: &[SupportNode],
    z: f64,
    config: &SupportToolpathConfig,
) -> Toolpath {
    let mut paths = Vec::new();

    // For each support node at this layer, generate a circular perimeter
    for node in nodes {
        let center = Point3D::new(node.position.x, node.position.y, z);
        let radius = node.radius;

        // Generate circular perimeter around node
        let perimeter_points = generate_circle_points(center, radius, 16);

        // Generate perimeter toolpath segments
        for i in 0..perimeter_points.len() {
            let p1 = perimeter_points[i];
            let p2 = perimeter_points[(i + 1) % perimeter_points.len()];

            let distance = (p2 - p1).norm();
            let extrusion = calculate_extrusion(
                distance,
                config.extrusion_width,
                config.layer_height,
                config.filament_diameter,
            );

            // `position` is the destination of the move (see ToolpathSegment).
            // The loop wraps with `%`, so using p2 still emits every point of
            // the circle -- it just attributes the flow to the correct move.
            paths.push(ToolpathSegment {
                position: p2,
                orientation: Vector3D::new(0.0, 0.0, 1.0),  // Vertical for supports
                extrusion,
                feedrate: config.feedrate,
            });
        }

        // Generate infill inside support column (zigzag pattern)
        if config.infill_density > 0.01 {
            let infill_paths = generate_support_infill(center, radius, z, config);
            paths.extend(infill_paths);
        }
    }

    Toolpath {
        paths,
        z,
        layer_height: 0.2,
    }
}

/// Generate circular perimeter points
fn generate_circle_points(center: Point3D, radius: f64, segments: usize) -> Vec<Point3D> {
    let mut points = Vec::new();

    for i in 0..segments {
        let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
        let x = center.x + radius * angle.cos();
        let y = center.y + radius * angle.sin();
        points.push(Point3D::new(x, y, center.z));
    }

    points
}

/// Generate zigzag infill inside support column
fn generate_support_infill(
    center: Point3D,
    radius: f64,
    z: f64,
    config: &SupportToolpathConfig,
) -> Vec<ToolpathSegment> {
    let mut segments = Vec::new();

    // Calculate line spacing based on infill density
    let line_spacing = config.extrusion_width / config.infill_density.max(0.05);

    // Generate horizontal zigzag lines
    let mut y = center.y - radius;
    let mut direction = 1.0;

    while y <= center.y + radius {
        // Calculate X extent at this Y position (circle intersection)
        let dy = (y - center.y).abs();
        if dy > radius {
            y += line_spacing;
            continue;
        }

        let dx = (radius * radius - dy * dy).sqrt();
        let x_start = center.x - dx * direction;
        let x_end = center.x + dx * direction;

        let p1 = Point3D::new(x_start, y, z);
        let p2 = Point3D::new(x_end, y, z);

        let distance = (p2 - p1).norm();
        let extrusion = calculate_extrusion(
            distance,
            config.extrusion_width,
            config.layer_height,
            config.filament_diameter,
        );

        // `position` is the destination of the move and `extrusion` is the
        // volume laid down *reaching* it (see ToolpathSegment). Both segments
        // used to carry the full scan-line extrusion, so every line was
        // extruded twice: once on the connector hop in from the previous
        // line's end, and again along the line itself.
        //
        // The hop to p1 crosses the column at `line_spacing` (0.4 / 0.15 is
        // ~2.7 mm at the default density), so it is a travel, not a bridge.
        segments.push(ToolpathSegment {
            position: p1,
            orientation: Vector3D::new(0.0, 0.0, 1.0),
            extrusion: 0.0,
            feedrate: config.feedrate,
        });

        segments.push(ToolpathSegment {
            position: p2,
            orientation: Vector3D::new(0.0, 0.0, 1.0),
            extrusion,
            feedrate: config.feedrate,
        });

        // Reverse direction for next line (zigzag)
        direction *= -1.0;
        y += line_spacing;
    }

    segments
}

/// Calculate extrusion amount based on volume
fn calculate_extrusion(
    distance: f64,
    width: f64,
    height: f64,
    filament_diameter: f64,
) -> f64 {
    // Volume of extruded material
    let extrusion_volume = width * height * distance;

    // Convert to filament length
    let filament_area = std::f64::consts::PI * (filament_diameter / 2.0).powi(2);

    extrusion_volume / filament_area
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circle_generation() {
        let center = Point3D::new(0.0, 0.0, 5.0);
        let radius = 10.0;
        let points = generate_circle_points(center, radius, 8);

        assert_eq!(points.len(), 8);

        // Check that all points are roughly at the correct radius
        for point in &points {
            let distance = ((point.x - center.x).powi(2) + (point.y - center.y).powi(2)).sqrt();
            assert!((distance - radius).abs() < 0.001);
        }
    }

    /// Regression test for D-01. Every infill scan line used to be emitted as
    /// two segments both carrying the *whole line's* extrusion, so each line
    /// was paid for twice -- once on the connector hop in from the previous
    /// line, once along the line itself. That is ~2x over-extrusion through
    /// the entire support column.
    #[test]
    fn test_infill_extrusion_matches_distance_actually_travelled() {
        let config = SupportToolpathConfig::default();
        let center = Point3D::new(0.0, 0.0, 0.0);
        let segments = generate_support_infill(center, 5.0, 0.0, &config);

        assert!(segments.len() >= 4, "expected several scan lines, got {}", segments.len());
        assert_eq!(segments.len() % 2, 0, "scan lines come in travel/extrude pairs");

        // Segments come as (connector hop, scan line). The hop across the
        // column is a travel; the scan line bills exactly its own length.
        for (pair_idx, pair) in segments.chunks(2).enumerate() {
            assert_eq!(
                pair[0].extrusion, 0.0,
                "connector hop into scan line {} should be a travel",
                pair_idx
            );

            let distance = (pair[1].position - pair[0].position).norm();
            let expected = calculate_extrusion(
                distance,
                config.extrusion_width,
                config.layer_height,
                config.filament_diameter,
            );
            assert!(
                (pair[1].extrusion - expected).abs() < 1e-12,
                "scan line {} extrudes {} over {} mm, expected {}",
                pair_idx,
                pair[1].extrusion,
                distance,
                expected
            );
        }

        // And the total is the scan-line length alone, not the round trip.
        let scan_length: f64 = segments
            .chunks(2)
            .map(|pair| (pair[1].position - pair[0].position).norm())
            .sum();
        let total: f64 = segments.iter().map(|s| s.extrusion).sum();
        let expected_total = calculate_extrusion(
            scan_length,
            config.extrusion_width,
            config.layer_height,
            config.filament_diameter,
        );
        assert!(
            (total - expected_total).abs() < 1e-12,
            "total extrusion {} should equal {} for {} mm of scan lines",
            total,
            expected_total,
            scan_length
        );
    }

    #[test]
    fn test_extrusion_calculation() {
        let distance = 10.0;
        let width = 0.4;
        let height = 0.2;
        let filament_diameter = 1.75;

        let extrusion = calculate_extrusion(distance, width, height, filament_diameter);

        // Should be positive
        assert!(extrusion > 0.0);

        // Rough sanity check (extrusion should be less than distance for typical values)
        assert!(extrusion < distance);
    }
}
