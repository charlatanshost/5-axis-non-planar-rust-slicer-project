// Marching Tetrahedra for Isosurface Extraction
//
// Extracts curved layer contours by intersecting tetrahedra with iso-values
// of the scalar field. Each tet has 4 vertices with scalar values, producing
// 16 possible configurations (2^4), reduced to a few cases by symmetry.
//
// Cases:
// - 0 or 4 vertices above: no intersection
// - 1 vertex above (or 3 below): triangle intersection (3 edge crossings)
// - 2 vertices above: quad intersection (4 edge crossings, split to 2 triangles)

use crate::geometry::{Point3D, LineSegment, Contour};
use crate::s3_slicer::tet_mesh::TetMesh;
use crate::s3_slicer::tet_scalar_field::TetScalarField;
use crate::s3_slicer::isosurface::CurvedLayer;

/// Extract curved layers from a tet mesh using marching tetrahedra
pub fn extract_tet_layers(
    tet_mesh: &TetMesh,
    scalar_field: &TetScalarField,
    layer_height: f64,
    min_contour_length: f64,
) -> Vec<CurvedLayer> {
    let height_range = scalar_field.max_value - scalar_field.min_value;
    let num_layers = (height_range / layer_height).ceil() as usize;

    log::info!("Extracting tet layers: {} layers (height range {:.2}, layer height {:.3})",
        num_layers, height_range, layer_height);

    let mut layers = Vec::with_capacity(num_layers);

    for i in 1..=num_layers {
        let iso_value = scalar_field.min_value + i as f64 * layer_height;

        if iso_value >= scalar_field.max_value {
            break;
        }

        let layer = extract_at_iso_value(tet_mesh, scalar_field, iso_value, min_contour_length);

        if !layer.segments.is_empty() {
            layers.push(layer);
        }

        if i % 50 == 0 {
            log::info!("  Extracted {}/{} layers...", i, num_layers);
        }
    }

    log::info!("  Total layers extracted: {}", layers.len());
    layers
}

/// Extract a single layer at a given iso-value
fn extract_at_iso_value(
    tet_mesh: &TetMesh,
    scalar_field: &TetScalarField,
    iso_value: f64,
    min_contour_length: f64,
) -> CurvedLayer {
    let mut segments: Vec<LineSegment> = Vec::new();

    // For each tetrahedron, find edge crossings
    for tet in &tet_mesh.tets {
        let v = &tet.vertices;

        // Get scalar values at each vertex
        let s = [
            scalar_field.value_at(v[0]),
            scalar_field.value_at(v[1]),
            scalar_field.value_at(v[2]),
            scalar_field.value_at(v[3]),
        ];

        // Classify vertices: above (true) or below (false) iso-value
        let above = [
            s[0] >= iso_value,
            s[1] >= iso_value,
            s[2] >= iso_value,
            s[3] >= iso_value,
        ];

        let count_above = above.iter().filter(|&&a| a).count();

        match count_above {
            0 | 4 => {
                // No intersection
                continue;
            }
            1 => {
                // One vertex above: triangle intersection (3 edge crossings)
                let above_idx = above.iter().position(|&a| a).unwrap();
                let below_indices: Vec<usize> = (0..4).filter(|&i| !above[i]).collect();

                // Find intersection points on 3 edges from above vertex to below vertices
                let mut points = Vec::with_capacity(3);
                for &bi in &below_indices {
                    if let Some(p) = interpolate_edge(
                        &tet_mesh.vertices[v[above_idx]],
                        &tet_mesh.vertices[v[bi]],
                        s[above_idx], s[bi], iso_value,
                    ) {
                        points.push(p);
                    }
                }

                // Create line segments forming the triangle
                add_triangle_segments(&points, &mut segments);
            }
            3 => {
                // Three vertices above: equivalent to one below (triangle intersection)
                let below_idx = above.iter().position(|&a| !a).unwrap();
                let above_indices: Vec<usize> = (0..4).filter(|&i| above[i]).collect();

                let mut points = Vec::with_capacity(3);
                for &ai in &above_indices {
                    if let Some(p) = interpolate_edge(
                        &tet_mesh.vertices[v[below_idx]],
                        &tet_mesh.vertices[v[ai]],
                        s[below_idx], s[ai], iso_value,
                    ) {
                        points.push(p);
                    }
                }

                add_triangle_segments(&points, &mut segments);
            }
            2 => {
                // Two vertices above: quad intersection (4 edge crossings)
                let above_indices: Vec<usize> = (0..4).filter(|&i| above[i]).collect();
                let below_indices: Vec<usize> = (0..4).filter(|&i| !above[i]).collect();

                // Find 4 intersection points on edges connecting above to below
                let mut points = Vec::with_capacity(4);
                for &ai in &above_indices {
                    for &bi in &below_indices {
                        if let Some(p) = interpolate_edge(
                            &tet_mesh.vertices[v[ai]],
                            &tet_mesh.vertices[v[bi]],
                            s[ai], s[bi], iso_value,
                        ) {
                            points.push(p);
                        }
                    }
                }

                // Create segments forming the quad (split into 2 triangles or keep as quad contour)
                add_quad_segments(&points, &mut segments);
            }
            _ => unreachable!(),
        }
    }

    // Compute average Z for the layer
    let avg_z = if segments.is_empty() {
        iso_value
    } else {
        let total_z: f64 = segments.iter()
            .map(|s| (s.start.z + s.end.z) / 2.0)
            .sum();
        total_z / segments.len() as f64
    };

    CurvedLayer {
        segments,
        z: avg_z,
        iso_value,
        normals: Vec::new(), // Normals can be computed later if needed
    }
}

/// Interpolate position along an edge where scalar field equals iso_value
fn interpolate_edge(
    p0: &Point3D,
    p1: &Point3D,
    s0: f64,
    s1: f64,
    iso_value: f64,
) -> Option<Point3D> {
    let ds = s1 - s0;
    if ds.abs() < 1e-12 {
        return None;
    }

    let t = (iso_value - s0) / ds;
    let t = t.clamp(0.0, 1.0);

    Some(Point3D::new(
        p0.x + t * (p1.x - p0.x),
        p0.y + t * (p1.y - p0.y),
        p0.z + t * (p1.z - p0.z),
    ))
}

/// Add triangle segments (3 edges of the intersection triangle)
fn add_triangle_segments(points: &[Point3D], segments: &mut Vec<LineSegment>) {
    if points.len() < 3 {
        return;
    }

    segments.push(LineSegment { start: points[0], end: points[1] });
    segments.push(LineSegment { start: points[1], end: points[2] });
    segments.push(LineSegment { start: points[2], end: points[0] });
}

/// Add quad segments (edges of the intersection quad)
fn add_quad_segments(points: &[Point3D], segments: &mut Vec<LineSegment>) {
    if points.len() < 4 {
        // Fall back to triangle if only 3 points
        add_triangle_segments(points, segments);
        return;
    }

    // Order quad points properly
    // The 4 points come from 2 above-to-2 below edges, in order:
    // [a0-b0, a0-b1, a1-b0, a1-b1]
    // We need to connect them as: p0-p1-p3-p2 (or similar ordering)
    segments.push(LineSegment { start: points[0], end: points[1] });
    segments.push(LineSegment { start: points[1], end: points[3] });
    segments.push(LineSegment { start: points[3], end: points[2] });
    segments.push(LineSegment { start: points[2], end: points[0] });
}

/// Assemble line segments into closed contours
fn assemble_contours(segments: &[LineSegment], min_length: f64) -> Vec<Contour> {
    if segments.is_empty() {
        return Vec::new();
    }

    // Build endpoint adjacency
    let tolerance = 1e-4; // Position tolerance for connecting segments
    let mut used = vec![false; segments.len()];
    let mut contours = Vec::new();

    for start_idx in 0..segments.len() {
        if used[start_idx] {
            continue;
        }

        used[start_idx] = true;
        let mut points = vec![segments[start_idx].start, segments[start_idx].end];

        // Try to extend the contour by finding connected segments
        let mut extended = true;
        while extended {
            extended = false;
            let current_end = *points.last().unwrap();

            for (si, seg) in segments.iter().enumerate() {
                if used[si] {
                    continue;
                }

                if distance_sq(&current_end, &seg.start) < tolerance * tolerance {
                    used[si] = true;
                    points.push(seg.end);
                    extended = true;
                    break;
                } else if distance_sq(&current_end, &seg.end) < tolerance * tolerance {
                    used[si] = true;
                    points.push(seg.start);
                    extended = true;
                    break;
                }
            }
        }

        // Check if contour is long enough
        let length: f64 = points.windows(2)
            .map(|w| ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2) + (w[1].z - w[0].z).powi(2)).sqrt())
            .sum();

        if length >= min_length && points.len() >= 3 {
            // Check if closed
            let is_closed = distance_sq(&points[0], points.last().unwrap()) < tolerance * tolerance;
            contours.push(Contour::new(points, is_closed));
        }
    }

    contours
}

fn distance_sq(a: &Point3D, b: &Point3D) -> f64 {
    (a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s3_slicer::tet_mesh::{Tetrahedron, TetMesh};
    use crate::s3_slicer::tet_scalar_field::{TetScalarField, TetScalarFieldConfig};

    #[test]
    fn test_interpolate_edge() {
        let p0 = Point3D::new(0.0, 0.0, 0.0);
        let p1 = Point3D::new(10.0, 0.0, 0.0);

        // Midpoint at iso=5
        let result = interpolate_edge(&p0, &p1, 0.0, 10.0, 5.0);
        assert!(result.is_some());
        let p = result.unwrap();
        assert!((p.x - 5.0).abs() < 1e-10);

        // Quarter point at iso=2.5
        let result2 = interpolate_edge(&p0, &p1, 0.0, 10.0, 2.5);
        assert!(result2.is_some());
        let p2 = result2.unwrap();
        assert!((p2.x - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_extract_single_tet() {
        // Create a tet with vertices at different Z heights
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),  // z=0
            Point3D::new(10.0, 0.0, 0.0), // z=0
            Point3D::new(5.0, 10.0, 0.0), // z=0
            Point3D::new(5.0, 5.0, 10.0), // z=10
        ];

        let tets = vec![Tetrahedron::new(0, 1, 2, 3)];
        let tet_mesh = TetMesh::new(vertices, tets);

        // Scalar field = Z coordinate (no smoothing)
        let config = TetScalarFieldConfig {
            smoothing_iterations: 0,
            ..Default::default()
        };
        let field = TetScalarField::from_deformed_mesh(&tet_mesh, &tet_mesh, &config);

        // Extract layer at Z=5 (midpoint)
        let layer = extract_at_iso_value(&tet_mesh, &field, 5.0, 0.1);

        // Should have intersection segments (triangle case: 1 above, 3 below)
        assert!(!layer.segments.is_empty(), "Should have segments at iso=5.0");
    }

    #[test]
    fn test_extract_multiple_layers() {
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(5.0, 10.0, 0.0),
            Point3D::new(5.0, 5.0, 10.0),
            Point3D::new(5.0, 5.0, -5.0),
        ];

        let tets = vec![
            Tetrahedron::new(0, 1, 2, 3),
            Tetrahedron::new(0, 1, 2, 4),
        ];

        let tet_mesh = TetMesh::new(vertices, tets);
        let config = TetScalarFieldConfig {
            smoothing_iterations: 0,
            ..Default::default()
        };
        let field = TetScalarField::from_deformed_mesh(&tet_mesh, &tet_mesh, &config);

        let layers = extract_tet_layers(&tet_mesh, &field, 2.0, 0.1);

        assert!(!layers.is_empty(), "Should extract at least one layer");
    }

    #[test]
    fn test_quad_intersection() {
        // Create tet where 2 vertices are above and 2 below the iso-value
        let vertices = vec![
            Point3D::new(0.0, 0.0, -5.0),  // below
            Point3D::new(10.0, 0.0, -5.0), // below
            Point3D::new(0.0, 10.0, 5.0),  // above
            Point3D::new(10.0, 10.0, 5.0), // above
        ];

        let tets = vec![Tetrahedron::new(0, 1, 2, 3)];
        let tet_mesh = TetMesh::new(vertices, tets);
        let config = TetScalarFieldConfig {
            smoothing_iterations: 0,
            ..Default::default()
        };
        let field = TetScalarField::from_deformed_mesh(&tet_mesh, &tet_mesh, &config);

        // iso=0 should give quad intersection (2 above, 2 below after normalization)
        let mid_iso = (field.min_value + field.max_value) / 2.0;
        let layer = extract_at_iso_value(&tet_mesh, &field, mid_iso, 0.01);

        assert!(!layer.segments.is_empty(), "Should have quad intersection segments");
    }
}
