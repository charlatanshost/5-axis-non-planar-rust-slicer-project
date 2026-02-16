//! 2D polygon inward offset for generating wall loops (perimeters).
//!
//! Algorithm: edge-normal offset with self-intersection cleanup.
//! For each edge, compute the inward normal, move the edge by `offset_distance`,
//! then compute intersection of adjacent offset edges to get new vertices.
//! Self-intersections are cleaned by detecting edge crossings and keeping the
//! largest valid loop.

use crate::geometry::{Point3D, Contour};

/// Compute the signed area of a 2D polygon (using only X,Y of points).
/// Positive = counter-clockwise (outer contour), Negative = clockwise (hole).
pub fn signed_area_2d(points: &[Point3D]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += points[i].x * points[j].y;
        area -= points[j].x * points[i].y;
    }
    area / 2.0
}

/// Compute intersection of two 2D line segments.
/// Returns Some((t, u)) where t is parameter on segment a1->a2, u on b1->b2.
/// Both t and u in [0,1] means the segments actually intersect.
fn segment_intersection_2d(
    a1x: f64, a1y: f64, a2x: f64, a2y: f64,
    b1x: f64, b1y: f64, b2x: f64, b2y: f64,
) -> Option<(f64, f64)> {
    let dx_a = a2x - a1x;
    let dy_a = a2y - a1y;
    let dx_b = b2x - b1x;
    let dy_b = b2y - b1y;

    let denom = dx_a * dy_b - dy_a * dx_b;
    if denom.abs() < 1e-12 {
        return None; // Parallel or coincident
    }

    let t = ((b1x - a1x) * dy_b - (b1y - a1y) * dx_b) / denom;
    let u = ((b1x - a1x) * dy_a - (b1y - a1y) * dx_a) / denom;

    Some((t, u))
}

/// Compute intersection point of two infinite lines defined by segments.
/// Returns None if lines are parallel.
fn line_intersection_2d(
    a1x: f64, a1y: f64, a2x: f64, a2y: f64,
    b1x: f64, b1y: f64, b2x: f64, b2y: f64,
) -> Option<(f64, f64)> {
    let dx_a = a2x - a1x;
    let dy_a = a2y - a1y;
    let dx_b = b2x - b1x;
    let dy_b = b2y - b1y;

    let denom = dx_a * dy_b - dy_a * dx_b;
    if denom.abs() < 1e-12 {
        return None;
    }

    let t = ((b1x - a1x) * dy_b - (b1y - a1y) * dx_b) / denom;
    let ix = a1x + t * dx_a;
    let iy = a1y + t * dy_a;
    Some((ix, iy))
}

/// Interpolate Z from the original contour for an offset (x,y) point.
/// Finds the nearest edge on the original contour and linearly interpolates Z.
fn interpolate_z_from_contour(x: f64, y: f64, original: &Contour) -> f64 {
    let pts = &original.points;
    if pts.is_empty() {
        return 0.0;
    }
    if pts.len() == 1 {
        return pts[0].z;
    }

    let mut best_dist_sq = f64::INFINITY;
    let mut best_z = pts[0].z;

    let n = pts.len();
    for i in 0..n {
        let j = if original.closed { (i + 1) % n } else if i + 1 < n { i + 1 } else { break };
        let ax = pts[i].x;
        let ay = pts[i].y;
        let bx = pts[j].x;
        let by = pts[j].y;

        let dx = bx - ax;
        let dy = by - ay;
        let len_sq = dx * dx + dy * dy;

        let t = if len_sq < 1e-12 {
            0.0
        } else {
            ((x - ax) * dx + (y - ay) * dy) / len_sq
        };
        let t = t.clamp(0.0, 1.0);

        let px = ax + t * dx;
        let py = ay + t * dy;
        let dist_sq = (x - px) * (x - px) + (y - py) * (y - py);

        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_z = pts[i].z + t * (pts[j].z - pts[i].z);
        }
    }

    best_z
}

/// Public wrapper for Z interpolation (used by toolpath_patterns for infill).
pub fn interpolate_z_from_contour_pub(x: f64, y: f64, contour: &Contour) -> f64 {
    interpolate_z_from_contour(x, y, contour)
}

/// Offset a single closed contour inward by `distance`.
/// Returns None if the contour collapses (too small for the offset).
pub fn offset_contour(contour: &Contour, distance: f64) -> Option<Contour> {
    let pts = &contour.points;
    let n = pts.len();
    if n < 3 {
        return None;
    }

    // Determine winding: positive area = CCW = outer contour
    let area = signed_area_2d(pts);
    if area.abs() < 1e-10 {
        return None; // Degenerate
    }

    // For CCW (outer), inward normal points left of edge direction
    // For CW (hole), inward normal points right of edge direction
    let sign = if area > 0.0 { 1.0 } else { -1.0 };

    // Compute offset edges: for each edge, move it inward by `distance`
    // Each offset edge is defined by two points (offset_a, offset_b)
    let mut offset_edges: Vec<((f64, f64), (f64, f64))> = Vec::with_capacity(n);

    for i in 0..n {
        let j = (i + 1) % n;
        let dx = pts[j].x - pts[i].x;
        let dy = pts[j].y - pts[i].y;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-12 {
            // Degenerate edge — use previous edge's normal or skip
            if !offset_edges.is_empty() {
                offset_edges.push(*offset_edges.last().unwrap());
            } else {
                offset_edges.push(((pts[i].x, pts[i].y), (pts[j].x, pts[j].y)));
            }
            continue;
        }

        // Inward normal: rotate edge direction 90° left (for CCW) or right (for CW)
        let nx = -dy / len * sign * distance;
        let ny = dx / len * sign * distance;

        offset_edges.push((
            (pts[i].x + nx, pts[i].y + ny),
            (pts[j].x + nx, pts[j].y + ny),
        ));
    }

    // Compute new vertices: intersection of adjacent offset edges
    let mut new_points_2d: Vec<(f64, f64)> = Vec::with_capacity(n);

    for i in 0..n {
        let j = (i + 1) % n;
        let (a1, a2) = offset_edges[i];
        let (b1, b2) = offset_edges[j];

        if let Some((ix, iy)) = line_intersection_2d(a1.0, a1.1, a2.0, a2.1, b1.0, b1.1, b2.0, b2.1) {
            new_points_2d.push((ix, iy));
        } else {
            // Parallel edges — use midpoint of endpoints
            new_points_2d.push(((a2.0 + b1.0) / 2.0, (a2.1 + b1.1) / 2.0));
        }
    }

    // Clean self-intersections
    clean_self_intersections(&mut new_points_2d);

    if new_points_2d.len() < 3 {
        return None; // Collapsed
    }

    // Verify the offset polygon has reasonable area (not inverted or not shrunk)
    let offset_area = signed_area_2d_raw(&new_points_2d);
    if (area > 0.0 && offset_area <= 0.0) || (area < 0.0 && offset_area >= 0.0) {
        return None; // Winding flipped — contour collapsed
    }
    // If the offset polygon didn't actually shrink, the offset exceeded the inradius
    if offset_area.abs() >= area.abs() - 1e-6 {
        return None;
    }

    // Bounding box sanity check: offset contour should be SMALLER than original
    let (orig_min_x, orig_max_x, orig_min_y, orig_max_y) = {
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        let mut min_y = f64::MAX;
        let mut max_y = f64::MIN;
        for p in pts {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        (min_x, max_x, min_y, max_y)
    };
    let tolerance = distance * 0.5; // Allow small overshoot from vertex expansion at sharp corners
    for &(x, y) in &new_points_2d {
        if x < orig_min_x - tolerance || x > orig_max_x + tolerance
            || y < orig_min_y - tolerance || y > orig_max_y + tolerance
        {
            return None; // Offset expanded beyond original — malformed
        }
    }

    // Restore Z coordinates by interpolating from the original contour
    let new_points: Vec<Point3D> = new_points_2d
        .iter()
        .map(|&(x, y)| {
            let z = interpolate_z_from_contour(x, y, contour);
            Point3D::new(x, y, z)
        })
        .collect();

    Some(Contour::new(new_points, contour.closed))
}

/// signed area from raw (f64,f64) tuples
fn signed_area_2d_raw(points: &[(f64, f64)]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += points[i].0 * points[j].1;
        area -= points[j].0 * points[i].1;
    }
    area / 2.0
}

/// Remove self-intersections from an offset polygon.
/// Walks edges, detects crossings, keeps the largest valid sub-loop.
fn clean_self_intersections(points: &mut Vec<(f64, f64)>) {
    if points.len() < 4 {
        return;
    }

    // Find all self-intersections
    let n = points.len();
    let mut _found_intersection = false;

    'outer: for i in 0..n {
        let i_next = (i + 1) % n;
        // Check against non-adjacent edges
        for j in (i + 2)..n {
            if j == n - 1 && i == 0 {
                continue; // Adjacent (last edge wraps to first vertex)
            }
            let j_next = (j + 1) % n;

            if let Some((t, u)) = segment_intersection_2d(
                points[i].0, points[i].1, points[i_next].0, points[i_next].1,
                points[j].0, points[j].1, points[j_next].0, points[j_next].1,
            ) {
                if t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99 {
                    // Found a real crossing — split and keep larger loop
                    _found_intersection = true;

                    // Intersection point
                    let ix = points[i].0 + t * (points[i_next].0 - points[i].0);
                    let iy = points[i].1 + t * (points[i_next].1 - points[i].1);

                    // Loop 1: i_next..=j with intersection point on both ends
                    let loop1_len = if j >= i_next { j - i_next + 1 } else { 0 };
                    // Loop 2: j_next..=i with intersection point
                    let loop2_len = if i >= j_next { i - j_next + 1 } else { n - j_next + i + 1 };

                    if loop1_len >= loop2_len && loop1_len >= 2 {
                        // Keep loop 1: intersection → i_next → ... → j → intersection
                        let mut new_pts = vec![(ix, iy)];
                        for k in 0..=loop1_len {
                            let idx = (i_next + k) % n;
                            if idx == (j + 1) % n { break; }
                            new_pts.push(points[idx]);
                        }
                        *points = new_pts;
                    } else if loop2_len >= 2 {
                        // Keep loop 2: intersection → j_next → ... → i → intersection
                        let mut new_pts = vec![(ix, iy)];
                        for k in 0..loop2_len {
                            let idx = (j_next + k) % n;
                            new_pts.push(points[idx]);
                        }
                        *points = new_pts;
                    }

                    // After modifying, recursively clean again
                    if points.len() >= 4 {
                        clean_self_intersections(points);
                    }
                    break 'outer;
                }
            }
        }
    }
}

/// Offset a contour inward multiple times to generate wall loops.
/// Returns contours from outermost (first offset) to innermost.
pub fn generate_wall_loops(
    contour: &Contour,
    num_walls: usize,
    line_width: f64,
) -> Vec<Contour> {
    if num_walls == 0 {
        return Vec::new();
    }

    let mut walls = Vec::with_capacity(num_walls);
    let mut current = contour.clone();

    for i in 0..num_walls {
        // First wall is offset by half line_width (so the extrusion center
        // sits at half a line_width from the edge), subsequent walls by full line_width
        let offset = if i == 0 { line_width / 2.0 } else { line_width };

        match offset_contour(&current, offset) {
            Some(offset_contour) => {
                walls.push(offset_contour.clone());
                current = offset_contour;
            }
            None => break, // Contour collapsed — can't fit more walls
        }
    }

    walls
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square_contour(size: f64) -> Contour {
        // CCW square centered at origin
        Contour::new(vec![
            Point3D::new(-size / 2.0, -size / 2.0, 0.0),
            Point3D::new(size / 2.0, -size / 2.0, 0.0),
            Point3D::new(size / 2.0, size / 2.0, 0.0),
            Point3D::new(-size / 2.0, size / 2.0, 0.0),
        ], true)
    }

    #[test]
    fn test_signed_area_ccw() {
        let sq = square_contour(10.0);
        let area = signed_area_2d(&sq.points);
        assert!(area > 0.0, "CCW square should have positive area, got {}", area);
        assert!((area - 100.0).abs() < 1e-6, "10x10 square area should be 100, got {}", area);
    }

    #[test]
    fn test_signed_area_cw() {
        // Reversed winding
        let sq = Contour::new(vec![
            Point3D::new(-5.0, -5.0, 0.0),
            Point3D::new(-5.0, 5.0, 0.0),
            Point3D::new(5.0, 5.0, 0.0),
            Point3D::new(5.0, -5.0, 0.0),
        ], true);
        let area = signed_area_2d(&sq.points);
        assert!(area < 0.0, "CW square should have negative area, got {}", area);
    }

    #[test]
    fn test_offset_square() {
        let sq = square_contour(10.0);
        let offset = offset_contour(&sq, 1.0).expect("Offset should succeed");

        // 10x10 square offset inward by 1 → 8x8
        let offset_area = signed_area_2d(&offset.points).abs();
        assert!(
            (offset_area - 64.0).abs() < 1.0,
            "Offset 10x10 square by 1 should give ~64 area, got {}",
            offset_area
        );
    }

    #[test]
    fn test_offset_collapse() {
        let sq = square_contour(2.0); // 2x2 square
        // Offset by 2 — should collapse
        let result = offset_contour(&sq, 2.0);
        assert!(result.is_none(), "Offsetting 2x2 square by 2 should collapse");
    }

    #[test]
    fn test_generate_wall_loops() {
        let sq = square_contour(20.0);
        let walls = generate_wall_loops(&sq, 3, 1.0);

        assert!(walls.len() >= 2, "Should generate at least 2 wall loops for 20x20 square, got {}", walls.len());

        // Each successive wall should have smaller area
        for i in 1..walls.len() {
            let area_prev = signed_area_2d(&walls[i - 1].points).abs();
            let area_curr = signed_area_2d(&walls[i].points).abs();
            assert!(
                area_curr < area_prev,
                "Wall {} area ({}) should be less than wall {} area ({})",
                i, area_curr, i - 1, area_prev
            );
        }
    }

    #[test]
    fn test_offset_triangle() {
        // Equilateral triangle
        let tri = Contour::new(vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(5.0, 8.66, 0.0),
        ], true);

        let offset = offset_contour(&tri, 0.5);
        assert!(offset.is_some(), "Small offset of triangle should succeed");

        let offset = offset.unwrap();
        let orig_area = signed_area_2d(&tri.points).abs();
        let new_area = signed_area_2d(&offset.points).abs();
        assert!(new_area < orig_area, "Offset triangle should have smaller area");
    }

    #[test]
    fn test_z_interpolation() {
        // Contour with varying Z (like a conical layer)
        let contour = Contour::new(vec![
            Point3D::new(0.0, 0.0, 10.0),
            Point3D::new(10.0, 0.0, 5.0),
            Point3D::new(10.0, 10.0, 5.0),
            Point3D::new(0.0, 10.0, 10.0),
        ], true);

        // Point at midpoint of first edge should have Z=7.5
        let z = interpolate_z_from_contour(5.0, 0.0, &contour);
        assert!((z - 7.5).abs() < 0.5, "Z at midpoint should be ~7.5, got {}", z);

        // Point at corner should have corner Z
        let z = interpolate_z_from_contour(0.0, 0.0, &contour);
        assert!((z - 10.0).abs() < 0.5, "Z at corner should be ~10.0, got {}", z);
    }
}
