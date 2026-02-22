// Ruled surface generation for wall seam transitions between consecutive curved layers.
//
// A ruled surface is a surface swept by a straight line (ruling line) between two curves.
// Here we use it to bridge the staircase gap between the outer wall of layer n and the outer
// wall of layer n+1, by extruding a zigzag path along ruling lines connecting the two contours.
//
// Algorithm:
//   1. Resample both contours to N evenly-spaced points (N = min(max(len1, len2), 256)).
//   2. Find the rotation offset of contour 2 that minimises total XY distance to contour 1
//      (so the zigzag doesn't unnecessarily cross over itself).
//   3. Generate N ruling lines (pairs of corresponding points).
//   4. Produce a zigzag transition path: C1[0]→C2[0]→C1[1]→C2[1]→…

use crate::geometry::Point3D;

#[derive(Debug, Clone)]
pub struct RuledSurface {
    /// Pairs of (bottom_point, top_point) along the ruling lines.
    pub ruling_lines: Vec<(Point3D, Point3D)>,
}

impl RuledSurface {
    /// Build a ruled surface between two closed contours.
    /// Returns `None` only if either contour is degenerate (< 2 points).
    pub fn detect_from_contours(c1: &[Point3D], c2: &[Point3D]) -> Option<Self> {
        if c1.len() < 2 || c2.len() < 2 {
            return None;
        }

        // Resample both to the same number of points
        let n = c1.len().max(c2.len()).min(256);
        let r1 = resample_to_n(c1, n);
        let r2 = resample_to_n(c2, n);

        // Align r2 to r1 by finding the rotation offset with minimum total XY cost
        let offset = find_alignment_offset(&r1, &r2);

        let ruling_lines = (0..n)
            .map(|i| (r1[i], r2[(i + offset) % n]))
            .collect();

        Some(RuledSurface { ruling_lines })
    }

    /// Generate a zigzag transition path along the ruling lines.
    ///
    /// The path alternates between the bottom and top endpoint of each ruling line:
    ///   bot[0] → top[0] → top[1] → bot[1] → bot[2] → top[2] → …
    /// This keeps extrusion direction consistent and fills the ruled surface evenly.
    pub fn generate_transition_path(&self) -> Vec<Point3D> {
        let mut pts = Vec::with_capacity(self.ruling_lines.len() * 2);
        for (i, &(bot, top)) in self.ruling_lines.iter().enumerate() {
            if i % 2 == 0 {
                pts.push(bot);
                pts.push(top);
            } else {
                pts.push(top);
                pts.push(bot);
            }
        }
        pts
    }
}

/// Resample `pts` (treated as a closed polyline) to exactly `n` evenly-spaced points
/// distributed by 3D arc length.
fn resample_to_n(pts: &[Point3D], n: usize) -> Vec<Point3D> {
    if pts.is_empty() || n == 0 {
        return Vec::new();
    }

    let m = pts.len();

    // Cumulative arc lengths around the closed loop
    let mut cum = vec![0.0f64; m + 1];
    for i in 0..m {
        let next = (i + 1) % m;
        let dx = pts[next].x - pts[i].x;
        let dy = pts[next].y - pts[i].y;
        let dz = pts[next].z - pts[i].z;
        cum[i + 1] = cum[i] + (dx * dx + dy * dy + dz * dz).sqrt();
    }
    let total = cum[m];
    if total < 1e-10 {
        return vec![pts[0]; n];
    }

    let mut result = Vec::with_capacity(n);
    for k in 0..n {
        let target = total * k as f64 / n as f64;
        // Find segment whose cumulative length bracket contains `target`
        let seg = cum[..=m]
            .partition_point(|&l| l <= target)
            .saturating_sub(1)
            .min(m - 1);
        let seg_len = cum[seg + 1] - cum[seg];
        let t = if seg_len < 1e-12 { 0.0 } else { (target - cum[seg]) / seg_len };
        let next = (seg + 1) % m;
        result.push(Point3D::new(
            pts[seg].x + t * (pts[next].x - pts[seg].x),
            pts[seg].y + t * (pts[next].y - pts[seg].y),
            pts[seg].z + t * (pts[next].z - pts[seg].z),
        ));
    }
    result
}

/// Find the rotation offset of `c2` that minimises the total squared XY distance to `c1`.
/// This aligns the two contours so the ruling lines are as short as possible.
/// O(n²) — fine for n ≤ 256.
fn find_alignment_offset(c1: &[Point3D], c2: &[Point3D]) -> usize {
    let n = c1.len().min(c2.len());
    if n == 0 {
        return 0;
    }
    let mut best_offset = 0usize;
    let mut best_cost = f64::INFINITY;
    for offset in 0..n {
        let cost: f64 = (0..n)
            .map(|i| {
                let j = (i + offset) % n;
                let dx = c1[i].x - c2[j].x;
                let dy = c1[i].y - c2[j].y;
                dx * dx + dy * dy
            })
            .sum();
        if cost < best_cost {
            best_cost = cost;
            best_offset = offset;
        }
    }
    best_offset
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_resample_square() {
        let pts = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(10.0, 10.0, 0.0),
            Point3D::new(0.0, 10.0, 0.0),
        ];
        let resampled = resample_to_n(&pts, 8);
        assert_eq!(resampled.len(), 8);
        // First point should be at origin
        assert!((resampled[0].x - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_ruled_surface_detect() {
        let c1: Vec<Point3D> = (0..8)
            .map(|i| {
                let angle = i as f64 * std::f64::consts::PI * 2.0 / 8.0;
                Point3D::new(angle.cos() * 5.0, angle.sin() * 5.0, 0.0)
            })
            .collect();
        let c2: Vec<Point3D> = (0..8)
            .map(|i| {
                let angle = i as f64 * std::f64::consts::PI * 2.0 / 8.0;
                Point3D::new(angle.cos() * 5.0, angle.sin() * 5.0, 2.0)
            })
            .collect();
        let surface = RuledSurface::detect_from_contours(&c1, &c2);
        assert!(surface.is_some());
        let s = surface.unwrap();
        assert_eq!(s.ruling_lines.len(), 8);

        let path = s.generate_transition_path();
        assert_eq!(path.len(), 16); // 2 points per ruling line
    }

    #[test]
    fn test_find_alignment_offset_identity() {
        let pts: Vec<Point3D> = (0..4)
            .map(|i| Point3D::new(i as f64, 0.0, 0.0))
            .collect();
        // Same contour — offset 0 should be best
        let offset = find_alignment_offset(&pts, &pts);
        assert_eq!(offset, 0);
    }
}
