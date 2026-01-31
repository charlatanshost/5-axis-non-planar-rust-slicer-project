// Isosurface extraction for curved layer generation
// Extracts contours from scalar field using marching triangles

use crate::geometry::{Point3D, Vector3D, LineSegment, Contour};
use crate::mesh::Mesh;
use crate::s3_slicer::scalar_field::ScalarField;
use crate::slicing::Layer;
use rayon::prelude::*;
use std::collections::HashMap;

/// Configuration for isosurface extraction
#[derive(Debug, Clone)]
pub struct IsosurfaceConfig {
    /// Number of layers to extract
    pub num_layers: usize,

    /// Layer height (in scalar field units)
    pub layer_height: f64,

    /// Minimum contour length (filter out small contours)
    pub min_contour_length: f64,

    /// Simplification tolerance
    pub simplification_tolerance: f64,

    /// Enable adaptive layer spacing (vs uniform)
    pub adaptive_spacing: bool,

    /// Target layer thickness for adaptive spacing (mm)
    pub target_layer_thickness: f64,

    /// Minimum allowed layer thickness (mm)
    pub min_layer_thickness: f64,

    /// Maximum allowed layer thickness (mm)
    pub max_layer_thickness: f64,

    /// Tolerance for binary search convergence (mm)
    pub search_tolerance: f64,
}

impl Default for IsosurfaceConfig {
    fn default() -> Self {
        Self {
            num_layers: 100,
            layer_height: 0.2,
            min_contour_length: 1.0,
            simplification_tolerance: 0.01,
            adaptive_spacing: true,
            target_layer_thickness: 0.2,
            min_layer_thickness: 0.1,
            max_layer_thickness: 0.4,
            search_tolerance: 0.01,
        }
    }
}

/// A curved layer extracted from scalar field
#[derive(Debug, Clone)]
pub struct CurvedLayer {
    /// Z-height of this layer (average)
    pub z: f64,

    /// Contour segments forming this layer
    pub segments: Vec<LineSegment>,

    /// Scalar field value for this isosurface
    pub iso_value: f64,

    /// Layer normal vectors (for toolpath orientation)
    pub normals: Vec<Vector3D>,
}

impl CurvedLayer {
    /// Convert to standard Layer format for compatibility
    pub fn to_layer(&self) -> Layer {
        // Chain segments into continuous contours
        let paths = chain_segments(&self.segments, 0.1);

        // Convert paths to Contour objects
        let contours: Vec<Contour> = paths.into_iter()
            .map(|points| {
                // Check if path is closed (first and last points are close)
                let is_closed = if points.len() >= 3 {
                    (points[0] - points[points.len() - 1]).norm() < 0.1
                } else {
                    false
                };

                Contour {
                    points,
                    closed: is_closed,
                }
            })
            .collect();

        Layer {
            z: self.z,
            contours,
            layer_height: 0.2,  // Will be adjusted based on deformation
        }
    }

    /// Get average Z coordinate
    pub fn average_z(&self) -> f64 {
        if self.segments.is_empty() {
            return self.z;
        }

        let sum: f64 = self.segments.iter()
            .map(|seg| (seg.start.z + seg.end.z) / 2.0)
            .sum();

        sum / self.segments.len() as f64
    }
}

/// Isosurface extractor
pub struct IsosurfaceExtractor {
    config: IsosurfaceConfig,
}

impl IsosurfaceExtractor {
    pub fn new(config: IsosurfaceConfig) -> Self {
        Self { config }
    }

    /// Extract isosurface at a specific scalar value
    pub fn extract_at_value(
        &self,
        mesh: &Mesh,
        scalar_field: &ScalarField,
        iso_value: f64,
    ) -> CurvedLayer {
        // Convert per-triangle scalar field to per-vertex values
        let vertex_scalar_map = triangle_field_to_vertex_field(mesh, &scalar_field.values);

        // Parallel extraction of contour segments for each triangle
        let results: Vec<(LineSegment, Vector3D)> = mesh.triangles
            .par_iter()
            .enumerate()
            .filter_map(|(tri_idx, triangle)| {
                if tri_idx >= scalar_field.values.len() {
                    return None;
                }

                // Get scalar values at triangle vertices by averaging triangles sharing each vertex
                let v0_hash = hash_vertex(&triangle.v0);
                let v1_hash = hash_vertex(&triangle.v1);
                let v2_hash = hash_vertex(&triangle.v2);

                let v0_value = vertex_scalar_map.get(&v0_hash).copied().unwrap_or(scalar_field.values[tri_idx]);
                let v1_value = vertex_scalar_map.get(&v1_hash).copied().unwrap_or(scalar_field.values[tri_idx]);
                let v2_value = vertex_scalar_map.get(&v2_hash).copied().unwrap_or(scalar_field.values[tri_idx]);

                // Check if isosurface crosses this triangle (marching triangles)
                extract_triangle_contour(
                    triangle,
                    v0_value,
                    v1_value,
                    v2_value,
                    iso_value,
                ).map(|segment| (segment, triangle.normal()))
            })
            .collect();

        // Unzip segments and normals
        let (segments, normals): (Vec<_>, Vec<_>) = results.into_iter().unzip();

        // Compute average Z
        let avg_z = if segments.is_empty() {
            iso_value
        } else {
            segments.iter()
                .map(|seg| (seg.start.z + seg.end.z) / 2.0)
                .sum::<f64>() / segments.len() as f64
        };

        CurvedLayer {
            z: avg_z,
            segments,
            iso_value,
            normals,
        }
    }

    /// Extract multiple layers uniformly distributed
    pub fn extract_layers(
        &self,
        mesh: &Mesh,
        scalar_field: &ScalarField,
    ) -> Vec<CurvedLayer> {
        let (min_value, max_value) = scalar_field.range();
        let range = max_value - min_value;

        if range < 1e-10 {
            return Vec::new();
        }

        // Choose extraction method based on config
        if self.config.adaptive_spacing {
            self.extract_layers_adaptive(mesh, scalar_field, min_value, max_value)
        } else {
            self.extract_layers_uniform(mesh, scalar_field, min_value, max_value)
        }
    }

    /// Extract layers with uniform scalar field spacing
    fn extract_layers_uniform(
        &self,
        mesh: &Mesh,
        scalar_field: &ScalarField,
        min_value: f64,
        max_value: f64,
    ) -> Vec<CurvedLayer> {
        let range = max_value - min_value;

        // Parallel extraction of all layers
        let layers: Vec<CurvedLayer> = (0..self.config.num_layers)
            .into_par_iter()
            .filter_map(|i| {
                let t = i as f64 / (self.config.num_layers - 1).max(1) as f64;
                let iso_value = min_value + t * range;

                let layer = self.extract_at_value(mesh, scalar_field, iso_value);

                // Filter out empty layers
                if !layer.segments.is_empty() {
                    Some(layer)
                } else {
                    None
                }
            })
            .collect();

        layers
    }

    /// Extract layers with adaptive spacing based on actual geometric distance
    fn extract_layers_adaptive(
        &self,
        mesh: &Mesh,
        scalar_field: &ScalarField,
        min_value: f64,
        max_value: f64,
    ) -> Vec<CurvedLayer> {
        log::info!("Extracting layers with adaptive spacing...");
        log::info!("  Target thickness: {:.3} mm", self.config.target_layer_thickness);
        log::info!("  Range: [{:.3}, {:.3}] mm", self.config.min_layer_thickness, self.config.max_layer_thickness);

        let mut layers = Vec::new();
        let mut current_iso_value = min_value;

        // Extract first layer
        let first_layer = self.extract_at_value(mesh, scalar_field, current_iso_value);
        if first_layer.segments.is_empty() {
            log::warn!("First layer is empty, falling back to uniform spacing");
            return self.extract_layers_uniform(mesh, scalar_field, min_value, max_value);
        }

        layers.push(first_layer);

        // Iteratively find next layers using binary search
        while current_iso_value < max_value && layers.len() < self.config.num_layers {
            // Binary search for next iso_value that gives target layer thickness
            match self.find_next_iso_value(
                mesh,
                scalar_field,
                &layers.last().unwrap(),
                current_iso_value,
                max_value,
            ) {
                Some(next_iso_value) => {
                    let next_layer = self.extract_at_value(mesh, scalar_field, next_iso_value);

                    if !next_layer.segments.is_empty() {
                        layers.push(next_layer);
                        current_iso_value = next_iso_value;
                    } else {
                        // Empty layer, try smaller step
                        current_iso_value += (max_value - min_value) / (self.config.num_layers as f64 * 2.0);
                    }
                }
                None => {
                    // Could not find next layer within constraints
                    break;
                }
            }

            // Safety check to prevent infinite loops
            if layers.len() > self.config.num_layers * 2 {
                log::warn!("Adaptive spacing generated too many layers, stopping");
                break;
            }
        }

        log::info!("  Extracted {} adaptive layers", layers.len());
        layers
    }

    /// Binary search for next iso-value that gives target layer thickness
    fn find_next_iso_value(
        &self,
        mesh: &Mesh,
        scalar_field: &ScalarField,
        previous_layer: &CurvedLayer,
        current_iso: f64,
        max_iso: f64,
    ) -> Option<f64> {
        let range = max_iso - current_iso;
        if range < 1e-10 {
            return None;
        }

        // Initial guess: small step in scalar field
        let mut low = current_iso + range * 0.001;
        let mut high = (current_iso + range * 0.1).min(max_iso);

        // Binary search iterations
        for _iteration in 0..20 {
            let mid = (low + high) / 2.0;

            // Extract candidate layer
            let candidate_layer = self.extract_at_value(mesh, scalar_field, mid);

            if candidate_layer.segments.is_empty() {
                // Empty layer, search higher
                low = mid;
                continue;
            }

            // Compute distance between layers
            let distance = compute_layer_distance(previous_layer, &candidate_layer);

            // Check if distance is within acceptable range
            if distance < self.config.min_layer_thickness {
                // Too close, search higher
                low = mid;
            } else if distance > self.config.max_layer_thickness {
                // Too far, search lower
                high = mid;
            } else {
                // Within acceptable range
                return Some(mid);
            }

            // Check convergence
            if (high - low) < self.config.search_tolerance * range {
                return Some(mid);
            }
        }

        // Return best guess if binary search didn't converge
        Some((low + high) / 2.0)
    }
}

/// Compute geometric distance between two curved layers
fn compute_layer_distance(layer1: &CurvedLayer, layer2: &CurvedLayer) -> f64 {
    if layer1.segments.is_empty() || layer2.segments.is_empty() {
        return 0.0;
    }

    // Sample points from both layers
    let points1 = sample_layer_points(layer1, 10); // Sample ~10 points per segment
    let points2 = sample_layer_points(layer2, 10);

    if points1.is_empty() || points2.is_empty() {
        return (layer2.z - layer1.z).abs();
    }

    // Compute distances from points in layer1 to closest points in layer2
    let distances: Vec<f64> = points1
        .iter()
        .map(|p1| {
            points2
                .iter()
                .map(|p2| (*p1 - *p2).norm())
                .fold(f64::INFINITY, f64::min)
        })
        .collect();

    // Return median distance for robustness
    if !distances.is_empty() {
        let mut sorted = distances.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        sorted[sorted.len() / 2]
    } else {
        (layer2.z - layer1.z).abs()
    }
}

/// Sample points along layer segments
fn sample_layer_points(layer: &CurvedLayer, samples_per_segment: usize) -> Vec<Point3D> {
    let mut points = Vec::new();

    for segment in &layer.segments {
        // Add segment endpoints
        points.push(segment.start);

        // Add intermediate points
        for i in 1..samples_per_segment {
            let t = i as f64 / samples_per_segment as f64;
            let point = Point3D::new(
                segment.start.x + t * (segment.end.x - segment.start.x),
                segment.start.y + t * (segment.end.y - segment.start.y),
                segment.start.z + t * (segment.end.z - segment.start.z),
            );
            points.push(point);
        }

        points.push(segment.end);
    }

    points
}

/// Extract contour segment from a single triangle using marching triangles
fn extract_triangle_contour(
    triangle: &crate::geometry::Triangle,
    v0_value: f64,
    v1_value: f64,
    v2_value: f64,
    iso_value: f64,
) -> Option<LineSegment> {
    // Marching triangles: find where isosurface intersects triangle edges
    let mut intersection_points = Vec::new();

    // Check edge v0-v1
    if let Some(point) = interpolate_edge(&triangle.v0, &triangle.v1, v0_value, v1_value, iso_value) {
        intersection_points.push(point);
    }

    // Check edge v1-v2
    if let Some(point) = interpolate_edge(&triangle.v1, &triangle.v2, v1_value, v2_value, iso_value) {
        intersection_points.push(point);
    }

    // Check edge v2-v0
    if let Some(point) = interpolate_edge(&triangle.v2, &triangle.v0, v2_value, v0_value, iso_value) {
        intersection_points.push(point);
    }

    // Isosurface crosses triangle if we have exactly 2 intersection points
    if intersection_points.len() == 2 {
        Some(LineSegment {
            start: intersection_points[0],
            end: intersection_points[1],
        })
    } else {
        None
    }
}

/// Interpolate point on edge where scalar field equals iso_value
fn interpolate_edge(
    p0: &Point3D,
    p1: &Point3D,
    v0: f64,
    v1: f64,
    iso_value: f64,
) -> Option<Point3D> {
    // Check if isosurface crosses this edge
    let crosses = (v0 <= iso_value && iso_value <= v1) || (v1 <= iso_value && iso_value <= v0);

    if !crosses {
        return None;
    }

    // Avoid division by zero
    let delta = v1 - v0;
    if delta.abs() < 1e-10 {
        return Some(*p0);
    }

    // Linear interpolation parameter
    let t = (iso_value - v0) / delta;
    let t = t.clamp(0.0, 1.0);

    // Interpolate position
    Some(Point3D::new(
        p0.x + t * (p1.x - p0.x),
        p0.y + t * (p1.y - p0.y),
        p0.z + t * (p1.z - p0.z),
    ))
}

/// Extract curved layers from mesh and scalar field
pub fn extract_curved_layers(
    mesh: &Mesh,
    scalar_field: &ScalarField,
    config: &IsosurfaceConfig,
) -> Vec<CurvedLayer> {
    let extractor = IsosurfaceExtractor::new(config.clone());
    extractor.extract_layers(mesh, scalar_field)
}

/// Chain contour segments into continuous paths
pub fn chain_segments(segments: &[LineSegment], tolerance: f64) -> Vec<Vec<Point3D>> {
    if segments.is_empty() {
        return Vec::new();
    }

    let mut paths = Vec::new();
    let mut used = vec![false; segments.len()];

    for i in 0..segments.len() {
        if used[i] {
            continue;
        }

        let mut path = vec![segments[i].start, segments[i].end];
        used[i] = true;

        // Try to extend path forward
        loop {
            let last_point = *path.last().unwrap();
            let mut found = false;

            for j in 0..segments.len() {
                if used[j] {
                    continue;
                }

                // Check if segment j connects to end of path
                if (segments[j].start - last_point).norm() < tolerance {
                    path.push(segments[j].end);
                    used[j] = true;
                    found = true;
                    break;
                } else if (segments[j].end - last_point).norm() < tolerance {
                    path.push(segments[j].start);
                    used[j] = true;
                    found = true;
                    break;
                }
            }

            if !found {
                break;
            }
        }

        // Try to extend path backward
        loop {
            let first_point = path[0];
            let mut found = false;

            for j in 0..segments.len() {
                if used[j] {
                    continue;
                }

                // Check if segment j connects to start of path
                if (segments[j].end - first_point).norm() < tolerance {
                    path.insert(0, segments[j].start);
                    used[j] = true;
                    found = true;
                    break;
                } else if (segments[j].start - first_point).norm() < tolerance {
                    path.insert(0, segments[j].end);
                    used[j] = true;
                    found = true;
                    break;
                }
            }

            if !found {
                break;
            }
        }

        paths.push(path);
    }

    paths
}

/// Simplify contour using Douglas-Peucker algorithm
pub fn simplify_contour(points: &[Point3D], tolerance: f64) -> Vec<Point3D> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    douglas_peucker(points, tolerance)
}

/// Douglas-Peucker line simplification algorithm
fn douglas_peucker(points: &[Point3D], epsilon: f64) -> Vec<Point3D> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    // Find point with maximum distance from line
    let first = points[0];
    let last = points[points.len() - 1];
    let mut max_dist = 0.0;
    let mut max_idx = 0;

    for (i, point) in points.iter().enumerate().skip(1).take(points.len() - 2) {
        let dist = point_to_line_distance(point, &first, &last);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if max_dist > epsilon {
        let mut result1 = douglas_peucker(&points[0..=max_idx], epsilon);
        let result2 = douglas_peucker(&points[max_idx..], epsilon);

        result1.pop(); // Remove duplicate point
        result1.extend(result2);
        result1
    } else {
        vec![first, last]
    }
}

/// Compute perpendicular distance from point to line
fn point_to_line_distance(point: &Point3D, line_start: &Point3D, line_end: &Point3D) -> f64 {
    let line_vec = *line_end - *line_start;
    let point_vec = *point - *line_start;

    let line_len = line_vec.norm();
    if line_len < 1e-10 {
        return point_vec.norm();
    }

    let cross = point_vec.cross(&line_vec);
    cross.norm() / line_len
}

/// Convert per-triangle scalar field to per-vertex by averaging triangles sharing each vertex
fn triangle_field_to_vertex_field(mesh: &Mesh, triangle_values: &[f64]) -> HashMap<u64, f64> {
    // Build map: vertex hash -> (sum of values, count)
    let mut vertex_accumulator: HashMap<u64, (f64, usize)> = HashMap::new();

    for (tri_idx, triangle) in mesh.triangles.iter().enumerate() {
        if tri_idx >= triangle_values.len() {
            continue;
        }

        let value = triangle_values[tri_idx];

        // Add this triangle's value to all three vertices
        let v0_hash = hash_vertex(&triangle.v0);
        let v1_hash = hash_vertex(&triangle.v1);
        let v2_hash = hash_vertex(&triangle.v2);

        for vertex_hash in [v0_hash, v1_hash, v2_hash] {
            vertex_accumulator
                .entry(vertex_hash)
                .and_modify(|(sum, count)| {
                    *sum += value;
                    *count += 1;
                })
                .or_insert((value, 1));
        }
    }

    // Average the accumulated values
    vertex_accumulator
        .into_iter()
        .map(|(hash, (sum, count))| (hash, sum / count as f64))
        .collect()
}

/// Hash a vertex position for use as a map key
fn hash_vertex(point: &Point3D) -> u64 {
    // Use high precision for vertex matching (same as deformation_v2.rs)
    let scale = 10000.0;
    let x = (point.x * scale).round() as i64;
    let y = (point.y * scale).round() as i64;
    let z = (point.z * scale).round() as i64;
    ((x as u64) << 42) ^ ((y as u64) << 21) ^ (z as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_edge_interpolation() {
        let p0 = Point3D::new(0.0, 0.0, 0.0);
        let p1 = Point3D::new(10.0, 0.0, 10.0);

        // Isosurface at value 5.0 should be at midpoint
        let result = interpolate_edge(&p0, &p1, 0.0, 10.0, 5.0);
        assert!(result.is_some());

        let point = result.unwrap();
        assert!((point.x - 5.0).abs() < 0.01);
        assert!((point.z - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_edge_no_crossing() {
        let p0 = Point3D::new(0.0, 0.0, 0.0);
        let p1 = Point3D::new(10.0, 0.0, 10.0);

        // Isosurface at value 15.0 doesn't cross edge (max value is 10.0)
        let result = interpolate_edge(&p0, &p1, 0.0, 10.0, 15.0);
        assert!(result.is_none());
    }

    #[test]
    fn test_point_to_line_distance() {
        let line_start = Point3D::new(0.0, 0.0, 0.0);
        let line_end = Point3D::new(10.0, 0.0, 0.0);
        let point = Point3D::new(5.0, 3.0, 0.0);

        let dist = point_to_line_distance(&point, &line_start, &line_end);
        assert!((dist - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_chain_segments() {
        let seg1 = LineSegment {
            start: Point3D::new(0.0, 0.0, 0.0),
            end: Point3D::new(1.0, 0.0, 0.0),
        };
        let seg2 = LineSegment {
            start: Point3D::new(1.0, 0.0, 0.0),
            end: Point3D::new(2.0, 0.0, 0.0),
        };

        let paths = chain_segments(&[seg1, seg2], 0.01);

        assert_eq!(paths.len(), 1);
        assert_eq!(paths[0].len(), 3);
    }
}
