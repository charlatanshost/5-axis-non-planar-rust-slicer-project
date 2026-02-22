use crate::geometry::{Contour, LineSegment, Point3D, Vector3D};
use crate::mesh::Mesh;
use crate::Result;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Configuration for slicing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SlicingConfig {
    /// Layer height for uniform slicing
    pub layer_height: f64,
    
    /// Minimum layer thickness
    pub min_layer_height: f64,
    
    /// Maximum layer thickness
    pub max_layer_height: f64,
    
    /// Enable adaptive layer heights
    pub adaptive: bool,

    /// Tolerance for geometric operations
    pub tolerance: f64,

    /// Maximum rotation angle per triangle for S3-Slicer (degrees)
    /// Only used in Curved slicing mode
    pub max_rotation_degrees: f64,
}

impl Default for SlicingConfig {
    fn default() -> Self {
        Self {
            layer_height: 0.2,
            min_layer_height: 0.1,
            max_layer_height: 0.3,
            adaptive: false,
            tolerance: 1e-6,
            max_rotation_degrees: 15.0,
        }
    }
}

/// A single layer with contours
#[derive(Debug, Clone)]
pub struct Layer {
    pub z: f64,
    pub contours: Vec<Contour>,
    pub layer_height: f64,
}

impl Layer {
    pub fn new(z: f64, contours: Vec<Contour>, layer_height: f64) -> Self {
        Self {
            z,
            contours,
            layer_height,
        }
    }

    /// Check if layer is empty
    pub fn is_empty(&self) -> bool {
        self.contours.is_empty() || self.contours.iter().all(|c| c.is_empty())
    }

    /// Get total number of points in all contours
    pub fn num_points(&self) -> usize {
        self.contours.iter().map(|c| c.len()).sum()
    }
}

/// Main slicer implementation
pub struct Slicer {
    config: SlicingConfig,
}

impl Slicer {
    pub fn new(config: SlicingConfig) -> Self {
        Self { config }
    }

    /// Slice mesh into layers (main entry point)
    pub fn slice(&self, mesh: &Mesh) -> Result<Vec<Layer>> {
        log::info!("Starting slicing process...");
        log::info!("Mesh: {} triangles", mesh.num_triangles());
        log::info!("Layer height: {}", self.config.layer_height);

        let slicing_planes = self.generate_slicing_planes(mesh);
        log::info!("Generated {} slicing planes (adaptive={})",
            slicing_planes.len(), self.config.adaptive);

        // Parallel slicing using Rayon
        let mut layers: Vec<Layer> = slicing_planes
            .par_iter()
            .map(|&(z, lh)| self.slice_at_height(mesh, z, lh))
            .filter(|layer| !layer.is_empty())
            .collect();

        // Ensure deterministic layer ordering (par_iter doesn't guarantee order)
        layers.sort_by(|a, b| a.z.partial_cmp(&b.z).unwrap());

        log::info!("Generated {} non-empty layers", layers.len());
        Ok(layers)
    }

    /// Generate slicing plane heights, returning (z, layer_height) pairs.
    /// When `config.adaptive` is true, layer height varies by local surface slope:
    ///   flat faces (|normal.z| ≈ 1) → max_layer_height (fast, accurate enough)
    ///   vertical faces (|normal.z| ≈ 0) → min_layer_height (slow, fine detail)
    fn generate_slicing_planes(&self, mesh: &Mesh) -> Vec<(f64, f64)> {
        let z_min = mesh.bounds_min.z;
        let z_max = mesh.bounds_max.z;

        if !self.config.adaptive {
            // Uniform spacing
            let mut planes = Vec::new();
            let mut z = z_min + self.config.layer_height / 2.0;
            while z < z_max {
                planes.push((z, self.config.layer_height));
                z += self.config.layer_height;
            }
            return planes;
        }

        // Adaptive spacing: sample slope at each candidate plane
        let min_h = self.config.min_layer_height.max(0.001);
        let max_h = self.config.max_layer_height.max(min_h);
        let band  = max_h; // look within one max-height band for relevant triangles

        let mut planes = Vec::new();
        let mut z = z_min + min_h / 2.0;

        while z < z_max {
            let slope = self.slope_factor(mesh, z, band);
            // slope ≈ 1 → horizontal → thicker layers; slope ≈ 0 → vertical → thinner
            let h = (min_h + slope * (max_h - min_h)).clamp(min_h, max_h);
            planes.push((z, h));
            z += h;
        }

        planes
    }

    /// Average |normal.z| of triangles whose Z range overlaps [z-band, z+band].
    /// Returns 1.0 (horizontal) when no triangles are found.
    fn slope_factor(&self, mesh: &Mesh, z: f64, band: f64) -> f64 {
        let mut sum = 0.0_f64;
        let mut count = 0usize;
        for tri in &mesh.triangles {
            let z_lo = tri.v0.z.min(tri.v1.z).min(tri.v2.z);
            let z_hi = tri.v0.z.max(tri.v1.z).max(tri.v2.z);
            if z_hi >= z - band && z_lo <= z + band {
                sum += tri.normal().z.abs();
                count += 1;
            }
        }
        if count == 0 { 1.0 } else { (sum / count as f64).clamp(0.0, 1.0) }
    }

    /// Slice mesh at a specific height
    fn slice_at_height(&self, mesh: &Mesh, z: f64, layer_height: f64) -> Layer {
        // Step 1: Get relevant triangles (those that intersect this plane)
        let triangles = mesh.triangles_at_height(z, self.config.tolerance);

        // Step 2: Compute intersection segments
        let segments: Vec<LineSegment> = triangles
            .iter()
            .filter_map(|tri| tri.intersect_plane(z))
            .collect();

        // Step 3: Build contours from segments using hash-based chaining (O(m))
        let contours = self.build_contours(&segments);

        Layer::new(z, contours, layer_height)
    }

    /// Build contours from line segments using hash table (Algorithm from Paper 1)
    /// Time complexity: O(m) where m is number of segments
    fn build_contours(&self, segments: &[LineSegment]) -> Vec<Contour> {
        if segments.is_empty() {
            return Vec::new();
        }

        // Hash map: endpoint -> list of segments containing that endpoint
        let mut endpoint_map: HashMap<PointKey, Vec<usize>> = HashMap::new();

        // Build endpoint map
        for (i, segment) in segments.iter().enumerate() {
            let start_key = PointKey::from_point(&segment.start, self.config.tolerance);
            let end_key = PointKey::from_point(&segment.end, self.config.tolerance);

            endpoint_map.entry(start_key).or_default().push(i);
            endpoint_map.entry(end_key).or_default().push(i);
        }

        // Track which segments have been used
        let mut used = vec![false; segments.len()];
        let mut contours = Vec::new();

        // Build each contour by chaining segments
        for start_idx in 0..segments.len() {
            if used[start_idx] {
                continue;
            }

            let contour = self.build_contour_from_segment(
                start_idx,
                segments,
                &endpoint_map,
                &mut used,
            );

            if !contour.is_empty() {
                contours.push(contour);
            }
        }

        contours
    }

    /// Build a single contour starting from a segment
    fn build_contour_from_segment(
        &self,
        start_idx: usize,
        segments: &[LineSegment],
        endpoint_map: &HashMap<PointKey, Vec<usize>>,
        used: &mut [bool],
    ) -> Contour {
        let mut points = Vec::new();
        let mut current_idx = start_idx;
        let mut current_point = segments[start_idx].start;

        used[current_idx] = true;
        points.push(current_point);

        // Follow the chain of segments
        let start_point = current_point;
        loop {
            let segment = &segments[current_idx];
            
            // Determine next point in the segment
            let next_point = if (segment.start.coords - current_point.coords).norm() < self.config.tolerance {
                segment.end
            } else {
                segment.start
            };

            points.push(next_point);

            // Check if we've closed the loop
            if (next_point.coords - start_point.coords).norm() < self.config.tolerance {
                return Contour::new(points, true);
            }

            // Find next segment
            let next_key = PointKey::from_point(&next_point, self.config.tolerance);
            let next_idx = endpoint_map
                .get(&next_key)
                .and_then(|indices| {
                    indices
                        .iter()
                        .find(|&&idx| !used[idx])
                        .copied()
                });

            match next_idx {
                Some(idx) => {
                    used[idx] = true;
                    current_idx = idx;
                    current_point = next_point;
                }
                None => {
                    // Open contour - couldn't find next segment
                    return Contour::new(points, false);
                }
            }
        }
    }
}

/// Discretized point key for hash map (handles floating point comparison)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct PointKey {
    x: i64,
    y: i64,
    z: i64,
}

impl PointKey {
    fn from_point(point: &Point3D, tolerance: f64) -> Self {
        let scale = 1.0 / tolerance;
        Self {
            x: (point.x * scale).round() as i64,
            y: (point.y * scale).round() as i64,
            z: (point.z * scale).round() as i64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Triangle;

    #[test]
    fn test_simple_slice() {
        let triangles = vec![
            Triangle::new(
                Point3D::new(0.0, 0.0, 0.0),
                Point3D::new(10.0, 0.0, 0.0),
                Point3D::new(5.0, 10.0, 10.0),
            ),
        ];

        let mesh = Mesh::new(triangles).unwrap();
        let slicer = Slicer::new(SlicingConfig::default());
        let layers = slicer.slice(&mesh).unwrap();

        assert!(!layers.is_empty());
    }

    #[test]
    fn test_contour_building() {
        let segments = vec![
            LineSegment::new(Point3D::new(0.0, 0.0, 0.0), Point3D::new(1.0, 0.0, 0.0)),
            LineSegment::new(Point3D::new(1.0, 0.0, 0.0), Point3D::new(1.0, 1.0, 0.0)),
            LineSegment::new(Point3D::new(1.0, 1.0, 0.0), Point3D::new(0.0, 1.0, 0.0)),
            LineSegment::new(Point3D::new(0.0, 1.0, 0.0), Point3D::new(0.0, 0.0, 0.0)),
        ];

        let slicer = Slicer::new(SlicingConfig::default());
        let contours = slicer.build_contours(&segments);

        assert_eq!(contours.len(), 1);
        assert!(contours[0].closed);
        assert_eq!(contours[0].points.len(), 5); // 4 corners + closing point
    }
}
