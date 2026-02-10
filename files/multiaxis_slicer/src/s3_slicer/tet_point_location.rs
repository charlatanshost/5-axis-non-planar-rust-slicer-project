// Barycentric point location in tetrahedral meshes
//
// Provides a spatial acceleration grid (bin grid) for fast O(1) point-in-tet
// lookups, plus barycentric coordinate computation and interpolation.
//
// Used by the S4 pipeline to untransform toolpath points: given a point on the
// deformed mesh surface, find which deformed tet contains it, compute barycentric
// coordinates, then apply those same barycentrics in the original tet to get the
// original-space position.

use crate::geometry::Point3D;
use crate::s3_slicer::tet_mesh::TetMesh;

/// Result of a point-in-tet location query.
pub struct TetLocationResult {
    /// Index of the containing tetrahedron
    pub tet_index: usize,

    /// Barycentric coordinates [lambda0, lambda1, lambda2, lambda3]
    /// such that point = sum(lambda_i * vertex_i)
    pub bary_coords: [f64; 4],
}

/// Spatial acceleration grid for fast point-in-tet lookups.
///
/// Divides the bounding box into a uniform 3D grid of cells. Each cell stores
/// the indices of tets whose AABB overlaps that cell. Point queries first find
/// the cell, then test candidate tets via barycentric coordinates.
pub struct TetSpatialGrid {
    bounds_min: Point3D,
    bounds_max: Point3D,
    dims: (usize, usize, usize),
    cell_size: (f64, f64, f64),
    /// cells[iz * (nx*ny) + iy * nx + ix] = list of tet indices
    cells: Vec<Vec<usize>>,
}

impl TetSpatialGrid {
    /// Build a spatial grid over a tet mesh.
    ///
    /// Uses ~15 cells along the longest axis. Each tet is inserted into all
    /// grid cells that its AABB overlaps.
    pub fn build(tet_mesh: &TetMesh) -> Self {
        let extent = tet_mesh.bounds_max - tet_mesh.bounds_min;
        let max_extent = extent.x.max(extent.y).max(extent.z).max(1e-10);

        // Target ~15 cells along longest axis
        let target_cells = 15.0;
        let base_cell_size = max_extent / target_cells;

        // Compute per-axis dims (at least 1 cell)
        let nx = ((extent.x / base_cell_size).ceil() as usize).max(1);
        let ny = ((extent.y / base_cell_size).ceil() as usize).max(1);
        let nz = ((extent.z / base_cell_size).ceil() as usize).max(1);

        let cell_size = (
            extent.x / nx as f64,
            extent.y / ny as f64,
            extent.z / nz as f64,
        );

        // Add small padding to bounds
        let pad = max_extent * 0.001;
        let bounds_min = Point3D::new(
            tet_mesh.bounds_min.x - pad,
            tet_mesh.bounds_min.y - pad,
            tet_mesh.bounds_min.z - pad,
        );
        let bounds_max = Point3D::new(
            tet_mesh.bounds_max.x + pad,
            tet_mesh.bounds_max.y + pad,
            tet_mesh.bounds_max.z + pad,
        );

        let total_cells = nx * ny * nz;
        let mut cells: Vec<Vec<usize>> = vec![Vec::new(); total_cells];

        // Insert each tet into overlapping cells
        for (ti, tet) in tet_mesh.tets.iter().enumerate() {
            let v0 = &tet_mesh.vertices[tet.vertices[0]];
            let v1 = &tet_mesh.vertices[tet.vertices[1]];
            let v2 = &tet_mesh.vertices[tet.vertices[2]];
            let v3 = &tet_mesh.vertices[tet.vertices[3]];

            // Tet AABB
            let tet_min_x = v0.x.min(v1.x).min(v2.x).min(v3.x);
            let tet_min_y = v0.y.min(v1.y).min(v2.y).min(v3.y);
            let tet_min_z = v0.z.min(v1.z).min(v2.z).min(v3.z);
            let tet_max_x = v0.x.max(v1.x).max(v2.x).max(v3.x);
            let tet_max_y = v0.y.max(v1.y).max(v2.y).max(v3.y);
            let tet_max_z = v0.z.max(v1.z).max(v2.z).max(v3.z);

            // Cell range
            let ix0 = cell_index_1d(tet_min_x, bounds_min.x, cell_size.0, nx);
            let iy0 = cell_index_1d(tet_min_y, bounds_min.y, cell_size.1, ny);
            let iz0 = cell_index_1d(tet_min_z, bounds_min.z, cell_size.2, nz);
            let ix1 = cell_index_1d(tet_max_x, bounds_min.x, cell_size.0, nx);
            let iy1 = cell_index_1d(tet_max_y, bounds_min.y, cell_size.1, ny);
            let iz1 = cell_index_1d(tet_max_z, bounds_min.z, cell_size.2, nz);

            for iz in iz0..=iz1 {
                for iy in iy0..=iy1 {
                    for ix in ix0..=ix1 {
                        let cell_idx = iz * (nx * ny) + iy * nx + ix;
                        cells[cell_idx].push(ti);
                    }
                }
            }
        }

        log::debug!("TetSpatialGrid: {}x{}x{} = {} cells for {} tets",
            nx, ny, nz, total_cells, tet_mesh.tets.len());

        TetSpatialGrid {
            bounds_min,
            bounds_max,
            dims: (nx, ny, nz),
            cell_size,
            cells,
        }
    }

    /// Find the tet containing a point.
    ///
    /// Returns None if the point is outside all tets. Uses epsilon tolerance
    /// on barycentric coordinates to handle points exactly on tet faces.
    pub fn find_containing_tet(
        &self,
        point: &Point3D,
        tet_mesh: &TetMesh,
    ) -> Option<TetLocationResult> {
        let (nx, ny, _nz) = self.dims;

        // Primary cell
        let ix = cell_index_1d(point.x, self.bounds_min.x, self.cell_size.0, self.dims.0);
        let iy = cell_index_1d(point.y, self.bounds_min.y, self.cell_size.1, self.dims.1);
        let iz = cell_index_1d(point.z, self.bounds_min.z, self.cell_size.2, self.dims.2);

        let cell_idx = iz * (nx * ny) + iy * nx + ix;
        if cell_idx < self.cells.len() {
            if let Some(result) = self.test_cell_tets(&self.cells[cell_idx], point, tet_mesh) {
                return Some(result);
            }
        }

        // Search 26 neighboring cells (handles boundary cases)
        for dz in -1i32..=1 {
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue; // Already tested
                    }
                    let nix = ix as i32 + dx;
                    let niy = iy as i32 + dy;
                    let niz = iz as i32 + dz;

                    if nix < 0 || niy < 0 || niz < 0 {
                        continue;
                    }
                    let nix = nix as usize;
                    let niy = niy as usize;
                    let niz = niz as usize;

                    if nix >= self.dims.0 || niy >= self.dims.1 || niz >= self.dims.2 {
                        continue;
                    }

                    let neighbor_cell_idx = niz * (nx * ny) + niy * nx + nix;
                    if let Some(result) = self.test_cell_tets(&self.cells[neighbor_cell_idx], point, tet_mesh) {
                        return Some(result);
                    }
                }
            }
        }

        None
    }

    /// Find the nearest tet to a point (fallback for points slightly outside the mesh).
    ///
    /// Searches all tets in the cell and neighbors, picks the one where the
    /// minimum barycentric coordinate is largest (least negative = closest to inside).
    /// Clamps negative barycentrics to zero and renormalizes.
    pub fn find_nearest_tet(
        &self,
        point: &Point3D,
        tet_mesh: &TetMesh,
    ) -> TetLocationResult {
        let (nx, ny, _nz) = self.dims;

        let ix = cell_index_1d(point.x, self.bounds_min.x, self.cell_size.0, self.dims.0);
        let iy = cell_index_1d(point.y, self.bounds_min.y, self.cell_size.1, self.dims.1);
        let iz = cell_index_1d(point.z, self.bounds_min.z, self.cell_size.2, self.dims.2);

        let mut best_ti = 0usize;
        let mut best_min_bary = f64::NEG_INFINITY;
        let mut best_bary = [0.25, 0.25, 0.25, 0.25];

        // Search 3x3x3 neighborhood
        let search_radius = 2i32;
        for dz in -search_radius..=search_radius {
            for dy in -search_radius..=search_radius {
                for dx in -search_radius..=search_radius {
                    let nix = ix as i32 + dx;
                    let niy = iy as i32 + dy;
                    let niz = iz as i32 + dz;

                    if nix < 0 || niy < 0 || niz < 0 {
                        continue;
                    }
                    let nix = nix as usize;
                    let niy = niy as usize;
                    let niz = niz as usize;

                    if nix >= self.dims.0 || niy >= self.dims.1 || niz >= self.dims.2 {
                        continue;
                    }

                    let cell_idx = niz * (nx * ny) + niy * nx + nix;
                    for &ti in &self.cells[cell_idx] {
                        let tet = &tet_mesh.tets[ti];
                        let bary = compute_barycentric(
                            point,
                            &tet_mesh.vertices[tet.vertices[0]],
                            &tet_mesh.vertices[tet.vertices[1]],
                            &tet_mesh.vertices[tet.vertices[2]],
                            &tet_mesh.vertices[tet.vertices[3]],
                        );

                        let min_bary = bary[0].min(bary[1]).min(bary[2]).min(bary[3]);
                        if min_bary > best_min_bary {
                            best_min_bary = min_bary;
                            best_ti = ti;
                            best_bary = bary;
                        }
                    }
                }
            }
        }

        // Clamp negative barycentrics and renormalize
        let mut clamped = [
            best_bary[0].max(0.0),
            best_bary[1].max(0.0),
            best_bary[2].max(0.0),
            best_bary[3].max(0.0),
        ];
        let sum: f64 = clamped.iter().sum();
        if sum > 1e-10 {
            for c in &mut clamped {
                *c /= sum;
            }
        } else {
            clamped = [0.25, 0.25, 0.25, 0.25];
        }

        TetLocationResult {
            tet_index: best_ti,
            bary_coords: clamped,
        }
    }

    /// Test candidate tets in a cell for containment.
    fn test_cell_tets(
        &self,
        tet_indices: &[usize],
        point: &Point3D,
        tet_mesh: &TetMesh,
    ) -> Option<TetLocationResult> {
        const EPSILON: f64 = 1e-6;

        for &ti in tet_indices {
            let tet = &tet_mesh.tets[ti];
            let bary = compute_barycentric(
                point,
                &tet_mesh.vertices[tet.vertices[0]],
                &tet_mesh.vertices[tet.vertices[1]],
                &tet_mesh.vertices[tet.vertices[2]],
                &tet_mesh.vertices[tet.vertices[3]],
            );

            // Point is inside if all barycentric coordinates >= -epsilon
            if bary[0] >= -EPSILON && bary[1] >= -EPSILON &&
               bary[2] >= -EPSILON && bary[3] >= -EPSILON {
                return Some(TetLocationResult {
                    tet_index: ti,
                    bary_coords: bary,
                });
            }
        }
        None
    }
}

/// Compute barycentric coordinates of point P in tet (v0, v1, v2, v3).
///
/// Solves [v1-v0 | v2-v0 | v3-v0]^T * [l1, l2, l3] = (P - v0)
/// then l0 = 1 - l1 - l2 - l3.
pub fn compute_barycentric(
    point: &Point3D,
    v0: &Point3D,
    v1: &Point3D,
    v2: &Point3D,
    v3: &Point3D,
) -> [f64; 4] {
    let d = point - v0;
    let e1 = v1 - v0;
    let e2 = v2 - v0;
    let e3 = v3 - v0;

    // Build 3x3 matrix [e1 | e2 | e3] (columns)
    let mat = nalgebra::Matrix3::new(
        e1.x, e2.x, e3.x,
        e1.y, e2.y, e3.y,
        e1.z, e2.z, e3.z,
    );

    match mat.try_inverse() {
        Some(inv) => {
            let coords = inv * nalgebra::Vector3::new(d.x, d.y, d.z);
            let l1 = coords.x;
            let l2 = coords.y;
            let l3 = coords.z;
            let l0 = 1.0 - l1 - l2 - l3;
            [l0, l1, l2, l3]
        }
        None => {
            // Degenerate tet — return centroid bary coords
            [0.25, 0.25, 0.25, 0.25]
        }
    }
}

/// Interpolate a point using barycentric coordinates in a tet.
///
/// point = l0*v0 + l1*v1 + l2*v2 + l3*v3
pub fn interpolate_point(
    bary: &[f64; 4],
    v0: &Point3D,
    v1: &Point3D,
    v2: &Point3D,
    v3: &Point3D,
) -> Point3D {
    Point3D::new(
        bary[0] * v0.x + bary[1] * v1.x + bary[2] * v2.x + bary[3] * v3.x,
        bary[0] * v0.y + bary[1] * v1.y + bary[2] * v2.y + bary[3] * v3.y,
        bary[0] * v0.z + bary[1] * v1.z + bary[2] * v2.z + bary[3] * v3.z,
    )
}

/// Compute 1D cell index for a coordinate value.
fn cell_index_1d(value: f64, min: f64, cell_size: f64, num_cells: usize) -> usize {
    if cell_size <= 0.0 || num_cells == 0 {
        return 0;
    }
    let idx = ((value - min) / cell_size).floor() as i64;
    idx.clamp(0, (num_cells - 1) as i64) as usize
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s3_slicer::tet_mesh::{TetMesh, Tetrahedron};

    fn make_single_tet() -> TetMesh {
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(10.0, 0.0, 0.0),
            Point3D::new(5.0, 10.0, 0.0),
            Point3D::new(5.0, 5.0, 10.0),
        ];
        let tets = vec![Tetrahedron { vertices: [0, 1, 2, 3] }];
        TetMesh::new(vertices, tets)
    }

    #[test]
    fn test_barycentric_at_vertex() {
        let v0 = Point3D::new(0.0, 0.0, 0.0);
        let v1 = Point3D::new(10.0, 0.0, 0.0);
        let v2 = Point3D::new(5.0, 10.0, 0.0);
        let v3 = Point3D::new(5.0, 5.0, 10.0);

        // Point at v0 should give (1, 0, 0, 0)
        let bary = compute_barycentric(&v0, &v0, &v1, &v2, &v3);
        assert!((bary[0] - 1.0).abs() < 1e-10);
        assert!(bary[1].abs() < 1e-10);
        assert!(bary[2].abs() < 1e-10);
        assert!(bary[3].abs() < 1e-10);

        // Point at v3 should give (0, 0, 0, 1)
        let bary = compute_barycentric(&v3, &v0, &v1, &v2, &v3);
        assert!(bary[0].abs() < 1e-10);
        assert!(bary[1].abs() < 1e-10);
        assert!(bary[2].abs() < 1e-10);
        assert!((bary[3] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_barycentric_at_centroid() {
        let v0 = Point3D::new(0.0, 0.0, 0.0);
        let v1 = Point3D::new(10.0, 0.0, 0.0);
        let v2 = Point3D::new(5.0, 10.0, 0.0);
        let v3 = Point3D::new(5.0, 5.0, 10.0);

        let centroid = Point3D::new(
            (v0.x + v1.x + v2.x + v3.x) / 4.0,
            (v0.y + v1.y + v2.y + v3.y) / 4.0,
            (v0.z + v1.z + v2.z + v3.z) / 4.0,
        );

        let bary = compute_barycentric(&centroid, &v0, &v1, &v2, &v3);
        for &b in &bary {
            assert!((b - 0.25).abs() < 1e-10, "Centroid bary should be 0.25, got {}", b);
        }
    }

    #[test]
    fn test_interpolation_roundtrip() {
        let v0 = Point3D::new(0.0, 0.0, 0.0);
        let v1 = Point3D::new(10.0, 0.0, 0.0);
        let v2 = Point3D::new(5.0, 10.0, 0.0);
        let v3 = Point3D::new(5.0, 5.0, 10.0);

        // Test with an interior point
        let p = Point3D::new(4.0, 3.0, 2.0);
        let bary = compute_barycentric(&p, &v0, &v1, &v2, &v3);
        let recovered = interpolate_point(&bary, &v0, &v1, &v2, &v3);

        assert!((recovered.x - p.x).abs() < 1e-8, "X mismatch");
        assert!((recovered.y - p.y).abs() < 1e-8, "Y mismatch");
        assert!((recovered.z - p.z).abs() < 1e-8, "Z mismatch");
    }

    #[test]
    fn test_spatial_grid_find_interior_point() {
        let mesh = make_single_tet();
        let grid = TetSpatialGrid::build(&mesh);

        // Centroid is definitely inside
        let centroid = Point3D::new(5.0, 3.75, 2.5);
        let result = grid.find_containing_tet(&centroid, &mesh);

        assert!(result.is_some(), "Centroid should be found inside tet");
        let result = result.unwrap();
        assert_eq!(result.tet_index, 0);

        // All bary coords should be positive
        for &b in &result.bary_coords {
            assert!(b >= -1e-6, "Bary coord should be non-negative: {}", b);
        }
    }

    #[test]
    fn test_spatial_grid_outside_point() {
        let mesh = make_single_tet();
        let grid = TetSpatialGrid::build(&mesh);

        // Point far outside
        let outside = Point3D::new(100.0, 100.0, 100.0);
        let result = grid.find_containing_tet(&outside, &mesh);
        assert!(result.is_none(), "Point far outside should not be found");
    }

    #[test]
    fn test_find_nearest_tet() {
        let mesh = make_single_tet();
        let grid = TetSpatialGrid::build(&mesh);

        // Point slightly outside — should find nearest tet
        let near_outside = Point3D::new(5.0, 3.75, 2.6);
        let result = grid.find_nearest_tet(&near_outside, &mesh);

        assert_eq!(result.tet_index, 0);
        // Bary coords should be non-negative (clamped)
        for &b in &result.bary_coords {
            assert!(b >= 0.0, "Clamped bary coord should be non-negative: {}", b);
        }
        // Should sum to 1
        let sum: f64 = result.bary_coords.iter().sum();
        assert!((sum - 1.0).abs() < 1e-10, "Bary coords should sum to 1, got {}", sum);
    }
}
