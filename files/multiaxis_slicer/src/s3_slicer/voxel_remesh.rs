// Voxel-based mesh reconstruction for TetGen compatibility
//
// Converts a potentially self-intersecting surface mesh into a clean,
// manifold surface via SDF computation + Surface Nets extraction.
// This guarantees non-self-intersecting output that TetGen can reliably
// tetrahedralize, even for meshes like the Stanford Bunny with overlapping facets.

use crate::geometry::{Point3D, Vector3D, Triangle};
use crate::mesh::Mesh;
use std::collections::HashMap;

/// Reconstruct a clean, manifold surface mesh from a potentially self-intersecting input.
///
/// Uses a voxel grid to compute a signed distance field, then extracts the
/// zero-isosurface using Naive Surface Nets. The output is guaranteed to be
/// manifold and non-self-intersecting.
pub fn voxel_remesh(mesh: &Mesh, grid_res: usize) -> Mesh {
    log::info!("Voxel remeshing: resolution={}", grid_res);
    log::info!("  Input: {} triangles", mesh.triangles.len());

    let grid = SdfGrid::from_mesh(mesh, grid_res);
    let result = grid.extract_surface_nets();

    log::info!("  Voxel remesh output: {} triangles", result.triangles.len());
    result
}

/// Compute a good grid resolution from the mesh bounding box.
/// Aims for ~100 cells along the longest axis, clamped 40-180.
pub fn auto_grid_resolution(mesh: &Mesh) -> usize {
    let extent = mesh.bounds_max - mesh.bounds_min;
    let longest = extent.x.max(extent.y).max(extent.z);
    if longest < 1e-10 { return 40; }
    // ~100 cells along longest axis
    100usize.clamp(40, 180)
}

// ============================================================================
// SDF Grid
// ============================================================================

struct SdfGrid {
    /// SDF values at grid vertices. Negative = inside, positive = outside.
    values: Vec<f64>,
    nx: usize,
    ny: usize,
    nz: usize,
    origin: Point3D,
    cell_size: f64,
}

impl SdfGrid {
    fn get(&self, ix: usize, iy: usize, iz: usize) -> f64 {
        self.values[ix + iy * self.nx + iz * self.nx * self.ny]
    }

    fn grid_point(&self, ix: usize, iy: usize, iz: usize) -> Point3D {
        Point3D::new(
            self.origin.x + ix as f64 * self.cell_size,
            self.origin.y + iy as f64 * self.cell_size,
            self.origin.z + iz as f64 * self.cell_size,
        )
    }

    fn from_mesh(mesh: &Mesh, resolution: usize) -> Self {
        let extent = mesh.bounds_max - mesh.bounds_min;
        let longest = extent.x.max(extent.y).max(extent.z);
        let cell_size = longest / resolution as f64;

        // Pad grid by 3 cells on each side (ensures all surface is interior)
        let padding = cell_size * 3.0;
        let origin = Point3D::new(
            mesh.bounds_min.x - padding,
            mesh.bounds_min.y - padding,
            mesh.bounds_min.z - padding,
        );

        let nx = ((extent.x + 2.0 * padding) / cell_size).ceil() as usize + 1;
        let ny = ((extent.y + 2.0 * padding) / cell_size).ceil() as usize + 1;
        let nz = ((extent.z + 2.0 * padding) / cell_size).ceil() as usize + 1;

        let total = nx * ny * nz;
        log::info!("  SDF grid: {}x{}x{} = {} points, cell_size={:.3}mm",
            nx, ny, nz, total, cell_size);

        let tris: Vec<[Point3D; 3]> = mesh.triangles.iter()
            .map(|t| [t.v0, t.v1, t.v2])
            .collect();

        // Step 1: Compute unsigned distances
        let mut values = vec![f64::MAX; total];
        compute_unsigned_distances(&mut values, nx, ny, nz, &origin, cell_size, &tris);

        // Step 2: Determine signs via 3-axis ray casting with majority vote
        determine_signs(&mut values, nx, ny, nz, &origin, cell_size, &tris);

        let inside_count = values.iter().filter(|&&v| v < 0.0).count();
        log::info!("  SDF: {} inside, {} outside", inside_count, total - inside_count);

        SdfGrid { values, nx, ny, nz, origin, cell_size }
    }

    /// Extract isosurface using Naive Surface Nets.
    /// For each grid cell straddling the surface, places a vertex at the average
    /// of edge crossing points. Connects adjacent cell vertices into quads.
    fn extract_surface_nets(&self) -> Mesh {
        let ncx = self.nx - 1;
        let ncy = self.ny - 1;
        let ncz = self.nz - 1;

        // Phase 1: Compute cell vertices
        let mut cell_verts: HashMap<(usize, usize, usize), usize> = HashMap::new();
        let mut vertices: Vec<Point3D> = Vec::new();

        // Cube corner offsets: 8 vertices of a unit cell
        let corners: [(usize, usize, usize); 8] = [
            (0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
            (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1),
        ];
        // 12 edges of a cube (pairs of corner indices)
        let edges: [(usize, usize); 12] = [
            (0, 1), (1, 2), (2, 3), (3, 0), // bottom
            (4, 5), (5, 6), (6, 7), (7, 4), // top
            (0, 4), (1, 5), (2, 6), (3, 7), // vertical
        ];

        for cz in 0..ncz {
            for cy in 0..ncy {
                for cx in 0..ncx {
                    // Get 8 vertex values
                    let vals: [f64; 8] = std::array::from_fn(|i| {
                        let (dx, dy, dz) = corners[i];
                        self.get(cx + dx, cy + dy, cz + dz)
                    });

                    let pos = vals.iter().filter(|&&v| v > 0.0).count();
                    if pos == 0 || pos == 8 { continue; }

                    // Cell straddles surface — compute vertex at average of edge crossings
                    let mut avg = Vector3D::zeros();
                    let mut count = 0;

                    for &(i, j) in &edges {
                        if (vals[i] > 0.0) != (vals[j] > 0.0) {
                            let t = vals[i].abs() / (vals[i].abs() + vals[j].abs());
                            let (di, dj) = (corners[i], corners[j]);
                            let pi = self.grid_point(cx + di.0, cy + di.1, cz + di.2);
                            let pj = self.grid_point(cx + dj.0, cy + dj.1, cz + dj.2);
                            let crossing = Point3D::from(pi.coords * (1.0 - t) + pj.coords * t);
                            avg += crossing.coords;
                            count += 1;
                        }
                    }

                    if count > 0 {
                        cell_verts.insert((cx, cy, cz), vertices.len());
                        vertices.push(Point3D::from(avg / count as f64));
                    }
                }
            }
        }

        log::info!("  Surface nets: {} cell vertices", vertices.len());

        // Phase 2: Generate quads for each grid edge crossing the surface
        let mut triangles: Vec<Triangle> = Vec::new();

        // X-aligned edges: (ix,iy,iz) to (ix+1,iy,iz)
        // Shared by cells: (ix, iy-1, iz-1), (ix, iy, iz-1), (ix, iy, iz), (ix, iy-1, iz)
        for iz in 1..self.nz - 1 {
            for iy in 1..self.ny - 1 {
                for ix in 0..self.nx - 1 {
                    let v0 = self.get(ix, iy, iz);
                    let v1 = self.get(ix + 1, iy, iz);
                    if (v0 > 0.0) == (v1 > 0.0) { continue; }

                    if let (Some(&a), Some(&b), Some(&c), Some(&d)) = (
                        cell_verts.get(&(ix, iy - 1, iz - 1)),
                        cell_verts.get(&(ix, iy, iz - 1)),
                        cell_verts.get(&(ix, iy, iz)),
                        cell_verts.get(&(ix, iy - 1, iz)),
                    ) {
                        let (pa, pb, pc, pd) = (vertices[a], vertices[b], vertices[c], vertices[d]);
                        if v0 < v1 {
                            triangles.push(Triangle::new(pa, pb, pc));
                            triangles.push(Triangle::new(pa, pc, pd));
                        } else {
                            triangles.push(Triangle::new(pa, pc, pb));
                            triangles.push(Triangle::new(pa, pd, pc));
                        }
                    }
                }
            }
        }

        // Y-aligned edges: (ix,iy,iz) to (ix,iy+1,iz)
        // Shared by cells: (ix-1, iy, iz-1), (ix, iy, iz-1), (ix, iy, iz), (ix-1, iy, iz)
        for iz in 1..self.nz - 1 {
            for iy in 0..self.ny - 1 {
                for ix in 1..self.nx - 1 {
                    let v0 = self.get(ix, iy, iz);
                    let v1 = self.get(ix, iy + 1, iz);
                    if (v0 > 0.0) == (v1 > 0.0) { continue; }

                    if let (Some(&a), Some(&b), Some(&c), Some(&d)) = (
                        cell_verts.get(&(ix - 1, iy, iz - 1)),
                        cell_verts.get(&(ix, iy, iz - 1)),
                        cell_verts.get(&(ix, iy, iz)),
                        cell_verts.get(&(ix - 1, iy, iz)),
                    ) {
                        let (pa, pb, pc, pd) = (vertices[a], vertices[b], vertices[c], vertices[d]);
                        if v0 < v1 {
                            triangles.push(Triangle::new(pa, pc, pb));
                            triangles.push(Triangle::new(pa, pd, pc));
                        } else {
                            triangles.push(Triangle::new(pa, pb, pc));
                            triangles.push(Triangle::new(pa, pc, pd));
                        }
                    }
                }
            }
        }

        // Z-aligned edges: (ix,iy,iz) to (ix,iy,iz+1)
        // Shared by cells: (ix-1, iy-1, iz), (ix, iy-1, iz), (ix, iy, iz), (ix-1, iy, iz)
        for iz in 0..self.nz - 1 {
            for iy in 1..self.ny - 1 {
                for ix in 1..self.nx - 1 {
                    let v0 = self.get(ix, iy, iz);
                    let v1 = self.get(ix, iy, iz + 1);
                    if (v0 > 0.0) == (v1 > 0.0) { continue; }

                    if let (Some(&a), Some(&b), Some(&c), Some(&d)) = (
                        cell_verts.get(&(ix - 1, iy - 1, iz)),
                        cell_verts.get(&(ix, iy - 1, iz)),
                        cell_verts.get(&(ix, iy, iz)),
                        cell_verts.get(&(ix - 1, iy, iz)),
                    ) {
                        let (pa, pb, pc, pd) = (vertices[a], vertices[b], vertices[c], vertices[d]);
                        if v0 < v1 {
                            triangles.push(Triangle::new(pa, pb, pc));
                            triangles.push(Triangle::new(pa, pc, pd));
                        } else {
                            triangles.push(Triangle::new(pa, pc, pb));
                            triangles.push(Triangle::new(pa, pd, pc));
                        }
                    }
                }
            }
        }

        log::info!("  Surface nets: {} output triangles", triangles.len());

        Mesh::new(triangles).unwrap_or_else(|_| {
            log::warn!("Surface nets produced invalid mesh");
            Mesh { triangles: Vec::new(), bounds_min: Point3D::origin(), bounds_max: Point3D::origin() }
        })
    }
}

// ============================================================================
// Unsigned distance computation with spatial acceleration
// ============================================================================

fn compute_unsigned_distances(
    values: &mut [f64],
    nx: usize, ny: usize, nz: usize,
    origin: &Point3D, cell_size: f64,
    tris: &[[Point3D; 3]],
) {
    // Build 3D spatial hash grid for triangle lookup
    let bin_size = cell_size * 4.0;
    let inv_bin = 1.0 / bin_size;

    let mut bins: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
    for (ti, tri) in tris.iter().enumerate() {
        let mut tmin = tri[0];
        let mut tmax = tri[0];
        for v in &tri[1..] {
            tmin.x = tmin.x.min(v.x); tmin.y = tmin.y.min(v.y); tmin.z = tmin.z.min(v.z);
            tmax.x = tmax.x.max(v.x); tmax.y = tmax.y.max(v.y); tmax.z = tmax.z.max(v.z);
        }
        let ix0 = ((tmin.x - origin.x) * inv_bin).floor() as i32;
        let iy0 = ((tmin.y - origin.y) * inv_bin).floor() as i32;
        let iz0 = ((tmin.z - origin.z) * inv_bin).floor() as i32;
        let ix1 = ((tmax.x - origin.x) * inv_bin).floor() as i32;
        let iy1 = ((tmax.y - origin.y) * inv_bin).floor() as i32;
        let iz1 = ((tmax.z - origin.z) * inv_bin).floor() as i32;

        for bx in ix0..=ix1 {
            for by in iy0..=iy1 {
                for bz in iz0..=iz1 {
                    bins.entry((bx, by, bz)).or_default().push(ti);
                }
            }
        }
    }

    // For each grid point, find closest triangle in neighborhood
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = Point3D::new(
                    origin.x + ix as f64 * cell_size,
                    origin.y + iy as f64 * cell_size,
                    origin.z + iz as f64 * cell_size,
                );
                let bx = ((p.x - origin.x) * inv_bin).floor() as i32;
                let by = ((p.y - origin.y) * inv_bin).floor() as i32;
                let bz = ((p.z - origin.z) * inv_bin).floor() as i32;

                let mut best_dist_sq = f64::MAX;

                // Search 3x3x3 neighborhood of bins
                for dbx in -1..=1 {
                    for dby in -1..=1 {
                        for dbz in -1..=1 {
                            if let Some(tri_list) = bins.get(&(bx + dbx, by + dby, bz + dbz)) {
                                for &ti in tri_list {
                                    let tri = &tris[ti];
                                    let cp = closest_point_on_tri(&p, &tri[0], &tri[1], &tri[2]);
                                    let d2 = (cp - p).norm_squared();
                                    if d2 < best_dist_sq {
                                        best_dist_sq = d2;
                                    }
                                }
                            }
                        }
                    }
                }

                let idx = ix + iy * nx + iz * nx * ny;
                values[idx] = if best_dist_sq < f64::MAX { best_dist_sq.sqrt() } else { f64::MAX };
            }
        }
    }
}

// ============================================================================
// Sign determination via 3-axis ray casting with majority vote
// ============================================================================

fn determine_signs(
    values: &mut [f64],
    nx: usize, ny: usize, nz: usize,
    origin: &Point3D, cell_size: f64,
    tris: &[[Point3D; 3]],
) {
    let total = nx * ny * nz;
    let eps = cell_size * 1e-4; // tiny perturbation to avoid exact edge hits

    // Build 2D bins for each axis direction
    let bin2d_size = cell_size * 5.0;
    let inv_bin2d = 1.0 / bin2d_size;

    // X-axis voting
    let mut x_inside = vec![false; total];
    {
        let mut yz_bins: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (ti, tri) in tris.iter().enumerate() {
            let ymin = tri[0].y.min(tri[1].y).min(tri[2].y);
            let ymax = tri[0].y.max(tri[1].y).max(tri[2].y);
            let zmin = tri[0].z.min(tri[1].z).min(tri[2].z);
            let zmax = tri[0].z.max(tri[1].z).max(tri[2].z);
            let iy0 = ((ymin - origin.y) * inv_bin2d).floor() as i32;
            let iy1 = ((ymax - origin.y) * inv_bin2d).floor() as i32;
            let iz0 = ((zmin - origin.z) * inv_bin2d).floor() as i32;
            let iz1 = ((zmax - origin.z) * inv_bin2d).floor() as i32;
            for by in iy0..=iy1 {
                for bz in iz0..=iz1 {
                    yz_bins.entry((by, bz)).or_default().push(ti);
                }
            }
        }

        for iz in 0..nz {
            for iy in 0..ny {
                let ray_y = origin.y + iy as f64 * cell_size + eps;
                let ray_z = origin.z + iz as f64 * cell_size + eps;
                let by = ((ray_y - origin.y) * inv_bin2d).floor() as i32;
                let bz = ((ray_z - origin.z) * inv_bin2d).floor() as i32;

                let mut hits: Vec<f64> = Vec::new();
                if let Some(tri_list) = yz_bins.get(&(by, bz)) {
                    for &ti in tri_list {
                        let tri = &tris[ti];
                        if let Some(x) = ray_x_triangle(&tri[0], &tri[1], &tri[2], ray_y, ray_z) {
                            hits.push(x);
                        }
                    }
                }
                hits.sort_by(|a, b| a.partial_cmp(b).unwrap());
                hits.dedup_by(|a, b| (*a - *b).abs() < cell_size * 1e-6);

                let mut hi = 0;
                let mut inside = false;
                for ix in 0..nx {
                    let x = origin.x + ix as f64 * cell_size;
                    while hi < hits.len() && hits[hi] < x {
                        inside = !inside;
                        hi += 1;
                    }
                    if inside {
                        x_inside[ix + iy * nx + iz * nx * ny] = true;
                    }
                }
            }
        }
    }

    // Y-axis voting
    let mut y_inside = vec![false; total];
    {
        let mut xz_bins: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (ti, tri) in tris.iter().enumerate() {
            let xmin = tri[0].x.min(tri[1].x).min(tri[2].x);
            let xmax = tri[0].x.max(tri[1].x).max(tri[2].x);
            let zmin = tri[0].z.min(tri[1].z).min(tri[2].z);
            let zmax = tri[0].z.max(tri[1].z).max(tri[2].z);
            let ix0 = ((xmin - origin.x) * inv_bin2d).floor() as i32;
            let ix1 = ((xmax - origin.x) * inv_bin2d).floor() as i32;
            let iz0 = ((zmin - origin.z) * inv_bin2d).floor() as i32;
            let iz1 = ((zmax - origin.z) * inv_bin2d).floor() as i32;
            for bx in ix0..=ix1 {
                for bz in iz0..=iz1 {
                    xz_bins.entry((bx, bz)).or_default().push(ti);
                }
            }
        }

        for iz in 0..nz {
            for ix in 0..nx {
                let ray_x = origin.x + ix as f64 * cell_size + eps;
                let ray_z = origin.z + iz as f64 * cell_size + eps;
                let bx = ((ray_x - origin.x) * inv_bin2d).floor() as i32;
                let bz = ((ray_z - origin.z) * inv_bin2d).floor() as i32;

                let mut hits: Vec<f64> = Vec::new();
                if let Some(tri_list) = xz_bins.get(&(bx, bz)) {
                    for &ti in tri_list {
                        let tri = &tris[ti];
                        if let Some(y) = ray_y_triangle(&tri[0], &tri[1], &tri[2], ray_x, ray_z) {
                            hits.push(y);
                        }
                    }
                }
                hits.sort_by(|a, b| a.partial_cmp(b).unwrap());
                hits.dedup_by(|a, b| (*a - *b).abs() < cell_size * 1e-6);

                let mut hi = 0;
                let mut inside = false;
                for iy in 0..ny {
                    let y = origin.y + iy as f64 * cell_size;
                    while hi < hits.len() && hits[hi] < y {
                        inside = !inside;
                        hi += 1;
                    }
                    if inside {
                        y_inside[ix + iy * nx + iz * nx * ny] = true;
                    }
                }
            }
        }
    }

    // Z-axis voting
    let mut z_inside = vec![false; total];
    {
        let mut xy_bins: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (ti, tri) in tris.iter().enumerate() {
            let xmin = tri[0].x.min(tri[1].x).min(tri[2].x);
            let xmax = tri[0].x.max(tri[1].x).max(tri[2].x);
            let ymin = tri[0].y.min(tri[1].y).min(tri[2].y);
            let ymax = tri[0].y.max(tri[1].y).max(tri[2].y);
            let ix0 = ((xmin - origin.x) * inv_bin2d).floor() as i32;
            let ix1 = ((xmax - origin.x) * inv_bin2d).floor() as i32;
            let iy0 = ((ymin - origin.y) * inv_bin2d).floor() as i32;
            let iy1 = ((ymax - origin.y) * inv_bin2d).floor() as i32;
            for bx in ix0..=ix1 {
                for by in iy0..=iy1 {
                    xy_bins.entry((bx, by)).or_default().push(ti);
                }
            }
        }

        for iy in 0..ny {
            for ix in 0..nx {
                let ray_x = origin.x + ix as f64 * cell_size + eps;
                let ray_y = origin.y + iy as f64 * cell_size + eps;
                let bx = ((ray_x - origin.x) * inv_bin2d).floor() as i32;
                let by = ((ray_y - origin.y) * inv_bin2d).floor() as i32;

                let mut hits: Vec<f64> = Vec::new();
                if let Some(tri_list) = xy_bins.get(&(bx, by)) {
                    for &ti in tri_list {
                        let tri = &tris[ti];
                        if let Some(z) = ray_z_triangle(&tri[0], &tri[1], &tri[2], ray_x, ray_y) {
                            hits.push(z);
                        }
                    }
                }
                hits.sort_by(|a, b| a.partial_cmp(b).unwrap());
                hits.dedup_by(|a, b| (*a - *b).abs() < cell_size * 1e-6);

                let mut hi = 0;
                let mut inside = false;
                for iz in 0..nz {
                    let z = origin.z + iz as f64 * cell_size;
                    while hi < hits.len() && hits[hi] < z {
                        inside = !inside;
                        hi += 1;
                    }
                    if inside {
                        z_inside[ix + iy * nx + iz * nx * ny] = true;
                    }
                }
            }
        }
    }

    // Majority vote: 2+ axes say inside → inside (negate SDF value)
    for i in 0..total {
        let votes = x_inside[i] as u8 + y_inside[i] as u8 + z_inside[i] as u8;
        if votes >= 2 {
            values[i] = -values[i].abs();
        } else {
            values[i] = values[i].abs();
        }
    }
}

// ============================================================================
// Geometry helpers
// ============================================================================

/// Closest point on triangle (a, b, c) to point p.
/// Ericson (2004) "Real-Time Collision Detection" algorithm.
fn closest_point_on_tri(p: &Point3D, a: &Point3D, b: &Point3D, c: &Point3D) -> Point3D {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 { return *a; }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 { return *b; }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return Point3D::from(a.coords + ab * v);
    }

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 { return *c; }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return Point3D::from(a.coords + ac * w);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Point3D::from(b.coords + (c - b) * w);
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    Point3D::from(a.coords + ab * v + ac * w)
}

/// Ray-triangle intersection for +X axis ray at (y0, z0).
/// Returns the X coordinate of intersection, or None.
fn ray_x_triangle(a: &Point3D, b: &Point3D, c: &Point3D, y0: f64, z0: f64) -> Option<f64> {
    // Project triangle to YZ plane, check if (y0, z0) is inside
    let d = (b.y - a.y) * (c.z - a.z) - (b.z - a.z) * (c.y - a.y);
    if d.abs() < 1e-15 { return None; }
    let inv_d = 1.0 / d;

    let u = ((y0 - a.y) * (c.z - a.z) - (z0 - a.z) * (c.y - a.y)) * inv_d;
    if u < 0.0 || u > 1.0 { return None; }

    let v = ((b.y - a.y) * (z0 - a.z) - (b.z - a.z) * (y0 - a.y)) * inv_d;
    if v < 0.0 || u + v > 1.0 { return None; }

    Some(a.x * (1.0 - u - v) + b.x * u + c.x * v)
}

/// Ray-triangle intersection for +Y axis ray at (x0, z0).
fn ray_y_triangle(a: &Point3D, b: &Point3D, c: &Point3D, x0: f64, z0: f64) -> Option<f64> {
    let d = (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
    if d.abs() < 1e-15 { return None; }
    let inv_d = 1.0 / d;

    let u = ((x0 - a.x) * (c.z - a.z) - (z0 - a.z) * (c.x - a.x)) * inv_d;
    if u < 0.0 || u > 1.0 { return None; }

    let v = ((b.x - a.x) * (z0 - a.z) - (b.z - a.z) * (x0 - a.x)) * inv_d;
    if v < 0.0 || u + v > 1.0 { return None; }

    Some(a.y * (1.0 - u - v) + b.y * u + c.y * v)
}

/// Ray-triangle intersection for +Z axis ray at (x0, y0).
fn ray_z_triangle(a: &Point3D, b: &Point3D, c: &Point3D, x0: f64, y0: f64) -> Option<f64> {
    let d = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if d.abs() < 1e-15 { return None; }
    let inv_d = 1.0 / d;

    let u = ((x0 - a.x) * (c.y - a.y) - (y0 - a.y) * (c.x - a.x)) * inv_d;
    if u < 0.0 || u > 1.0 { return None; }

    let v = ((b.x - a.x) * (y0 - a.y) - (b.y - a.y) * (x0 - a.x)) * inv_d;
    if v < 0.0 || u + v > 1.0 { return None; }

    Some(a.z * (1.0 - u - v) + b.z * u + c.z * v)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_unit_cube_mesh() -> Mesh {
        let v = [
            Point3D::new(0.0, 0.0, 0.0), Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(1.0, 1.0, 0.0), Point3D::new(0.0, 1.0, 0.0),
            Point3D::new(0.0, 0.0, 1.0), Point3D::new(1.0, 0.0, 1.0),
            Point3D::new(1.0, 1.0, 1.0), Point3D::new(0.0, 1.0, 1.0),
        ];
        let triangles = vec![
            Triangle::new(v[0], v[2], v[1]), Triangle::new(v[0], v[3], v[2]),
            Triangle::new(v[4], v[5], v[6]), Triangle::new(v[4], v[6], v[7]),
            Triangle::new(v[0], v[1], v[5]), Triangle::new(v[0], v[5], v[4]),
            Triangle::new(v[3], v[7], v[6]), Triangle::new(v[3], v[6], v[2]),
            Triangle::new(v[0], v[4], v[7]), Triangle::new(v[0], v[7], v[3]),
            Triangle::new(v[1], v[2], v[6]), Triangle::new(v[1], v[6], v[5]),
        ];
        Mesh::new(triangles).unwrap()
    }

    #[test]
    fn test_ray_x_triangle() {
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(0.0, 1.0, 0.0);
        let c = Point3D::new(0.0, 0.0, 1.0);
        // Ray at (0.2, 0.2) should hit the triangle
        let hit = ray_x_triangle(&a, &b, &c, 0.2, 0.2);
        assert!(hit.is_some());
        assert!((hit.unwrap() - 0.0).abs() < 1e-10);

        // Ray at (0.9, 0.9) should miss (outside triangle)
        let miss = ray_x_triangle(&a, &b, &c, 0.9, 0.9);
        assert!(miss.is_none());
    }

    #[test]
    fn test_voxel_remesh_cube() {
        let mesh = make_unit_cube_mesh();
        let result = voxel_remesh(&mesh, 20);

        assert!(result.triangles.len() > 0, "Voxel remesh should produce triangles");
        // Should produce a closed surface approximating the cube
        // With resolution 20, we expect hundreds of triangles
        assert!(result.triangles.len() > 50,
            "Expected > 50 triangles, got {}", result.triangles.len());
    }

    #[test]
    fn test_closest_point_on_tri() {
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(1.0, 0.0, 0.0);
        let c = Point3D::new(0.0, 1.0, 0.0);

        // Point above triangle interior
        let p = Point3D::new(0.2, 0.2, 1.0);
        let cp = closest_point_on_tri(&p, &a, &b, &c);
        assert!((cp.x - 0.2).abs() < 1e-10);
        assert!((cp.y - 0.2).abs() < 1e-10);
        assert!((cp.z - 0.0).abs() < 1e-10);

        // Point closest to vertex a
        let p2 = Point3D::new(-1.0, -1.0, 0.0);
        let cp2 = closest_point_on_tri(&p2, &a, &b, &c);
        assert!((cp2 - a).norm() < 1e-10);
    }

    #[test]
    fn test_sdf_signs_cube() {
        let mesh = make_unit_cube_mesh();
        let grid = SdfGrid::from_mesh(&mesh, 16);

        // Center of the cube should be inside (negative SDF)
        let cx = grid.nx / 2;
        let cy = grid.ny / 2;
        let cz = grid.nz / 2;
        let center_val = grid.get(cx, cy, cz);
        assert!(center_val < 0.0, "Center should be inside (negative), got {}", center_val);

        // Corner of grid (outside padding) should be outside (positive SDF)
        let corner_val = grid.get(0, 0, 0);
        assert!(corner_val > 0.0, "Corner should be outside (positive), got {}", corner_val);
    }
}
