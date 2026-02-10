// Voxel-based mesh reconstruction for TetGen compatibility
//
// Converts a potentially self-intersecting surface mesh into a clean,
// manifold surface via SDF computation + Marching Cubes extraction.
// Marching Cubes guarantees non-self-intersecting output because triangles
// are bounded within their cube cells.

use crate::geometry::{Point3D, Triangle};
use crate::mesh::Mesh;
use std::collections::HashMap;

/// Reconstruct a clean, manifold surface mesh from a potentially self-intersecting input.
///
/// Uses a voxel grid to compute a signed distance field, then extracts the
/// zero-isosurface using Marching Cubes. The output is guaranteed to be
/// non-self-intersecting because each triangle is confined to its cube cell.
pub fn voxel_remesh(mesh: &Mesh, grid_res: usize) -> Mesh {
    let (vertices, faces) = voxel_remesh_indexed(mesh, grid_res);
    let triangles: Vec<Triangle> = faces.iter()
        .filter(|f| f[0] != f[1] && f[1] != f[2] && f[0] != f[2])
        .map(|f| Triangle::new(vertices[f[0]], vertices[f[1]], vertices[f[2]]))
        .collect();
    log::info!("  Voxel remesh output: {} triangles", triangles.len());
    Mesh::new(triangles).unwrap_or_else(|_| {
        Mesh { triangles: Vec::new(), bounds_min: Point3D::origin(), bounds_max: Point3D::origin() }
    })
}

/// Like `voxel_remesh` but returns indexed data (vertices + face indices).
/// This avoids lossy vertex deduplication when passing to TetGen.
pub fn voxel_remesh_indexed(mesh: &Mesh, grid_res: usize) -> (Vec<Point3D>, Vec<[usize; 3]>) {
    log::info!("Voxel remeshing: resolution={}", grid_res);
    log::info!("  Input: {} triangles", mesh.triangles.len());

    let grid = SdfGrid::from_mesh(mesh, grid_res);
    grid.extract_marching_cubes_indexed()
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
// Marching Cubes Lookup Tables (Paul Bourke / Lorensen & Cline)
// ============================================================================

/// For each of 256 cube configurations, which of the 12 edges are intersected.
/// Bit i set means edge i has a crossing.
#[rustfmt::skip]
static EDGE_TABLE: [u16; 256] = [
    0x000, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x099, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x033, 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0x0aa, 0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x066, 0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0x0ff, 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x055, 0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0x0cc,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0x0cc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x055, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0x0ff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x066, 0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0x0aa, 0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x033, 0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x099, 0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x000,
];

/// For each of 256 cube configurations, up to 5 triangles defined by edge indices.
/// Each row is terminated by -1. Edges are numbered 0-11.
#[rustfmt::skip]
static TRI_TABLE: [[i8; 16]; 256] = [
    [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 8, 3, 9, 8, 1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 1, 2,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 2,10, 0, 2, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 8, 3, 2,10, 8,10, 9, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 3,11, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0,11, 2, 8,11, 0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 9, 0, 2, 3,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1,11, 2, 1, 9,11, 9, 8,11,-1,-1,-1,-1,-1,-1,-1],
    [ 3,10, 1,11,10, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0,10, 1, 0, 8,10, 8,11,10,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 9, 0, 3,11, 9,11,10, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 8,10,10, 8,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 7, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 3, 0, 7, 3, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1, 9, 8, 4, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 1, 9, 4, 7, 1, 7, 3, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 8, 4, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 4, 7, 3, 0, 4, 1, 2,10,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 2,10, 9, 0, 2, 8, 4, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 2,10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4,-1,-1,-1,-1],
    [ 8, 4, 7, 3,11, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [11, 4, 7,11, 2, 4, 2, 0, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 0, 1, 8, 4, 7, 2, 3,11,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 7,11, 9, 4,11, 9,11, 2, 9, 2, 1,-1,-1,-1,-1],
    [ 3,10, 1, 3,11,10, 7, 8, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 1,11,10, 1, 4,11, 1, 0, 4, 7,11, 4,-1,-1,-1,-1],
    [ 4, 7, 8, 9, 0,11, 9,11,10,11, 0, 3,-1,-1,-1,-1],
    [ 4, 7,11, 4,11, 9, 9,11,10,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 4, 0, 8, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 5, 4, 1, 5, 0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 5, 4, 8, 3, 5, 3, 1, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 9, 5, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 8, 1, 2,10, 4, 9, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 2,10, 5, 4, 2, 4, 0, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 2,10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8,-1,-1,-1,-1],
    [ 9, 5, 4, 2, 3,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0,11, 2, 0, 8,11, 4, 9, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 5, 4, 0, 1, 5, 2, 3,11,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 1, 5, 2, 5, 8, 2, 8,11, 4, 8, 5,-1,-1,-1,-1],
    [10, 3,11,10, 1, 3, 9, 5, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 9, 5, 0, 8, 1, 8,10, 1, 8,11,10,-1,-1,-1,-1],
    [ 5, 4, 0, 5, 0,11, 5,11,10,11, 0, 3,-1,-1,-1,-1],
    [ 5, 4, 8, 5, 8,10,10, 8,11,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 7, 8, 5, 7, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 3, 0, 9, 5, 3, 5, 7, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 7, 8, 0, 1, 7, 1, 5, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 5, 3, 3, 5, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 7, 8, 9, 5, 7,10, 1, 2,-1,-1,-1,-1,-1,-1,-1],
    [10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3,-1,-1,-1,-1],
    [ 8, 0, 2, 8, 2, 5, 8, 5, 7,10, 5, 2,-1,-1,-1,-1],
    [ 2,10, 5, 2, 5, 3, 3, 5, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 9, 5, 7, 8, 9, 3,11, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7,11,-1,-1,-1,-1],
    [ 2, 3,11, 0, 1, 8, 1, 7, 8, 1, 5, 7,-1,-1,-1,-1],
    [11, 2, 1,11, 1, 7, 7, 1, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 8, 8, 5, 7,10, 1, 3,10, 3,11,-1,-1,-1,-1],
    [ 5, 7, 0, 5, 0, 9, 7,11, 0, 1, 0,10,11,10, 0,-1],
    [11,10, 0,11, 0, 3,10, 5, 0, 8, 0, 7, 5, 7, 0,-1],
    [11,10, 5, 7,11, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [10, 6, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 5,10, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 0, 1, 5,10, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 8, 3, 1, 9, 8, 5,10, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 6, 5, 2, 6, 1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 6, 5, 1, 2, 6, 3, 0, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 6, 5, 9, 0, 6, 0, 2, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8,-1,-1,-1,-1],
    [ 2, 3,11,10, 6, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [11, 0, 8,11, 2, 0,10, 6, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1, 9, 2, 3,11, 5,10, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 5,10, 6, 1, 9, 2, 9,11, 2, 9, 8,11,-1,-1,-1,-1],
    [ 6, 3,11, 6, 5, 3, 5, 1, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8,11, 0,11, 5, 0, 5, 1, 5,11, 6,-1,-1,-1,-1],
    [ 3,11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9,-1,-1,-1,-1],
    [ 6, 5, 9, 6, 9,11,11, 9, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 5,10, 6, 4, 7, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 3, 0, 4, 7, 3, 6, 5,10,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 9, 0, 5,10, 6, 8, 4, 7,-1,-1,-1,-1,-1,-1,-1],
    [10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4,-1,-1,-1,-1],
    [ 6, 1, 2, 6, 5, 1, 4, 7, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7,-1,-1,-1,-1],
    [ 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6,-1,-1,-1,-1],
    [ 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9,-1],
    [ 3,11, 2, 7, 8, 4,10, 6, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 5,10, 6, 4, 7, 2, 4, 2, 0, 2, 7,11,-1,-1,-1,-1],
    [ 0, 1, 9, 4, 7, 8, 2, 3,11, 5,10, 6,-1,-1,-1,-1],
    [ 9, 2, 1, 9,11, 2, 9, 4,11, 7,11, 4, 5,10, 6,-1],
    [ 8, 4, 7, 3,11, 5, 3, 5, 1, 5,11, 6,-1,-1,-1,-1],
    [ 5, 1,11, 5,11, 6, 1, 0,11, 7,11, 4, 0, 4,11,-1],
    [ 0, 5, 9, 0, 6, 5, 0, 3, 6,11, 6, 3, 8, 4, 7,-1],
    [ 6, 5, 9, 6, 9,11, 4, 7, 9, 7,11, 9,-1,-1,-1,-1],
    [10, 4, 9, 6, 4,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4,10, 6, 4, 9,10, 0, 8, 3,-1,-1,-1,-1,-1,-1,-1],
    [10, 0, 1,10, 6, 0, 6, 4, 0,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1,10,-1,-1,-1,-1],
    [ 1, 4, 9, 1, 2, 4, 2, 6, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4,-1,-1,-1,-1],
    [ 0, 2, 4, 4, 2, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 3, 2, 8, 2, 4, 4, 2, 6,-1,-1,-1,-1,-1,-1,-1],
    [10, 4, 9,10, 6, 4,11, 2, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 2, 2, 8,11, 4, 9,10, 4,10, 6,-1,-1,-1,-1],
    [ 3,11, 2, 0, 1, 6, 0, 6, 4, 6, 1,10,-1,-1,-1,-1],
    [ 6, 4, 1, 6, 1,10, 4, 8, 1, 2, 1,11, 8,11, 1,-1],
    [ 9, 6, 4, 9, 3, 6, 9, 1, 3,11, 6, 3,-1,-1,-1,-1],
    [ 8,11, 1, 8, 1, 0,11, 6, 1, 9, 1, 4, 6, 4, 1,-1],
    [ 3,11, 6, 3, 6, 0, 0, 6, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 6, 4, 8,11, 6, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 7,10, 6, 7, 8,10, 8, 9,10,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 7, 3, 0,10, 7, 0, 9,10, 6, 7,10,-1,-1,-1,-1],
    [10, 6, 7, 1,10, 7, 1, 7, 8, 1, 8, 0,-1,-1,-1,-1],
    [10, 6, 7,10, 7, 1, 1, 7, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7,-1,-1,-1,-1],
    [ 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9,-1],
    [ 7, 8, 0, 7, 0, 6, 6, 0, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 3, 2, 6, 7, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 3,11,10, 6, 8,10, 8, 9, 8, 6, 7,-1,-1,-1,-1],
    [ 2, 0, 7, 2, 7,11, 0, 9, 7, 6, 7,10, 9,10, 7,-1],
    [ 1, 8, 0, 1, 7, 8, 1,10, 7, 6, 7,10, 2, 3,11,-1],
    [11, 2, 1,11, 1, 7,10, 6, 1, 6, 7, 1,-1,-1,-1,-1],
    [ 8, 9, 6, 8, 6, 7, 9, 1, 6,11, 6, 3, 1, 3, 6,-1],
    [ 0, 9, 1,11, 6, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 8, 0, 7, 0, 6, 3,11, 0,11, 6, 0,-1,-1,-1,-1],
    [ 7,11, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 6,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 8,11, 7, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1, 9,11, 7, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 1, 9, 8, 3, 1,11, 7, 6,-1,-1,-1,-1,-1,-1,-1],
    [10, 1, 2, 6,11, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 3, 0, 8, 6,11, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 9, 0, 2,10, 9, 6,11, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 6,11, 7, 2,10, 3,10, 8, 3,10, 9, 8,-1,-1,-1,-1],
    [ 7, 2, 3, 6, 2, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 0, 8, 7, 6, 0, 6, 2, 0,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 7, 6, 2, 3, 7, 0, 1, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6,-1,-1,-1,-1],
    [10, 7, 6,10, 1, 7, 1, 3, 7,-1,-1,-1,-1,-1,-1,-1],
    [10, 7, 6, 1, 7,10, 1, 8, 7, 1, 0, 8,-1,-1,-1,-1],
    [ 0, 3, 7, 0, 7,10, 0,10, 9, 6,10, 7,-1,-1,-1,-1],
    [ 7, 6,10, 7,10, 8, 8,10, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 6, 8, 4,11, 8, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 6,11, 3, 0, 6, 0, 4, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 6,11, 8, 4, 6, 9, 0, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 4, 6, 9, 6, 3, 9, 3, 1,11, 3, 6,-1,-1,-1,-1],
    [ 6, 8, 4, 6,11, 8, 2,10, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 3, 0,11, 0, 6,11, 0, 4, 6,-1,-1,-1,-1],
    [ 4,11, 8, 4, 6,11, 0, 2, 9, 2,10, 9,-1,-1,-1,-1],
    [10, 9, 3,10, 3, 2, 9, 4, 3,11, 3, 6, 4, 6, 3,-1],
    [ 8, 2, 3, 8, 4, 2, 4, 6, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 4, 2, 4, 6, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8,-1,-1,-1,-1],
    [ 1, 9, 4, 1, 4, 2, 2, 4, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 1, 3, 8, 6, 1, 8, 4, 6, 6,10, 1,-1,-1,-1,-1],
    [10, 1, 0,10, 0, 6, 6, 0, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 6, 3, 4, 3, 8, 6,10, 3, 0, 3, 9,10, 9, 3,-1],
    [10, 9, 4, 6,10, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 9, 5, 7, 6,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 4, 9, 5,11, 7, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 0, 1, 5, 4, 0, 7, 6,11,-1,-1,-1,-1,-1,-1,-1],
    [11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5,-1,-1,-1,-1],
    [ 9, 5, 4,10, 1, 2, 7, 6,11,-1,-1,-1,-1,-1,-1,-1],
    [ 6,11, 7, 1, 2,10, 0, 8, 3, 4, 9, 5,-1,-1,-1,-1],
    [ 7, 6,11, 5, 4,10, 4, 2,10, 4, 0, 2,-1,-1,-1,-1],
    [ 3, 4, 8, 3, 5, 4, 3, 2, 5,10, 5, 2,11, 7, 6,-1],
    [ 7, 2, 3, 7, 6, 2, 5, 4, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7,-1,-1,-1,-1],
    [ 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0,-1,-1,-1,-1],
    [ 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8,-1],
    [ 9, 5, 4,10, 1, 6, 1, 7, 6, 1, 3, 7,-1,-1,-1,-1],
    [ 1, 6,10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4,-1],
    [ 4, 0,10, 4,10, 5, 0, 3,10, 6,10, 7, 3, 7,10,-1],
    [ 7, 6,10, 7,10, 8, 5, 4,10, 4, 8,10,-1,-1,-1,-1],
    [ 6, 9, 5, 6,11, 9,11, 8, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 6,11, 0, 6, 3, 0, 5, 6, 0, 9, 5,-1,-1,-1,-1],
    [ 0,11, 8, 0, 5,11, 0, 1, 5, 5, 6,11,-1,-1,-1,-1],
    [ 6,11, 3, 6, 3, 5, 5, 3, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 9, 5,11, 9,11, 8,11, 5, 6,-1,-1,-1,-1],
    [ 0,11, 3, 0, 6,11, 0, 9, 6, 5, 6, 9, 1, 2,10,-1],
    [11, 8, 5,11, 5, 6, 8, 0, 5,10, 5, 2, 0, 2, 5,-1],
    [ 6,11, 3, 6, 3, 5, 2,10, 3,10, 5, 3,-1,-1,-1,-1],
    [ 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2,-1,-1,-1,-1],
    [ 9, 5, 6, 9, 6, 0, 0, 6, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8,-1],
    [ 1, 5, 6, 2, 1, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 3, 6, 1, 6,10, 3, 8, 6, 5, 6, 9, 8, 9, 6,-1],
    [10, 1, 0,10, 0, 6, 9, 5, 0, 5, 6, 0,-1,-1,-1,-1],
    [ 0, 3, 8, 5, 6,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [10, 5, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [11, 5,10, 7, 5,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [11, 5,10,11, 7, 5, 8, 3, 0,-1,-1,-1,-1,-1,-1,-1],
    [ 5,11, 7, 5,10,11, 1, 9, 0,-1,-1,-1,-1,-1,-1,-1],
    [10, 7, 5,10,11, 7, 9, 8, 1, 8, 3, 1,-1,-1,-1,-1],
    [11, 1, 2,11, 7, 1, 7, 5, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2,11,-1,-1,-1,-1],
    [ 9, 7, 5, 9, 2, 7, 9, 0, 2, 2,11, 7,-1,-1,-1,-1],
    [ 7, 5, 2, 7, 2,11, 5, 9, 2, 3, 2, 8, 9, 8, 2,-1],
    [ 2, 5,10, 2, 3, 5, 3, 7, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 2, 0, 8, 5, 2, 8, 7, 5,10, 2, 5,-1,-1,-1,-1],
    [ 9, 0, 1, 5,10, 3, 5, 3, 7, 3,10, 2,-1,-1,-1,-1],
    [ 9, 8, 2, 9, 2, 1, 8, 7, 2,10, 2, 5, 7, 5, 2,-1],
    [ 1, 3, 5, 3, 7, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 7, 0, 7, 1, 1, 7, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 0, 3, 9, 3, 5, 5, 3, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 8, 7, 5, 9, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 8, 4, 5,10, 8,10,11, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 0, 4, 5,11, 0, 5,10,11,11, 3, 0,-1,-1,-1,-1],
    [ 0, 1, 9, 8, 4,10, 8,10,11,10, 4, 5,-1,-1,-1,-1],
    [10,11, 4,10, 4, 5,11, 3, 4, 9, 4, 1, 3, 1, 4,-1],
    [ 2, 5, 1, 2, 8, 5, 2,11, 8, 4, 5, 8,-1,-1,-1,-1],
    [ 0, 4,11, 0,11, 3, 4, 5,11, 2,11, 1, 5, 1,11,-1],
    [ 0, 2, 5, 0, 5, 9, 2,11, 5, 4, 5, 8,11, 8, 5,-1],
    [ 9, 4, 5, 2,11, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 5,10, 3, 5, 2, 3, 4, 5, 3, 8, 4,-1,-1,-1,-1],
    [ 5,10, 2, 5, 2, 4, 4, 2, 0,-1,-1,-1,-1,-1,-1,-1],
    [ 3,10, 2, 3, 5,10, 3, 8, 5, 4, 5, 8, 0, 1, 9,-1],
    [ 5,10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2,-1,-1,-1,-1],
    [ 8, 4, 5, 8, 5, 3, 3, 5, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 4, 5, 1, 0, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5,-1,-1,-1,-1],
    [ 9, 4, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4,11, 7, 4, 9,11, 9,10,11,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 4, 9, 7, 9,11, 7, 9,10,11,-1,-1,-1,-1],
    [ 1,10,11, 1,11, 4, 1, 4, 0, 7, 4,11,-1,-1,-1,-1],
    [ 3, 1, 4, 3, 4, 8, 1,10, 4, 7, 4,11,10,11, 4,-1],
    [ 4,11, 7, 9,11, 4, 9, 2,11, 9, 1, 2,-1,-1,-1,-1],
    [ 9, 7, 4, 9,11, 7, 9, 1,11, 2,11, 1, 0, 8, 3,-1],
    [11, 7, 4,11, 4, 2, 2, 4, 0,-1,-1,-1,-1,-1,-1,-1],
    [11, 7, 4,11, 4, 2, 8, 3, 4, 3, 2, 4,-1,-1,-1,-1],
    [ 2, 9,10, 2, 7, 9, 2, 3, 7, 7, 4, 9,-1,-1,-1,-1],
    [ 9,10, 7, 9, 7, 4,10, 2, 7, 8, 7, 0, 2, 0, 7,-1],
    [ 3, 7,10, 3,10, 2, 7, 4,10, 1,10, 0, 4, 0,10,-1],
    [ 1,10, 2, 8, 7, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 9, 1, 4, 1, 7, 7, 1, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1,-1,-1,-1,-1],
    [ 4, 0, 3, 7, 4, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 8, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9,10, 8,10,11, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 9, 3, 9,11,11, 9,10,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1,10, 0,10, 8, 8,10,11,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 1,10,11, 3,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,11, 1,11, 9, 9,11, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 9, 3, 9,11, 1, 2, 9, 2,11, 9,-1,-1,-1,-1],
    [ 0, 2,11, 8, 0,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 2,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 3, 8, 2, 8,10,10, 8, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 9,10, 2, 0, 9, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 3, 8, 2, 8,10, 0, 1, 8, 1,10, 8,-1,-1,-1,-1],
    [ 1,10, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 3, 8, 9, 1, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 9, 1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 3, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
];

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

    /// Extract isosurface using Marching Cubes, returning indexed data.
    /// Triangles are confined within each cube cell, guaranteeing no
    /// cross-cell self-intersections.
    fn extract_marching_cubes_indexed(&self) -> (Vec<Point3D>, Vec<[usize; 3]>) {
        // Cube corner offsets (standard MC vertex ordering):
        //   0: (0,0,0)  1: (1,0,0)  2: (1,1,0)  3: (0,1,0)
        //   4: (0,0,1)  5: (1,0,1)  6: (1,1,1)  7: (0,1,1)
        let corners: [(usize, usize, usize); 8] = [
            (0,0,0), (1,0,0), (1,1,0), (0,1,0),
            (0,0,1), (1,0,1), (1,1,1), (0,1,1),
        ];
        // 12 edges as pairs of corner indices (standard MC edge ordering)
        let edges: [(usize, usize); 12] = [
            (0,1), (1,2), (2,3), (3,0),
            (4,5), (5,6), (6,7), (7,4),
            (0,4), (1,5), (2,6), (3,7),
        ];

        // Edge-to-vertex cache: keyed by the two grid vertex indices of the edge
        let mut edge_vertex_cache: HashMap<(usize, usize), usize> = HashMap::new();
        let mut vertices: Vec<Point3D> = Vec::new();
        let mut faces: Vec<[usize; 3]> = Vec::new();

        let ncx = self.nx - 1;
        let ncy = self.ny - 1;
        let ncz = self.nz - 1;

        for cz in 0..ncz {
            for cy in 0..ncy {
                for cx in 0..ncx {
                    // Get 8 corner values
                    let vals: [f64; 8] = std::array::from_fn(|i| {
                        let (dx, dy, dz) = corners[i];
                        self.get(cx + dx, cy + dy, cz + dz)
                    });

                    // Compute cube index (which corners are inside)
                    // Convention: negative = inside
                    let mut cube_index: usize = 0;
                    for i in 0..8 {
                        if vals[i] < 0.0 {
                            cube_index |= 1 << i;
                        }
                    }

                    let edge_bits = EDGE_TABLE[cube_index];
                    if edge_bits == 0 { continue; }

                    // Compute or retrieve vertex for each active edge
                    let mut edge_verts = [0usize; 12];
                    for e in 0..12 {
                        if edge_bits & (1 << e) == 0 { continue; }
                        let (c0, c1) = edges[e];
                        let (d0, d1) = (corners[c0], corners[c1]);
                        // Global grid vertex indices for this edge
                        let gi0 = (cx+d0.0) + (cy+d0.1)*self.nx + (cz+d0.2)*self.nx*self.ny;
                        let gi1 = (cx+d1.0) + (cy+d1.1)*self.nx + (cz+d1.2)*self.nx*self.ny;
                        let key = if gi0 < gi1 { (gi0, gi1) } else { (gi1, gi0) };

                        edge_verts[e] = *edge_vertex_cache.entry(key).or_insert_with(|| {
                            let v0 = vals[c0];
                            let v1 = vals[c1];
                            let t = v0.abs() / (v0.abs() + v1.abs());
                            let p0 = self.grid_point(cx+d0.0, cy+d0.1, cz+d0.2);
                            let p1 = self.grid_point(cx+d1.0, cy+d1.1, cz+d1.2);
                            let p = Point3D::from(p0.coords*(1.0-t) + p1.coords*t);
                            let idx = vertices.len();
                            vertices.push(p);
                            idx
                        });
                    }

                    // Emit triangles from TRI_TABLE
                    let tri_row = &TRI_TABLE[cube_index];
                    let mut i = 0;
                    while i < tri_row.len() && tri_row[i] != -1 {
                        let a = edge_verts[tri_row[i] as usize];
                        let b = edge_verts[tri_row[i+1] as usize];
                        let c = edge_verts[tri_row[i+2] as usize];
                        if a != b && b != c && a != c {
                            faces.push([a, b, c]);
                        }
                        i += 3;
                    }
                }
            }
        }

        log::info!("  Marching cubes: {} vertices, {} faces", vertices.len(), faces.len());

        (vertices, faces)
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
