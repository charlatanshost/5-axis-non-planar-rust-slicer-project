// Boundary-conforming layer extraction for the tetrahedral pipeline.
//
// The volumetric pipeline produces a scalar field on a tet mesh whose level
// sets are the layers. Marching tetrahedra can turn that field into geometry,
// but what it emits is the wrong thing to print from, for three independent
// reasons:
//
//  1. It emits every edge of every isosurface polygon in every crossed tet,
//     interior ones included, each shared edge twice. A "layer" was therefore
//     the wireframe of the whole isosurface rather than its outline, and the
//     toolpath generator treated interior mesh edges as walls. A Stanford
//     bunny came out as 2.94 million moves.
//  2. Its outline, where it has one, is the boundary of the TET mesh. For any
//     model too large for TetGen that is the grid tetrahedralizer's staircase,
//     0.75 mm steps under 0.2 mm layers.
//  3. That tet mesh deliberately overshoots the model so every surface vertex
//     has an enclosing tet, so the layers overshot too: 2.6 mm of contours
//     above the top of the bunny.
//
// All three go away by asking a different question. The outline of a layer is
// where the isosurface meets the model's surface, so sample the volumetric
// field at the SURFACE mesh's vertices and take level sets there, on the
// model's own triangles. The contours then lie exactly on the model, are the
// true boundary and nothing else, and cannot leave it. The enclosing overshoot
// stops being a defect and becomes the thing that makes this work: it is what
// guarantees every surface vertex can be located in a tet.
//
// The layer normal still comes from the volumetric gradient, which is the
// normal of the isosurface itself and so the direction the tool should point.

use crate::geodesic::{extract_level_set, MeshTopology};
use crate::geometry::{Contour, Point3D, Vector3D};
use crate::mesh::Mesh;
use crate::s3_slicer::marching_tet::tet_gradient;
use crate::s3_slicer::tet_mesh::TetMesh;
use crate::s3_slicer::tet_point_location::TetSpatialGrid;
use crate::s3_slicer::tet_scalar_field::TetScalarField;
use crate::slicing::Layer;
use rayon::prelude::*;

/// Value of the volumetric field at an arbitrary point, and the tet it was
/// read from.
///
/// Points on the model surface should always be inside the tet mesh, but a
/// TetGen mesh is only as tight as its input and round-off can leave a vertex
/// a hair outside, so fall back to the nearest tet rather than failing.
fn sample_field(
    p: &Point3D,
    tet_mesh: &TetMesh,
    grid: &TetSpatialGrid,
    field: &TetScalarField,
) -> (f64, usize) {
    let loc = grid
        .find_containing_tet(p, tet_mesh)
        .unwrap_or_else(|| grid.find_nearest_tet(p, tet_mesh));
    let v = &tet_mesh.tets[loc.tet_index].vertices;
    let value = (0..4)
        .map(|i| loc.bary_coords[i] * field.value_at(v[i]))
        .sum();
    (value, loc.tet_index)
}

/// Unit gradient of the field in a given tet: the isosurface normal there.
fn layer_normal(tet_index: usize, tet_mesh: &TetMesh, field: &TetScalarField) -> Vector3D {
    let v = &tet_mesh.tets[tet_index].vertices;
    let s = [
        field.value_at(v[0]),
        field.value_at(v[1]),
        field.value_at(v[2]),
        field.value_at(v[3]),
    ];
    tet_gradient(tet_mesh, v, &s).unwrap_or_else(|| Vector3D::new(0.0, 0.0, 1.0))
}

/// Extract layers as level sets of the volumetric field on the model surface.
///
/// `surface` is the original model; `tet_mesh` and `field` are the REST-pose
/// tet mesh and the scalar field defined on its vertices. Each returned
/// contour lies on the triangles of `surface` and carries the layer normal at
/// every point.
pub fn extract_surface_layers(
    surface: &Mesh,
    tet_mesh: &TetMesh,
    field: &TetScalarField,
    layer_height: f64,
) -> Vec<Layer> {
    if layer_height <= 0.0 || surface.triangles.is_empty() || tet_mesh.tets.is_empty() {
        return Vec::new();
    }

    let topo = MeshTopology::from_mesh(surface);
    let grid = TetSpatialGrid::build(tet_mesh);

    // Field value at every surface vertex.
    let values: Vec<f64> = topo
        .vertices
        .par_iter()
        .map(|p| sample_field(p, tet_mesh, &grid, field).0)
        .collect();

    // Range over the SURFACE, not over the tet mesh. The tet mesh extends past
    // the model, so its extremes belong to space that is not part of the part.
    let s_min = values.iter().cloned().fold(f64::INFINITY, f64::min);
    let s_max = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if !s_min.is_finite() || !s_max.is_finite() || s_max <= s_min {
        log::warn!("Surface layer extraction: degenerate field range on the surface");
        return Vec::new();
    }

    let num_levels = ((s_max - s_min) / layer_height).ceil() as usize;
    log::info!(
        "Extracting surface layers: field {:.2}..{:.2} on {} surface vertices, up to {} levels",
        s_min,
        s_max,
        topo.vertices.len(),
        num_levels
    );

    let layers: Vec<Layer> = (1..=num_levels)
        .into_par_iter()
        .filter_map(|i| {
            let level = s_min + i as f64 * layer_height;
            if level >= s_max {
                return None;
            }

            let contours: Vec<Contour> = extract_level_set(&topo, &values, level)
                .into_iter()
                .filter(|c| c.points.len() >= 2)
                .map(|c| {
                    let normals: Vec<Vector3D> = c
                        .points
                        .iter()
                        .map(|p| {
                            let (_, ti) = sample_field(p, tet_mesh, &grid, field);
                            layer_normal(ti, tet_mesh, field)
                        })
                        .collect();
                    Contour::with_normals(c.points, c.closed, normals)
                })
                .collect();

            if contours.is_empty() {
                return None;
            }

            let (sum, n) = contours
                .iter()
                .flat_map(|c| c.points.iter())
                .fold((0.0, 0usize), |(s, n), p| (s + p.z, n + 1));
            let z = if n > 0 { sum / n as f64 } else { level };
            Some(Layer::new(z, contours, layer_height))
        })
        .collect();

    log::info!("  Surface layers extracted: {}", layers.len());
    layers
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Triangle;

    /// A closed axis-aligned box from (0,0,0) to (sx,sy,sz).
    fn box_mesh(sx: f64, sy: f64, sz: f64) -> Mesh {
        let p = |x: f64, y: f64, z: f64| Point3D::new(x, y, z);
        let v = [
            p(0.0, 0.0, 0.0), p(sx, 0.0, 0.0), p(sx, sy, 0.0), p(0.0, sy, 0.0),
            p(0.0, 0.0, sz),  p(sx, 0.0, sz),  p(sx, sy, sz),  p(0.0, sy, sz),
        ];
        let q = |a: usize, b: usize, c: usize, d: usize| {
            vec![Triangle::new(v[a], v[b], v[c]), Triangle::new(v[a], v[c], v[d])]
        };
        let mut t = Vec::new();
        t.extend(q(0, 3, 2, 1));
        t.extend(q(4, 5, 6, 7));
        t.extend(q(0, 1, 5, 4));
        t.extend(q(1, 2, 6, 5));
        t.extend(q(2, 3, 7, 6));
        t.extend(q(3, 0, 4, 7));
        Mesh::new(t).expect("box is a valid mesh")
    }

    /// The grid tetrahedralizer with a plain height field: the configuration
    /// that used to produce staircase outlines, overshoot and interior
    /// wireframe all at once.
    fn grid_setup(surface: &Mesh) -> (TetMesh, TetScalarField) {
        let tet_mesh = TetMesh::from_surface_mesh_grid(surface, 12).expect("grid tet mesh");
        let values: Vec<f64> = tet_mesh.vertices.iter().map(|v| v.z).collect();
        let min_value = values.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_value = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        (tet_mesh, TetScalarField { values, min_value, max_value })
    }

    /// Regression test for D-18. The tet mesh overshoots the model on purpose,
    /// and layers cut from it inherited the overshoot: contours 2.6 mm above a
    /// bunny, and below the base. Layers cut on the model surface cannot leave
    /// the model.
    #[test]
    fn test_layers_stay_inside_the_model() {
        let surface = box_mesh(20.0, 20.0, 10.0);
        let (tet_mesh, field) = grid_setup(&surface);

        // Precondition: the tet mesh really does overshoot, or this proves nothing.
        assert!(
            tet_mesh.bounds_max.z > surface.bounds_max.z + 1e-6
                || tet_mesh.bounds_min.z < surface.bounds_min.z - 1e-6,
            "grid tet mesh was expected to extend past the model"
        );

        let layers = extract_surface_layers(&surface, &tet_mesh, &field, 1.0);
        assert!(layers.len() >= 8, "expected ~9 layers, got {}", layers.len());

        for layer in &layers {
            for c in &layer.contours {
                for p in &c.points {
                    assert!(
                        p.x >= -1e-6 && p.x <= 20.0 + 1e-6
                            && p.y >= -1e-6 && p.y <= 20.0 + 1e-6
                            && p.z >= -1e-6 && p.z <= 10.0 + 1e-6,
                        "contour point {:?} lies outside the model",
                        p
                    );
                }
            }
        }
    }

    /// The outline must lie on the model's surface, not on the tetrahedralizer's
    /// staircase, and must be the boundary only: one closed loop per level for
    /// a box, not the wireframe of the isosurface.
    #[test]
    fn test_layers_are_clean_outlines_on_the_surface() {
        let surface = box_mesh(20.0, 20.0, 10.0);
        let (tet_mesh, field) = grid_setup(&surface);
        let layers = extract_surface_layers(&surface, &tet_mesh, &field, 1.0);

        for layer in &layers {
            assert_eq!(
                layer.contours.len(),
                1,
                "a box cross-section is one loop, got {} at z={:.2}",
                layer.contours.len(),
                layer.z
            );
            let c = &layer.contours[0];
            assert!(c.closed, "outline at z={:.2} is not closed", layer.z);

            for p in &c.points {
                // On a side wall of the box: x or y pinned to an extreme.
                let on_wall = p.x.abs() < 1e-6
                    || (p.x - 20.0).abs() < 1e-6
                    || p.y.abs() < 1e-6
                    || (p.y - 20.0).abs() < 1e-6;
                assert!(on_wall, "contour point {:?} is not on the model surface", p);
            }

            // A height field has flat level sets.
            let z0 = c.points[0].z;
            for p in &c.points {
                assert!((p.z - z0).abs() < 1e-6, "level set of z is not flat");
            }
        }
    }

    /// Every contour point carries the isosurface normal. For a height field
    /// that is straight up, everywhere.
    #[test]
    fn test_layers_carry_the_layer_normal() {
        let surface = box_mesh(20.0, 20.0, 10.0);
        let (tet_mesh, field) = grid_setup(&surface);
        let layers = extract_surface_layers(&surface, &tet_mesh, &field, 1.0);
        assert!(!layers.is_empty());

        for layer in &layers {
            for c in &layer.contours {
                let normals = c.normals.as_ref().expect("contours must carry normals");
                assert_eq!(normals.len(), c.points.len());
                for n in normals {
                    assert!(
                        (n.z - 1.0).abs() < 1e-6 && n.x.abs() < 1e-6 && n.y.abs() < 1e-6,
                        "normal of a height field should be +Z, got {:?}",
                        n
                    );
                }
            }
        }
    }

    /// A tilted field must give tilted layers: the mechanism has to follow the
    /// field, not just reproduce planar slicing.
    #[test]
    fn test_tilted_field_gives_tilted_layers() {
        let surface = box_mesh(20.0, 20.0, 10.0);
        let (tet_mesh, mut field) = grid_setup(&surface);
        // s = z + 0.2 x: level sets are planes tilted about Y.
        field.values = tet_mesh.vertices.iter().map(|v| v.z + 0.2 * v.x).collect();
        field.min_value = field.values.iter().cloned().fold(f64::INFINITY, f64::min);
        field.max_value = field.values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let layers = extract_surface_layers(&surface, &tet_mesh, &field, 1.0);
        assert!(!layers.is_empty());

        // A level set that spans the full box in X rises by 0.2 * 20 = 4 mm.
        let widest = layers
            .iter()
            .flat_map(|l| l.contours.iter())
            .map(|c| {
                let lo = c.points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
                let hi = c.points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
                hi - lo
            })
            .fold(0.0f64, f64::max);
        assert!(
            (widest - 4.0).abs() < 1e-6,
            "expected a 4 mm rise across the tilted layer, got {:.6}",
            widest
        );

        // And the normal is the normalised gradient (0.2, 0, 1).
        let expect = Vector3D::new(0.2, 0.0, 1.0).normalize();
        let n = layers[layers.len() / 2].contours[0].normals.as_ref().unwrap()[0];
        assert!((n - expect).norm() < 1e-6, "normal {:?} != {:?}", n, expect);
    }
}
