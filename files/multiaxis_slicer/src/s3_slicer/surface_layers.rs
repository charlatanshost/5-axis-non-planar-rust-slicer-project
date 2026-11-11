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

/// The volumetric field and what is needed to query it, shared by every layer
/// cut from it.
struct FieldData {
    tet_mesh: TetMesh,
    grid: TetSpatialGrid,
    field: TetScalarField,
    /// Height range of the tet mesh, which bounds the search.
    z_lo: f64,
    z_hi: f64,
    /// Height range of the model itself, which bounds acceptable answers.
    model_z_lo: f64,
    model_z_hi: f64,
}

impl std::fmt::Debug for FieldData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FieldData")
            .field("tets", &self.tet_mesh.tets.len())
            .finish()
    }
}

/// One level set of the volumetric field: the surface a layer lies on.
///
/// The layer's contours are where this surface meets the model. Inner wall
/// loops and infill are placed in XY inside those contours and then need a
/// height, and the right height is this surface's -- not the model's outer
/// skin, which is what ray-casting gives, and not a blend of boundary
/// heights, which is what inverse-distance weighting gives. Both are wrong
/// in the interior of any layer that is not flat.
#[derive(Debug)]
pub struct FieldSurface {
    data: std::sync::Arc<FieldData>,
    level: f64,
}

impl FieldSurface {
    /// Field value and its vertical rate of change at a point.
    ///
    /// Evaluates the located tet's linear function directly rather than
    /// interpolating with clamped barycentrics, so a point slightly outside
    /// the tet mesh extrapolates linearly instead of hitting a plateau, which
    /// would stall the solve below.
    fn eval(&self, p: &Point3D) -> Option<(f64, f64)> {
        let d = &*self.data;
        let loc = d
            .grid
            .find_containing_tet(p, &d.tet_mesh)
            .unwrap_or_else(|| d.grid.find_nearest_tet(p, &d.tet_mesh));
        let v = &d.tet_mesh.tets[loc.tet_index].vertices;
        let s = [
            d.field.value_at(v[0]),
            d.field.value_at(v[1]),
            d.field.value_at(v[2]),
            d.field.value_at(v[3]),
        ];
        let g = crate::s3_slicer::marching_tet::tet_gradient_raw(&d.tet_mesh, v, &s)?;
        let value = s[0] + g.dot(&(*p - d.tet_mesh.vertices[v[0]]));
        Some((value, g.z))
    }
}

impl crate::slicing::LayerSurface for FieldSurface {
    /// Solve `field(x, y, z) = level` for z.
    ///
    /// The field is piecewise linear, so Newton's method along the vertical
    /// lands exactly once it is in the right tet, typically in two or three
    /// steps. Steps are capped so a shallow gradient cannot throw the iterate
    /// across the model.
    fn z_at(&self, x: f64, y: f64, near_z: f64) -> Option<f64> {
        let d = &*self.data;
        let mut z = near_z.clamp(d.z_lo, d.z_hi);
        const MAX_STEP: f64 = 3.0;

        for _ in 0..16 {
            let (value, dz_rate) = self.eval(&Point3D::new(x, y, z))?;
            let err = self.level - value;
            if err.abs() < 1e-7 {
                return self.inside_model(z);
            }
            if !dz_rate.is_finite() || dz_rate.abs() < 1e-9 {
                // Level set is vertical here: no single height above (x, y).
                return None;
            }
            z += (err / dz_rate).clamp(-MAX_STEP, MAX_STEP);
            if z < d.z_lo - MAX_STEP || z > d.z_hi + MAX_STEP {
                return None;
            }
        }

        // Accept a near miss.
        let (value, _) = self.eval(&Point3D::new(x, y, z))?;
        if (self.level - value).abs() < 1e-4 {
            return self.inside_model(z);
        }

        // Newton did not settle: the field is only piecewise linear, and
        // across a kink between tets the iterate can bounce between two of
        // them indefinitely. Fall back to something that cannot cycle --
        // bracket the nearest sign change around the starting guess, then
        // bisect.
        let start = near_z.clamp(d.z_lo, d.z_hi);
        let g = |z: f64| self.eval(&Point3D::new(x, y, z)).map(|(v, _)| v - self.level);
        let g0 = g(start)?;
        const H: f64 = 0.2;
        const REACH: usize = 30; // 6 mm either way
        let mut bracket: Option<(f64, f64, f64)> = None; // (z_a, z_b, g_a)
        let (mut up_prev, mut dn_prev) = ((start, g0), (start, g0));
        for k in 1..=REACH {
            let zu = start + k as f64 * H;
            if let Some(gu) = g(zu) {
                if gu == 0.0 { return self.inside_model(zu); }
                if (gu > 0.0) != (up_prev.1 > 0.0) {
                    bracket = Some((up_prev.0, zu, up_prev.1));
                    break;
                }
                up_prev = (zu, gu);
            }
            let zd = start - k as f64 * H;
            if let Some(gd) = g(zd) {
                if gd == 0.0 { return self.inside_model(zd); }
                if (gd > 0.0) != (dn_prev.1 > 0.0) {
                    bracket = Some((dn_prev.0, zd, dn_prev.1));
                    break;
                }
                dn_prev = (zd, gd);
            }
        }
        let (mut a, mut b, ga) = bracket?;
        for _ in 0..50 {
            let m = 0.5 * (a + b);
            let gm = g(m)?;
            if (gm > 0.0) == (ga > 0.0) { a = m; } else { b = m; }
            if (b - a).abs() < 1e-7 { break; }
        }
        self.inside_model(0.5 * (a + b))
    }
}

impl FieldSurface {
    /// The tet mesh deliberately extends past the model, so the level set
    /// does too. A height outside the model's own range is the layer leaving
    /// the part, not somewhere to put plastic.
    fn inside_model(&self, z: f64) -> Option<f64> {
        let d = &*self.data;
        if z >= d.model_z_lo - 1e-6 && z <= d.model_z_hi + 1e-6 { Some(z) } else { None }
    }
}

/// Split every surface triangle edge longer than `max_edge`, conformingly.
///
/// The level sets below are traced through a field that is sampled at surface
/// vertices and taken to be linear across each triangle. The real field is
/// linear only within a tet, and a CAD-style model has triangles spanning
/// dozens of tets: on a Benchy, with hull triangles up to 18 mm long, 29 % of
/// outline points sat more than 0.05 mm off the layer they were meant to
/// outline (worst 0.88 mm), and the walls faithfully followed them. Sampling
/// at tet resolution removes the disagreement at its source.
///
/// Whether an edge is split depends only on its own length, and both triangles
/// sharing it compute the same midpoint, so no T-junctions are created and the
/// level sets stay closed.
fn refine_surface(surface: &Mesh, max_edge: f64) -> Vec<crate::geometry::Triangle> {
    use crate::geometry::Triangle;
    let mid = |a: &Point3D, b: &Point3D| Point3D::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z));
    let long = |a: &Point3D, b: &Point3D| (*a - *b).norm() > max_edge;

    let mut done: Vec<Triangle> = Vec::with_capacity(surface.triangles.len());
    let mut work: Vec<Triangle> = surface.triangles.clone();
    while let Some(t) = work.pop() {
        let v = [t.v0, t.v1, t.v2];
        // Edge i runs from v[i] to v[i+1].
        let marked = [long(&v[0], &v[1]), long(&v[1], &v[2]), long(&v[2], &v[0])];
        let m = |i: usize| mid(&v[i], &v[(i + 1) % 3]);
        match marked.iter().filter(|&&x| x).count() {
            0 => done.push(t),
            3 => {
                let (a, b, c) = (m(0), m(1), m(2));
                work.push(Triangle::new(v[0], a, c));
                work.push(Triangle::new(a, v[1], b));
                work.push(Triangle::new(c, b, v[2]));
                work.push(Triangle::new(a, b, c));
            }
            1 => {
                let i = marked.iter().position(|&x| x).unwrap();
                let (p, q, r) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
                let a = m(i);
                work.push(Triangle::new(p, a, r));
                work.push(Triangle::new(a, q, r));
            }
            _ => {
                // Two marked edges meet at q; the third edge p-r is short.
                let i = marked.iter().position(|&x| !x).unwrap();
                let (r, p, q) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
                // Marked: p-q (edge i+1) and q-r (edge i+2).
                let (a, b) = (m((i + 1) % 3), m((i + 2) % 3));
                work.push(Triangle::new(a, q, b));
                work.push(Triangle::new(r, p, a));
                work.push(Triangle::new(r, a, b));
            }
        }
    }
    done
}

/// Longest surface edge to leave unsplit: about one tet across, relaxed on
/// very large models so refinement cannot run away with memory.
fn refine_edge_limit(surface: &Mesh) -> f64 {
    const TARGET_EDGE: f64 = 1.0;
    const TRIANGLE_BUDGET: f64 = 1.5e6;
    let area: f64 = surface
        .triangles
        .iter()
        .map(|t| 0.5 * (t.v1 - t.v0).cross(&(t.v2 - t.v0)).norm())
        .sum();
    // An equilateral triangle of edge L covers 0.433 L^2.
    TARGET_EDGE.max((area / (0.433 * TRIANGLE_BUDGET)).sqrt())
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
    extract_surface_layers_refined(surface, tet_mesh, field, layer_height, refine_edge_limit(surface))
}

/// As `extract_surface_layers`, with the surface refinement limit explicit.
fn extract_surface_layers_refined(
    surface: &Mesh,
    tet_mesh: &TetMesh,
    field: &TetScalarField,
    layer_height: f64,
    max_edge: f64,
) -> Vec<Layer> {
    if layer_height <= 0.0 || surface.triangles.is_empty() || tet_mesh.tets.is_empty() {
        return Vec::new();
    }

    let refined = match Mesh::new(refine_surface(surface, max_edge)) {
        Ok(m) => m,
        Err(_) => return Vec::new(),
    };
    let topo = MeshTopology::from_mesh(&refined);

    // Shared by every layer, so each can answer "how high are you at (x, y)?"
    // for the wall loops and infill placed inside its outline.
    let data = std::sync::Arc::new(FieldData {
        grid: TetSpatialGrid::build(tet_mesh),
        tet_mesh: tet_mesh.clone(),
        field: field.clone(),
        z_lo: tet_mesh.bounds_min.z,
        z_hi: tet_mesh.bounds_max.z,
        model_z_lo: surface.bounds_min.z,
        model_z_hi: surface.bounds_max.z,
    });
    let grid = &data.grid;

    // Field value at every surface vertex.
    let values: Vec<f64> = topo
        .vertices
        .par_iter()
        .map(|p| sample_field(p, tet_mesh, grid, field).0)
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
                            let (_, ti) = sample_field(p, tet_mesh, grid, field);
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
            // extract_level_set nudges the level off exact vertex values by a
            // relative 1e-9; far below anything the solve tolerance resolves.
            let surface = std::sync::Arc::new(FieldSurface { data: data.clone(), level });
            Some(Layer::new(z, contours, layer_height).with_surface(surface))
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

    fn tilted_setup() -> (Mesh, TetMesh, TetScalarField) {
        let surface = box_mesh(20.0, 20.0, 10.0);
        let (tet_mesh, mut field) = grid_setup(&surface);
        // s = z + 0.2 x: level sets are planes tilted about Y.
        field.values = tet_mesh.vertices.iter().map(|v| v.z + 0.2 * v.x).collect();
        field.min_value = field.values.iter().cloned().fold(f64::INFINITY, f64::min);
        field.max_value = field.values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        (surface, tet_mesh, field)
    }

    /// Regression test for D-19. A layer must be able to say how high it is
    /// anywhere inside its outline, not just on it. For s = z + 0.2x the
    /// level set through a boundary point is the plane z = z0 - 0.2 (x - x0).
    #[test]
    fn test_layer_surface_gives_exact_interior_height() {
        let (surface, tet_mesh, field) = tilted_setup();
        let layers = extract_surface_layers(&surface, &tet_mesh, &field, 1.0);
        let layer = &layers[layers.len() / 2];
        let surf = layer.surface.as_ref().expect("tet layers must carry their surface");

        let p0 = layer.contours[0].points[0];
        let level = p0.z + 0.2 * p0.x;

        for &(x, y) in &[(10.0, 10.0), (3.0, 17.0), (16.5, 4.2), (10.0, 1.0)] {
            let expect = level - 0.2 * x;
            if expect < 0.0 || expect > 10.0 {
                continue; // this level does not pass over that point inside the box
            }
            // Deliberately bad starting guesses: the solve must not depend on them.
            for &guess in &[layer.z, 0.0, 10.0] {
                let z = surf.z_at(x, y, guess).expect("interior point should resolve");
                assert!(
                    (z - expect).abs() < 1e-6,
                    "z_at({}, {}) from guess {} = {:.9}, expected {:.9}",
                    x, y, guess, z, expect
                );
            }
        }
    }

    /// The point of D-19, end to end: everything the toolpath generator
    /// extrudes for a layer -- outer wall, inner wall loops, infill -- must lie
    /// on that layer. Before this, only the outermost wall did; inner loops sat
    /// at the layer's nominal Z and infill was interpolated from the boundary.
    #[test]
    fn test_walls_and_infill_lie_on_the_curved_layer() {
        let (surface, tet_mesh, field) = tilted_setup();
        let layers = extract_surface_layers(&surface, &tet_mesh, &field, 1.0);

        let generator = crate::toolpath::ToolpathGenerator::new(0.4, 1.0);
        let toolpaths = generator.generate(&layers, Some(&surface));
        assert_eq!(toolpaths.len(), layers.len());

        let mut checked = 0usize;
        for tp in &toolpaths {
            // On the layer, z + 0.2x is one constant. Measure how far the
            // extruded points stray from that.
            let s: Vec<f64> = tp
                .paths
                .iter()
                .filter(|seg| seg.extrusion > 0.0)
                .map(|seg| seg.position.z + 0.2 * seg.position.x)
                .collect();
            if s.len() < 10 {
                continue;
            }
            let lo = s.iter().cloned().fold(f64::INFINITY, f64::min);
            let hi = s.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            assert!(
                hi - lo < 1e-3,
                "extruded points stray {:.4} mm off their layer (z={:.2})",
                hi - lo,
                tp.z
            );
            checked += 1;
        }
        assert!(checked >= 5, "only {} layers had enough extrusion to check", checked);

        // Control: the same layers without their surface. If this did not
        // stray, the assertion above would be passing for some other reason.
        let stripped: Vec<Layer> = layers
            .iter()
            .map(|l| Layer::new(l.z, l.contours.clone(), l.layer_height))
            .collect();
        let worst_without = generator
            .generate(&stripped, Some(&surface))
            .iter()
            .map(|tp| {
                let s: Vec<f64> = tp
                    .paths
                    .iter()
                    .filter(|seg| seg.extrusion > 0.0)
                    .map(|seg| seg.position.z + 0.2 * seg.position.x)
                    .collect();
                if s.len() < 10 {
                    return 0.0;
                }
                s.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
                    - s.iter().cloned().fold(f64::INFINITY, f64::min)
            })
            .fold(0.0f64, f64::max);
        assert!(
            worst_without > 0.05,
            "without the layer surface the points strayed only {:.4} mm, so this \
             test is not exercising the defect",
            worst_without
        );
        eprintln!("stray without layer surface: {:.4} mm", worst_without);
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
    /// Refinement must shorten every edge without changing the surface or
    /// opening it: a T-junction would leave level sets unclosed.
    #[test]
    fn test_refine_surface_is_conforming() {
        // A long thin box exercises the one- and two-edge split patterns as
        // well as the uniform one.
        let surface = box_mesh(20.0, 3.0, 1.4);
        let area = |ts: &[Triangle]| -> f64 {
            ts.iter().map(|t| 0.5 * (t.v1 - t.v0).cross(&(t.v2 - t.v0)).norm()).sum()
        };
        let refined = refine_surface(&surface, 1.0);
        assert!(refined.len() > surface.triangles.len());
        assert!((area(&refined) - area(&surface.triangles)).abs() < 1e-9);

        let key = |p: &Point3D| ((p.x * 1e6).round() as i64, (p.y * 1e6).round() as i64, (p.z * 1e6).round() as i64);
        let mut edges: std::collections::HashMap<_, usize> = std::collections::HashMap::new();
        for t in &refined {
            for (a, b) in [(t.v0, t.v1), (t.v1, t.v2), (t.v2, t.v0)] {
                assert!((a - b).norm() <= 1.0 + 1e-9, "edge of {:.3} mm survived", (a - b).norm());
                let (ka, kb) = (key(&a), key(&b));
                *edges.entry(if ka < kb { (ka, kb) } else { (kb, ka) }).or_insert(0) += 1;
            }
        }
        assert!(
            edges.values().all(|&n| n == 2),
            "refined surface is not watertight: {} edges not shared by exactly two triangles",
            edges.values().filter(|&&n| n != 2).count()
        );
    }

    /// Worst disagreement between a layer's outline and the layer's own
    /// surface, in mm of height.
    fn worst_outline_error(layers: &[Layer]) -> f64 {
        let mut worst = 0.0f64;
        for layer in layers {
            let surf = layer.surface.as_ref().unwrap();
            for p in layer.contours.iter().flat_map(|c| c.points.iter()) {
                if let Some(z) = surf.z_at(p.x, p.y, p.z) {
                    worst = worst.max((z - p.z).abs());
                }
            }
        }
        worst
    }

    /// Regression test for D-21. An outline is traced through field values
    /// sampled at surface vertices, so on a model with large triangles it cut
    /// straight across a field that curves between them: the outline and the
    /// layer it outlines disagreed (0.88 mm on a Benchy), and the walls
    /// followed the outline. A box with two triangles per face and a bowed
    /// field is the smallest case.
    #[test]
    fn test_outlines_agree_with_their_layer_on_a_coarse_mesh() {
        let surface = box_mesh(20.0, 20.0, 10.0);
        let (tet_mesh, mut field) = grid_setup(&surface);
        // Level sets bow 2 mm between the middle of the box and its sides.
        field.values = tet_mesh.vertices.iter().map(|v| v.z + 0.02 * (v.x - 10.0).powi(2)).collect();
        field.min_value = field.values.iter().cloned().fold(f64::INFINITY, f64::min);
        field.max_value = field.values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let layers = extract_surface_layers(&surface, &tet_mesh, &field, 1.0);
        assert!(layers.len() >= 5);
        let worst = worst_outline_error(&layers);
        assert!(worst < 0.05, "outline is {:.4} mm off its own layer", worst);

        // Control: the same extraction on the unrefined surface.
        let coarse = extract_surface_layers_refined(&surface, &tet_mesh, &field, 1.0, f64::INFINITY);
        let worst_coarse = worst_outline_error(&coarse);
        assert!(
            worst_coarse > 0.5,
            "unrefined outline was only {:.4} mm off, so this test is not exercising the defect",
            worst_coarse
        );
        eprintln!("outline error: refined {:.4} mm, unrefined {:.4} mm", worst, worst_coarse);
    }
}