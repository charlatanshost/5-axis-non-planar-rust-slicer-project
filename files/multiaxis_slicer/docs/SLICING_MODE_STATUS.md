# Slicing mode status

What each slicing mode actually does as of commit `2b0e972` (November 2026), what
was repaired to get there, and what is still broken. Every number below was
measured, not estimated; the method is at the end so results can be reproduced.

Reference models used throughout (a third, `StanfordBunny_fixed.stl`, 80 281
triangles and 83 mm tall, joined them for the later end-to-end measurements):

- **sphere** — a UV sphere, radius 15 mm, 24 × 32 tessellation, 1472 triangles,
  generated analytically and verified watertight (2208 edges, 0 boundary,
  0 non-manifold).
- **3dbenchy** — `files/3dbenchy.stl`, 225 706 triangles, 48 mm tall. Verified
  watertight: 112 569 vertices, 225 154 faces, 337 731 edges all shared by
  exactly two faces, Euler characteristic −8 (closed, genus 5).

---

## Which mode to use

| Mode | State | Use it? |
|---|---|---|
| **Planar** | No known correctness defect | Yes — the dependable baseline |
| **TetVolumetric** | Runs end to end to sane G-code; walls correct, infill still planar | Yes, for non-planar work |
| S4 Deform | Runs, but structurally limited to 1–2 rotation axes | Only for single-direction overhangs |
| Conical | Correct, but a single global cone | Only for radially symmetric parts |
| Geodesic | **Broken** — Poisson solve does not converge | No |
| CoordTransform Cylindrical / Spherical | Branch-cut artifact at θ = ±π | Niche |

---

## TetVolumetric

The volumetric S3 pipeline: tetrahedralize → per-tet quaternion field →
volumetric ASAP deformation → volume scalar field → level sets of that field on
the model surface. This is the only mode in the tree that can produce
**fillable solid cross-sections with per-feature orientation**, which is what
simultaneous non-planar printing needs.

It did not work at all before this batch. Three independent defects each
prevented it from completing:

1. **Vertex dedup destroyed the mesh** (`afeef21`). `hash_point` folded the
   quantised coordinates into a `u64` and used that *hash* as the vertex
   identity key, so collisions welded distinct vertices together. On the sphere
   this merged 17.1 % of all vertices (738 distinct positions → 612 keys),
   producing degenerate and self-intersecting facets. TetGen correctly refused
   and returned 0 tetrahedra — then reported *"mesh may have self-intersecting
   facets"*, blaming the model for corruption introduced in our own dedup.
   That misleading message is why the voxel reconstruction, isotropic remeshing,
   vertex clustering and convex-hull fallbacks were written.
2. **The ASAP global solve asked for an inverted mesh** (`b84f228`). The
   Laplacian rows encode `w·(pₐ − p_b)` but the right-hand side was built from
   `vertices[b] − vertices[a]` and added with a plus, so the system demanded the
   *negation* of every edge. Even an identity transform asked the solver to turn
   the mesh inside out. Separately, `solve_gauss_seidel` ignored its starting
   point in favour of `rhs * 0.01`, which approximates nothing — for this
   Laplacian the right-hand side is near zero at interior vertices because the
   surrounding edges cancel — so every solve began with the mesh collapsed onto
   the origin and 30 sweeps could not climb out.
3. **The TetGen-free tetrahedralizer was unreachable** (`3b781a8`). TetGen does
   not always fail by returning an error: on marching-cubes output it trips an
   assertion in its C++ (`tetgen.cxx:15273`, `ivf.iloc == NEARVERTEX`) and calls
   `abort()`. No Rust error handling can catch that — it takes the process down,
   GUI included, from a worker thread. `from_surface_mesh_grid` works and cannot
   meaningfully fail, but sat fifth in the cascade behind up to eleven TetGen
   invocations, so nothing large enough to need it ever got there.

With those fixed, the rotation field turned out to be throttled too
(`693f573`): the relaxation recomputed each tet's objective from its own normal
every pass, and since `compute_tet_representative_normal` gives interior tets a
synthetic *upward* normal that no objective rotates, identity was re-injected
into the interior on every iteration. The field decayed exponentially from the
surface. **400 iterations produced bit-identical output to 50**, which proves
the decay was the fixed point rather than under-iteration. Recasting it as a
diffusion with boundary conditions — overhang tets hold their objective, the
bottom 5 % pins to identity, everything else takes the neighbour average with
no identity pull — made the field reach the full depth of the mesh.

Mean rotation angle by adjacency depth, on the sphere (13 hops deep):

| | d0 | d1 | d2 | d3 | d4 | d5 | d6 | d7 | reach |
|---|---|---|---|---|---|---|---|---|---|
| before | 2.14° | 0.44 | 0.10 | 0.02 | 0.00 | — | — | — | 5/13 |
| after | 1.48° | 1.36 | 1.16 | 0.94 | 0.76 | 0.69 | 0.51 | 0.38 | **13/13** |

### From "runs" to printable output

The pipeline completing was not the same as its output being usable. Driving the
real GUI code path headlessly from slice to G-code exposed a second set of
defects, each fixed in turn:

- **Tool tilt was bounded by axis travel, not by the head** (`ebb11fb`). The
  default profile lets A and B travel ±180°, and four passes derived their tilt
  budget from that, so G-code contained tilts up to 177.98° — nozzle pointing
  into the bed. `PrinterProfile::max_tool_tilt_deg` (default 45) now bounds it.
- **The collision pass was serial** (`95317d6`) and never finished on a Benchy.
  Parallelised; the BVH is read-only.
- **The deformation was unfinished, not limited** (`080d1c6`). Of four damping
  parameters only `asap_max_iterations` binds; at 3 the ASAP alternation had not
  converged. Raised to 20, the knee before folding sets in (0.9 % inverted tets
  on the sphere at 22 iterations, 3.6 % at 60).
- **Collision tested against unprinted geometry** (`02a3975`). It now queries a
  ladder of colliders banded by height, so only material at or below the
  nozzle counts.
- **Orientation came from the nearest model triangle** (`9d442cf`) rather than
  from the layer being printed. At the base of a wall that is the wall, so a
  flat first layer was tilted to the limit: a Benchy's first extrusion came out
  at 44.9°. Layer normals are now the scalar-field gradient, computed per tet
  and carried on `Contour::normals`. First extrusion: 0.9°.
- **Layers were cut from the wrong surface** (`419842d`). Marching tetrahedra
  emitted every edge of every isosurface polygon, interior ones included, so a
  layer was the wireframe of the isosurface and interior mesh edges were printed
  as walls. Its outline was the tet mesh boundary — the grid's 0.75 mm staircase
  — and overshot the model because the tet mesh does. `surface_layers.rs` now
  samples the field at the model's surface vertices and takes level sets on the
  model's own triangles.
- **Only the outline lay on the layer** (`607bd80`). Inner wall loops sat at the
  layer's nominal Z and infill heights were interpolated inward from the
  boundary. A layer now carries its surface (`LayerSurface::z_at`); for the tet
  pipeline that solves `field(x, y, z) = level` along the vertical, and the
  toolpath generator uses it for every wall loop and infill point.
- **Holes were treated as solid islands** (`66898e2`): wall loops offset into
  the hole and infill laid across it. Nesting depth now classifies contours,
  windings are normalised to what the offsetter expects, and infill is cut at
  each hole's outermost wall.
- **Outlines were traced through a field sampled only at surface vertices**
  (`2b0e972`), so on a model with large triangles the outline cut across a field
  that curves between them. On the Benchy (hull triangles to 18 mm) 28.9 % of
  outline points were more than 0.05 mm off their own layer, worst 0.88 mm. The
  surface is now refined to about one tet per edge (1 mm) before tracing.

Extruded points more than 0.05 mm from the layer they belong to, across those
three commits:

| | 3dbenchy | Stanford bunny |
|---|---|---|
| before `607bd80` | 45.6 % | 60.5 % |
| with layer surfaces and holes | 5.8 % | 0.1 % |
| with surface refinement | 0.3 % (82 of 23 978, max 0.78 mm) | 0.1 % (137 of 137 393, max 1.19 mm) |

The remainder is not diagnosed. On the bunny it shows up as short runs of
consecutive infill points 0.05-0.1 mm off, which suggests a level set with more
than one height above a point, but that has not been checked.

### Current behaviour

End to end, slice through G-code, layer height 0.5 mm, default profile:

| | 3dbenchy | Stanford bunny |
|---|---|---|
| tetrahedralizer | grid, 271 492 tets | grid |
| layers | 94 | 163 |
| layer contours | 316, all closed | 198, all closed |
| layer Z range | 0.477–48.000 (model 0–48) | 1.307–83.534 (model 0.88–83.75) |
| extruding moves | 23 978 | 137 393 |
| max tool tilt | 43.57° | 45.00° (at the budget) |
| A / B range | −40.4…43.0° / −25.2…35.9° | −42.4…42.3° / −33.6…40.7° |
| non-finite, negative-E or below-model moves | 0 | 0 |

Tilt, A/B range and move sanity were last measured at `419842d`, before the
three commits above; timings have not been re-measured since and will have
grown, since every wall and infill point now solves for its height.

Toolpath Z reads about 2 mm above each model. That is travel z-hop; extruding
moves alone span 0.539–47.962 and 1.407–83.350.

How non-planar the layers are was last measured before the switch to surface
extraction: on the Benchy, median per-layer Z spread 1.234 mm with 0 inverted
tets at 20 ASAP iterations. It has not been re-measured on the new contours.

### Known limitations

- **Infill lies on the curved layer but is laid out in XY.** Scan lines are
  generated in the plane and lifted onto the layer surface, so line spacing is
  correct seen from above and stretches on steep parts of a layer. Extrusion
  amounts do not yet account for that.
- **A layer is assumed to have one height above each (x, y).** Where a level set
  folds over itself the solver returns the root nearest the boundary's height.
- **The tet mesh encloses the model on purpose.** `from_surface_mesh_grid` keeps
  any cube with at least one inside vertex, so every surface vertex has a
  containing tet. Both `deform_surface_through_tets` and `surface_layers.rs`
  depend on that. **Do not shrink the grid.**
- **The grid's resolution still bounds the field**, though no longer the
  outline: the scalar field is linear per tet, so features of the deformation
  finer than a 0.75 mm cell are not represented. TetGen gives a better mesh and
  is still tried first, but rejects anything over 80 000 vertices / 160 000
  facets, so real models land on the grid.
- **The quality gate is too permissive.** A deformation is rejected only at
  30 % inverted tets or 5× extent; 3.6 % already means visible folding.
- **Kinematics are partly configurable.** Axis order, rotation axis and sign
  convention are (`4ffb29c`), with a round-trip check on the profile page. Joint
  offsets beyond a single scalar TCP offset, and bed-side rotary axes, are not;
  see `KINEMATICS_DESIGN_NOTE.md`.

---

## Geodesic — do not use

The heat method's Poisson step solves a **singular** system: with natural
boundary conditions the cotangent Laplacian annihilates the constant vector,
because φ is only determined up to an additive constant. The code tried to
remove that null space by adding `1e-6` to one diagonal entry. Cotangent weights
here are O(1), so the perturbation was about six orders of magnitude too small
to pin anything; the system stayed singular to working precision and CG wandered
along the null direction.

Worse, `conjugate_gradient` returned that non-solution **silently**, so nothing
downstream could distinguish a converged solve from a failed one.

Measured on the Benchy with GUI defaults, before `af6622a`:

- Poisson relative residual never fell below ~0.3, reaching 7.3, against a
  1e-8 tolerance — at every one of the six multi-scale timesteps.
- Maximum geodesic distance came back as **11.8 mm on a 48 mm model**. Geodesic
  distance is bounded below by Euclidean distance, so that is impossible by
  inspection.
- Output was **not reproducible**: two identical runs gave max distances of
  11.78 mm and 79.93 mm, and layer counts of 18 441 and 13 508. Which vertex
  lands at index 0 depends on vertex-welding order, and the pin was the only
  thing breaking the symmetry.
- 18 412 of 18 441 contours came out unsupported, because level sets of a field
  that is not a distance function fragment into thousands of disconnected loops.

`af6622a` made the solvers report failure, added a guard that flags
`max_dist < z_range` as the impossibility it is, replaced the pin with a
consistent right-hand side plus null-space deflation, and raised the heat budget
to 20 000 iterations. **The heat solve now converges; the Poisson solve still
does not** — its residual plateaus near 0.4. Geodesic mode remains unusable and
now says so loudly instead of emitting plausible-looking garbage.

Ruled out as the cause of the remaining stall: extra null-space dimensions from
disconnected geometry. The welded Benchy topology is a single component, closed
and manifold, so the null space is exactly `span{1}`. The next suspect is the
Jacobi preconditioner, which does not commute with the projection and so breaks
the Krylov orthogonality deflation relies on; a residual *plateau* rather than
slow descent fits that. Not investigated further.

**Separately, and independent of the bug:** geodesic level sets are curves on
the *surface*. This mode can only ever produce shells, never fillable solid
cross-sections. It is a surface-finishing technique, not a route to simultaneous
non-planar slicing of solids. `force_nonplanar_infill` exists as an override
because of this.

---

## Conical — correct, but single-axis by construction

`conical_forward` is the whole transform: `z += sign · r · tan(angle)`, where
`r` is the distance from **one** apex at `(cx, cy)` with **one** constant angle.
Every layer is a nested cone about a single vertical axis.

This is a method ceiling, not a defect. An overhang on one side of a hull and
one on the opposite side get radial slopes pointing in opposite directions — one
is helped, the other made worse — and a flat deck gets the same 45° as a hull
flare. Per-feature cones cannot be expressed. RotBot's conical slicing targets
roughly radially symmetric parts: a central spire, a funnel, a bowl.

---

## S4 Deform — limited to one or two rotation axes

Three defaults each independently collapse it toward a global bend:

- `s4_multi_axis: false` → `normalize_single_axis()` forces *every* rotation in
  the model onto one axis, that of the single most severe overhang tet. The code
  says so: *"This guarantees stable SLERP averaging at the cost of
  multi-directional accuracy."* Setting it true allows up to **two** axis
  clusters.
- `s4_use_asap: false` → `deform_tet_mesh_direct` rotates every vertex about one
  global anchor at the base-plane bbox centre. One axis plus one anchor is a
  global bend.
- `s4_max_rotation_degrees: 15.0` caps the result regardless.

The single-axis constraint exists *because* of the direct deformation — averaging
conflicting axes at shared vertices gives degenerate rotations and mesh
collapse. ASAP is what solves that, so `use_asap: false` is what forces the
crutch. For multi-directional overhangs, use TetVolumetric.

Also note `S4RotationConfig.z_bias` is never read anywhere in
`s4_rotation_field.rs` — a dead field that looks live. `pipeline.rs:763` passes
the raw slider value into it while step 2 used the auto-computed
`effective_z_bias`, an inconsistency that is currently harmless only because the
field is unused.

---

## Outstanding

Ordered roughly by value.

1. **Re-measure the end-to-end output** (tilt, A/B range, timings, layer
   non-planarity) after `607bd80`…`2b0e972`, and chase the last 0.1–0.3 % of
   off-layer points.
2. **Tighten the deformation quality gate** from 30 % inverted tets to something
   that reflects when folding becomes visible.
3. **Kinematics: joint offsets and the part branch**, per the design note. Needed
   before G-code is trusted on a real tilting head with two pivots.
4. **Geodesic Poisson convergence.** Next suspect is the Jacobi preconditioner
   versus the null-space projection. Low value: the mode only makes shells.
5. **`hash_point` in three more modules.** `deformation_v2.rs`,
   `quaternion_field.rs` and `scalar_field.rs` use a different variant,
   `(x<<42) ^ (y<<21) ^ z`. Also a hash used as an identity key and so unsound by
   construction, but measured loss is **zero** on both reference models, so this
   is hardening rather than a live defect. Note those modules pass `u64` around
   as values too (`HashMap<u64, Vec<u64>>`, `vertex_hash: u64`), so it is a wider
   change than the two fixed in `af93e92`.
6. **Support infill still double-counts the connector.** Fixed for the extrusion
   contract in `f80990a`, but the first scan line of every column is degenerate:
   the sweep starts at `y = center.y − radius` where `dy == radius` gives
   `dx == 0`, emitting a zero-length line.
7. **`scalar_field.rs`'s `hash_point`** documents its 1e-3 quantisation as
   "1 mm precision". It is micron precision; the comment is wrong.
8. **Motion planning is scaffolding.** `MotionPlan.path` is never consumed and
   `get_joint_angles()` returns zeros. `004ff15` made the GUI report what the
   planner actually achieved rather than claiming success, but the module still
   does not influence G-code.
9. **CI lint job is non-blocking.** ~25 unused variables remain, several of them
   unused *function parameters* in `toolpath_patterns.rs` and `marching_tet.rs`
   where an underscore would paper over a signature that ignores an argument.
   Plus rustfmt (~15k diff lines) and the remaining clippy warnings.

---

## Reproducing the measurements

All figures came from throwaway `examples/` probes, deliberately not committed —
they hardcode `../3dbenchy.stl`, which is not in the repository. To rebuild one:

```rust
// examples/probe.rs
let mesh = Mesh::from_stl("../3dbenchy.stl")?;
let config = S3PipelineConfig {
    deformation_method: DeformationMethod::TetVolumetric,
    layer_height: 0.2,
    ..S3PipelineConfig::default()
};
let result = execute_s3_pipeline(mesh, config);
```

Run with `cargo run --release --example probe`. The release profile matters:
`debug` takes minutes on a Benchy.

Useful things to measure:

- **Is the G-code sane, not just the slice?** `SlicerApp::default()` needs no
  egui context, so a probe can set `app.mesh` and `app.layers` and call
  `generate_toolpaths()` and `generate_gcode_from_toolpaths()` — the real code
  path. Check tilt against the budget, A/B ranges, non-finite values, and the Z
  of *extruding* moves separately from travels. This is what found the 177°
  tilts; nothing upstream of G-code showed them.
- **Is a layer genuinely non-planar?** Per-layer internal Z spread, i.e.
  `max(p.z) − min(p.z)` over each layer's contour points.
- **Did the pipeline silently fall back?** Watch for `Falling back to
  VirtualScalarField` in the log. Flat layers and "mode produced output" look
  identical in the GUI otherwise.
- **Is a mesh watertight?** Count half-edges keyed on quantised endpoint pairs;
  every edge in a closed manifold appears exactly twice. Do this before blaming
  a tetrahedralizer — and before trusting a procedurally generated test mesh. An
  earlier version of the sphere probe above silently omitted both pole fans.
- **Is a distance field really a distance field?** Geodesic distance is bounded
  below by Euclidean distance, so `max_dist` must be at least the model's Z
  extent. That single check is what exposed the Poisson failure.
