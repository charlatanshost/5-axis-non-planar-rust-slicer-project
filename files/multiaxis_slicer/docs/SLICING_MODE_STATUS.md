# Slicing mode status

What each slicing mode actually does as of commit `51c0195` (November 2026), what
was repaired to get there, and what is still broken. Every number below was
measured, not estimated; the method is at the end so results can be reproduced.

Two reference models are used throughout:

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
| **TetVolumetric** | Runs end to end; deformation is modest | Yes, for non-planar work |
| S4 Deform | Runs, but structurally limited to 1–2 rotation axes | Only for single-direction overhangs |
| Conical | Correct, but a single global cone | Only for radially symmetric parts |
| Geodesic | **Broken** — Poisson solve does not converge | No |
| CoordTransform Cylindrical / Spherical | Branch-cut artifact at θ = ±π | Niche |

---

## TetVolumetric

The volumetric S3 pipeline: tetrahedralize → per-tet quaternion field →
volumetric ASAP deformation → volume scalar field → marching tetrahedra. This
is the only mode in the tree that can produce **fillable solid cross-sections
with per-feature orientation**, which is what simultaneous non-planar printing
needs.

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

### Current behaviour

| | sphere | 3dbenchy |
|---|---|---|
| tetrahedralizer | TetGen direct, 8701 tets | grid, 271 492 tets |
| tets carrying rotation | 2017 (23.2 %) | 73 784 (27.2 %) |
| max rotation | 14.9° (hits the 15° cap) | 14.2° |
| peak ASAP displacement | 0.237 mm | 0.187 mm |
| inverted tets | 0 | 0 |
| extent ratio | 1.00× | 1.00× |
| layers | 57 | 239 |
| per-layer Z spread (median) | 1.558 mm | 0.888 mm |
| wall-clock | 0.7 s | 27 s |

### Known limitations

- **Deformation is modest.** 0.187 mm of peak displacement on a 48 mm model is
  real (7× what it was) but small. Four dampers remain in series:
  `max_rotation_degrees` defaults to 15, `quaternion_weight` to 0.3,
  `asap_max_iterations` to 3, and the diffusion itself decays with depth.
  Iteration count now genuinely matters — 50 iterations rotate 46.6 % of tets,
  400 rotate 95.7 % — so these are worth tuning before assuming an algorithmic
  limit.
- **Layers extend outside the model.** On the Benchy they span Z −0.28 to 48.38
  on a 0..48 model, i.e. 0.28 mm below the build plate. This is a consequence of
  a deliberate design choice: `from_surface_mesh_grid` keeps any cube with at
  least one inside vertex so the tet mesh *encloses* every surface vertex, which
  `deform_surface_through_tets` requires for its barycentric lookup. The
  enclosure is load-bearing — **do not shrink the grid.** The clipping belongs
  in layer extraction.
- **Grid boundary is a staircase** at the cell size (0.75 mm on a Benchy),
  because the grid tetrahedralizer is axis-aligned. `extract_tet_layers` takes
  contours off the tet mesh surface, so this shows up in the layers. TetGen's
  output is boundary-conforming and much better, which is why the direct attempt
  is still tried first — but TetGen rejects anything over 80 000 vertices /
  160 000 facets, and a Benchy is 112 569, so real models land on the grid.

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

1. **Tune the TetVolumetric dampers.** `max_rotation_degrees`,
   `quaternion_weight`, `asap_max_iterations`. Iteration count now has real
   effect; the others are untested above their defaults.
2. **Clip extracted layers to the model.** Contours currently fall below the
   build plate. Must be done in layer extraction, *not* by shrinking the grid.
3. **Geodesic Poisson convergence.** Next suspect is the Jacobi preconditioner
   versus the null-space projection.
4. **Grid boundary fidelity.** Either snap boundary tets to the surface, or
   raise the grid resolution and accept the tet-count cost.
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

- **Is a layer genuinely non-planar?** Per-layer internal Z spread, i.e.
  `max(p.z) − min(p.z)` over each layer's contour points. Compare it against the
  grid cell size — if the median sits *at* the cell size, you are measuring the
  staircase, not deformation.
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
