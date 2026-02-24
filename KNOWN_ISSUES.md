# Known Issues, Bugs, and Todo List

Last updated: 2026-02-23

---

## Priority legend
- **P1** — Blocks correct output / causes print failures
- **P2** — Significant quality degradation, workaround exists
- **P3** — Minor annoyance or cosmetic
- **Enhancement** — New capability, not a bug

---

## P1 — Blocks Correct Output

### [BUG] Example binaries don't compile
**Files:** `src/bin/simple_slicer.rs`, `src/bin/slice_benchy.rs`
**Symptom:** `cargo test` (without `--lib`) or `cargo build --examples` fails with "missing field `max_rotation_degrees`".
**Cause:** `S3PipelineConfig` gained a `max_rotation_degrees` field after the example files were written.
**Fix:** Add `max_rotation_degrees: 15.0` (and `z_bias: 0.8`) to the `S3PipelineConfig` struct literal in each example.
**Workaround:** Always use `cargo test --lib` or `cargo run --bin gui --release`.

### [BUG] Pre-existing test failures (3)
- `test_s3_pipeline` — single-triangle test mesh is too small for TetGen to tetrahedralise; the pipeline falls back and produces no layers.
- `test_global_unfold_rotation` — degenerate test geometry with co-planar faces produces a zero rotation that the test asserts is non-zero.
- `test_overhang_detection_vertical` — numerical tolerance: the angle of a perfectly-vertical face comes out as 89.999… degrees, not 90°, so the assertion `angle_deg > 90.0` fails.

None of these were introduced by recent work. All three need fresh test geometry or relaxed assertions.

---

## P2 — Quality / Correctness Issues

### [BUG] S4 — deep undercuts still require support on some models
**Symptom:** Even with the Support-Free Preset (z_bias 0.85, max_rotation 35°), overhangs beyond ~55–60° may not be fully corrected. Tets at the deepest undercut hit the `max_rotation_degrees` cap before the overhang angle reaches the threshold.
**Root cause:** `max_rotation_degrees` prevents tet inversion but also limits correction depth. The deformation is a rigid rotation, not a shear, so very deep undercuts cannot be fixed without also distorting the layer thickness significantly.
**Options:**
- Raise `max_rotation_degrees` further (risk: tet inversion → fallback to VirtualScalarField).
- Add a shear component to the per-tet deformation in addition to rotation.
- Combine S4 deformation with S3-style scalar-field correction for residual overhangs.

### [BUG] S4 — layer ordering after untransform is approximate
**Symptom:** After barycentric untransform, each layer's `z` is set to the average Z of all its untransformed points. For strongly-deformed models this can cause layers from different regions to interleave in Z order, leading to non-monotone toolpath sequences.
**Fix:** After untransform, sort all layers by their true mean Z, then re-verify contour support using the conical-style 2D grid filter (or at minimum sort and warn).

### [BUG] Conical — floating contour filter misses very thin isolated features
**Symptom:** Features with XY footprint < ~2% of total object footprint may fall into a single bin cell alongside the main body and be marked "supported" prematurely.
**Root cause:** The 64×64 grid resolution is too coarse for thin features relative to large objects.
**Fix:** Increase grid resolution to 128×128, or use a finer per-contour point test (sample all bin cells the contour's bounding box spans, not just one cell per point).

### [BUG] Conical — outward mode can still produce sub-bed extrusion on non-flat bottoms
**Symptom:** Models with curved or off-axis bottom surfaces can have a few points clamped to bed_z that later get slightly negative Z from rounding in downstream transforms.
**Fix:** Add a final `p.z = p.z.max(bed_z)` clamp at the very end of `conical_slice()`, after all inverse transforms and contour filtering.

### [BUG] MeshRayCaster — misses at mesh boundary produce incorrect Z
**Symptom:** Infill lines that extend slightly outside the mesh XY bounding box get no raycaster hit and fall back to IDW from nearby wall-loop points. On thin shell edges this can place the infill 1–2 mm above or below the surface.
**Fix:** Extend the raycaster's search radius to also try adjacent bins before falling back to IDW, or clip infill polygons to the mesh footprint (expensive).

### [BUG] Wall seam transitions — multi-island layers produce artifacts
**Symptom:** If a layer has two or more separate contour islands (e.g. a model that splits into two separate regions at a given height), `resample_to_n()` pairs points from different islands and generates a transition path that crosses empty space.
**Fix:** Only generate transitions when the current and previous layer each have exactly one outer contour, or implement a matching step that pairs islands by centroid proximity before resampling.

### [BUG] S3 VirtualScalarField fallback is silent
**Symptom:** If TetVolumetric ASAP deformation fails quality checks (>30% inverted tets or >5× bounding-box growth), the pipeline silently falls back to VirtualScalarField. The user sees slightly different layers with no warning.
**Fix:** Set a flag in `S3PipelineResult` (e.g. `used_fallback: bool`) and show a warning banner in the GUI stats panel.

### [BUG] Surface normal orientation near mesh boundary / thin edges
**Symptom:** The 48×48 XY bin grid lookup for surface normal orientations occasionally misses triangles near the mesh boundary or on very thin features. The expanding-ring search falls back to the last-found normal from a distant face, causing a few segments at the mesh edge to get an incorrect tilt angle.
**Root cause:** Grid resolution (48×48) is tuned for mid-range meshes. Very large or very elongated models map all triangles into a narrow band of cells, leaving the outer ring mostly empty.
**Fix:** Scale grid resolution to `sqrt(triangle_count).clamp(32, 128)` instead of the hardcoded 48, or fall back to a linear scan when no bin-neighbours are found within radius 4.

### [BUG] Travel Z-lift doesn't insert explicit lift segments
**Symptom:** The current travel-lift implementation raises the Z coordinate of the travel destination point but does not insert a separate "lift" move at the departure point. On machines that interpret linear G1 moves literally this means the Z rise happens simultaneously with the XY travel, which may cause a slight graze at the start of the travel.
**Fix:** Before each lifted travel segment, insert an explicit Z-only move to `last_extrude_z + lift`, then the XY travel, then the contact Z move at arrival.

---

## P3 — Minor / Cosmetic

### [BUG] Stray `nul` file in repository root
**File:** `c:/Users/danie/Documents/Non-planar slicer project/nul`
A zero-byte file named `nul` was created at the repo root (Windows artifact from a redirected command). Should be deleted and added to `.gitignore`.

### [BUG] Geodesic `CustomVectorField` — no multi-scale support
Multi-scale mode is silently ignored when `diffusion_mode = CustomVectorField`. The custom vector path bypasses the heat step and goes directly to the Poisson solve, so it cannot run at multiple timesteps.
**Fix:** Either disable the multi-scale checkbox in the GUI when CustomVectorField is selected, or document the limitation with a tooltip.

### [BUG] Branch-cut seam in cylindrical and spherical modes
**Symptom:** A vertical artifact line appears on the side of the object where θ = ±π. Triangles that straddle the branch cut get two vertices mapped to θ ≈ +π and one to θ ≈ −π (or vice versa), producing a degenerate triangle in deformed space.
**Fix:** Detect and split branch-cut triangles before slicing — re-sample the edge at the cut and insert a new vertex exactly at θ = ±π. Non-trivial but geometrically well-defined.

### [BUG] Cargo.toml author placeholder
`authors = ["Your Name <your.email@example.com>"]` — should be updated before publishing.

---

## Enhancements — Things to Work On

### [ENH] Fix example binaries (`simple_slicer.rs`, `slice_benchy.rs`)
Update both example files to compile cleanly with the current `S3PipelineConfig` struct. This unblocks `cargo test` without the `--lib` flag and enables benchmark runs.
**Effort:** Small — add two fields to each struct literal.

### [ENH] Verify and tune S4 support-free preset on real hardware
The Support-Free Preset values (z_bias 0.85, 35° overhang, 35° max rotation) were determined analytically. Real print testing on the Stanford Bunny will reveal whether the deformation is sufficient for specific problem areas (underside belly, inner ear concavity, tail). Likely needs per-region tuning or a higher `max_rotation_degrees`.

### [ENH] S4 — height-weighted interior tet rotation propagation
Currently interior tets (those with no surface boundary face) copy a damped average of their surface-tet neighbours. For tall thin features (e.g. bunny ears) the propagation sometimes misses interior tets far from any surface, leaving them at identity rotation and creating a "soft" un-deformed core.
**Fix:** Run the propagation for multiple passes (breadth-first from surface inward) rather than one pass from immediate neighbours only.

### [ENH] Progress indicator during voxel reconstruction
Voxel reconstruction takes 2–5 seconds with no visible feedback. The GUI shows the last slicing message but does not update during the repair step.
**Fix:** Break `voxel_remesh()` into stages (SDF, sign, MC) and send progress updates on the channel, or at minimum log a "Voxel reconstruction in progress..." message that appears in the stats panel.

### [ENH] ✅ Printer profiles with persistence — COMPLETE (2026-02-23)
Named printer profiles now persist across sessions via eframe key-value storage. Each profile stores nozzle geometry, TCP offset, axis limits, bed/head dimensions, and optional STL overrides. Profiles load and apply automatically on startup.

### [ENH] ✅ Machine simulation in viewport — COMPLETE (2026-02-23)
Bed and printhead are now rendered in the 3D viewport as parametric boxes/cylinders or custom STL files. Geometry updates live as the toolpath playback scrubber moves. Nozzle tip is marked with a gold sphere. Viewport renders even before a mesh is loaded.

### [ENH] ✅ Surface normal orientations for all slicing modes — COMPLETE (2026-02-23)
A 48×48 XY bin grid assigns each toolpath segment's rotary-axis direction from the nearest mesh face normal. Axis tilt is clamped to the profile's configured limits. Conical mode uses analytical cone-surface normals.

### [ENH] ✅ Travel Z-lift — COMPLETE (2026-02-23)
Travel moves longer than 1 mm are raised above the last extrusion Z by a configurable clearance (default 2 mm). Exposed in the "Rotary Axes & Collision Avoidance" collapsible in the control panel.

### [ENH] Machine profile presets
The Machine Profile section currently uses individual named profiles saved to disk. Add a dropdown of built-in presets for common 5-axis configurations (e.g. "Generic AB head-tilt", "Rotary table BC", "Robotic arm 6-axis") that fills in nozzle radius, TCP offset, and axis mode automatically.

### [ENH] Geodesic — multi-scale for CustomVectorField
Run the Poisson solve twice at different regularisation strengths and fuse the results, analogous to what multi-scale does for the heat step. This would give CustomVectorField the same fine-detail / full-coverage blend as other modes.

### [ENH] S3 — expose fallback warning in GUI
When VirtualScalarField fallback is triggered, show a yellow warning banner in the stats panel: "Deformation quality too low — fell back to VirtualScalarField". Currently only logged to console.
**Note:** Stats panel now shows collision count (red text) when collision segments are present.

### [ENH] Adaptive layer height — expose min/max height in GUI
`min_layer_height` and `max_layer_height` are currently hardcoded constants in `slicing.rs`. Expose them as sliders in the control panel so users can tune the range without recompiling.

### [ENH] Conical — per-region deferred contour timeout
The current floating-contour filter defers unsupported contours indefinitely and only appends them at the very end sorted by min_z. For pathological geometries (e.g. a floating island that is never supported) this means those contours are printed last, which may cause a poor final surface. Add a max-defer count and emit a warning if a contour is deferred more than N times.

### [ENH] Support generation — full pipeline integration
`support_toolpath.rs` generates toolpaths for supports but the GUI does not yet have a "Generate Supports" → "Slice with Supports" flow for non-planar modes. Supports are only generated for planar slicing. Extend to work with S4 and Conical modes.

### [ENH] Toolpath — seam placement control
The wall loop seam (where the outer perimeter starts and ends) is currently placed at an arbitrary point on the contour (wherever `contour_offset` starts the offset). Add a seam placement option: back of model, minimum-visibility angle, or user-specified direction.

### [ENH] GPU-accelerated CG solver
The Conjugate Gradient solver in `geodesic.rs` is the primary bottleneck for large meshes (> 200K vertices: 10–30s per solve). A WGPU compute shader implementation would reduce this to < 1s. The matrix is sparse symmetric positive definite — ideal for GPU CG.

### [ENH] Export — full 5-axis G-code validation
The current G-code exporter outputs A/B (or B/C) rotation angles but does not verify the motion is kinematically feasible for a specific machine. Add a post-processing pass that checks joint angle limits and interpolates through singularities.

---

## Architecture Notes for Future Work

- **`collision.rs` (root)** and **`singularity.rs` (root)** are legacy stubs with TODO comments. The real implementations are in `motion_planning/`. The stubs should be removed or re-exported from the motion_planning module.
- **`isotropic_remesh.rs`** is no longer on any active code path (voxel reconstruction replaced it). It can be kept for reference or deleted to reduce codebase size.
- **`deformation.rs`** and **`deformation_v2.rs`** — two versions of the surface ASAP solver exist. `deformation_v2.rs` is the active one; `deformation.rs` is legacy. Consider removing `deformation.rs`.
