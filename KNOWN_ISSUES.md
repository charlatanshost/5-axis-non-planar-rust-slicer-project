# Known Issues, Bugs, and Todo List

Last updated: 2026-03-15

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
**Cause:** `S3PipelineConfig` gained several fields after the example files were written.
**Fix:** Add `max_rotation_degrees: 15.0`, `z_bias: 0.8`, and any other missing fields to the `S3PipelineConfig` struct literal in each example.
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

### [BUG] S4 — misplaced contour fragments / gaps in some layers
**Symptom:** The viewport shows floating path fragments or incomplete loops on some layers, most visibly near the neck/transition regions of complex organic models (e.g. Stanford Bunny).
**Root cause (partial fix applied 2026-03-15):**
- Points that fall just outside the tet mesh surface are mapped via `find_nearest_tet()` with clamped barycentric coordinates. When the clamped result lands far from the correct location it produces teleporting path segments.
- Contours were previously split at 20× layer height (too lenient); now split at 10×.
- Out-of-bounds points now filtered by AABB ± 5× layer height instead of the original loose sphere (1.5× diagonal from center).
- Closed contour flag now preserved when the contour survives untransform intact.
**Remaining issue:** The nearest-tet clamped coords are inherently inaccurate for points far outside the mesh. Better fix: walk from the nearest tet face toward the query point rather than clamping bary coords.

### [BUG] S4 — layer ordering after untransform is approximate
**Symptom:** After barycentric untransform, each layer's `z` is set to the minimum Z of all its untransformed points. For strongly-deformed models this can cause layers from different regions to interleave in Z order, leading to non-monotone toolpath sequences.
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

### [BUG] Travel Z-lift — no explicit Z-only departure move
**Symptom:** The travel-lift implementation raises the Z of the travel destination and inserts a non-extruding descend segment before the next extrusion, but does not insert a Z-only departure move at the start of travel. On machines that execute linear G1 moves literally, the Z rise happens simultaneously with XY travel, which may cause a slight graze at the start of the travel.
**Fix:** Before the lifted travel segment, insert an explicit Z-only `G1 Z{lift}` move, then the XY travel, then the descend segment at arrival. The departure lift is currently missing.

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

### [BUG] Machine simulation — collision count over-reports on heavily tilted bed
**Symptom:** When the bed is tilted to large angles (> 30°) and `show_machine` is enabled, the collision segment count can be higher than expected because the AABB-of-OBB for the head is a conservative overestimate (allows false positives but not false negatives).
**Root cause:** Collision is checked in the bed's local frame using the head's OBB half-extents expanded by the AABB-of-OBB formula, which is conservative. Segments where the head is near but not actually intersecting the bed may be flagged.
**Fix:** Upgrade to a true OBB-OBB separating-axis test (15 axes) instead of AABB-of-OBB.

---

## Recently Fixed (2026-03-15)

### ✅ [FIXED] Machine simulation — 180° bed flip at start of playback
**Was:** `machine_smooth_init` was never reset when toolpaths were regenerated or playback was rewound, causing the bed to flip 180° at the start.
**Fix:** Added `machine_prev_pos` field; when playback position decreases (rewind), `machine_smooth_init` resets to `false` so the machine snaps to the correct orientation.

### ✅ [FIXED] Machine simulation — head/bed assembly rendered upside down
**Was:** The head pivot formula used `±tcp_offset × orientation` which does not apply the head rotation. The nozzle tip in local space is at `(0,0,-nl)`, so the pivot formula must be `nozzle_world + R_head × (0,0,nl)`.
**Fix:** `pivot_head = nozzle_world + machine_apply_rot(rh, Vec3::new(0, 0, nl))` — uses the actual head rotation matrix.

### ✅ [FIXED] Z-hop travel — extrusion on descend from lift
**Was:** `apply_travel_lifts` raised the Z of travel destinations but didn't insert a descent segment, so the first extrusion after travel was extruding from the lifted height downward to the print surface.
**Fix:** A non-extruding clone of the first post-travel extrusion segment is inserted immediately before it, giving the nozzle a non-extruding descent to contact height.

### ✅ [FIXED] Machine simulation collision — almost all segments reported as colliding
**Was (round 1):** `compute_all_collisions` used world-AABB for the tilted bed, producing ±74mm Z extent at 45° for a 200mm bed — flagging any head within 114mm Z of bed center.
**Fix (round 1):** Check in bed's local frame using the head's OBB in bed-relative coordinates, preserving the bed's actual 10mm Z extent.
**Was (round 2):** Bed world pivot Z was static (`grid_z − bed_height`) for all segments; the viewport places the bed progressively lower as the print height increases via `bed_travel[2]`.
**Fix (round 2):** `bed_pivot_z` is now computed per-segment: `grid_z − bed_height − (seg_z − grid_z).clamp(0, bed_travel_z)` — exactly matching the viewport's bed Z travel logic.

### ✅ [FIXED] Mesh collision avoidance — forces 90° tilt on bottom-layer segments
**Was:** `optimize_toolpath_orientations_for_mesh` set `best_orient = trial` inside the sweep loop *before* the collision check, so when no clear orientation was found (all steps still collide — typical at the bottom of a print where the capsule is inside the mesh) `best_orient` held the last-tried max-tilt orientation. The condition `found_clear || best_orient != seg.orientation` then committed that max-tilt, forcing the nozzle horizontal at every bottom segment.
**Fix:** `best_orient` is only updated when the check **passes**. Commit condition changed to `if found_clear` only. If no tilt angle clears the mesh, the original slicing-direction orientation is preserved.

### ✅ [FIXED] Support generation — not wired into toolpath flow
**Was:** Support generation was only accessible via separate manual buttons ("Generate Supports" → "Generate Support Toolpaths") in a collapsible section. The main "Generate Toolpaths" button ignored supports entirely.
**Fix:** Added `enable_supports: bool` field. When checked, `generate_toolpaths()` automatically runs `generate_supports()` + `generate_support_toolpaths()` and merges the support toolpaths into `self.toolpaths`. The UI collapsible now shows only configuration controls plus a status line.

### ✅ [FIXED] S4 untransform — teleporting path fragments and loose bounds check
**Was:** The bounds check used a sphere of radius 1.5× diagonal from mesh center (too loose — allowed almost anything). Jump-split threshold was 20× layer height (too lenient). Closed-loop flag was always `false` even for intact loops.
**Fix:** AABB filter ± 5× layer_height (tight, rejects genuine teleports). Jump threshold reduced to 10× layer_height. `closed` flag is now preserved when a contour is untransformed without splitting.

---

## Enhancements — Things to Work On

### [ENH] Fix example binaries (`simple_slicer.rs`, `slice_benchy.rs`)
Update both example files to compile cleanly with the current `S3PipelineConfig` struct. This unblocks `cargo test` without the `--lib` flag and enables benchmark runs.
**Effort:** Small — add a few missing fields to each struct literal.

### [ENH] Verify and tune S4 support-free preset on real hardware
The Support-Free Preset values (z_bias 0.85, 35° overhang, 35° max rotation) were determined analytically. Real print testing on the Stanford Bunny will reveal whether the deformation is sufficient for specific problem areas (underside belly, inner ear concavity, tail). Likely needs per-region tuning or a higher `max_rotation_degrees`.

### [ENH] S4 — better nearest-tet fallback for untransform
Current `find_nearest_tet()` clamps negative barycentric coordinates, which can place out-of-mesh points on the face of the nearest tet (potentially wrong side of the mesh). Better approach: project the query point onto the nearest tet's closest face and use proper face bary coords, or use a gradient-descent walk from the nearest tet toward the query point.

### [ENH] S4 — height-weighted interior tet rotation propagation
Currently interior tets copy a damped average of their surface-tet neighbours. For tall thin features (e.g. bunny ears) the propagation sometimes misses interior tets far from any surface, leaving them at identity rotation and creating a "soft" un-deformed core.
**Fix:** Run the propagation for multiple passes (breadth-first from surface inward) rather than one pass from immediate neighbours only.

### [ENH] Progress indicator during voxel reconstruction
Voxel reconstruction takes 2–5 seconds with no visible feedback. The GUI shows the last slicing message but does not update during the repair step.
**Fix:** Break `voxel_remesh()` into stages (SDF, sign, MC) and send progress updates on the channel, or at minimum log a "Voxel reconstruction in progress..." message that appears in the stats panel.

### [ENH] ✅ Printer profiles with persistence — COMPLETE (2026-02-23)
Named printer profiles now persist across sessions via eframe key-value storage.

### [ENH] ✅ Machine simulation in viewport — COMPLETE (2026-02-23)
Bed and printhead rendered in 3D viewport with kinematic transforms. Corrected: SLERP flip on rewind, upside-down assembly, per-segment bed Z travel in collision detection.

### [ENH] ✅ Surface normal orientations for all slicing modes — COMPLETE (2026-02-23)

### [ENH] ✅ Travel Z-lift — COMPLETE (2026-02-23, descent fix 2026-03-15)
Travel moves raised above last extrusion Z. Non-extruding descent segment inserted at arrival. Departure Z-only move still missing (see P2 bug above).

### [ENH] ✅ Support generation integrated into toolpath flow — COMPLETE (2026-03-15)
`enable_supports` checkbox auto-generates and merges supports when "Generate Toolpaths" is clicked.

### [ENH] Machine collision detection — OBB-OBB separating axis test
Upgrade from AABB-of-OBB (conservative, can produce false positives) to a true 15-axis OBB-OBB SAT test for more accurate collision counts.

### [ENH] Machine profile presets
Add a dropdown of built-in presets for common 5-axis configurations ("Generic AB head-tilt", "Rotary table BC", "Robotic arm 6-axis") that fills in nozzle radius, TCP offset, and axis mode automatically.

### [ENH] Geodesic — multi-scale for CustomVectorField
Run the Poisson solve twice at different regularisation strengths and fuse the results, analogous to what multi-scale does for the heat step.

### [ENH] S3 — expose fallback warning in GUI
When VirtualScalarField fallback is triggered, show a yellow warning banner in the stats panel.

### [ENH] Adaptive layer height — expose min/max height in GUI
`min_layer_height` and `max_layer_height` are currently hardcoded constants in `slicing.rs`. Expose them as sliders.

### [ENH] Conical — per-region deferred contour timeout
Add a max-defer count and emit a warning if a contour is deferred more than N times.

### [ENH] Support generation — extend to non-planar modes (S4, Conical)
Supports are currently generated with a planar slicer. Extend to work with S4 and Conical modes where the print is non-planar.

### [ENH] Toolpath — seam placement control
Add seam placement option: back of model, minimum-visibility angle, or user-specified direction.

### [ENH] GPU-accelerated CG solver
The Conjugate Gradient solver in `geodesic.rs` is the primary bottleneck for large meshes (> 200K vertices: 10–30s per solve). A WGPU compute shader implementation would reduce this to < 1s.

### [ENH] Export — full 5-axis G-code validation
Add a post-processing pass that checks joint angle limits and interpolates through singularities.

### [ENH] Explicit Z-only departure lift for travel moves
Before each lifted travel segment, insert a `G1 Z{lift}` move at the departure position before the XY travel. Currently only the arrival descent is inserted.

---

## Architecture Notes for Future Work

- **`collision.rs` (root)** and **`singularity.rs` (root)** are legacy stubs with TODO comments. The real implementations are in `motion_planning/`. The stubs should be removed or re-exported.
- **`isotropic_remesh.rs`** is no longer on any active code path (voxel reconstruction replaced it). Can be deleted to reduce codebase size.
- **`deformation.rs`** and **`deformation_v2.rs`** — two versions of the surface ASAP solver exist. `deformation_v2.rs` is the active one; `deformation.rs` is legacy. Consider removing `deformation.rs`.
- **Machine simulation helper functions** (`rodrigues_f32`, `chained_rotation_f32`, `aabb_of_obb`, etc.) live in `app.rs`. They should be moved to a dedicated `kinematics.rs` module if reused elsewhere.
