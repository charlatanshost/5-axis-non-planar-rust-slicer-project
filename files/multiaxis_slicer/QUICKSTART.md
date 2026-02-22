# Quick Start Guide

## 1. Build and Run (2 minutes)

**Requirements:** Rust 1.75+ stable, a C++ compiler

```bash
cd files/multiaxis_slicer

# Run the GUI — release mode is strongly recommended
cargo run --bin gui --release
```

The GUI opens with a 3D viewport. Everything is controlled from the left-side panel.

---

## 2. Slice Your First Model

1. Click **Load STL** (top of the left panel) and open any `.stl` file
2. The mesh appears in the 3D viewport — drag to rotate, scroll to zoom
3. Select a slicing mode from the **Slicing Mode** dropdown
4. Adjust parameters as needed
5. Click **Slice** — progress appears in the stats panel (bottom right)
6. Layers appear as coloured tubes in the viewport
7. Click **Export G-code** to write the toolpaths to a file

---

## 3. Choosing a Slicing Mode

| Mode | Best For | Notes |
|---|---|---|
| **Planar** | Any model, quick test | Standard flat layers. Fastest. |
| **Conical** | Radially symmetric models (vases, bowls) | Eliminates overhangs on rotational geometry. |
| **S4 Non-Planar** | Complex organic models (Stanford Bunny, figurines) | Click "Support-Free Preset" for a good starting point. Slicing takes 5–30 seconds. |
| **S3 Curved Layer** | Research / maximum quality | Slowest. Full S3-Slicer paper pipeline. |
| **Geodesic** | Surface-conforming layers on curved objects | Layers follow the mesh surface rather than flat planes. |
| **Cylindrical** | Vases, cylinders, tubes | Layers are concentric radial shells. |
| **Spherical** | Spheres, domes | Layers are concentric spherical shells. |

### S4 — Support-Free Printing

S4 Non-Planar is the recommended mode for eliminating support structures on complex models:

1. Select **S4 Non-Planar**
2. Click **"★ Support-Free Preset"** — sets z_bias=0.85, overhang=35°, max rotation=35°
3. Click **Slice**

If overhangs are still present after slicing, try increasing **Max Rotation** (up to ~45°) or lowering **Overhang Threshold** (down to ~25°). Raising too high may cause tet inversion and a silent fallback to a simpler method.

### Geodesic — Surface-Conforming Layers

1. Select **Geodesic (Heat Method)**
2. Set **Layer Height** (e.g. 0.2mm)
3. Leave **Diffusion Mode** as **Isotropic** for most models
4. Click **Slice**

For strongly anisotropic models (e.g. a model that's much taller in one axis), try **Print Direction Biased** mode and point the preferred direction along the tall axis.

---

## 4. Toolpath Settings

Under **Toolpath Settings** in the left panel:

- **Nozzle Diameter** — sets wall loop width and infill line spacing
- **Layer Height** — controls extrusion thickness
- **Wall Count** — number of perimeter loops (1–4)
- **Infill Density** — fraction of interior filled (0.0 = none, 1.0 = solid)
- **Wall Seam Transitions** — inserts a ruled-surface path between consecutive curved layers to fill the staircase gap on the outer wall (recommended for geodesic and S4 modes)

---

## 5. G-code Export

Under **G-code Settings**:

- **Nozzle Temp / Bed Temp** — printer temperatures
- **Print Speed / Travel Speed** — mm/s
- **5-Axis G-code Settings** — set TCP offset (mm from pivot to nozzle tip) and axis format (A/B or B/C) for multi-axis machines

Click **Export G-code** to save. For 5-axis machines, ensure your post-processor or machine controller accepts the `A`/`B` (or `B`/`C`) rotation columns in the G1 moves.

---

## 6. Running Tests

```bash
# Library tests only — 108 pass, 3 pre-existing failures
cargo test --lib

# Run a specific module's tests
cargo test --lib -- geodesic
cargo test --lib -- voxel_remesh
```

> **Note:** `cargo test` without `--lib` will fail. Two example binaries
> (`simple_slicer.rs`, `slice_benchy.rs`) have a missing struct field
> (`max_rotation_degrees`) that has not yet been updated. Always use
> `cargo test --lib`.

---

## 7. Common Issues

### The GUI window is blank or doesn't open

Ensure you are running in **release** mode:

```bash
cargo run --bin gui --release
```

Debug builds of `three-d` (the 3D renderer) are very slow and may appear frozen.

### S4 slicing falls back to a flat result

The ASAP deformation quality check failed — too many inverted tetrahedra. Try:
- Lowering **Max Rotation Degrees** (e.g. 25° instead of 35°)
- Increasing **Smoothing Iterations** to better distribute rotations

### No layers produced (0 layers after slicing)

- For **Cylindrical/Spherical** modes: the model must span a meaningful radial range. A flat disc with all vertices at the same radius produces no layers.
- For **Planar/S4**: ensure the model is above Z=0 (bed level). Very thin models may produce 0 layers if the layer height is too large.

### TetGen error / mesh repair message

For complex STL files (e.g. the Stanford Bunny), TetGen rejects self-intersecting faces. The pipeline automatically falls back to **voxel reconstruction** (SDF + Marching Cubes), which produces a clean manifold surface in 2–5 seconds. No action needed — this is normal.

### Example binaries don't compile

`cargo build` or `cargo test` (without `--lib`) compiles the example binaries, which reference an outdated `S3PipelineConfig` struct. This is a known issue. Use `cargo run --bin gui --release` and `cargo test --lib` instead.

---

## 8. Getting Help

- See [KNOWN_ISSUES.md](../../KNOWN_ISSUES.md) for a prioritised bug and enhancement list
- See [TECHNICAL.md](../../TECHNICAL.md) for algorithm details and implementation notes
- Run `cargo doc --open` for inline API documentation
