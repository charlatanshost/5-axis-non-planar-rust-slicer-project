# GUI Features Reference

Last updated: 2026-03-22

This document describes all GUI controls and their effect on the slicing pipeline.

---

## Slicing Mode Selector

| Mode | Description |
|---|---|
| **Planar** | Traditional flat-layer slicing |
| **Conical** | Cone-shifted Z for radially symmetric overhangs |
| **S4 Non-Planar** | Dijkstra deform → planar slice → barycentric untransform |
| **S3 Curved Layer** | Quaternion field + volumetric ASAP + marching tetrahedra |
| **Geodesic (Heat Method)** | Layers follow geodesic distance from source boundary |
| **Cylindrical** | Coordinate transform — radial shell layers |
| **Spherical** | Coordinate transform — spherical shell layers |

---

## Common Slicing Parameters

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Layer height | Slider | 0.2 mm | 0.05–1.0 | Vertical distance between layers |
| Nozzle diameter | Slider | 0.4 mm | 0.1–2.0 | Controls wall loop offset and infill spacing |
| Adaptive layer height | Checkbox | Off | — | Per-layer height from local surface slope |
| Wall count | Slider | 2 | 1–10 | Number of perimeter loops |
| Infill density | Slider | 20% | 0–100 | Interior fill percentage |
| Infill pattern | Dropdown | Rectilinear | 6 options | See below |

### Infill Patterns

| Pattern | Description |
|---|---|
| Rectilinear | Alternating horizontal/vertical lines |
| Grid | Cross-hatching at 0° and 90° on alternating layers |
| Triangles | Cross-hatching at 0°, 60°, 120° on alternating layers |
| Concentric | Shrinking copies of the outer contour |
| Gyroid | Sine-wave pattern approximating TPMS gyroid geometry |
| Adaptive Cubic | Cubic grid with 3-layer rotation cycle for isotropic strength |

---

## S4 Non-Planar Configuration

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Auto Z-bias | Checkbox | On | — | Computes optimal z_bias from mesh aspect ratio |
| Z-bias | Slider | 0.80 | 0.0–1.0 | Blends Euclidean vs vertical distance (disabled when auto is on) |
| Overhang threshold | Slider | 45° | 20–80° | Angle above which overhangs are corrected |
| Max rotation | Slider | 15° | 5–60° | Maximum per-tet rotation to correct overhangs |
| Smoothing iterations | Slider | 25 | 5–100 | SLERP smoothing passes over tet graph |
| Smoothness weight | Slider | 0.5 | 0.0–1.0 | Balance between overhang correction and smoothness |
| Multi-axis overhangs | Checkbox | Off | — | 2-cluster axis normalization for multi-directional overhangs |
| Use ASAP deformation | Checkbox | Off | — | Full volumetric ASAP instead of direct vertex rotation |
| Support-Free Preset | Button | — | — | Sets z_bias=0.85, overhang=35°, rotation=35°, iterations=40, weight=0.6 |

### S4 Advanced Section (collapsible)

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Base detection threshold | Slider | 5% | 1–20% | Z-threshold for build-plate tet identification |
| Edge filter multiplier | Slider | 15.0 | 5–30 | Removes triangles with edges > median × multiplier |
| Jump-split multiplier | Slider | 10.0 | 5–30 | Contour split threshold as multiple of layer height |

---

## S3 Curved Layer Configuration

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Max rotation degrees | Slider | 15° | 5–60° | Maximum quaternion field rotation |
| Deformation method | Dropdown | TetVolumetric | 3 options | TetVolumetric / ASAP / VirtualScalarField |

---

## Conical Configuration

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Cone angle | Slider | 45° | 5–85° | Half-angle of the cone surface |
| Cone direction | Dropdown | Outward | In/Out | Inward or outward cone |

---

## Geodesic Configuration

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Heat timestep factor | Slider | 1.0 | 0.1–100.0 | Multiplier for heat diffusion time |
| Bottom tolerance | Slider | 0.1 | 0.0–1.0 | Source boundary thickness |
| Multi-scale | Checkbox | Off | — | Runs at N doubling timesteps |
| Number of scales | Slider | 4 | 2–8 | How many doubling timesteps (when multi-scale on) |
| Diffusion mode | Dropdown | Isotropic | 4 options | See below |

### Geodesic Diffusion Modes

| Mode | Extra Controls | Effect |
|---|---|---|
| Isotropic | None | Standard cotangent-weight Laplacian |
| Adaptive Scalar | Kappa base (0.1–10.0) | Per-face κ scales with triangle size |
| Anisotropic | Anisotropy ratio (0.1–10.0), Smoothing iters (0–5) | FEM tensor aligned to curvature direction |
| Print Direction | Direction axis (X/Y/Z), Direction ratio (0.1–10.0) | FEM tensor aligned to preferred print direction |

---

## Cylindrical / Spherical Configuration

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Auto-center | Checkbox | On | — | Centers transform on mesh centroid |
| Center X/Y/Z | Sliders | 0.0 | ±500 | Manual center (when auto-center off) |

---

## Toolpath Configuration

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Wall seam transitions | Checkbox | Off | — | Ruled-surface zigzag between consecutive wall contours |

---

## 5-Axis G-code Settings (collapsible)

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| TCP offset | Slider | 0.0 mm | 0–200 | Distance from pivot to nozzle tip |
| Rotary axis mode | Dropdown | AB | AB / BC | Axis letter convention |

---

## Rotary Axes & Collision Avoidance (collapsible)

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Use surface normals | Checkbox | On | — | Per-segment orientation from nearest mesh face |
| Travel Z-lift | Slider | 2.0 mm | 0–20 | Clearance above deposited material during travel |
| Smooth orientations | Checkbox | On | — | Bidirectional SLERP smoothing pass |
| Max angular rate | Slider | 15.0 °/mm | 1–45 | Maximum orientation change per mm (when smoothing on) |
| Look-ahead | Slider | 3 | 1–5 | Segments to look ahead in collision scoring |
| Singularity avoidance | Checkbox | Off | — | Penalty for near-gimbal-lock orientations |
| Singularity threshold | Slider | 0.1 | 0.01–0.5 | Manipulability threshold (when singularity on) |

---

## Support Generation (collapsible)

| Control | Type | Default | Range | Effect |
|---|---|---|---|---|
| Generate supports | Checkbox | Off | — | Auto-generates supports with toolpaths |
| Overhang angle | Slider | 45° | 20–80° | Angle threshold for support detection |
| Min area | Slider | 1.0 mm² | 0.1–50 | Minimum overhang area to generate support |

---

## Printer Profiles (separate page)

| Control | Type | Description |
|---|---|---|
| Profile list | List | Named profiles with add/delete/rename |
| Axis configuration | Fields | Per-axis name, limits, head/bed assignment |
| Nozzle length | Slider | Physical nozzle length in mm |
| Bed dimensions | Sliders | Width, depth, height of build platform |
| Head dimensions | Sliders | Width, depth, height of printhead body |
| STL bed/head | File picker | Override parametric shapes with custom STL |
| STL tip offset | Vector3 | Local coordinate of nozzle tip in STL model |
| Bed Z travel | Slider | Maximum bed descent during printing |

---

## Viewport Controls

| Action | Control |
|---|---|
| Rotate | Left mouse drag |
| Zoom | Scroll wheel |
| Pan | Right mouse drag |
| Layer scrubber | Slider in viewport overlay |
| Machine simulation | Rendered automatically from active printer profile |

---

## Actions

| Button | Effect |
|---|---|
| Load STL | Opens file dialog to load a mesh |
| Slice | Runs the selected slicing pipeline on background thread |
| Generate Toolpaths | Converts layers to toolpaths (includes supports if enabled) |
| Export G-code | Writes G-code file with 5-axis compensation |
| Support-Free Preset (★) | One-click S4 configuration for complex organic models |
