# 5-Axis Non-Planar Slicer

A high-performance multi-axis non-planar 3D printing slicer written in Rust. Generates curved, surface-following, and volumetrically deformed toolpaths for 3, 4, and 5-axis 3D printers and CNC machines — going well beyond flat-layer slicing.

---

## What It Does

Traditional slicers cut geometry into flat horizontal layers. This slicer can instead:

- **Follow surface curvature** — layers wrap around the mesh surface using geodesic distance fields, so a sphere prints in concentric shells rather than flat discs
- **Deform space and slice** — the mesh is warped so overhangs unfold, sliced flat, then the toolpaths are warped back to the original geometry
- **Optimize print orientation per-voxel** — a quaternion field assigns the best local print direction to every region of the volume simultaneously
- **Handle any STL** — broken, self-intersecting, or non-manifold files are automatically repaired via voxel reconstruction before processing

The output is standard G-code extended with A/B rotation axes for 5-axis machines, with optional TCP (tool-centre-point) compensation.

---

## Slicing Modes

| Mode | Description |
|---|---|
| **Planar** | Traditional flat-layer slicing. Fast baseline. |
| **Conical** | Cone-shifted Z for radially symmetric overhangs (RotBot-style). Axis tilt assigned analytically from cone-surface normal. |
| **S4 Non-Planar** | Dijkstra-based mesh deformation → planar slice → barycentric untransform. Z-biased distance field keeps topologically-close features (e.g. two ears of the Stanford Bunny) on separate layers. |
| **S3 Curved Layer** | Full S3-Slicer pipeline: quaternion field + volumetric ASAP deformation + marching tetrahedra. |
| **Geodesic (Heat Method)** | Layers follow geodesic distance from a source boundary using the heat method. Multiple diffusion modes: isotropic, adaptive scalar, anisotropic, print-direction biased. |
| **Cylindrical** | Coordinate transform to cylindrical space — layers are concentric radial shells. |
| **Spherical** | Coordinate transform to spherical space — layers are concentric spherical shells. |

---

## Key Features

- **Printer profiles** — named machine profiles persist across sessions; each profile stores nozzle geometry, TCP offset, axis limits, bed/head dimensions, and optional STL overrides; profiles load automatically on startup
- **Machine simulation** — bed and printhead rendered in the 3D viewport as parametric boxes/cylinders or custom STL files; updates live as the toolpath playback scrubber moves; nozzle tip marked with a gold sphere
- **STL head/bed geometry** — load any STL file to replace the default box shapes; set a local "tip offset" so the model's nozzle contact point aligns exactly with the toolpath position
- **Surface normal orientations** — for all slicing modes, each toolpath segment's rotary-axis angle is set from the nearest mesh face normal; axis tilt is clamped to the profile's configured limits
- **Travel Z-lift** — travel moves longer than 1 mm are raised by a configurable clearance height (default 2 mm) above the last extruded position, avoiding surface collisions during traversal
- **Conical axis orientations** — in Conical mode the rotary tilt is computed analytically from the cone-surface normal rather than from the mesh surface
- **Voxel reconstruction** — SDF + Marching Cubes repairs self-intersecting STL files in 2–5 seconds (versus ~28 minutes for isotropic remeshing)
- **Multi-scale geodesic** — runs heat diffusion at several doubling timesteps and fuses results, giving both fine local detail and full-mesh coverage simultaneously
- **Anisotropic diffusion** — curvature-aligned, print-direction biased, or custom vector field modes shape how geodesic layers follow the geometry
- **S4 Z-biased Dijkstra** — edge weights blend |ΔZ| and Euclidean distance so the distance field tracks actual print height, not graph path length; includes a one-click "Support-Free Preset" (35° overhang, 35° max rotation, z_bias 0.85) tuned for complex organic models
- **Adaptive layer height** — per-layer height set by local surface slope; viewport tube diameter scales to match actual layer height
- **Mesh-mapped gap filling** — `MeshRayCaster` projects infill and wall loop Z onto the actual surface; slope-adaptive scanline insertion fills 3D coverage gaps on steep curved surfaces
- **Wall seam transitions** — optional ruled-surface zigzag paths between consecutive curved layers to fill the staircase gap at the outer wall
- **5-axis G-code** — direct output with A/B (or B/C) rotary axes and TCP compensation
- **Capsule-vs-mesh collision detection** — parry3d capsule shape tested against every triangle with AABB pre-filter
- **Conical floating-contour filter** — 2D per-XY bin grid tracks maximum printed Z at each location; defers unsupported contours until the print surface below them has been deposited
- **Interactive 3D GUI** — egui sidebar with live parameter controls, three-d viewport with layer-height-scaled tube rendering, G-code preview panel; viewport renders machine geometry even without a loaded mesh
- **Parallel processing** — Rayon used throughout slicing and field computation
- **113 unit tests** passing (3 pre-existing failures unrelated to current work)

---

## Quick Start

**Requirements:** Rust stable 1.75+, a C++ compiler (for TetGen)

```bash
git clone <this-repo>
cd "Non-planar slicer project/files/multiaxis_slicer"

# Run the GUI (release mode strongly recommended)
cargo run --bin gui --release

# Run tests
cargo test --lib
```

The GUI opens with a 3D viewport. Use **File → Load STL** to open a mesh, choose a slicing mode from the left panel, configure parameters, then click **Slice**.

---

## Project Structure

```
files/multiaxis_slicer/src/
├── geometry.rs              Point3D, Vector3D, Plane, Triangle
├── mesh.rs                  Mesh loading, STL I/O
├── slicing.rs               Core planar slicer
├── toolpath.rs              Wall loops and infill generation
├── gcode.rs                 G-code output with 5-axis TCP compensation
├── geodesic.rs              Heat Method geodesic slicing (all diffusion modes)
├── conical.rs               Conical coordinate-transform pipeline
├── coordinate_transform.rs  Cylindrical and spherical pipelines
├── s3_slicer/
│   ├── pipeline.rs          S3 and S4 pipeline orchestration
│   ├── tet_mesh.rs          TetMesh — TetGen + grid-based Freudenthal decomposition
│   ├── voxel_remesh.rs      SDF + Marching Cubes mesh repair
│   ├── tet_asap_deformation.rs  Volumetric ASAP deformation (SVD)
│   ├── tet_quaternion_field.rs  Per-tet quaternion field optimization
│   ├── tet_dijkstra_field.rs    Multi-source Dijkstra for S4
│   ├── marching_tet.rs      Isosurface extraction via marching tetrahedra
│   └── tet_point_location.rs   Spatial bin grid + barycentric coordinates
├── motion_planning/
│   └── collision.rs         Capsule-vs-mesh collision detection (parry3d)
├── support_generation/
│   ├── overhang_detection.rs
│   └── tree_skeleton.rs
└── gui/
    ├── app.rs               Application state, background slicing threads, surface normals, travel lift
    ├── control_panel.rs     Full parameter UI for all modes + rotary axes & collision avoidance
    ├── printer_profiles_page.rs  Printer profile editor with machine simulation settings
    ├── viewport_3d.rs       3D rendering, machine bed/head geometry, collision highlighting
    └── stats_panel.rs       Slicing statistics
```

For a full explanation of every algorithm and module see [TECHNICAL.md](TECHNICAL.md).

---

## Dependencies and Credits

### Academic Papers This Project Implements

| Paper | Authors | Where Used |
|---|---|---|
| [S3-Slicer: A General Slicing Framework for Multi-Axis 3D Printing](https://dl.acm.org/doi/10.1145/3550469.3555430) | Tianyu Zhang et al. — SIGGRAPH Asia 2022 | S3 pipeline, quaternion field, ASAP deformation |
| [Geodesics in Heat](https://dl.acm.org/doi/10.1145/2516971.2516977) | Crane, Weischedel, Wardetzky — ACM TOG 2013 | Geodesic slicing |
| [An Optimal Algorithm for 3D Triangle Mesh Slicing](https://doi.org/10.1016/j.cad.2017.07.001) | Minetto, Volpato et al. — CAD 2017 | Core planar slicing algorithm |
| [Support-Free Volume Printing by Multi-Axis Motion](https://doi.org/10.1145/3386569.3392436) | Xu et al. — ACM TOG 2020 | Overhang-free print orientation |
| [A Remeshing Approach to Multiresolution Modeling](https://doi.org/10.2312/SGP/SGP04/185-192) | Botsch & Kobbelt — SGP 2004 | Isotropic remeshing (legacy path) |
| [Singularity-Aware Motion Planning for Multi-Axis Additive Manufacturing](https://doi.org/10.1109/LRA.2021.3061398) | RAL 2021 | Singularity avoidance |
| [TetGen: A Delaunay-Based Quality Tetrahedral Mesh Generator](https://dl.acm.org/doi/10.1145/2629697) | Hang Si — ACM TOMS 2015 | Constrained Delaunay tetrahedralization |

### Open-Source Projects

| Project | License | Authors / Org | Used For |
|---|---|---|---|
| [S3-Slicer reference implementation](https://github.com/zhangty019/S3_DeformFDM) | — | Tianyu Zhang | Algorithm reference for S3 pipeline |
| [S4-Slicer](https://github.com/jyjblrd/S4_Slicer) | — | jyjblrd | Inspiration for S4 deform/slice/untransform |
| [nalgebra](https://nalgebra.org) | Apache-2.0 / MIT | Dimforge | Linear algebra, quaternions, SVD, sparse matrices |
| [egui](https://github.com/emilk/egui) | Apache-2.0 / MIT | Emil Ernerfeldt | Immediate-mode GUI |
| [eframe](https://github.com/emilk/egui) | Apache-2.0 / MIT | Emil Ernerfeldt | Application framework for egui |
| [three-d](https://github.com/asny/three-d) | MIT | asny | 3D viewport rendering |
| [parry3d](https://parry.rs) | Apache-2.0 | Dimforge | Collision detection (capsule vs triangle) |
| [tritet / TetGen](https://github.com/cpmech/tritet) | AGPL-3.0 | Hang Si / cpmech | Constrained Delaunay tetrahedralization |
| [rayon](https://github.com/rayon-rs/rayon) | Apache-2.0 / MIT | Josh Stone, Niko Matsakis | Data-parallel iteration |
| [sprs](https://github.com/vbarrielle/sprs) | Apache-2.0 / MIT | Vincent Barrielle | Sparse matrix storage and arithmetic |
| [spade](https://github.com/Stoeoef/spade) | Apache-2.0 / MIT | Stoeoef | Spatial data structures |
| [geo](https://github.com/georust/geo) | Apache-2.0 / MIT | GeoRust | 2D geometry primitives |
| [glam](https://github.com/bitshifter/glam-rs) | MIT / Apache-2.0 | bitshifter | SIMD vector math |
| [stl_io](https://github.com/hmeyer/stl_io) | MIT | hmeyer | STL file parsing |
| [serde](https://serde.rs) | Apache-2.0 / MIT | Erick Tryzelaar, David Tolnay | Serialization |
| [crossbeam](https://github.com/crossbeam-rs/crossbeam) | Apache-2.0 / MIT | crossbeam-rs | Background thread channels |
| [rfd](https://github.com/PolyMeilex/rfd) | MIT | PolyMeilex | Native file dialogs |
| [palette](https://github.com/Ogeon/palette) | Apache-2.0 / MIT | Ogeon | Color management |
| [anyhow](https://github.com/dtolnay/anyhow) / [thiserror](https://github.com/dtolnay/thiserror) | Apache-2.0 / MIT | David Tolnay | Error handling |
| [ordered-float](https://github.com/reem/rust-ordered-float) | MIT | Jonathan Reem | Ord/Hash for f64 |

> **TetGen note:** The `tritet` crate wraps TetGen which is AGPL-3.0 licensed. If AGPL is not acceptable for your use case, build with `--no-default-features` to disable TetGen and fall back to the grid-based tetrahedralization.

---

## License

GNU General Public License v3 — see [LICENSE](LICENSE).

Portions of the algorithm are derived from or inspired by the papers and projects listed above; see their respective licenses for those components.
