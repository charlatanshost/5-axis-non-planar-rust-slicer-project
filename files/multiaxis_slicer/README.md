# MultiAxis Slicer

High-performance multi-axis non-planar 3D printing slicer written in Rust with an interactive egui GUI.

## Slicing Modes

| Mode | Description |
|---|---|
| **Planar** | Traditional flat-layer slicing. Fast baseline. |
| **Conical** | Cone-shifted Z for radially symmetric overhangs (RotBot-style). Axis tilt assigned analytically from cone-surface normal. |
| **S4 Non-Planar** | Z-biased Dijkstra distance field → mesh deformation → planar slice → barycentric untransform. One-click Support-Free Preset (35° overhang, 35° max rotation, z_bias 0.85). |
| **S3 Curved Layer** | Full S3-Slicer pipeline: quaternion field + volumetric ASAP deformation + marching tetrahedra. |
| **Geodesic (Heat Method)** | Layers follow geodesic distance from a source boundary. Four diffusion modes: isotropic, adaptive scalar, anisotropic, print-direction biased. |
| **Cylindrical** | Coordinate transform to cylindrical space — layers are concentric radial shells. |
| **Spherical** | Coordinate transform to spherical space — layers are concentric spherical shells. |

## Key Features

- **Printer profiles** — named machine profiles persist across sessions (eframe key-value storage); each profile stores axis limits, TCP offset, nozzle geometry, bed/head dimensions, and optional STL overrides; loaded and applied automatically on startup
- **Machine simulation** — bed and printhead rendered in the 3D viewport using parametric boxes/cylinders or custom STL files; geometry updates live as the toolpath playback scrubber moves; nozzle tip marked with a gold sphere; viewport renders even before a mesh is loaded
- **STL head/bed geometry** — override box shapes with any STL file; per-STL "tip offset" pins the model's local nozzle contact point to the segment world position
- **Surface normal orientations** — for all slicing modes a 48×48 XY bin grid finds the nearest mesh face; the face normal becomes the segment's rotary-axis direction, clamped to profile axis limits
- **Travel Z-lift** — travel moves longer than 1 mm are raised above the last extrusion Z by a configurable clearance (default 2 mm), preventing surface graze during traversal
- **Conical axis orientations** — analytical tilt from cone-surface normal: n = (−sinα·dx/r, −sinα·dy/r, cosα) for outward cone
- **Voxel reconstruction** — SDF + Marching Cubes repairs self-intersecting STL files in 2–5 seconds
- **S4 Z-biased Dijkstra** — edge weights blend `|ΔZ|` and Euclidean distance so the distance field tracks actual print height; prevents topologically-close features (e.g. bunny ears) from merging onto the same layer
- **Support-Free Preset** — one-click configuration for complex organic models (35° overhang / 35° max rotation / z_bias 0.85)
- **Adaptive layer height** — per-layer height set by local surface slope; viewport tube diameter scales to match
- **Multi-scale geodesic** — heat diffusion at several doubling timesteps, fused for fine detail + full coverage
- **Anisotropic geodesic diffusion** — curvature-aligned, print-direction biased, or custom vector field modes
- **Mesh-mapped gap filling** — `MeshRayCaster` projects infill and wall loop Z onto the actual surface; slope-adaptive scanline insertion fills 3D coverage gaps on steep curved surfaces
- **Wall seam transitions** — optional ruled-surface zigzag paths between consecutive curved layers
- **5-axis G-code** — A/B or B/C rotary axes with TCP (tool-centre-point) compensation
- **Capsule-vs-mesh collision detection** — parry3d capsule tested against every triangle with AABB pre-filter
- **Conical floating-contour filter** — 2D bin grid defers unsupported contours until the print surface below has been deposited
- **Interactive 3D GUI** — egui sidebar with live parameter controls, three-d viewport, G-code preview panel
- **Parallel processing** — Rayon used throughout slicing and field computation
- **113 unit tests** passing (3 pre-existing failures unrelated to current work)

## Quick Start

**Requirements:** Rust stable 1.75+, a C++ compiler (for TetGen)

```bash
cd files/multiaxis_slicer

# Run the GUI (release mode strongly recommended)
cargo run --bin gui --release

# Run tests
cargo test --lib
```

The GUI opens with a 3D viewport. Use **File → Load STL** to open a mesh, choose a slicing mode from the left panel, configure parameters, then click **Slice**.

## Architecture

```
src/
├── geometry.rs              Point3D, Vector3D, Plane, Triangle
├── mesh.rs                  Mesh loading, STL I/O
├── slicing.rs               Core planar slicer
├── toolpath.rs              Wall loops and infill generation (mesh-mapped Z)
├── toolpath_patterns.rs     MeshRayCaster, coverage_gap_fill, infill patterns
├── contour_offset.rs        2D polygon offset for wall loops
├── gcode.rs                 G-code output with 5-axis TCP compensation
├── geodesic.rs              Heat Method geodesic slicing (all diffusion modes)
├── conical.rs               Conical coordinate-transform pipeline
├── coordinate_transform.rs  Cylindrical and spherical pipelines
├── ruled_surface.rs         Wall seam transition path generation
├── centroidal_axis.rs       Centroidal axis computation
├── singularity.rs           Singularity avoidance (motion planning)
├── s3_slicer/
│   ├── pipeline.rs          S3 and S4 pipeline orchestration
│   ├── tet_mesh.rs          TetMesh — TetGen + grid-based Freudenthal decomposition
│   ├── voxel_remesh.rs      SDF + Marching Cubes mesh repair
│   ├── tet_asap_deformation.rs  Volumetric ASAP deformation (SVD)
│   ├── tet_quaternion_field.rs  Per-tet quaternion field optimization
│   ├── tet_dijkstra_field.rs    Multi-source Z-biased Dijkstra for S4
│   ├── s4_rotation_field.rs     Overhang-based rotation field for S4
│   ├── marching_tet.rs      Isosurface extraction via marching tetrahedra
│   └── tet_point_location.rs   Spatial bin grid + barycentric coordinates
├── motion_planning/
│   └── collision.rs         Capsule-vs-mesh collision detection (parry3d)
├── support_generation/
│   ├── overhang_detection.rs
│   ├── tree_skeleton.rs
│   └── support_toolpath.rs
└── gui/
    ├── app.rs               Application state, slicing threads, surface normal orientations, travel lift
    ├── control_panel.rs     Full parameter UI for all modes + Rotary Axes & Collision Avoidance
    ├── printer_profiles_page.rs  Printer profile editor — axes, TCP, bed/head geometry, STL overrides
    ├── viewport_3d.rs       3D rendering, machine bed/head, kinematic transforms, collision overlay
    └── stats_panel.rs       Slicing statistics + collision count
```

## Pipeline Overviews

```
Planar:      Flat planes at constant Z
Conical:     Z-shift by r·tan(angle) → planar slice → reverse shift
S4:          TetMesh → Z-biased Dijkstra → overhang rotations → ASAP deform → slice → bary untransform
S3:          TetMesh → quaternion field → ASAP deform → scalar field → marching tets
Geodesic:    Cotangent Laplacian → heat diffusion → gradient normalize → Poisson → level sets
Cylindrical: (x,y,z) → (θ, z, r) → planar slice → inverse transform
Spherical:   (x,y,z) → (θ, φ, r) → planar slice → inverse transform
```

## Testing

```bash
# Run all library tests (113 pass, 3 pre-existing failures)
cargo test --lib

# Run specific module
cargo test --lib -- geodesic
cargo test --lib -- voxel_remesh
cargo test --lib -- collision
```

Note: `cargo test` without `--lib` fails because two example binaries (`simple_slicer.rs`,
`slice_benchy.rs`) need a missing `max_rotation_degrees` field added to their config structs.
Use `cargo test --lib` until that is fixed.

## Key Dependencies

| Crate | Purpose |
|---|---|
| `nalgebra` 0.32 | Linear algebra, quaternions, SVD, sparse matrices |
| `tritet` 3.1 (optional) | TetGen wrapper — constrained Delaunay tetrahedralization |
| `sprs` 0.11 | Sparse matrix storage and arithmetic (Laplacian systems) |
| `rayon` 1.8 | Data-parallel iteration |
| `parry3d` 0.13 | Collision detection — capsule vs triangle |
| `eframe`/`egui` | Immediate-mode GUI |
| `three-d` | 3D viewport rendering |
| `stl_io` | STL file parsing |

> **TetGen note:** `tritet` is AGPL-3.0 licensed. Disable with `--no-default-features` to use the grid-based fallback tetrahedralization instead.

## References

1. "S3-Slicer: A General Slicing Framework for Multi-Axis 3D Printing" — Zhang et al., SIGGRAPH Asia 2022
2. "Geodesics in Heat" — Crane, Weischedel, Wardetzky, ACM TOG 2013
3. "An Optimal Algorithm for 3D Triangle Mesh Slicing" — Minetto, Volpato et al., CAD 2017
4. "Support-Free Volume Printing by Multi-Axis Motion" — Xu et al., ACM TOG 2020
5. "TetGen: A Delaunay-Based Quality Tetrahedral Mesh Generator" — Hang Si, ACM TOMS 2015
6. "A Remeshing Approach to Multiresolution Modeling" — Botsch & Kobbelt, SGP 2004
7. "Singularity-Aware Motion Planning for Multi-Axis Additive Manufacturing" — RAL 2021
8. "S4-Slicer" — jyjblrd (deform/slice/untransform approach)

## License

GNU General Public License v3 — see [LICENSE](../../LICENSE).
