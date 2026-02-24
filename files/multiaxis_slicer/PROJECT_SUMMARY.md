# MultiAxis Slicer — Project Summary

## What This Is

A fully-implemented Rust slicer for multi-axis non-planar 3D printing. All major features are complete and working. The GUI runs, slices real STL files, and exports valid G-code.

---

## What's Implemented

### Slicing Modes (7 total)

| Mode | Status | Key Algorithm |
|---|---|---|
| Planar | Complete | O(n log k + k + m) plane-intersection slicer |
| Conical | Complete | Z-shift by r·tan(α), planar slice, inverse shift |
| S4 Non-Planar | Complete | Z-biased Dijkstra + ASAP deformation + barycentric untransform |
| S3 Curved Layer | Complete | Quaternion field + volumetric ASAP + marching tetrahedra |
| Geodesic (Heat Method) | Complete | Cotangent Laplacian, CG solver, 4 diffusion modes, multi-scale |
| Cylindrical | Complete | (x,y,z) → (θ, z, r) coordinate transform |
| Spherical | Complete | (x,y,z) → (θ, φ, r) coordinate transform |

### Core Infrastructure

- **Mesh loading** — STL (ASCII and binary), bounds computation, triangle adjacency
- **Voxel reconstruction** — SDF + Marching Cubes repairs self-intersecting meshes in 2–5 seconds
- **Tetrahedral mesh generation** — TetGen (via `tritet`) with 4-strategy cascade: direct, voxel reconstruction, vertex clustering, convex hull
- **Toolpath generation** — wall loops (contour offset), rectilinear infill, adaptive layer height
- **MeshRayCaster** — 2D bin grid + vertical ray casting; projects wall loop and infill Z onto the actual mesh surface for curved layers
- **Coverage gap fill** — slope-adaptive scanline insertion fills 3D coverage gaps where infill spacing exceeds 2× nominal on steep surfaces
- **Wall seam transitions** — ruled-surface zigzag path between consecutive curved layer contours; fills staircase gap on the outer wall
- **5-axis G-code** — A/B or B/C rotary axis output with TCP (tool-centre-point) compensation
- **Capsule collision detection** — parry3d capsule vs mesh triangles with AABB pre-filter
- **Conical floating-contour filter** — 2D per-XY bin grid; defers unsupported contours until the bed below is printed
- **Support generation** — overhang detection, tree skeleton, support toolpaths (not yet integrated into non-planar GUI flow)
- **Printer profiles** — named machine profiles with persistent storage (eframe key-value); each stores axis limits, TCP offset, nozzle geometry, bed/head dimensions, and optional STL overrides; applied automatically on startup
- **Machine simulation** — bed and printhead rendered in the 3D viewport as parametric boxes/cylinders or custom STL files; live kinematic updates during toolpath playback; nozzle tip marked with gold sphere
- **Surface normal orientations** — per-segment rotary axis direction from nearest mesh face normal (48×48 XY bin grid), clamped to profile axis limits; conical mode uses analytical cone-surface normals
- **Travel Z-lift** — travel moves raised above last extrusion Z by configurable clearance (default 2 mm)
- **Interactive 3D GUI** — egui sidebar, three-d viewport with layer-height-scaled tube rendering, G-code preview, stats panel; renders machine geometry even without a loaded mesh

### S4 Non-Planar — Key Details

- **Z-biased Dijkstra** (`tet_dijkstra_field.rs`): edge weight = `|ΔZ| × z_bias + Euclidean × (1 − z_bias)`. Prevents topologically-close features (e.g. two ears of the Stanford Bunny) from merging onto the same layer because they have the same graph path length.
- **Support-Free Preset**: z_bias=0.85, overhang_threshold=35°, max_rotation_degrees=35°, smoothing_iterations=40, smoothness_weight=0.6
- **Quality check**: if ASAP deformation produces >30% inverted tets or >5× bounding-box growth, the pipeline silently falls back to VirtualScalarField

### Geodesic Slicing — Diffusion Modes

- **Isotropic** — standard cotangent-weight Laplacian
- **AdaptiveScalar** — per-face κ(f) = kappa_base × (avg_edge)², clamped [0.05, 50]
- **Anisotropic** — FEM tensor aligned to local curvature direction, with optional Laplacian smoothing
- **PrintDirectionBiased** — FEM tensor aligned to a user-specified global direction projected per-face
- **Multi-scale** — runs at several doubling timesteps and fuses results for fine detail + full coverage

---

## Code Statistics

- **Language**: Rust (stable 1.75+)
- **Build target**: `cargo run --bin gui --release`
- **Test count**: 113 passing, 3 pre-existing failures (unrelated to current features)
- **Key source files**: ~25 `.rs` files in `src/`, ~15 in `src/s3_slicer/`, ~7 in `src/gui/`

---

## File Structure

```
files/multiaxis_slicer/
├── Cargo.toml                   Dependencies (nalgebra, egui, three-d, tritet, parry3d, rayon, sprs)
├── src/
│   ├── lib.rs                   Module declarations
│   ├── geometry.rs              Point3D, Vector3D, Plane, Triangle
│   ├── mesh.rs                  Mesh, STL I/O
│   ├── slicing.rs               Planar slicer (Layer, Contour, adaptive height)
│   ├── toolpath.rs              ToolpathGenerator (walls, infill, mesh Z-projection, transitions)
│   ├── toolpath_patterns.rs     MeshRayCaster, coverage_gap_fill, infill patterns
│   ├── contour_offset.rs        2D polygon offset for wall loops
│   ├── gcode.rs                 G-code with 5-axis TCP compensation, AB/BC axis modes
│   ├── geodesic.rs              Heat Method, all diffusion modes, multi-scale, marching triangles
│   ├── conical.rs               Conical pipeline + floating-contour filter
│   ├── coordinate_transform.rs  Cylindrical + spherical pipelines
│   ├── ruled_surface.rs         Wall seam transition generation (RuledSurface, resample, align)
│   ├── centroidal_axis.rs       Fast centroidal axis computation
│   ├── singularity.rs           Singularity avoidance utilities
│   ├── collision.rs             Legacy stub (active implementation in motion_planning/)
│   ├── s3_slicer/
│   │   ├── pipeline.rs          execute_s3_pipeline, execute_s4_pipeline, execute_tet_pipeline
│   │   ├── tet_mesh.rs          TetMesh, TetGen integration, 4-strategy cascade
│   │   ├── voxel_remesh.rs      SDF + Marching Cubes mesh repair
│   │   ├── tet_asap_deformation.rs  Volumetric ASAP (SVD per-tet)
│   │   ├── tet_quaternion_field.rs  Per-tet quaternion optimization
│   │   ├── tet_dijkstra_field.rs    Z-biased multi-source Dijkstra
│   │   ├── s4_rotation_field.rs     Overhang-based rotation field, SLERP smoothing
│   │   ├── tet_point_location.rs    Spatial bin grid + barycentric coordinates
│   │   ├── marching_tet.rs      Marching tetrahedra isosurface extraction
│   │   ├── tet_scalar_field.rs  Per-vertex scalar field, Laplacian smoothing
│   │   └── isotropic_remesh.rs  Legacy isotropic remeshing (no longer active path)
│   ├── motion_planning/
│   │   └── collision.rs         check_collision_with_mesh() — capsule vs triangles (parry3d)
│   ├── support_generation/
│   │   ├── overhang_detection.rs
│   │   ├── tree_skeleton.rs
│   │   └── support_toolpath.rs
│   └── gui/
│       ├── app.rs               SlicerApp, slicing threads, surface normals, travel lift
│       ├── control_panel.rs     All parameter controls (all 7 modes + toolpath + G-code + rotary axes)
│       ├── printer_profiles_page.rs  Printer profile editor, machine simulation settings
│       ├── viewport_3d.rs       3D rendering, machine bed/head, kinematic transforms
│       └── stats_panel.rs       Slicing statistics + collision count display
└── src/bin/
    ├── gui.rs                   Entry point for cargo run --bin gui
    ├── simple_slicer.rs         Example (currently does not compile — missing struct field)
    └── slice_benchy.rs          Example (currently does not compile — missing struct field)
```

---

## Known Limitations

See [KNOWN_ISSUES.md](../../KNOWN_ISSUES.md) for the full prioritised list. Key items:

- Example binaries (`simple_slicer.rs`, `slice_benchy.rs`) need one struct field added before they compile
- S4 deep undercuts (>55°) may not be fully corrected even with the support-free preset
- Wall seam transitions produce artifacts on layers with multiple separate contour islands
- Branch-cut seam artifact at θ = ±π in cylindrical and spherical modes
- Support generation is not yet wired into the non-planar slicing modes in the GUI

---

## Dependencies

| Crate | License | Purpose |
|---|---|---|
| `nalgebra` | Apache-2.0/MIT | Linear algebra, quaternions, SVD |
| `tritet` (optional) | AGPL-3.0 | TetGen wrapper — Delaunay tetrahedralization |
| `sprs` | Apache-2.0/MIT | Sparse matrix storage and arithmetic |
| `rayon` | Apache-2.0/MIT | Parallel iteration |
| `parry3d` | Apache-2.0 | Collision detection |
| `eframe`/`egui` | Apache-2.0/MIT | GUI framework |
| `three-d` | MIT | 3D viewport rendering |
| `stl_io` | MIT | STL file parsing |
| `crossbeam` | Apache-2.0/MIT | Background thread channels |
| `rfd` | MIT | Native file dialogs |

> TetGen (`tritet`) is AGPL-3.0. Build with `--no-default-features` to disable it and use the grid-based fallback instead.
