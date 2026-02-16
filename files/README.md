# MultiAxis Slicer

High-performance multi-axis non-planar 3D printing slicer written in Rust, featuring multiple slicing methods including curved-layer (S3/S4), conical, and geodesic slicing.

## Features

### 5 Slicing Modes
- **Planar (Traditional)** - Fast flat-layer slicing at constant Z height
- **Conical (RotBot)** - Cone-based Z-shift for radially symmetric overhangs
- **S4 Non-Planar** - Deform mesh via Dijkstra distance field, planar slice, un-deform toolpaths back to original space
- **S3 Curved Layer** - Full S3-Slicer pipeline: quaternion field optimization, volumetric ASAP deformation, isosurface extraction
- **Geodesic (Heat Method)** - NEW: Layers follow surface curvature via geodesic distance field (Crane et al. 2013). Two source modes: bottom boundary or point source

### Core Capabilities
- **S3-Slicer curved-layer pipeline** - Full implementation of the SIGGRAPH Asia 2022 paper
- **Tetrahedral volumetric deformation** - Per-tet ASAP deformation with SVD-based scaling
- **Grid-based tet mesh** - Freudenthal 6-tet decomposition bypasses TetGen for self-intersecting meshes
- **Voxel-based mesh reconstruction** - SDF + Marching Cubes for clean manifold input
- **Geodesic distance field** - Heat Method with cotangent Laplacian and Conjugate Gradient solver
- **Wall loops and infill** - Configurable perimeter count and rectilinear infill density
- **Interactive GUI** - egui-based application with 3D viewport and parameter controls
- **Fast mesh slicing** - O(n log k + k + m) algorithm
- **5-axis toolpath generation** - Support for A/B rotation axes
- **G-code generation** - Direct output for CNC/3D printers
- **Support generation** - Overhang detection and tree-based support structures

## Quick Start

### Prerequisites

- [Rust toolchain](https://rustup.rs/) (stable, 1.75+)
- C++ compiler (required by TetGen dependency)

```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Build & Run

```bash
cd multiaxis_slicer

# Build and run the GUI application (release mode recommended)
cargo run --bin gui --release

# Run tests
cargo test --lib

# Build library only
cargo build --release
```

The GUI will open with a 3D viewport. Load an STL file and select a deformation method to start slicing.

## Slicing Pipelines

### Geodesic (Heat Method) - NEW
Layers follow the mesh surface curvature using geodesic distance:
1. **Build topology** - Weld vertices, compute adjacency and normals
2. **Cotangent Laplacian** - Build sparse symmetric matrix with lumped mass
3. **Heat Method** - Diffuse heat from source, normalize gradient, Poisson solve (CG solver)
4. **Level set extraction** - Marching triangles at uniform geodesic spacing

### S4 Non-Planar (Deform + Slice + Untransform)
1. **Grid tet mesh** - Freudenthal 6-tet decomposition (bypasses TetGen)
2. **Dijkstra distance field** - Per-tet gradients from base
3. **Rotation field** - Overhang-based rotations with SLERP smoothing
4. **Deform + slice + untransform** - Barycentric interpolation preserves original mesh topology

### S3 Curved Layer (Tet Volumetric)
Full S3-Slicer paper implementation:
1. **Mesh preprocessing** - Voxel reconstruction for clean manifold input
2. **Tetrahedralization** - TetGen or grid-based tet mesh
3. **Quaternion field optimization** - Per-tet rotations for fabrication objectives
4. **Volumetric ASAP deformation** - Per-tet deformation with SVD scaling
5. **Scalar field + Marching tetrahedra** - Isosurface extraction

### Conical (RotBot)
Simple Z-shift by `r * tan(angle)`, planar slice, reverse shift. Fast and effective for radially symmetric overhangs.

## Architecture

```
multiaxis_slicer/
├── src/
│   ├── lib.rs                    # Library entry point
│   ├── bin/gui.rs                # GUI application entry point
│   │
│   ├── geometry.rs               # Geometric primitives (Triangle, Point3D, etc.)
│   ├── mesh.rs                   # Mesh loading (STL) and manipulation
│   ├── slicing.rs                # Core planar slicing algorithms
│   ├── toolpath.rs               # Toolpath generation (walls + infill)
│   ├── toolpath_patterns.rs      # Infill patterns (linear, concentric, etc.)
│   ├── contour_offset.rs         # 2D polygon offset for wall loops
│   ├── geodesic.rs               # Geodesic slicing (Heat Method + level sets)
│   ├── conical.rs                # Conical slicing pipeline
│   ├── gcode.rs                  # G-code output
│   ├── centroidal_axis.rs        # Centroidal axis computation
│   ├── ruled_surface.rs          # Ruled surface detection
│   ├── singularity.rs            # Singularity optimization
│   ├── collision.rs              # Collision detection
│   │
│   ├── s3_slicer/                # S3-Slicer curved-layer pipeline
│   │   ├── pipeline.rs           # Main pipeline orchestration
│   │   ├── voxel_remesh.rs       # SDF + Marching Cubes mesh reconstruction
│   │   ├── tet_mesh.rs           # TetMesh struct, TetGen integration
│   │   ├── tet_quaternion_field.rs  # Per-tet quaternion field optimization
│   │   ├── tet_asap_deformation.rs  # Volumetric ASAP with per-tet scaling
│   │   ├── tet_scalar_field.rs   # Volume-based scalar field
│   │   ├── marching_tet.rs       # Marching tetrahedra isosurface extraction
│   │   ├── quaternion_field.rs   # Surface-based quaternion field
│   │   ├── asap_deformation.rs   # Surface ASAP solver
│   │   ├── deformation.rs        # Mesh deformation utilities
│   │   ├── deformation_v2.rs     # Scale-controlled deformation
│   │   ├── scalar_field.rs       # Surface scalar field
│   │   ├── isosurface.rs         # Marching triangles isosurface extraction
│   │   ├── heat_method.rs        # Geodesic distance via heat method
│   │   └── isotropic_remesh.rs   # Botsch & Kobbelt 2004 remeshing (legacy)
│   │
│   ├── gui/                      # egui GUI application
│   │   ├── app.rs                # Main application state and logic
│   │   ├── control_panel.rs      # Parameter controls sidebar
│   │   ├── viewport_3d.rs        # 3D mesh rendering
│   │   ├── stats_panel.rs        # Slicing statistics display
│   │   ├── gcode_panel.rs        # G-code preview
│   │   └── theme.rs              # UI theming
│   │
│   ├── support_generation/       # Support structure generation
│   │   ├── overhang_detection.rs # Detect overhanging faces
│   │   ├── tree_skeleton.rs      # Tree-based support structures
│   │   └── support_toolpath.rs   # Support toolpath generation
│   │
│   └── motion_planning/          # Multi-axis motion planning
│
├── Cargo.toml
└── README.md
```

## Key Dependencies

| Crate | Purpose |
|-------|---------|
| `nalgebra` | Linear algebra (vectors, matrices, quaternions, SVD) |
| `tritet` | TetGen wrapper for constrained Delaunay tetrahedralization |
| `sprs` | Sparse matrix operations for Laplacian systems |
| `rayon` | Parallel iteration |
| `parry3d` | Convex hull computation |
| `eframe`/`egui` | GUI framework |
| `three-d` | 3D rendering in the viewport |

**Note:** TetGen (`tritet`) is AGPL licensed and enabled via the `tetgen` cargo feature flag (on by default). Disable with `--no-default-features` if AGPL is not acceptable.

## Testing

```bash
# Run all library tests (93 pass, 3 pre-existing failures; skips examples)
cargo test --lib

# Run specific test module
cargo test --lib -- geodesic
cargo test --lib -- voxel_remesh
cargo test --lib -- tet_mesh

# Run with log output
RUST_LOG=info cargo test --lib -- --nocapture

# Run benchmarks
cargo bench
```

## Performance

The voxel reconstruction + TetGen pipeline processes an 80K-triangle Stanford Bunny in ~5-10 seconds (release mode), compared to ~28 minutes for the previous isotropic remeshing approach.

| Operation | Time (Release) |
|-----------|---------------|
| Voxel reconstruction (80K tris) | ~2-5s |
| TetGen tetrahedralization | ~1-2s |
| Quaternion field optimization | ~1s |
| ASAP volumetric deformation | ~2-5s |
| Scalar field + layer extraction | ~1-2s |

## References

1. "S3-Slicer: A General Slicing Framework for Multi-Axis 3D Printing" (SIGGRAPH Asia 2022)
   - [Paper](https://dl.acm.org/doi/10.1145/3550469.3555430) | [Code](https://github.com/zhangty019/S3_DeformFDM)
2. "The Heat Method for Distance Computation" - Crane, Weischedel, Wardetzky (2013)
3. "S4-Slicer" - jyjblrd (Deform/slice/untransform approach)
4. "An Optimal Algorithm for 3D Triangle Mesh Slicing"
5. "Support-Free Volume Printing by Multi-Axis Motion" (TOG 2020)
6. "A Remeshing Approach to Multiresolution Modeling" - Botsch & Kobbelt (2004)
7. "Singularity-Aware Motion Planning for Multi-Axis Additive Manufacturing" (RAL 2021)

## License

GNU General Public License v3 - See LICENSE file for details.

**Note:** The TetGen dependency (`tritet`) is AGPL licensed. Disable with `--no-default-features` if needed.
