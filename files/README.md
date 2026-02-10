# MultiAxis Slicer

High-performance 5-axis non-planar slicer written in Rust, featuring curved-layer slicing based on the S3-Slicer algorithm (SIGGRAPH Asia 2022 Best Paper).

## Features

- **S3-Slicer curved-layer pipeline** - Full implementation of the "S3-Slicer: A General Slicing Framework for Multi-Axis 3D Printing" paper
- **Tetrahedral volumetric deformation** - Per-tetrahedron ASAP (As-Similar-As-Possible) deformation with scaling, matching the original paper's approach
- **Voxel-based mesh reconstruction** - SDF + Marching Cubes preprocessing that guarantees clean, manifold input for TetGen, even from self-intersecting STL files
- **Interactive GUI** - egui-based application with 3D viewport, real-time mesh preview, and parameter controls
- **Fast mesh slicing** - O(n log k + k + m) algorithm from optimal slicing paper
- **Centroidal axis computation** - 100x faster than medial axis
- **Parallel processing** - Multi-threaded computation with Rayon
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

## S3-Slicer Pipeline

The core of this project is a full implementation of the S3-Slicer algorithm for curved-layer non-planar 3D printing. The pipeline supports multiple deformation methods:

### Tet Volumetric (Recommended)

The full volumetric pipeline from the original paper:

1. **Mesh preprocessing** - Voxel reconstruction (SDF + Marching Cubes) produces a clean, manifold surface from potentially self-intersecting STL files
2. **Tetrahedralization** - TetGen (via `tritet` crate) generates a constrained Delaunay tet mesh
3. **Quaternion field optimization** - Per-tetrahedron rotation field optimized for fabrication objectives (support-free, strength, etc.)
4. **Volumetric ASAP deformation** - Per-tet As-Similar-As-Possible deformation with scaling via SVD of deformation gradients
5. **Scalar field computation** - Volume-based scalar field with Laplacian smoothing
6. **Marching tetrahedra** - Isosurface extraction produces curved layers

### Other Methods

- **Virtual Scalar Field** - Computes scalar field directly without mesh deformation. Good fallback for complex models.
- **ASAP Deformation** - Surface-based ASAP solver. Can cause mesh collapse on complex geometry.
- **Scale-Controlled** - Local deformation with scale control.

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
│   ├── toolpath.rs               # Toolpath generation
│   ├── toolpath_patterns.rs      # Infill patterns (linear, concentric, etc.)
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
# Run all library tests (skips examples that have known compile issues)
cargo test --lib

# Run specific test module
cargo test --lib -- voxel_remesh
cargo test --lib -- tet_mesh
cargo test --lib -- isotropic_remesh

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
2. "An Optimal Algorithm for 3D Triangle Mesh Slicing"
3. "Support-Free Volume Printing by Multi-Axis Motion" (TOG 2020)
4. "A Remeshing Approach to Multiresolution Modeling" - Botsch & Kobbelt (2004)
5. "Singularity-Aware Motion Planning for Multi-Axis Additive Manufacturing" (RAL 2021)

## License

MIT License - See LICENSE file for details.

**Note:** The TetGen dependency (`tritet`) is AGPL licensed. If distributing binaries, ensure compliance or build with `--no-default-features` to exclude it.
