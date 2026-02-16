# MultiAxis Slicer

High-performance multi-axis non-planar 3D printing slicer written in Rust with egui GUI.

## Features

### Slicing Modes
- **Planar (Traditional)** - Fast flat-layer slicing at constant Z height
- **Conical (RotBot)** - Cone-based Z-shift for radially symmetric overhangs
- **S4 Non-Planar** - Deform mesh via Dijkstra distance field, planar slice, un-deform toolpaths
- **S3 Curved Layer** - Full quaternion field optimization with multiple deformation methods (TetVolumetric, ASAP, Virtual Scalar Field)
- **Geodesic (Heat Method)** - Layers follow surface curvature via geodesic distance field (Crane et al. 2013)

### Core Capabilities
- Fast mesh slicing - O(n log k + k + m) algorithm
- 5-axis toolpath generation with tool orientation
- G-code generation for multi-axis CNC/3D printers
- Wall loops (perimeters) and rectilinear infill
- Centroidal axis computation (100x faster than medial axis)
- Interactive egui GUI with 3D viewport
- Parallel processing with Rayon

### Non-Planar Pipeline (S3/S4)
- Volumetric tetrahedral mesh generation (TetGen or grid-based)
- Per-tet quaternion field optimization
- ASAP deformation with SVD-based scaling
- Voxel reconstruction for self-intersecting STL files (SDF + Marching Cubes)
- Barycentric untransform for original mesh topology preservation

### Geodesic Slicing (NEW)
- Heat Method distance field computation on mesh surface
- Cotangent Laplacian + Conjugate Gradient sparse solver
- Two source modes: bottom boundary (auto-detect) or point source
- Marching triangles level-set extraction
- Layers naturally conform to surface curvature

## Quick Start

### Prerequisites

```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Build & Run

```bash
cd files/multiaxis_slicer

# Build release version
cargo build --release

# Run the GUI application
cargo run --release

# Run tests (skip examples with pre-existing compile issues)
cargo test --lib
```

## Architecture

```
multiaxis_slicer/src/
├── lib.rs                  # Module declarations and re-exports
├── geometry.rs             # Point3D, Vector3D, Triangle, Contour, Plane
├── mesh.rs                 # Mesh struct, STL I/O
├── slicing.rs              # Planar slicer (Layer, Contour extraction)
├── toolpath.rs             # ToolpathGenerator (walls, infill, extrusion)
├── toolpath_patterns.rs    # Spiral, Zigzag, Contour, infill patterns
├── contour_offset.rs       # 2D polygon offset for wall loops
├── geodesic.rs             # Geodesic slicing (Heat Method + level sets)
├── conical.rs              # Conical slicing pipeline
├── gcode.rs                # G-code output
├── centroidal_axis.rs      # Centroidal axis computation
├── collision.rs            # Collision detection
├── ruled_surface.rs        # Ruled surface detection
├── singularity.rs          # Singularity optimization
├── motion_planning.rs      # Path planning
├── support_generation/     # Support structure generation
│   ├── mod.rs
│   ├── overhang_detection.rs
│   └── tree_skeleton.rs
├── s3_slicer/              # S3/S4 non-planar pipeline
│   ├── mod.rs
│   ├── pipeline.rs         # Main pipelines (S3, S4, Tet)
│   ├── quaternion_field.rs # Per-triangle quaternion optimization
│   ├── tet_mesh.rs         # TetMesh (TetGen + grid-based)
│   ├── tet_quaternion_field.rs
│   ├── tet_asap_deformation.rs
│   ├── tet_scalar_field.rs
│   ├── tet_dijkstra_field.rs
│   ├── s4_rotation_field.rs
│   ├── tet_point_location.rs
│   ├── marching_tet.rs
│   ├── voxel_remesh.rs     # SDF + Marching Cubes mesh repair
│   └── ...
└── gui/
    ├── app.rs              # SlicerApp state, slicing dispatch
    ├── control_panel.rs    # UI for all slicing modes
    ├── viewport_3d.rs      # 3D visualization
    └── theme.rs            # Visual theme
```

## Slicing Pipeline Overview

```
Load STL → Select Mode → Mode-specific pipeline → Vec<Layer> → Toolpath → G-code

Planar:    Flat planes at constant Z
Conical:   Z-shift by r·tan(angle) → Planar slice → Reverse shift
S4:        TetMesh → Dijkstra → Rotations → Deform → Slice → Untransform
S3:        Quaternion field → Deformation method → Isosurface extraction
Geodesic:  Build topology → Cotangent Laplacian → Heat Method → Level sets
```

## Testing

```bash
# Run library tests (93 pass, 3 pre-existing failures)
cargo test --lib

# Run specific module tests
cargo test --lib geodesic
cargo test --lib contour_offset
cargo test --lib toolpath
```

## Dependencies

- **nalgebra** 0.32 - Linear algebra (vectors, matrices, quaternions)
- **nalgebra-sparse** 0.9 - Sparse matrix support
- **egui/eframe** - Immediate-mode GUI framework
- **three-d** - 3D rendering
- **rayon** 1.8 - Parallel processing
- **sprs** 0.11 - Sparse matrices
- **tritet** 3.1 (optional) - TetGen integration (AGPL, `tetgen` feature flag)

## References

1. "An Optimal Algorithm for 3D Triangle Mesh Slicing"
2. "Support-Free Volume Printing by Multi-Axis Motion" (TOG 2020) - S3-Slicer
3. "S4-Slicer" (jyjblrd) - Deform/slice/untransform approach
4. "The Heat Method for Distance Computation" (Crane et al. 2013) - Geodesic slicing
5. "Singularity-Aware Motion Planning for Multi-Axis Additive Manufacturing" (RAL 2021)

## License

GNU General Public License v3 - See LICENSE file for details
