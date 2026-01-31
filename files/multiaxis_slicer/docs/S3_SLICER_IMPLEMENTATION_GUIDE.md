# S3-Slicer Implementation Guide

This document catalogs the missing components from the original S3_DeformFDM implementation and provides a roadmap for implementation in Rust.

**Last Updated**: January 2026
**Based on**: S3_DeformFDM GitHub repository analysis

---

## Table of Contents
1. [Overview](#overview)
2. [Critical Missing Components](#critical-missing-components)
3. [Third-Party Dependencies](#third-party-dependencies)
4. [Implementation Phases](#implementation-phases)
5. [Rust Crate Mappings](#rust-crate-mappings)
6. [Current Status](#current-status)

---

## Overview

S3-Slicer (Support-free, Strength-reinforced, Surface-quality) is a multi-axis 3D printing framework that generates curved printing layers through rotation-driven deformation on tetrahedral meshes using quaternion field optimization.

### Three Fabrication Objectives:
1. **Support-Free (SL)**: Minimize overhangs to eliminate support structures
2. **Strength (SR)**: Align layers with principal stress directions
3. **Surface Quality (SQ)**: Preserve original geometry fidelity

---

## Critical Missing Components

### 🔴 CRITICAL (Must Have)

#### 1. ASAP Tetrahedral Deformation
- **File**: `ShapeLab/DeformTet.cpp` (345.8 KB)
- **What**: As-Rigid-As-Possible mesh deformation with SVD-based rotation optimization
- **Algorithm**:
  - Local phase: SVD rotation per tetrahedral element
  - Global phase: Sparse LDLT solve for vertex positions
  - Quaternion smoothing with weighted Laplacian
- **Current Status**: ❌ Missing (basic rotation exists but not ASAP)
- **Impact**: Core deformation quality

#### 2. Heat Method for Geodesic Distances
- **Files**: `ShapeLab/heatMethod.cpp` (17.6 KB), `heatmethodfield.cpp` (34.5 KB)
- **Algorithm** (3 stages):
  1. Heat diffusion: Solve `(I - t·M⁻¹·L)u = u₀`
  2. Vector field extraction: Compute normalized gradients
  3. Distance recovery: Solve Poisson equation from divergence
- **Current Status**: ❌ Missing (using simple Z-heights instead)
- **Impact**: Scalar field quality - this is why layers look incorrect

#### 3. Cotangent-Weighted Laplacian
- **Formula**: `weight = (edge_length × cot(dihedral_angle)) / 6.0`
- **Current Status**: ❌ Missing (simple neighbor averaging)
- **Impact**: Smoothing quality and field computation accuracy

#### 4. Tetrahedral Mesh Support
- **What**: Volumetric mesh data structure with tetrahedra
- **Current Status**: ❌ Missing (triangle surface mesh only)
- **Impact**: Cannot do proper volumetric deformation

### 🟡 HIGH Priority

#### 5. Adaptive Isosurface Extraction
- **File**: `ShapeLab/IsoLayerGeneration.cpp` (73.1 KB)
- **Algorithm**:
  - Binary search for optimal iso-values
  - Linear interpolation: `p = (1-α)·p1 + α·p2` where `α = (iso - val1)/(val2 - val1)`
  - PQP distance checking between layers
- **Current Status**: ⚠️ Partial (basic framework exists)

#### 6. Principal Stress Field
- **File**: `ShapeLab/PrincipleStressField.cpp` (18.2 KB)
- **What**: FEM stress analysis with eigenvalue decomposition
- **Current Status**: ❌ Missing

#### 7. Support Tree Generation
- **File**: `ShapeLab/SupportGeneration.cpp`
- **Algorithm**: Hierarchical tree with convex hull, ring neighbors
- **Current Status**: ⚠️ Partial (basic overhang detection)

#### 8. Surface Heat Method
- **File**: `ShapeLab/heatmethodfield.cpp`
- **What**: Boundary/zigzag kernel, PCA plane selection
- **Current Status**: ❌ Missing

---

## Third-Party Dependencies

### C++ Library → Rust Equivalent Mapping

| C++ Library | Purpose | Rust Crate | Status | Priority |
|-------------|---------|------------|--------|----------|
| **Eigen3** | Linear algebra, SVD, sparse solvers | `nalgebra` + `ndarray` | ✅ Available | CRITICAL |
| **PQP 1.3** | Collision detection, proximity queries | `parry3d` | ✅ Available | CRITICAL |
| **QHull 2020.2** | Convex hulls, Delaunay triangulation | `qhull` / `geo` | ✅ Available | HIGH |
| **GLUT/GLEW** | OpenGL rendering | `wgpu` / `three-d` (current) | ✅ In Use | MEDIUM |
| **Pardiso** | Sparse matrix solver (LDLT) | `nalgebra-sparse` | ⚠️ Partial | HIGH |
| **Intel MKL** | Math optimizations | Native Rust | ✅ N/A | LOW |

### Recommended Rust Crates

```toml
[dependencies]
# Linear Algebra
nalgebra = "0.32"
nalgebra-sparse = "0.9"
ndarray = "0.15"
ndarray-linalg = "0.16"

# Collision Detection
parry3d = "0.13"

# Geometry
geo = "0.27"  # For convex hulls
delaunator = "1.0"  # For Delaunay triangulation

# Numerics
sprs = "0.11"  # Sparse matrix operations
faer = "0.16"  # Fast linear algebra

# Visualization (already in use)
three-d = "0.16"
egui = "0.24"
```

---

## Implementation Phases

### Phase 1: Fix Scalar Field (Week 1)
**Goal**: Get proper geodesic distance-based scalar fields

1. **Implement Cotangent-Weighted Laplacian** ✓
   - Compute edge dihedral angles
   - Calculate cotangent weights
   - Build sparse Laplacian matrix
   - **Crate**: `nalgebra-sparse`

2. **Add Simplified Heat Method** ✓
   - Heat diffusion on surface mesh (skip tetrahedral for now)
   - Gradient computation on triangles
   - Poisson solve for distance recovery
   - **Crate**: `nalgebra-sparse` for solvers

3. **Fix Scalar Field Computation** ✓
   - Replace Z-heights with geodesic distances
   - Use heat method output
   - Smooth field with boundary conditions

### Phase 2: Improve Layer Extraction (Week 2)
**Goal**: Better isosurface quality

4. **Edge Interpolation for Iso-nodes** ✓
   - Linear interpolation on edges
   - Sign-change detection
   - Iso-node generation

5. **Adaptive Layer Spacing** ✓
   - Binary search for iso-values
   - Distance checking between layers
   - **Crate**: `parry3d` for distance queries

### Phase 3: Enhanced Deformation (Future)

6. **Tetrahedral Mesh Support**
   - Tetrahedralization of surface mesh
   - **Crate**: Consider `tetgen` or custom implementation

7. **ASAP Solver**
   - SVD-based rotation extraction
   - Sparse system solve
   - **Crate**: `nalgebra` for SVD, `nalgebra-sparse` for solver

---

## Rust Crate Mappings

### Linear Algebra (Eigen3 → Rust)

```rust
// Dense matrices and vectors
use nalgebra::{Matrix3, Vector3, DMatrix, DVector};

// Sparse matrices
use nalgebra_sparse::CsrMatrix;
use sprs::CsMat;

// SVD decomposition
use nalgebra::SVD;

// Eigenvalue decomposition
use nalgebra::SymmetricEigen;
```

### Collision Detection (PQP → Rust)

```rust
// Parry3d for proximity queries
use parry3d::query::{ClosestPoints, Distance};
use parry3d::shape::TriMesh;
use parry3d::bounding_volume::AABB;

// Example usage
let trimesh = TriMesh::new(vertices, indices);
let distance = parry3d::query::distance(
    &pos1, &trimesh1,
    &pos2, &trimesh2,
)?;
```

### Convex Hull (QHull → Rust)

```rust
// Using geo crate
use geo::{ConvexHull, Point};

let points: Vec<Point<f64>> = vec![...];
let hull = ConvexHull::convex_hull(&points);
```

---

## Current Status

### What We Have ✅
- Basic mesh I/O (STL format)
- Simple height-based scalar field
- Basic Laplacian smoothing (neighbor averaging)
- Triangle mesh data structure
- Basic quaternion field optimization
- Zigzag toolpath pattern
- Dijkstra's shortest path
- Basic collision detection framework
- Scale-controlled deformation (Laplacian-based)

### What We've Implemented ✅

**Phase 1 (Complete):**
- [x] Cotangent-weighted Laplacian
- [x] Heat method on surface mesh
- [x] Geodesic distance scalar field

**Phase 2 (Complete):**
- [x] Edge interpolation for isosurfaces (already present)
- [x] Adaptive layer spacing with binary search
- [x] Geometric distance-based layer thickness control

**Phase 3 (Complete):**
- [x] ASAP deformation solver architecture
- [x] SVD-based rotation extraction
- [x] Sparse linear system solver
- [x] Cotangent weight computation
- [x] Iterative local-global optimization

### What's Next (Future Enhancements) 📋
- [ ] Pipeline integration for ASAP (optional)
- [ ] Tetrahedral mesh support (volumetric)
- [ ] Principal stress field analysis
- [ ] Advanced support tree generation
- [ ] Hybrid toolpath strategies
- [ ] Performance optimizations (better solver, parallelization)

---

## Algorithm Complexity Summary

| Algorithm | Time | Space | Implementation |
|-----------|------|-------|----------------|
| Cotangent Laplacian | O(E) | O(E) | Phase 1 |
| Heat Diffusion | O(n log n) | O(E) | Phase 1 |
| Vector Field Gradient | O(T) | O(T) | Phase 1 |
| Poisson Recovery | O(n log n) | O(E) | Phase 1 |
| Edge Interpolation | O(E) | O(n) | Phase 1 |
| Adaptive Spacing | O(n·log n) | O(n) | Phase 1 |
| ASAP Deformation | O(n log n) | O(E) | Phase 3 |
| Tetrahedral Mesh | O(n log n) | O(V+T) | Phase 3 |

Legend:
- E = number of edges
- T = number of triangles/tetrahedra
- V = number of vertices
- n = mesh size (vertices or elements)

---

## Key References

### Papers
- [S3: A General-Purpose Slicing Framework for Multi-Axis 3D Printing](https://dl.acm.org/doi/10.1145/3550454.3555458) (SIGGRAPH Asia 2022)
- [The Heat Method for Distance Computation](https://www.cs.cmu.edu/~kmcrane/Projects/HeatMethod/) (Crane et al.)

### Repositories
- [S3_DeformFDM](https://github.com/Spiritdude/S3_DeformFDM) - Original C++ implementation
- [S4_Slicer](https://github.com/jyjblrd/S4_Slicer) - Simplified variant

### Documentation
- [Parry3d Documentation](https://docs.rs/parry3d/)
- [Nalgebra Documentation](https://docs.rs/nalgebra/)
- [Geo Documentation](https://docs.rs/geo/)

---

## Notes

### Design Decisions

1. **Surface vs. Volumetric**: Starting with surface mesh heat method before tetrahedral mesh
2. **Solver Choice**: Using `nalgebra-sparse` for linear system solving (native Rust)
3. **Collision**: Using `parry3d` instead of porting PQP
4. **Convex Hull**: Using `geo` crate for simplicity

### Known Limitations

1. Surface-only heat method is less accurate than volumetric
2. No stress field optimization yet
3. No multi-objective optimization (only single objective at a time)
4. Simplified support generation

### Future Improvements

1. Add tetrahedral mesh support for volumetric deformation
2. Implement ASAP solver for better deformation quality
3. Add stress field computation for strength optimization
4. Implement hybrid toolpath strategies
5. Add dual-solution IK for 5-axis machining

---

## Contact & Contribution

For questions or contributions regarding this implementation, refer to the main project documentation.

**Project**: Non-planar Multi-Axis Slicer
**Language**: Rust
**Target**: S3-Slicer algorithm implementation
