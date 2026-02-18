# 5-Axis Slicer Implementation Roadmap

## Status: ✅ ALL MAJOR FEATURES COMPLETE
Date: 2026-01-24
Last Updated: 2026-01-24 (S3-Slicer Curved Layer Slicing Complete)

---

## 🎉 Completion Summary

Both Phase A (Support Generation) and Phase B (S3-Slicer Curved Layers) are now **FULLY IMPLEMENTED** and integrated into the GUI!

### ✅ Completed (January 24, 2026)

**Phase A - Support Generation:**
1. ✅ Overhang Detection - Angle-based detection with clustering
2. ✅ Tree Skeleton Generation - Branch merging and optimization
3. ✅ Support Visualization - 3D rendering with color-coded nodes
4. ✅ Support Toolpath Generation - Circular perimeters + zigzag infill
5. ✅ GUI Integration - Full configuration and generation workflow

**Phase B - S3-Slicer Curved Layers:**
1. ✅ Scalar Field Computation - Height, Geodesic, Deformation modes
2. ✅ Model Deformation Pipeline - Rotation-based field deformation
3. ✅ Isosurface Extraction - Marching triangles algorithm
4. ✅ Curved Layer Generation - Complete workflow from field to layers
5. ✅ GUI Integration - Slicing mode selector (Planar vs Curved)

### 🎯 Feature Status Summary

#### ✅ Phase A Complete (Support Generation)
- Overhang detection ✅
- Tree skeleton generation ✅
- Support visualization ✅
- Support toolpath generation ✅
- GUI integration ✅

#### ✅ Phase B Complete (S3-Slicer)
- Scalar field computation ✅
- Model deformation pipeline ✅
- Isosurface extraction ✅
- Curved layer generation ✅
- GUI integration ✅
- Slicing mode selection ✅

---

## Phase A: Support Generation (SIGGRAPH Asia 2022)

### Overview
Implementing automatic support generation for curved/non-planar printing based on the paper:
"Support Generation for Curved RoboFDM" by Tianyu Zhang et al.

### Goals
- ✅ Detect overhangs that need support
- ✅ Generate minimal tree-based support structures
- ⏳ Create toolpaths for support material
- ⏳ Integrate with multi-axis motion planning

### Components

#### 1. Overhang Detection ✅ COMPLETED
**File:** `src/support_generation/overhang_detection.rs`

**Algorithm:**
- Analyze each triangle's normal vector
- If angle with build direction > threshold (typically 45°), mark as overhang
- Account for curved layers from centroidal axis
- Build overhang face map

**Status:** ✅ Complete
**Input:** Mesh triangles, build direction per layer
**Output:** List of overhang faces with severity scores

**Implementation Details:**
- `OverhangConfig` struct with configurable angle threshold
- `detect_overhangs()` function analyzing mesh triangles
- `cluster_overhangs()` for grouping nearby overhangs
- Severity scoring based on overhang angle

#### 2. Tree Skeleton Generation ✅ COMPLETED
**File:** `src/support_generation/tree_skeleton.rs`

**Algorithm:**
- Start from overhang regions
- Trace paths downward to build platform
- Merge nearby branches to minimize material
- Create tree structure with nodes and edges
- Optimize for minimal footprint

**Status:** ✅ Complete
**Input:** Overhang faces, mesh geometry
**Output:** Support skeleton (nodes + connections)

**Implementation Details:**
- `SupportNode` and `SupportTree` data structures
- `generate_tree_skeleton()` orchestrates generation
- `grow_branch_to_platform()` creates individual support columns
- `cluster_by_proximity()` groups nearby overhangs
- `merge_close_branches()` optimizes tree efficiency
- Configurable radius (thicker at bottom, thinner at contact points)

#### 5. GUI Integration ✅ COMPLETED
**Files:** `src/gui/app.rs`, `src/gui/control_panel.rs`

**Features:**
- Support configuration panel with:
  - Overhang angle slider (0-89°)
  - Minimum area threshold
  - Curved layer analysis toggle
- "Generate Supports" button in Actions section
- Show/hide supports checkbox in Visualization section
- Statistics display (overhang faces, area, support nodes, contact points)

**Status:** ✅ Complete

#### 3. Support Mesh Generation ⏳ PENDING
**File:** `src/support_generation/support_mesh.rs`

**Algorithm:**
- Thicken tree skeleton into volumetric structure
- Create slim support walls (0.8-1.2mm typical)
- Add connection points to model
- Generate break-away interfaces

**Status:** 📋 Planned
**Input:** Tree skeleton
**Output:** Support mesh (triangles)

#### 4. Support Toolpaths ✅ COMPLETED
**File:** `src/support_generation/support_toolpath.rs`

**Algorithm:**
- Group support nodes by layer height
- Generate circular perimeters around each support column
- Generate zigzag infill (configurable density, default 15%)
- Calculate extrusion based on volume
- Integrate with main model toolpaths

**Status:** ✅ Complete
**Input:** Support tree skeleton, toolpath configuration
**Output:** Toolpath segments per layer

**Implementation Details:**
- `SupportToolpathConfig` for configuration (layer height, infill density, etc.)
- `generate_support_toolpaths()` orchestrates generation
- `generate_circle_points()` creates circular perimeters
- `generate_support_infill()` creates zigzag infill pattern
- Volume-based extrusion calculation matching main toolpath algorithm

#### 6. Support Visualization ✅ COMPLETED
**File:** `src/gui/viewport_3d.rs`

**Features:**
- Render support tree skeleton as connected lines
- Orange color for support branches (thickness based on radius)
- Red circles at contact points (model interface)
- Blue circles at root nodes (platform contact)
- Display support statistics (branches, contact points)
- Toggle visibility with "Show supports" checkbox

**Status:** ✅ Complete

---

## Phase B: S3-Slicer Curved Layers (SIGGRAPH Asia 2022 Best Paper)

### Overview
Implementing true curved layer slicing through deformation-based methods.
Paper: "S3-Slicer: A General Slicing Framework for Multi-Axis 3D Printing"

### Goals
- Generate curved layers that follow object geometry
- Optimize for support-free, strength, or surface quality
- Replace planar slicing with curved isosurfaces
- Maintain compatibility with existing toolpath generation

### Components

#### 1. Scalar Field Computation ✅ COMPLETED
**File:** `src/s3_slicer/scalar_field.rs`

**Algorithm:**
- Compute height-based, geodesic, or deformation-based scalar fields
- Apply Laplacian smoothing for regularization
- Support multiple field types (Height, Geodesic, Deformation, Custom)
- Normalize field values to standard range

**Mathematical Method:**
```
f(v) = deform(height(v), surface_normal(v), build_direction)
```

**Status:** ✅ Complete
**Input:** Mesh triangles, build direction, configuration
**Output:** Scalar value per triangle/vertex

**Implementation Details:**
- `ScalarField` struct with field values and range
- `FieldType` enum for different field computation methods
- `compute_height_field()` - simple Z-coordinate based
- `compute_geodesic_field()` - distance-based (placeholder for Dijkstra)
- `compute_deformation_field()` - alignment-based deformation
- `smooth_field()` - Laplacian smoothing with adjacency graph
- `build_triangle_adjacency()` - mesh connectivity analysis
- Barycentric interpolation support for field queries

**Challenges Addressed:**
- ✅ Smooth field interpolation via Laplacian smoothing
- ✅ Triangle adjacency computation for mesh connectivity
- ⏳ Full geodesic distance (currently using height approximation)
- ⏳ Vertex-based field (currently triangle-based)

#### 2. Model Deformation Pipeline ✅ COMPLETED
**File:** `src/s3_slicer/deformation.rs`

**Algorithm:**
- Apply rotation-driven deformation based on surface normals
- Map original coordinates to deformed space using quaternions
- Optional volume preservation with scaling
- Compute Jacobian determinant for layer thickness adjustment

**Status:** ✅ Complete
**Input:** Original mesh, scalar field, deformation configuration
**Output:** Deformed mesh, transformation map, Jacobian values

**Implementation Details:**
- `DeformationConfig` with rotation factor, volume preservation, build direction
- `DeformedMesh` struct containing deformed geometry and transformation data
- `DeformationTransform` tracking original→deformed mapping per vertex
- `deform_mesh()` - main deformation function using rotation fields
- `compute_rotation_field()` - computes rotation based on surface alignment
- `apply_deformation()` - applies rotation and scaling to individual points
- `compute_jacobian_determinant()` - calculates volume change ratio
- `inverse_deform_point()` - maps from deformed space back to original
- `adjusted_layer_thickness()` - compensates layer height for deformation

**Technical Notes:**
- Uses nalgebra UnitQuaternion for rotation
- Rotation axis perpendicular to normal and build direction
- Scaling factor inversely proportional to rotation for volume preservation
- Jacobian approximated from triangle area ratios
- Invertible transformation

#### 3. Isosurface Extraction ✅ COMPLETED
**File:** `src/s3_slicer/isosurface.rs`

**Algorithm:**
- Extract level sets of scalar field using marching triangles
- Generate curved layer contours from isosurfaces
- Chain contour segments into continuous paths
- Simplify contours using Douglas-Peucker algorithm

**Mathematical:**
```
Layer_i = {v | f(v) = h_i}
```
where h_i are evenly spaced isovalue heights

**Status:** ✅ Complete
**Input:** Mesh with scalar field, isosurface configuration
**Output:** Curved layer contours as connected segments

**Implementation Details:**
- `IsosurfaceExtractor` main extraction class
- `CurvedLayer` struct with segments, iso-value, and normals
- `extract_at_value()` - extracts single isosurface at specific value
- `extract_layers()` - generates multiple layers uniformly distributed
- `extract_triangle_contour()` - marching triangles per-triangle extraction
- `interpolate_edge()` - linear interpolation for edge crossings
- `chain_segments()` - connects segments into continuous paths
- `simplify_contour()` - Douglas-Peucker line simplification
- `to_layer()` - converts CurvedLayer to standard Layer format

**Challenges Addressed:**
- ✅ Efficient isosurface computation via marching triangles
- ✅ Segment chaining for continuous contours
- ✅ Contour simplification for performance
- ✅ Conversion to existing Layer format for compatibility

#### 4. Curved Layer Integration ✅ COMPLETED
**Files:** `src/gui/app.rs`, `src/gui/control_panel.rs`

**Algorithm:**
- Complete pipeline from mesh to curved layers
- Scalar field computation → isosurface extraction → layer conversion
- Background thread processing for responsiveness
- Compatible with existing toolpath generation

**Status:** ✅ Complete
**Input:** Mesh, slicing configuration, slicing mode
**Output:** Layer objects compatible with existing pipeline

**Implementation Details:**
- `SlicingMode` enum (Planar, Curved) in app state
- `start_curved_slicing()` - background thread for S3-Slicer pipeline
- Three-step process:
  1. Compute deformation-based scalar field
  2. Extract isosurface layers
  3. Convert to standard Layer format
- Progress tracking throughout process
- GUI selector for Planar vs Curved slicing mode
- Full integration with existing toolpath/G-code workflow

**GUI Features:**
- Slicing mode dropdown in "Slicing Parameters" section
- Options: "Planar (Traditional)" and "Curved (S3-Slicer)"
- Progress messages for each step of curved slicing
- Seamless integration with existing workflow

---

## Current Architecture

### Module Structure
```
multiaxis_slicer/
├── src/
│   ├── geometry.rs              ✅ Core geometry types
│   ├── mesh.rs                  ✅ STL loading
│   ├── slicing.rs               ✅ Planar slicing (current)
│   ├── toolpath.rs              ✅ Path generation
│   ├── toolpath_patterns.rs    ✅ Spiral/Zigzag/Contour
│   ├── motion_planning/         ✅ 6-step pipeline
│   ├── support_generation/      🔄 IN PROGRESS
│   │   ├── mod.rs
│   │   ├── overhang_detection.rs
│   │   ├── tree_skeleton.rs
│   │   └── support_toolpath.rs
│   └── s3_slicer/               📋 PLANNED
│       ├── mod.rs
│       ├── scalar_field.rs
│       ├── deformation.rs
│       ├── isosurface.rs
│       └── curved_layers.rs
```

---

## Implementation Timeline

### Week 1: Support Generation (Current)
- [x] Research and planning
- [ ] Overhang detection (2-3 hours)
- [ ] Tree skeleton generation (3-4 hours)
- [ ] Support mesh creation (2-3 hours)
- [ ] Support toolpaths (1-2 hours)
- [ ] GUI integration (1 hour)
- [ ] Testing with real models (2 hours)

**Total:** ~12-15 hours

### Week 2-3: S3-Slicer Foundation
- [ ] Scalar field computation (4-6 hours)
- [ ] Basic deformation (3-4 hours)
- [ ] Isosurface extraction (4-5 hours)
- [ ] Integration with existing slicing (3-4 hours)
- [ ] Testing and refinement (4-6 hours)

**Total:** ~18-25 hours

### Week 4: Optimization & Polish
- [ ] Quaternion field optimization
- [ ] Performance improvements
- [ ] Advanced deformation modes
- [ ] User-friendly presets
- [ ] Documentation

---

## Technical Challenges

### Support Generation
| Challenge | Severity | Solution Approach |
|-----------|----------|-------------------|
| Overhang detection in curved layers | Medium | Use centroidal axis for local build direction |
| Tree merging algorithm | Medium | Hierarchical clustering + A* pathfinding |
| Support-model interface | Low | Contact point detection + offset |
| Integration with motion planning | Medium | Treat supports as separate toolpath sequences |

### S3-Slicer
| Challenge | Severity | Solution Approach |
|-----------|----------|-------------------|
| Scalar field computation | High | Incremental field propagation |
| Deformation without self-intersection | High | Constrained optimization |
| Isosurface extraction performance | Medium | Spatial hashing + parallel marching cubes |
| Curved layer orientation field | High | Gradient-based normal computation |
| Maintaining 5-axis feasibility | Very High | Continuous IK solution verification |

---

## Dependencies

### New Rust Crates Needed

```toml
[dependencies]
# Existing
nalgebra = { version = "0.32", features = ["serde-serialize"] }
rayon = "1.7"                    # Parallel processing
parry3d = "0.13"                 # Collision detection

# New for support generation
petgraph = "0.6"                 # Graph algorithms for tree skeleton

# New for S3-Slicer
ndarray = "0.15"                 # N-dimensional arrays for scalar fields
sprs = "0.11"                    # Sparse matrices for field solving
faer = "0.13"                    # Fast linear algebra
```

---

## Testing Strategy

### Support Generation Tests
1. **Simple overhang** - 45° angled surface
2. **Complex geometry** - Multiple disconnected overhangs
3. **Minimal supports** - Verify tree merging works
4. **Curved layer supports** - Non-planar base surfaces

### S3-Slicer Tests
1. **Smooth scalar field** - Verify no discontinuities
2. **Volume preservation** - Check deformation is valid
3. **Isosurface quality** - Smooth layer surfaces
4. **Support-free result** - Validate goal achievement
5. **5-axis feasibility** - All orientations reachable

---

## Documentation TODOs

- [ ] Algorithm pseudocode for each module
- [ ] Mathematical derivations for S3
- [ ] Parameter tuning guide
- [ ] Example workflows
- [ ] Performance benchmarks
- [ ] Comparison with traditional slicing

---

## Future Enhancements (Post-Implementation)

### Priority 1
- Adaptive support density based on load
- Support interface layers for easier removal
- Multi-material support (soluble)

### Priority 2
- Strength-optimized S3 mode
- Surface-quality-optimized S3 mode
- Hybrid planar/curved slicing

### Priority 3
- Support optimization via topology optimization
- Machine learning for optimal deformation
- Real-time preview of curved layers

---

---

## Servo Firmware & Advanced G-code (Future)

> **Context:** The slicer targets a custom servo-driven machine (not stepper-based).
> Servo motors have fundamentally different motion characteristics — they can execute
> true arc interpolation and jerk-limited profiling much more accurately than steppers,
> making the following G-code features actually worth implementing (unlike on typical
> FDM printers where firmware support is spotty).

### G5 / G6 — Cubic B-Spline / Circular Interpolation
- **G5 IJK** — cubic spline move (Marlin 2.x, LinuxCNC)
- **G6.1 Q1** — NURBS feedforward (Fanuc/Siemens CNC)
- **Why servo makes this viable:** Servos close the position loop fast enough to track
  smooth curves without losing steps; steppers cannot.
- **Implementation plan:**
  1. Add a `GCodeDialect` enum to `gcode.rs`: `Marlin`, `Klipper`, `LinuxCNC`, `Fanuc`
  2. In the G-code generator, detect sequences of collinear-ish segments and fit arcs
     (radius-of-curvature > configurable threshold)
  3. Emit `G2`/`G3` (arc) or `G5` (spline) instead of a chain of `G1` moves
  4. Fall back to `G1` subdivision if the controller dialect doesn't support it

### G64 — Path Blending / Continuous Mode
- **G64 P<tolerance>** (LinuxCNC) / **G64** (Fanuc) — allows the controller to blend
  path segments without coming to a full stop at each corner
- Reduces print time and eliminates the "dot" artefact at segment junctions
- **Implementation plan:**
  - Emit `G64 P0.05` in the G-code header when `dialect = LinuxCNC`
  - Add a `path_blend_tolerance` field to `MachineConfig` (default `0.0` = disabled)
  - Document that this requires the servo firmware to support look-ahead planning

### Jerk-Limited Velocity Profiling (S-Curve)
- Servo drives can execute S-curve acceleration profiles natively
- Slicer side: pre-compute segment speeds so the firmware doesn't have to
- Long-term: generate `M566` (Duet) or equivalent jerk limits per-segment

### Relevant Files to Modify
| File | Change |
|------|--------|
| `src/gcode.rs` | Add `GCodeDialect`, arc/spline emission, G64 header |
| `src/gui/app.rs` | Add `gcode_dialect` field to app state |
| `src/gui/control_panel.rs` | Dialect selector dropdown |
| `src/motion_planning/mod.rs` | Feed arc-fitting results into G-code stage |

---

## Notes

**Current Status:** Core slicing methods complete (Planar, Conical, S3, S4, Geodesic).
Active work: slicing quality fixes, section view, playback visualization.

**Servo G-code features** are deferred until the servo firmware project reaches a stage
where these G-codes are supported — at that point, `gcode.rs` is the primary entry point.

**Last Updated:** 2026-02-18
