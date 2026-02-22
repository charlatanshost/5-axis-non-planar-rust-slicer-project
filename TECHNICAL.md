# Technical Reference — 5-Axis Non-Planar Slicer

This document covers every algorithm, module, and data structure in the project in depth. It is intended for contributors, researchers, or anyone who wants to understand how the code actually works.

---

## Table of Contents

1. [High-Level Architecture](#1-high-level-architecture)
2. [Core Data Structures](#2-core-data-structures)
3. [Slicing Modes — How Each Works](#3-slicing-modes)
   - 3.1 [Planar Slicing](#31-planar-slicing)
   - 3.2 [Conical Slicing](#32-conical-slicing)
   - 3.3 [S4 Non-Planar (Dijkstra Deform)](#33-s4-non-planar)
   - 3.4 [S3 Curved Layer (Tet Volumetric)](#34-s3-curved-layer)
   - 3.5 [Geodesic Slicing (Heat Method)](#35-geodesic-slicing)
   - 3.6 [Cylindrical Slicing](#36-cylindrical-slicing)
   - 3.7 [Spherical Slicing](#37-spherical-slicing)
4. [Mesh Repair — Voxel Reconstruction](#4-mesh-repair--voxel-reconstruction)
5. [Tetrahedral Meshing](#5-tetrahedral-meshing)
6. [Geodesic Diffusion Modes](#6-geodesic-diffusion-modes)
7. [Toolpath Generation](#7-toolpath-generation)
8. [5-Axis G-Code and TCP Compensation](#8-5-axis-g-code-and-tcp-compensation)
9. [Collision Detection](#9-collision-detection)
10. [Support Generation](#10-support-generation)
11. [GUI Architecture](#11-gui-architecture)
12. [Module Reference](#12-module-reference)
13. [Test Coverage](#13-test-coverage)
14. [Performance Notes](#14-performance-notes)
15. [Known Limitations](#15-known-limitations)

---

## 1. High-Level Architecture

```
STL file
   │
   ▼
mesh.rs ─────── Mesh (triangle soup, no topology)
   │
   ├─► Planar ──────────────────────────────────────────► slicing.rs → Vec<Layer>
   │
   ├─► Conical ─────────── conical.rs (Z-shift)  ──────► slicing.rs → Vec<Layer>
   │
   ├─► Cylindrical ─────── coordinate_transform.rs ────► slicing.rs → Vec<Layer>
   │
   ├─► Spherical ───────── coordinate_transform.rs ────► slicing.rs → Vec<Layer>
   │
   ├─► S4 Non-Planar ───── s3_slicer/pipeline.rs
   │       │  TetMesh (grid)
   │       │  TetDijkstraField
   │       │  S4RotationField
   │       │  TetAsapSolver
   │       │  Slicer (planar)
   │       └─ TetPointLocation (barycentric untransform) ► Vec<Layer>
   │
   ├─► S3 Curved Layer ─── s3_slicer/pipeline.rs
   │       │  voxel_remesh (repair)
   │       │  TetMesh (TetGen or grid)
   │       │  TetQuaternionField
   │       │  TetAsapSolver
   │       │  TetScalarField
   │       └─ MarchingTet ──────────────────────────────► Vec<Layer>
   │
   └─► Geodesic ─────────── geodesic.rs
           │  MeshTopology (weld + adjacency)
           │  CotangentLaplacian (FEM)
           │  HeatMethod (CG solver × N scales)
           └─ MarchingTriangles (level sets) ──────────► Vec<Layer>
                                                              │
                                                              ▼
                                                    toolpath.rs → Toolpath
                                                              │
                                                              ▼
                                                    gcode.rs → G-code string
```

Everything runs on a background thread spawned from `gui/app.rs`. Results are passed back via a `crossbeam` channel and rendered in the viewport.

---

## 2. Core Data Structures

### `Mesh` (`mesh.rs`)
A flat triangle soup — no topological connectivity. Every triangle stores its three vertices directly. This is what you get from an STL file.

```rust
pub struct Mesh {
    pub triangles: Vec<Triangle>,
    pub bounding_box: BoundingBox,
}

pub struct Triangle {
    pub v0, v1, v2: Point3D,
    pub normal: Vector3D,
}
```

No shared vertices, no edge lists. Algorithms that need topology (geodesic, S3/S4) build their own connectivity data on top.

### `Layer` (`slicing.rs`)
The universal output of every slicing pipeline — a collection of closed contours at a nominal Z height.

```rust
pub struct Layer {
    pub contours: Vec<Contour>,
    pub z_height: f64,
}

pub struct Contour {
    pub points: Vec<Point3D>,    // 3D points (Z varies for curved layers)
    pub is_closed: bool,
}
```

### `MeshTopology` (`geodesic.rs`)
Built once from a `Mesh` by welding vertices (quantized to 1e-6 resolution via `HashMap`) and computing adjacency. This is the connectivity layer used by the geodesic solver.

```rust
pub struct MeshTopology {
    pub vertices: Vec<Vector3D>,          // welded vertex positions
    pub faces: Vec<[usize; 3]>,           // triangle vertex indices
    pub vertex_faces: Vec<Vec<usize>>,    // which faces share each vertex
    pub vertex_normals: Vec<Vector3D>,    // area-weighted normals
}
```

### `TetMesh` (`s3_slicer/tet_mesh.rs`)
Volumetric tetrahedral mesh. Each tetrahedron is 4 vertex indices.

```rust
pub struct TetMesh {
    pub vertices: Vec<Vector3D>,
    pub tets: Vec<[usize; 4]>,
    pub tet_neighbors: Vec<[Option<usize>; 4]>,  // face-adjacent tets
}
```

Face `i` of a tetrahedron is the face opposite vertex `i` — this winding convention is used throughout the quaternion field and ASAP solver.

---

## 3. Slicing Modes

### 3.1 Planar Slicing

**File:** `slicing.rs`

The reference baseline. Intersects the mesh with horizontal planes at uniform Z spacing.

**Algorithm** (based on Minetto et al. 2017):
1. Build a sorted edge list: for each triangle edge that crosses a Z plane, record the crossing.
2. At each layer Z, collect all edge crossings, link them into closed contours by following triangle adjacency.
3. Sort contours by winding (outer vs. inner).

Time complexity: O(n log k + k + m) where n = triangles, k = edges crossing this layer, m = contour vertices.

### 3.2 Conical Slicing

**File:** `conical.rs`

Inspired by RotBot (Etienne et al., ICRA 2021). Instead of flat planes, layers are cones. Good for radially symmetric objects (vases, bottles) — eliminates radial overhangs.

**Forward transform:**
```
z_deformed = z_original + r * tan(cone_angle)
```
where `r = sqrt((x-cx)² + (y-cy)²)` is the radial distance from the cone axis.

**Pipeline:**
1. Apply forward transform to every vertex → deformed Mesh
2. Run planar slicer on deformed mesh
3. Apply inverse transform to every contour point

The inverse just subtracts the same `r * tan(angle)` from Z. Since r is preserved in XY, this is exact.

**Limitation:** Artifacts where triangles straddle the cone axis (r → 0). Degenerate triangles near center are filtered.

### 3.3 S4 Non-Planar

**Files:** `s3_slicer/pipeline.rs`, `tet_dijkstra_field.rs`, `s4_rotation_field.rs`, `tet_point_location.rs`

Inspired by [jyjblrd/S4_Slicer](https://github.com/jyjblrd/S4_Slicer). The key insight: if you deform the mesh so overhangs unfold, you can slice flat and then un-deform the toolpaths.

**Pipeline steps:**

**Step 1 — Grid tet mesh** (`tet_mesh.rs`)
A regular grid covers the mesh bounding box. Each cube is split into 6 tetrahedra using Freudenthal decomposition. Tets with all 4 vertices outside the mesh are discarded. This bypasses TetGen entirely and works on any mesh.

**Step 2 — Dijkstra distance field** (`tet_dijkstra_field.rs`)
Multi-source Dijkstra from "base" tets (those touching the bottom of the mesh). Edge weights use a **Z-biased blend**:
```
weight = |ΔZ| × z_bias  +  Euclidean × (1 − z_bias)
```
`z_bias` defaults to 0.8. At 0.0 the field is pure Euclidean graph distance (original behaviour), which causes topologically-adjacent but geometrically-separated features — e.g. the two ears of the Stanford Bunny, which share the neck as a common path — to end up at the same distance and therefore on the same layer. At 0.8 the distance field strongly tracks actual vertical height, so each ear is correctly on its own set of layers. The Euclidean floor (via the `1 − z_bias` term) ensures horizontal adjacencies still cost something so gradient directions remain valid.

**Step 3 — Rotation field** (`s4_rotation_field.rs`)
For each surface tet (those with at least one boundary face), compute the overhang angle. If it exceeds `overhang_threshold` (default 45°, preset 35°), compute a rotation about the horizontal axis perpendicular to the overhang normal to reduce the overhang to threshold. Clamp by `max_rotation_degrees` (default 15°, preset 35°). Interior tets copy a damped average of their surface neighbours. Apply SLERP smoothing over the tet graph for `smoothing_iterations` passes to prevent discontinuities. The result is a per-tet quaternion.

**Support-Free Preset** (★ button in GUI, tuned for complex organic models like the Stanford Bunny):

| Parameter | Default | Preset |
|---|---|---|
| `z_bias` | 0.80 | 0.85 |
| `overhang_threshold` | 45° | 35° |
| `max_rotation_degrees` | 15° | 35° |
| `smoothing_iterations` | 25 | 40 |
| `smoothness_weight` | 0.5 | 0.6 |

**Step 4 — ASAP deformation** (`tet_asap_deformation.rs`)
Apply the quaternion field as a volumetric deformation. Each tet's vertices are moved according to its rotation. "As-Rigid-As-Possible" minimizes distortion. See §3.4 for ASAP details.

**Step 5 — Planar slice**
The deformed surface mesh is extracted and given to the standard planar slicer.

**Step 6 — Barycentric untransform** (`tet_point_location.rs`)
For each contour point in deformed space:
- Find the containing tet (spatial bin grid for acceleration)
- Compute barycentric coordinates within that tet
- Interpolate the same barycentric coordinates in the *original* tet mesh

This maps every point back to its correct location on the original geometry.

### 3.4 S3 Curved Layer

**Files:** `s3_slicer/pipeline.rs`, `tet_quaternion_field.rs`, `tet_asap_deformation.rs`, `tet_scalar_field.rs`, `marching_tet.rs`

Full implementation of the S3-Slicer paper (Zhang et al., SIGGRAPH Asia 2022).

**Pipeline steps:**

**Step 1 — Mesh repair** (voxel reconstruction, see §4)

**Step 2 — Tetrahedralization**
Tries in order: (a) TetGen on original mesh, (b) TetGen on voxel-reconstructed mesh, (c) grid-based fallback. TetGen produces a higher quality mesh but fails on self-intersecting input.

**Step 3 — Quaternion field optimization** (`tet_quaternion_field.rs`)
Each tet gets a quaternion representing the local print orientation. The field is optimized to:
- Align with the global +Z (gravity) direction where possible
- Minimize overhang angles across the volume
- Maintain continuity with neighboring tets (minimize rotation across shared faces)

Implemented as iterated face-adjacency smoothing: each tet's quaternion is updated toward the weighted average of its neighbors, biased by the overhang objective.

**Step 4 — Volumetric ASAP deformation** (`tet_asap_deformation.rs`)
"As-Rigid-As-Possible" minimizes the distortion energy:
```
E = Σ_t ||J_t - R_t||²_F
```
where J_t is the Jacobian of the deformation for tet t, and R_t is the target rotation from the quaternion field.

Each tet's rotation R is extracted via SVD of the deformation gradient. The system is solved iteratively: update rotations → solve global linear system for vertex positions → repeat.

Quality check: if more than 30% of tets invert (det(J) < 0) or the bounding box grows by more than 5×, falls back to VirtualScalarField mode.

**Step 5 — Scalar field** (`tet_scalar_field.rs`)
After deformation, assign a scalar value to each vertex proportional to its deformed Z coordinate. Smooth with Gauss-Seidel Laplacian (30 iterations) to remove artifacts from the deformation.

**Step 6 — Marching tetrahedra** (`marching_tet.rs`)
Extract isosurfaces of the scalar field at uniform spacing. Each tet is tested against a lookup table (analogous to Marching Cubes but for tets). The result is a set of curved triangulated surfaces — the print layers.

**Deformation method variants** (chosen in GUI):
- **TetVolumetric** — full pipeline above (best quality)
- **ASAP** — surface-only ASAP without volumetric tet mesh
- **VirtualScalarField** — no deformation, scalar field computed analytically

### 3.5 Geodesic Slicing

**File:** `geodesic.rs`

Layers follow the geodesic distance from a source boundary measured across the mesh surface. A sphere would produce concentric spherical layers; a vase would produce layers that hug the vase wall.

**Step 1 — Topology** (`MeshTopology::from_mesh`)
Weld coincident vertices by quantizing coordinates to 1e-6 mm precision. Build `vertex_faces` adjacency. Compute area-weighted vertex normals.

**Step 2 — Cotangent Laplacian**
Build a sparse symmetric matrix L where:
```
L[i,j] = (cot α_ij + cot β_ij) / 2    for adjacent vertices i,j
L[i,i] = -Σ_j L[i,j]
```
α_ij and β_ij are the two angles opposite edge (i,j) in the two adjacent triangles. This is the standard FEM Laplace-Beltrami operator on triangle meshes.

Also build a lumped mass matrix M (diagonal, each entry = 1/3 × area of incident triangles).

**Step 3 — Heat diffusion**
Solve the backward-Euler heat equation:
```
(M + t·L) u = M δ_S
```
where δ_S is 1 at source vertices, 0 elsewhere, and t = factor × avg_edge². The solution u is a heat distribution that has flowed from the source for time t.

The linear system is solved with Jacobi-preconditioned Conjugate Gradient (500 iterations, tolerance 1e-8).

**Step 4 — Gradient normalization**
Compute the gradient of u on each triangle, normalize to unit length. This gives the direction of geodesic flow on each face.

**Step 5 — Poisson solve**
Solve:
```
L φ = div(X)
```
where X are the normalized gradient vectors, and div is their discrete divergence at each vertex. Pin one vertex (+1e-6 on diagonal) to remove the null space. Shift result so source vertices have φ = 0.

The output φ is the geodesic distance field.

**Step 6 — Level-set extraction** (marching triangles)
For each triangle, find edges where φ crosses a threshold. Interpolate crossing points. Connect crossings into contours. This is the 2D analogue of Marching Cubes.

**Multi-scale mode:**
Run steps 3–5 at N doubling timesteps: t, 2t, 4t, ..., 2^(N-1)·t. At each vertex, keep the result from the finest scale where the heat value exceeds a threshold (indicating the heat reached that vertex). This gives fine detail in thin features (small t) and full-mesh coverage in large bodies (large t).

See §6 for diffusion mode details.

### 3.6 Cylindrical Slicing

**File:** `coordinate_transform.rs`

Maps the mesh into cylindrical coordinates so the planar slicer produces radial shells.

**Forward transform** (Cartesian → cylindrical deformed space):
```
θ = atan2(y - cy, x - cx)
r = sqrt((x-cx)² + (y-cy)²)
→ deformed = (θ, z_original, r)
```
The slicer cuts at constant deformed-Z = constant r. So each layer is a surface at constant radius from the cylinder axis.

**Inverse transform:**
```
x = cx + r·cos(θ)
y = cy + r·sin(θ)
z = z_original
```

**Limitation:** Triangles that straddle the branch cut at θ = ±π will produce artifacts at that seam. This affects one "line" on the back of the object relative to the axis.

### 3.7 Spherical Slicing

**File:** `coordinate_transform.rs`

Same idea as cylindrical but with spherical coordinates. Layers are concentric spheres.

**Forward transform:**
```
r = sqrt((x-cx)² + (y-cy)² + (z-cz)²)
θ = atan2(y-cy, x-cx)
φ = acos((z-cz) / r)
→ deformed = (θ, φ, r)
```
Slicer cuts at constant r.

**Inverse:**
```
x = cx + r·sin(φ)·cos(θ)
y = cy + r·sin(φ)·sin(θ)
z = cz + r·cos(φ)
```

---

## 4. Mesh Repair — Voxel Reconstruction

**File:** `s3_slicer/voxel_remesh.rs`

STL files from the real world often have self-intersecting faces, open boundaries, or flipped normals. TetGen refuses such input (returns status 3). This module converts any triangle soup into a clean manifold surface.

**Algorithm:**

**Step 1 — Bounding box + grid**
Allocate a 3D grid of ~100 cells along the longest axis. Each cell is a voxel.

**Step 2 — Unsigned distance field**
For each grid vertex, find the nearest triangle surface point (spatial bin acceleration). Record the unsigned distance.

**Step 3 — Sign (inside/outside) via ray casting**
Fire rays along +X, +Y, +Z for each grid vertex. Count triangle crossings. Use majority vote across the 3 axes to determine inside/outside. 2D spatial bins accelerate the ray-triangle intersection tests.

**Step 4 — Signed distance field**
Combine: SDF = unsigned_distance × sign (negative inside, positive outside).

**Step 5 — Marching Cubes**
Extract the isosurface at SDF = 0 using standard MC lookup tables (EDGE_TABLE 256×u16, TRI_TABLE 256×[i8;16]). An edge-to-vertex cache ensures shared edges between adjacent cubes reuse the same vertex.

**Why Marching Cubes (not Surface Nets):**
Surface Nets was tried first but its cross-cell quads produced intersecting faces that TetGen still rejected. MC triangles are confined within their cube cells, so they cannot intersect adjacent cubes' triangles — guaranteed manifold output.

**Resolution cascade:**
Tries base resolution → 75% → 50%. Falls back to vertex-clustering simplification, then convex hull if all fail.

**Performance:**
~2–5 seconds for an 80K-triangle mesh. Previous approach (isotropic remeshing) took ~28 minutes.

---

## 5. Tetrahedral Meshing

**File:** `s3_slicer/tet_mesh.rs`

Two strategies, selected automatically:

### TetGen (via tritet crate)
Constrained Delaunay tetrahedralization. Produces high-quality tets with controlled dihedral angles. Requires clean manifold input — use voxel reconstruction first.

Feature flag: `tetgen = ["tritet/with_tetgen"]` (enabled by default). Disable with `--no-default-features` if AGPL is unacceptable.

### Grid-Based (Freudenthal decomposition)
Fallback when TetGen fails. Divide the bounding box into a regular grid. Split each cube into 6 tetrahedra using Freudenthal decomposition (specific diagonal orientations that tile space without gaps or overlaps). Discard tets outside the mesh using a point-in-mesh test.

Lower quality than TetGen (less uniform tet shapes) but works on any input.

### `from_surface_mesh()` cascade:
1. Try TetGen on original mesh
2. Voxel reconstruct → try TetGen again
3. Vertex clustering simplification → TetGen
4. Convex hull (last resort, loses concavities)

---

## 6. Geodesic Diffusion Modes

**File:** `geodesic.rs`

The `GeodesicDiffusionMode` enum controls how the heat-step Laplacian is constructed. The Poisson step always uses the standard isotropic cotangent Laplacian.

### Isotropic
Standard heat method. One uniform Laplacian for heat diffusion. κ = 1 everywhere.

### Adaptive Scalar
```
κ(face) = kappa_base × (avg_edge_length(face))²
```
Clamped to [0.05, 50]. Large triangles diffuse faster (ensuring coverage); small triangles diffuse slower (preserving detail). The ² exponent matches the physical scaling of heat diffusion with distance.

### Anisotropic
Per-face curvature directions drive a rank-1 tensor update:
```
σ_f = I + (ratio - 1) · d_f ⊗ d_f
```
where d_f is the principal curvature direction of face f, estimated as the tangential component of the deviation between vertex normals and the flat face normal.

ratio < 1: heat diffuses slower across curvature directions → layers hug creases and bends.
ratio > 1: heat diffuses faster across curvature directions → smoother wavefronts.

`smoothing_iters` applies Laplacian smoothing on the S² of face directions before building the tensor, averaging out noise on coarse meshes.

FEM stiffness assembly (shared kernel `build_stiffness_from_face_dirs`):
```
K[i,j] += area × (∇φ_i · σ_f · ∇φ_j)
```
where ∇φ_i is the shape function gradient for vertex i in face f:
```
∇φ_i = cross(n̂_f, e_opposite_i) / (2 × area_f)
```

### Print Direction Biased
Same rank-1 tensor, but d_f is computed by projecting a global preferred direction (X, Y, or Z axis) onto each face's tangent plane. Heat flows faster along the print direction, producing layers more aligned with the chosen axis.

### Custom Vector Field
Bypasses the heat step entirely. The user supplies per-face direction vectors. `compute_vertex_divergence` computes the divergence of these directions at each vertex, then the Poisson equation is solved directly to get the distance field. Does not support multi-scale.

---

## 7. Toolpath Generation

**File:** `toolpath.rs`, `toolpath_patterns.rs`, `contour_offset.rs`, `ruled_surface.rs`

After slicing produces `Vec<Layer>`, toolpaths are generated per-layer.

### Wall Loops (`contour_offset.rs`)
2D polygon offset using the Clipper algorithm concept: shrink each contour inward by one nozzle width per perimeter. The outermost perimeter is printed first (best surface quality).

For non-planar layers, wall loop points are post-projected onto the original mesh surface using `MeshRayCaster::project_z()` after the 2D offset is computed — this corrects the Z values which would otherwise be approximated from boundary point interpolation.

### Infill (`toolpath_patterns.rs`)
- **Rectilinear** — alternating horizontal/vertical lines
- **Zigzag** — single connected zigzag path
- **Concentric** — shrinking copies of the outer contour
- **Spiral** — continuous Archimedean spiral

Infill density is the line spacing as a fraction of the bounding box.

### Mesh-Mapped Z Projection (`MeshRayCaster`)
A 2D spatial bin grid (64×64 default) indexes all mesh triangles by XY bounding box. For any XY query point, a vertical ray is cast and the hit with Z nearest to the layer's nominal Z is returned (correctly handles multi-shell objects).

**Vertical ray-triangle intersection (XY barycentric):**
```
denom = (v1y-v2y)(v0x-v2x) + (v2x-v1x)(v0y-v2y)
a = ((v1y-v2y)(x-v2x) + (v2x-v1x)(y-v2y)) / denom
b = ((v2y-v0y)(x-v2x) + (v0x-v2x)(y-v2y)) / denom
c = 1-a-b
if a,b,c >= -1e-6:  z_hit = a*v0.z + b*v1.z + c*v2.z
```
Priority order for infill Z: raycaster hit → IDW from wall loop points → boundary interpolation.

### Coverage Gap Fill (`coverage_gap_fill`)
After projecting infill scanlines onto the mesh, checks the 3D distance between consecutive scanline rows. If the 3D gap > 2 × `(line_width / infill_density)`, inserts a midpoint scanline at Y_mid, projects it onto the mesh, and recurses (max 3 levels = up to 8× refinement). Only fires when the mesh is available and the layer has curved Z.

### Adaptive Layer Height
`SlicingConfig.adaptive` enables per-layer height computation. `slope_factor()` in `slicing.rs` averages `|normal.z|` of triangles in a band around each candidate Z height:
- Flat faces (`|normal.z|` ≈ 1) → `max_layer_height` (fast, coarser)
- Vertical faces (`|normal.z|` ≈ 0) → `min_layer_height` (slow, finer)

The per-layer height flows: `Layer.layer_height` → `Toolpath.layer_height` → viewport tube radius (`extrusion_radius = layer_height / 2`, clamped to [0.04, 0.30] mm). Travel moves use a fixed thin radius (0.05 mm) for visual distinction.

`extrude_contour()` uses the per-layer height when computing extrusion volume, so extrusion amounts are correct for adaptive layers.

### Wall Seam Transitions (`ruled_surface.rs`)
Optional: when `wall_seam_transitions` is enabled and the layer has curved Z, generate a ruled-surface zigzag path between consecutive outer wall contours. This fills the staircase gap between layers on the outer surface.

Algorithm: resample both contours to the same N points (N ≤ 256), find the minimum-rotation alignment offset by minimising total point-to-point distance, then generate a single-pass transition path: C_n[0]→C_{n+1}[0], C_n[1]→C_{n+1}[1], etc.

### 5-Axis Orientation
For non-planar layers, each toolpath point carries a 3D orientation vector (the local surface normal, pointing away from the surface). The G-code generator converts this to A/B rotation angles.

---

## 8. 5-Axis G-Code and TCP Compensation

**File:** `gcode.rs`

### Output Format
Standard G-code extended with rotary axes:
```
G1 X{x:.3} Y{y:.3} Z{z:.3} A{a:.3} B{b:.3} E{e:.4} F{f:.0}
```

`RotaryAxisMode::AB` → A (pitch) and B (roll)
`RotaryAxisMode::BC` → B (tilt) and C (rotate) — for table-tilt/table-rotate machines

### Orientation to Angles (`orientation_to_angles`)
Converts the toolpath orientation vector to A/B angles. A is pitch (rotation around X), B is roll (rotation around Y).

### TCP Compensation (`tcp_compensate`)
On a 5-axis machine, the pivot point is typically at the head gearbox, not the nozzle tip. When the head tilts, the nozzle moves in XYZ by:
```
Δx = offset × sin(A) × cos(B)
Δy = offset × sin(A) × sin(B)
Δz = offset × (1 - cos(A))
```
TCP compensation subtracts this offset so the nozzle tip lands at the intended position regardless of tilt angle. Set `tcp_offset` to the distance from the pivot to the nozzle tip in mm.

---

## 9. Collision Detection

**File:** `motion_planning/collision.rs`

### Platform Collision
AABB test: checks if the printhead bounding box intersects the build platform at the current Z position.

### Mesh Collision (`check_collision_with_mesh`)
Models the printhead as a capsule (cylinder with hemispherical caps) from the nozzle tip upward. Tests this capsule against every triangle of the already-printed mesh.

**parry3d API:**
```rust
Capsule::new(
    Point::new(0.0, 0.0, -half_height),  // bottom endpoint
    Point::new(0.0, 0.0,  half_height),  // top endpoint
    radius
)
```
Note: parry3d's Capsule takes explicit endpoints, not a half-length. The capsule is positioned via an `Isometry::translation` at the current nozzle position.

**Acceleration:** Per-triangle AABB pre-filter (6 comparisons) before the exact `intersection_test`. Reduces expensive convex queries to only nearby triangles.

---

## 10. Support Generation

**Files:** `support_generation/`

### Overhang Detection (`overhang_detection.rs`)
Each triangle face is classified as overhanging if the angle between its normal and the global print direction exceeds a threshold (typically 45°). The dot product `normal · print_dir < cos(threshold)` is the test.

### Tree-Based Supports (`tree_skeleton.rs`)
Overhanging regions are connected to the build platform by tree-like support structures. Branch points are computed using a medial-axis approximation (centroidal axis). Branches taper toward tips and merge toward the base to minimize material use.

### Support Toolpath (`support_toolpath.rs`)
Supports are sliced the same as the main object (planar by default) and given a sparse rectilinear infill for easy removal.

---

## 11. GUI Architecture

**Files:** `gui/`

### Threading Model
Slicing runs on a background thread to keep the UI responsive. Launched via `std::thread::spawn` in `app.rs`. Results are sent back on a `crossbeam::channel`. The main thread polls the channel each frame and updates the viewport when results arrive.

### State (`app.rs`)
`SlicerApp` holds all UI state: loaded mesh, slice results, all parameter values for every mode. Parameters are plain `f64`/`u8`/`bool` fields — no trait objects or dynamic dispatch.

### Control Panel (`control_panel.rs`)
Immediate-mode egui: every frame, the panel redraws all controls. Conditional sections (e.g., geodesic diffusion mode options) are shown/hidden using `if app.geodesic_diffusion_mode == N { ... }` — no retained widget state.

### 3D Viewport (`viewport_3d.rs`)
three-d renders the mesh and toolpath preview. The mesh is uploaded as a vertex buffer once on load. Layer contours are rendered as line strips. Camera is orbit-controlled (mouse drag to rotate, scroll to zoom).

### Theme (`theme.rs`)
Dark theme with custom accent colors applied to egui's `Visuals`.

---

## 12. Module Reference

| Module | File | Description |
|---|---|---|
| `geometry` | `geometry.rs` | `Point3D`, `Vector3D`, `Plane`, `Triangle`, `BoundingBox`, `Contour` |
| `mesh` | `mesh.rs` | `Mesh` struct, STL I/O, bounding box computation |
| `slicing` | `slicing.rs` | Planar slicer, `Layer`, `SlicingConfig` |
| `toolpath` | `toolpath.rs` | `ToolpathGenerator`, wall loops, infill dispatch |
| `toolpath_patterns` | `toolpath_patterns.rs` | Rectilinear, zigzag, concentric, spiral infill |
| `contour_offset` | `contour_offset.rs` | 2D polygon inset for perimeters |
| `gcode` | `gcode.rs` | `GCodeGenerator`, `MachineKinematics`, TCP compensation |
| `geodesic` | `geodesic.rs` | Full geodesic pipeline, all diffusion modes, CG solver |
| `conical` | `conical.rs` | Conical coordinate transform pipeline |
| `coordinate_transform` | `coordinate_transform.rs` | Cylindrical and spherical pipelines |
| `centroidal_axis` | `centroidal_axis.rs` | Fast medial axis approximation |
| `ruled_surface` | `ruled_surface.rs` | Ruled surface detection for print orientation |
| `singularity` | `singularity.rs` | Singularity configuration optimization |
| `collision` | `collision.rs` | (legacy) — see `motion_planning/collision.rs` |
| `s3_slicer::pipeline` | `s3_slicer/pipeline.rs` | S3 and S4 pipeline orchestration |
| `s3_slicer::tet_mesh` | `s3_slicer/tet_mesh.rs` | `TetMesh`, TetGen and grid construction |
| `s3_slicer::voxel_remesh` | `s3_slicer/voxel_remesh.rs` | SDF + Marching Cubes repair |
| `s3_slicer::tet_asap_deformation` | `s3_slicer/tet_asap_deformation.rs` | Volumetric ASAP solver |
| `s3_slicer::tet_quaternion_field` | `s3_slicer/tet_quaternion_field.rs` | Per-tet quaternion field optimization |
| `s3_slicer::tet_dijkstra_field` | `s3_slicer/tet_dijkstra_field.rs` | Multi-source Dijkstra, gradient computation |
| `s3_slicer::s4_rotation_field` | `s3_slicer/s4_rotation_field.rs` | Overhang-aware rotation field for S4 |
| `s3_slicer::tet_point_location` | `s3_slicer/tet_point_location.rs` | Spatial bins + barycentric coordinates |
| `s3_slicer::marching_tet` | `s3_slicer/marching_tet.rs` | Marching tetrahedra isosurface extraction |
| `s3_slicer::tet_scalar_field` | `s3_slicer/tet_scalar_field.rs` | Volume scalar field with Laplacian smoothing |
| `s3_slicer::isotropic_remesh` | `s3_slicer/isotropic_remesh.rs` | Botsch & Kobbelt 2004 (legacy, not primary) |
| `motion_planning::collision` | `motion_planning/collision.rs` | Capsule-vs-mesh collision, platform AABB |
| `motion_planning::graph_search` | `motion_planning/graph_search.rs` | Graph-based path planning |
| `support_generation::overhang_detection` | `support_generation/overhang_detection.rs` | Overhang angle classification |
| `support_generation::tree_skeleton` | `support_generation/tree_skeleton.rs` | Tree support structure generation |
| `gui::app` | `gui/app.rs` | `SlicerApp`, background thread dispatch |
| `gui::control_panel` | `gui/control_panel.rs` | Full sidebar UI |
| `gui::viewport_3d` | `gui/viewport_3d.rs` | three-d 3D rendering |
| `gui::stats_panel` | `gui/stats_panel.rs` | Slicing statistics display |
| `gui::gcode_panel` | `gui/gcode_panel.rs` | G-code text preview |

---

## 13. Test Coverage

Run with: `cargo test --lib`

**Passing: 108** (3 pre-existing failures unrelated to current work)

| Module | Tests |
|---|---|
| `geodesic` | 6 — CG solver, topology welding, boundary detection, distance field, level sets |
| `contour_offset` | polygon inset correctness |
| `toolpath` | wall loop and infill generation |
| `voxel_remesh` | SDF construction, MC output, manifold check |
| `tet_mesh` | TetGen integration, grid fallback |
| `tet_asap_deformation` | SVD deformation, inversion detection |
| `motion_planning::collision` | platform hit, platform miss, mesh hit, mesh miss |

**Pre-existing failures (not caused by current work):**
- `test_s3_pipeline` — single-triangle mesh too small for TetGen
- `test_global_unfold_rotation` — degenerate test geometry
- `test_overhang_detection_vertical` — numerical tolerance issue

---

## 14. Performance Notes

All timings on an 80K-triangle Stanford Bunny, release build (`opt-level=3, lto=true`):

| Operation | Time |
|---|---|
| STL load | < 0.1s |
| Voxel reconstruction (mesh repair) | 2–5s |
| TetGen tetrahedralization | 1–2s |
| Quaternion field optimization | ~1s |
| Volumetric ASAP deformation | 2–5s |
| Scalar field + Marching tetrahedra | 1–2s |
| Geodesic (single scale, 50K vertices) | < 1s |
| Geodesic (8 scales, 50K vertices) | ~3–6s |
| Planar slicing (80K triangles, 200 layers) | < 0.5s |

Isotropic remeshing (legacy, now replaced by voxel reconstruction): ~28 minutes for 80K triangles. Voxel reconstruction achieves equivalent mesh quality in 2–5 seconds.

---

## 15. Known Limitations

- **Branch-cut artifacts** in cylindrical and spherical modes: triangles straddling θ = ±π produce incorrect contours at that seam line. Workaround: orient the object so the seam is in an unimportant region.

- **TetGen AGPL:** The `tritet` crate wraps TetGen which is AGPL-3.0. Disable with `--no-default-features` to use grid-based tetrahedralization only (slightly lower tet quality).

- **S3 quality check:** If ASAP deformation produces > 30% inverted tets or expands the bounding box by > 5×, the system falls back to VirtualScalarField mode without warning in the viewport.

- **S4 support-free on complex models:** Even with the support-free preset (z_bias 0.85, max_rotation 35°), very deep undercuts (> 60° overhang) may still require some support material. The `max_rotation_degrees` cap prevents the deformation from becoming so large it inverts tets.

- **Geodesic CustomVectorField:** No multi-scale support. Single solve only.

- **Examples don't compile:** `simple_slicer.rs` and `slice_benchy.rs` have a pre-existing missing field (`max_rotation_degrees`). Use `cargo test --lib` or `cargo run --bin gui` — do not use `cargo test` (which attempts to compile examples).

- **No GPU acceleration:** All solvers are CPU-only. The CG solver for large meshes (> 200K vertices) may take 10–30s for a single geodesic solve.

- **Conical floating-contour filter resolution:** The 64×64 bin grid may not separate very thin isolated features (< 2% of object footprint) from the main body. Such features may still be deferred indefinitely or printed at the wrong time.

- **Wall seam transitions:** Off by default. Not fully validated on all layer topologies — contour resampling assumes single outer loops and may produce artifacts on multi-island layers.

- **MeshRayCaster misses at mesh boundary:** Infill/wall points that project outside the mesh XY footprint get no raycaster hit and fall back to IDW interpolation. Visible as slight Z inaccuracy at the very edge of the print.
