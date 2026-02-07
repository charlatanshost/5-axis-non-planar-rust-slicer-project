// Tetrahedral Mesh for Volumetric Deformation
// Provides volumetric mesh structure for the S3-Slicer pipeline
//
// When the `tetgen` feature is enabled, uses TetGen (via tritet crate) for
// high-quality constrained Delaunay tetrahedralization.
// Otherwise falls back to a naive star-shaped method.

use crate::geometry::{Point3D, Vector3D};
use crate::mesh::Mesh;
use std::collections::HashMap;

/// A single tetrahedron element
#[derive(Debug, Clone, Copy)]
pub struct Tetrahedron {
    /// Four vertex indices
    pub vertices: [usize; 4],
}

impl Tetrahedron {
    /// Create new tetrahedron
    pub fn new(v0: usize, v1: usize, v2: usize, v3: usize) -> Self {
        Self {
            vertices: [v0, v1, v2, v3],
        }
    }

    /// Compute signed volume of tetrahedron (positive = correct orientation)
    pub fn signed_volume(&self, positions: &[Point3D]) -> f64 {
        let v0 = positions[self.vertices[0]];
        let v1 = positions[self.vertices[1]];
        let v2 = positions[self.vertices[2]];
        let v3 = positions[self.vertices[3]];

        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let e3 = v3 - v0;

        e1.dot(&e2.cross(&e3)) / 6.0
    }

    /// Compute volume of tetrahedron (absolute value)
    pub fn volume(&self, positions: &[Point3D]) -> f64 {
        self.signed_volume(positions).abs()
    }

    /// Get the 3x3 edge matrix D = [v1-v0, v2-v0, v3-v0] (columns)
    /// Used for deformation gradient computation
    pub fn edge_matrix(&self, positions: &[Point3D]) -> nalgebra::Matrix3<f64> {
        let v0 = positions[self.vertices[0]];
        let v1 = positions[self.vertices[1]];
        let v2 = positions[self.vertices[2]];
        let v3 = positions[self.vertices[3]];

        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let e3 = v3 - v0;

        nalgebra::Matrix3::new(
            e1.x, e2.x, e3.x,
            e1.y, e2.y, e3.y,
            e1.z, e2.z, e3.z,
        )
    }

    /// Get edges of tetrahedron
    pub fn edges(&self) -> [(usize, usize); 6] {
        [
            (self.vertices[0], self.vertices[1]),
            (self.vertices[0], self.vertices[2]),
            (self.vertices[0], self.vertices[3]),
            (self.vertices[1], self.vertices[2]),
            (self.vertices[1], self.vertices[3]),
            (self.vertices[2], self.vertices[3]),
        ]
    }

    /// Get faces of tetrahedron (each face is a triangle)
    /// Face i is opposite to vertex i
    pub fn faces(&self) -> [[usize; 3]; 4] {
        [
            [self.vertices[1], self.vertices[2], self.vertices[3]], // Face opposite to v0
            [self.vertices[0], self.vertices[3], self.vertices[2]], // Face opposite to v1
            [self.vertices[0], self.vertices[1], self.vertices[3]], // Face opposite to v2
            [self.vertices[0], self.vertices[2], self.vertices[1]], // Face opposite to v3
        ]
    }

    /// Compute outward face normals (unnormalized, magnitude = 2*area)
    pub fn face_normals(&self, positions: &[Point3D]) -> [Vector3D; 4] {
        let faces = self.faces();
        let mut normals = [Vector3D::new(0.0, 0.0, 0.0); 4];

        for (i, face) in faces.iter().enumerate() {
            let v0 = positions[face[0]];
            let v1 = positions[face[1]];
            let v2 = positions[face[2]];
            let e1 = v1 - v0;
            let e2 = v2 - v0;
            normals[i] = e1.cross(&e2);
        }

        normals
    }

    /// Compute centroid
    pub fn centroid(&self, positions: &[Point3D]) -> Point3D {
        let v0 = positions[self.vertices[0]];
        let v1 = positions[self.vertices[1]];
        let v2 = positions[self.vertices[2]];
        let v3 = positions[self.vertices[3]];

        Point3D::new(
            (v0.x + v1.x + v2.x + v3.x) / 4.0,
            (v0.y + v1.y + v2.y + v3.y) / 4.0,
            (v0.z + v1.z + v2.z + v3.z) / 4.0,
        )
    }

    /// Check if tetrahedron is degenerate (zero or negative volume)
    pub fn is_degenerate(&self, positions: &[Point3D]) -> bool {
        self.volume(positions) < 1e-10
    }

    /// Check if tetrahedron is inverted (negative signed volume)
    pub fn is_inverted(&self, positions: &[Point3D]) -> bool {
        self.signed_volume(positions) < 0.0
    }
}

/// Tetrahedral mesh for volumetric deformation
#[derive(Debug, Clone)]
pub struct TetMesh {
    /// Vertex positions
    pub vertices: Vec<Point3D>,

    /// Tetrahedra
    pub tets: Vec<Tetrahedron>,

    /// Surface triangle indices (for rendering/slicing)
    pub surface_triangles: Vec<[usize; 3]>,

    /// Tet adjacency: for each tet, the neighboring tet across each face (None if boundary)
    /// tet_neighbors[i][j] = neighbor across face j of tet i
    pub tet_neighbors: Vec<[Option<usize>; 4]>,

    /// Vertex-to-tet mapping: which tets contain each vertex
    pub vertex_tets: Vec<Vec<usize>>,

    /// Bounding box
    pub bounds_min: Point3D,
    pub bounds_max: Point3D,
}

impl TetMesh {
    /// Create tetrahedral mesh with full adjacency computation
    pub fn new(vertices: Vec<Point3D>, tets: Vec<Tetrahedron>) -> Self {
        let bounds_min = vertices.iter().fold(
            Point3D::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            |min, v| Point3D::new(min.x.min(v.x), min.y.min(v.y), min.z.min(v.z)),
        );

        let bounds_max = vertices.iter().fold(
            Point3D::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
            |max, v| Point3D::new(max.x.max(v.x), max.y.max(v.y), max.z.max(v.z)),
        );

        let surface_triangles = extract_surface_triangles(&tets);
        let tet_neighbors = compute_tet_adjacency(&tets);
        let vertex_tets = compute_vertex_tets(&tets, vertices.len());

        Self {
            vertices,
            tets,
            surface_triangles,
            tet_neighbors,
            vertex_tets,
            bounds_min,
            bounds_max,
        }
    }

    /// Tetrahedralize a surface mesh using TetGen (requires `tetgen` feature)
    ///
    /// Large meshes (>10K triangles) are always simplified first via vertex
    /// clustering to avoid TetGen crashes from C++ assertion failures on
    /// complex geometry. Tries increasingly coarse grids until TetGen succeeds.
    #[cfg(feature = "tetgen")]
    pub fn from_surface_mesh(surface_mesh: &Mesh) -> Result<Self, String> {
        use crate::s3_slicer::voxel_remesh::{voxel_remesh, auto_grid_resolution};

        log::info!("Tetrahedralizing surface mesh with TetGen...");
        log::info!("  Surface triangles: {}", surface_mesh.triangles.len());

        // ── Strategy 1: Try original mesh directly ──────────────────────
        // For small, clean meshes this often succeeds immediately.
        let tri_count = surface_mesh.triangles.len();
        if tri_count <= 10_000 {
            match Self::run_tetgen(surface_mesh) {
                Ok(mesh) => return Ok(mesh),
                Err(e) => {
                    log::warn!("TetGen failed on original mesh: {}", e);
                }
            }
        } else {
            log::info!("  Mesh has {} triangles (>10K) — skipping raw TetGen attempt", tri_count);
        }

        // ── Strategy 2: Voxel reconstruction (primary approach) ─────────
        // Converts mesh to SDF on regular grid, then extracts a clean
        // manifold surface via Surface Nets. Guarantees no self-intersections
        // regardless of input mesh quality. Fast (~2-5 seconds).
        log::info!("  Attempting voxel reconstruction before TetGen...");
        let base_res = auto_grid_resolution(surface_mesh);
        for &res in &[base_res, (base_res * 3 / 4).max(32), (base_res / 2).max(32)] {
            log::info!("  Voxel remesh (resolution: {})...", res);
            let voxel_mesh = voxel_remesh(surface_mesh, res);

            if voxel_mesh.triangles.len() < 4 {
                log::warn!("  Voxel mesh too small ({} tris), trying next resolution",
                    voxel_mesh.triangles.len());
                continue;
            }

            log::info!("  Voxel reconstructed: {} → {} triangles",
                surface_mesh.triangles.len(), voxel_mesh.triangles.len());

            match Self::run_tetgen(&voxel_mesh) {
                Ok(mesh) => {
                    log::info!("  TetGen succeeded on voxel mesh (resolution {})", res);
                    return Ok(mesh);
                }
                Err(e) => {
                    log::warn!("  TetGen failed on voxel mesh at resolution {}: {}", res, e);
                }
            }
        }

        // ── Strategy 3: Vertex clustering (legacy fallback) ─────────────
        // Cruder simplification that snaps vertices to a grid. Can introduce
        // self-intersections on complex meshes but works for simpler geometry.
        log::info!("  Isotropic remeshing didn't help — trying vertex clustering...");
        for &resolution in &[0.5, 1.0, 2.0, 4.0] {
            log::info!("  Simplifying mesh (grid resolution: {:.2} mm)...", resolution);
            let simplified = simplify_mesh_for_tetgen(surface_mesh, resolution);
            log::info!("  Simplified: {} → {} triangles",
                surface_mesh.triangles.len(), simplified.triangles.len());

            if simplified.triangles.len() < 4 {
                log::warn!("  Too few triangles after simplification at {:.2} mm", resolution);
                continue;
            }

            match Self::run_tetgen(&simplified) {
                Ok(mesh) => {
                    log::info!("  TetGen succeeded after simplification at {:.2} mm", resolution);
                    return Ok(mesh);
                }
                Err(e) => {
                    log::warn!("  TetGen still failed at {:.2} mm resolution: {}", resolution, e);
                }
            }
        }

        // ── Strategy 4: Convex hull (last resort) ───────────────────────
        // Guaranteed manifold, but loses all concavities. ASAP deformation
        // on a convex hull is usually poor quality.
        log::info!("  All approaches failed — trying convex hull fallback...");
        for &hull_resolution in &[2.0, 4.0, 8.0] {
            match convex_hull_at_resolution(surface_mesh, hull_resolution) {
                Ok(hull_mesh) => {
                    match Self::run_tetgen_opts(&hull_mesh, None, None) {
                        Ok(mesh) => {
                            log::info!("  TetGen succeeded on convex hull (grid {:.1}mm)", hull_resolution);
                            return Ok(mesh);
                        }
                        Err(e) => {
                            log::warn!("  TetGen failed on convex hull at {:.1}mm: {}", hull_resolution, e);
                        }
                    }
                }
                Err(e) => {
                    log::warn!("  Convex hull at {:.1}mm failed: {}", hull_resolution, e);
                }
            }
        }

        Err("TetGen failed on all attempts including convex hull fallback".to_string())
    }

    /// Run TetGen on a prepared surface mesh with default quality constraints
    #[cfg(feature = "tetgen")]
    fn run_tetgen(surface_mesh: &Mesh) -> Result<Self, String> {
        let mesh_volume = (surface_mesh.bounds_max.x - surface_mesh.bounds_min.x)
            * (surface_mesh.bounds_max.y - surface_mesh.bounds_min.y)
            * (surface_mesh.bounds_max.z - surface_mesh.bounds_min.z);
        Self::run_tetgen_opts(surface_mesh, Some(mesh_volume / 5000.0), Some(15.0))
    }

    /// Run TetGen with explicit quality options.
    /// Pass None for max_tet_volume and min_angle to skip quality refinement
    /// (pure constrained Delaunay, safer for edge-case geometry like convex hulls).
    #[cfg(feature = "tetgen")]
    fn run_tetgen_opts(
        surface_mesh: &Mesh,
        max_tet_volume: Option<f64>,
        min_angle: Option<f64>,
    ) -> Result<Self, String> {
        // Extract unique vertices and build face index list
        let mut vertices: Vec<Point3D> = Vec::new();
        let mut vertex_map: HashMap<u64, usize> = HashMap::new();
        let mut facets: Vec<[usize; 3]> = Vec::new();

        for triangle in &surface_mesh.triangles {
            let mut face_indices = [0usize; 3];
            for (i, vertex) in [triangle.v0, triangle.v1, triangle.v2].iter().enumerate() {
                let hash = hash_point(vertex);
                let idx = *vertex_map.entry(hash).or_insert_with(|| {
                    let idx = vertices.len();
                    vertices.push(*vertex);
                    idx
                });
                face_indices[i] = idx;
            }
            // Skip degenerate triangles (two or more vertices merged to same index)
            if face_indices[0] != face_indices[1]
                && face_indices[1] != face_indices[2]
                && face_indices[0] != face_indices[2]
            {
                facets.push(face_indices);
            }
        }

        let npoint = vertices.len();
        let nfacet = facets.len();
        log::info!("  Unique vertices: {}, facets: {}", npoint, nfacet);

        if npoint < 4 || nfacet < 4 {
            return Err("Too few vertices/facets for tetrahedralization".to_string());
        }

        let facet_npoint: Vec<usize> = vec![3; nfacet];

        let mut tetgen = tritet::Tetgen::new(
            npoint,
            Some(facet_npoint),
            None,
            None,
        ).map_err(|e| format!("TetGen init error: {}", e))?;

        for (i, v) in vertices.iter().enumerate() {
            tetgen.set_point(i, 0, v.x, v.y, v.z)
                .map_err(|e| format!("TetGen set_point error: {}", e))?;
        }

        for (fi, face) in facets.iter().enumerate() {
            for (m, &pidx) in face.iter().enumerate() {
                tetgen.set_facet_point(fi, m, pidx)
                    .map_err(|e| format!("TetGen set_facet_point error: {}", e))?;
            }
        }

        log::info!("  Generating tet mesh (max_vol={}, min_angle={})...",
            max_tet_volume.map_or("none".to_string(), |v| format!("{:.4}", v)),
            min_angle.map_or("none".to_string(), |a| format!("{:.1}", a)));

        tetgen.generate_mesh(
            false,
            false,
            max_tet_volume,
            min_angle,
        ).map_err(|e| format!("TetGen generate_mesh error: {}", e))?;

        let out_npoint = tetgen.out_npoint();
        let out_ncell = tetgen.out_ncell();
        let cell_npoint = tetgen.out_cell_npoint();

        log::info!("  TetGen output: {} vertices, {} tetrahedra ({} nodes/tet)",
            out_npoint, out_ncell, cell_npoint);

        if cell_npoint != 4 {
            return Err(format!("Expected 4 nodes per tet, got {}", cell_npoint));
        }

        if out_ncell == 0 {
            return Err("TetGen produced 0 tetrahedra (mesh may have self-intersecting facets)".to_string());
        }

        let mut out_vertices = Vec::with_capacity(out_npoint);
        for i in 0..out_npoint {
            out_vertices.push(Point3D::new(
                tetgen.out_point(i, 0),
                tetgen.out_point(i, 1),
                tetgen.out_point(i, 2),
            ));
        }

        let mut out_tets = Vec::with_capacity(out_ncell);
        for i in 0..out_ncell {
            out_tets.push(Tetrahedron::new(
                tetgen.out_cell_point(i, 0),
                tetgen.out_cell_point(i, 1),
                tetgen.out_cell_point(i, 2),
                tetgen.out_cell_point(i, 3),
            ));
        }

        let mesh = Self::new(out_vertices, out_tets);

        let quality = mesh.check_quality();
        log::info!("  Tet mesh quality:");
        log::info!("    Vertices: {}", quality.num_vertices);
        log::info!("    Tetrahedra: {}", quality.num_tets);
        log::info!("    Surface triangles: {}", quality.num_surface_triangles);
        log::info!("    Volume range: {:.6} to {:.6}", quality.min_tet_volume, quality.max_tet_volume);
        log::info!("    Degenerate tets: {}", quality.degenerate_tets);

        Ok(mesh)
    }

    /// Tetrahedralize a surface mesh using naive fallback (no TetGen)
    #[cfg(not(feature = "tetgen"))]
    pub fn from_surface_mesh(surface_mesh: &Mesh) -> Result<Self, String> {
        Self::from_surface_mesh_naive(surface_mesh, 8)
    }

    /// Naive star-shaped tetrahedralization (fallback when TetGen is unavailable)
    pub fn from_surface_mesh_naive(surface_mesh: &Mesh, num_interior_points: usize) -> Result<Self, String> {
        log::info!("Tetrahedralizing surface mesh (naive fallback)...");
        log::info!("  Surface triangles: {}", surface_mesh.triangles.len());
        log::info!("  Target interior points: {}", num_interior_points);

        // Extract unique vertices from surface
        let mut vertices = Vec::new();
        let mut vertex_map = HashMap::new();

        for triangle in &surface_mesh.triangles {
            for &vertex in &[triangle.v0, triangle.v1, triangle.v2] {
                let hash = hash_point(&vertex);
                vertex_map.entry(hash).or_insert_with(|| {
                    let idx = vertices.len();
                    vertices.push(vertex);
                    idx
                });
            }
        }

        log::info!("  Surface vertices: {}", vertices.len());

        let initial_vertex_count = vertices.len();
        add_interior_points(
            &mut vertices,
            &surface_mesh.bounds_min,
            &surface_mesh.bounds_max,
            num_interior_points,
        );

        log::info!("  Total vertices (with interior): {}", vertices.len());

        let tets = create_simple_tetrahedralization(
            &vertices,
            &surface_mesh.triangles,
            &vertex_map,
            initial_vertex_count,
        );

        log::info!("  Tetrahedra created: {}", tets.len());

        if tets.is_empty() {
            return Err("Naive tetrahedralization produced no tetrahedra".to_string());
        }

        Ok(Self::new(vertices, tets))
    }

    /// Convert back to surface mesh
    pub fn to_surface_mesh(&self) -> Mesh {
        let triangles = self
            .surface_triangles
            .iter()
            .map(|&[i0, i1, i2]| crate::geometry::Triangle {
                v0: self.vertices[i0],
                v1: self.vertices[i1],
                v2: self.vertices[i2],
            })
            .collect();

        Mesh::new(triangles).unwrap()
    }

    /// Compute total volume
    pub fn total_volume(&self) -> f64 {
        self.tets
            .iter()
            .map(|tet| tet.volume(&self.vertices))
            .sum()
    }

    /// Get vertex positions
    pub fn vertex_positions(&self) -> &[Point3D] {
        &self.vertices
    }

    /// Get vertex positions (mutable)
    pub fn vertex_positions_mut(&mut self) -> &mut [Point3D] {
        &mut self.vertices
    }

    /// Get unique edges across the entire mesh
    pub fn unique_edges(&self) -> Vec<(usize, usize)> {
        let mut edge_set: HashMap<(usize, usize), ()> = HashMap::new();
        for tet in &self.tets {
            for (a, b) in tet.edges() {
                let key = if a < b { (a, b) } else { (b, a) };
                edge_set.entry(key).or_insert(());
            }
        }
        edge_set.into_keys().collect()
    }

    /// Find boundary vertices (vertices on surface faces)
    pub fn boundary_vertices(&self) -> Vec<usize> {
        let mut boundary: Vec<bool> = vec![false; self.vertices.len()];
        for face in &self.surface_triangles {
            for &vi in face {
                boundary[vi] = true;
            }
        }
        boundary.iter().enumerate()
            .filter(|(_, &is_boundary)| is_boundary)
            .map(|(i, _)| i)
            .collect()
    }

    /// Find bottom vertices (below a Z threshold)
    pub fn bottom_vertices(&self, z_fraction: f64) -> Vec<usize> {
        let z_range = self.bounds_max.z - self.bounds_min.z;
        let z_threshold = self.bounds_min.z + z_range * z_fraction;

        self.vertices.iter().enumerate()
            .filter(|(_, v)| v.z <= z_threshold)
            .map(|(i, _)| i)
            .collect()
    }

    /// Check mesh quality
    pub fn check_quality(&self) -> MeshQuality {
        let mut min_volume = f64::INFINITY;
        let mut max_volume = f64::NEG_INFINITY;
        let mut degenerate_count = 0;
        let mut inverted_count = 0;

        for tet in &self.tets {
            let vol = tet.volume(&self.vertices);
            if tet.is_degenerate(&self.vertices) {
                degenerate_count += 1;
            }
            if tet.is_inverted(&self.vertices) {
                inverted_count += 1;
            }
            min_volume = min_volume.min(vol);
            max_volume = max_volume.max(vol);
        }

        MeshQuality {
            num_vertices: self.vertices.len(),
            num_tets: self.tets.len(),
            num_surface_triangles: self.surface_triangles.len(),
            min_tet_volume: min_volume,
            max_tet_volume: max_volume,
            avg_tet_volume: self.total_volume() / self.tets.len().max(1) as f64,
            degenerate_tets: degenerate_count,
            inverted_tets: inverted_count,
        }
    }

    /// Recompute adjacency data after vertex positions change (e.g., after deformation)
    pub fn recompute_bounds(&mut self) {
        self.bounds_min = self.vertices.iter().fold(
            Point3D::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            |min, v| Point3D::new(min.x.min(v.x), min.y.min(v.y), min.z.min(v.z)),
        );
        self.bounds_max = self.vertices.iter().fold(
            Point3D::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
            |max, v| Point3D::new(max.x.max(v.x), max.y.max(v.y), max.z.max(v.z)),
        );
    }
}

/// Tetrahedral mesh quality metrics
#[derive(Debug, Clone)]
pub struct MeshQuality {
    pub num_vertices: usize,
    pub num_tets: usize,
    pub num_surface_triangles: usize,
    pub min_tet_volume: f64,
    pub max_tet_volume: f64,
    pub avg_tet_volume: f64,
    pub degenerate_tets: usize,
    pub inverted_tets: usize,
}

/// Compute tet-to-tet adjacency via shared faces
fn compute_tet_adjacency(tets: &[Tetrahedron]) -> Vec<[Option<usize>; 4]> {
    // Map from sorted face → (tet_index, face_index)
    let mut face_map: HashMap<[usize; 3], Vec<(usize, usize)>> = HashMap::new();

    for (ti, tet) in tets.iter().enumerate() {
        for (fi, face) in tet.faces().iter().enumerate() {
            let mut sorted = *face;
            sorted.sort();
            face_map.entry(sorted).or_default().push((ti, fi));
        }
    }

    let mut neighbors = vec![[None; 4]; tets.len()];

    for entries in face_map.values() {
        if entries.len() == 2 {
            let (ti0, fi0) = entries[0];
            let (ti1, fi1) = entries[1];
            neighbors[ti0][fi0] = Some(ti1);
            neighbors[ti1][fi1] = Some(ti0);
        }
        // entries.len() == 1 means boundary face (no neighbor)
    }

    neighbors
}

/// Compute vertex-to-tet mapping
fn compute_vertex_tets(tets: &[Tetrahedron], num_vertices: usize) -> Vec<Vec<usize>> {
    let mut vertex_tets = vec![Vec::new(); num_vertices];

    for (ti, tet) in tets.iter().enumerate() {
        for &vi in &tet.vertices {
            if vi < num_vertices {
                vertex_tets[vi].push(ti);
            }
        }
    }

    vertex_tets
}

/// Extract surface triangles from tetrahedral mesh
fn extract_surface_triangles(tets: &[Tetrahedron]) -> Vec<[usize; 3]> {
    let mut face_count: HashMap<[usize; 3], [usize; 3]> = HashMap::new();

    for tet in tets {
        for face in tet.faces() {
            let mut sorted_face = face;
            sorted_face.sort();

            // Store the original (unsorted) face for correct winding
            face_count.entry(sorted_face).or_insert(face);
        }
    }

    // Count occurrences via the adjacency approach: surface faces appear in exactly 1 tet
    let mut face_occ: HashMap<[usize; 3], usize> = HashMap::new();
    for tet in tets {
        for face in tet.faces() {
            let mut sorted = face;
            sorted.sort();
            *face_occ.entry(sorted).or_insert(0) += 1;
        }
    }

    face_occ
        .into_iter()
        .filter(|(_, count)| *count == 1)
        .map(|(sorted_face, _)| {
            // Return the original winding from face_count
            *face_count.get(&sorted_face).unwrap_or(&sorted_face)
        })
        .collect()
}

/// Add interior points using grid sampling
fn add_interior_points(
    vertices: &mut Vec<Point3D>,
    bounds_min: &Point3D,
    bounds_max: &Point3D,
    num_points: usize,
) {
    let grid_size = (num_points as f64).cbrt().ceil() as usize;

    let dx = (bounds_max.x - bounds_min.x) / (grid_size + 1) as f64;
    let dy = (bounds_max.y - bounds_min.y) / (grid_size + 1) as f64;
    let dz = (bounds_max.z - bounds_min.z) / (grid_size + 1) as f64;

    for i in 1..=grid_size {
        for j in 1..=grid_size {
            for k in 1..=grid_size {
                let point = Point3D::new(
                    bounds_min.x + i as f64 * dx,
                    bounds_min.y + j as f64 * dy,
                    bounds_min.z + k as f64 * dz,
                );
                vertices.push(point);
            }
        }
    }
}

/// Create simple tetrahedralization by connecting surface triangles to centroid
fn create_simple_tetrahedralization(
    vertices: &[Point3D],
    surface_triangles: &[crate::geometry::Triangle],
    vertex_map: &HashMap<u64, usize>,
    surface_vertex_count: usize,
) -> Vec<Tetrahedron> {
    let mut tets = Vec::new();

    let centroid_idx = if surface_vertex_count < vertices.len() {
        surface_vertex_count
    } else {
        log::warn!("No interior points available for tetrahedralization");
        return tets;
    };

    for triangle in surface_triangles {
        let v0 = *vertex_map.get(&hash_point(&triangle.v0)).unwrap();
        let v1 = *vertex_map.get(&hash_point(&triangle.v1)).unwrap();
        let v2 = *vertex_map.get(&hash_point(&triangle.v2)).unwrap();

        let tet = Tetrahedron::new(v0, v1, v2, centroid_idx);

        if !tet.is_degenerate(vertices) {
            tets.push(tet);
        }
    }

    tets
}

/// Simplify a surface mesh via vertex clustering for TetGen compatibility
///
/// Snaps vertices to a grid of the given resolution, merges duplicates,
/// and removes degenerate triangles. This fixes self-intersecting facets
/// that are common in high-poly STL files.
fn simplify_mesh_for_tetgen(mesh: &Mesh, grid_resolution: f64) -> Mesh {
    use crate::geometry::Triangle;

    let inv_res = 1.0 / grid_resolution;

    // Snap vertices to grid and build vertex map
    let mut vertex_map: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut vertices: Vec<Point3D> = Vec::new();
    let mut triangles: Vec<Triangle> = Vec::new();

    for tri in &mesh.triangles {
        let mut indices = [0usize; 3];
        let tri_verts = [tri.v0, tri.v1, tri.v2];

        for (i, v) in tri_verts.iter().enumerate() {
            let gx = (v.x * inv_res).round() as i64;
            let gy = (v.y * inv_res).round() as i64;
            let gz = (v.z * inv_res).round() as i64;
            let key = (gx, gy, gz);

            let idx = *vertex_map.entry(key).or_insert_with(|| {
                let idx = vertices.len();
                // Use the grid cell center as the vertex position
                vertices.push(Point3D::new(
                    gx as f64 * grid_resolution,
                    gy as f64 * grid_resolution,
                    gz as f64 * grid_resolution,
                ));
                idx
            });
            indices[i] = idx;
        }

        // Skip degenerate triangles
        if indices[0] != indices[1] && indices[1] != indices[2] && indices[0] != indices[2] {
            let t = Triangle::new(
                vertices[indices[0]],
                vertices[indices[1]],
                vertices[indices[2]],
            );
            // Also skip zero-area triangles
            let e1 = t.v1 - t.v0;
            let e2 = t.v2 - t.v0;
            if e1.cross(&e2).norm() > 1e-12 {
                triangles.push(t);
            }
        }
    }

    // Remove duplicate triangles (same sorted vertex indices)
    let mut seen: std::collections::HashSet<(usize, usize, usize)> = std::collections::HashSet::new();
    let mut unique_triangles: Vec<Triangle> = Vec::new();

    for tri in &mesh.triangles {
        let tri_verts = [tri.v0, tri.v1, tri.v2];
        let mut indices = [0usize; 3];
        for (i, v) in tri_verts.iter().enumerate() {
            let gx = (v.x * inv_res).round() as i64;
            let gy = (v.y * inv_res).round() as i64;
            let gz = (v.z * inv_res).round() as i64;
            indices[i] = *vertex_map.get(&(gx, gy, gz)).unwrap();
        }

        if indices[0] == indices[1] || indices[1] == indices[2] || indices[0] == indices[2] {
            continue;
        }

        let mut sorted = [indices[0], indices[1], indices[2]];
        sorted.sort();
        let key = (sorted[0], sorted[1], sorted[2]);

        if seen.insert(key) {
            unique_triangles.push(Triangle::new(
                vertices[indices[0]],
                vertices[indices[1]],
                vertices[indices[2]],
            ));
        }
    }

    log::info!("  Vertex clustering: {} → {} vertices, {} → {} triangles",
        mesh.triangles.len() * 3, vertices.len(),
        mesh.triangles.len(), unique_triangles.len());

    // Fix non-manifold edges: edges shared by >2 triangles cause TetGen to
    // report "self-intersecting facets". Iteratively remove the smallest-area
    // triangle on each non-manifold edge until every edge has at most 2 faces.
    let unique_triangles = fix_non_manifold_edges(unique_triangles, &vertex_map, inv_res);

    // Fix fold-back pairs: two adjacent triangles (sharing an edge) whose
    // non-shared vertices are on the same side of the edge, causing the
    // triangle surfaces to geometrically cross each other.
    let unique_triangles = fix_foldback_intersections(unique_triangles, &vertex_map, inv_res);

    Mesh::new(unique_triangles).unwrap_or_else(|_| mesh.clone())
}

/// Remove triangles that create non-manifold edges (edges shared by >2 triangles).
///
/// For each non-manifold edge, removes the incident triangle with the smallest
/// area, repeating until every edge has at most 2 incident triangles.
fn fix_non_manifold_edges(
    mut triangles: Vec<crate::geometry::Triangle>,
    vertex_map: &HashMap<(i64, i64, i64), usize>,
    inv_res: f64,
) -> Vec<crate::geometry::Triangle> {
    // Helper: get vertex index for a point
    let get_idx = |p: &Point3D| -> usize {
        let gx = (p.x * inv_res).round() as i64;
        let gy = (p.y * inv_res).round() as i64;
        let gz = (p.z * inv_res).round() as i64;
        *vertex_map.get(&(gx, gy, gz)).unwrap()
    };

    // Helper: compute triangle area
    let tri_area = |tri: &crate::geometry::Triangle| -> f64 {
        let e1 = tri.v1 - tri.v0;
        let e2 = tri.v2 - tri.v0;
        e1.cross(&e2).norm() * 0.5
    };

    let mut iterations = 0;
    let max_iterations = 50;

    loop {
        iterations += 1;
        if iterations > max_iterations {
            log::warn!("  Non-manifold fix: hit iteration limit ({}), {} triangles remain",
                max_iterations, triangles.len());
            break;
        }

        // Build edge → list of triangle indices
        let mut edge_tris: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        for (ti, tri) in triangles.iter().enumerate() {
            let idx = [get_idx(&tri.v0), get_idx(&tri.v1), get_idx(&tri.v2)];
            let edges = [(idx[0], idx[1]), (idx[1], idx[2]), (idx[0], idx[2])];
            for (a, b) in edges {
                let key = if a < b { (a, b) } else { (b, a) };
                edge_tris.entry(key).or_default().push(ti);
            }
        }

        // Find all non-manifold edges
        let non_manifold: Vec<_> = edge_tris.iter()
            .filter(|(_, tris)| tris.len() > 2)
            .collect();

        if non_manifold.is_empty() {
            if iterations > 1 {
                log::info!("  Non-manifold fix: clean after {} iterations, {} triangles remain",
                    iterations - 1, triangles.len());
            }
            break;
        }

        if iterations == 1 {
            log::info!("  Found {} non-manifold edges, fixing...", non_manifold.len());
        }

        // Collect triangles to remove: for each non-manifold edge, mark the
        // smallest-area incident triangle for removal
        let mut to_remove: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for (_, tri_indices) in &non_manifold {
            // Sort incident triangles by area (ascending)
            let mut by_area: Vec<(usize, f64)> = tri_indices.iter()
                .map(|&ti| (ti, tri_area(&triangles[ti])))
                .collect();
            by_area.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

            // Remove all but the 2 largest (keep the last 2)
            let remove_count = by_area.len().saturating_sub(2);
            for &(ti, _) in &by_area[..remove_count] {
                to_remove.insert(ti);
            }
        }

        if to_remove.is_empty() {
            break;
        }

        // Remove marked triangles
        let before = triangles.len();
        let mut keep = Vec::with_capacity(triangles.len() - to_remove.len());
        for (i, tri) in triangles.into_iter().enumerate() {
            if !to_remove.contains(&i) {
                keep.push(tri);
            }
        }
        triangles = keep;
        log::info!("  Non-manifold fix iteration {}: removed {} triangles ({} → {})",
            iterations, before - triangles.len(), before, triangles.len());
    }

    triangles
}

/// Remove triangles that form fold-back pairs (geometric self-intersections).
///
/// After vertex clustering, adjacent triangles can fold back on each other,
/// creating geometric self-intersections that TetGen rejects. For each pair
/// of triangles sharing an edge (A,B) with opposite vertices C and D, projects
/// C and D perpendicular to edge AB — if both are on the same side, the mesh
/// folds back and the smaller-area triangle is removed.
fn fix_foldback_intersections(
    mut triangles: Vec<crate::geometry::Triangle>,
    vertex_map: &HashMap<(i64, i64, i64), usize>,
    inv_res: f64,
) -> Vec<crate::geometry::Triangle> {
    let get_idx = |p: &Point3D| -> usize {
        let gx = (p.x * inv_res).round() as i64;
        let gy = (p.y * inv_res).round() as i64;
        let gz = (p.z * inv_res).round() as i64;
        *vertex_map.get(&(gx, gy, gz)).unwrap()
    };

    let tri_area = |tri: &crate::geometry::Triangle| -> f64 {
        let e1 = tri.v1 - tri.v0;
        let e2 = tri.v2 - tri.v0;
        e1.cross(&e2).norm() * 0.5
    };

    let mut iterations = 0;
    let max_iterations = 20;

    loop {
        iterations += 1;
        if iterations > max_iterations {
            log::warn!("  Fold-back fix: hit iteration limit ({})", max_iterations);
            break;
        }

        // Build vertex index → position map
        let mut idx_to_pos: HashMap<usize, Point3D> = HashMap::new();
        for tri in &triangles {
            for (v, p) in [(get_idx(&tri.v0), tri.v0), (get_idx(&tri.v1), tri.v1), (get_idx(&tri.v2), tri.v2)] {
                idx_to_pos.entry(v).or_insert(p);
            }
        }

        // Build edge → list of (triangle_index, opposite_vertex_index)
        let mut edge_tris: HashMap<(usize, usize), Vec<(usize, usize)>> = HashMap::new();
        for (ti, tri) in triangles.iter().enumerate() {
            let idx = [get_idx(&tri.v0), get_idx(&tri.v1), get_idx(&tri.v2)];
            let edges_with_opp = [
                ((idx[0], idx[1]), idx[2]),
                ((idx[1], idx[2]), idx[0]),
                ((idx[0], idx[2]), idx[1]),
            ];
            for ((a, b), opp) in edges_with_opp {
                let key = if a < b { (a, b) } else { (b, a) };
                edge_tris.entry(key).or_default().push((ti, opp));
            }
        }

        // Find fold-back pairs and mark smaller triangle for removal
        let mut to_remove: std::collections::HashSet<usize> = std::collections::HashSet::new();

        for (&(a, b), pairs) in &edge_tris {
            if pairs.len() != 2 {
                continue; // Non-manifold or boundary, handled by other fix
            }
            let (ti0, opp0) = pairs[0];
            let (ti1, opp1) = pairs[1];

            if to_remove.contains(&ti0) || to_remove.contains(&ti1) {
                continue;
            }

            let pa = idx_to_pos[&a];
            let pb = idx_to_pos[&b];
            let pc = idx_to_pos[&opp0];
            let pd = idx_to_pos[&opp1];

            // Project C and D perpendicular to edge AB
            let edge = pb - pa;
            let edge_len_sq = edge.dot(&edge);
            if edge_len_sq < 1e-20 {
                continue;
            }
            let edge_hat = edge / edge_len_sq.sqrt();

            let ac = pc - pa;
            let ad = pd - pa;
            let c_perp = ac - edge_hat * ac.dot(&edge_hat);
            let d_perp = ad - edge_hat * ad.dot(&edge_hat);

            // If perpendicular projections point in the same direction,
            // C and D are on the same side of edge AB → fold-back
            if c_perp.dot(&d_perp) > 0.0 {
                let area0 = tri_area(&triangles[ti0]);
                let area1 = tri_area(&triangles[ti1]);
                if area0 < area1 {
                    to_remove.insert(ti0);
                } else {
                    to_remove.insert(ti1);
                }
            }
        }

        if to_remove.is_empty() {
            if iterations > 1 {
                log::info!("  Fold-back fix: clean after {} iterations, {} triangles remain",
                    iterations - 1, triangles.len());
            }
            break;
        }

        if iterations == 1 {
            log::info!("  Found {} fold-back triangles, fixing...", to_remove.len());
        }

        let before = triangles.len();
        let mut keep = Vec::with_capacity(triangles.len() - to_remove.len());
        for (i, tri) in triangles.into_iter().enumerate() {
            if !to_remove.contains(&i) {
                keep.push(tri);
            }
        }
        triangles = keep;
        log::info!("  Fold-back fix iteration {}: removed {} triangles ({} → {})",
            iterations, before - triangles.len(), before, triangles.len());
    }

    triangles
}

/// Compute a convex hull of the mesh vertices at a given grid resolution.
///
/// Uses parry3d's convex hull algorithm which is guaranteed to produce a
/// manifold, non-self-intersecting surface. Vertices are subsampled by
/// snapping to a grid to avoid near-coplanar triangles that cause TetGen
/// status=4 (small feature) errors on high-poly meshes.
fn convex_hull_at_resolution(mesh: &Mesh, resolution: f64) -> Result<Mesh, String> {
    use crate::geometry::Triangle;

    let inv_res = 1.0 / resolution;
    let mut seen_grid: std::collections::HashSet<(i64, i64, i64)> = std::collections::HashSet::new();
    let mut points: Vec<nalgebra::Point3<f32>> = Vec::new();

    for tri in &mesh.triangles {
        for v in &[tri.v0, tri.v1, tri.v2] {
            let gx = (v.x * inv_res).round() as i64;
            let gy = (v.y * inv_res).round() as i64;
            let gz = (v.z * inv_res).round() as i64;
            if seen_grid.insert((gx, gy, gz)) {
                // Use actual vertex position (not grid center) for better hull accuracy
                points.push(nalgebra::Point3::new(v.x as f32, v.y as f32, v.z as f32));
            }
        }
    }

    if points.len() < 4 {
        return Err(format!("Too few vertices ({}) at {:.1}mm grid", points.len(), resolution));
    }

    log::info!("  Computing convex hull from {} subsampled vertices (grid {:.1}mm)...",
        points.len(), resolution);

    let (hull_verts, hull_faces) = parry3d::transformation::try_convex_hull(&points)
        .map_err(|e| format!("Convex hull error: {}", e))?;

    if hull_faces.is_empty() {
        return Err("Convex hull produced 0 faces".to_string());
    }

    log::info!("  Convex hull: {} triangles, {} vertices", hull_faces.len(), hull_verts.len());

    // Convert back to f64 Triangle list
    let triangles: Vec<Triangle> = hull_faces.iter().map(|face| {
        let v0 = hull_verts[face[0] as usize];
        let v1 = hull_verts[face[1] as usize];
        let v2 = hull_verts[face[2] as usize];
        Triangle::new(
            Point3D::new(v0.x as f64, v0.y as f64, v0.z as f64),
            Point3D::new(v1.x as f64, v1.y as f64, v1.z as f64),
            Point3D::new(v2.x as f64, v2.y as f64, v2.z as f64),
        )
    }).collect();

    Mesh::new(triangles).map_err(|e| format!("Failed to create hull mesh: {}", e))
}

/// Hash a point for deduplication
fn hash_point(p: &Point3D) -> u64 {
    let x = (p.x * 1000000.0).round() as i64;
    let y = (p.y * 1000000.0).round() as i64;
    let z = (p.z * 1000000.0).round() as i64;

    let mut hash = 0u64;
    hash = hash.wrapping_mul(73856093) ^ (x as u64);
    hash = hash.wrapping_mul(19349663) ^ (y as u64);
    hash = hash.wrapping_mul(83492791) ^ (z as u64);
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tetrahedron_volume() {
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(0.0, 1.0, 0.0),
            Point3D::new(0.0, 0.0, 1.0),
        ];

        let tet = Tetrahedron::new(0, 1, 2, 3);
        let volume = tet.volume(&vertices);
        assert!((volume - 1.0 / 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_signed_volume() {
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(0.0, 1.0, 0.0),
            Point3D::new(0.0, 0.0, 1.0),
        ];

        let tet = Tetrahedron::new(0, 1, 2, 3);
        assert!(tet.signed_volume(&vertices) > 0.0);

        // Swap two vertices to invert
        let tet_inv = Tetrahedron::new(0, 2, 1, 3);
        assert!(tet_inv.signed_volume(&vertices) < 0.0);
    }

    #[test]
    fn test_edge_matrix() {
        let vertices = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(0.0, 1.0, 0.0),
            Point3D::new(0.0, 0.0, 1.0),
        ];

        let tet = Tetrahedron::new(0, 1, 2, 3);
        let d = tet.edge_matrix(&vertices);

        // Should be identity for unit tet at origin
        assert!((d[(0, 0)] - 1.0).abs() < 1e-10);
        assert!((d[(1, 1)] - 1.0).abs() < 1e-10);
        assert!((d[(2, 2)] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_tetrahedron_edges() {
        let tet = Tetrahedron::new(0, 1, 2, 3);
        let edges = tet.edges();
        assert_eq!(edges.len(), 6);
        assert!(edges.contains(&(0, 1)));
        assert!(edges.contains(&(2, 3)));
    }

    #[test]
    fn test_tetrahedron_faces() {
        let tet = Tetrahedron::new(0, 1, 2, 3);
        let faces = tet.faces();
        assert_eq!(faces.len(), 4);
        for face in &faces {
            assert_eq!(face.len(), 3);
        }
    }

    #[test]
    fn test_surface_extraction() {
        let tets = vec![
            Tetrahedron::new(0, 1, 2, 3),
            Tetrahedron::new(0, 1, 2, 4),
        ];

        let surface = extract_surface_triangles(&tets);
        assert_eq!(surface.len(), 6);
    }

    #[test]
    fn test_tet_adjacency() {
        let tets = vec![
            Tetrahedron::new(0, 1, 2, 3),
            Tetrahedron::new(0, 1, 2, 4),
        ];

        let neighbors = compute_tet_adjacency(&tets);
        assert_eq!(neighbors.len(), 2);

        // They should be neighbors via their shared face [0,1,2]
        let has_neighbor_0 = neighbors[0].iter().any(|n| *n == Some(1));
        let has_neighbor_1 = neighbors[1].iter().any(|n| *n == Some(0));
        assert!(has_neighbor_0, "Tet 0 should have tet 1 as neighbor");
        assert!(has_neighbor_1, "Tet 1 should have tet 0 as neighbor");
    }

    #[test]
    fn test_vertex_tets() {
        let tets = vec![
            Tetrahedron::new(0, 1, 2, 3),
            Tetrahedron::new(0, 1, 2, 4),
        ];

        let vt = compute_vertex_tets(&tets, 5);
        assert_eq!(vt[0].len(), 2); // vertex 0 in both tets
        assert_eq!(vt[3].len(), 1); // vertex 3 only in tet 0
        assert_eq!(vt[4].len(), 1); // vertex 4 only in tet 1
    }
}
