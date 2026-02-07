// Isotropic Remeshing for Surface Meshes
//
// Implements the Botsch & Kobbelt (2004) incremental remeshing algorithm:
//   1. Split edges longer than 4/3 * target_length
//   2. Collapse edges shorter than 4/5 * target_length
//   3. Flip edges to equalize vertex valences
//   4. Tangential Laplacian smoothing with back-projection
//
// This preprocessing step produces a clean, uniformly-tessellated surface mesh
// that TetGen can reliably tetrahedralize, matching the MeshLab isotropic
// remeshing used in the original S3-Slicer paper.

use crate::geometry::{Point3D, Vector3D, Triangle};
use crate::mesh::Mesh;
use std::collections::{HashMap, HashSet};

/// Perform isotropic remeshing on a triangle mesh.
///
/// Produces a mesh with approximately uniform edge lengths close to
/// `target_edge_length`. The algorithm iterates split/collapse/flip/smooth
/// operations to gradually regularize the triangulation.
///
/// # Arguments
/// * `mesh` - Input surface mesh
/// * `target_edge_length` - Desired edge length in mesh units (mm)
/// * `iterations` - Number of remeshing iterations (typically 3-5)
pub fn isotropic_remesh(mesh: &Mesh, target_edge_length: f64, iterations: usize) -> Mesh {
    log::info!("Isotropic remeshing: target_edge_length={:.2}mm, {} iterations",
        target_edge_length, iterations);
    log::info!("  Input: {} triangles", mesh.triangles.len());

    let mut rmesh = IndexedMesh::from_mesh(mesh);
    log::info!("  Indexed mesh: {} vertices, {} faces",
        rmesh.vertices.len(), rmesh.active_face_count());

    let high = target_edge_length * 4.0 / 3.0;
    let low = target_edge_length * 4.0 / 5.0;

    // Build spatial index of original mesh for back-projection
    let original_triangles: Vec<[Point3D; 3]> = mesh.triangles.iter()
        .map(|t| [t.v0, t.v1, t.v2])
        .collect();
    let grid = SpatialGrid::build(&original_triangles, target_edge_length * 3.0);

    for iter in 0..iterations {
        // Step 1: Split long edges
        rmesh.split_long_edges(high);

        // Step 2: Collapse short edges
        rmesh.collapse_short_edges(low, high);

        // Step 3: Equalize valences via edge flips
        rmesh.equalize_valences();

        // Step 4: Tangential smoothing (every iteration)
        rmesh.tangential_smoothing_only(0.5);

        // Step 5: Back-project to original surface (every other iteration + final)
        // This is the expensive step, so we don't do it every iteration.
        // Max projection distance = 2x target edge length — prevents vertices from
        // snapping across to topologically-distant surfaces on self-intersecting meshes.
        if iter % 2 == 0 || iter == iterations - 1 {
            rmesh.back_project_grid(&grid, &original_triangles, target_edge_length * 2.0);
        }

        // Compact dead vertices/faces
        rmesh.compact();

        log::info!("  Iteration {}/{}: {} vertices, {} faces",
            iter + 1, iterations, rmesh.vertices.len(), rmesh.active_face_count());
    }

    // Final cleanup: remove non-manifold edges (edges shared by >2 faces).
    // These can arise from remeshing self-intersecting input meshes.
    rmesh.remove_non_manifold_faces();
    rmesh.compact();

    log::info!("  Output: {} vertices, {} faces",
        rmesh.vertices.len(), rmesh.active_face_count());

    rmesh.to_mesh()
}

/// Compute a good target edge length from the mesh bounding box.
/// Uses ~1% of the diagonal, clamped to a reasonable range.
pub fn auto_target_edge_length(mesh: &Mesh) -> f64 {
    let diag = (mesh.bounds_max - mesh.bounds_min).norm();
    // Target: roughly 1% of diagonal, but at least 0.5mm and at most 5mm
    (diag * 0.01).clamp(0.5, 5.0)
}

// ============================================================================
// Internal indexed mesh representation
// ============================================================================

/// Indexed mesh for efficient local topology operations.
/// Uses vertex indices instead of embedded positions, with alive/dead tracking
/// to avoid index invalidation during incremental operations.
struct IndexedMesh {
    vertices: Vec<Point3D>,
    faces: Vec<[usize; 3]>,
    face_alive: Vec<bool>,
    vertex_alive: Vec<bool>,
}

impl IndexedMesh {
    /// Build indexed mesh from flat triangle list, deduplicating vertices
    fn from_mesh(mesh: &Mesh) -> Self {
        let mut vertices: Vec<Point3D> = Vec::new();
        let mut vertex_map: HashMap<[i64; 3], usize> = HashMap::new();
        let mut faces: Vec<[usize; 3]> = Vec::new();

        // Spatial hashing with ~1 nanometer resolution for deduplication
        let eps = 1e-6;

        for tri in &mesh.triangles {
            let mut face = [0usize; 3];
            for (i, v) in [tri.v0, tri.v1, tri.v2].iter().enumerate() {
                let key = [
                    (v.x / eps).round() as i64,
                    (v.y / eps).round() as i64,
                    (v.z / eps).round() as i64,
                ];
                let idx = *vertex_map.entry(key).or_insert_with(|| {
                    vertices.push(*v);
                    vertices.len() - 1
                });
                face[i] = idx;
            }
            // Skip degenerate faces
            if face[0] != face[1] && face[1] != face[2] && face[0] != face[2] {
                faces.push(face);
            }
        }

        let nf = faces.len();
        let nv = vertices.len();
        IndexedMesh {
            vertices,
            faces,
            face_alive: vec![true; nf],
            vertex_alive: vec![true; nv],
        }
    }

    /// Convert back to flat Triangle mesh
    fn to_mesh(& self) -> Mesh {
        let triangles: Vec<Triangle> = self.faces.iter()
            .enumerate()
            .filter(|(i, _)| self.face_alive[*i])
            .filter_map(|(_, f)| {
                // Skip faces with dead vertices
                if !self.vertex_alive[f[0]] || !self.vertex_alive[f[1]] || !self.vertex_alive[f[2]] {
                    return None;
                }
                let tri = Triangle::new(
                    self.vertices[f[0]],
                    self.vertices[f[1]],
                    self.vertices[f[2]],
                );
                // Skip degenerate triangles
                if tri.area() < 1e-12 { return None; }
                Some(tri)
            })
            .collect();

        Mesh::new(triangles).unwrap_or_else(|_| {
            log::warn!("Remeshed mesh has 0 valid triangles, returning empty mesh");
            Mesh {
                triangles: Vec::new(),
                bounds_min: Point3D::origin(),
                bounds_max: Point3D::origin(),
            }
        })
    }

    fn active_face_count(&self) -> usize {
        self.face_alive.iter().filter(|&&a| a).count()
    }

    // ========================================================================
    // Adjacency queries
    // ========================================================================

    /// Build edge → [face indices] map. Edges stored as (min_idx, max_idx).
    fn build_edge_faces(&self) -> HashMap<(usize, usize), Vec<usize>> {
        let mut map: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        for (fi, face) in self.faces.iter().enumerate() {
            if !self.face_alive[fi] { continue; }
            for e in 0..3 {
                let edge = sorted_edge(face[e], face[(e + 1) % 3]);
                map.entry(edge).or_default().push(fi);
            }
        }
        map
    }

    /// Build vertex → [face indices] map
    fn build_vertex_faces(&self) -> Vec<Vec<usize>> {
        let mut vf = vec![Vec::new(); self.vertices.len()];
        for (fi, face) in self.faces.iter().enumerate() {
            if !self.face_alive[fi] { continue; }
            for &vi in face {
                vf[vi].push(fi);
            }
        }
        vf
    }

    /// Build vertex → {neighbor vertex indices}
    fn build_vertex_neighbors(&self) -> Vec<HashSet<usize>> {
        let mut vn = vec![HashSet::new(); self.vertices.len()];
        for (fi, face) in self.faces.iter().enumerate() {
            if !self.face_alive[fi] { continue; }
            for e in 0..3 {
                let a = face[e];
                let b = face[(e + 1) % 3];
                vn[a].insert(b);
                vn[b].insert(a);
            }
        }
        vn
    }

    /// Check if vertex is on a boundary edge (edge with only 1 adjacent face)
    fn is_boundary_vertex(&self, vi: usize, edge_faces: &HashMap<(usize, usize), Vec<usize>>) -> bool {
        for (&(a, b), faces) in edge_faces {
            if (a == vi || b == vi) && faces.len() == 1 {
                return true;
            }
        }
        false
    }

    /// Compute area-weighted vertex normal
    fn vertex_normal(&self, vi: usize, vertex_faces: &[Vec<usize>]) -> Vector3D {
        let mut normal = Vector3D::zeros();
        for &fi in &vertex_faces[vi] {
            if !self.face_alive[fi] { continue; }
            let f = &self.faces[fi];
            let e1 = self.vertices[f[1]] - self.vertices[f[0]];
            let e2 = self.vertices[f[2]] - self.vertices[f[0]];
            normal += e1.cross(&e2); // magnitude = 2 * area
        }
        let len = normal.norm();
        if len > 1e-12 { normal / len } else { Vector3D::z() }
    }

    // ========================================================================
    // Step 1: Split long edges
    // ========================================================================

    fn split_long_edges(&mut self, max_length: f64) {
        let max_sq = max_length * max_length;

        // We may need multiple passes since splitting can create new long edges.
        // Limit passes to avoid infinite loops.
        for _pass in 0..3 {
            let ef = self.build_edge_faces();
            let mut edges_to_split: Vec<(usize, usize, f64)> = Vec::new();

            for (&(a, b), _) in &ef {
                if !self.vertex_alive[a] || !self.vertex_alive[b] { continue; }
                let len_sq = (self.vertices[a] - self.vertices[b]).norm_squared();
                if len_sq > max_sq {
                    edges_to_split.push((a, b, len_sq));
                }
            }

            if edges_to_split.is_empty() { break; }

            // Split longest first for stability
            edges_to_split.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());

            for (a, b, _) in edges_to_split {
                if !self.vertex_alive[a] || !self.vertex_alive[b] { continue; }
                // Re-check length (previous splits may have changed things)
                let len_sq = (self.vertices[a] - self.vertices[b]).norm_squared();
                if len_sq > max_sq {
                    self.split_edge(a, b);
                }
            }
        }
    }

    /// Split edge (a, b) by inserting a midpoint vertex.
    /// Each adjacent triangle is replaced by two triangles.
    fn split_edge(&mut self, a: usize, b: usize) {
        let midpoint = nalgebra::center(&self.vertices[a], &self.vertices[b]);
        let mid = self.vertices.len();
        self.vertices.push(midpoint);
        self.vertex_alive.push(true);

        // Find all alive faces containing edge (a, b)
        let mut faces_to_split = Vec::new();
        for (fi, face) in self.faces.iter().enumerate() {
            if !self.face_alive[fi] { continue; }
            if face.contains(&a) && face.contains(&b) {
                faces_to_split.push(fi);
            }
        }

        for fi in faces_to_split {
            let f = self.faces[fi];
            // Find the vertex that is not a or b
            let c = f.iter().find(|&&v| v != a && v != b).copied().unwrap();

            // Determine winding order: find positions of a, b in the face
            let (pos_a, pos_b) = if f[0] == a && f[1] == b { (0, 1) }
                else if f[1] == a && f[2] == b { (1, 2) }
                else if f[2] == a && f[0] == b { (2, 0) }
                else if f[0] == b && f[1] == a { (0, 1) }
                else if f[1] == b && f[2] == a { (1, 2) }
                else { (2, 0) }; // f[2] == b && f[0] == a

            // Kill original face
            self.face_alive[fi] = false;

            // Create two new faces preserving winding order
            // Original face visits: ... a → b → c ... (in some rotation)
            // We split into: (a, mid, c) and (mid, b, c)
            // But we must respect the actual winding of the original triangle.
            if (pos_a + 1) % 3 == pos_b {
                // a comes before b in winding: face = (..., a, b, c, ...)
                self.push_face([a, mid, c]);
                self.push_face([mid, b, c]);
            } else {
                // b comes before a in winding: face = (..., b, a, c, ...)
                self.push_face([b, mid, c]);
                self.push_face([mid, a, c]);
            }
        }
    }

    // ========================================================================
    // Step 2: Collapse short edges
    // ========================================================================

    fn collapse_short_edges(&mut self, min_length: f64, max_length: f64) {
        let min_sq = min_length * min_length;
        let max_sq = max_length * max_length;

        let ef = self.build_edge_faces();
        let mut edges: Vec<(usize, usize, f64)> = Vec::new();

        for (&(a, b), _) in &ef {
            if !self.vertex_alive[a] || !self.vertex_alive[b] { continue; }
            let len_sq = (self.vertices[a] - self.vertices[b]).norm_squared();
            if len_sq < min_sq {
                edges.push((a, b, len_sq));
            }
        }

        // Collapse shortest first
        edges.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

        for (a, b, _) in edges {
            if !self.vertex_alive[a] || !self.vertex_alive[b] { continue; }
            let len_sq = (self.vertices[a] - self.vertices[b]).norm_squared();
            if len_sq >= min_sq { continue; }

            // Check if collapse is safe (won't create edges longer than max_length)
            if self.can_collapse(a, b, max_sq) {
                self.collapse_edge(a, b);
            }
        }
    }

    /// Check if collapsing edge (a,b) to midpoint is safe.
    /// Verifies no new edge would exceed max_length_sq.
    fn can_collapse(&self, a: usize, b: usize, max_length_sq: f64) -> bool {
        let midpoint = nalgebra::center(&self.vertices[a], &self.vertices[b]);

        // Check all neighbors of a and b: new edges from midpoint must not be too long
        for (fi, face) in self.faces.iter().enumerate() {
            if !self.face_alive[fi] { continue; }
            let has_a = face.contains(&a);
            let has_b = face.contains(&b);

            if has_a || has_b {
                for &vi in face {
                    if vi != a && vi != b && self.vertex_alive[vi] {
                        if (self.vertices[vi] - midpoint).norm_squared() > max_length_sq {
                            return false;
                        }
                    }
                }
            }
        }

        // Check that collapsing won't create flipped triangles.
        // For each face that will survive (contains a or b but not both),
        // verify the triangle with the replaced vertex has positive area
        // and consistent normal direction.
        for (fi, face) in self.faces.iter().enumerate() {
            if !self.face_alive[fi] { continue; }
            let has_a = face.contains(&a);
            let has_b = face.contains(&b);

            // Faces containing both a and b will be deleted (degenerate after merge)
            if has_a && has_b { continue; }

            if has_a || has_b {
                // This face survives with one vertex moved to midpoint
                let mut new_face = *face;
                for v in new_face.iter_mut() {
                    if *v == a || *v == b { *v = usize::MAX; } // placeholder
                }

                // Get the two vertices that stay
                let others: Vec<usize> = face.iter()
                    .filter(|&&v| v != a && v != b)
                    .copied()
                    .collect();

                if others.len() < 2 { continue; }

                // Check new triangle normal vs old
                let old_e1 = self.vertices[face[1]] - self.vertices[face[0]];
                let old_e2 = self.vertices[face[2]] - self.vertices[face[0]];
                let old_normal = old_e1.cross(&old_e2);

                // New triangle with midpoint replacing a/b
                let v0 = if face[0] == a || face[0] == b { midpoint } else { self.vertices[face[0]] };
                let v1 = if face[1] == a || face[1] == b { midpoint } else { self.vertices[face[1]] };
                let v2 = if face[2] == a || face[2] == b { midpoint } else { self.vertices[face[2]] };

                let new_e1 = v1 - v0;
                let new_e2 = v2 - v0;
                let new_normal = new_e1.cross(&new_e2);

                // Check for inversion (normals pointing opposite directions)
                if old_normal.dot(&new_normal) < 0.0 {
                    return false;
                }

                // Check for degeneracy
                if new_normal.norm() < 1e-12 {
                    return false;
                }
            }
        }

        true
    }

    /// Collapse edge (a,b): move vertex a to midpoint, redirect b → a, kill degenerate/duplicate faces
    fn collapse_edge(&mut self, a: usize, b: usize) {
        let midpoint = nalgebra::center(&self.vertices[a], &self.vertices[b]);
        self.vertices[a] = midpoint;
        self.vertex_alive[b] = false;

        // Redirect all references from b to a
        for fi in 0..self.faces.len() {
            if !self.face_alive[fi] { continue; }
            let face = &mut self.faces[fi];

            for v in face.iter_mut() {
                if *v == b { *v = a; }
            }

            // Kill degenerate faces (two identical vertex indices)
            if face[0] == face[1] || face[1] == face[2] || face[0] == face[2] {
                self.face_alive[fi] = false;
            }
        }

        // Kill duplicate faces: after vertex replacement, two previously-distinct
        // faces can end up with the same vertex set (e.g., [b,c,d] and [a,c,d]
        // both become [a,c,d] after collapsing b→a). TetGen rejects duplicates.
        self.remove_duplicate_faces();
    }

    /// Remove duplicate faces (faces with the same sorted vertex set).
    /// Keeps the first occurrence, kills subsequent duplicates.
    fn remove_duplicate_faces(&mut self) {
        let mut seen: HashSet<[usize; 3]> = HashSet::new();
        for fi in 0..self.faces.len() {
            if !self.face_alive[fi] { continue; }
            let mut key = self.faces[fi];
            key.sort();
            if !seen.insert(key) {
                self.face_alive[fi] = false; // duplicate
            }
        }
    }

    /// Remove faces on non-manifold edges (edges shared by >2 faces).
    /// For each non-manifold edge, keeps the two largest-area faces, kills the rest.
    fn remove_non_manifold_faces(&mut self) {
        let ef = self.build_edge_faces();
        let mut killed = 0usize;

        for (_, face_indices) in &ef {
            if face_indices.len() <= 2 { continue; }

            // Non-manifold edge: keep the 2 largest-area faces, kill the rest
            let mut indexed: Vec<(usize, f64)> = face_indices.iter()
                .filter(|&&fi| self.face_alive[fi])
                .map(|&fi| {
                    let f = &self.faces[fi];
                    let e1 = self.vertices[f[1]] - self.vertices[f[0]];
                    let e2 = self.vertices[f[2]] - self.vertices[f[0]];
                    let area = e1.cross(&e2).norm();
                    (fi, area)
                })
                .collect();

            if indexed.len() <= 2 { continue; }

            // Sort by area (largest first)
            indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

            // Kill all but the two largest
            for &(fi, _) in &indexed[2..] {
                if self.face_alive[fi] {
                    self.face_alive[fi] = false;
                    killed += 1;
                }
            }
        }

        if killed > 0 {
            log::info!("  Removed {} non-manifold faces", killed);
        }
    }

    // ========================================================================
    // Step 3: Equalize valences via edge flips
    // ========================================================================

    fn equalize_valences(&mut self) {
        let ef = self.build_edge_faces();
        let vn = self.build_vertex_neighbors();

        let edges: Vec<(usize, usize)> = ef.keys().copied().collect();

        for (a, b) in edges {
            let Some(adj) = ef.get(&(a, b)) else { continue };
            if adj.len() != 2 { continue; } // Only flip interior (manifold) edges

            let fi1 = adj[0];
            let fi2 = adj[1];
            if !self.face_alive[fi1] || !self.face_alive[fi2] { continue; }

            let f1 = self.faces[fi1];
            let f2 = self.faces[fi2];

            // Find opposite vertices c, d of the edge diamond
            let c = f1.iter().find(|&&v| v != a && v != b).copied().unwrap();
            let d = f2.iter().find(|&&v| v != a && v != b).copied().unwrap();

            if c == d { continue; } // Degenerate diamond

            // Target valence: 6 for interior, 4 for boundary
            let target_a = if self.is_boundary_vertex(a, &ef) { 4i32 } else { 6 };
            let target_b = if self.is_boundary_vertex(b, &ef) { 4i32 } else { 6 };
            let target_c = if self.is_boundary_vertex(c, &ef) { 4i32 } else { 6 };
            let target_d = if self.is_boundary_vertex(d, &ef) { 4i32 } else { 6 };

            let val_a = vn[a].len() as i32;
            let val_b = vn[b].len() as i32;
            let val_c = vn[c].len() as i32;
            let val_d = vn[d].len() as i32;

            // Deviation from target before flip
            let dev_before = (val_a - target_a).abs() + (val_b - target_b).abs()
                           + (val_c - target_c).abs() + (val_d - target_d).abs();

            // After flip: a and b lose one connection, c and d gain one
            let dev_after = (val_a - 1 - target_a).abs() + (val_b - 1 - target_b).abs()
                          + (val_c + 1 - target_c).abs() + (val_d + 1 - target_d).abs();

            if dev_after < dev_before {
                // Check geometric validity of the flip
                if self.is_flip_valid(a, b, c, d) {
                    // Kill old faces, create new ones with flipped diagonal
                    self.face_alive[fi1] = false;
                    self.face_alive[fi2] = false;

                    // New edge is (c, d) instead of (a, b)
                    // Preserve winding: (c, d, a) and (d, c, b) — or determine from originals
                    // Use normal consistency check
                    let old_n1 = face_normal(&self.vertices, &f1);
                    let new_f1 = [c, d, a];
                    let new_n1 = face_normal(&self.vertices, &new_f1);

                    if old_n1.dot(&new_n1) >= 0.0 {
                        self.push_face([c, d, a]);
                        self.push_face([d, c, b]);
                    } else {
                        self.push_face([d, c, a]);
                        self.push_face([c, d, b]);
                    }
                }
            }
        }
    }

    /// Check if flipping edge (a,b) to (c,d) produces valid (non-degenerate, non-inverted) triangles
    fn is_flip_valid(&self, a: usize, b: usize, c: usize, d: usize) -> bool {
        let pa = self.vertices[a];
        let pb = self.vertices[b];
        let pc = self.vertices[c];
        let pd = self.vertices[d];

        // New triangles: (c, d, a) and (d, c, b)
        let n1 = (pd - pc).cross(&(pa - pc));
        let n2 = (pc - pd).cross(&(pb - pd));

        // Both must have nonzero area
        if n1.norm() < 1e-12 || n2.norm() < 1e-12 { return false; }

        // Normals should agree (same side of surface)
        if n1.dot(&n2) < 0.0 { return false; }

        // Check that the new edge (c, d) doesn't already exist
        // (would create a non-manifold edge)
        for (fi, face) in self.faces.iter().enumerate() {
            if !self.face_alive[fi] { continue; }
            if face.contains(&c) && face.contains(&d) {
                // Check it's not one of the faces we're about to delete
                let has_a = face.contains(&a);
                let has_b = face.contains(&b);
                if !(has_a && has_b) {
                    return false; // Edge (c,d) already exists elsewhere
                }
            }
        }

        true
    }

    // ========================================================================
    // Step 4: Tangential smoothing
    // ========================================================================

    fn tangential_smoothing_only(&mut self, weight: f64) {
        let vf = self.build_vertex_faces();
        let ef = self.build_edge_faces();
        let vn = self.build_vertex_neighbors();

        let mut new_positions = self.vertices.clone();

        for vi in 0..self.vertices.len() {
            if !self.vertex_alive[vi] { continue; }
            if vn[vi].is_empty() { continue; }
            if self.is_boundary_vertex(vi, &ef) { continue; }

            // Laplacian: average of neighbor positions
            let mut avg = Vector3D::zeros();
            let mut count = 0;
            for &ni in &vn[vi] {
                if self.vertex_alive[ni] {
                    avg += self.vertices[ni].coords;
                    count += 1;
                }
            }
            if count == 0 { continue; }
            let avg_pos = Point3D::from(avg / count as f64);

            // Project displacement to tangent plane (preserves surface shape)
            let normal = self.vertex_normal(vi, &vf);
            let displacement = avg_pos - self.vertices[vi];
            let tangential = displacement - normal * normal.dot(&displacement);

            new_positions[vi] = self.vertices[vi] + tangential * weight;
        }

        self.vertices = new_positions;
    }

    // ========================================================================
    // Step 5: Back-projection to original surface (grid-accelerated)
    // ========================================================================

    fn back_project_grid(&mut self, grid: &SpatialGrid, original_tris: &[[Point3D; 3]], max_dist: f64) {
        let max_dist_sq = max_dist * max_dist;
        for vi in 0..self.vertices.len() {
            if !self.vertex_alive[vi] { continue; }

            let p = self.vertices[vi];
            if let Some(closest) = grid.closest_point(&p, original_tris) {
                // Only project if close enough — prevents snapping to topologically
                // distant surfaces (e.g., bunny's overlapping ear, benchy's cabin/hull)
                if (closest - p).norm_squared() <= max_dist_sq {
                    self.vertices[vi] = closest;
                }
            }
        }
    }

    // ========================================================================
    // Compact: remove dead vertices and faces, re-index
    // ========================================================================

    fn compact(&mut self) {
        // Find which vertices are actually referenced by alive faces
        let mut referenced = vec![false; self.vertices.len()];
        for (fi, face) in self.faces.iter().enumerate() {
            if self.face_alive[fi] {
                for &vi in face {
                    referenced[vi] = true;
                }
            }
        }

        // Build vertex remapping
        let mut new_index = vec![0usize; self.vertices.len()];
        let mut new_verts = Vec::new();
        for (old, &is_ref) in referenced.iter().enumerate() {
            if is_ref {
                new_index[old] = new_verts.len();
                new_verts.push(self.vertices[old]);
            }
        }

        // Rebuild faces with new indices
        let mut new_faces = Vec::new();
        for (fi, face) in self.faces.iter().enumerate() {
            if self.face_alive[fi] {
                new_faces.push([
                    new_index[face[0]],
                    new_index[face[1]],
                    new_index[face[2]],
                ]);
            }
        }

        self.vertices = new_verts;
        self.vertex_alive = vec![true; self.vertices.len()];
        self.faces = new_faces;
        self.face_alive = vec![true; self.faces.len()];

        // Final safety: remove any remaining duplicate faces
        self.remove_duplicate_faces();
    }

    // ========================================================================
    // Helpers
    // ========================================================================

    fn push_face(&mut self, face: [usize; 3]) {
        self.faces.push(face);
        self.face_alive.push(true);
    }
}

// ============================================================================
// Free functions
// ============================================================================

/// Canonical edge representation: (min, max)
fn sorted_edge(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

/// Compute face normal from vertex positions and face indices
fn face_normal(verts: &[Point3D], face: &[usize; 3]) -> Vector3D {
    let e1 = verts[face[1]] - verts[face[0]];
    let e2 = verts[face[2]] - verts[face[0]];
    e1.cross(&e2)
}

/// Find the closest point on triangle (t0, t1, t2) to point p.
/// Uses the Ericson (2004) real-time collision detection algorithm.
fn closest_point_on_triangle(p: &Point3D, t0: &Point3D, t1: &Point3D, t2: &Point3D) -> Point3D {
    let ab = t1 - t0;
    let ac = t2 - t0;
    let ap = p - t0;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return *t0; // Closest to vertex t0
    }

    let bp = p - t1;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *t1; // Closest to vertex t1
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return Point3D::from(t0.coords + ab * v); // On edge t0-t1
    }

    let cp = p - t2;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *t2; // Closest to vertex t2
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return Point3D::from(t0.coords + ac * w); // On edge t0-t2
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Point3D::from(t1.coords + (t2 - t1) * w); // On edge t1-t2
    }

    // Inside the triangle
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    Point3D::from(t0.coords + ab * v + ac * w)
}

// ============================================================================
// Spatial acceleration for back-projection
// ============================================================================

/// Grid-based spatial index for fast closest-point queries on triangle meshes.
/// Bins triangles by their AABB so we only check nearby triangles.
struct SpatialGrid {
    cell_size: f64,
    min: Point3D,
    /// Map from grid cell (ix, iy, iz) to list of triangle indices
    cells: HashMap<(i32, i32, i32), Vec<usize>>,
}

impl SpatialGrid {
    /// Build spatial grid from triangle array. Cell size should be ~2-3x target edge length.
    fn build(tris: &[[Point3D; 3]], cell_size: f64) -> Self {
        // Compute overall bounds
        let mut bmin = Point3D::new(f64::MAX, f64::MAX, f64::MAX);
        let mut bmax = Point3D::new(f64::MIN, f64::MIN, f64::MIN);
        for tri in tris {
            for v in tri {
                bmin.x = bmin.x.min(v.x);
                bmin.y = bmin.y.min(v.y);
                bmin.z = bmin.z.min(v.z);
                bmax.x = bmax.x.max(v.x);
                bmax.y = bmax.y.max(v.y);
                bmax.z = bmax.z.max(v.z);
            }
        }

        let inv = 1.0 / cell_size;
        let mut cells: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();

        for (ti, tri) in tris.iter().enumerate() {
            // Compute AABB of this triangle
            let mut tmin = tri[0];
            let mut tmax = tri[0];
            for v in &tri[1..] {
                tmin.x = tmin.x.min(v.x); tmin.y = tmin.y.min(v.y); tmin.z = tmin.z.min(v.z);
                tmax.x = tmax.x.max(v.x); tmax.y = tmax.y.max(v.y); tmax.z = tmax.z.max(v.z);
            }

            // Insert into all overlapping cells
            let ix0 = ((tmin.x - bmin.x) * inv).floor() as i32;
            let iy0 = ((tmin.y - bmin.y) * inv).floor() as i32;
            let iz0 = ((tmin.z - bmin.z) * inv).floor() as i32;
            let ix1 = ((tmax.x - bmin.x) * inv).floor() as i32;
            let iy1 = ((tmax.y - bmin.y) * inv).floor() as i32;
            let iz1 = ((tmax.z - bmin.z) * inv).floor() as i32;

            for ix in ix0..=ix1 {
                for iy in iy0..=iy1 {
                    for iz in iz0..=iz1 {
                        cells.entry((ix, iy, iz)).or_default().push(ti);
                    }
                }
            }
        }

        SpatialGrid { cell_size, min: bmin, cells }
    }

    /// Find the closest point on the original surface to query point p.
    /// Searches nearby grid cells in expanding rings until a result is found.
    fn closest_point(&self, p: &Point3D, tris: &[[Point3D; 3]]) -> Option<Point3D> {
        let inv = 1.0 / self.cell_size;
        let cx = ((p.x - self.min.x) * inv).floor() as i32;
        let cy = ((p.y - self.min.y) * inv).floor() as i32;
        let cz = ((p.z - self.min.z) * inv).floor() as i32;

        let mut best_dist_sq = f64::MAX;
        let mut best_point = *p;

        // Search expanding rings of cells. Start with the cell containing p,
        // then expand outward. Stop when we've found a hit and the ring distance
        // exceeds our best distance.
        for ring in 0..10 {
            let ring_dist = (ring as f64 - 1.0).max(0.0) * self.cell_size;
            if ring > 0 && ring_dist * ring_dist > best_dist_sq {
                break; // No closer triangles possible in further rings
            }

            for ix in (cx - ring)..=(cx + ring) {
                for iy in (cy - ring)..=(cy + ring) {
                    for iz in (cz - ring)..=(cz + ring) {
                        // Only process cells on the ring boundary (skip inner cells already processed)
                        if ring > 0 {
                            let on_boundary = ix == cx - ring || ix == cx + ring
                                           || iy == cy - ring || iy == cy + ring
                                           || iz == cz - ring || iz == cz + ring;
                            if !on_boundary { continue; }
                        }

                        if let Some(tri_indices) = self.cells.get(&(ix, iy, iz)) {
                            for &ti in tri_indices {
                                let tri = &tris[ti];
                                let closest = closest_point_on_triangle(p, &tri[0], &tri[1], &tri[2]);
                                let dist_sq = (closest - p).norm_squared();
                                if dist_sq < best_dist_sq {
                                    best_dist_sq = dist_sq;
                                    best_point = closest;
                                }
                            }
                        }
                    }
                }
            }
        }

        if best_dist_sq < f64::MAX { Some(best_point) } else { None }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_unit_cube_mesh() -> Mesh {
        // Simple 12-triangle cube mesh
        let verts = [
            Point3D::new(0.0, 0.0, 0.0), // 0
            Point3D::new(1.0, 0.0, 0.0), // 1
            Point3D::new(1.0, 1.0, 0.0), // 2
            Point3D::new(0.0, 1.0, 0.0), // 3
            Point3D::new(0.0, 0.0, 1.0), // 4
            Point3D::new(1.0, 0.0, 1.0), // 5
            Point3D::new(1.0, 1.0, 1.0), // 6
            Point3D::new(0.0, 1.0, 1.0), // 7
        ];

        let triangles = vec![
            // Bottom (z=0)
            Triangle::new(verts[0], verts[2], verts[1]),
            Triangle::new(verts[0], verts[3], verts[2]),
            // Top (z=1)
            Triangle::new(verts[4], verts[5], verts[6]),
            Triangle::new(verts[4], verts[6], verts[7]),
            // Front (y=0)
            Triangle::new(verts[0], verts[1], verts[5]),
            Triangle::new(verts[0], verts[5], verts[4]),
            // Back (y=1)
            Triangle::new(verts[3], verts[7], verts[6]),
            Triangle::new(verts[3], verts[6], verts[2]),
            // Left (x=0)
            Triangle::new(verts[0], verts[4], verts[7]),
            Triangle::new(verts[0], verts[7], verts[3]),
            // Right (x=1)
            Triangle::new(verts[1], verts[2], verts[6]),
            Triangle::new(verts[1], verts[6], verts[5]),
        ];

        Mesh::new(triangles).unwrap()
    }

    #[test]
    fn test_indexed_mesh_roundtrip() {
        let mesh = make_unit_cube_mesh();
        let indexed = IndexedMesh::from_mesh(&mesh);
        assert_eq!(indexed.vertices.len(), 8);
        assert_eq!(indexed.faces.len(), 12);

        let mesh2 = indexed.to_mesh();
        assert_eq!(mesh2.triangles.len(), 12);
    }

    #[test]
    fn test_isotropic_remesh_cube() {
        let mesh = make_unit_cube_mesh();
        // Use a small target edge length to refine the cube
        let remeshed = isotropic_remesh(&mesh, 0.5, 3);

        // Should have more triangles than original (edges were split)
        assert!(remeshed.triangles.len() > 12,
            "Expected more triangles after remeshing, got {}", remeshed.triangles.len());

        // All triangles should have reasonable area (no degenerates)
        for tri in &remeshed.triangles {
            assert!(tri.area() > 1e-10, "Degenerate triangle found");
        }
    }

    #[test]
    fn test_closest_point_on_triangle() {
        let t0 = Point3D::new(0.0, 0.0, 0.0);
        let t1 = Point3D::new(1.0, 0.0, 0.0);
        let t2 = Point3D::new(0.0, 1.0, 0.0);

        // Point inside triangle
        let p = Point3D::new(0.2, 0.2, 1.0);
        let closest = closest_point_on_triangle(&p, &t0, &t1, &t2);
        assert!((closest.z - 0.0).abs() < 1e-10);
        assert!((closest.x - 0.2).abs() < 1e-10);
        assert!((closest.y - 0.2).abs() < 1e-10);

        // Point closest to vertex
        let p2 = Point3D::new(-1.0, -1.0, 0.0);
        let closest2 = closest_point_on_triangle(&p2, &t0, &t1, &t2);
        assert!((closest2 - t0).norm() < 1e-10);

        // Point closest to edge
        let p3 = Point3D::new(0.5, -1.0, 0.0);
        let closest3 = closest_point_on_triangle(&p3, &t0, &t1, &t2);
        assert!((closest3.y - 0.0).abs() < 1e-10);
        assert!((closest3.x - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_auto_target_edge_length() {
        let mesh = make_unit_cube_mesh();
        let target = auto_target_edge_length(&mesh);
        // Cube diagonal is sqrt(3) ≈ 1.73, 1% = 0.017, clamped to 0.5
        assert!((target - 0.5).abs() < 1e-10);
    }
}
