# Python to Rust Migration Guide

## Overview

This guide helps you migrate your existing Python slicer code to Rust.

## Key Differences

### 1. Type System

**Python:**
```python
def slice_mesh(mesh, layer_height):
    layers = []
    # ...
    return layers
```

**Rust:**
```rust
fn slice_mesh(mesh: &Mesh, layer_height: f64) -> Result<Vec<Layer>> {
    let mut layers = Vec::new();
    // ...
    Ok(layers)
}
```

### 2. Memory Management

**Python:**
```python
# Garbage collected - automatic
points = [Point(x, y, z) for x, y, z in data]
```

**Rust:**
```rust
// Ownership-based - explicit but safe
let points: Vec<Point3D> = data.iter()
    .map(|&(x, y, z)| Point3D::new(x, y, z))
    .collect();
```

### 3. Error Handling

**Python:**
```python
try:
    mesh = load_stl("model.stl")
except Exception as e:
    print(f"Error: {e}")
```

**Rust:**
```rust
match Mesh::from_stl("model.stl") {
    Ok(mesh) => { /* use mesh */ },
    Err(e) => eprintln!("Error: {}", e),
}

// Or with ? operator:
let mesh = Mesh::from_stl("model.stl")?;
```

### 4. Parallelism

**Python:**
```python
from multiprocessing import Pool

with Pool() as pool:
    layers = pool.map(slice_at_height, heights)
```

**Rust:**
```rust
use rayon::prelude::*;

let layers: Vec<Layer> = heights.par_iter()
    .map(|&z| slice_at_height(z))
    .collect();
```

## Common Operations Comparison

### Loading Mesh

**Python (trimesh):**
```python
import trimesh

mesh = trimesh.load("model.stl")
print(f"Triangles: {len(mesh.faces)}")
```

**Rust:**
```rust
use multiaxis_slicer::Mesh;

let mesh = Mesh::from_stl("model.stl")?;
println!("Triangles: {}", mesh.num_triangles());
```

### Triangle-Plane Intersection

**Python:**
```python
def intersect_triangle_plane(tri, z):
    v0, v1, v2 = tri
    # ... math operations
    return line_segment
```

**Rust:**
```rust
impl Triangle {
    fn intersect_plane(&self, z: f64) -> Option<LineSegment> {
        // ... math operations (same logic)
        Some(line_segment)
    }
}
```

### Contour Building

**Python:**
```python
from collections import defaultdict

endpoint_map = defaultdict(list)
for i, segment in enumerate(segments):
    endpoint_map[segment.start].append(i)
    endpoint_map[segment.end].append(i)
```

**Rust:**
```rust
use std::collections::HashMap;

let mut endpoint_map: HashMap<PointKey, Vec<usize>> = HashMap::new();
for (i, segment) in segments.iter().enumerate() {
    endpoint_map.entry(start_key).or_default().push(i);
    endpoint_map.entry(end_key).or_default().push(i);
}
```

## Performance Tips

### 1. Use References to Avoid Copies

**Slow:**
```rust
fn process_mesh(mesh: Mesh) { // Takes ownership, copies data
    // ...
}
```

**Fast:**
```rust
fn process_mesh(mesh: &Mesh) { // Borrows, no copy
    // ...
}
```

### 2. Pre-allocate Vectors

**Python:**
```python
layers = []
for z in heights:
    layers.append(slice_at(z))
```

**Rust:**
```rust
let mut layers = Vec::with_capacity(heights.len());
for &z in &heights {
    layers.push(slice_at(z));
}
```

### 3. Use Iterators Instead of Loops

**Less efficient:**
```rust
let mut sum = 0.0;
for i in 0..points.len() {
    sum += points[i].x;
}
```

**More efficient:**
```rust
let sum: f64 = points.iter().map(|p| p.x).sum();
```

## Common Pitfalls

### 1. String Handling

**Python:**
```python
filename = "model.stl"
path = f"/path/to/{filename}"
```

**Rust:**
```rust
let filename = "model.stl";
let path = format!("/path/to/{}", filename);
// Or use PathBuf for paths:
let path = PathBuf::from("/path/to").join(filename);
```

### 2. Array Indexing

**Python:**
```python
first = points[0]
last = points[-1]
slice = points[1:4]
```

**Rust:**
```rust
let first = points[0];
let last = points[points.len() - 1];
let slice = &points[1..4];
```

### 3. None/Option Handling

**Python:**
```python
result = find_something()
if result is not None:
    use(result)
```

**Rust:**
```rust
let result = find_something();
if let Some(value) = result {
    use_value(value);
}
```

## Migration Checklist

- [ ] Set up Rust development environment
- [ ] Create basic project structure
- [ ] Port geometry primitives (Point, Triangle, etc.)
- [ ] Port mesh loading functionality
- [ ] Port core slicing algorithm
- [ ] Port contour building
- [ ] Add parallel processing with Rayon
- [ ] Port advanced algorithms (centroidal axis, etc.)
- [ ] Add tests for each module
- [ ] Benchmark performance vs Python version
- [ ] Generate documentation

## Debugging Tips

### Print Debugging

**Python:**
```python
print(f"Point: {point}")
```

**Rust:**
```rust
println!("Point: {:?}", point); // Requires #[derive(Debug)]
dbg!(point); // Better for development
```

### Logging

**Python:**
```python
import logging
logging.info("Processing layer")
```

**Rust:**
```rust
use log::info;
info!("Processing layer");

// Initialize in main:
env_logger::init();
```

## Performance Expectations

After migration, you should see:
- **10-100x faster** mesh operations
- **50-100x faster** slicing
- **Much lower** memory usage
- **No GIL issues** for parallel processing
- **Predictable** performance (no GC pauses)

## Next Steps

1. Start with the example: `cargo run --example simple_slicer`
2. Port your Python mesh class to `mesh.rs`
3. Port your slicing algorithm to `slicing.rs`
4. Add tests as you go
5. Profile and optimize

## Resources

- [The Rust Book](https://doc.rust-lang.org/book/)
- [Rust by Example](https://doc.rust-lang.org/rust-by-example/)
- [Rayon Documentation](https://docs.rs/rayon/)
- [nalgebra Documentation](https://docs.rs/nalgebra/)
