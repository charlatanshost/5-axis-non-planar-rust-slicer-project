# S3-Slicer Testing Guide

**Last Updated**: January 2026
**Purpose**: Validate Phase 1-3 implementations with real models

---

## 🎯 Testing Objectives

### Phase 1 (Heat Method):
- ✅ Verify geodesic distances are computed correctly
- ✅ Check that scalar field follows mesh topology
- ✅ Validate cotangent Laplacian construction

### Phase 2 (Adaptive Spacing):
- ✅ Confirm layers have consistent thickness
- ✅ Verify binary search converges properly
- ✅ Check fallback to uniform spacing when needed

### Phase 3 (ASAP Deformation):
- ✅ Test ASAP solver convergence
- ✅ Verify rotations are computed via SVD
- ✅ Check deformed mesh quality

---

## 📋 Test Cases

### Test 1: 3DBenchy (Standard Benchmark)

**Model**: `3dbenchy.stl`
**Characteristics**:
- Complex geometry with overhangs
- ~10,000 triangles
- Multiple feature scales
- Industry-standard benchmark

**Test Procedure**:
```bash
# 1. Launch GUI
cargo run --release --bin gui

# 2. Load model
File → Load STL → 3dbenchy.stl

# 3. Configure S3-Slicer
Slicing Mode: "Curved (S3-Slicer)"
Layer Height: 0.2 mm
Objective: Support-Free

# 4. Compute Quaternion Field
Click "Compute Quaternion Field"

# 5. Slice Model
Click "Slice!"
```

**Expected Results**:
- Heat method logs show geodesic distance range
- Adaptive spacing extracts ~100-150 layers
- No scrambled toolpaths
- Layers follow mesh topology

**Log Output to Check**:
```
Computing scalar field for S3-Slicer:
  Using Heat Method for geodesic distances
  Found X source vertices at bottom (Z=...)
  Mesh has Y unique vertices
  Auto time step: ...
  Distance range: [0.00, Z.ZZ]

Extracting layers with adaptive spacing...
  Target thickness: 0.200 mm
  Range: [0.100, 0.400] mm
  Extracted X adaptive layers
```

### Test 2: Simple Cube (Baseline)

**Purpose**: Validate basic functionality with trivial geometry

**Model**: Create programmatically
```rust
// 10x10x10 mm cube
let cube_triangles = create_cube_mesh(10.0, 10.0, 10.0);
let mesh = Mesh::new(cube_triangles)?;
```

**Expected Results**:
- Geodesic distances ≈ Z-heights (flat geometry)
- Adaptive spacing produces uniform layers
- ASAP solver converges quickly (minimal deformation)

### Test 3: Sphere (Smooth Geometry)

**Purpose**: Test on smooth, curved surface

**Characteristics**:
- No sharp features
- Uniform curvature
- Good for stress-testing heat method

**Expected Results**:
- Smooth geodesic distance field
- Even layer distribution
- Fast convergence (low distortion)

### Test 4: Overhang Test

**Purpose**: Validate fabrication objective

**Model**: Part with severe overhang (45°+)

**Expected Results**:
- Quaternion field rotates normals upward
- Heat method follows deformed topology
- Layers avoid steep angles

### Test 5: Large Model (Performance)

**Purpose**: Stress test with large mesh

**Characteristics**:
- 100k+ triangles
- Complex topology

**Expected Results**:
- Heat method: < 5 seconds
- Adaptive spacing: < 10 seconds
- ASAP (if tested): < 30 seconds
- No memory issues

---

## 🔬 Unit Tests

### Heat Method Tests

**File**: `src/s3_slicer/heat_method.rs:426-440`

```rust
#[test]
fn test_cotangent_computation() {
    // Validates cotangent formula
    let p0 = Point3D::new(0.0, 0.0, 0.0);
    let p1 = Point3D::new(1.0, 0.0, 0.0);
    let p2 = Point3D::new(0.5, 0.866, 0.0); // 60°

    let cot = compute_cotangent(&p0, &p1, &p2);
    let expected = 1.0 / (60.0_f64.to_radians().tan());

    assert!((cot - expected).abs() < 0.01);
}
```

**Run**:
```bash
cargo test --release test_cotangent_computation
```

### Isosurface Tests

**File**: `src/s3_slicer/isosurface.rs:465-517`

```rust
#[test]
fn test_edge_interpolation() {
    // Validates linear interpolation
    let p0 = Point3D::new(0.0, 0.0, 0.0);
    let p1 = Point3D::new(10.0, 0.0, 10.0);

    let result = interpolate_edge(&p0, &p1, 0.0, 10.0, 5.0);
    assert!(result.is_some());

    let point = result.unwrap();
    assert!((point.x - 5.0).abs() < 0.01);
    assert!((point.z - 5.0).abs() < 0.01);
}

#[test]
fn test_chain_segments() {
    // Validates segment chaining
    let seg1 = LineSegment { ... };
    let seg2 = LineSegment { ... };

    let paths = chain_segments(&[seg1, seg2], 0.01);
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].len(), 3);
}
```

**Run**:
```bash
cargo test --release isosurface::tests
```

### ASAP Tests

**File**: `src/s3_slicer/asap_deformation.rs:573-604`

```rust
#[test]
fn test_optimal_rotation_identity() {
    // Validates SVD rotation extraction
    let e1 = Vector3D::new(1.0, 0.0, 0.0);
    let e2 = Vector3D::new(0.0, 1.0, 0.0);

    let rotation = compute_optimal_rotation(e1, e2, e1, e2, &qfield, 0, 0.0);

    let identity = Matrix3::identity();
    let diff = (rotation - identity).norm();
    assert!(diff < 0.01);
}
```

**Run**:
```bash
cargo test --release asap_deformation::tests
```

---

## 🚀 Automated Test Script

**File**: `tests/integration_test.rs` (to create)

```rust
use multiaxis_slicer::prelude::*;

#[test]
fn test_full_s3_pipeline_benchy() {
    // Load 3DBenchy
    let mesh = Mesh::load("../3dbenchy.stl")
        .expect("Failed to load 3DBenchy");

    println!("Loaded mesh: {} triangles", mesh.triangles.len());

    // Configure pipeline
    let config = S3PipelineConfig {
        layer_height: 0.2,
        fabrication_objective: FabricationObjective::SupportFree,
        smoothness_weight: 1.0,
        overhang_threshold: 45.0,
    };

    // Execute pipeline
    let result = execute_s3_pipeline(mesh, config);

    // Validate results
    assert!(!result.layers.is_empty(), "No layers generated");
    assert!(result.layers.len() > 50, "Too few layers");
    assert!(result.layers.len() < 200, "Too many layers");

    // Check heat method was used
    assert!(result.deformed_scalar_field.max_value > 0.0);

    // Check adaptive spacing
    // Should have varying layer iso-values
    let iso_values: Vec<f64> = result.curved_layers
        .iter()
        .map(|l| l.iso_value)
        .collect();

    let spacing_variance = compute_variance(&iso_values);
    assert!(spacing_variance > 0.0, "Spacing should be adaptive");

    println!("✅ Pipeline test passed:");
    println!("  - Layers: {}", result.layers.len());
    println!("  - Scalar field range: [{:.2}, {:.2}]",
        result.deformed_scalar_field.min_value,
        result.deformed_scalar_field.max_value
    );
}

#[test]
fn test_heat_method_convergence() {
    // Create simple mesh
    let mesh = create_test_cube(10.0);

    // Configure heat method
    let config = HeatMethodConfig {
        time_step: 0.0, // Auto-compute
        sources: vec![0], // Bottom vertex
        use_implicit: true,
        smoothing_iterations: 3,
    };

    // Compute geodesic distances
    let result = compute_geodesic_distances(&mesh, config);

    // Validate
    assert_eq!(result.distances.len(), count_unique_vertices(&mesh));
    assert!(result.min_distance >= 0.0);
    assert!(result.max_distance > result.min_distance);

    println!("✅ Heat method test passed:");
    println!("  - Distance range: [{:.3}, {:.3}]",
        result.min_distance,
        result.max_distance
    );
}

#[test]
fn test_adaptive_spacing_consistency() {
    let mesh = Mesh::load("../3dbenchy.stl")?;

    // Compute scalar field
    let scalar_field = ScalarField::compute(&mesh, ScalarFieldConfig::default());

    // Extract with adaptive spacing
    let config = IsosurfaceConfig {
        num_layers: 100,
        adaptive_spacing: true,
        target_layer_thickness: 0.2,
        min_layer_thickness: 0.1,
        max_layer_thickness: 0.4,
        ..Default::default()
    };

    let extractor = IsosurfaceExtractor::new(config);
    let layers = extractor.extract_layers(&mesh, &scalar_field);

    // Check layer thickness consistency
    for i in 0..layers.len() - 1 {
        let dist = compute_layer_distance(&layers[i], &layers[i + 1]);
        assert!(dist >= 0.09, "Layer too thin: {}", dist);
        assert!(dist <= 0.41, "Layer too thick: {}", dist);
    }

    println!("✅ Adaptive spacing test passed: {} layers", layers.len());
}

fn create_test_cube(size: f64) -> Mesh {
    // Helper to create cube mesh
    // ... implementation ...
}

fn compute_variance(values: &[f64]) -> f64 {
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    values.iter()
        .map(|v| (v - mean).powi(2))
        .sum::<f64>() / values.len() as f64
}

fn count_unique_vertices(mesh: &Mesh) -> usize {
    // ... implementation ...
}
```

**Run All Tests**:
```bash
cargo test --release
```

---

## 📊 Performance Benchmarks

### Benchmark Suite

**File**: `benches/s3_benchmark.rs` (to create)

```rust
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use multiaxis_slicer::prelude::*;

fn benchmark_heat_method(c: &mut Criterion) {
    let mesh = Mesh::load("../3dbenchy.stl").unwrap();
    let config = HeatMethodConfig::default();

    c.bench_function("heat_method_benchy", |b| {
        b.iter(|| {
            compute_geodesic_distances(black_box(&mesh), black_box(config.clone()))
        });
    });
}

fn benchmark_adaptive_spacing(c: &mut Criterion) {
    let mesh = Mesh::load("../3dbenchy.stl").unwrap();
    let scalar_field = ScalarField::compute(&mesh, ScalarFieldConfig::default());
    let config = IsosurfaceConfig::default();
    let extractor = IsosurfaceExtractor::new(config);

    c.bench_function("adaptive_spacing_benchy", |b| {
        b.iter(|| {
            extractor.extract_layers(black_box(&mesh), black_box(&scalar_field))
        });
    });
}

fn benchmark_asap_deformation(c: &mut Criterion) {
    let mesh = Mesh::load("../3dbenchy.stl").unwrap();
    let quat_field = QuaternionField::optimize(&mesh, QuaternionFieldConfig::default());
    let config = AsapConfig::default();

    c.bench_function("asap_deformation_benchy", |b| {
        b.iter(|| {
            let solver = AsapSolver::new(
                black_box(mesh.clone()),
                black_box(quat_field.clone()),
                black_box(config.clone())
            );
            solver.solve()
        });
    });
}

criterion_group!(
    benches,
    benchmark_heat_method,
    benchmark_adaptive_spacing,
    benchmark_asap_deformation
);

criterion_main!(benches);
```

**Run Benchmarks**:
```bash
cargo bench
```

**Expected Results** (for 3DBenchy ~10k triangles):
- Heat method: 500-1000 ms
- Adaptive spacing: 1-2 seconds
- ASAP deformation: 2-5 seconds

---

## ✅ Test Checklist

### Manual Testing:
- [ ] Load 3DBenchy successfully
- [ ] Compute quaternion field without errors
- [ ] Slice with S3-Slicer mode
- [ ] Verify heat method logs appear
- [ ] Check adaptive spacing logs
- [ ] Inspect visual layer quality
- [ ] Export toolpaths
- [ ] Verify no crashes or memory leaks

### Unit Tests:
- [x] Heat method cotangent test
- [x] Isosurface edge interpolation test
- [x] Segment chaining test
- [x] ASAP rotation extraction test

### Integration Tests:
- [ ] Full S3 pipeline test
- [ ] Heat method convergence test
- [ ] Adaptive spacing consistency test
- [ ] ASAP solver convergence test

### Performance Benchmarks:
- [ ] Heat method benchmark
- [ ] Adaptive spacing benchmark
- [ ] ASAP deformation benchmark

### Regression Tests:
- [ ] Compare output with Phase 0 (simple Z-heights)
- [ ] Verify improvements are measurable
- [ ] Check no degradation in other modes

---

## 🐛 Known Issues / Expected Failures

### Current Limitations:

1. **ASAP Not in Pipeline Yet**
   - ASAP solver exists but not integrated
   - Pipeline still uses scale-controlled deformation
   - **Action**: Manual testing required for ASAP

2. **Large Meshes (>100k triangles)**
   - May be slow (heat method ~5-10 sec)
   - Adaptive spacing may take 20+ seconds
   - **Action**: Performance optimization needed (Task 4)

3. **Extreme Deformations**
   - ASAP may not converge for >90° rotations
   - Heat method assumes smooth scalar field
   - **Action**: Document edge cases

4. **Memory Usage**
   - Large meshes use significant memory
   - Sparse matrices can be 50+ MB
   - **Action**: Monitor for OOM errors

---

## 📈 Quality Metrics

### Metrics to Track:

1. **Layer Quality**
   - Layer thickness variance (should be low with adaptive)
   - Number of empty layers (should be 0)
   - Contour continuity (no gaps)

2. **Performance**
   - Heat method time (< 1 sec for 10k triangles)
   - Adaptive spacing time (< 2 sec for 100 layers)
   - Memory usage (< 100 MB for typical meshes)

3. **Correctness**
   - Geodesic distances monotonic from source
   - Layer iso-values increasing
   - No inverted triangles in deformed mesh

---

## 🎉 Success Criteria

**Phase 1-3 Implementation is Successful if**:

✅ All unit tests pass
✅ 3DBenchy slices without errors
✅ Heat method produces topology-aware distances
✅ Adaptive spacing produces consistent layers
✅ ASAP solver converges in < 10 iterations
✅ No memory leaks or crashes
✅ Performance is acceptable for typical models
✅ Visual quality improves over Phase 0

---

## 📞 Troubleshooting

### Issue: Heat method produces incorrect distances

**Check**:
- Source vertices detected correctly
- Laplacian matrix is symmetric
- Time step is reasonable (not too large)

**Fix**:
- Verify bottom vertex detection threshold
- Check mesh has no isolated components
- Try smaller time step

### Issue: Adaptive spacing produces too few/many layers

**Check**:
- Target thickness setting
- Min/max thickness range
- Binary search convergence

**Fix**:
- Adjust target_layer_thickness
- Widen min/max range
- Increase search iterations

### Issue: ASAP doesn't converge

**Check**:
- Initial mesh quality
- Quaternion field weights
- Convergence threshold

**Fix**:
- Clean up mesh (remove degenerates)
- Reduce quaternion_weight
- Increase max_iterations or convergence_threshold

---

*Created for Phase 1-3 testing and validation*
