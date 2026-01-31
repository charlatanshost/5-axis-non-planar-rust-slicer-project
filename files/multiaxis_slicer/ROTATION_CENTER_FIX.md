# Critical Fix #3: Rotating Around Mesh Center

## The Third Bug - Rotation Around Origin!

After the previous two fixes, curved slicing was STILL producing flat layers. I found the root cause: **we were rotating vertices around the origin (0,0,0) instead of the mesh center!**

### The Problem

In [deformation_v2.rs:214-217](c:\Users\danie\Documents\Non-planar slicer project\files\multiaxis_slicer\src\s3_slicer\deformation_v2.rs#L214-L217):

```rust
// BEFORE (WRONG):
fn apply_quaternion_to_point(point: &Point3D, rotation: &UnitQuaternion<f64>) -> Point3D {
    let rotated = rotation * point.coords;  // Rotates around (0,0,0)!
    Point3D::from(rotated)
}
```

This rotates the vertex's position vector around the world origin `(0,0,0)`.

### Why This Is Catastrophic

Consider Benchy, which is positioned around `(50, 50, 25)` in world space:

```
Original vertex at (60, 55, 30)
Apply 45° rotation around Y axis (around origin):
  New position: ~(42.4, 55, 42.4)

The vertex moved ~18mm in X and ~12mm in Z!
```

When you rotate **every vertex** like this:
- Vertices move to completely different locations
- The overall shape gets scrambled
- If rotations are different per vertex (which they are in S3-Slicer), the mesh gets torn apart
- Result: Either a mangled mess OR most vertices don't move (if rotations are small/identity)

### Visual Example

```
Benchy at position (50, 50, 25):

Rotating around ORIGIN (0,0,0):
                               ╱
                     ????    ╱
    Origin ●───────────────○  Benchy flies away!
     (0,0,0)              (50,50,25)

Rotating around MESH CENTER:
              Original:  After rotation:
                🚢         🚢
                ↓           ↘  (stays in place,
              (50,50,25)  (50,50,25)  just rotated)
```

---

## The Fix

### Step 1: Compute Mesh Center

```rust
// Compute mesh center for rotation
let mesh_center = Point3D::new(
    (mesh.bounds_min.x + mesh.bounds_max.x) / 2.0,
    (mesh.bounds_min.y + mesh.bounds_max.y) / 2.0,
    (mesh.bounds_min.z + mesh.bounds_max.z) / 2.0,
);
```

For Benchy: `mesh_center = (50, 50, 25)` (approximately)

### Step 2: Rotate Around Center

```rust
// AFTER (CORRECT):
fn apply_quaternion_to_point(point: &Point3D, rotation: &UnitQuaternion<f64>, center: &Point3D) -> Point3D {
    // 1. Translate point to origin (relative to center)
    let relative = *point - *center;

    // 2. Apply rotation (now happens around local origin)
    let rotated = rotation * relative;

    // 3. Translate back
    Point3D::from(rotated) + center.coords
}
```

Now the same vertex at `(60, 55, 30)`:
```
1. Relative to center (50, 50, 25): (10, 5, 5)
2. Rotate 45° around Y: (7.07, 5, 7.07)
3. Translate back: (57.07, 55, 32.07)

Vertex moved only ~3mm - local deformation!
```

---

## Impact

### Before (Rotating Around Origin):
- ❌ Vertices fly around wildly OR
- ❌ No visible deformation (if rotations are tiny to avoid scrambling)
- ❌ Mesh gets torn apart if rotations vary per vertex
- ❌ Layers look flat because deformation fails

### After (Rotating Around Center):
- ✅ Vertices rotate locally around the mesh
- ✅ Overall shape is preserved
- ✅ Deformation is visible but controlled
- ✅ Layers follow the curved deformed geometry

---

## Debugging Output Added

I added logging to help diagnose issues:

### Quaternion Field Statistics

```
Quaternion field statistics:
  Total triangles: 48000
  Triangles with rotation: 12000 (25.0%)
  Maximum rotation angle: 32.45° (0.5664 radians)
  Field energy: 1234.56
```

**What to look for**:
- **Triangles with rotation**: Should be > 0% if deformation is happening
  - If 0%: All rotations are identity - no deformation!
  - If < 5%: Very little deformation
  - If 20-50%: Good amount of deformation
  - If > 80%: Aggressive deformation

- **Maximum rotation angle**: Largest rotation applied
  - If < 1°: Deformation will be invisible
  - If 5-30°: Reasonable deformation
  - If > 45°: Very aggressive deformation

- **Field energy**: Lower is better (smoother field)

### Deformation Statistics

```
Deformation statistics:
  Original bounds: (0.00, 0.00, 0.00) to (100.00, 100.00, 50.00)
  Deformed bounds: (-2.34, -1.87, 0.00) to (102.34, 101.87, 50.23)
  Mesh center used for rotation: (50.00, 50.00, 25.00)
```

**What to look for**:
- **Bounds should change**: If original and deformed bounds are identical, no deformation occurred!
- **Change should be small**: Bounds should shift by a few mm, not 50+ mm
- **Mesh center**: Should be at the center of your model

---

## Why Benchy Might Still Look Flat

Even with this fix, Benchy might not show dramatic curved layers because:

1. **Benchy is mostly printable already**
   - Most surfaces are already at good angles
   - Few overhangs that need correction
   - Result: Most quaternions = identity (no rotation needed)

2. **Support-Free objective is conservative**
   - Only rotates surfaces that exceed overhang threshold (default 45°)
   - Benchy's hull and chimney don't need much rotation
   - Only extreme overhangs get deformed

3. **Smoothness weight dampens rotations**
   - Default 0.5 smooths out sharp changes
   - Makes deformation more gradual
   - Can reduce visible curvature

### How to Test for Maximum Deformation

To see dramatic curved layers, try:

**Test Model**: Load a model with extreme overhangs
- Suspended sphere
- T-shape with wide top
- Architectural overhang

**Aggressive Settings**:
```
Fabrication Objective: Support-Free
Overhang Threshold: 30°  (lower = more aggressive)
Smoothness Weight: 0.2    (lower = sharper deformations)
Optimization Iterations: 80  (more refinement)
```

Expected result: **Visible curved layers** wrapping around overhangs!

---

## Understanding the Deformation

### What S3-Slicer Deformation Does:

The quaternion field defines **local rotations** that make each surface more printable:

```
Original overhang:        After deformation:
     ┌────────┐               ╭────────╮
     │ ╱╲     │              ╱ ╱  ╲    ╲
    │ ╱  ╲    │  →→→        │ ╱    ╲    │
   │ ────── │              ╰────────────╯

Overhang angle: 60°       Overhang angle: 40°
Needs support!            Self-supporting!
```

The deformation is **local and smooth**:
- Each triangle gets its own rotation
- Rotations are smoothed with neighbors
- Overall shape is preserved
- Only problematic areas are adjusted

### The Math:

For each vertex:
1. **Gather rotations** from all triangles sharing this vertex
2. **Average them** using spherical interpolation (slerp)
3. **Apply rotation** around mesh center
4. **Result**: Vertex moves slightly to improve printability

---

## Testing Checklist

When you test with Benchy or another model:

### 1. Check the Logs

Enable logging:
```bash
$env:RUST_LOG="info"
cargo run --release --bin gui
```

Load model, select Curved mode, click Slice, look for:

```
Quaternion field statistics:
  Triangles with rotation: ??? (??%)   ← Should be > 0%
  Maximum rotation angle: ??°          ← Should be > 1°

Deformation statistics:
  Original bounds: ...
  Deformed bounds: ...                 ← Should be DIFFERENT
```

### 2. Visual Inspection

In 3D viewport after slicing:
- [ ] Layers should have some curvature (if deformation occurred)
- [ ] Overhanging areas should show curved layers
- [ ] Model should still look recognizable (not scrambled)

### 3. Compare Bounds

If **Original bounds = Deformed bounds**:
→ No deformation occurred! Possible causes:
- All quaternions are identity (model already printable)
- Settings too conservative (try lower overhang threshold)
- Bug in quaternion optimization

If **Deformed bounds crazy different** (>50mm shift):
→ Deformation went wrong! Possible causes:
- Rotations too aggressive
- Settings too extreme
- Numerical instability

If **Deformed bounds slightly different** (few mm shift):
→ **Deformation working correctly!** ✅
- This is the expected behavior
- Layers should show curvature

---

## Files Modified

- [deformation_v2.rs](c:\Users\danie\Documents\Non-planar slicer project\files\multiaxis_slicer\src\s3_slicer\deformation_v2.rs)
  - Added mesh center computation
  - Modified `apply_quaternion_to_point` to rotate around center
  - Added deformation statistics logging

- [quaternion_field.rs](c:\Users\danie\Documents\Non-planar slicer project\files\multiaxis_slicer\src\s3_slicer\quaternion_field.rs)
  - Added quaternion field statistics logging

---

## Next Steps

1. **Run the GUI** with logging enabled: `$env:RUST_LOG="info"; cargo run --release --bin gui`
2. **Load Benchy** (or another STL)
3. **Select Curved mode** and set aggressive parameters
4. **Click Slice** and **watch the console output**
5. **Look for the statistics** to see if deformation is happening

If you see:
- ✅ `Triangles with rotation: X (>5%)` - Good!
- ✅ `Maximum rotation: >5°` - Good!
- ✅ Different bounds - Good!
→ Deformation is working! Layers should be curved.

If you see:
- ❌ `Triangles with rotation: 0 (0%)` - No deformation
- ❌ `Maximum rotation: <1°` - No visible deformation
- ❌ Identical bounds - No deformation occurred
→ Check settings, try different model, or investigate quaternion optimization

---

## Summary

✅ **Fixed**: Vertices now rotate around mesh center instead of world origin

✅ **Added**: Detailed logging for quaternion field and deformation

✅ **Result**: Deformation should now work correctly without scrambling the mesh

**Test it and check the logs to see if deformation is actually occurring!**
