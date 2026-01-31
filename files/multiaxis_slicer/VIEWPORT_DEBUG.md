# Viewport Rendering Debug Mode

## Changes Made to Diagnose Incomplete Rendering

### 1. Disabled Backface Culling
**Location**: `src/gui/viewport_3d.rs` line ~322

**Change**:
```rust
// OLD: Strict backface culling
let is_facing_camera = normal[2] > 0.0;

// NEW: Backface culling DISABLED
let is_facing_camera = true;  // Draw ALL triangles
```

**Why**: Backface culling might have been too aggressive, filtering out valid triangles. By disabling it completely, we can see if this was the issue.

**Expected Result**: ALL triangles should now be visible from both sides (double-sided rendering).

### 2. Added Culling Counter
**Location**: `src/gui/viewport_3d.rs`

**Change**: Added `triangles_culled` counter that tracks how many triangles are being filtered out.

**Display**: Shows in viewport HUD as:
```
Triangles: X/Y visible (Z culled)
```

Where:
- X = number rendered
- Y = total triangles in mesh
- Z = number culled (should be 0 with backface culling disabled)

### 3. Added Detailed Logging
**Location**: `src/gui/viewport_3d.rs` in `set_mesh()`

**Logs Output When Model Loads**:
```
Viewport: Setting mesh bounds
  Bounds min: (x, y, z)
  Bounds max: (x, y, z)
  Mesh center: (x, y, z)
  Max dimension: value
  Camera distance: value
  Triangles: count
```

**How to See Logs**:
```powershell
# In PowerShell:
$env:RUST_LOG="info"
cd "c:\Users\danie\Documents\Non-planar slicer project\files\multiaxis_slicer"
cargo run --release --bin gui
```

## What to Check

### When You Load a Model

1. **Check Console Output**
   - Look for "Viewport: Setting mesh bounds" message
   - Verify triangle count matches your STL file
   - Check that mesh center makes sense for your model
   - Confirm camera distance is reasonable (should be ~2x the max dimension)

2. **Check Viewport HUD** (top-left corner)
   - **Triangles**: Should show `X/Y visible (0 culled)` where X ≈ Y
   - If X is much less than Y, something is still filtering triangles
   - **Size**: Should match your model's dimensions
   - **Center**: Should be near the middle of your model's coordinate space

3. **What You Should See**
   - The **COMPLETE model** from both front and back (double-sided)
   - All triangles visible (since backface culling is disabled)
   - Model centered in the viewport
   - Smooth rotation around the model center

### If Model is Still Incomplete

If the model still doesn't show completely, the issue is NOT backface culling. Other possibilities:

1. **Projection Issues**
   - Triangles might be projected outside viewport bounds
   - Scale calculation might be incorrect
   - Check the Center coordinates - if they're way off, that's a clue

2. **Depth Sorting Issues**
   - Triangles might be sorted incorrectly
   - Far triangles might occlude near triangles

3. **STL File Issues**
   - Mesh might have holes or disconnected components
   - Check triangle count in console vs. expected count

4. **Memory/Performance**
   - For very large models (>100k triangles), rendering might be slow
   - Try with a simpler test model first

## Test Models to Try

### Simple Test Cube
Create a cube STL to test basic rendering:
- Expected: 12 triangles (2 per face)
- Should see complete cube with all 6 faces
- Easy to verify if rendering is working

### 3DBenchy
- Expected: ~20,000 triangles typically
- Good real-world test
- Should see boat shape completely

## Reverting Changes

If you want to re-enable backface culling later (for performance):

**In `src/gui/viewport_3d.rs` line ~322**, change:
```rust
// Strict culling (only front faces):
let is_facing_camera = normal[2] > 0.0;

// Lenient culling (front and edge faces):
let is_facing_camera = normal[2] > -0.1;

// No culling (all faces, current setting):
let is_facing_camera = true;
```

## Performance Impact

**Current Setting** (backface culling disabled):
- Renders ~2x more triangles (front + back)
- May be slower on large models (>100k triangles)
- Provides best debugging visibility

**Recommended for Production**:
- Use lenient culling: `normal[2] > -0.1`
- Balances performance with visibility
- Catches edge cases while maintaining speed

## Next Debugging Steps

1. **Run the program with logging enabled**
2. **Load a test model** (start simple, like a cube)
3. **Check the console output** for mesh bounds
4. **Check the viewport HUD** for triangle counts
5. **Report back** with:
   - Triangle count (X/Y visible (Z culled))
   - Console log output
   - Screenshot if possible
   - Model file details (name, expected triangle count)

This will help identify the exact issue!
