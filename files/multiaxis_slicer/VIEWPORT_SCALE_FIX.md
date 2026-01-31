# Viewport Scale Calculation Fix

## The Problem

The viewport was showing an incomplete model because the **scale calculation was fundamentally wrong**.

### Previous (Broken) Code:
```rust
let scale = viewport_size / (self.camera_distance * 2.0);
```

For a typical Benchy model:
- `camera_distance = 120.0` (which is `max_dim * 2.0 = 60.0 * 2.0`)
- `viewport_size = 800` pixels (typical)
- **Scale = 800 / 240 = 3.33 pixels/unit**

This means:
- Max mesh dimension: 60 units
- Projected size: 60 × 3.33 = **200 pixels**
- **Problem**: The model only occupied 200 pixels in an 800-pixel viewport!

Most triangles were projecting **way outside** the viewport bounds, getting marked as "off-screen" and not rendered.

## The Fix

### New (Correct) Code:
```rust
// Calculate mesh dimensions
let mesh_size = [
    bounds_max[0] - bounds_min[0],
    bounds_max[1] - bounds_min[1],
    bounds_max[2] - bounds_min[2],
];
let max_dim = mesh_size[0].max(mesh_size[1]).max(mesh_size[2]);

// Calculate scale factor based on mesh size, not camera distance
let viewport_size = rect.width().min(rect.height());
let scale = viewport_size / (max_dim * 2.5);  // 2.5 gives padding
```

For the same Benchy model:
- `max_dim = 60.0` units
- `viewport_size = 800` pixels
- **Scale = 800 / 150 = 5.33 pixels/unit**

This means:
- Max mesh dimension: 60 units
- Projected size: 60 × 5.33 = **320 pixels**
- **Result**: Model fills ~40% of viewport with proper padding

## Why This Matters

The scale determines how world units map to screen pixels. Using `camera_distance` for scale was incorrect because:

1. **Camera distance is for viewing**, not scaling
2. **Camera distance changes** when user zooms (scrolls)
3. **Mesh size is constant** and represents the actual object dimensions

The correct approach is to:
- Calculate the mesh bounding box dimensions
- Use the max dimension to determine how much screen space it needs
- Add padding factor (2.5) to give breathing room around the model

## Expected Results

After this fix, you should see:
- **Complete model rendering** - all triangles visible
- **Proper viewport framing** - model fills most of the viewport
- **Correct proportions** - model maintains its aspect ratio
- **Better on-screen triangle ratio** - diagnostic should show most triangles on-screen

## Testing

Run the program with logging:
```powershell
cd "c:\Users\danie\Documents\Non-planar slicer project\files\multiaxis_slicer"
$env:RUST_LOG="info"
.\target\release\gui.exe
```

Load a model and check:
1. **Visual**: Complete model should be visible and properly sized
2. **Console log**: Should show high on-screen triangle count
3. **HUD display**: Should show proper mesh dimensions

Example expected output:
```
Rendering frame:
  Total triangles to draw: 37554
  Triangles on-screen: 35000+  <-- Most should be on-screen now!
  Triangles off-screen: <2000  <-- Very few off-screen
  Viewport size: 800.0x600.0
  Max mesh dimension: 60.00
  Scale factor: 5.33 pixels/unit  <-- Reasonable scale
```

## Additional Changes

Also improved diagnostic logging to show:
- Viewport dimensions (width × height)
- Max mesh dimension (for context)
- Scale factor in pixels/unit (easier to understand)

This helps verify the scale calculation is working correctly.

## Related Files Modified

- [viewport_3d.rs:232-250](src/gui/viewport_3d.rs#L232-L250) - Scale calculation fix
- [viewport_3d.rs:365-370](src/gui/viewport_3d.rs#L365-L370) - Enhanced diagnostic logging
