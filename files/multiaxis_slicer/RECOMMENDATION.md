# Viewport Rendering Recommendation

## Current Situation

Your project has TWO viewport implementations:
1. **Software Renderer** (`viewport_3d.rs`) - Currently active, using egui painter
2. **Hardware Context** (`three_d::Context`) - Created but unused

## The Problem

The software renderer is showing incomplete models because:
1. Backface culling was too strict (now disabled for debugging)
2. Possible triangle clipping issues (triangles outside viewport bounds)
3. Possible depth sorting problems

## Recommended Solution: Fix the Software Renderer

**Why?** Because it's 90% there and simpler than integrating three-d properly.

### What to Do:

1. **Keep backface culling disabled** (already done)
   - This makes rendering slower but ensures all triangles show

2. **Add proper clipping** - Don't discard triangles, clip them to viewport
   - Currently triangles outside the viewport are just not drawn
   - Should clip triangles at viewport bounds instead

3. **Add bounding sphere culling** - Fast viewport check
   - Only process triangles that could be visible
   - Much faster than per-triangle checks

4. **Optimize depth sorting** - Use spatial data structure
   - Current O(n log n) sort every frame is slow
   - Could use octree or BSP tree for faster sorting

### Implementation:

```rust
// Add bounding sphere test before processing triangles
let mesh_center_screen = project_point(mesh_center);
let mesh_radius_screen = max_dim * scale;  // Approximate

if !rect.contains(mesh_center_screen) &&
   mesh_center_screen.distance(rect.center()) > mesh_radius_screen {
    // Mesh is completely outside viewport, skip rendering
    return;
}

// Process all triangles (backface culling disabled)
for triangle in mesh.triangles {
    // Project all vertices
    let p0 = project_point(v0);
    let p1 = project_point(v1);
    let p2 = project_point(v2);

    // Clip triangle to viewport bounds
    let clipped = clip_triangle_to_rect(p0, p1, p2, rect);

    if let Some(tri) = clipped {
        triangles_to_draw.push(tri);
    }
}
```

## Alternative: Migrate to three-d (More Work)

If you want true hardware acceleration:

### Pros:
- Much faster for large models (>100k triangles)
- Proper depth buffering (no sorting needed)
- Anti-aliasing and other GPU features
- Can render toolpaths and supports efficiently

### Cons:
- Complex integration with egui (requires custom paint callbacks)
- Need to maintain two rendering contexts
- More code to maintain
- Overkill for typical slicer models (<50k triangles)

### Implementation Complexity:

**High**. You need to:
1. Use `egui_glow::CallbackFn` for custom GL rendering
2. Manage GL state transitions between egui and three-d
3. Handle viewport coordination
4. Deal with potential GL context issues

```rust
// Simplified example (actual implementation is 200+ lines)
ui.painter().add(egui::PaintCallback {
    rect,
    callback: Arc::new(egui_glow::CallbackFn::new(move |info, painter| {
        unsafe {
            // Save egui GL state
            // Bind three-d framebuffer
            // Render mesh
            // Restore egui GL state
        }
    })),
});
```

## My Recommendation

**Stick with the software renderer** and fix it properly:

1. ✅ Camera centering (already fixed)
2. ✅ Backface culling (disabled for now)
3. ⬜ Add triangle clipping
4. ⬜ Optimize with bounding sphere test
5. ⬜ Later: Re-enable backface culling with proper threshold

This gives you:
- Complete model visibility (working now with culling disabled)
- Good enough performance for typical models
- Simple, maintainable code
- Works with your current egui integration

The software renderer is fine for models up to ~50k triangles at 60fps. Most slicer models are way below that.

## When to Consider three-d

Only migrate to three-d if:
- Models regularly exceed 100k triangles
- You need anti-aliasing
- You want fancy rendering features (shadows, reflections)
- Performance becomes a real bottleneck

For a slicer, the software renderer is totally adequate.

## Next Steps

1. Test the current version (backface culling disabled)
2. If model still incomplete, add clipping
3. If performance is bad, add bounding sphere culling
4. Once stable, re-enable backface culling with `normal[2] > -0.1`

This gets you a working, complete renderer with minimal additional work.
