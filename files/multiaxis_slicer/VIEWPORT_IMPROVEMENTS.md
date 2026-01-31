# 3D Viewport Rendering Improvements

## Overview
Fixed critical rendering issues in the 3D viewport that were causing incomplete model visualization.

## Problems Fixed

### 1. Camera Not Centered on Model
**Problem**: The camera was targeting world origin (0,0,0) instead of the mesh center, causing most models to appear off-screen or only partially visible.

**Fix**: Modified `set_mesh()` to calculate and target the mesh center:
```rust
// Calculate mesh center - this is where the camera should look
let mesh_center = [
    (bounds_min[0] + bounds_max[0]) / 2.0,
    (bounds_min[1] + bounds_max[1]) / 2.0,
    (bounds_min[2] + bounds_max[2]) / 2.0,
];

// Set camera to look at mesh center, not world origin!
self.camera_target = mesh_center;
```

**Impact**: Models now automatically frame correctly in the viewport regardless of their position in world space.

### 2. Camera Distance Too Close
**Problem**: Camera was positioned at `max_dim * 1.5`, which was too close for comfortable viewing.

**Fix**: Increased camera distance to `max_dim * 2.0` for better framing and perspective.

### 3. Missing Wireframe Edges
**Problem**: In wireframe mode, only filled triangles were shown without edge outlines, making mesh structure hard to see.

**Fix**: Added wireframe edge rendering after filled triangles:
```rust
// Draw wireframe edges if enabled
if app.show_wireframe {
    let edge_color = egui::Color32::from_rgb(80, 80, 80);
    let edge_width = 0.5;

    painter.line_segment([tri_data.points[0], tri_data.points[1]], ...);
    painter.line_segment([tri_data.points[1], tri_data.points[2]], ...);
    painter.line_segment([tri_data.points[2], tri_data.points[0]], ...);
}
```

**Impact**: Wireframe mode now properly shows mesh topology with visible edges.

### 4. Limited Debug Information
**Problem**: Users couldn't see detailed information about what was being rendered.

**Fix**: Added mesh center display to viewport HUD:
```rust
painter.text(
    rect.left_top() + egui::vec2(10.0, 60.0),
    egui::Align2::LEFT_TOP,
    format!("Center: ({:.1}, {:.1}, {:.1})", mesh_center[0], mesh_center[1], mesh_center[2]),
    egui::FontId::monospace(12.0),
    egui::Color32::from_rgb(150, 150, 150),
);
```

**Impact**: Users can now see mesh center coordinates for debugging positioning issues.

### 5. Overlapping UI Text
**Problem**: Text labels were overlapping due to incorrect Y offsets after adding new debug info.

**Fix**: Adjusted Y offsets for all viewport info displays:
- Triangles info: 30px
- Size info: 45px
- Center info: 60px
- Toolpath info: 75px, 90px
- Support info: 105px, 120px

**Impact**: Clean, readable viewport HUD without overlapping text.

## How to Use

### Viewing the Complete Model
1. Load any STL file
2. The model will automatically center in the viewport
3. Use camera controls to explore:
   - **Left drag**: Rotate camera around model
   - **Right drag**: Pan camera
   - **Mouse wheel**: Zoom in/out

### Wireframe Mode
1. Enable "Wireframe mode" checkbox in the control panel
2. Mesh edges will be outlined in dark gray
3. Combine with regular shading to see both geometry and topology

### Debug Information
The viewport HUD shows:
- **Triangles**: Visible/Total triangle count (helps identify culling issues)
- **Size**: Model dimensions in mm
- **Center**: Mesh center coordinates in world space
- **Toolpath segments**: When toolpaths are generated
- **Total extrusion**: Material usage
- **Support info**: When supports are enabled

### Face Orientation Mode
When using the "Orient by Face" feature:
- Hovered faces highlight in **bright orange**
- Click to orient that face to the build plate
- Camera controls still work normally

## Technical Details

### Camera System
- **Camera type**: Orbital camera rotating around target point
- **Target**: Mesh center (not world origin)
- **Distance**: Automatically calculated as `max_dimension * 2.0`
- **Default angles**: Yaw=45°, Pitch=30° for good initial view

### Projection
- **Type**: Simple orthographic projection with perspective scaling
- **Scale**: `viewport_size / (camera_distance * 2.0)`
- **Perspective factor**: `1.0 / (1.0 + z / (camera_distance * 2.0))`

### Rendering Pipeline
1. **Transform**: Vertices centered on mesh, then rotated by camera angles
2. **Backface Culling**: Only draw faces with normal.z > 0 (facing camera)
3. **Depth Sorting**: Triangles sorted back-to-front by average Z
4. **Shading**: Directional lighting with ambient + diffuse
5. **Wireframe**: Edge outlines drawn after filled triangles (if enabled)

### Performance
- **Culling**: Backface culling typically removes ~50% of triangles
- **Sorting**: O(n log n) depth sort per frame
- **Drawing**: Software rasterizer using egui painter
- **Expected FPS**: 60fps for models <100k triangles

## Known Limitations

1. **Software Rendering**: All rendering done on CPU, no GPU acceleration
2. **No Clipping**: Triangles extending outside viewport aren't clipped
3. **Simple Shading**: Single directional light, no shadows or reflections
4. **No Anti-aliasing**: Edges may appear jagged at low resolution

## Future Enhancements

Potential improvements for future versions:
- GPU-accelerated rendering using three-d properly
- Proper perspective projection matrix
- Frustum culling for better performance
- Multi-light support
- Shadow mapping
- Ambient occlusion
- Screen-space anti-aliasing (SSAA/MSAA)

## Files Modified

- **src/gui/viewport_3d.rs**: All rendering improvements
  - Fixed camera targeting (line 139)
  - Increased camera distance (line 138)
  - Added wireframe edges (lines 406-424)
  - Added mesh center debug info (lines 505-510)
  - Adjusted UI text positioning (multiple locations)
