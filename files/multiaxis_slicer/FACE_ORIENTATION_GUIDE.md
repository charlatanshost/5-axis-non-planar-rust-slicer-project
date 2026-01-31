# Face-Based Model Orientation Guide

## Overview
The face-based model orientation feature allows you to quickly orient your 3D model by selecting a face that should be flat against the build plate. This is similar to the "Orient by Face" feature in OrcaSlicer.

## How to Use

### Step 1: Load Your Model
1. Click the **"📁 Load STL"** button in the control panel
2. Select your STL file

### Step 2: Enable Face Orientation Mode
1. Click the **"🔄 Orient by Face"** button in the File Operations section
2. The status message **"👆 Click a face to orient"** will appear in green
3. The viewport is now in face selection mode

### Step 3: Select a Face
1. Move your mouse over the 3D model
2. Faces will be highlighted in **bright orange** as you hover over them
3. Click on the face you want to orient to the build plate
4. The model will automatically rotate so that the selected face is flat on the build plate (facing downward)

### Step 4: Continue with Slicing
- After orientation, you can proceed with slicing as normal
- The model's new orientation will be used for all subsequent operations
- Your slicing state will be reset (layers, toolpaths cleared) since the mesh has changed

## Technical Details

### What Happens Behind the Scenes
1. **Ray Casting**: When you click, a ray is cast from the camera through the click position
2. **Triangle Intersection**: The ray is tested against all visible triangles using the Möller–Trumbore algorithm
3. **Face Selection**: The closest intersected triangle is selected
4. **Rotation Calculation**: A rotation matrix is computed to align the face normal with the -Z axis (build plate normal)
5. **Mesh Transformation**: All vertices are rotated around the mesh center
6. **Bounds Update**: The mesh bounding box is recalculated

### Rotation Method
The rotation uses **axis-angle representation** via `nalgebra::Rotation3`:
- **From**: Selected face normal vector
- **To**: Downward Z-axis (-Z = build plate normal)
- **Rotation Axis**: Cross product of from and to vectors
- **Rotation Angle**: Arccos of dot product

### Edge Cases Handled
- **Parallel vectors**: No rotation needed
- **Anti-parallel vectors**: 180° rotation around perpendicular axis
- **Zero-length axis**: Automatic fallback to perpendicular axis selection

## Visual Feedback
- **Normal mode**: Faces are rendered with blue-tinted shading based on lighting
- **Face orientation mode**: Hovered face is highlighted in **bright orange (#FF9632)**
- **Camera controls**: Still work normally - you can rotate/pan/zoom while in face selection mode

## Camera Controls
- **Left drag**: Rotate camera
- **Right drag**: Pan camera
- **Mouse wheel**: Zoom in/out
- These controls continue to work in face orientation mode (only single clicks trigger face selection)

## Tips
1. **Rotate the view first**: Rotate the camera to get a good view of the face you want to select
2. **Zoom in for precision**: Zoom in on small faces for easier selection
3. **Multiple orientations**: You can orient the model multiple times - just click "Orient by Face" again
4. **Undo**: Currently no undo - reload the model if you make a mistake

## Implementation Files
- **src/mesh.rs**: Added `rotate()` and `orient_face_to_direction()` methods
- **src/gui/app.rs**: Added face orientation state and `apply_face_orientation()` method
- **src/gui/control_panel.rs**: Added "Orient by Face" button and status display
- **src/gui/viewport_3d.rs**: Added ray-triangle intersection and face highlighting

## Future Enhancements
Potential improvements for future versions:
- Undo/redo support for orientations
- Visual preview of rotation before applying
- Auto-orient to minimize overhangs
- Orient to face with maximum area
- Snap to common angles (45°, 90°, etc.)
