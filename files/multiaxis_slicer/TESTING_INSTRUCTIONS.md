# Testing Instructions - Viewport Rendering Fix

## What Was Fixed

The viewport scale calculation was corrected to use the actual mesh dimensions instead of camera distance. This should fix the incomplete model rendering issue.

## How to Test

### Step 1: Run the Application

Open PowerShell and run:
```powershell
cd "c:\Users\danie\Documents\Non-planar slicer project\files\multiaxis_slicer"
$env:RUST_LOG="info"
.\target\release\gui.exe
```

### Step 2: Load a Model

1. Click the **"Load Mesh"** button
2. Select any STL file (3DBenchy is a good test model)
3. The model should load and display in the 3D viewport

### Step 3: Check the Visual Result

You should now see:
- ✅ **Complete model** - all parts visible, no missing faces
- ✅ **Proper size** - model fills most of the viewport (not tiny)
- ✅ **Correct centering** - model centered in the viewport
- ✅ **Smooth rotation** - use right-click drag to rotate and verify all angles show completely

### Step 4: Check the Console Output

Look for the diagnostic logs. You should see something like:

**When model loads:**
```
[timestamp INFO] Viewport: Setting mesh bounds
[timestamp INFO]   Bounds min: (-29.18, -15.50, 0.00)
[timestamp INFO]   Bounds max: (30.83, 15.50, 48.00)
[timestamp INFO]   Mesh center: (0.82, 0.00, 24.00)
[timestamp INFO]   Max dimension: 60.00
[timestamp INFO]   Camera distance: 120.00
[timestamp INFO]   Triangles: 37554
```

**When rendering (appears continuously):**
```
[timestamp INFO] Rendering frame:
[timestamp INFO]   Total triangles to draw: 37554
[timestamp INFO]   Triangles on-screen: 35000+     <-- KEY: Should be HIGH
[timestamp INFO]   Triangles off-screen: <2000     <-- KEY: Should be LOW
[timestamp INFO]   Viewport size: 800.0x600.0
[timestamp INFO]   Max mesh dimension: 60.00
[timestamp INFO]   Scale factor: 5.33 pixels/unit  <-- KEY: Should be reasonable (not tiny like 3.33)
```

### Step 5: Check the HUD Display

Look at the top-left corner of the viewport. You should see:
```
Camera: Yaw=0.0° Pitch=30.0° Dist=120.0
Triangles: 37554/37554 visible (0 culled)     <-- All triangles visible!
Size: 60.0x31.0x48.0 mm
Center: (0.8, 0.0, 24.0)
```

## What to Look For

### ✅ Success Indicators:
1. **Complete model visible** - no missing parts
2. **Proper scale** - model fills viewport appropriately (not tiny)
3. **High on-screen triangle count** - 90%+ of triangles on-screen
4. **Scale factor** - around 5-10 pixels/unit (not 3.33 like before)
5. **All rotations show complete model** - rotate 360° and verify

### ❌ Failure Indicators:
1. **Still incomplete** - missing faces or parts
2. **Still too small** - model occupies <25% of viewport
3. **Low on-screen count** - <50% of triangles on-screen
4. **Scale factor still ~3.33** - means the fix didn't apply

## Testing the Face Orientation Feature

Once the viewport is working properly:

1. Click **"Orient by Face"** button in the control panel
2. Status should show: "👆 Click a face to orient"
3. Click on any face of the model
4. The model should rotate so that face is flat on the build plate (facing down)
5. Log should show: "Model oriented. Face is now flat on the build plate."

## Reporting Results

If the fix works:
- ✅ Simply confirm "Viewport is now working correctly!"
- 🎉 You can proceed to use the face orientation feature

If the fix doesn't work:
- Copy and paste the console output (both mesh loading and rendering frame logs)
- Describe what you see visually
- This will help diagnose the next issue

## Additional Controls

While testing:
- **Right-click + drag**: Rotate camera around model
- **Middle-click + drag**: Pan camera
- **Scroll wheel**: Zoom in/out
- **Orient by Face**: Click the button, then click a face to orient model

## Expected Performance

- Models under 50k triangles: Smooth 60 FPS
- Models 50k-100k triangles: 30-60 FPS
- Models over 100k triangles: May be slower (consider using hardware renderer later)

Benchy (~37k triangles) should render smoothly at 60 FPS.
