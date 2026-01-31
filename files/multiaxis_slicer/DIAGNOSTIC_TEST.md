# Diagnostic Test for Viewport Rendering

## Purpose
Identify why the viewport shows an incomplete model by checking how many triangles are actually being projected onto the screen.

## How to Run the Test

### Step 1: Run with Logging Enabled

In PowerShell:
```powershell
cd "c:\Users\danie\Documents\Non-planar slicer project\files\multiaxis_slicer"
$env:RUST_LOG="info"
.\target\release\gui.exe
```

### Step 2: Load a Model
1. Click "Load Mesh"
2. Select any STL file (3DBenchy recommended for testing)
3. Watch the console output

### Step 3: Capture the Diagnostic Output

You should see two sets of logs:

**When model loads:**
```
Viewport: Setting mesh bounds
  Bounds min: (x, y, z)
  Bounds max: (x, y, z)
  Mesh center: (x, y, z)
  Max dimension: value
  Camera distance: value
  Triangles: count
```

**When rendering (printed every frame while visible):**
```
Rendering frame:
  Total triangles to draw: X
  Triangles on-screen: Y
  Triangles off-screen: Z
  Viewport rect: Rect { ... }
  Scale: value
```

### Step 4: What to Look For

The **KEY DIAGNOSTIC** is the ratio of on-screen vs off-screen triangles:

- **If most triangles are OFF-SCREEN**: The projection/scale calculation is wrong
- **If most triangles are ON-SCREEN but model looks incomplete**: The issue is with triangle clipping or rendering
- **If total triangles to draw is much less than mesh triangles**: Something is filtering them before projection

### Step 5: Report Back

Copy the console output (both the initial mesh loading and a few "Rendering frame" outputs) and share it so we can identify the exact issue.

## What This Will Tell Us

This diagnostic will pinpoint whether:
1. **Scale is too large** - Triangles project way outside viewport bounds
2. **Center offset** - Mesh center calculation is wrong, camera looking at wrong spot
3. **Clipping needed** - Triangles need to be clipped at viewport edges instead of discarded
4. **Other issue** - Something else in the rendering pipeline

Once we see these numbers, we can apply the exact fix needed.
