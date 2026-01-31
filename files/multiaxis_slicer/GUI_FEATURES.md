# GUI Features Reference

## 🎛️ Control Panel (Now with Scroll Bar!)

The left control panel now has a **vertical scroll bar** to accommodate all features.

---

## 📂 **File Operations**
- **Load STL** - Import 3D mesh
- **Export G-code** - Save generated toolpaths

---

## 🖥️ **Visualization Controls**
Toggle visibility of:
- ✅ Show mesh
- ✅ Show layers
- ✅ Show toolpaths
- ✅ Show supports (orange branches, red/blue nodes)
- ✅ Wireframe mode
- ✅ Show G-code terminal

---

## ⚙️ **Machine Configuration**
- Workspace X, Y, Z limits
- A/B axis ranges (-180° to 180°)
- Max feedrate
- Heated bed toggle

---

## 📐 **Slicing Parameters**

### Slicing Mode
**Dropdown selector:**
- **Planar (Traditional)** - Standard horizontal layers
- **Curved (S3-Slicer)** - Curved layers following geometry

### Layer Settings
- Layer height (0.05 - 0.5 mm)
- Min height (0.05 - 0.3 mm)
- Max height (0.1 - 1.0 mm)
- Adaptive slicing toggle

### Nozzle & Speed
- Nozzle diameter (0.2 - 1.2 mm)
- Feedrate (10 - 300 mm/s)

### **Toolpath Pattern Selection** ⭐ NOW VISIBLE
**Pattern dropdown:**
- **Contour** - Follows layer perimeter
- **Spiral** - Continuous spiral from outside-in
- **Zigzag** - Back-and-forth lines

**Pattern Settings:**
- Line width (0.2 - 2.0 mm)
- Infill density (0-100%, for Zigzag pattern)

---

## 🏗️ **Support Configuration**
- Overhang angle threshold (0-89°)
- Minimum area threshold
- Curved layer analysis toggle

---

## 🎬 **Actions Workflow**

### 1. **Slice Model**
Button: **🔪 Slice!**
- Uses selected slicing mode (Planar or Curved)
- Shows progress for each step
- Generates layers

### 2. **Generate Supports** (Optional)
Button: **🏗 Generate Supports**
- Enabled after slicing
- Detects overhangs
- Creates tree structure
- Shows statistics in log

### 3. **Generate Support Toolpaths** (Optional)
Button: **🔧 Generate Support Toolpaths**
- Appears after support generation
- Creates printable support paths
- Shows layer/segment count

### 4. **Generate Main Toolpaths**
Button: **⚙ Generate Toolpaths**
- Uses selected pattern (Contour/Spiral/Zigzag)
- Shows total moves and pattern used
- Compatible with both slicing modes

### 5. **Run Motion Planning** (Optional)
Button: **🤖 Run Motion Planning**
- Optimizes for 5-axis motion
- Detects singularities and collisions
- Shows waypoint count and statistics

### 6. **Generate G-code**
Button: **📄 Generate G-code**
- Creates final G-code output
- Works with or without motion planning
- Shows warning if motion planning skipped

---

## 📊 **Progress & Status Indicators**

### Real-time Messages
- ✓ Green checkmarks for completed steps
- ⏳ Progress indicators during processing
- 💡 Tips and recommendations
- ⚠️ Warnings for missing steps

### Statistics Display
**After Support Generation:**
- Overhang faces detected
- Overhang area (mm²)
- Support nodes created
- Contact points

**After Toolpath Generation:**
- Total toolpath moves
- Pattern used
- Extrusion amount (if available)

**After Motion Planning:**
- Waypoints processed
- Collisions detected (if any)
- Optimization results

---

## 🎮 **Playback Controls**
- Enable/disable playback mode
- Play/Pause button
- Speed slider (segments per second)
- Current position indicator

---

## 📝 **G-code Preview**
- First 50 lines preview
- Total line count
- Resizable terminal panel

---

## 🎨 **Color Coding**

### 3D Viewport
- **White/Gray** - Mesh surface
- **Green** - Toolpath extrusion moves
- **Light Gray** - Toolpath travel moves
- **Orange** - Support branches (thickness = radius)
- **Red** - Support contact points (model interface)
- **Blue** - Support root nodes (platform contact)

### Status Messages
- **Green** - Success/Complete
- **Yellow/Orange** - Warnings/Tips
- **White** - Information

---

## 💡 **Tips**

1. **Choose your slicing mode first** - Planar for traditional, Curved for advanced
2. **Generate supports before toolpaths** - If your model needs them
3. **Run motion planning** - Recommended for 5-axis optimization
4. **Use the scroll bar** - All features are accessible, just scroll down!
5. **Watch the log** - Progress and statistics appear in the log panel

---

## 🔧 **Troubleshooting**

**Can't see toolpath pattern selector?**
- ✅ **FIXED!** Scroll down in the control panel

**Control panel too long?**
- ✅ **FIXED!** Added vertical scroll bar

**Support visualization not showing?**
- Make sure "Show supports" is checked in Visualization section
- Generate supports first (after slicing)

**Curved slicing not working?**
- Select "Curved (S3-Slicer)" mode BEFORE clicking Slice
- Check log for progress messages

---

## 📈 **Complete Workflow Example**

1. Click **📁 Load STL**
2. Select **Curved (S3-Slicer)** mode (or keep Planar)
3. Adjust **overhang angle** if needed (e.g., 45°)
4. Click **🔪 Slice!** - watch progress
5. Click **🏗 Generate Supports** (if overhangs detected)
6. Click **🔧 Generate Support Toolpaths**
7. Select **Spiral** pattern (or Contour/Zigzag)
8. Click **⚙ Generate Toolpaths**
9. Click **🤖 Run Motion Planning** (recommended)
10. Click **📄 Generate G-code**
11. Click **💾 Export G-code**

Done! Your 5-axis non-planar G-code is ready! 🎉
