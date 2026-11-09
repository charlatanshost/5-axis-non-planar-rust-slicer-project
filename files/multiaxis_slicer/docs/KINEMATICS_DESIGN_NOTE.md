# Configurable machine kinematics — design note

**Status: proposal, not implemented. For review before any code is written.**
This is handoff task T-14, which asks for a design note first.

Target: let a user describe a custom 5-axis machine in the slicer, the way
LinuxCNC kinematics modules or a Fusion 360 machine definition do, and derive
the inverse kinematics from that description rather than hard-coding one
arrangement. Immediate motivating machine: a **tilting head** (both rotary axes
carried on the head, part fixed to the bed), servo-driven.

---

## 1. Why the current model cannot be made correct by configuration

`GCodeGenerator::compute_axis_angles` derives each axis angle independently:

```rust
let ik_angle = |rot_axis: &[f64; 3]| -> f64 {
    if ax.abs() > 0.7 {        ny.atan2(nz).to_degrees() }       // about X
    else if ay.abs() > 0.7 {   (-nx).atan2(nz).to_degrees() }    // about Y
    else {                     ny.atan2(nx).to_degrees() }       // about Z
};
```

Two separate problems are baked in, and neither is expressible in the config.

### 1a. The handedness is hard-coded, and is inverted under the usual convention

Measured, with the default profile (A about X, B about Y, both on the head),
feeding a requested tool direction through `compute_axis_angles` and then
recomposing the rotation with `nalgebra` (right-handed):

| requested direction | A | B | achieved | error |
|---|---|---|---|---|
| (0.71, 0.00, 0.71) — 45° toward +X | 0.00° | −45.00° | (−0.71, 0.00, 0.71) | **90.0°** |
| (0.00, 0.71, 0.71) — 45° toward +Y | 45.00° | 0.00° | (0.00, −0.71, 0.71) | **90.0°** |

A right-handed rotation about **+Y** by `b` maps `ẑ → (sin b, 0, cos b)`, so
reaching +X needs `b = atan2(nx, nz)`. The code computes `atan2(-nx, nz)`, so
the tool tilts the opposite way. The X axis has the same inversion: reaching +Y
needs `a = atan2(-ny, nz)`, and the code computes `atan2(ny, nz)`.

**This is not necessarily wrong for a given machine.** Plenty of real machines
define a positive A or B as tilting the spindle the other way. The defect is
that the convention is hard-coded, undocumented, and unverifiable: nothing in
`AxisConfig`, `GCodeAxis` or the emitted G-code records which sign convention
the machine expects, so there is no way to check the output against the
hardware short of crashing the head into the part.

### 1b. Serial joints are solved as if they were independent

Each `atan2` computes its axis's angle from the tool vector as though every
other axis were at zero. On a serial gimbal the joints compose, so this is only
valid when the tilt lies in a single joint's plane.

Sweeping a 45° tool tilt around the full azimuth and recomposing under both
plausible stack orders:

| stack order | worst orientation error |
|---|---|
| A inner, B outer (`R_y(b) · R_x(a) · ẑ`) | **92.4°** |
| B inner, A outer (`R_x(a) · R_y(b) · ẑ`) | **92.4°** |

Correct serial IK for `n = R_y(b) · R_x(a) · ẑ` is:

```
a = −asin(n_y)
b =  atan2(n_x, n_z)
```

which is not a pair of independent `atan2`s. And which of the two stack orders
applies is a property of how the head is physically built — information the
config has no field for.

### 1c. What else is missing

- **No joint order.** `rotary_axes: Vec<AxisConfig>` is a list, not a chain. For
  a tilting head, whether A carries B or B carries A changes the IK, and the
  list order is not documented as meaningful.
- **No joint offsets.** The only spatial quantity is a scalar
  `tcp_offset`, used as `position + L·(n̂ − ẑ)`. That models one pivot a
  distance `L` above the nozzle along the tool axis. A real head has two pivots
  at different heights, generally with lateral offsets, and the second pivot
  moves when the first rotates. A single scalar cannot express that.
- **Linear axes are not part of the chain.** `workspace_*`, `bed_travel` and
  `head_travel` record travel limits, but nothing says whether Z moves the bed
  or the gantry — which changes nothing for a tilting head but changes
  everything for a machine that tilts the part.
- **`is_head` does only one thing**: negate the angle
  (`raw[i] = if ax.is_head { angle } else { -angle }`). A tilting bed is not a
  sign flip of a tilting head: when the part rotates, the whole workpiece frame
  rotates, so the XYZ path must be transformed, not just the angle negated.
  The current code emits identical XYZ for both machine types.

---

## 2. Proposed model

Describe the machine as **two kinematic branches from a common base**, which is
how both LinuxCNC and Fusion machine definitions treat it:

```
base ──> [part branch]  ──> workpiece origin
     └─> [tool branch]   ──> nozzle tip (TCP)
```

Each branch is an **ordered** list of joints, parent to child. Child joints are
carried by their parents, so a rotation at a parent moves every joint below it.

```rust
pub enum JointKind {
    Linear  { letter: char },           // X, Y, Z, U, V, W
    Rotary  { letter: char },           // A, B, C
    Fixed,                              // a rigid offset only
}

pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    /// Offset from the parent joint's frame to this joint's frame, in the
    /// parent's coordinates, at zero position. This is what `tcp_offset`
    /// cannot express.
    pub origin: [f64; 3],
    /// Motion axis in this joint's own frame: direction of travel for Linear,
    /// rotation axis for Rotary.
    pub axis: [f64; 3],
    /// Sign convention: +1 if positive command = right-handed rotation about
    /// `axis`, -1 if the machine defines it the other way. This is the field
    /// whose absence makes 1a unverifiable.
    pub direction: i8,
    pub min: f64,
    pub max: f64,
    pub max_speed: f64,          // deg/s or mm/s
    pub max_accel: f64,          // deg/s^2 or mm/s^2 — needed for servo tracking
}

pub struct MachineDefinition {
    pub name: String,
    pub tool_branch: Vec<Joint>,   // base -> ... -> nozzle
    pub part_branch: Vec<Joint>,   // base -> ... -> workpiece
}
```

A tilting head is then: `tool_branch = [X, Y, Z, A, B, Fixed(nozzle)]`,
`part_branch = []`. A trunnion machine is `tool_branch = [Z]`,
`part_branch = [X, Y, A, C]`. A hybrid is whatever it is. No code branches on
machine type.

### Forward kinematics

Compose the chain. For the tool branch, with `T_i` the transform of joint `i` at
command `q_i`:

```
T_tool(q) = T_0(q_0) · T_1(q_1) · … · T_n(q_n)
```

The nozzle position is `T_tool · 0` and the tool direction is `T_tool · ẑ`.
Likewise for the part branch. What the slicer needs is the tool pose *in the
workpiece frame*:

```
T_effective = T_part(q_part)⁻¹ · T_tool(q_tool)
```

This single expression replaces the `is_head` sign flip and handles a tilting
bed correctly, because the part branch's inverse transforms the XYZ path too.

### Inverse kinematics

Analytic solutions exist per configuration but multiply the code paths. Given
there are at most 2 rotary degrees of freedom to solve (orientation is 2-DoF —
the tool is rotationally symmetric about its own axis, so C or spindle roll is
free), I propose:

1. **Solve orientation numerically** for the rotary joints — 2 unknowns, smooth,
   with a good initial guess from the previous waypoint. Newton or
   Levenberg-Marquardt converges in a handful of iterations.
2. **Solve position in closed form** afterwards: once the rotaries are fixed,
   `T_effective` is known, so the linear joints follow by subtraction. This is
   the generalisation of TCP compensation and it drops out of the same
   expression instead of being a separate formula.
3. **Pick among multiple solutions** by continuity with the previous waypoint —
   which is what the existing `graph_search.rs` Dijkstra over
   (waypoint, solution) was built to do, and would finally have real solutions
   to choose between.

Numerical IK is affordable here: the collision pass already does hundreds of
thousands of BVH queries in 0.3 s after D-13, and IK is far cheaper per
waypoint.

### The validation invariant

This is the part that would have caught 1a and 1b, and it must exist before any
of the above is trusted:

> For every requested tool pose, running IK then FK must return the requested
> pose to within tolerance.

A round-trip test over a swept azimuth at several tilt angles, for each shipped
machine configuration, asserting < 0.01°. The measurements in section 1 are
exactly this test, failing at 90°.

---

## 3. Migration

`AxisConfig` maps onto `Joint` field-for-field except `origin`, `direction` and
`max_accel`. A `#[serde(default)]` conversion can lift existing profiles into
single-branch definitions, with `is_head` choosing which branch and `direction`
defaulting to whatever reproduces today's output — so saved profiles keep
behaving as they do now, and a user opts into the corrected convention
explicitly. Without that, every existing profile silently changes meaning.

---

## 4. Open questions for Daniel

1. **Stack order on your head**: which rotary is carried by the other? Does A
   tilt a sub-assembly that B then rotates, or the reverse?
2. **Pivot geometry**: distances from the nozzle tip to each rotation axis, and
   any lateral offset between them. Two numbers minimum; four if the axes do not
   intersect.
3. **Sign convention**: for positive A, which way does the nozzle lean? Same for
   B. This is the single most consequential answer, since getting it backwards
   is the 90° error above.
4. **Axis letters**: A and B, or A and C?
5. **Does the tool axis ever need to pass below horizontal** (tilt > 90°)? The
   current tilt budget clamps at 90° and assumes not.
6. Should the machine definition be a separate importable file (like a
   LinuxCNC `.ini` or a Fusion post), or stay embedded in `PrinterProfile`?

---

## 5. Recommendation on sequencing

The G-code currently emitted is very likely commanding the wrong tool direction.
Before this design lands, **do not run generated G-code on hardware** without
checking a few A/B values by hand against the machine's own convention.

The cheapest useful increment, if the full model is too large a step right now:
add `direction: i8` per axis and the FK round-trip test, with analytic IK for
the two-rotary serial case only. That fixes 1a, detects 1b, and is a few hours
rather than a redesign — while being a strict subset of the model above, so it
is not wasted work.
