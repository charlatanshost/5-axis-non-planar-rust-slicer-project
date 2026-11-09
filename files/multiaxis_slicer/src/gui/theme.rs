// Visual theme and styling for the slicer GUI

use egui::{Color32, Visuals, Style};
use palette::{Srgba, FromColor, Hsv};

/// Color scheme for the slicer
pub struct SlicerColors {
    // Mesh colors
    pub mesh_base: Color32,
    pub mesh_edge: Color32,
    pub mesh_highlight: Color32,

    // Slicing visualization
    pub slicing_plane: Color32,
    pub layer_outline: Color32,

    // Toolpath colors
    pub toolpath_start: Color32,
    pub toolpath_end: Color32,
    pub tool_orientation: Color32,

    // UI elements
    pub panel_bg: Color32,
    pub accent: Color32,
    pub success: Color32,
    pub warning: Color32,
    pub error: Color32,
}

impl Default for SlicerColors {
    fn default() -> Self {
        Self {
            // Mesh
            mesh_base: Color32::from_rgb(180, 180, 180),
            mesh_edge: Color32::from_rgb(100, 150, 220),
            mesh_highlight: Color32::from_rgb(255, 200, 100),

            // Slicing
            slicing_plane: Color32::from_rgba_premultiplied(255, 255, 0, 60),
            layer_outline: Color32::from_rgb(255, 165, 0),

            // Toolpath
            toolpath_start: Color32::from_rgb(100, 255, 100),
            toolpath_end: Color32::from_rgb(255, 100, 100),
            tool_orientation: Color32::from_rgb(0, 255, 255),

            // UI
            panel_bg: Color32::from_rgb(40, 40, 40),
            accent: Color32::from_rgb(0, 120, 215),
            success: Color32::from_rgb(100, 200, 100),
            warning: Color32::from_rgb(255, 200, 0),
            error: Color32::from_rgb(220, 50, 50),
        }
    }
}

// ── Semantic status colours ───────────────────────────────────────────────
//
// Panels used to inline `Color32::from_rgb(...)` at each call site, so the same
// idea — "this is a warning" — was spelled several slightly different ways and
// `SlicerColors` below went unused. Reference these instead, so a palette
// change lands everywhere at once.

/// Something is fine, verified, or complete.
pub const OK: Color32 = Color32::from_rgb(90, 190, 110);
/// Works, but with a caveat the user should know about.
pub const CAUTION: Color32 = Color32::from_rgb(200, 160, 60);
/// Broken, or will not produce a usable result.
pub const BAD: Color32 = Color32::from_rgb(220, 90, 90);
/// Interactive accent, matching the selection fill.
pub const ACCENT: Color32 = Color32::from_rgb(0, 120, 215);

/// Apply dark theme optimized for 3D viewport
pub fn apply_theme(ctx: &egui::Context) {
    let mut style = Style::default();
    let mut visuals = Visuals::dark();

    let colors = SlicerColors::default();

    // Darker background for better 3D contrast
    visuals.window_fill = Color32::from_rgb(30, 30, 30);
    visuals.panel_fill = colors.panel_bg;

    // Accent colour, taken from the palette rather than repeated literals.
    visuals.selection.bg_fill = colors.accent;
    visuals.widgets.active.bg_fill = colors.accent;
    visuals.widgets.hovered.bg_fill = Color32::from_rgb(0, 100, 180);

    // Softer, consistent corners. `Style::default()` leaves these sharp, and
    // the panels never set them, so every widget had square corners regardless
    // of the rest of the styling.
    let rounding = egui::Rounding::same(4.0);
    visuals.widgets.noninteractive.rounding = rounding;
    visuals.widgets.inactive.rounding = rounding;
    visuals.widgets.hovered.rounding = rounding;
    visuals.widgets.active.rounding = rounding;
    visuals.widgets.open.rounding = rounding;
    visuals.window_rounding = egui::Rounding::same(6.0);

    // Breathing room. The control panel stacks a lot of rows, and the egui
    // defaults pack them tightly enough to read as one undifferentiated block.
    style.spacing.item_spacing = egui::vec2(8.0, 6.0);
    style.spacing.button_padding = egui::vec2(8.0, 4.0);
    style.spacing.indent = 16.0;
    style.spacing.slider_width = 140.0;

    style.visuals = visuals;

    ctx.set_style(style);
}

/// Get a gradient color between start and end based on t (0.0 to 1.0)
pub fn gradient_color(start: Color32, end: Color32, t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    Color32::from_rgba_premultiplied(
        lerp(start.r(), end.r(), t),
        lerp(start.g(), end.g(), t),
        lerp(start.b(), end.b(), t),
        lerp(start.a(), end.a(), t),
    )
}

fn lerp(a: u8, b: u8, t: f32) -> u8 {
    (a as f32 * (1.0 - t) + b as f32 * t) as u8
}

/// Generate a rainbow color based on value (0.0 to 1.0)
pub fn rainbow_color(value: f32) -> Color32 {
    let hsv = Hsv::new(value * 360.0, 1.0, 1.0);
    let rgb: Srgba = Srgba::from_color(hsv);
    Color32::from_rgb(
        (rgb.red * 255.0) as u8,
        (rgb.green * 255.0) as u8,
        (rgb.blue * 255.0) as u8,
    )
}
