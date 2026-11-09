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
pub const OK: Color32 = Color32::from_rgb(86, 196, 134);
/// Works, but with a caveat the user should know about.
pub const CAUTION: Color32 = Color32::from_rgb(226, 170, 76);
/// Broken, or will not produce a usable result.
pub const BAD: Color32 = Color32::from_rgb(232, 104, 104);
/// Interactive accent, matching the selection fill.
pub const ACCENT: Color32 = Color32::from_rgb(62, 142, 232);

// ── Surfaces ──────────────────────────────────────────────────────────────
//
// A three-level surface ramp, slightly blue-tinted rather than neutral grey.
// Pure greys read as flat and dated on a dark UI, and with window, panel and
// widget fills all within a few points of each other there was nothing to
// separate a control from the panel behind it.

/// Deepest surface: behind everything, and the 3D viewport's surround.
const SURFACE_0: Color32 = Color32::from_rgb(22, 24, 29);
/// Panels sit above the window background.
const SURFACE_1: Color32 = Color32::from_rgb(30, 33, 40);
/// Resting state of an interactive widget.
const SURFACE_2: Color32 = Color32::from_rgb(42, 46, 56);
/// Hovered widget.
const SURFACE_3: Color32 = Color32::from_rgb(54, 59, 71);
/// Hairline separators and widget outlines.
const HAIRLINE: Color32 = Color32::from_rgb(58, 63, 75);

/// Primary body text.
const TEXT: Color32 = Color32::from_rgb(226, 230, 238);
/// Secondary text: hints, units, inactive labels.
const TEXT_DIM: Color32 = Color32::from_rgb(150, 158, 172);

/// Resting fill of an interactive widget, for callers that need to match it.
pub fn surface_2() -> Color32 {
    SURFACE_2
}

/// Apply the dark theme.
pub fn apply_theme(ctx: &egui::Context) {
    let mut style = Style::default();
    let mut visuals = Visuals::dark();

    // ── Surfaces ──────────────────────────────────────────────────────
    visuals.window_fill = SURFACE_0;
    visuals.panel_fill = SURFACE_1;
    visuals.extreme_bg_color = SURFACE_0; // text edit / plot backgrounds
    visuals.faint_bg_color = Color32::from_rgb(36, 40, 48); // striped rows

    // ── Widget states ─────────────────────────────────────────────────
    // Each state gets a distinct fill and a matching outline, so a control is
    // visibly a control rather than a word floating on the panel.
    visuals.widgets.noninteractive.bg_fill = SURFACE_1;
    visuals.widgets.noninteractive.weak_bg_fill = SURFACE_1;
    visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, HAIRLINE);
    visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, TEXT);

    // `inactive` is also what egui fills a slider handle with. Left at the
    // same tone as the panel the handle reads as a hollow outline and the
    // control looks disabled, so this sits deliberately brighter than the
    // trough behind it.
    visuals.widgets.inactive.bg_fill = Color32::from_rgb(96, 104, 122);
    visuals.widgets.inactive.weak_bg_fill = SURFACE_2;
    visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, HAIRLINE);
    visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0, TEXT);

    visuals.widgets.hovered.bg_fill = SURFACE_3;
    visuals.widgets.hovered.weak_bg_fill = SURFACE_3;
    visuals.widgets.hovered.bg_stroke = egui::Stroke::new(1.0, ACCENT);
    visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.0, Color32::WHITE);

    visuals.widgets.active.bg_fill = ACCENT;
    visuals.widgets.active.weak_bg_fill = ACCENT;
    visuals.widgets.active.bg_stroke = egui::Stroke::new(1.0, ACCENT);
    visuals.widgets.active.fg_stroke = egui::Stroke::new(1.0, Color32::WHITE);

    visuals.widgets.open.bg_fill = SURFACE_2;
    visuals.widgets.open.weak_bg_fill = SURFACE_2;
    visuals.widgets.open.bg_stroke = egui::Stroke::new(1.0, HAIRLINE);

    visuals.selection.bg_fill = ACCENT.linear_multiply(0.55);
    visuals.selection.stroke = egui::Stroke::new(1.0, Color32::WHITE);
    visuals.hyperlink_color = ACCENT;
    visuals.override_text_color = None;

    // Separators should be hairlines, not slabs.
    visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, HAIRLINE);

    // ── Shape ─────────────────────────────────────────────────────────
    let rounding = egui::Rounding::same(5.0);
    visuals.widgets.noninteractive.rounding = rounding;
    visuals.widgets.inactive.rounding = rounding;
    visuals.widgets.hovered.rounding = rounding;
    visuals.widgets.active.rounding = rounding;
    visuals.widgets.open.rounding = rounding;
    visuals.window_rounding = egui::Rounding::same(8.0);
    visuals.menu_rounding = egui::Rounding::same(6.0);

    // Drop the heavy default drop shadows; a hairline does the separating.
    visuals.window_shadow = egui::epaint::Shadow {
        offset: egui::vec2(0.0, 2.0),
        blur: 12.0,
        spread: 0.0,
        color: Color32::from_black_alpha(90),
    };
    visuals.popup_shadow = visuals.window_shadow;

    // ── Typography ────────────────────────────────────────────────────
    // egui's defaults are uniformly small, so a heading read at the same
    // weight as a slider label and the panel had no hierarchy.
    use egui::{FontFamily::{Monospace, Proportional}, FontId, TextStyle};
    style.text_styles = [
        (TextStyle::Heading, FontId::new(18.0, Proportional)),
        (TextStyle::Body, FontId::new(13.5, Proportional)),
        (TextStyle::Button, FontId::new(13.5, Proportional)),
        (TextStyle::Small, FontId::new(11.5, Proportional)),
        (TextStyle::Monospace, FontId::new(12.5, Monospace)),
    ]
    .into();

    // ── Spacing ───────────────────────────────────────────────────────
    style.spacing.item_spacing = egui::vec2(8.0, 7.0);
    style.spacing.button_padding = egui::vec2(10.0, 5.0);
    style.spacing.menu_margin = egui::Margin::same(6.0);
    style.spacing.indent = 18.0;
    style.spacing.slider_width = 140.0;
    style.spacing.interact_size.y = 24.0; // taller, easier-to-hit controls
    style.spacing.scroll.bar_width = 9.0;

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
