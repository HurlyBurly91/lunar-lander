// Lunar Lander: a playable SDL3 game around a compact binary moon system.
//
// This file is intentionally thin over lander::Simulation: all physics
// (fixed 1/120 s timestep, fuel, the two-body gravity field, body-relative
// landing/crash/takeoff rules, contract loop, determinism) lives in
// include/lander/sim.hpp and lander::Terrain / lander::BinarySystem and is
// covered headlessly by tests/test_sim.cpp and tests/test_binary.cpp. The
// GUI only
//   (a) feeds keyboard input into the simulation,
//   (b) advances it with elapsed real time, and
//   (c) renders both moons, the lander, landing pads, and a HUD.
//
// World space: a fixed inertial frame with the binary barycentre at the
// origin. The two moons orbit it on the analytic ephemeris and are tidally
// locked (M05-R3): each body's terrain spins prograde with the orbit, so a
// body-local angle becomes a world angle by adding the body's rotation.
// Angle 0 = thrust toward +y, positive angle = counter-clockwise. The local
// camera rotates with the selected reference body's surface frame; the
// system view stays inertial.

#include <SDL3/SDL.h>

#include "lander/autopilot.hpp"
#include "lander/binary.hpp"
#include "lander/ballistic.hpp"
#include "lander/camera.hpp"
#include "lander/debug_font.hpp"
#include "lander/debug_subsystem.hpp"
#include "lander/flight_computer.hpp"
#include "lander/guarded_actions.hpp"
#include "lander/landing.hpp"
#include "lander/predictor.hpp"
#include "lander/render_geom.hpp"
#include "lander/sim.hpp"
#include "lander/starfield.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;

// How fast the main-engine throttle moves while its increase/decrease key is
// held. One full 0->1 ramp takes ~1.3 s, which is quick enough to react but
// slow enough to set a fine hover value deliberately.
constexpr double kThrottleRamp = 0.75;

// How long (presentation seconds) the contract-completion banner stays up.
constexpr double kContractBannerTime = 3.0;

// M06-R2-11: the on-screen zero-thrust prediction is the expensive part of a
// rendered frame (a full ~2-binary-period horizon is ~52k fixed steps, worth
// ~18-24 ms in sustained orbit). We rebuild it only at this bounded cadence in
// simulation time (12 Hz) and reuse the cached arc between rebuilds, so a
// 60 fps frame spends ~12/60 of a frame on prediction on average instead of
// every frame. It is rebuilt immediately whenever the inputs that define the
// arc change (maneuver node, target body, or reference body).
constexpr double kPredictRefreshSec = 1.0 / 12.0;

// M06-R8 (R8-06): the per-frame rebuild cap for the low-rate predictor. File
// scope so the expanded predictor panel (which reports the live cost against
// this budget) can reference it alongside the main loop that uses it.
constexpr int kPredictBudget = 240;

// M06-R5 / D05: the two-level transfer midcourse runs a bounded-rate WARM
// re-aim (warm differential correction via plan_transfer) at this cadence in
// simulation time, driving a fast O(1) VGO. 10 Hz keeps each re-aim (~a few ms)
// well within the per-frame budget; a re-aim only re-targets the VGO when the
// required correction exceeds the miss tolerance below.
constexpr double kMidcourseReplanSec = 0.1;
constexpr double kMidcourseMissTolerance = 0.25;

struct Vec2 {
    double x{};
    double y{};

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
};

struct Color {
    Uint8 r{};
    Uint8 g{};
    Uint8 b{};
};

Color make_color(Uint8 r, Uint8 g, Uint8 b) { return {r, g, b}; }

const char* body_name(int index) {
    return index == 0 ? "PRIMARY" : "COMPANION";
}

// ---------------------------------------------------------------- 5x7 font
// A tiny embedded bitmap font (5 wide x 7 tall, bit 4 = leftmost column) so
// the HUD needs no font files or extra dependencies.

struct Glyph {
    Uint8 rows[7];
};

constexpr Glyph kGlyphSpace = {
    {0, 0, 0, 0, 0, 0, 0}};
constexpr Glyph kGlyphA = {
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}};
constexpr Glyph kGlyphB = {
    {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}};
constexpr Glyph kGlyphC = {
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}};
constexpr Glyph kGlyphD = {
    {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}};
constexpr Glyph kGlyphE = {
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}};
constexpr Glyph kGlyphF = {
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}};
constexpr Glyph kGlyphG = {
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E}};
constexpr Glyph kGlyphH = {
    {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}};
constexpr Glyph kGlyphI = {
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}};
constexpr Glyph kGlyphJ = {
    {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C}};
constexpr Glyph kGlyphK = {
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}};
constexpr Glyph kGlyphL = {
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}};
constexpr Glyph kGlyphM = {
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}};
constexpr Glyph kGlyphN = {
    {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}};
constexpr Glyph kGlyphO = {
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}};
constexpr Glyph kGlyphP = {
    {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}};
constexpr Glyph kGlyphQ = {
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}};
constexpr Glyph kGlyphR = {
    {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}};
constexpr Glyph kGlyphS = {
    {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}};
constexpr Glyph kGlyphT = {
    {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}};
constexpr Glyph kGlyphU = {
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}};
constexpr Glyph kGlyphV = {
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}};
constexpr Glyph kGlyphW = {
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}};
constexpr Glyph kGlyphX = {
    {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}};
constexpr Glyph kGlyphY = {
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}};
constexpr Glyph kGlyphZ = {
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}};
constexpr Glyph kGlyphZero = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}};
constexpr Glyph kGlyphOne = {
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}};
constexpr Glyph kGlyphTwo = {
    {0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F}};
constexpr Glyph kGlyphThree = {
    {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}};
constexpr Glyph kGlyphFour = {
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}};
constexpr Glyph kGlyphFive = {
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}};
constexpr Glyph kGlyphSix = {
    {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E}};
constexpr Glyph kGlyphSeven = {
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}};
constexpr Glyph kGlyphEight = {
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}};
constexpr Glyph kGlyphNine = {
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E}};
constexpr Glyph kGlyphDash = {
    {0, 0, 0, 0x0E, 0, 0, 0}};
constexpr Glyph kGlyphPlus = {
    {0, 0x04, 0x04, 0x1F, 0x04, 0x04, 0}};
constexpr Glyph kGlyphDot = {
    {0, 0, 0, 0, 0, 0x0C, 0x0C}};
constexpr Glyph kGlyphColon = {
    {0, 0x0C, 0x0C, 0, 0x0C, 0x0C, 0}};
constexpr Glyph kGlyphSlash = {
    {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10}};
constexpr Glyph kGlyphEquals = {
    {0, 0, 0x1F, 0, 0x1F, 0, 0}};
constexpr Glyph kGlyphPercent = {
    {0x19, 0x1A, 0x04, 0x02, 0x13, 0, 0}};
constexpr Glyph kGlyphBang = {
    {0x04, 0x04, 0x04, 0x04, 0x04, 0, 0x04}};
constexpr Glyph kGlyphLParen = {
    {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02}};
constexpr Glyph kGlyphRParen = {
    {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08}};
constexpr Glyph kGlyphComma = {
    {0, 0, 0, 0, 0, 0x04, 0x08}};
constexpr Glyph kGlyphLBracket = {
    {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E}};
constexpr Glyph kGlyphRBracket = {
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}};

// M06-R11: glyph() renders whatever the debug font understands; the exact
// character set (uppercase + digits + punctuation, with lowercase normalized to
// uppercase) is defined by lander::debug_font so it stays testable headlessly.
const Glyph* glyph(char c) {
    const int n = lander::debug_font::normalize(c);
    switch (n) {
        case 'A': return &kGlyphA;
        case 'B': return &kGlyphB;
        case 'C': return &kGlyphC;
        case 'D': return &kGlyphD;
        case 'E': return &kGlyphE;
        case 'F': return &kGlyphF;
        case 'G': return &kGlyphG;
        case 'H': return &kGlyphH;
        case 'I': return &kGlyphI;
        case 'J': return &kGlyphJ;
        case 'K': return &kGlyphK;
        case 'L': return &kGlyphL;
        case 'M': return &kGlyphM;
        case 'N': return &kGlyphN;
        case 'O': return &kGlyphO;
        case 'P': return &kGlyphP;
        case 'Q': return &kGlyphQ;
        case 'R': return &kGlyphR;
        case 'S': return &kGlyphS;
        case 'T': return &kGlyphT;
        case 'U': return &kGlyphU;
        case 'V': return &kGlyphV;
        case 'W': return &kGlyphW;
        case 'X': return &kGlyphX;
        case 'Y': return &kGlyphY;
        case 'Z': return &kGlyphZ;
        case '0': return &kGlyphZero;
        case '1': return &kGlyphOne;
        case '2': return &kGlyphTwo;
        case '3': return &kGlyphThree;
        case '4': return &kGlyphFour;
        case '5': return &kGlyphFive;
        case '6': return &kGlyphSix;
        case '7': return &kGlyphSeven;
        case '8': return &kGlyphEight;
        case '9': return &kGlyphNine;
        case '-': return &kGlyphDash;
        case '+': return &kGlyphPlus;
        case '.': return &kGlyphDot;
        case ':': return &kGlyphColon;
        case '/': return &kGlyphSlash;
        case '=': return &kGlyphEquals;
        case '%': return &kGlyphPercent;
        case '!': return &kGlyphBang;
        case '(': return &kGlyphLParen;
        case ')': return &kGlyphRParen;
        case ',': return &kGlyphComma;
        case '[': return &kGlyphLBracket;
        case ']': return &kGlyphRBracket;
        default: return &kGlyphSpace;
    }
}

int text_width(const std::string& text, int scale) {
    return static_cast<int>(text.size()) * 6 * scale - scale;
}

void draw_text(SDL_Renderer* renderer, const std::string& text, int x, int y,
               int scale, Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
    for (const char ch : text) {
        const Glyph* g = glyph(ch);
        for (int row = 0; row < 7; ++row) {
            for (int col = 0; col < 5; ++col) {
                if (g->rows[row] & (1u << (4 - col))) {
                    SDL_FRect r{static_cast<float>(x + col * scale),
                                static_cast<float>(y + row * scale),
                                static_cast<float>(scale),
                                static_cast<float>(scale)};
                    SDL_RenderFillRect(renderer, &r);
                }
            }
        }
        x += 6 * scale;
    }
}

void draw_center_text(SDL_Renderer* renderer, const std::string& text, int y,
                      int scale, Color color) {
    draw_text(renderer, text,
              (kWindowWidth - text_width(text, scale)) / 2, y, scale, color);
}

std::string fmt1(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.1f", value);
    return buffer;
}

// M05-R3-14: adaptive zoom formatting. Wide values below 2.0 use two
// decimals so 0.04X / 0.10X / 0.25X / 1.00X stay unambiguous; 2.0 and up
// use one (2.0X, 3.5X, 4.0X).
std::string fmt_zoom(double value) {
    char buffer[32];
    if (value < 2.0) {
        std::snprintf(buffer, sizeof buffer, "%.2f", value);
    } else {
        std::snprintf(buffer, sizeof buffer, "%.1f", value);
    }
    return std::string(buffer) + "X";
}

// ------------------------------------------------------------------- camera

// The single world-to-screen transform. Every piece of world geometry (the
// lander, terrain, pads, guides, debris) is drawn through this, so the camera
// can only rigidly rotate, move, and scale the fixed world-space geometry,
// never change its shape. Coordinates stay floating point here; SDL
// quantizes to pixels.
Vec2 to_screen(double world_x, double world_y, const lander::Camera& cam) {
    // Single source of truth for the world->screen transform (render_geom.hpp);
    // the camera's window size equals kWindowWidth/kWindowHeight here. The
    // pure-geometry API uses lander::Vec2; this adapts it to the renderer's
    // local Vec2.
    const lander::Vec2 p = lander::to_screen_point(world_x, world_y, cam);
    return {p.x, p.y};
}

// Convert a lander::Vec2 polygon (as produced by the pure render geometry)
// into the renderer's local Vec2 form for SDL drawing.
std::vector<Vec2> to_vec2s(const std::vector<lander::Vec2>& in) {
    std::vector<Vec2> out;
    out.reserve(in.size());
    for (const auto& v : in) {
        out.push_back({v.x, v.y});
    }
    return out;
}

// ------------------------------------------------------------- primitives

// Scanline fill for polygons (the terrain body, lander, pads, flame). The
// renderer has no primitive for arbitrary polygons, and this keeps the game
// free of texture assets. Rows outside the window are skipped.
void fill_poly(SDL_Renderer* renderer, const std::vector<Vec2>& pts,
               Color color, Uint8 alpha = 255) {
    if (pts.size() < 3) {
        return;
    }
    // M05-R3-17: a polygon with even one non-finite vertex is rejected rather
    // than pushed through the double->int casts below, so a degenerate or
    // wide-zoom transform can never turn a single bad point into a bogus huge
    // fill (integer overflow / NaN / huge-coordinate conversion).
    double top = 1e30;
    double bottom = -1e30;
    for (const auto& p : pts) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            return;
        }
        top = std::min(top, p.y);
        bottom = std::max(bottom, p.y);
    }
    const double kIntMax = 1.0e9;  // clamp before the int casts (out-of-range
                                   // double -> int is undefined behaviour)
    const int row_begin = std::max(
        0, static_cast<int>(std::floor(std::clamp(top, -kIntMax, kIntMax))));
    const int row_end = std::min(
        kWindowHeight - 1,
        static_cast<int>(std::ceil(std::clamp(bottom, -kIntMax, kIntMax))));
    if (row_end < row_begin) {
        return;
    }
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, alpha);
    const int n = static_cast<int>(pts.size());
    for (int row = row_begin; row <= row_end; ++row) {
        const double yc = row + 0.5;
        std::vector<double> xs;
        for (int i = 0; i < n; ++i) {
            const auto& a = pts[i];
            const auto& b = pts[(i + 1) % n];
            if ((a.y > yc) != (b.y > yc)) {
                const double t = (yc - a.y) / (b.y - a.y);
                const double x = a.x + t * (b.x - a.x);
                if (std::isfinite(x)) {
                    xs.push_back(x);
                }
            }
        }
        std::sort(xs.begin(), xs.end());
        for (size_t i = 0; i + 1 < xs.size(); i += 2) {
            const int x0 = static_cast<int>(
                std::floor(std::clamp(xs[i], -kIntMax, kIntMax)));
            const int x1 = static_cast<int>(
                std::ceil(std::clamp(xs[i + 1], -kIntMax, kIntMax)) - 1);
            if (x1 >= x0) {
                SDL_FRect r{static_cast<float>(x0), static_cast<float>(row),
                            static_cast<float>(x1 - x0 + 1), 1.0f};
                SDL_RenderFillRect(renderer, &r);
            }
        }
    }
}

void draw_thick_line(SDL_Renderer* renderer, const Vec2& a, const Vec2& b,
                     double width, Color color, Uint8 alpha = 255) {
    const Vec2 d{b.x - a.x, b.y - a.y};
    const double len = std::sqrt(d.x * d.x + d.y * d.y);
    if (len < 1e-9) {
        return;
    }
    const Vec2 n{-d.y / len * width / 2.0, d.x / len * width / 2.0};
    fill_poly(renderer, {a + n, b + n, b - n, a - n}, color, alpha);
}

void fill_rect(SDL_Renderer* renderer, int x, int y, int w, int h,
               Color color, Uint8 alpha = 255) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, alpha);
    SDL_FRect r{static_cast<float>(x), static_cast<float>(y),
                static_cast<float>(w), static_cast<float>(h)};
    SDL_RenderFillRect(renderer, &r);
}

// ------------------------------------------------------------------- stars
// The starfield is an inertial celestial backdrop (M05-R3-12): its generated
// base positions (see lander/starfield.hpp) are rotated about the viewport
// centre by the final presentation camera angle, so it turns with the scene
// in both LOCAL and SYSTEM modes, but it never parallax-translates with the
// world and never depends on camera mode, reference body, phase, or pan.
void draw_space(SDL_Renderer* renderer,
                const std::vector<lander::Star>& stars, double camera_angle) {
    fill_rect(renderer, 0, 0, kWindowWidth, kWindowHeight,
              make_color(8, 10, 22));
    for (const lander::Star& star : stars) {
        const lander::ScreenPoint p = lander::star_screen_pos(
            star, kWindowWidth, kWindowHeight, camera_angle);
        fill_rect(renderer, static_cast<int>(p.x), static_cast<int>(p.y),
                  star.size, star.size,
                  make_color(star.bright, star.bright,
                             std::min<Uint8>(255, star.bright + 20)));
    }
}

// ------------------------------------------------------------------ terrain

// Renders one moon: the actual collision surface as a closed arc around the
// body (a local-view window centred on the ship, or the full circle in the
// system view), plus its landing pads. `bpos` is the body's centre in world
// coordinates; the terrain lives in body-local coordinates and every
// terrain angle is rotated by the body's tidal-lock spin (`body_rotation`)
// into world space (M05-R3), so the drawn surface matches exactly what the
// simulation collides with. `highlight_base` draws the destination base pad
// (the pad at arc 0) in amber so the current contract target is easy to
// spot.
void draw_body(SDL_Renderer* renderer, const lander::Body& body,
                const lander::Vec2& bpos, const lander::Camera& cam,
                const lander::State& ship,
                bool highlight_base, double body_rotation) {
    const lander::Terrain& terrain = body.terrain;
    const double scale = cam.scale();
    const double C = terrain.circumference();
    const double theta_ship = std::atan2(ship.y - bpos.y, ship.x - bpos.x);
    // The local-view window is centred on the terrain arc that currently
    // faces the ship: the ship's world direction minus the body's spin.
    const double s_target = terrain.arc_at_angle(theta_ship - body_rotation);

    // M05-R3-23..26: both bodies use the same geometry-driven rule. A body
    // that fits the viewport is drawn as the full closed circle; a partial
    // patch is used only when its closure can be placed outside the viewport.
    const lander::Vec2 ship_pos{ship.x, ship.y};
    const lander::BodyRenderCoverage coverage =
        lander::body_render_coverage(body, bpos, cam, ship_pos, body_rotation);
    const bool full_body = coverage == lander::BodyRenderCoverage::kFull;

    double half_angle = 0.0;
    if (!full_body) {
        // Local view: an arc window centred on the direction from this
        // body's centre to the ship (the M04 framing, now body-relative).
        const double target_r =
            std::max(1.0, std::hypot(ship.x - bpos.x, ship.y - bpos.y));
        half_angle =
            (kWindowWidth / 2.0 + 256.0) / scale / target_r;
        half_angle = std::clamp(half_angle, 0.10, 0.75);
    }

    // The surface is one continuous, shared-vertex mesh (M05-R3-18): an
    // interior fill plus two radial annulus bands built from the same ring
    // vertices. Rendering the rim/pads as single closed polygons, instead of
    // hundreds of independent thick-line quads whose per-segment offsets never
    // met at the joints, removes the black radial seams and the per-segment
    // overdraw that banded the surface at wide zooms (M05-R3-17), while the
    // tessellation count stays bounded at every zoom.
    lander::SurfaceRing ring = lander::body_surface_ring(
        body, bpos, cam, ship_pos, coverage, body_rotation);
    const std::vector<lander::Vec2>& l_outer = ring.outer;
    if (l_outer.size() < 2) {
        return;
    }
    const lander::Vec2 l_centre = lander::to_screen_point(bpos.x, bpos.y, cam);

    fill_poly(renderer,
              to_vec2s(lander::body_fill_polygon(ring, l_centre, coverage,
                                                 cam)),
              make_color(66, 70, 82));

    // Dark inner rim: one continuous 8-px band centred on the inner ring.
    fill_poly(renderer,
              to_vec2s(lander::thick_ring(ring.inner, l_centre, 4.0, 4.0)),
              make_color(58, 62, 74));
    // Light surface edge: one continuous 2.5-px band centred on the surface.
    fill_poly(renderer,
              to_vec2s(lander::thick_ring(ring.outer, l_centre, 1.25, 1.25)),
              make_color(125, 130, 145));

    for (const lander::Pad& pad : terrain.pads()) {
        if (!full_body) {
            const double angular_pad_width =
                pad.half_width / terrain.reference_radius();
            const double angular_distance =
                std::fabs(terrain.normalize_arc(
                              pad.center_arc - s_target + 0.5 * C) -
                          0.5 * C);
            if (angular_distance / terrain.reference_radius() >
                half_angle + angular_pad_width + 0.10) {
                continue;
            }
        }

        const bool is_base = pad.center_arc == 0.0;
        const Color pad_color =
            (is_base && highlight_base)
                ? make_color(255, 196, 64)
                : make_color(72, 210, 120);
        // The pad is one continuous annulus sector (no per-segment seams):
        // a 4-px band on the pad surface and a 1.5-px highlight just above it.
        const int segments = 24;
        std::vector<lander::Vec2> pad_ring;
        std::vector<lander::Vec2> pad_ring_hi;
        pad_ring.reserve(segments);
        pad_ring_hi.reserve(segments);
        for (int i = 0; i < segments; ++i) {
            const double u =
                pad.center_arc - pad.half_width +
                (2.0 * pad.half_width * i / segments);
            const double t = terrain.angle_at_arc(u) + body_rotation;
            pad_ring.push_back(
                lander::to_screen_point(bpos.x + std::cos(t) * pad.radius,
                                        bpos.y + std::sin(t) * pad.radius,
                                        cam));
            pad_ring_hi.push_back(
                lander::to_screen_point(
                    bpos.x + std::cos(t) * (pad.radius + 0.25),
                    bpos.y + std::sin(t) * (pad.radius + 0.25), cam));
        }
        fill_poly(renderer,
                  to_vec2s(lander::thick_ring(pad_ring, l_centre, 2.0, 2.0)),
                  pad_color);
        fill_poly(renderer,
                  to_vec2s(lander::thick_ring(pad_ring_hi, l_centre, 0.75,
                                              0.75)),
                  make_color(205, 255, 220), 220);

        const double tc =
            terrain.angle_at_arc(pad.center_arc) + body_rotation;
        const double guide_len =
            (is_base && highlight_base) ? 30.0 : 14.0;
        const Vec2 guide_top =
            to_screen(bpos.x + std::cos(tc) * (pad.radius + guide_len),
                      bpos.y + std::sin(tc) * (pad.radius + guide_len), cam);
        const Vec2 guide_bottom =
            to_screen(bpos.x + std::cos(tc) * (pad.radius + 0.25),
                      bpos.y + std::sin(tc) * (pad.radius + 0.25), cam);
        draw_thick_line(renderer, guide_top, guide_bottom, 1.0,
                        make_color(205, 255, 220),
                        (is_base && highlight_base) ? 160 : 70);
    }
}

// ----------------------------------------------------------------- lander

// The lander is drawn in its local frame (local +y = the thrust axis, with
// the leg feet at local y = 0 so the feet touch the surface exactly when
// the simulation's reference point reaches the terrain height) and each
// vertex is rotated by the state angle into world space. That keeps the
// on-screen orientation honest under the y-flip: positive angle =
// counter-clockwise on screen. When the correctly scaled hull would be
// smaller than a couple of pixels (system view), a minimum-size marker with
// a heading tick is drawn instead so the ship remains visible.
void draw_lander(SDL_Renderer* renderer, const lander::State& s,
                 double thrust_level, double flame_time,
                 const lander::Camera& cam) {
    const double scale = cam.scale();

    if (!lander::lander_uses_full_model(scale)) {
        // Minimum-size ship marker (system view).
        const Vec2 p = to_screen(s.x, s.y, cam);
        const Color marker =
            s.crashed ? make_color(158, 64, 52) : make_color(235, 240, 250);

        // M05-R4-02: a compact, deterministic thrust plume behind the
        // marker. It is drawn first so the marker and heading tick remain
        // readable over it, and it is suppressed on a crashed ship even if a
        // stale throttle value is present.
        const double plume_throttle = s.crashed ? 0.0 : thrust_level;
        const lander::MarkerPlume plume = lander::marker_plume(
            p.x, p.y, s.angle, cam.angle(), 9.0, plume_throttle);
        if (plume.active) {
            // The renderer's simple colour model has no alpha channel, so the
            // throttle's "intensity" cue is expressed as a deterministic
            // brightness ramp instead of transparency.
            const double t = std::clamp(thrust_level, 0.0, 1.0);
            fill_poly(
                renderer,
                {Vec2{plume.left.x, plume.left.y},
                 Vec2{plume.tip.x, plume.tip.y},
                 Vec2{plume.right.x, plume.right.y}},
                make_color(255, static_cast<Uint8>(96 + 42 * t),
                           static_cast<Uint8>(24 + 14 * t)));
        }

        const lander::MarkerTriangle tri =
            lander::marker_triangle(p.x, p.y, s.angle, cam.angle(), 9.0);
        fill_poly(
            renderer,
            {Vec2{tri.nose.x, tri.nose.y},
             Vec2{tri.left.x, tri.left.y},
             Vec2{tri.right.x, tri.right.y}},
            marker);
        draw_thick_line(renderer, p, {tri.nose.x, tri.nose.y}, 1.0, marker,
                        220);
        return;
    }

    const double c = std::cos(s.angle);
    const double sn = std::sin(s.angle);
    auto local = [&](double lx, double ly) {
        return to_screen(s.x + c * lx - sn * ly, s.y + sn * lx + c * ly, cam);
    };

    const bool crashed = s.crashed;
    const Color body = crashed ? make_color(158, 64, 52)
                               : make_color(206, 211, 223);
    const Color shade = crashed ? make_color(104, 42, 36)
                                : make_color(148, 154, 170);

    if (thrust_level > 0.0) {
        // Flame length is a smooth function of the continuous presentation
        // clock (flame_time), not the integer simulation tick counter, so it
        // no longer stutters in discrete steps at any display refresh rate.
        // The magnitude is still scaled by the throttle: full throttle gives
        // the original flame extent, low throttle gives a short puff.
        const double len = lander::flame_length(thrust_level, flame_time);
        const double width_scale = 0.35 + 0.65 * thrust_level;
        const double outer_half = 0.2 * width_scale;
        const double inner_half = 0.1 * width_scale;
        fill_poly(renderer, {local(-outer_half, 0.18),
                             local(0.0, 0.18 - len),
                             local(outer_half, 0.18)},
                  make_color(255, 138, 38));
        fill_poly(renderer, {local(-inner_half, 0.18),
                             local(0.0, 0.18 - len * 0.55),
                             local(inner_half, 0.18)},
                  make_color(255, 228, 120));
    }

    // Legs and feet.
    draw_thick_line(renderer, local(-0.6, 0.5), local(-1.05, 0.02), 0.09,
                    shade);
    draw_thick_line(renderer, local(0.6, 0.5), local(1.05, 0.02), 0.09,
                    shade);
    fill_poly(renderer, {local(-1.25, 0.0), local(-0.9, 0.0),
                         local(-0.9, 0.09), local(-1.25, 0.09)},
              shade);
    fill_poly(renderer, {local(0.9, 0.0), local(1.25, 0.0),
                         local(1.25, 0.09), local(0.9, 0.09)},
              shade);

    // Engine nozzle, then the hull over it.
    fill_poly(renderer, {local(-0.18, 0.45), local(0.18, 0.45),
                         local(0.0, 0.15)},
              shade);
    fill_poly(renderer, {local(-0.7, 0.45), local(0.7, 0.45),
                         local(0.45, 1.5), local(-0.45, 1.5)},
              body);
    // Top cap and a small viewport for a bit of character.
    fill_poly(renderer, {local(-0.45, 1.5), local(0.45, 1.5),
                         local(0.38, 1.62), local(-0.38, 1.62)},
              shade);
    fill_poly(renderer, {local(-0.12, 1.05), local(0.12, 1.05),
                         local(0.12, 1.32), local(-0.12, 1.32)},
              make_color(110, 175, 215));

    // A short heading line along the thrust axis makes the orientation easy
    // to read at a glance.
    draw_thick_line(renderer, local(0.0, 1.62), local(0.0, 2.4), 0.05,
                    make_color(235, 240, 250), 110);
}

// Small debris field at the impact point; purely cosmetic and deterministic
// from the game seed and the tick the crash happened on. The positions are
// computed relative to the crash body's centre at the (frozen) crash time so
// the debris stays on the wreck even though the moon keeps... not moving
// (time is frozen after a crash, but the convention is kept explicit).
void draw_debris(SDL_Renderer* renderer, const lander::State& s,
                 const lander::Camera& cam, std::uint64_t seed,
                 const lander::Body& body, const lander::Vec2& bpos) {
    std::uint64_t state = seed ^ (s.ticks * 0x9E3779B97F4A7C15ULL);
    const double rx = s.x - bpos.x;
    const double ry = s.y - bpos.y;
    const double theta =
        (rx == 0.0 && ry == 0.0) ? 0.0 : std::atan2(ry, rx);
    const double surface = body.terrain.surface_radius_at_angle(theta);
    const double up_x = std::cos(theta);
    const double up_y = std::sin(theta);
    const double right_x = std::sin(theta);
    const double right_y = -std::cos(theta);
    for (int i = 0; i < 7; ++i) {
        const double u1 =
            static_cast<double>(lander::splitmix64_next(state) >> 11) *
            0x1.0p-53;
        const double u2 =
            static_cast<double>(lander::splitmix64_next(state) >> 11) *
            0x1.0p-53;
        const double u3 =
            static_cast<double>(lander::splitmix64_next(state) >> 11) *
            0x1.0p-53;
        const double angle = u1 * lander::kTwoPi;
        const double radial = 0.05 + u3 * 0.25;
        const double tangential = std::cos(angle) * (0.3 + 1.1 * u2);
        const double wx =
            bpos.x + up_x * (surface + radial) + right_x * tangential;
        const double wy =
            bpos.y + up_y * (surface + radial) + right_y * tangential;
        const Vec2 p = to_screen(wx, wy, cam);
        const int size = 2 + static_cast<int>(u3 * 3);
        fill_rect(renderer, static_cast<int>(p.x), static_cast<int>(p.y),
                  size, size, make_color(140, 90, 78));
    }
}

// --------------------------------------------------------------------- HUD

// M05-R3 compact navigation/gravity overlay: true target, relative-velocity,
// and per-body/net gravity vectors drawn near the ship in screen space.
// Vectors are transformed by the camera's rigid rotation only; their
// on-screen lengths use the documented `cue_arrow_length` scaling (fixed for
// the target direction, linear and clamped for everything else).
void draw_navigation_overlay(SDL_Renderer* renderer,
                              const lander::Simulation& sim,
                              const lander::State& render_state,
                              const lander::Camera& cam) {
    const lander::NavCues cues = lander::navigation_cues(
        sim.state(), sim.binary(), sim.sim_time(),
        sim.contract().destination_body);

    const Vec2 center = to_screen(render_state.x, render_state.y, cam);
    const double c = std::cos(cam.angle());
    const double s = std::sin(cam.angle());

    auto screen_dir = [&](const lander::Vec2& v) -> Vec2 {
        const double len = std::hypot(v.x, v.y);
        if (len < 1.0e-9) {
            return Vec2{};
        }
        const double ux = v.x / len;
        const double uy = v.y / len;
        const double lx = ux * c + uy * s;
        const double ly = -ux * s + uy * c;
        return {lx, -ly};
    };

    auto draw_cue = [&](const lander::Vec2& world, double px_per_unit,
                         double max_px, Color color, const char* label) {
        const double magnitude = std::hypot(world.x, world.y);
        if (magnitude < 1.0e-9) {
            return;
        }
        const double len = lander::cue_arrow_length(magnitude, px_per_unit,
                                                    max_px);
        if (len < 1.0) {
            return;
        }
        const Vec2 dir = screen_dir(world);
        const Vec2 tip{center.x + dir.x * len, center.y + dir.y * len};
        draw_thick_line(renderer, center, tip, 1.5, color, 210);
        const double dx = tip.x - center.x;
        const double dy = tip.y - center.y;
        const double dl = std::hypot(dx, dy);
        if (dl > 1.0e-9) {
            const Vec2 ux{dx / dl, dy / dl};
            const Vec2 perp{-ux.y, ux.x};
            const Vec2 base{tip.x - ux.x * 7.0, tip.y - ux.y * 7.0};
            fill_poly(renderer,
                      {tip,
                       {base.x + perp.x * 3.5, base.y + perp.y * 3.5},
                       {base.x - perp.x * 3.5, base.y - perp.y * 3.5}},
                      color, 210);
            draw_text(renderer, label,
                      static_cast<int>(tip.x) + 4,
                      static_cast<int>(tip.y) - 4, 1, color);
        }
    };

    draw_cue(cues.target_direction, lander::kTargetCueLengthPx,
             lander::kTargetCueLengthPx, make_color(255, 196, 64), "TGT");
    draw_cue(cues.relative_velocity, lander::kVelocityCueScale,
             lander::kCueMaxLengthPx, make_color(120, 220, 255), "VREL");
    draw_cue(cues.gravity_primary, lander::kGravityCueScale,
             lander::kCueMaxLengthPx, make_color(120, 240, 160), "G0");
    draw_cue(cues.gravity_companion, lander::kGravityCueScale,
             lander::kCueMaxLengthPx, make_color(230, 130, 255), "G1");
    draw_cue(cues.net_gravity, lander::kGravityCueScale,
             lander::kCueMaxLengthPx, make_color(255, 230, 120), "GNET");
}

void draw_hud(SDL_Renderer* renderer, const lander::Simulation& sim,
               std::uint64_t seed, double throttle,
               bool reaction_wheels_enabled, bool reaction_wheels_hold,
               const lander::Camera& cam,
               const std::string& tap_progress) {
    const lander::State& s = sim.state();
    const lander::BinarySystem& bin = sim.binary();
    const lander::Config& config = sim.config();

    const Color panel(12, 14, 26);
    const Color white(228, 233, 244);
    const Color dim(130, 138, 156);
    const Color green(120, 240, 160);
    const Color red(255, 92, 80);
    const Color amber(255, 196, 64);

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    fill_rect(renderer, 8, 8, 330, 360, panel, 160);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    draw_text(renderer, "LUNAR LANDER", 18, 16, 3, white);
    char seed_line[48];
    std::snprintf(seed_line, sizeof seed_line, "SEED %llu",
                  static_cast<unsigned long long>(seed));
    draw_text(renderer, seed_line, 18, 44, 2, dim);

    // Explicit reference-frame readouts (M05-R3): REF/JOB identify the
    // frames; ALT, V RAD, V TAN, ATT, SPIN, and ORB are relative to the
    // current REF; DIST, V REL, and RANGE are relative to the contract
    // destination.
    const int ref = sim.reference_body();
    const lander::Vec2 ref_pos = bin.position(ref, sim.sim_time());
    const lander::Vec2 ref_vel = bin.velocity(ref, sim.sim_time());
    const double rot = bin.body_rotation(sim.sim_time());
    const double alt_m = std::max(
        0.0,
        lander::altitude_at(bin.body(ref).terrain, s, ref_pos, rot));
    const lander::LocalVelocity lv =
        lander::local_velocity(s, ref_pos, ref_vel);
    const int throttle_pct =
        static_cast<int>(std::lround(std::clamp(throttle, 0.0, 1.0) * 100.0));
    const int dest = sim.contract().destination_body;
    // The base pad is a point fixed on the destination body's rotating
    // surface (M05-R3): its position and velocity both track the spin, so
    // DIST/VREL/RANGE follow the moving pad, not a static marker.
    const lander::BinarySystem::SurfacePoint dest_base = bin.surface_point(
        dest, bin.body(dest).terrain.angle_at_arc(0.0),
        bin.body(dest).terrain.surface_radius_at_arc(0.0), sim.sim_time());
    const double base_x = dest_base.position.x;
    const double base_y = dest_base.position.y;
    const double dist_m = std::hypot(s.x - base_x, s.y - base_y);
    const double v_rel =
        std::hypot(s.vx - dest_base.velocity.x,
                   s.vy - dest_base.velocity.y);
    const double range_rate = lander::target_range_rate(
        {s.x, s.y}, {s.vx, s.vy}, {base_x, base_y}, dest_base.velocity);
    const char* range_label = lander::range_rate_label(range_rate);
    const double orb_deg =
        lander::orbital_rate(s, ref_pos, ref_vel) * (180.0 / lander::kPi);

    draw_text(renderer, std::string("REF  ") + body_name(ref), 18, 64, 2,
              white);
    draw_text(renderer, std::string("JOB  ") + body_name(dest) + " BASE",
              18, 82, 2, amber);
    draw_text(renderer, "ALT   " + fmt1(alt_m) + " M", 18, 100, 2, white);
    draw_text(renderer, "VRAD  " + fmt1(lv.radial) + " M/S", 18, 118, 2,
              white);
    draw_text(renderer, "VTAN  " + fmt1(lv.tangential) + " M/S", 18, 136, 2,
              white);
    draw_text(renderer,
              "ATT   " +
                  fmt1(lander::local_attitude_angle(s, ref_pos) *
                       180.0 / lander::kPi) +
                  " DEG",
              18, 154, 2, white);
    draw_text(renderer, "SPIN  " + fmt1(lander::spin_deg_per_s(s)) + " D/S",
              18, 172, 2, white);
    draw_text(renderer, "ORB   " + fmt1(orb_deg) + " D/S", 18, 190, 2, white);
    draw_text(renderer, "THR   " + std::to_string(throttle_pct) + "%", 18,
              208, 2, white);

    constexpr int kThrBarX = 146;
    constexpr int kThrBarY = 212;
    constexpr int kThrBarWidth = 154;
    constexpr int kThrBarHeight = 8;
    fill_rect(renderer, kThrBarX, kThrBarY, kThrBarWidth, kThrBarHeight,
              make_color(40, 44, 60));
    const double throttle_frac = std::clamp(throttle, 0.0, 1.0);
    const int thr_bar_w =
        static_cast<int>(std::lround(kThrBarWidth * throttle_frac));
    if (thr_bar_w > 0) {
        fill_rect(renderer, kThrBarX, kThrBarY, thr_bar_w, kThrBarHeight,
                  green);
    }

    char fuel_buffer[48];
    std::snprintf(fuel_buffer, sizeof fuel_buffer, "FUEL  %.1f/%.0f",
                  s.fuel, config.fuel);
    draw_text(renderer, fuel_buffer, 18, 228, 2, white);
    fill_rect(renderer, 18, 248, 180, 12, make_color(40, 44, 60));
    const double frac = config.fuel > 0.0
                            ? std::clamp(s.fuel / config.fuel, 0.0, 1.0)
                            : 0.0;
    const int bar_w = static_cast<int>(180 * frac);
    if (bar_w > 0) {
        fill_rect(renderer, 18, 248, bar_w, 12,
                  frac > 0.25 ? make_color(90, 200, 130) : red);
    }

    draw_text(renderer, "DIST  " + fmt1(dist_m) + " M", 18, 270, 2, white);
    draw_text(renderer, "VREL  " + fmt1(v_rel) + " M/S", 18, 288, 2, white);
    Color range_color = white;
    if (std::strcmp(range_label, "CLOSE") == 0) {
        range_color = green;
    } else if (std::strcmp(range_label, "OPEN") == 0) {
        range_color = amber;
    }
    char rate_buffer[48];
    std::snprintf(rate_buffer, sizeof rate_buffer, "RANGE %+.1f %s",
                  range_rate, range_label);
    draw_text(renderer, rate_buffer, 18, 306, 2, range_color);

    char score_line[32];
    std::snprintf(score_line, sizeof score_line, "SCORE %d", s.score);
    draw_text(renderer, score_line, 18, 324, 2, white);

    std::string status = "IN FLIGHT";
    Color status_color = white;
    if (s.landed) {
        status = "LANDED";
        status_color = green;
    } else if (s.crashed) {
        status = "CRASHED";
        status_color = red;
    }
    draw_text(renderer, status, 18, 342, 2, status_color);

    // M05-R3-07: while a N / O / Shift+O triple-tap sequence is partially
    // armed, its progress is shown under the HUD panel until it fires or
    // expires.
    if (!tap_progress.empty()) {
        draw_text(renderer, tap_progress, 18, 372, 2, amber);
    }

    // Control help along the bottom. The guarded controls only fire on the
    // third tap within 700 ms (M05-R3-07, and M05-R3-10/15/16 for
    // R/B/T).
    draw_text(renderer, "UP/W INC  DN/S DEC  X CUT", 18,
              kWindowHeight - 72, 1, dim);
    draw_text(renderer, "L/R ROT  M CAM  V SYSTEM  G NAV+PATH", 18,
              kWindowHeight - 56, 1, dim);
    draw_text(renderer, "E RW TOG  SH+E RW HOLD  O X3 CIRC  SH+O X3 CCW", 18,
              kWindowHeight - 40, 1, dim);
    draw_text(renderer, "F FUEL  P PAUSE  N X3 SEED  R X3 RETRY", 18,
              kWindowHeight - 24, 1, dim);
    draw_text(renderer, "B X3 SYNC ORBIT  T X3 TRANSFER  WHEEL ZOOM", 18,
              kWindowHeight - 8, 1, dim);
    draw_text(renderer, "C NODE  SH+C OTHER  DEL RM  H/J TIME  K/L DV", 18,
              kWindowHeight - 88, 1, dim);
    draw_text(renderer, "U CIRC  I TRANSFER  Y MATCH  1-8 ATT", 18,
              kWindowHeight - 104, 1, dim);
    draw_text(renderer, "RET EXEC  SH+RET ABORT  X ABORT/CUT  9 LAND", 18,
              kWindowHeight - 120, 1, dim);

    // Camera indicator, top-right.
    std::string cam_line;
    Color cam_color = dim;
    if (cam.mode() == lander::CameraMode::kSystem) {
        cam_line = "CAM SYSTEM " + fmt_zoom(cam.zoom());
        cam_color = amber;
    } else if (cam.mode() == lander::CameraMode::kAuto) {
        cam_line = "CAM AUTO";
    } else {
        cam_line = "CAM MANUAL " + fmt_zoom(cam.zoom());
        cam_color = green;
    }
    const int cam_x =
        kWindowWidth - 8 - static_cast<int>(cam_line.size()) * 6;
    draw_text(renderer, cam_line, cam_x, 16, 1, cam_color);

    // M05-R4-03 / M05-R5: compact reaction-wheel state. While `Shift+E` is
    // held, the indicator reports the transient hold; otherwise it reports
    // the stored `E` toggle. The indicator reports control state, not the
    // instantaneous input: manual rotation suppresses damping for that step
    // without changing either state.
    const std::string rw_line =
        reaction_wheels_hold
            ? "RW HOLD"
            : (reaction_wheels_enabled ? "RW ON" : "RW OFF");
    const Color rw_color =
        reaction_wheels_hold
            ? amber
            : (reaction_wheels_enabled ? green : dim);
    const int rw_x =
        kWindowWidth - 8 - static_cast<int>(rw_line.size()) * 6;
    draw_text(renderer, rw_line, rw_x, 32, 1, rw_color);
}

// ----------------------------------------------------------------- overlay

// A short, non-blocking banner shown while the ship sits on a surface:
// landing is no longer terminal, and the player departs by holding the
// throttle-up key.
void draw_landed_banner(SDL_Renderer* renderer, const lander::State& s) {
    if (!s.landed || s.crashed) {
        return;
    }
    const Color green(120, 240, 160);
    const Color dim(150, 158, 176);
    const Color amber(255, 196, 64);
    draw_center_text(renderer,
                     std::string("LANDED ON ") +
                         body_name(s.landed_body < 0 ? 0 : s.landed_body),
                     kWindowHeight / 2 - 130, 2, green);
    draw_center_text(renderer, "HOLD UP TO TAKE OFF",
                      kWindowHeight / 2 - 104, 2, dim);
    draw_center_text(renderer, "R X3 RETRY   N X3 NEW SEED",
                      kWindowHeight / 2 - 80, 1, amber);
}

// A short banner confirming that the current contract completed and showing
// the next one.
void draw_contract_banner(SDL_Renderer* renderer,
                          const std::optional<lander::Contract>& last,
                          double remaining) {
    if (remaining <= 0.0 || !last.has_value()) {
        return;
    }
    const Color amber(255, 196, 64);
    const Color white(232, 236, 246);
    draw_center_text(renderer, "CONTRACT COMPLETE  +" +
                                   std::to_string(last->reward),
                     kWindowHeight / 2 - 150, 3, amber);
    draw_center_text(
        renderer,
        std::string("NEXT JOB: ") +
            body_name(1 - last->destination_body) + " BASE",
        kWindowHeight / 2 - 122, 2, white);
}

// The terminal states get the M04 treatment: a dimmed screen and a restart
// prompt. The crash report is a compact, content-sized modal (M05-R3-08):
// centred in the screen, independent of terrain, reference body, camera
// mode, and SYSTEM zoom.
void draw_overlay(SDL_Renderer* renderer, const lander::State& s,
                  bool paused) {
    if (!paused && !s.landed && !s.crashed) {
        return;
    }
    const Color white(232, 236, 246);
    const Color dim(150, 158, 176);
    const Color red(255, 92, 80);

    if (paused && !s.landed && !s.crashed) {
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        fill_rect(renderer, 0, 0, kWindowWidth, kWindowHeight,
                  make_color(4, 5, 10), 120);
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
        draw_center_text(renderer, "PAUSED", kWindowHeight / 2 - 70, 6, white);
        draw_center_text(renderer, "PRESS P TO RESUME",
                         kWindowHeight / 2 + 10, 2, dim);
        return;
    }

    if (!s.crashed) {
        return;
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    fill_rect(renderer, 0, 0, kWindowWidth, kWindowHeight,
              make_color(4, 5, 10), 120);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    const std::string score_line = "SCORE " + std::to_string(s.score);
    const std::string hint = "R RETRY   N X3 NEW SEED";
    const lander::ScreenRect box = lander::crash_modal_rect(
        7, 6, static_cast<int>(score_line.size()), 3,
        static_cast<int>(hint.size()), 2, kWindowWidth, kWindowHeight);
    fill_rect(renderer, box.x, box.y, box.w, box.h,
              make_color(18, 20, 34), 235);
    // The vertical layout mirrors crash_modal_rect: 24 px top padding, the
    // scale-6 title, a 16 px gap, the scale-3 score, a 16 px gap, then the
    // scale-2 hint.
    auto center_in_box = [&](const std::string& text, int scale) {
        return box.x + (box.w - text_width(text, scale)) / 2;
    };
    draw_text(renderer, "CRASHED", center_in_box("CRASHED", 6), box.y + 24,
              6, red);
    draw_text(renderer, score_line, center_in_box(score_line, 3),
              box.y + 24 + 7 * 6 + 16, 3, white);
    draw_text(renderer, hint, center_in_box(hint, 2),
              box.y + 24 + 7 * 6 + 16 + 7 * 3 + 16, 2, dim);
}

// ------------------------------------------------------------------- misc

bool write_ppm(SDL_Renderer* renderer, const char* path) {
    // SDL3's SDL_RenderReadPixels returns a newly allocated surface in the
    // renderer's native format; convert it to 24-bit RGB (bytes per pixel
    // in R,G,B order) and copy it pixel by pixel, so the PPM rows are
    // plain triples regardless of the source layout or row pitch.
    SDL_Surface* rendered = SDL_RenderReadPixels(renderer, nullptr);
    if (rendered == nullptr) {
        return false;
    }
    SDL_Surface* surface =
        SDL_ConvertSurface(rendered, SDL_PIXELFORMAT_RGB24);
    SDL_DestroySurface(rendered);
    if (surface == nullptr || surface->w != kWindowWidth ||
        surface->h != kWindowHeight) {
        if (surface != nullptr) {
            SDL_DestroySurface(surface);
        }
        return false;
    }
    const int bpps = SDL_BYTESPERPIXEL(surface->format);
    if (bpps < 3) {
        SDL_DestroySurface(surface);
        return false;
    }
    bool ok = false;
    FILE* file = std::fopen(path, "wb");
    if (file != nullptr) {
        std::fprintf(file, "P6\n%d %d\n255\n", kWindowWidth, kWindowHeight);
        const std::size_t pitch = static_cast<std::size_t>(surface->pitch);
        std::vector<Uint8> row_bytes(3 * static_cast<std::size_t>(kWindowWidth));
        for (int row = 0; row < kWindowHeight; ++row) {
            const Uint8* src =
                static_cast<const Uint8*>(surface->pixels) + row * pitch;
            for (int col = 0; col < kWindowWidth; ++col) {
                std::memcpy(&row_bytes[col * 3], src + col * bpps, 3);
            }
            std::fwrite(row_bytes.data(), 1, row_bytes.size(), file);
        }
        std::fclose(file);
        ok = true;
    }
    SDL_DestroySurface(surface);
    return ok;
}

std::uint64_t random_seed() {
    std::random_device device;
    return ((std::uint64_t{device()} << 32) |
            static_cast<std::uint64_t>(device())) ^
           static_cast<std::uint64_t>(SDL_GetTicksNS());
}

void print_usage() {
    std::printf(
        "Usage: lander_gui [options]\n"
        "  --seed N         start with a deterministic seed (default: random)\n"
        "  --frames N       render exactly N frames, then exit\n"
        "  --fps N          cap the frame rate to N frames/second\n"
        "  --screenshot F   save the final frame to F as a PPM image\n"
        "  --orbit-demo     start in a terrain-clearing circular orbit around\n"
        "                   the primary (developer mode)\n"
        "  --system-view    start in the inertial system-scale view of the whole\n"
        "                   binary (developer mode; the V key toggles it)\n"
        "  --debug-subsystem NAME  isolate one M06 subsystem for inspection\n"
        "                   (developer mode; a focused panel replaces the\n"
        "                   flight-computer panel and a deterministic startup\n"
        "                   fixture exercises that subsystem). Valid names:\n"
        "                   none, manual, predictor, attitude, node-edit,\n"
        "                   node-executor, transfer-cold, transfer-warm,\n"
        "                   autoland-primary, autoland-companion,\n"
        "                   autoland-cross, ui\n"
        "  --help           show this message\n"
        "Controls: Up/W increase throttle, Down/S decrease throttle, X\n"
        "throttle cutoff, Left/Right/A/D rotate, M camera mode (Auto/Manual;\n"
        "from System enters Manual), V system view, mouse wheel zoom, O x3\n"
        "clockwise circularize around the reference body, Shift+O x3\n"
        "counter-clockwise circularize, E toggle reaction-wheel damping,\n"
        "Shift+E hold reaction-wheel damping while pressed, G\n"
        "navigation/gravity overlay, F refill fuel (also on the ground), N x3\n"
        "new seed, P pause, Esc/Q quit.\n"
        "Developer controls (all triple-tap guarded, progress shown on the\n"
        "HUD, firing on the third tap within 700 ms):\n"
        "  R x3  retry the same seed\n"
        "  B x3  body-synchronous orbit around the source body (the landed\n"
        "        body when landed, else the reference body), on the side\n"
        "        opposite the other body\n"
        "  T x3  ballistic inter-body transfer to the other body; shows\n"
        "        'TRANSFER: NO SOLUTION' if none exists for this phase\n"
        "Flight computer: C place a node 5 s ahead on the reference body,\n"
        "Shift+C place it on the other body, Delete remove it, H/J move it\n"
        "-/+ 1 s, K/Shift+K and L/Shift+L adjust prograde/radial DV by\n"
        "+/- 0.1 m/s, U plan circularize, I plan transfer, Y plan velocity\n"
        "match, 1-8 select OFF/PROGRADE/RETRO/RAD OUT/RAD IN/TARGET/ANTI-\n"
        "TARGET/MANEUVER attitude, Return execute the node, Shift+Return\n"
        "abort the executor, 9 arm the target-pad autoland for the reference\n"
        "body (manual throttle or X reclaims control), X aborts/cuts the\n"
        "engine, G shows the nav overlay and the predicted pre/post-node path.\n");
}

void draw_line_simple(SDL_Renderer* renderer, const Vec2& a, const Vec2& b,
                      Color color) {
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) ||
        !std::isfinite(b.y)) {
        return;
    }
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
    SDL_RenderLine(renderer, static_cast<float>(a.x), static_cast<float>(a.y),
                   static_cast<float>(b.x), static_cast<float>(b.y));
}

void draw_trajectory(SDL_Renderer* renderer,
                      const lander::TrajectoryPrediction& prediction,
                      const lander::Camera& cam, bool has_node,
                      const lander::Simulation& sim) {
    auto draw_path = [&](const std::vector<lander::Vec2>& points, Color color) {
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            const Vec2 a = to_screen(points[i].x, points[i].y, cam);
            const Vec2 b = to_screen(points[i + 1].x, points[i + 1].y, cam);
            draw_line_simple(renderer, a, b, color);
        }
    };

    draw_path(prediction.pre, make_color(185, 195, 215));
    draw_path(prediction.post, make_color(120, 240, 160));

    if (has_node) {
        const Vec2 p =
            to_screen(prediction.node_position.x, prediction.node_position.y,
                      cam);
        const Color node_color = make_color(230, 130, 255);
        draw_thick_line(renderer, {p.x - 6, p.y}, {p.x + 6, p.y}, 1.5,
                        node_color);
        draw_thick_line(renderer, {p.x, p.y - 6}, {p.x, p.y + 6}, 1.5,
                        node_color);
    }

    if (prediction.impact.valid) {
        const Vec2 p = to_screen(prediction.impact.position.x,
                                  prediction.impact.position.y, cam);
        const Color red = make_color(255, 92, 80);
        draw_thick_line(renderer, {p.x - 5, p.y - 5}, {p.x + 5, p.y + 5}, 1.5,
                        red);
        draw_thick_line(renderer, {p.x - 5, p.y + 5}, {p.x + 5, p.y - 5}, 1.5,
                        red);
    }

    if (prediction.closest.valid) {
        const Vec2 p = to_screen(prediction.closest.target_position.x,
                                  prediction.closest.target_position.y, cam);
        fill_rect(renderer, static_cast<int>(p.x) - 3,
                  static_cast<int>(p.y) - 3, 6, 6, make_color(255, 196, 64));
        // M06-R11: the amber square is the closest approach to the destination
        // body's pad; label it with that body and the time-to-approach (ETA)
        // so the marker is self-explanatory in the scene.
        const int dest = sim.contract().destination_body;
        char label[32];
        std::snprintf(label, sizeof label, "CP %s T%+.0fs", body_name(dest),
                      prediction.closest.time - sim.sim_time());
        draw_text(renderer, label, static_cast<int>(p.x) + 8,
                  static_cast<int>(p.y) + 4, 1, make_color(255, 196, 64));
    }

    if (prediction.peri.valid) {
        const Vec2 p = to_screen(prediction.peri.position.x,
                                  prediction.peri.position.y, cam);
        fill_rect(renderer, static_cast<int>(p.x) - 2,
                  static_cast<int>(p.y) - 2, 4, 4, make_color(235, 240, 250));
    }

    if (prediction.apo.valid) {
        const Vec2 p =
            to_screen(prediction.apo.position.x, prediction.apo.position.y,
                      cam);
        fill_rect(renderer, static_cast<int>(p.x) - 2,
                  static_cast<int>(p.y) - 2, 4, 4, make_color(120, 220, 255));
    }
}

// M06-R3: render the LIVE trajectory projection (the powered forecast using
// the current persistent controls, no further input) as a solid cyan polyline,
// plus its authoritative predicted-contact marker (PRED LAND / PRED CRASH) with
// a t-to-go readout. The system view decimates the polyline for clutter; the
// local frame draws it at full resolution (M06-R3-05).
void draw_live_prediction(SDL_Renderer* renderer,
                          const lander::RecedingHorizonPredictor& predictor,
                          const lander::Simulation& sim,
                          const lander::Camera& cam) {
    const auto& samples = predictor.samples();
    if (samples.size() < 2) {
        return;
    }

    const int decimate = cam.system_view() ? 4 : 1;
    const Color live_color(96, 224, 255);  // cyan (LIVE)
    for (size_t i = 0; i + decimate < samples.size(); i += decimate) {
        const Vec2 a = to_screen(samples[i].state.x, samples[i].state.y, cam);
        const Vec2 b = to_screen(samples[i + decimate].state.x,
                                 samples[i + decimate].state.y, cam);
        draw_line_simple(renderer, a, b, live_color);
    }

    const lander::PredictedContact& c = predictor.contact();
    if (!c.valid) {
        return;
    }
    const Vec2 p = to_screen(c.position.x, c.position.y, cam);
    const Color col =
        c.landed ? make_color(120, 240, 160) : make_color(255, 92, 80);
    const double h = 7.0;
    draw_thick_line(renderer, {p.x - h, p.y - h}, {p.x + h, p.y + h}, 2.0, col);
    draw_thick_line(renderer, {p.x - h, p.y + h}, {p.x + h, p.y - h}, 2.0, col);

    const double t_go = c.time - sim.sim_time();
    char label[32];
    std::snprintf(label, sizeof label, "%s %+.1fs",
                  c.landed ? "PRED LAND" : "PRED CRASH", t_go);
    draw_text(renderer, label, static_cast<int>(p.x) + 12,
              static_cast<int>(p.y) - 8, 1, col);
}

// M06-R3: a compact legend (bottom-left, clear of the HUD) distinguishing the
// three prediction kinds drawn in the scene: the zero-thrust COAST arc, the
// powered LIVE projection, and the planned-node PLAN arc.
void draw_prediction_legend(SDL_Renderer* renderer) {
    const int x = 18;
    int y = 384;
    const Color dim(150, 158, 172);
    draw_text(renderer, "PREDICTION", x, y, 2, dim);
    y += 24;
    const struct {
        const char* label;
        Color col;
    } rows[] = {
        {"COAST  no thrust", make_color(185, 195, 215)},
        {"LIVE   current controls", make_color(96, 224, 255)},
        {"PLAN   planned node", make_color(120, 240, 160)},
    };
    for (const auto& r : rows) {
        draw_thick_line(renderer, {x, y + 6}, {x + 18, y + 6}, 2.0, r.col);
        draw_text(renderer, r.label, x + 26, y, 1, dim);
        y += 20;
    }
}

void draw_flight_computer(
    SDL_Renderer* renderer, const lander::Simulation& sim,
    const std::optional<lander::ManeuverNode>& node,
    const lander::NodeBasis& basis, bool basis_valid,
    const lander::TrajectoryPrediction& prediction, bool prediction_valid,
    const lander::NodeExecutor& executor, lander::AttitudeMode attitude_mode,
    const std::string& message,
    const lander::TransferMidcourse* midcourse = nullptr,
    const lander::LandingAutopilot* landing = nullptr) {
    const Color panel(12, 14, 26);
    const Color white(228, 233, 244);
    const Color dim(130, 138, 156);
    const Color green(120, 240, 160);
    const Color red(255, 92, 80);
    const Color amber(255, 196, 64);

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    fill_rect(renderer, 832, 56, 440, 232, panel, 160);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    const lander::State& s = sim.state();
    const double t = sim.sim_time();
    (void)s;
    (void)basis_valid;
    int row = 74;

    auto line = [&](const std::string& text, Color color) {
        draw_text(renderer, text, 842, row, 1, color);
        row += 14;
    };

    line("FLIGHT COMPUTER", white);

    if (midcourse != nullptr) {
        char buffer[80];
        if (midcourse->active()) {
            std::snprintf(
                buffer, sizeof buffer, "MIDCOURSE  RE-AIM %d  VTG %d  %s",
                midcourse->slow_plans(), midcourse->retargets(),
                lander::executor_state_name(midcourse->fast().state()));
            line(buffer, green);
        } else {
            line("MIDCOURSE  --  [Z] to engage", dim);
        }
    }

    if (landing != nullptr) {
        if (landing->armed()) {
            char buffer[80];
            const auto& st = landing->status();
            std::snprintf(
                buffer, sizeof buffer,
                "AUTOLAND   %-9s  TG %4.1f  AC %4.1f",
                lander::landing_phase_name(st.phase), st.t_go,
                st.command_accel);
            line(buffer, st.terminal ? green : amber);
        } else {
            line("AUTOLAND   --  [9] to engage", dim);
        }
    }

    if (node) {
        char buffer[80];
        std::snprintf(buffer, sizeof buffer, "NODE %s T%+.1f",
                      body_name(node->frame_body), node->time - t);
        line(buffer, white);
        std::snprintf(buffer, sizeof buffer, "DPGR %+.1f  DRAD %+.1f",
                      node->dv_prograde, node->dv_radial);
        line(buffer, dim);

        const lander::Vec2 dv =
            prediction_valid && prediction.basis_valid
                ? prediction.dv_world
                : lander::node_world_dv(*node, basis);
        const double total = std::hypot(dv.x, dv.y);
        const double burn = sim.config().main_accel > 0.0
                                ? total / sim.config().main_accel
                                : 0.0;
        std::snprintf(buffer, sizeof buffer, "TOT %.1f  BURN %.1f S", total,
                      burn);
        line(buffer, white);
    } else {
        line("NODE NONE", dim);
        line("", dim);
        line("", dim);
    }

    Color executor_color = dim;
    if (executor.state() == lander::ExecutorState::Wait ||
        executor.state() == lander::ExecutorState::Align) {
        executor_color = amber;
    } else if (executor.state() == lander::ExecutorState::Burn) {
        executor_color = green;
    } else if (executor.state() == lander::ExecutorState::Aborted ||
               executor.state() == lander::ExecutorState::Incomplete) {
        executor_color = red;
    } else if (executor.state() == lander::ExecutorState::Complete) {
        executor_color = green;
    }
    std::string executor_line =
        std::string("EXEC ") + lander::executor_state_name(executor.state());
    if (executor.active() && executor.late()) {
        executor_line += " LATE";
    }
    line(executor_line, executor_color);

    Color attitude_color =
        attitude_mode == lander::AttitudeMode::Off ? dim : green;
    line(std::string("ATT  ") + lander::attitude_mode_name(attitude_mode),
         attitude_color);

    if (executor.active()) {
        char buffer[80];
        const double remaining =
            std::hypot(executor.dv_remaining().x, executor.dv_remaining().y);
        std::snprintf(buffer, sizeof buffer, "IGN T%+.1f  REM %.1f",
                      executor.ignite_time() - t, remaining);
        line(buffer, green);
    } else {
        line("IGN  --", dim);
    }

    if (prediction_valid) {
        char buffer[80];
        if (prediction.closest.valid) {
            std::snprintf(buffer, sizeof buffer, "CP  %.0f M T%+.1f",
                          prediction.closest.distance,
                          prediction.closest.time - t);
        } else {
            std::snprintf(buffer, sizeof buffer, "CP  --");
        }
        line(buffer, white);

        if (prediction.peri.valid) {
            std::snprintf(buffer, sizeof buffer, "PE  %.0f M",
                          prediction.peri.distance);
        } else {
            std::snprintf(buffer, sizeof buffer, "PE  --");
        }
        line(buffer, white);

        if (prediction.apo.valid) {
            std::snprintf(buffer, sizeof buffer, "AP  %.0f M",
                          prediction.apo.distance);
        } else {
            std::snprintf(buffer, sizeof buffer, "AP  --");
        }
        line(buffer, white);

        if (prediction.impact.valid) {
            std::snprintf(buffer, sizeof buffer, "IMPACT %s T%+.1f",
                          body_name(prediction.impact.body < 0 ? 0
                                                                : prediction.impact.body),
                          prediction.impact.time - t);
            line(buffer, red);
        } else {
            line("IMPACT --", dim);
        }
    } else {
        line("CP  --", dim);
        line("PE  --", dim);
        line("AP  --", dim);
        line("IMPACT --", dim);
    }

    if (!message.empty()) {
        line(message, amber);
    }
}

// M06-R6: O(1) zero-effort lookup for the landing autopilot, backed by a
// maintained zero-thrust (Coast) receding-horizon predictor. Mirrors the
// headless test: index the sample ring at the requested t_go, and continue a
// pure ballistic coast from the last non-terminal cached sample when the
// requested time lies beyond the cached horizon (or in a terminal tail).
// The autopilot never mutates the live sim; it only reads this predictor and
// emits ordinary input.
lander::BallisticState zero_from_predictor(
    const lander::BinarySystem& bin,
    const lander::RecedingHorizonPredictor& pred, double t_go, double dt) {
    const auto& s = pred.samples();
    if (s.empty()) {
        return {{0.0, 0.0}, {0.0, 0.0}, 0.0};
    }
    int req = (int)std::lround(t_go / dt);
    if (req < 0) {
        req = 0;
    }
    const int idx = std::min(req, (int)s.size() - 1);
    const auto& sample = s[idx];
    if (req < (int)s.size() && !sample.state.crashed &&
        !sample.state.landed) {
        return {{sample.state.x, sample.state.y},
                {sample.state.vx, sample.state.vy}, sample.time};
    }
    int last = idx;
    while (last > 0 && (s[last].state.crashed || s[last].state.landed)) {
        --last;
    }
    const lander::BallisticState start{{s[last].state.x, s[last].state.y},
                                       {s[last].state.vx, s[last].state.vy},
                                       s[last].time};
    const int extra = req - last;
    return lander::propagate_ballistic(bin, start, extra, dt);
}

// M06-R8 (expanded per R8-05..R8-15): gui-local observation context for the
// subsystem-isolation debug panels. It is pure debug-layer state: it records
// values the normal loop already computes (the last composed per-step input,
// the cached navigation prediction, low-rate predictor cost, the predictor's
// current prediction kind and policy signature) so the expanded per-mode
// panels can display them without any subsystem change. It never feeds back
// into the simulation or any controller.
struct DebugPanelCtx {
    double throttle{0.0};
    bool rw_enabled{false};
    bool rw_hold{false};
    bool rotate_left{false};
    bool rotate_right{false};
    lander::PredictionKind predictor_kind{lander::PredictionKind::Live};
    lander::PolicySignature predictor_signature{};
    bool predictor_signature_changed{false};
    double predictor_ms_ema{0.0};
    lander::Input last_step_input{};
    const lander::TrajectoryPrediction* prediction{nullptr};
    bool prediction_valid{false};
};

static double wrap_angle(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

static const char* prediction_kind_name(lander::PredictionKind k) {
    switch (k) {
        case lander::PredictionKind::Coast:
            return "COAST";
        case lander::PredictionKind::Live:
            return "LIVE";
        case lander::PredictionKind::Plan:
            return "PLAN";
    }
    return "LIVE";
}

// The nose angle whose thrust axis points along world direction `d`
// (thrust_hat(theta) = {-sin(theta), cos(theta)}), matching the bang-bang
// attitude controller's convention.
static double nose_angle_for(const lander::Vec2& d) {
    return std::atan2(-d.x, d.y);
}

// M06-R8: the subsystem-isolation debug panel. Shown in place of the normal
// flight-computer / nav / prediction / banner surfaces when a
// --debug-subsystem selector is active (the `ui` mode keeps the normal
// surfaces and shows only a minimal header). It always shows the common
// minimum readout (subsystem name, sim time, body / target, flight state,
// position / altitude, velocity / relative velocity) and then the full
// mode-specific detail section for the isolated subsystem (R8-05..R8-15).
// Read-only: it never drives or mutates the simulation; it only reads the
// live subsystems, the common readout, and the debug-layer context.
void draw_debug_subsystem_panel(
    SDL_Renderer* renderer, lander::DebugSubsystem mode,
    const lander::DebugCommonReadout& common, const lander::Simulation& sim,
    const lander::NodeExecutor& node_executor,
    const lander::TransferMidcourse& transfer_mc,
    const lander::LandingAutopilot& landing_ap,
    const lander::RecedingHorizonPredictor& live_predictor,
    const std::optional<lander::ManeuverNode>& maneuver_node,
    const lander::TransferDebugResult& transfer_debug,
    lander::AttitudeMode attitude_mode, const DebugPanelCtx& ctx) {
    const Color panel(12, 14, 26);
    const Color white(228, 233, 244);
    const Color dim(130, 138, 156);
    const Color green(120, 240, 160);
    const Color red(255, 92, 80);
    const Color amber(255, 196, 64);
    const Color cyan(96, 224, 255);

    // R8-15: the `ui` isolation keeps every normal player-facing surface
    // (flight computer, HUD, contract banner, nav overlay); only a minimal
    // debug header identifies the scenario. No internal solver dumps.
    if (mode == lander::DebugSubsystem::Ui) {
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        fill_rect(renderer, 8, 664, 470, 48, panel, 160);
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
        const lander::State& us = sim.state();
        const std::string ustate =
            us.crashed ? "CRASHED" : (us.landed ? "LANDED" : "FLIGHT");
        char buffer[96];
        std::snprintf(buffer, sizeof buffer,
                      "DEBUG: ui  (T %8.2f  state %-6s  ref %d)",
                      common.sim_time, ustate.c_str(), common.reference_body);
        draw_text(renderer, buffer, 18, 672, 1, cyan);
        draw_text(
            renderer,
            "camera / contract / HUD / banner isolation - normal "
            "flight computer active, no solver dumps",
            18, 688, 1, dim);
        return;
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    fill_rect(renderer, 832, 56, 440, 540, panel, 160);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    int row = 70;
    auto line = [&](const std::string& text, Color color) {
        draw_text(renderer, text, 842, row, 1, color);
        row += 13;
    };
    auto blank = [&]() { row += 3; };

    char buffer[160];

    // --- Common readout (always shown, every mode). ---
    line(std::string("DEBUG: ") + lander::debug_subsystem_name(mode), white);
    line(lander::debug_subsystem_description(mode), dim);
    std::snprintf(buffer, sizeof buffer, "T %8.2f s", common.sim_time);
    line(buffer, white);
    // M06-R11: make the reference / target body identity explicit (a named
    // body, not a bare index) and label every frame. Frames used below:
    //   W  = world / inertial frame (fixed barycentric axes)
    //   R  = the reference body's surface (altitude is relative to it)
    //   B  = body-centre-relative (rotating with the tidal spin)
    std::snprintf(buffer, sizeof buffer, "REF %-10s(#%d)  TGT %-10s(#%d)",
                  common.reference_label.c_str(), common.reference_body,
                  common.target_label.c_str(), common.target_body);
    line(buffer, white);
    const std::string state =
        common.crashed ? "CRASHED" : (common.landed ? "LANDED" : "FLIGHT");
    std::snprintf(buffer, sizeof buffer,
                  "STATE %-7s  W X %7.0f  W Y %7.0f",
                  state.c_str(), common.x, common.y);
    line(buffer, common.crashed ? red : white);
    std::snprintf(buffer, sizeof buffer,
                  "R-ALT %7.1f m  B VR %+6.1f  B VT %+6.1f",
                  common.altitude, common.radial_velocity,
                  common.tangential_velocity);
    line(buffer, dim);
    std::snprintf(buffer, sizeof buffer,
                  "B V-REL %8.2f m/s  (vs target body centre)",
                  common.relative_speed);
    line(buffer, dim);
    line("frames: W=inertial(world) R=ref-surface B=body-centre(rot)", dim);
    blank();

    // --- Mode-specific detail section for the isolated subsystem. ---
    const lander::State& st = sim.state();
    const double t = sim.sim_time();
    const auto& bin = sim.binary();
    const int ref = common.reference_body >= 0 ? common.reference_body : 0;

    line(std::string("  [ ") + lander::debug_subsystem_name(mode) + " ]", cyan);
    switch (mode) {
        case lander::DebugSubsystem::Manual: {
            // R8-05: the controls that exist (throttle, reaction wheels,
            // rotation), the surface-relative velocity, and the attitude
            // angle / spin rate.
            std::snprintf(buffer, sizeof buffer,
                          "  THR   %4.2f    [Up/Down ramp, X cut]",
                          ctx.throttle);
            line(buffer, white);
            std::snprintf(
                buffer, sizeof buffer,
                "  RW    %-3s  hold %-3s   [E toggle, Shift+E damp]",
                ctx.rw_enabled ? "on" : "off", ctx.rw_hold ? "on" : "off");
            line(buffer, ctx.rw_enabled ? green : dim);
            std::snprintf(
                buffer, sizeof buffer,
                "  ROT   %-4s            [A/D or L/R]",
                (ctx.rotate_left && ctx.rotate_right)
                    ? "BOTH"
                    : (ctx.rotate_left ? "L" : (ctx.rotate_right ? "R" : "--")));
            line(buffer, white);
            const lander::LocalVelocity lv =
                lander::local_velocity(st, bin.position(ref, t),
                                       bin.velocity(ref, t));
            // M06-R11: this is the body-CENTRE-relative velocity (the ship's
            // velocity in the reference body's centre, rotating with the tidal
            // spin), NOT a surface-relative quantity -- the ship may be far
            // above the surface, so "surface-relative" was a misleading label.
            std::snprintf(buffer, sizeof buffer,
                          "  B  VR %+7.2f   VT %+7.2f  (body-centre frame)",
                          lv.radial, lv.tangential);
            line(buffer, dim);
            std::snprintf(buffer, sizeof buffer,
                          "  ANG   %7.1f deg   W %+7.2f deg/s",
                          st.angle * 57.29578, lander::spin_deg_per_s(st));
            line(buffer, white);
            const std::string mstate =
                st.crashed ? "CRASHED" : (st.landed ? "LANDED" : "FLIGHT");
            std::snprintf(buffer, sizeof buffer, "  CLEAR %8.1f m   state %s",
                          common.altitude, mstate.c_str());
            line(buffer, st.crashed ? red : dim);
            break;
        }
        case lander::DebugSubsystem::Predictor: {
            // R8-06: the prediction policy, cache behaviour (shift vs cold
            // rebuild), horizon / sample count, predicted contact, PE / AP /
            // closest approach, and the low-rate cost of the advance.
            std::snprintf(
                buffer, sizeof buffer,
                "  ROLLING MODE %-5s   [F2]COAST  [F3]LIVE  [F4]PLAN",
                prediction_kind_name(ctx.predictor_kind));
            line(buffer, white);
            // M06-R11: make explicit that the readouts below are the rolling
            // predictor's own cache, distinct from the long COAST / PLAN
            // projected arcs drawn on the scene.
            line("  (readouts = rolling predictor; COAST/PLAN = long arcs)",
                 dim);
            std::snprintf(
                buffer, sizeof buffer,
                "  CACHE last: %s   rebuild %d  shift %d  invalidate %d",
                live_predictor.cache_hit_last() ? "SHIFT" : "COLD REBUILD",
                live_predictor.cold_rebuilds(), live_predictor.shifts(),
                live_predictor.invalidations());
            line(buffer, live_predictor.cache_hit_last() ? green : amber);
            // M06-R11: show the horizon in both steps and seconds, the sample
            // count in both steps and seconds (the actual available forecast
            // duration while the ring is filling), and make an empty ring
            // explicit instead of printing 0 / blank.
            {
                const double dt = sim.config().fixed_dt;
                const int hs = live_predictor.horizon_steps();
                const int n = (int)live_predictor.samples().size();
                if (n == 0) {
                    std::snprintf(buffer, sizeof buffer,
                                  "  HORIZON %4d(%5.1fs)  SAMPLES --  (not primed)",
                                  hs, hs * dt);
                    line(buffer, dim);
                } else {
                    std::snprintf(
                        buffer, sizeof buffer,
                        "  HORIZON %4d(%5.1fs)  SAMPLES %4d(%5.1fs)",
                        hs, hs * dt, n, n * dt);
                    line(buffer, white);
                    std::snprintf(
                        buffer, sizeof buffer,
                        "  primed %-3s  terminal %-3s  avail %5.1f s",
                        live_predictor.primed() ? "yes" : "no",
                        live_predictor.terminal() ? "yes" : "no",
                        n * dt);
                    line(buffer, dim);
                }
            }
            const auto& sig = ctx.predictor_signature;
            std::snprintf(
                buffer, sizeof buffer,
                "  SIG   %s thr %.2f att %s rw %s%s node %s ref %d tgt %d",
                prediction_kind_name(sig.kind), sig.throttle,
                lander::attitude_mode_name(sig.attitude_mode),
                sig.rw_enabled ? "on" : "off", sig.rw_hold ? "+hold" : "",
                sig.node_present ? "yes" : "no", sig.reference_body,
                sig.target_body);
            line(buffer, dim);
            if (ctx.predictor_signature_changed) {
                line("  SIG   CHANGED this frame (rebuild)", amber);
            }
            const auto& contact = live_predictor.contact();
            if (contact.valid) {
                // M06-R11: show an ETA (contact time minus now), the absolute
                // time labelled separately, the named contact body, and the
                // world-frame contact position.
                const int cbody =
                    contact.landed ? contact.body : contact.crash_body;
                std::snprintf(
                    buffer, sizeof buffer,
                    "  PRED  %s  %-10s#%d  ETA %5.1f s (t=%6.1f)  W(%4.0f,%4.0f)",
                    contact.landed ? "LAND" : "CRASH",
                    lander::debug_body_label(cbody).c_str(), cbody,
                    contact.time - t, contact.time,
                    contact.position.x, contact.position.y);
                line(buffer, contact.landed ? green : red);
            } else {
                line("  PRED  -- no contact in horizon", dim);
            }
            // M06-R11: min / max body-centre radius over the horizon (the old
            // "PE / AP" -- these are NOT apsis-finding, just the closest and
            // farthest reference body-centre distance in the ring) and the
            // reference-POINT clearance, now computed with the same terrain
            // altitude the readout uses (altitude_at per sample, per body)
            // instead of r - reference_radius (a constant that ignores the
            // actual terrain shape).
            const int dest = sim.contract().destination_body;
            double min_r = 1e300, max_r = -1e300, clr = 1e300;
            const int nscan = (int)live_predictor.samples().size();
            if (nscan == 0) {
                line("  MIN R --  MAX R --  CLR-PT --  (no forecast)", dim);
            } else {
                for (const auto& smp : live_predictor.samples()) {
                    for (int b = 0; b < 2; b++) {
                        const lander::Vec2 bp = bin.position(b, smp.time);
                        const double r =
                            std::hypot(smp.state.x - bp.x,
                                       smp.state.y - bp.y);
                        if (b == ref) {
                            min_r = std::min(min_r, r);
                            max_r = std::max(max_r, r);
                        }
                        const double alt = lander::altitude_at(
                            bin.body(b).terrain, smp.state, bp,
                            bin.body_rotation(smp.time));
                        clr = std::min(clr, alt);
                    }
                }
                std::snprintf(
                    buffer, sizeof buffer,
                    "  MIN R %7.1f  MAX R %7.1f m  (ref body, W)", min_r, max_r);
                line(buffer, dim);
                std::snprintf(
                    buffer, sizeof buffer,
                    "  CLR-PT %7.1f m  (ref-pt clearance, both bodies)", clr);
                line(buffer, clr < 15.0 ? red : dim);
            }
            // Closest approach to the destination pad point over the ring.
            const auto& dterrain = bin.body(dest).terrain;
            double pad_closest = 1e300;
            for (const auto& smp : live_predictor.samples()) {
                const lander::Vec2 pad = bin.surface_point(
                    dest, dterrain.angle_at_arc(0.0),
                    dterrain.surface_radius_at_arc(0.0), smp.time)
                    .position;
                pad_closest = std::min(
                    pad_closest,
                    std::hypot(smp.state.x - pad.x, smp.state.y - pad.y));
            }
            std::snprintf(buffer, sizeof buffer,
                          "  PAD   %7.1f m  (closest to %s pad)", pad_closest,
                          body_name(dest));
            line(buffer, pad_closest < 20.0 ? amber : dim);
            std::snprintf(buffer, sizeof buffer,
                          "  COST  %6.3f ms/step avg  (budget %d steps/frame)",
                          ctx.predictor_ms_ema, kPredictBudget);
            line(buffer, ctx.predictor_ms_ema > 0.0 ? dim : white);
            break;
        }
        case lander::DebugSubsystem::Attitude: {
            // R8-07: the requested mode, target vs actual angle, angular
            // error / spin, the stop angle, the commanded wheel direction,
            // and alignment.
            const int dest = sim.contract().destination_body;
            const lander::NodeBasis basis = lander::compute_node_basis(
                bin, t, ref, {st.x, st.y}, {st.vx, st.vy});
            const lander::Vec2 pad = bin.surface_point(
                dest, bin.body(dest).terrain.angle_at_arc(0.0),
                bin.body(dest).terrain.surface_radius_at_arc(0.0), t)
                .position;
            lander::Vec2 maneuver_dv{};
            if (attitude_mode == lander::AttitudeMode::Maneuver &&
                maneuver_node && ctx.prediction_valid && ctx.prediction &&
                ctx.prediction->basis_valid) {
                maneuver_dv = ctx.prediction->dv_world;
            }
            const auto dir = lander::attitude_target_direction(
                attitude_mode, basis, {st.x, st.y}, pad, maneuver_dv);
            std::snprintf(buffer, sizeof buffer,
                          "  MODE  %-10s   [1-8 select]",
                          lander::attitude_mode_name(attitude_mode));
            line(buffer, white);
            if (dir.has_value()) {
                const double target_angle = nose_angle_for(*dir);
                const double err = wrap_angle(target_angle - st.angle);
                const bool aligned =
                    std::fabs(err) <= 0.01 && std::fabs(st.omega) <= 0.02;
                std::snprintf(buffer, sizeof buffer,
                              "  TGT   %7.1f deg   ACT %7.1f deg",
                              target_angle * 57.29578,
                              st.angle * 57.29578);
                line(buffer, white);
                std::snprintf(buffer, sizeof buffer,
                              "  ERR   %+7.1f deg   W %+7.2f deg/s",
                              err * 57.29578, lander::spin_deg_per_s(st));
                line(buffer, std::fabs(err) > 0.01 ? amber : green);
                std::snprintf(buffer, sizeof buffer,
                              "  STOP  0.57 deg (0.01 rad)   aligned %s",
                              aligned ? "YES" : "no");
                line(buffer, aligned ? green : dim);
            } else {
                line("  TGT   n/a (mode off or degenerate target)", dim);
                std::snprintf(buffer, sizeof buffer,
                              "  W     %+7.2f deg/s", lander::spin_deg_per_s(st));
                line(buffer, white);
            }
            std::snprintf(
                buffer, sizeof buffer,
                "  RW cmd: %s   (last step input)",
                (ctx.last_step_input.rotate_left &&
                 ctx.last_step_input.rotate_right)
                    ? "BOTH"
                    : (ctx.last_step_input.rotate_left
                           ? "L"
                           : (ctx.last_step_input.rotate_right ? "R" : "--")));
            line(buffer, dim);
            line("  1 OFF  2 PRO  3 RETRO  4 RDO  5 RDI  6 TGT  7 ANTI  8 MAN",
                 dim);
            break;
        }
        case lander::DebugSubsystem::NodeEdit: {
            // R8-08: the node's time / frame / delta-v, its world position,
            // the predicted pre/post arcs, and the plan kind.
            if (maneuver_node) {
                const double total =
                    std::hypot(maneuver_node->dv_prograde,
                               maneuver_node->dv_radial);
                std::snprintf(
                    buffer, sizeof buffer,
                    "  NODE  %s   T+%6.1f s   dPGR %+.2f   dRAD %+.2f",
                    body_name(maneuver_node->frame_body),
                    maneuver_node->time - t, maneuver_node->dv_prograde,
                    maneuver_node->dv_radial);
                line(buffer, white);
                std::snprintf(buffer, sizeof buffer,
                              "  TOTAL %6.2f m/s", total);
                line(buffer, white);
                if (ctx.prediction_valid && ctx.prediction) {
                    const auto& p = *ctx.prediction;
                    std::snprintf(buffer, sizeof buffer,
                                  "  POS   (%8.1f, %8.1f)", p.node_position.x,
                                  p.node_position.y);
                    line(buffer, dim);
                    if (p.pre.size() >= 2) {
                        const auto& a = p.pre.front();
                        const auto& b = p.pre.back();
                        std::snprintf(
                            buffer, sizeof buffer,
                            "  PRE   %3d pts  (%6.0f,%6.0f)->(%6.0f,%6.0f)",
                            (int)p.pre.size(), a.x, a.y, b.x, b.y);
                        line(buffer, dim);
                    }
                    if (p.post.size() >= 2) {
                        const auto& a = p.post.front();
                        const auto& b = p.post.back();
                        std::snprintf(
                            buffer, sizeof buffer,
                            "  POST  %3d pts  (%6.0f,%6.0f)->(%6.0f,%6.0f)",
                            (int)p.post.size(), a.x, a.y, b.x, b.y);
                        line(buffer, dim);
                        std::snprintf(buffer, sizeof buffer,
                                      "  PE  %7.1f m   AP  %7.1f m",
                                      p.peri.valid ? p.peri.distance : -1.0,
                                      p.apo.valid ? p.apo.distance : -1.0);
                        line(buffer, dim);
                    }
                    line("  PLAN  n/a (plan kind not stored on the node)",
                         dim);
                } else {
                    line("  PRED  (recomputing at bounded cadence)", dim);
                }
            } else {
                line("  no node  -  [C] create  [H/J] time  [K/L] dv", dim);
            }
            line("  [U/I/Y] plan  [Return] exec  [Del] clear", dim);
            break;
        }
        case lander::DebugSubsystem::NodeExecutor: {
            // R8-09: executor state, ignition / burn estimates, remaining /
            // delivered VGO, throttle, alignment, fuel, and the outcome.
            const auto es = node_executor.state();
            std::snprintf(buffer, sizeof buffer, "  STATE  %s",
                          lander::executor_state_name(es));
            line(buffer, node_executor.active() ? green : dim);
            std::snprintf(
                buffer, sizeof buffer,
                "  NODE  %s   ignite T+%7.1f s   burn ~%5.1f s%s",
                body_name(node_executor.frame_body()),
                node_executor.ignite_time() - t, node_executor.burn_time(),
                node_executor.late() ? "   LATE" : "");
            line(buffer, white);
            const lander::Vec2 dv_rem = node_executor.dv_remaining();
            const lander::Vec2 dv_tot = node_executor.dv_total();
            const double rem_n = std::hypot(dv_rem.x, dv_rem.y);
            const double tot_n = std::hypot(dv_tot.x, dv_tot.y);
            std::snprintf(buffer, sizeof buffer,
                          "  VGO   remaining %6.2f  of %6.2f  (delivered %5.2f)",
                          rem_n, tot_n, std::max(0.0, tot_n - rem_n));
            line(buffer, white);
            std::snprintf(buffer, sizeof buffer,
                          "  THR   %4.2f   (held during burn)",
                          ctx.last_step_input.main_throttle);
            line(buffer, dim);
            std::snprintf(buffer, sizeof buffer,
                          "  FUEL  %7.1f kg", st.fuel);
            line(buffer, st.fuel < 100.0 ? amber : dim);
            if (es == lander::ExecutorState::Complete) {
                line("  RESULT  COMPLETE", green);
            } else if (es == lander::ExecutorState::Aborted) {
                line("  RESULT  ABORTED", red);
            } else if (es == lander::ExecutorState::Incomplete) {
                line("  RESULT  INCOMPLETE (fuel exhausted)", red);
            }
            line("  [Return] exec  [Shift+Return | X] abort", dim);
            break;
        }
        case lander::DebugSubsystem::TransferCold: {
            // R8-10: the one-shot COLD solve: result, flight time, departure
            // delta-v, miss, arrival speed, terrain-clear validation,
            // propagation count, and solve wall time.
            const auto& c = transfer_debug.cold;
            std::snprintf(
                buffer, sizeof buffer,
                "  COLD  one-shot @ startup (T0)   %s -> %s",
                body_name(transfer_debug.source),
                body_name(transfer_debug.target));
            line(buffer, dim);
            std::snprintf(
                buffer, sizeof buffer,
                "  RESULT  %s   miss %8.3f m   TOF %7.1f s",
                !transfer_debug.computed ? "NOT RUN"
                                         : (c.valid ? "SOLVED"
                                                    : "NO SOLUTION"),
                c.achieved_miss, c.time_of_flight);
            line(buffer, c.valid ? green : red);
            if (transfer_debug.computed) {
                std::snprintf(
                    buffer, sizeof buffer,
                    "  DEP   (%7.2f,%7.2f) -> (%7.2f,%7.2f)",
                    c.departure_state.x, c.departure_state.y,
                    c.departure_velocity.x, c.departure_velocity.y);
                line(buffer, dim);
                std::snprintf(buffer, sizeof buffer,
                              "  ARR   %6.2f m/s  target-relative",
                              c.arrival_rel_speed);
                line(buffer, dim);
                line("  TERRAIN  validated (solver gate, full arc)", green);
                std::snprintf(
                    buffer, sizeof buffer,
                    "  PROP  %5d (bounded coarse grid)   wall %6.2f ms",
                    transfer_debug.cold_propagations,
                    transfer_debug.cold_solve_ms);
                line(buffer, dim);
            }
            line("  TFD-1/TFD-2 surface raw here; this mode does not fix them",
                 dim);
            break;
        }
        case lander::DebugSubsystem::TransferWarm: {
            // R8-11: the WARM midcourse: cache validity, last correction
            // iterations, propagation count, miss before/after, fallback,
            // and the bounded re-plan cadence.
            const auto& c = transfer_debug.cold;
            const auto& w = transfer_mc.cache();
            std::snprintf(buffer, sizeof buffer,
                          "  WARM  active %s   [Z engage]  [X abort]",
                          transfer_mc.active() ? "yes" : "no");
            line(buffer, transfer_mc.active() ? green : dim);
            std::snprintf(buffer, sizeof buffer,
                          "  COLD  seed %s   TOF %6.1f s   miss %7.3f m",
                          c.valid ? "SOLVED" : "NO SOLUTION",
                          c.time_of_flight, c.achieved_miss);
            line(buffer, c.valid ? green : red);
            std::snprintf(
                buffer, sizeof buffer,
                "  CACHE  %-7s   TOF %6.1f s   dep v (%6.2f,%6.2f)",
                w.valid ? "valid" : "invalid", w.time_of_flight,
                w.departure_velocity.x, w.departure_velocity.y);
            line(buffer, w.valid ? green : dim);
            std::snprintf(buffer, sizeof buffer,
                          "  NEWTON  %2d iters   fallback COLD: %s",
                          w.newton_iterations,
                          transfer_debug.warm_fallback_last ? "yes" : "no");
            line(buffer, transfer_debug.warm_fallback_last ? amber : dim);
            std::snprintf(
                buffer, sizeof buffer,
                "  PROP  last %5d   total %6d   (0 = warm path only)",
                transfer_debug.warm_propagations_last,
                transfer_debug.warm_propagations_total);
            line(buffer, dim);
            std::snprintf(buffer, sizeof buffer,
                          "  MISS  %7.3f -> %7.3f m   (tol %4.2f)",
                          transfer_debug.warm_miss_before,
                          transfer_debug.warm_miss_after,
                          kMidcourseMissTolerance);
            line(buffer,
                 transfer_debug.warm_miss_after > kMidcourseMissTolerance
                     ? amber
                     : dim);
            std::snprintf(
                buffer, sizeof buffer,
                "  REPLAN  %4d   RETARGET %4d   cadence %.1f s",
                transfer_mc.slow_plans(), transfer_mc.retargets(),
                kMidcourseReplanSec);
            line(buffer, dim);
            line("  [manual throttle reclaims control]", dim);
            line("  TFD-1/TFD-2 surface raw here; this mode does not fix them",
                 dim);
            break;
        }
        case lander::DebugSubsystem::AutolandPrimary:
        case lander::DebugSubsystem::AutolandCompanion:
        case lander::DebugSubsystem::AutolandCross: {
            // R8-12..14: the landing autopilot: target pad, phase,
            // target-relative velocity, commanded acceleration / attitude /
            // throttle, t_go, the active guidance path, and the touchdown
            // result. Cross adds the handoff gate; companion adds the
            // terminal-law path.
            if (landing_ap.armed()) {
                const auto& status = landing_ap.status();
                const int tgt = landing_ap.target_body();
                const int src = landing_ap.source_body();
                const lander::Vec2 tgt_pos = bin.position(tgt, t);
                const lander::Vec2 tgt_vel = bin.velocity(tgt, t);
                const lander::LocalVelocity tlv =
                    lander::local_velocity(st, tgt_pos, tgt_vel);
                const double trel =
                    std::hypot(st.vx - tgt_vel.x, st.vy - tgt_vel.y);
                std::snprintf(
                    buffer, sizeof buffer,
                    "  TARGET  %s pad   source %s   [9] arms ref body",
                    body_name(tgt), src >= 0 ? body_name(src) : "-");
                line(buffer, white);
                if (mode == lander::DebugSubsystem::AutolandCross) {
                    static const char* kShort[] = {
                        "ASC",  "XFER", "CAP",  "DEO", "BRK",  "APP",
                        "DESC", "TD"};
                    const int idx = (int)status.phase;
                    std::string seq;
                    for (int i = 0; i < 8; i++) {
                        if (i > 0) {
                            seq += " > ";
                        }
                        seq += (i == idx) ? ("[" + std::string(kShort[i]) + "]")
                                          : std::string(kShort[i]);
                    }
                    line("  SEQ   " + seq, white);
                } else {
                    std::snprintf(buffer, sizeof buffer, "  PHASE  %s",
                                  lander::landing_phase_name(status.phase));
                    line(buffer, white);
                }
                std::snprintf(buffer, sizeof buffer,
                              "  T-REL %6.2f m/s   VR %+6.2f   VT %+6.2f",
                              trel, tlv.radial, tlv.tangential);
                line(buffer, dim);
                const auto& cmd = landing_ap.command();
                if (cmd.valid) {
                    std::snprintf(buffer, sizeof buffer,
                                  "  CMD   %5.2f m/s^2  @ %6.1f deg",
                                  std::hypot(cmd.acceleration.x,
                                             cmd.acceleration.y),
                                  nose_angle_for(cmd.acceleration) *
                                      57.29578);
                    line(buffer, white);
                    std::snprintf(buffer, sizeof buffer,
                                  "  THR   %4.2f   t_go %6.1f s",
                                  cmd.throttle, cmd.t_go);
                    line(buffer, dim);
                    const double att_err = wrap_angle(
                        nose_angle_for(cmd.acceleration) - st.angle);
                    std::snprintf(buffer, sizeof buffer,
                                  "  ATT   want %6.1f  act %6.1f  err %+6.1f deg",
                                  nose_angle_for(cmd.acceleration) * 57.29578,
                                  st.angle * 57.29578,
                                  att_err * 57.29578);
                    line(buffer, std::fabs(att_err) > 0.15 ? amber : green);
                } else {
                    line("  CMD   n/a (no feasible candidate this update)",
                         amber);
                }
                const auto& pv = landing_ap.terminal_preview();
                const std::string path =
                    landing_ap.high_energy()
                        ? "high-energy VGO (CAP/DEO/BRK)"
                        : (landing_ap.target_disturbed()
                               ? "gentle gravity-feedforward (disturbed)"
                               : "ZEM/ZEV terminal (clean target)");
                std::snprintf(buffer, sizeof buffer, "  PATH  %s", path.c_str());
                line(buffer, dim);
                if (mode == lander::DebugSubsystem::AutolandCross) {
                    std::snprintf(
                        buffer, sizeof buffer,
                        "  GATE  handoff %s  (t_go %5.1f / peak %4.2f / r0 %+5.2f)",
                        pv.feasible ? "feasible" : "blocked", pv.t_go,
                        pv.peak_accel, pv.initial_radial);
                    line(buffer, pv.feasible ? green : red);
                } else {
                    std::snprintf(
                        buffer, sizeof buffer,
                        "  TERM  %s   t_go %5.1f s",
                        status.terminal ? "ZEM/ZEV active" : "held command",
                        status.t_go);
                    line(buffer, status.terminal ? green : dim);
                }
                if (status.phase == lander::LandingPhase::Touchdown) {
                    line("  RESULT  TOUCHDOWN", green);
                } else if (st.crashed) {
                    line("  RESULT  CRASHED (autopilot aborted)", red);
                } else {
                    std::snprintf(buffer, sizeof buffer,
                                  "  RESULT  in flight (phase %s)",
                                  lander::landing_phase_name(status.phase));
                    line(buffer, dim);
                }
            } else {
                line("  not armed  -  [9] arm the ref-body autopilot", dim);
                line("  [X abort]  [manual throttle reclaims control]", dim);
            }
            break;
        }
        case lander::DebugSubsystem::Ui:
        case lander::DebugSubsystem::None:
        default:
            break;  // `ui` early-returned above; `none` never reaches here.
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::uint64_t seed = 0;
    bool have_seed = false;
    int max_frames = 0;
    int fps_cap = 0;
    std::string screenshot_path;
    bool orbit_demo = false;
    bool system_view = false;
    std::string debug_subsystem_arg;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return 0;
        } else if (arg == "--seed" && i + 1 < argc) {
            seed = std::strtoull(argv[++i], nullptr, 10);
            have_seed = true;
        } else if (arg == "--frames" && i + 1 < argc) {
            max_frames = std::atoi(argv[++i]);
        } else if (arg == "--fps" && i + 1 < argc) {
            fps_cap = std::atoi(argv[++i]);
        } else if (arg == "--screenshot" && i + 1 < argc) {
            screenshot_path = argv[++i];
        } else if (arg == "--orbit-demo") {
            orbit_demo = true;
        } else if (arg == "--system-view") {
            system_view = true;
        } else if (arg == "--debug-subsystem" && i + 1 < argc) {
            debug_subsystem_arg = argv[++i];
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            print_usage();
            return 2;
        }
    }
    if (!have_seed) {
        seed = random_seed();
    }

    // M06-R8: resolve the subsystem-isolation debug selector. An absent or
    // "none" selector runs ordinary gameplay with no debug surface; an
    // unrecognized name is a hard error reported before any window is created.
    lander::DebugSubsystem debug_mode = lander::DebugSubsystem::None;
    if (!debug_subsystem_arg.empty()) {
        const auto parsed =
            lander::parse_debug_subsystem(debug_subsystem_arg);
        if (!parsed) {
            std::fprintf(stderr,
                         "Unknown debug subsystem: %s\n",
                         debug_subsystem_arg.c_str());
            print_usage();
            return 2;
        }
        debug_mode = *parsed;
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "SDL init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow(
        "Lunar Lander", kWindowWidth, kWindowHeight, 0);
    if (window == nullptr) {
        std::fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED,
                          SDL_WINDOWPOS_CENTERED);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (renderer == nullptr) {
        // Headless or software-only environments (e.g. SDL_VIDEODRIVER=dummy).
        renderer = SDL_CreateRenderer(window, "software");
    }
    if (renderer == nullptr) {
        std::fprintf(stderr, "Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);

    lander::Simulation sim;
    // The camera owns the framing anchor: in local modes it follows the raw
    // lander position and rotates with the reference body's local surface
    // frame, so the lander stays at the configured screen position at every
    // scale. In system view it holds a fixed inertial orientation with the
    // ship at the viewport centre.
    lander::CameraParams cam_params;
    cam_params.window_width = kWindowWidth;
    cam_params.window_height = kWindowHeight;
    lander::Camera cam(cam_params);
    std::vector<lander::Star> stars =
        lander::make_stars(seed, kWindowWidth, kWindowHeight);

    bool paused = false;
    bool running = true;
    bool nav_overlay = true;
    double throttle = 0.0;
    int pending_wheel = 0;
    bool pending_cam_toggle = false;
    // Transient contract-completion presentation.
    std::optional<lander::Contract> last_completed_seen;
    double contract_banner_time = 0.0;
    // Transient developer-control feedback (M05-R3-16 no-solution message
    // and T confirmation).
    std::string debug_message;
    double debug_message_time = 0.0;
    Uint64 prev_ns = SDL_GetTicksNS();
    int frame = 0;
    // Continuous presentation clock (seconds) that drives the cosmetic flame
    // animation. It advances with real frame time and is independent of the
    // integer simulation tick counter, so the flame is smooth at any display
    // refresh rate.
    double flame_clock = 0.0;

    // M06-R2-11: opt-in per-frame development diagnostic (set the
    // LL_FRAME_DEBUG environment variable to a non-"0" value). Prints frame
    // wall dt, the prediction rebuild cost, and the number of fixed physics
    // steps drained this rendered frame, which is what the launch-hitch
    // investigation needs to verify the prediction stays bounded.
    const bool frame_debug = [] {
        const char* e = std::getenv("LL_FRAME_DEBUG");
        return e != nullptr && e[0] != '\0' && e[0] != '0';
    }();
    int ticks_before_frame = 0;

    // M06-R2-13: smoothed frames-per-second for the on-screen counter. An
    // exponential moving average of the instantaneous 1/dt keeps the number
    // stable instead of flickering with every frame's jitter.
    double fps_ema = 0.0;

    // M06-R3: the LIVE trajectory projection. A receding-horizon predictor that
    // rolls a powered forecast using the current persistent controls (throttle,
    // attitude, reaction wheels, and the active maneuver node) through the same
    // Simulation-stepping core as live flight, so its contact outcome is
    // authoritative. The existing flight-computer arc already renders the
    // zero-thrust COAST (`pre`) and the planned-node PLAN (`post`).
    lander::RecedingHorizonPredictor live_predictor;
    constexpr int kPredictHorizon = 1200;  // ~10 s at the fixed step
    live_predictor.configure(kPredictHorizon);

    // M05-R3-07: triple-tap guard for the dangerous controls (new seed,
    // circularize CW/CCW).
    lander::TripleTapGuard tap_guard;

    // M05-R4-03: persistent reaction-wheel toggle. It is GUI/control state,
    // not simulation state; the per-frame input below composes it with the
    // manual-rotation priority rule.
    lander::ReactionWheelToggle reaction_wheels;

    std::optional<lander::ManeuverNode> maneuver_node;
    lander::NodeExecutor node_executor;
    // M06-R5 / D05: two-level transfer midcourse controller (fast O(1) VGO +
    // bounded-rate WARM re-aim). Distinct from the one-shot R4 node_executor;
    // engaged with Z from a valid transfer plan and disengaged on abort /
    // land / crash.
    lander::TransferMidcourse transfer_mc;

    // M06-R6: target-pad powered-landing autopilot (ZEM/ZEV terminal law with
    // an Apollo-polynomial held command over a bounded t_go scan). It emits
    // only ordinary input (attitude + main throttle) and disengages on abort /
    // land / crash. The maintained zero-thrust Coast predictor below is its
    // O(1) zero-effort source (R6-03 / R6-09).
    lander::LandingAutopilot landing_ap;
    lander::LandingConfig landing_cfg{};
    lander::RecedingHorizonPredictor landing_coast;
    int landing_horizon = 0;

    // M06-R8: subsystem-isolation debug harness state. `debug_mode` (resolved
    // from --debug-subsystem before the window opens) selects which subsystem
    // the startup fixture arms and which focused panel is shown. For
    // transfer-cold / transfer-warm the fixture stores a one-shot COLD solve
    // here for the panel to display alongside the live WARM midcourse cache.
    lander::TransferDebugResult transfer_debug{};
    // M06-R8 (R8-05..R8-15): the expanded panel's gui-local observation
    // context. It records values the normal loop already computes (the last
    // composed per-step input, the prediction cache, the predictor's current
    // kind / policy signature / cost, and the WARM re-plan observations stored
    // on transfer_debug above). It never feeds back into the simulation or
    // any subsystem; without a selector it stays at its defaults.
    DebugPanelCtx panel_ctx{};

    lander::AttitudeMode attitude_mode = lander::AttitudeMode::Off;
    // M06-R5: last inter-body transfer solution, used to warm-start the next
    // transfer plan (bounded differential correction) instead of a full coarse
    // search. Route- and crash-scoped: reset with the flight computer.
    lander::TransferSolution transfer_cache{};
    lander::TrajectoryPrediction prediction{};
    bool prediction_valid = false;
    std::string pc_message;
    // Bounded-cadence prediction cache (M06-R2-11 / M06-R2-D06). Tracks the
    // simulation time and inputs at which the current `prediction` was built,
    // so the ~52k-step arc is only rebuilt at kPredictRefreshSec cadence or
    // when those inputs actually change.
    double predict_last_sim_time = 0.0;
    int predict_ref_body = -1;
    int predict_dest_body = -1;
    bool predict_node_present = false;
    double predict_node_time = 0.0;
    int predict_node_frame_body = -1;
    double predict_node_prograde = 0.0;
    double predict_node_radial = 0.0;

    auto reset_flight_computer = [&]() {
        maneuver_node.reset();
        node_executor.clear();
        transfer_mc.abort();
        landing_ap.abort();
        landing_coast.reset();
        attitude_mode = lander::AttitudeMode::Off;
        transfer_cache = {};
        prediction = {};
        prediction_valid = false;
        pc_message.clear();
        predict_last_sim_time = 0.0;
        predict_ref_body = -1;
        predict_dest_body = -1;
        predict_node_present = false;
        predict_node_time = 0.0;
        predict_node_frame_body = -1;
        predict_node_prograde = 0.0;
        predict_node_radial = 0.0;
    };

    // True when the inputs that define the predicted arc (reference body,
    // target body, or the maneuver node) differ from the ones the cached
    // `prediction` was built from, meaning it must be rebuilt right away
    // rather than waiting for the next cadence tick.
    auto prediction_inputs_changed = [&]() {
        if (sim.reference_body() != predict_ref_body) return true;
        if (sim.contract().destination_body != predict_dest_body) return true;
        if (maneuver_node.has_value() != predict_node_present) return true;
        if (maneuver_node.has_value()) {
            if (std::abs(maneuver_node->time - predict_node_time) > 1e-9)
                return true;
            if (maneuver_node->frame_body != predict_node_frame_body)
                return true;
            if (std::abs(maneuver_node->dv_prograde - predict_node_prograde) >
                1e-9)
                return true;
            if (std::abs(maneuver_node->dv_radial - predict_node_radial) > 1e-9)
                return true;
        }
        return false;
    };

    auto destination_pad_at = [&](double t) {
        const int dest = sim.contract().destination_body;
        const lander::Terrain& terrain =
            sim.binary().body(dest).terrain;
        return sim.binary().surface_point(
            dest, terrain.angle_at_arc(0.0),
            terrain.surface_radius_at_arc(0.0), t);
    };

    auto start_mission = [&]() {
        sim.reset(seed);
        throttle = 0.0;
        paused = false;
        flame_clock = 0.0;
        tap_guard.reset();
        reaction_wheels.reset();
        last_completed_seen = sim.last_completed();
        contract_banner_time = 0.0;
        debug_message.clear();
        debug_message_time = 0.0;
        panel_ctx = {};
        reset_flight_computer();
        if (orbit_demo) {
            // Developer mode: start in flight on a terrain-clearing
            // circular-ish orbit around the primary at t = 0 (primary
            // centred at (-a, 0), ship due north of it, velocity along the
            // local tangent plus the primary's own barycentric velocity).
            const lander::BinarySystem& bin = sim.binary();
            const double r = sim.terrain().max_surface_radius() + 20.0;
            const lander::Vec2 p0 = bin.position(0, 0.0);
            const lander::Vec2 v0 = bin.velocity(0, 0.0);
            const double speed = std::sqrt(sim.config().mu / r);
            lander::State orbit{};
            orbit.x = p0.x;
            orbit.y = p0.y + r;
            orbit.vx = v0.x + speed;
            orbit.vy = v0.y;
            orbit.angle = 0.0;
            orbit.fuel = sim.config().fuel;
            sim.set_state(orbit);
        }
        if (debug_mode != lander::DebugSubsystem::None) {
            // M06-R8: build the deterministic startup-only fixture for the
            // selected subsystem. It re-seeds with the per-mode seed, places the
            // ship, and arms exactly the one subsystem under test; from here on
            // the armed subsystem runs through the normal simulation / control
            // paths (no hidden forces). The maintained Coast predictor is the
            // landing autopilot's O(1) zero-effort source, looked up through the
            // gui-local helper.
            lander::ZeroEffortQuery zero_effort = [&](double t_go) {
                return zero_from_predictor(sim.binary(), landing_coast, t_go,
                                           sim.config().fixed_dt);
            };
            lander::DebugSubsystems subs{
                sim, node_executor, transfer_mc, landing_ap, live_predictor,
                landing_coast, landing_cfg, maneuver_node, attitude_mode,
                &transfer_debug, zero_effort,
            };
            lander::setup_debug_scenario(debug_mode, subs);
        }
        const lander::State& state = sim.state();
        const lander::Vec2 ref_pos =
            sim.binary().position(sim.reference_body(), 0.0);
        cam.snap(state.x, state.y,
                 lander::local_up_angle(state, ref_pos));
    };
    start_mission();
    if (system_view) {
        cam.set_system(true);
    }

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            } else if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
                switch (event.key.scancode) {
                    case SDL_SCANCODE_ESCAPE:
                    case SDL_SCANCODE_Q:
                        running = false;
                        break;
                    case SDL_SCANCODE_R: {
                        // Triple-tap guarded (M05-R3-10): a single press no
                        // longer restarts; the third tap within 700 ms
                        // retries the same seed.
                        const lander::GuardedAction action =
                            tap_guard.press(lander::GuardedKey::kRetry,
                                            SDL_GetTicks());
                        if (action == lander::GuardedAction::kRetry) {
                            // Retry the same seed: both pad layouts are
                            // identical.
                            start_mission();
                        }
                        break;
                    }
                    case SDL_SCANCODE_N: {
                        // Triple-tap guarded: one or two taps only arm the
                        // sequence; the third tap within 700 ms rolls a new
                        // seed and restarts.
                        const lander::GuardedAction action =
                            tap_guard.press(lander::GuardedKey::kNewSeed,
                                            SDL_GetTicks());
                        if (action == lander::GuardedAction::kNewSeed) {
                            seed = random_seed();
                            stars = lander::make_stars(seed, kWindowWidth,
                                                       kWindowHeight);
                            start_mission();
                        }
                        break;
                    }
                    case SDL_SCANCODE_E:
                        // M05-R4-03 / M05-R5: an unmodified, non-repeat
                        // `E` press toggles the stored reaction-wheel state.
                        // A `Shift+E` press is intentionally ignored here;
                        // the per-frame `Shift+E` hold below arms damping
                        // only while the keys are physically held.
                        reaction_wheels.press(
                            false,
                            (event.key.mod & SDL_KMOD_SHIFT) != 0);
                        break;
                    case SDL_SCANCODE_M:
                        // M05-R4-01: in a local view, M toggles AUTO and
                        // MANUAL. In SYSTEM view, M leaves SYSTEM directly
                        // into MANUAL (V still performs the normal
                        // save/restore round trip).
                        if (cam.system_view()) {
                            cam.enter_manual();
                        } else {
                            pending_cam_toggle = true;
                        }
                        break;
                    case SDL_SCANCODE_V:
                        // Toggle the system-scale view of the whole binary.
                        cam.set_system(!cam.system_view());
                        break;
                    case SDL_SCANCODE_P:
                        paused = !paused;
                        break;
                    case SDL_SCANCODE_G:
                        nav_overlay = !nav_overlay;
                        break;
                    case SDL_SCANCODE_O: {
                        // Triple-tap guarded (M05-R3-07).
                        const bool ccw =
                            (event.key.mod & SDL_KMOD_SHIFT) != 0;
                        const lander::GuardedAction action =
                            tap_guard.press(
                                ccw ? lander::GuardedKey::kCircularizeCCW
                                    : lander::GuardedKey::kCircularizeCW,
                                SDL_GetTicks());
                        if (action != lander::GuardedAction::kNone &&
                            !sim.state().landed && !sim.state().crashed) {
                            sim.circularize(ccw);
                        }
                        break;
                    }
                    case SDL_SCANCODE_B: {
                        // Triple-tap guarded (M05-R3-15): one-shot
                        // body-synchronous orbit initializer. Works from
                        // the ground too (source is the landed body).
                        const lander::GuardedAction action =
                            tap_guard.press(lander::GuardedKey::kSyncOrbit,
                                            SDL_GetTicks());
                        if (action == lander::GuardedAction::kSyncOrbit) {
                            sim.sync_orbit();
                        }
                        break;
                    }
                    case SDL_SCANCODE_T: {
                        // Triple-tap guarded (M05-R3-16): one-shot ballistic
                        // inter-body transfer initializer.
                        const lander::GuardedAction action =
                            tap_guard.press(lander::GuardedKey::kTransfer,
                                            SDL_GetTicks());
                        if (action == lander::GuardedAction::kTransfer) {
                            debug_message =
                                sim.transfer() ? "TRANSFER: SET"
                                               : "TRANSFER: NO SOLUTION";
                            debug_message_time = 3.0;
                        }
                        break;
                    }
                    case SDL_SCANCODE_F:
                        // Refill is allowed in flight and on the ground;
                        // only a crash blocks it.
                        if (!sim.state().crashed) {
                            sim.refuel();
                        }
                        break;
                    case SDL_SCANCODE_X:
                        throttle = 0.0;
                        if (transfer_mc.active()) {
                            transfer_mc.abort();
                            attitude_mode = lander::AttitudeMode::Off;
                            debug_message = "MIDCOURSE: ABORTED";
                            debug_message_time = 3.0;
                        }
                        if (node_executor.active()) {
                            node_executor.abort();
                            attitude_mode = lander::AttitudeMode::Off;
                        }
                        if (landing_ap.armed()) {
                            landing_ap.abort();
                            attitude_mode = lander::AttitudeMode::Off;
                            debug_message = "AUTOLAND: ABORTED";
                            debug_message_time = 3.0;
                        }
                        break;
                    case SDL_SCANCODE_F2:
                    case SDL_SCANCODE_F3:
                    case SDL_SCANCODE_F4: {
                        // M06-R8 (R8-06): in predictor isolation, explicitly
                        // switch the projection's prediction policy (COAST /
                        // LIVE / PLAN). Only the projected arc's policy
                        // changes; the live ship, its controls, and every
                        // other subsystem are untouched, and the predictor
                        // rebuilds through its normal invalidation path.
                        if (debug_mode == lander::DebugSubsystem::Predictor) {
                            panel_ctx.predictor_kind =
                                event.key.scancode == SDL_SCANCODE_F2
                                    ? lander::PredictionKind::Coast
                                    : (event.key.scancode == SDL_SCANCODE_F3
                                           ? lander::PredictionKind::Live
                                           : lander::PredictionKind::Plan);
                            debug_message = std::string("PREDICTOR: ") +
                                            prediction_kind_name(
                                                panel_ctx.predictor_kind);
                            debug_message_time = 2.0;
                        }
                        break;
                    }
                    case SDL_SCANCODE_9: {
                        // M06-R6: arm the target-pad powered-landing autopilot
                        // for the body the ship is currently flying around (the
                        // gravitational reference body), aiming at that body's
                        // base pad. The terminal ZEM/ZEV / Apollo-polynomial
                        // guidance converges over its bounded t_go scan; the
                        // maintained zero-thrust Coast predictor below is the
                        // O(1) zero-effort source (R6-03 / R6-09). Manual
                        // throttle or X reclaims control (aborts the autopilot).
                        if (!sim.state().crashed && !sim.state().landed) {
                            const int target = sim.reference_body();
                            const lander::Config& cfg = sim.config();
                            landing_horizon =
                                (int)std::lround(landing_cfg.t_go_max /
                                                 cfg.fixed_dt) +
                                50;
                            landing_coast.configure(landing_horizon, 1e-9);
                            lander::FlightPolicy coast_policy{};
                            coast_policy.kind = lander::PredictionKind::Coast;
                            lander::NodeExecutor coast_exec{};
                            landing_coast.cold_rebuild(sim, coast_policy,
                                                       coast_exec,
                                                       landing_horizon);
                            landing_ap.arm(
                                target, landing_cfg,
                                [&](double t_go) {
                                    return zero_from_predictor(
                                        sim.binary(), landing_coast, t_go,
                                        cfg.fixed_dt);
                                });
                            node_executor.clear();
                            transfer_mc.abort();
                            attitude_mode = lander::AttitudeMode::Off;
                            debug_message = "AUTOLAND: ENGAGED [9]";
                            debug_message_time = 3.0;
                        }
                        break;
                    }
                    case SDL_SCANCODE_C: {
                        if (!sim.state().crashed && !sim.state().landed &&
                            !node_executor.active()) {
                            const bool shift =
                                (event.key.mod & SDL_KMOD_SHIFT) != 0;
                            const int frame_body =
                                shift ? 1 - sim.reference_body()
                                      : sim.reference_body();
                            maneuver_node = lander::default_node(
                                sim.sim_time(), frame_body,
                                sim.config().fixed_dt);
                            node_executor.clear();
                        }
                        break;
                    }
                    case SDL_SCANCODE_DELETE:
                        if (!sim.state().crashed && !sim.state().landed) {
                            maneuver_node.reset();
                            node_executor.clear();
                        }
                        break;
                    case SDL_SCANCODE_H:
                    case SDL_SCANCODE_J: {
                        if (maneuver_node && !sim.state().crashed &&
                            !sim.state().landed && !node_executor.active()) {
                            const double delta =
                                event.key.scancode == SDL_SCANCODE_J ? 1.0
                                                                     : -1.0;
                            maneuver_node->time = lander::snap_time(
                                maneuver_node->time + delta,
                                sim.config().fixed_dt);
                        }
                        break;
                    }
                    case SDL_SCANCODE_K:
                    case SDL_SCANCODE_L: {
                        if (maneuver_node && !sim.state().crashed &&
                            !sim.state().landed && !node_executor.active()) {
                            const bool shift =
                                (event.key.mod & SDL_KMOD_SHIFT) != 0;
                            const double delta = shift ? -0.1 : 0.1;
                            if (event.key.scancode == SDL_SCANCODE_K) {
                                maneuver_node->dv_prograde += delta;
                            } else {
                                maneuver_node->dv_radial += delta;
                            }
                        }
                        break;
                    }
                    case SDL_SCANCODE_U:
                    case SDL_SCANCODE_I:
                    case SDL_SCANCODE_Y: {
                        if (!sim.state().crashed && !sim.state().landed &&
                            !node_executor.active()) {
                            const lander::State& st = sim.state();
                            const double t0 = sim.sim_time();
                            const lander::Config& cfg = sim.config();
                            const int ref = sim.reference_body();
                            const int dest = sim.contract().destination_body;
                            std::optional<lander::ManeuverNode> planned;
                            if (event.key.scancode == SDL_SCANCODE_U) {
                                planned = lander::plan_circularize(
                                    sim.binary(), cfg, st, t0, ref,
                                    maneuver_node);
                            } else if (event.key.scancode == SDL_SCANCODE_I) {
                                planned = lander::plan_transfer(sim.binary(),
                                                                cfg, st, t0,
                                                                ref,
                                                                maneuver_node,
                                                                &transfer_cache);
                            } else {
                                planned = lander::plan_match_target(
                                    sim.binary(), cfg, st, t0, ref, dest,
                                    maneuver_node);
                            }
                            if (planned.has_value()) {
                                maneuver_node = planned;
                                node_executor.clear();
                                debug_message.clear();
                                debug_message_time = 0.0;
                            } else {
                                debug_message = "PLANNER: NO SOLUTION";
                                debug_message_time = 3.0;
                            }
                        }
                        break;
                    }
                    case SDL_SCANCODE_Z: {
                        // M06-R5 / D05: engage the two-level transfer midcourse
                        // from a valid transfer plan (planned with I). The fast
                        // VGO arms coasting on the solved arc; bounded-rate
                        // warm re-aims (maybe_replan) re-target it as the ship
                        // flies. Distinct from the one-shot R4 node executor.
                        if (!sim.state().crashed && !sim.state().landed &&
                            !node_executor.active()) {
                            if (transfer_cache.valid) {
                                const lander::State& st = sim.state();
                                const double t0 = sim.sim_time();
                                const lander::Config& cfg = sim.config();
                                const int source =
                                    transfer_cache.source >= 0
                                        ? transfer_cache.source
                                        : sim.reference_body();
                                const lander::NodeBasis basis =
                                    lander::compute_node_basis(
                                        sim.binary(), t0, source,
                                        {st.x, st.y}, {st.vx, st.vy});
                                lander::ManeuverNode arm_node{};
                                arm_node.time = t0;
                                arm_node.frame_body = source;
                                transfer_mc.arm(arm_node, transfer_cache,
                                                source, basis, t0, cfg);
                                node_executor.clear();
                                landing_ap.abort();
                                attitude_mode = lander::AttitudeMode::Off;
                                debug_message = "MIDCOURSE: ENGAGED [Z]";
                                debug_message_time = 3.0;
                            } else {
                                debug_message =
                                    "MIDCOURSE: PLAN A TRANSFER FIRST [I]";
                                debug_message_time = 3.0;
                            }
                        }
                        break;
                    }
                    case SDL_SCANCODE_RETURN: {
                        const bool shift =
                            (event.key.mod & SDL_KMOD_SHIFT) != 0;
                        if (shift) {
                            if (transfer_mc.active()) {
                                transfer_mc.abort();
                                attitude_mode = lander::AttitudeMode::Off;
                            }
                            if (node_executor.active()) {
                                node_executor.abort();
                                attitude_mode = lander::AttitudeMode::Off;
                            }
                        } else if (maneuver_node && !sim.state().crashed &&
                                   !sim.state().landed &&
                                   !node_executor.active()) {
                            const lander::State& st = sim.state();
                            const double t0 = sim.sim_time();
                            const lander::Config& cfg = sim.config();
                            const double t = std::max(
                                t0,
                                lander::snap_time(maneuver_node->time,
                                                  cfg.fixed_dt));
                            const lander::BallisticState initial{
                                {st.x, st.y}, {st.vx, st.vy}, t0};
                            const int steps =
                                lander::ballistic_steps(t0, t, cfg.fixed_dt);
                            const lander::BallisticState pre =
                                lander::propagate_ballistic(
                                    sim.binary(), initial, steps,
                                    cfg.fixed_dt);
                            const lander::NodeBasis basis =
                                lander::compute_node_basis(
                                    sim.binary(), t, maneuver_node->frame_body,
                                    pre.p, pre.v);
                            node_executor.arm(*maneuver_node, basis, t0, cfg);
                            landing_ap.abort();
                            attitude_mode = lander::AttitudeMode::Maneuver;
                        }
                        break;
                    }
                    case SDL_SCANCODE_1:
                    case SDL_SCANCODE_2:
                    case SDL_SCANCODE_3:
                    case SDL_SCANCODE_4:
                    case SDL_SCANCODE_5:
                    case SDL_SCANCODE_6:
                    case SDL_SCANCODE_7:
                    case SDL_SCANCODE_8:
                        if (!node_executor.active()) {
                            switch (event.key.scancode) {
                                case SDL_SCANCODE_1:
                                    attitude_mode = lander::AttitudeMode::Off;
                                    break;
                                case SDL_SCANCODE_2:
                                    attitude_mode =
                                        lander::AttitudeMode::Prograde;
                                    break;
                                case SDL_SCANCODE_3:
                                    attitude_mode =
                                        lander::AttitudeMode::Retrograde;
                                    break;
                                case SDL_SCANCODE_4:
                                    attitude_mode =
                                        lander::AttitudeMode::RadialOut;
                                    break;
                                case SDL_SCANCODE_5:
                                    attitude_mode =
                                        lander::AttitudeMode::RadialIn;
                                    break;
                                case SDL_SCANCODE_6:
                                    attitude_mode =
                                        lander::AttitudeMode::Target;
                                    break;
                                case SDL_SCANCODE_7:
                                    attitude_mode =
                                        lander::AttitudeMode::AntiTarget;
                                    break;
                                default:
                                    attitude_mode =
                                        lander::AttitudeMode::Maneuver;
                                    break;
                            }
                        }
                        break;
                    default:
                        break;
                }
            } else if (event.type == SDL_EVENT_MOUSE_WHEEL) {
                // Accumulate vertical scroll notches; the camera consumes the
                // sum once per frame. Positive y scrolls away from the user.
                pending_wheel +=
                    static_cast<int>(std::lround(event.wheel.y));
            }
        }

        const Uint64 now_ns = SDL_GetTicksNS();
        // M06-R2-13: measure FPS from the unclamped real frame interval so the
        // counter reflects the true render rate; `dt` stays clamped for sim
        // stability.
        const double raw_dt = static_cast<double>(now_ns - prev_ns) * 1.0e-9;
        const double dt = std::min(0.25, raw_dt);
        prev_ns = now_ns;
        ticks_before_frame = sim.state().ticks;
        if (raw_dt > 0.0) {
            const double inst_fps = 1.0 / raw_dt;
            fps_ema = (fps_ema <= 0.0) ? inst_fps
                                       : (0.9 * fps_ema + 0.1 * inst_fps);
        }

        // Advance the continuous flame clock with real frame time. Frozen
        // while paused so the flame holds still with the rest of the scene.
        if (!paused) {
            flame_clock += dt;
            contract_banner_time =
                std::max(0.0, contract_banner_time - dt);
            debug_message_time = std::max(0.0, debug_message_time - dt);
        }

        // The polled state array is indexed by scancode (SDL3's
        // SDL_GetKeyboardState returns a const bool* of numkeys entries)
        // and is null when no keyboard is available; out-of-range
        // scancodes are treated as unpressed.
        int numkeys = 0;
        const bool* keys = SDL_GetKeyboardState(&numkeys);
        const auto key = [&](SDL_Scancode sc) {
            return keys != nullptr && sc < numkeys && keys[sc];
        };
        // Persistent main-engine throttle: it ramps while its key is held,
        // holds when the key is released, and is cut instantly by X. It is
        // independent of the camera mode.
        const bool throttle_up =
            key(SDL_SCANCODE_UP) || key(SDL_SCANCODE_W);
        const bool throttle_down =
            key(SDL_SCANCODE_DOWN) || key(SDL_SCANCODE_S);
        if (!paused) {
            if (throttle_up) {
                throttle += kThrottleRamp * dt;
            }
            if (throttle_down) {
                throttle -= kThrottleRamp * dt;
            }
            throttle = std::clamp(throttle, 0.0, 1.0);
        }

        const bool rw_hold =
            (SDL_GetModState() & SDL_KMOD_SHIFT) != 0 && key(SDL_SCANCODE_E);
        const bool manual_left = key(SDL_SCANCODE_RIGHT) || key(SDL_SCANCODE_D);
        const bool manual_right = key(SDL_SCANCODE_LEFT) || key(SDL_SCANCODE_A);

        if (!paused && node_executor.active() &&
            (throttle_up || throttle_down)) {
            node_executor.abort();
        }
        // Manual throttle reclaims control from the two-level midcourse too.
        if (!paused && transfer_mc.active() &&
            (throttle_up || throttle_down)) {
            transfer_mc.abort();
            attitude_mode = lander::AttitudeMode::Off;
        }
        // Manual throttle reclaims control from the landing autopilot too.
        if (!paused && landing_ap.armed() &&
            (throttle_up || throttle_down)) {
            landing_ap.abort();
            attitude_mode = lander::AttitudeMode::Off;
        }

        if (!paused && !sim.state().crashed) {
            const double fixed_dt = sim.config().fixed_dt;
            sim.set_accumulator(std::min(sim.accumulator() + dt, 10.0));
            while (sim.accumulator() >= fixed_dt) {
                sim.set_accumulator(sim.accumulator() - fixed_dt);

                const lander::State before = sim.state();
                const double now = sim.sim_time();
                lander::Input step_input{};

                const bool landing_active = landing_ap.armed();
                const bool mc_active = transfer_mc.active();
                const bool executor_active = node_executor.active();

                if (landing_active) {
                    // M06-R6: terminal ZEM/ZEV / Apollo-polynomial command,
                    // held and re-emitted at the 120 Hz control rate. It routes
                    // the commanded direction through the ordinary bang-bang
                    // attitude controller and gates the main throttle on nose
                    // alignment (R6-04). It never touches the live sim directly.
                    step_input =
                        landing_ap.make_input(before, sim.config(), now);
                } else if (mc_active) {
                    // WARM (bounded rate): re-aim the arc at most once per
                    // interval; never on every 1/120 s fixed step.
                    const bool warm_dbg =
                        debug_mode == lander::DebugSubsystem::TransferWarm;
                    const int prop_before =
                        warm_dbg ? lander::ballistic_propagation_count() : 0;
                    const double miss_before =
                        warm_dbg ? transfer_mc.cache().achieved_miss : 0.0;
                    const bool replanned = transfer_mc.maybe_replan(
                        sim.binary(), sim.config(), before, now,
                        kMidcourseReplanSec, kMidcourseMissTolerance);
                    if (warm_dbg && replanned) {
                        // Debug observation only (R8-11): the last re-plan's
                        // miss / correction count / propagation cost / cold
                        // fallback, stored on the panel context. It never
                        // feeds back into the midcourse (P07).
                        transfer_debug.warm_miss_before = miss_before;
                        transfer_debug.warm_miss_after =
                            transfer_mc.cache().achieved_miss;
                        transfer_debug.warm_propagations_last =
                            lander::ballistic_propagation_count() -
                            prop_before;
                        transfer_debug.warm_propagations_total +=
                            transfer_debug.warm_propagations_last;
                        transfer_debug.warm_fallback_last =
                            transfer_mc.cache().valid &&
                            transfer_mc.cache().newton_iterations == 0;
                    }
                    // HOT (O(1), every fixed step): the fast VGO drives
                    // attitude and the finite correction burn.
                    step_input = transfer_mc.make_input(
                        before, now, sim.config(), manual_left, manual_right);
                } else if (executor_active) {
                    step_input = node_executor.make_input(
                        before, now, sim.config(), manual_left, manual_right);
                } else {
                    const auto& bin = sim.binary();
                    const int ref = sim.reference_body();
                    const lander::NodeBasis current_basis =
                        lander::compute_node_basis(
                            bin, now, ref, {before.x, before.y},
                            {before.vx, before.vy});
                    const auto dest_pad = destination_pad_at(now);
                    lander::Vec2 maneuver_dv{};
                    if (attitude_mode == lander::AttitudeMode::Maneuver &&
                        maneuver_node && prediction_valid &&
                        prediction.basis_valid) {
                        maneuver_dv = prediction.dv_world;
                    }
                    const auto direction = lander::attitude_target_direction(
                        attitude_mode, current_basis, {before.x, before.y},
                        dest_pad.position, maneuver_dv);
                    step_input = lander::attitude_input(
                        before, sim.config(), direction, manual_left,
                        manual_right);
                    step_input.main_throttle = throttle;
                }

                step_input.reaction_wheels = reaction_wheels.input(
                    step_input.rotate_left || step_input.rotate_right,
                    !before.crashed, rw_hold);

                // M06-R8 (R8-05..R8-15): remember the last composed per-step
                // input for the debug panels (display only).
                panel_ctx.last_step_input = step_input;

                (void)sim.step_once(step_input);

                if (landing_active) {
                    // Advance the zero-thrust Coast predictor (the autopilot's
                    // O(1) zero-effort source). Under thrust the live state
                    // never matches the coast, so this rebuilds from the
                    // current state each step (cheap); the ring is always
                    // projected forward from the latest actual state.
                    if (!sim.state().crashed && !sim.state().landed) {
                        lander::FlightPolicy coast_policy{};
                        coast_policy.kind = lander::PredictionKind::Coast;
                        lander::NodeExecutor coast_exec{};
                        landing_coast.advance(sim, sim.state(), coast_policy,
                                              coast_exec, landing_horizon);
                    }
                    // Recompute the guidance at its own cadence and advance the
                    // phase machine; the command is held for the next ticks.
                    landing_ap.after_step(before, sim.state(), sim.binary(),
                                          sim.config(), sim.sim_time());
                } else if (mc_active) {
                    transfer_mc.after_step(before, sim.state(), step_input,
                                           sim.sim_time(), sim.config());
                } else {
                    node_executor.after_step(before, sim.state(), step_input,
                                             sim.sim_time(), sim.config());
                }
                if (executor_active && !node_executor.active() &&
                    !sim.state().crashed && !sim.state().landed) {
                    attitude_mode = lander::AttitudeMode::Off;
                }

                // M06-R3: roll the live-trajectory projection one fixed step
                // using the current persistent controls (the "no further input"
                // powered forecast). In steady state this is a single
                // shift+append; a control change triggers a bounded cold
                // rebuild. The predictor reuses the live Simulation-stepping
                // core, so its contact outcome is authoritative.
                if (!sim.state().crashed && !sim.state().landed) {
                    lander::FlightPolicy live_policy{};
                    // M06-R8 (R8-06): in predictor isolation the user can
                    // explicitly switch the projection's policy (F2 COAST /
                    // F3 LIVE / F4 PLAN). Outside that mode the policy is
                    // always LIVE, exactly as before the harness existed.
                    live_policy.kind =
                        (debug_mode == lander::DebugSubsystem::Predictor)
                            ? panel_ctx.predictor_kind
                            : lander::PredictionKind::Live;
                    live_policy.throttle = throttle;
                    live_policy.attitude_mode = attitude_mode;
                    live_policy.rw_enabled = reaction_wheels.enabled();
                    live_policy.rw_hold = rw_hold;
                    if (maneuver_node.has_value()) {
                        live_policy.node = maneuver_node;
                    }
                    live_policy.reference_body = sim.reference_body();
                    live_policy.target_body = sim.contract().destination_body;
                    const bool pred_dbg =
                        debug_mode == lander::DebugSubsystem::Predictor;
                    const auto pred_clock0 =
                        pred_dbg ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
                    live_predictor.advance(sim, sim.state(), live_policy,
                                           node_executor, kPredictBudget);
                    if (pred_dbg) {
                        // Debug observation only (R8-06): the low-rate cost of
                        // the advance and the policy signature (and whether it
                        // changed, which forces a cold rebuild). Display only.
                        const double pred_ms =
                            std::chrono::duration_cast<std::chrono::duration<
                                double, std::milli>>(
                                std::chrono::steady_clock::now() - pred_clock0)
                                .count();
                        panel_ctx.predictor_ms_ema =
                            (panel_ctx.predictor_ms_ema <= 0.0)
                                ? pred_ms
                                : (0.9 * panel_ctx.predictor_ms_ema +
                                   0.1 * pred_ms);
                        const lander::PolicySignature sig =
                            lander::make_policy_signature(live_policy,
                                                          node_executor);
                        panel_ctx.predictor_signature_changed =
                            (sig != panel_ctx.predictor_signature);
                        panel_ctx.predictor_signature = sig;
                    }
                }

                if (sim.state().crashed) {
                    // A crash ends the run: clear the flight computer and stop
                    // draining this frame's remaining fixed steps (the
                    // simulation does not advance after a crash).
                    reset_flight_computer();
                    break;
                }
                if (sim.state().landed && !before.landed) {
                    // Just landed from flight: the flight plan is over, so
                    // clear it -- but do NOT break. The ship stays attached to
                    // its moving body and system time keeps advancing, so this
                    // frame's remaining fixed steps must still drain, exactly
                    // as M05's Simulation::advance always did. Breaking here
                    // (the old `crashed || landed` test) let the accumulator
                    // backlog grow unbounded while on the ground and be flushed
                    // all at once on the next takeoff, launching the ship
                    // abruptly into space.
                    reset_flight_computer();
                }
            }
        }

        const lander::State& s = sim.state();
        // Presentation uses one coherent interpolated time for everything
        // drawn in the frame: the ship (linear blend between the previous
        // and current authoritative states, or the attached pose on the
        // moving surface) and both moons' ephemeris positions all use
        // t_present. Physics, collision, fuel, scoring, and the HUD keep
        // using the authoritative states only.
        const bool stepped = !(sim.previous_state() == s);
        const double t_present =
            (paused || !stepped) ? sim.sim_time() : sim.presentation_time();

        lander::State render_state;
        if (s.landed) {
            render_state = lander::attached_state(
                sim.binary(), s.landed_body, s.landed_arc, t_present);
        } else if (s.crashed || paused || !stepped) {
            render_state = s;
        } else {
            const double render_alpha =
                std::clamp(sim.accumulator() / sim.config().fixed_dt, 0.0,
                           1.0);
            render_state = lander::interpolated_state(
                sim.previous_state(), s, render_alpha, false);
        }

        const lander::BinarySystem& bin = sim.binary();
        // Both bodies share the same prograde spin rate (tidal locking), so
        // one rotation value at the presentation time drives everything
        // derived from the rotating surfaces this frame.
        const double body_rot = bin.body_rotation(t_present);
        const int ref = sim.reference_body();
        const lander::Vec2 ref_pos = bin.position(ref, t_present);

        // The camera follows the interpolated lander position and chooses
        // its scale from the lander's radial altitude above the reference
        // body's terrain directly below it. It runs even while paused so the
        // mode toggle and wheel stay responsive; while paused the rendered
        // state is the frozen authoritative state, so the camera simply
        // holds.
        const double altitude = std::max(
            0.0,
            lander::altitude_at(bin.body(ref).terrain, render_state, ref_pos,
                                body_rot));
        const double target_angle =
            lander::local_up_angle(render_state, ref_pos);
        const int nav_dest = sim.contract().destination_body;
        // The contract destination is the base pad: a point fixed on the
        // destination body's rotating surface (M05-R3 tidal locking), so the
        // camera's system-destination and the offscreen indicator track the
        // moving pad.
        const lander::BinarySystem::SurfacePoint nav_dest_pad =
            bin.surface_point(
                nav_dest, bin.body(nav_dest).terrain.angle_at_arc(0.0),
                bin.body(nav_dest).terrain.surface_radius_at_arc(0.0),
                t_present);
        const double nav_dest_x = nav_dest_pad.position.x;
        const double nav_dest_y = nav_dest_pad.position.y;
        if (cam.system_view()) {
            cam.set_system_destination(nav_dest_x, nav_dest_y);
        } else {
            cam.clear_system_destination();
        }
        cam.update(dt, render_state.x, render_state.y, altitude,
                   pending_wheel, pending_cam_toggle, target_angle);
        pending_wheel = 0;
        pending_cam_toggle = false;

        // The flame is only present when the engine can actually burn, and
        // its size follows the throttle rather than a binary on/off state.
        const double thrust_level =
            (s.fuel > 0.0 && !s.landed && !s.crashed)
                ? std::clamp(throttle, 0.0, 1.0)
                : 0.0;

        // A new contract completion shows its banner for a few seconds.
        const auto& last_completed = sim.last_completed();
        if (last_completed.has_value() != last_completed_seen.has_value() ||
            (last_completed.has_value() &&
              !(*last_completed == *last_completed_seen))) {
            last_completed_seen = last_completed;
            contract_banner_time = kContractBannerTime;
            reset_flight_computer();
        }

        pc_message.clear();
        if (!s.crashed && !s.landed) {
            const double sim_now = sim.sim_time();
            const bool cadence_due =
                !prediction_valid ||
                (sim_now - predict_last_sim_time) >= kPredictRefreshSec;
            const bool inputs_changed = prediction_inputs_changed();

            const auto pred_t0 = std::chrono::steady_clock::now();
            if (cadence_due || inputs_changed) {
                const double horizon = 2.0 * sim.binary().period();
                prediction = lander::predict_trajectory(
                    sim.binary(), sim.config(), s, sim.sim_time(),
                    sim.reference_body(), sim.contract().destination_body,
                    maneuver_node, horizon, 512);
                prediction_valid = true;
                predict_last_sim_time = sim_now;
                predict_ref_body = sim.reference_body();
                predict_dest_body = sim.contract().destination_body;
                predict_node_present = maneuver_node.has_value();
                if (maneuver_node.has_value()) {
                    predict_node_time = maneuver_node->time;
                    predict_node_frame_body = maneuver_node->frame_body;
                    predict_node_prograde = maneuver_node->dv_prograde;
                    predict_node_radial = maneuver_node->dv_radial;
                } else {
                    predict_node_time = 0.0;
                    predict_node_frame_body = -1;
                    predict_node_prograde = 0.0;
                    predict_node_radial = 0.0;
                }
            }
            // When the cadence tick has not arrived and nothing invalidated the
            // cache, `prediction` is reused as-is: the ~52k-step arc is not
            // rebuilt this frame (M06-R2-11 / M06-R2-D06).
            const double predict_ms =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    std::chrono::steady_clock::now() - pred_t0)
                    .count();
            if (frame_debug) {
                std::printf(
                    "frame=%d dt=%.4fs pred=%.2fms steps=%d sim_t=%.3f "
                    "landed=%d crashed=%d\n",
                    frame, dt, predict_ms,
                    sim.state().ticks - ticks_before_frame, sim.sim_time(),
                    (int)sim.state().landed, (int)sim.state().crashed);
            }

            if (attitude_mode != lander::AttitudeMode::Off) {
                const auto& bin = sim.binary();
                const lander::NodeBasis current_basis =
                    lander::compute_node_basis(
                        bin, sim.sim_time(), sim.reference_body(), {s.x, s.y},
                        {s.vx, s.vy});
                lander::Vec2 maneuver_dv{};
                if (attitude_mode == lander::AttitudeMode::Maneuver &&
                    maneuver_node && prediction.basis_valid) {
                    maneuver_dv = prediction.dv_world;
                }
                const auto direction = lander::attitude_target_direction(
                    attitude_mode, current_basis, {s.x, s.y},
                    destination_pad_at(sim.sim_time()).position,
                    maneuver_dv);
                if (!direction.has_value()) {
                    pc_message = "ATT INVALID";
                }
            }

            if (maneuver_node && !prediction.basis_valid) {
                pc_message = "NODE BASIS INVALID";
            }
        } else {
            reset_flight_computer();
        }

        // The backdrop rotates with the final presentation angle (M05-R3-12).
        draw_space(renderer, stars, cam.angle());
        const int dest = sim.contract().destination_body;
        const lander::Vec2 other_pos = bin.position(1 - ref, t_present);
        draw_body(renderer, bin.body(1 - ref), other_pos, cam, render_state,
                  1 - ref == dest, body_rot);
        draw_body(renderer, bin.body(ref), ref_pos, cam, render_state,
                  ref == dest, body_rot);
        draw_lander(renderer, render_state, thrust_level, flame_clock, cam);
        if (s.crashed) {
            // Time is frozen after a crash, so the crash body's centre at
            // sim_time() is exactly where the wreck was placed.
            draw_debris(renderer, s, cam, seed,
                         bin.body(s.crash_body < 0 ? 0 : s.crash_body),
                         bin.position(s.crash_body < 0 ? 0 : s.crash_body,
                                      sim.sim_time()));
        }
        // M06-R8: subsystem-isolation debug harness. When a --debug-subsystem
        // selector is active, hide the unrelated nav / prediction /
        // flight-computer / contract surfaces and show the isolated
        // subsystem's expanded panel (common minimum readout plus the full
        // mode-specific detail). The `ui` isolation is the exception: it keeps
        // every normal player-facing surface and adds only a minimal debug
        // header. `node-edit` keeps the planned-arc overlay so the edited node
        // and its pre/post branches stay visible while editing. With no
        // selector (or `none`) the rendering path is unchanged, so normal
        // gameplay is untouched.
        const bool debug_active = debug_mode != lander::DebugSubsystem::None;
        const bool debug_ui = debug_mode == lander::DebugSubsystem::Ui;
        // M06-R8 (follow-up): the predictor isolation keeps its primary
        // graphical observable -- the predicted-trajectory overlay -- visible
        // by default (not gated on the G toggle) so the projected path can be
        // read alongside the predictor-specific debug numbers. It reuses the
        // exact normal-gameplay drawing calls (no new rendering); the gravity
        // / orbit nav field is left out to keep the view on the trajectory.
        const bool predictor_keeps_overlay =
            lander::keeps_prediction_overlay(debug_mode);
        if (debug_active) {
            // M06-R8 (R8-05..R8-15): refresh the panel context from values the
            // frame already has (display only; it never feeds back in).
            panel_ctx.throttle = throttle;
            panel_ctx.rw_enabled = reaction_wheels.enabled();
            panel_ctx.rw_hold = rw_hold;
            panel_ctx.rotate_left = manual_left;
            panel_ctx.rotate_right = manual_right;
            panel_ctx.prediction = &prediction;
            panel_ctx.prediction_valid = prediction_valid;
        }
        if (predictor_keeps_overlay && !s.crashed && !paused) {
            // M06-R8 (follow-up): the predictor's predicted-trajectory overlay,
            // kept visible. The same calls normal gameplay uses, so the
            // projected path is the mode's visible observable: the COAST / PLAN
            // arc (with its PE / AP / closest-approach / impact markers), the
            // powered LIVE projection, and the three-kind legend. F2 / F3 / F4
            // re-point the live predictor's policy, so the visible projection
            // follows the selected kind.
            if (prediction_valid && !s.landed) {
                draw_trajectory(renderer, prediction, cam,
                                maneuver_node.has_value(), sim);
            }
            if (!s.landed) {
                draw_live_prediction(renderer, live_predictor, sim, cam);
            }
            draw_prediction_legend(renderer);
        } else if ((!debug_active || debug_ui) && nav_overlay && !s.crashed &&
                   !paused) {
            if (prediction_valid && !s.landed) {
                draw_trajectory(renderer, prediction, cam,
                                maneuver_node.has_value(), sim);
            }
            if (!s.landed) {
                // M06-R3: the powered LIVE projection and its predicted-contact
                // marker, plus the legend distinguishing the three kinds.
                draw_live_prediction(renderer, live_predictor, sim, cam);
            }
            draw_navigation_overlay(renderer, sim, render_state, cam);
            draw_prediction_legend(renderer);
        } else if (debug_mode == lander::DebugSubsystem::NodeEdit && nav_overlay &&
                   !s.crashed && !s.landed && !paused && prediction_valid) {
            // R8-08: keep the pre/post prediction arc visible while editing.
            draw_trajectory(renderer, prediction, cam,
                            maneuver_node.has_value(), sim);
        }
        lander::DebugCommonReadout debug_common{};
        if (debug_active) {
            debug_common = lander::make_common_readout(sim);
        }
        if (debug_active && !debug_ui) {
            draw_debug_subsystem_panel(
                renderer, debug_mode, debug_common, sim, node_executor,
                transfer_mc, landing_ap, live_predictor, maneuver_node,
                transfer_debug, attitude_mode, panel_ctx);
        } else {
            draw_hud(renderer, sim, seed, throttle,
                     reaction_wheels.enabled(), rw_hold, cam,
                     tap_guard.progress_label(SDL_GetTicks()));
        }
        // M06-R2-13: on-screen frames-per-second counter, top-right corner
        // (above the flight-computer panel, clear of the left HUD).
        {
            const Color dim(130, 138, 156);
            char fps_line[24];
            std::snprintf(fps_line, sizeof fps_line, "FPS %.0f", fps_ema);
            draw_text(renderer, fps_line,
                      kWindowWidth - 8 - text_width(fps_line, 2), 16, 2, dim);
        }
        if (!debug_active || debug_ui) {
            draw_flight_computer(renderer, sim, maneuver_node, prediction.basis,
                                  prediction.basis_valid, prediction,
                                  prediction_valid, node_executor, attitude_mode,
                                  pc_message, &transfer_mc, &landing_ap);
            draw_contract_banner(renderer, last_completed_seen,
                                  contract_banner_time);
        }
        draw_landed_banner(renderer, s);
        if (debug_ui) {
            // R8-15: the ui isolation keeps all normal surfaces; the panel
            // call here only adds the minimal debug header.
            draw_debug_subsystem_panel(
                renderer, debug_mode, debug_common, sim, node_executor,
                transfer_mc, landing_ap, live_predictor, maneuver_node,
                transfer_debug, attitude_mode, panel_ctx);
        }
        if (debug_message_time > 0.0 && !debug_message.empty()) {
            const Color amber(255, 196, 64);
            draw_center_text(renderer, debug_message,
                              kWindowHeight / 2 + 150, 2, amber);
        }
        if (cam.system_view()) {
            const lander::OffscreenIndicator indicator =
                lander::offscreen_target_indicator(nav_dest_x, nav_dest_y,
                                                   cam);
            if (!indicator.on_screen) {
                const Color amber(255, 196, 64);
                const double len = 14.0;
                const Vec2 tip{indicator.x + indicator.dir_x * len,
                               indicator.y + indicator.dir_y * len};
                const Vec2 perp{-indicator.dir_y, indicator.dir_x};
                const Vec2 left{indicator.x - indicator.dir_x * len * 0.7 +
                                    perp.x * len * 0.55,
                                 indicator.y - indicator.dir_y * len * 0.7 +
                                     perp.y * len * 0.55};
                const Vec2 right{indicator.x - indicator.dir_x * len * 0.7 -
                                     perp.x * len * 0.55,
                                  indicator.y - indicator.dir_y * len * 0.7 -
                                      perp.y * len * 0.55};
                fill_poly(renderer, {tip, left, right}, amber);
            }
        }
        draw_overlay(renderer, s, paused);
        SDL_RenderPresent(renderer);

        ++frame;
        if (fps_cap > 0) {
            const Uint64 budget_ns =
                1000000000ULL / static_cast<Uint64>(fps_cap);
            const Uint64 now_after_ns = SDL_GetTicksNS();
            const Uint64 spent = now_after_ns - now_ns;
            if (spent < budget_ns) {
                SDL_Delay(
                    static_cast<Uint32>((budget_ns - spent) / 1000000ULL));
            }
        }
        if (max_frames > 0 && frame >= max_frames) {
            running = false;
        }
    }

    if (!screenshot_path.empty()) {
        if (!write_ppm(renderer, screenshot_path.c_str())) {
            std::fprintf(stderr, "Failed to write screenshot %s\n",
                         screenshot_path.c_str());
        }
    }

    const lander::State& final_state = sim.state();
    std::printf(
        "final: x=%.3f y=%.3f vx=%.3f vy=%.3f angle=%.3f fuel=%.2f "
        "ticks=%llu state=%s score=%d contracts=%d ref=%d seed=%llu\n",
        final_state.x, final_state.y, final_state.vx, final_state.vy,
        final_state.angle, final_state.fuel,
        static_cast<unsigned long long>(final_state.ticks),
        final_state.landed ? "landed" : (final_state.crashed ? "crashed"
                                                             : "flying"),
        final_state.score, sim.contracts_completed(), sim.reference_body(),
        static_cast<unsigned long long>(seed));

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
