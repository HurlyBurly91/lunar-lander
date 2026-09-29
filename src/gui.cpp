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

#include "lander/binary.hpp"
#include "lander/camera.hpp"
#include "lander/guarded_actions.hpp"
#include "lander/render_geom.hpp"
#include "lander/sim.hpp"
#include "lander/starfield.hpp"

#include <algorithm>
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

const Glyph* glyph(char c) {
    switch (c) {
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
                const lander::State& ship, bool full_body,
                bool highlight_base, double body_rotation) {
    const lander::Terrain& terrain = body.terrain;
    const double scale = cam.scale();
    const double C = terrain.circumference();
    const double theta_ship = std::atan2(ship.y - bpos.y, ship.x - bpos.x);
    // The local-view window is centred on the terrain arc that currently
    // faces the ship: the ship's world direction minus the body's spin.
    const double s_target = terrain.arc_at_angle(theta_ship - body_rotation);

    double half_angle = 0.0;
    double u_start = 0.0;
    double u_end = C;
    if (!full_body) {
        // Local view: an arc window centred on the direction from this
        // body's centre to the ship (the M04 framing, now body-relative).
        const double target_r =
            std::max(1.0, std::hypot(ship.x - bpos.x, ship.y - bpos.y));
        half_angle =
            (kWindowWidth / 2.0 + 256.0) / scale / target_r;
        half_angle = std::clamp(half_angle, 0.10, 0.75);
        const double half_arc = half_angle * terrain.reference_radius();
        u_start = s_target - half_arc;
        u_end = s_target + half_arc;
    }

    // The surface is one continuous, shared-vertex mesh (M05-R3-18): an
    // interior fill plus two radial annulus bands built from the same ring
    // vertices. Rendering the rim/pads as single closed polygons, instead of
    // hundreds of independent thick-line quads whose per-segment offsets never
    // met at the joints, removes the black radial seams and the per-segment
    // overdraw that banded the surface at wide zooms (M05-R3-17), while the
    // tessellation count stays bounded at every zoom.
    lander::SurfaceRing ring = lander::body_surface_ring(
        body, bpos, cam, lander::Vec2{ship.x, ship.y}, full_body,
        body_rotation);
    const std::vector<lander::Vec2>& l_outer = ring.outer;
    if (l_outer.size() < 2) {
        return;
    }
    const lander::Vec2 l_centre = lander::to_screen_point(bpos.x, bpos.y, cam);

    std::vector<Vec2> polygon;
    polygon.reserve(l_outer.size() + 2);
    for (const auto& p : l_outer) {
        polygon.push_back({p.x, p.y});
    }
    if (!full_body) {
        // Close the sampled arc on the interior/downward side of the surface,
        // well outside the viewport, so the moon body does not show radial
        // chords back to the moon centre.
        const double bottom = kWindowHeight + 512.0;
        polygon.push_back({polygon.back().x, bottom});
        polygon.push_back({polygon.front().x, bottom});
    }
    fill_poly(renderer, polygon, make_color(66, 70, 82));

    // Dark inner rim: one continuous 8-px band centred on the inner ring.
    fill_poly(renderer,
              to_vec2s(lander::thick_ring(ring.inner, l_centre, 4.0, 4.0)),
              make_color(58, 62, 74));
    // Light surface edge: one continuous 2.5-px band centred on the surface.
    fill_poly(renderer,
              to_vec2s(lander::thick_ring(ring.outer, l_centre, 1.25, 1.25)),
              make_color(125, 130, 145));

    // Small surface ticks are useful up close; at system scale they would
    // just add noise.
    if (scale > 0.35) {
        const double tick_start =
            full_body ? 0.0 : std::ceil(u_start / 10.0) * 10.0;
        const double tick_end = full_body ? C - 1.0e-9 : u_end;
        for (double u = tick_start; u <= tick_end + 1.0e-9; u += 10.0) {
            const double theta = terrain.angle_at_arc(u) + body_rotation;
            const double r = terrain.surface_radius_at_arc(u);
            const Vec2 a = to_screen(bpos.x + std::cos(theta) * r,
                                     bpos.y + std::sin(theta) * r, cam);
            const Vec2 b = to_screen(bpos.x + std::cos(theta) * (r - 0.5),
                                     bpos.y + std::sin(theta) * (r - 0.5),
                                     cam);
            draw_thick_line(renderer, a, b, 1.0, make_color(44, 47, 58));
        }
    }

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
    draw_text(renderer, "L/R ROT  M CAM  V SYSTEM  G NAV", 18,
              kWindowHeight - 56, 1, dim);
    draw_text(renderer, "E WHEELS  O X3 CIRC  SH+O X3 CCW", 18,
              kWindowHeight - 40, 1, dim);
    draw_text(renderer, "F FUEL  P PAUSE  N X3 SEED  R X3 RETRY", 18,
              kWindowHeight - 24, 1, dim);
    draw_text(renderer, "B X3 SYNC ORBIT  T X3 TRANSFER  WHEEL ZOOM", 18,
              kWindowHeight - 8, 1, dim);

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
        "  --help           show this message\n"
        "Controls: Up/W increase throttle, Down/S decrease throttle, X\n"
        "throttle cutoff, Left/Right/A/D rotate, M camera mode (Auto/Manual),\n"
        "V system view, mouse wheel zoom, O x3 clockwise circularize around\n"
        "the reference body, Shift+O x3 counter-clockwise circularize, E\n"
        "reaction-wheel angular damping, G navigation/gravity overlay, F\n"
        "refill fuel (also on the ground), N x3 new seed, P pause, Esc/Q\n"
        "quit.\n"
        "Developer controls (all triple-tap guarded, progress shown on the\n"
        "HUD, firing on the third tap within 700 ms):\n"
        "  R x3  retry the same seed\n"
        "  B x3  body-synchronous orbit around the source body (the landed\n"
        "        body when landed, else the reference body), on the side\n"
        "        opposite the other body\n"
        "  T x3  ballistic inter-body transfer to the other body; shows\n"
        "        'TRANSFER: NO SOLUTION' if none exists for this phase\n");
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
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            print_usage();
            return 2;
        }
    }
    if (!have_seed) {
        seed = random_seed();
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

    // M05-R3-07: triple-tap guard for the dangerous controls (new seed,
    // circularize CW/CCW).
    lander::TripleTapGuard tap_guard;

    auto start_mission = [&]() {
        sim.reset(seed);
        throttle = 0.0;
        paused = false;
        flame_clock = 0.0;
        tap_guard.reset();
        last_completed_seen = sim.last_completed();
        contract_banner_time = 0.0;
        debug_message.clear();
        debug_message_time = 0.0;
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
                    case SDL_SCANCODE_X:
                        // Immediate main-engine cutoff.
                        throttle = 0.0;
                        break;
                    case SDL_SCANCODE_M:
                        // Toggle between the automatic and the manual local
                        // camera. (Ignored while the system view is up; V
                        // leaves it.)
                        pending_cam_toggle = true;
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
        const double dt = std::min(
            0.25, static_cast<double>(now_ns - prev_ns) * 1.0e-9);
        prev_ns = now_ns;

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

        lander::Input input;
        input.main_throttle = throttle;
        // The simulation's rotate_left decreases the angle (a clockwise
        // turn), so the visually-left keys drive rotate_right and vice
        // versa. This keeps the on-screen feel natural without touching
        // the preserved physics.
        input.rotate_right = key(SDL_SCANCODE_LEFT) || key(SDL_SCANCODE_A);
        input.rotate_left = key(SDL_SCANCODE_RIGHT) || key(SDL_SCANCODE_D);
        input.reaction_wheels = key(SDL_SCANCODE_E);

        if (!paused) {
            sim.advance(dt, input);
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
        }

        // The backdrop rotates with the final presentation angle (M05-R3-12).
        draw_space(renderer, stars, cam.angle());
        const bool sys_view = cam.system_view();
        const int dest = sim.contract().destination_body;
        const lander::Vec2 other_pos = bin.position(1 - ref, t_present);
        draw_body(renderer, bin.body(1 - ref), other_pos, cam, render_state,
                  true, 1 - ref == dest, body_rot);
        draw_body(renderer, bin.body(ref), ref_pos, cam, render_state,
                  sys_view, ref == dest, body_rot);
        draw_lander(renderer, render_state, thrust_level, flame_clock, cam);
        if (s.crashed) {
            // Time is frozen after a crash, so the crash body's centre at
            // sim_time() is exactly where the wreck was placed.
            draw_debris(renderer, s, cam, seed,
                         bin.body(s.crash_body < 0 ? 0 : s.crash_body),
                         bin.position(s.crash_body < 0 ? 0 : s.crash_body,
                                      sim.sim_time()));
        }
        if (nav_overlay && !s.crashed && !paused) {
            draw_navigation_overlay(renderer, sim, render_state, cam);
        }
        draw_hud(renderer, sim, seed, throttle, cam,
                 tap_guard.progress_label(SDL_GetTicks()));
        draw_contract_banner(renderer, last_completed_seen,
                              contract_banner_time);
        draw_landed_banner(renderer, s);
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
