// Lunar Lander: a playable SDL3 game around a small closed circular moon.
//
// This file is intentionally thin over lander::Simulation: all physics
// (fixed 1/120 s timestep, fuel, radial inverse-square gravity, local-frame
// landing/crash rules, scoring, determinism) lives in include/lander/sim.hpp
// and lander::Terrain (include/lander/terrain.hpp) and is covered headlessly
// by tests/test_sim.cpp. The GUI only
//   (a) feeds keyboard input into the simulation,
//   (b) advances it with elapsed real time, and
//   (c) renders the curved world, lander, landing sites, and a HUD.
//
// World space: global inertial x/y with the moon at the origin, angle 0 =
// thrust toward +y, positive angle = counter-clockwise. The camera rotates
// with the local surface frame, so screen up follows the local outward
// direction.

#include <SDL3/SDL.h>

#include "lander/camera.hpp"
#include "lander/sim.hpp"
#include "lander/starfield.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}};
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

std::string fmt1(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.1f", value);
    return buffer;
}

// ------------------------------------------------------------------- camera

// The single world-to-screen transform. Every piece of world geometry (the
// lander, terrain, pads, guides, debris) is drawn through this, so the camera
// can only rigidly rotate, move, and scale the fixed world-space geometry,
// never change its shape. Coordinates stay floating point here; SDL
// quantizes to pixels.
Vec2 to_screen(double world_x, double world_y, const lander::Camera& cam) {
    const double dx = world_x - cam.center_x();
    const double dy = world_y - cam.center_y();
    const double c = std::cos(cam.angle());
    const double s = std::sin(cam.angle());
    const double local_x = dx * c + dy * s;
    const double local_y = -dx * s + dy * c;
    return {kWindowWidth / 2.0 + local_x * cam.scale(),
            kWindowHeight / 2.0 - local_y * cam.scale()};
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
    double top = 1e30;
    double bottom = -1e30;
    for (const auto& p : pts) {
        top = std::min(top, p.y);
        bottom = std::max(bottom, p.y);
    }
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, alpha);
    const int row_begin =
        std::max(0, static_cast<int>(std::floor(top)));
    const int row_end =
        std::min(kWindowHeight - 1, static_cast<int>(std::ceil(bottom)));
    const int n = static_cast<int>(pts.size());
    for (int row = row_begin; row <= row_end; ++row) {
        const double yc = row + 0.5;
        std::vector<double> xs;
        for (int i = 0; i < n; ++i) {
            const auto& a = pts[i];
            const auto& b = pts[(i + 1) % n];
            if ((a.y > yc) != (b.y > yc)) {
                const double t = (yc - a.y) / (b.y - a.y);
                xs.push_back(a.x + t * (b.x - a.x));
            }
        }
        std::sort(xs.begin(), xs.end());
        for (size_t i = 0; i + 1 < xs.size(); i += 2) {
            const int x0 = static_cast<int>(std::floor(xs[i]));
            const int x1 = static_cast<int>(std::ceil(xs[i + 1]) - 1);
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
// The starfield is a cosmetic background generated deterministically from the
// game seed (see lander/starfield.hpp). The stars are treated as infinitely
// distant: they do not move with camera translation or zoom, but they rotate
// around the viewport center with the local-frame camera so the horizon
// stays stable while the player rotates or orbits the moon.
void draw_space(SDL_Renderer* renderer,
                const std::vector<lander::Star>& stars,
                const lander::Camera& cam) {
    fill_rect(renderer, 0, 0, kWindowWidth, kWindowHeight,
              make_color(8, 10, 22));
    for (const lander::Star& star : stars) {
        const lander::ScreenPoint p =
            lander::star_screen_pos(star, kWindowWidth / 2.0,
                                    kWindowHeight / 2.0, cam.angle(),
                                    cam.center_x(), cam.center_y(),
                                    cam.scale());
        fill_rect(renderer, static_cast<int>(p.x), static_cast<int>(p.y),
                  star.size, star.size,
                  make_color(star.bright, star.bright,
                             std::min<Uint8>(255, star.bright + 20)));
    }
}

// ------------------------------------------------------------------ terrain

// Renders the actual collision surface as a closed arc around the moon.
// Samples are taken on a fixed uniform arc lattice, so camera motion,
// rotation, and zoom never deform the terrain contour.
void draw_terrain(SDL_Renderer* renderer, const lander::Terrain& terrain,
                  const lander::Camera& cam, const lander::State& state) {
    const double scale = cam.scale();
    const double target_r = std::max(1.0, std::hypot(state.x, state.y));
    double half_angle =
        (kWindowWidth / 2.0 + 256.0) / scale / target_r;
    half_angle = std::clamp(half_angle, 0.10, 0.75);

    const double theta_target = cam.angle() + lander::kPi / 2.0;
    const double s_target = lander::Terrain::arc_at_angle(theta_target);
    const double C = lander::Terrain::circumference();
    const double half_arc = half_angle * lander::Terrain::reference_radius();
    const int samples = 4096;
    const double u_start = s_target - half_arc;
    const double u_end = s_target + half_arc;

    std::vector<Vec2> outer;
    std::vector<Vec2> inner;
    for (int i = static_cast<int>(std::floor(u_start / C * samples));
         i <= static_cast<int>(std::ceil(u_end / C * samples)); ++i) {
        const double u = i * C / samples;
        const double theta = lander::Terrain::angle_at_arc(u);
        const double r = terrain.surface_radius_at_arc(u);
        outer.push_back(to_screen(std::cos(theta) * r,
                                  std::sin(theta) * r, cam));
        inner.push_back(to_screen(std::cos(theta) * (r - 0.75),
                                  std::sin(theta) * (r - 0.75), cam));
    }
    if (outer.size() < 2) {
        return;
    }

    std::vector<Vec2> polygon = outer;
    polygon.push_back(to_screen(0.0, 0.0, cam));
    fill_poly(renderer, polygon, make_color(66, 70, 82));

    for (size_t i = 0; i + 1 < outer.size(); ++i) {
        draw_thick_line(renderer, inner[i], inner[i + 1], 8.0,
                        make_color(58, 62, 74));
        draw_thick_line(renderer, outer[i], outer[i + 1], 2.5,
                        make_color(125, 130, 145));
    }

    const double tick_start =
        std::ceil(u_start / 10.0) * 10.0;
    for (double u = tick_start; u <= u_end; u += 10.0) {
        const double theta = lander::Terrain::angle_at_arc(u);
        const double r = terrain.surface_radius_at_arc(u);
        const Vec2 a = to_screen(std::cos(theta) * r,
                                 std::sin(theta) * r, cam);
        const Vec2 b = to_screen(std::cos(theta) * (r - 0.5),
                                 std::sin(theta) * (r - 0.5), cam);
        draw_thick_line(renderer, a, b, 1.0, make_color(44, 47, 58));
    }

    for (const lander::Pad& pad : terrain.pads()) {
        const double pad_center = pad.center_arc;
        const double angular_pad_width =
            pad.half_width / lander::Terrain::reference_radius();
        const double angular_distance =
            std::fabs(lander::Terrain::normalize_arc(
                          pad_center - s_target + 0.5 * C) -
                      0.5 * C);
        if (angular_distance / lander::Terrain::reference_radius() >
            half_angle + angular_pad_width + 0.10) {
            continue;
        }

        const int segments = 24;
        for (int i = 0; i < segments; ++i) {
            const double u0 =
                pad_center - pad.half_width +
                (2.0 * pad.half_width * i / segments);
            const double u1 =
                pad_center - pad.half_width +
                (2.0 * pad.half_width * (i + 1) / segments);
            const double t0 = lander::Terrain::angle_at_arc(u0);
            const double t1 = lander::Terrain::angle_at_arc(u1);
            const Vec2 a = to_screen(std::cos(t0) * pad.radius,
                                     std::sin(t0) * pad.radius, cam);
            const Vec2 b = to_screen(std::cos(t1) * pad.radius,
                                     std::sin(t1) * pad.radius, cam);
            draw_thick_line(renderer, a, b, 4.0, make_color(72, 210, 120));
            const Vec2 ah = to_screen(std::cos(t0) * (pad.radius + 0.25),
                                      std::sin(t0) * (pad.radius + 0.25),
                                      cam);
            const Vec2 bh = to_screen(std::cos(t1) * (pad.radius + 0.25),
                                      std::sin(t1) * (pad.radius + 0.25),
                                      cam);
            draw_thick_line(renderer, ah, bh, 1.5,
                            make_color(205, 255, 220), 220);
        }

        const double tc = lander::Terrain::angle_at_arc(pad_center);
        const Vec2 guide_top =
            to_screen(std::cos(tc) * (pad.radius + 14.0),
                      std::sin(tc) * (pad.radius + 14.0), cam);
        const Vec2 guide_bottom =
            to_screen(std::cos(tc) * (pad.radius + 0.25),
                      std::sin(tc) * (pad.radius + 0.25), cam);
        draw_thick_line(renderer, guide_top, guide_bottom, 1.0,
                        make_color(205, 255, 220), 70);
    }
}

// ----------------------------------------------------------------- lander

// The lander is drawn in its local frame (local +y = the thrust axis, with
// the leg feet at local y = 0 so the feet touch the surface exactly when
// the simulation's reference point reaches the terrain height) and each
// vertex is rotated by the state angle into world space. That keeps the
// on-screen orientation honest under the y-flip: positive angle =
// counter-clockwise on screen.
void draw_lander(SDL_Renderer* renderer, const lander::State& s,
                  double thrust_level, const lander::Camera& cam) {
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
        // Flame length flickers deterministically off the simulation tick
        // counter, so replays of the same state look identical. The flicker is
        // scaled by the throttle: full throttle gives the original flame
        // extent, low throttle gives a short puff.
        const double flick = 0.5 + 0.5 * std::sin(0.7 * s.ticks) +
                             0.3 * std::sin(1.3 * s.ticks + 1.0);
        const double full_len = 0.7 + 0.9 * flick;
        const double len = thrust_level * full_len;
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
// from the game seed and the tick the crash happened on.
void draw_debris(SDL_Renderer* renderer, const lander::State& s,
                  const lander::Camera& cam, std::uint64_t seed,
                  const lander::Terrain& terrain) {
    std::uint64_t state = seed ^ (s.ticks * 0x9E3779B97F4A7C15ULL);
    const double theta =
        s.x == 0.0 && s.y == 0.0 ? 0.0 : std::atan2(s.y, s.x);
    const double surface = terrain.surface_radius_at_angle(theta);
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
            up_x * (surface + radial) + right_x * tangential;
        const double wy =
            up_y * (surface + radial) + right_y * tangential;
        const Vec2 p = to_screen(wx, wy, cam);
        const int size = 2 + static_cast<int>(u3 * 3);
        fill_rect(renderer, static_cast<int>(p.x), static_cast<int>(p.y),
                  size, size, make_color(140, 90, 78));
    }
}

// --------------------------------------------------------------------- HUD

void draw_hud(SDL_Renderer* renderer, const lander::State& s,
              std::uint64_t seed, const lander::Terrain& terrain,
              double throttle, const lander::Camera& cam) {
    const Color panel(12, 14, 26);
    const Color white(228, 233, 244);
    const Color dim(130, 138, 156);
    const Color green(120, 240, 160);
    const Color red(255, 92, 80);

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    fill_rect(renderer, 8, 8, 330, 264, panel, 160);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    draw_text(renderer, "LUNAR LANDER", 18, 16, 3, white);
    char seed_line[48];
    std::snprintf(seed_line, sizeof seed_line, "SEED %llu",
                  static_cast<unsigned long long>(seed));
    draw_text(renderer, seed_line, 18, 46, 2, dim);

    // Altitude and velocity are shown in the local surface frame: altitude
    // is radial height above the terrain, radial velocity is negative while
    // descending, and tangential velocity is positive clockwise.
    const double alt_m =
        std::max(0.0, lander::altitude_at(terrain, s));
    const lander::LocalVelocity lv = lander::local_velocity(s);
    const int throttle_pct =
        static_cast<int>(std::lround(std::clamp(throttle, 0.0, 1.0) * 100.0));
    const std::string alt = "ALT   " + fmt1(alt_m) + " M";
    const std::string vel = "VEL T " + fmt1(lv.tangential) +
                            " R " + fmt1(lv.radial) + " M/S";
    const std::string ang = "ANG   " +
                            fmt1(lander::local_attitude_angle(s) * 180.0 /
                                 lander::kPi) +
                            " DEG";
    const std::string thr = "THR   " + std::to_string(throttle_pct) + "%";
    const std::string fuel = "FUEL  " + fmt1(s.fuel) + " / 100";
    draw_text(renderer, alt, 18, 74, 2, white);
    draw_text(renderer, vel, 18, 94, 2, white);
    draw_text(renderer, ang, 18, 114, 2, white);
    draw_text(renderer, thr, 18, 134, 2, white);

    // Commanded-throttle bar beside the THR readout. It shows the persistent
    // player throttle setting, not inferred thrust, flame size, or motion.
    constexpr int kThrBarX = 146;
    constexpr int kThrBarY = 138;
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

    draw_text(renderer, fuel, 18, 154, 2, white);

    // Fuel bar.
    fill_rect(renderer, 18, 178, 180, 12, make_color(40, 44, 60));
    const double frac =
        std::max(0.0, std::min(1.0, s.fuel / 100.0));
    const int bar_w = static_cast<int>(180 * frac);
    if (bar_w > 0) {
        fill_rect(renderer, 18, 178, bar_w, 12,
                  frac > 0.25 ? make_color(90, 200, 130) : red);
    }

    char score_line[32];
    std::snprintf(score_line, sizeof score_line, "SCORE %d", s.score);
    draw_text(renderer, score_line, 18, 202, 2, white);

    std::string status = "IN FLIGHT";
    Color status_color = white;
    if (s.landed) {
        status = "LANDED";
        status_color = green;
    } else if (s.crashed) {
        status = "CRASHED";
        status_color = red;
    }
    draw_text(renderer, status, 18, 228, 2, status_color);

    // Control help along the bottom.
    draw_text(renderer, "UP/W INCREASE   DOWN/S DECREASE   X CUTOFF", 18,
              kWindowHeight - 56, 1, dim);
    draw_text(renderer, "LEFT/RIGHT ROTATE   M CAMERA", 18,
              kWindowHeight - 40, 1, dim);
    draw_text(renderer, "WHEEL ZOOM   P PAUSE   R/N   ESC", 18,
              kWindowHeight - 24, 1, dim);

    // Camera indicator, top-right. In MANUAL the current zoom is shown.
    std::string cam_line =
        cam.mode() == lander::CameraMode::kAuto ? "CAM AUTO"
                                               : "CAM MANUAL " + fmt1(cam.zoom()) + "X";
    const Color cam_color =
        cam.mode() == lander::CameraMode::kAuto ? dim : green;
    const int cam_x =
        kWindowWidth - 8 - static_cast<int>(cam_line.size()) * 6;
    draw_text(renderer, cam_line, cam_x, 16, 1, cam_color);
}

void draw_overlay(SDL_Renderer* renderer, const lander::State& s,
                  bool paused) {
    if (!paused && !s.landed && !s.crashed) {
        return;
    }
    const Color white(232, 236, 246);
    const Color dim(150, 158, 176);
    const Color green(120, 240, 160);
    const Color red(255, 92, 80);

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    fill_rect(renderer, 0, 0, kWindowWidth, kWindowHeight,
              make_color(4, 5, 10), 120);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    auto center = [&](const std::string& text, int y, int scale, Color c) {
        draw_text(renderer, text, (kWindowWidth - text_width(text, scale)) / 2,
                  y, scale, c);
    };

    if (paused && !s.landed && !s.crashed) {
        center("PAUSED", kWindowHeight / 2 - 70, 6, white);
        center("PRESS P TO RESUME", kWindowHeight / 2 + 10, 2, dim);
        return;
    }

    if (s.landed) {
        center("LANDED!", kWindowHeight / 2 - 90, 6, green);
        center("SCORE " + std::to_string(s.score), kWindowHeight / 2, 3,
               white);
        center("R RETRY   N NEW SEED", kWindowHeight / 2 + 50, 2, dim);
    } else if (s.crashed) {
        center("CRASHED", kWindowHeight / 2 - 90, 6, red);
        center("SCORE 0", kWindowHeight / 2, 3, white);
        center("R RETRY   N NEW SEED", kWindowHeight / 2 + 50, 2, dim);
    }
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
           static_cast<std::uint64_t>(SDL_GetTicks());
}

void print_usage() {
    std::printf(
        "Usage: lander_gui [options]\n"
        "  --seed N         start with a deterministic seed (default: random)\n"
        "  --frames N       render exactly N frames, then exit\n"
        "  --fps N          cap the frame rate to N frames/second\n"
        "  --screenshot F   save the final frame to F as a PPM image\n"
        "  --orbit-demo     start in a terrain-clearing circular orbit\n"
        "                   (developer mode)\n"
        "  --help           show this message\n"
        "Controls: Up/W increase throttle, Down/S decrease throttle, X\n"
        "throttle cutoff, Left/Right/A/D rotate, M camera mode (Auto/Manual),\n"
        "mouse wheel zoom (Manual), R retry same seed, N new seed, P pause,\n"
        "Esc/Q quit.\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::uint64_t seed = 0;
    bool have_seed = false;
    int max_frames = 0;
    int fps_cap = 0;
    std::string screenshot_path;
    bool orbit_demo = false;

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
    // The camera owns the framing anchor: it follows the raw lander position
    // and rotates with the local surface frame, so the lander stays at the
    // configured screen position at every scale and screen up follows the
    // local outward direction.
    lander::CameraParams cam_params;
    cam_params.window_width = kWindowWidth;
    cam_params.window_height = kWindowHeight;
    lander::Camera cam(cam_params);
    std::vector<lander::Star> stars =
        lander::make_stars(seed, kWindowWidth, kWindowHeight);

    bool paused = false;
    bool running = true;
    double throttle = 0.0;
    int pending_wheel = 0;
    bool pending_cam_toggle = false;
    Uint64 prev_ticks = SDL_GetTicks();
    int frame = 0;

    auto start_mission = [&]() {
        sim.reset(seed);
        throttle = 0.0;
        paused = false;
        if (orbit_demo) {
            const double orbit_radius =
                sim.terrain().max_surface_radius() + 20.0;
            const double circular_speed =
                std::sqrt(sim.config().mu / orbit_radius);
            lander::State orbit{};
            orbit.x = 0.0;
            orbit.y = orbit_radius;
            orbit.vx = circular_speed;
            orbit.vy = 0.0;
            orbit.angle = 0.0;
            orbit.fuel = sim.config().fuel;
            sim.set_state(orbit);
        }
        const lander::State& state = sim.state();
        cam.snap(state.x, state.y);
    };
    start_mission();

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
                    case SDL_SCANCODE_R:
                        // Retry the same seed: the pad layout is identical.
                        start_mission();
                        break;
                    case SDL_SCANCODE_N: {
                        seed = random_seed();
                        stars = lander::make_stars(seed, kWindowWidth,
                                                   kWindowHeight);
                        start_mission();
                        break;
                    }
                    case SDL_SCANCODE_X:
                        // Immediate main-engine cutoff.
                        throttle = 0.0;
                        break;
                    case SDL_SCANCODE_M:
                        // Toggle between the automatic and the manual camera.
                        pending_cam_toggle = true;
                        break;
                    case SDL_SCANCODE_P:
                        paused = !paused;
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

        const Uint64 now = SDL_GetTicks();
        const double dt =
            std::min(0.25, static_cast<double>(now - prev_ticks) / 1000.0);
        prev_ticks = now;

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

        if (!paused) {
            sim.advance(dt, input);
        }

        const lander::State& s = sim.state();
        // The camera follows the raw lander position and chooses its scale
        // from the lander's radial altitude above the terrain directly below
        // it. It runs even while paused so the mode toggle and wheel stay
        // responsive; with the lander frozen the position target is constant,
        // so the camera simply holds.
        const double altitude =
            std::max(0.0, lander::altitude_at(sim.terrain(), s));
        cam.update(dt, s.x, s.y, altitude, pending_wheel,
                   pending_cam_toggle);
        pending_wheel = 0;
        pending_cam_toggle = false;

        // The flame is only present when the engine can actually burn, and
        // its size follows the throttle rather than a binary on/off state.
        const double thrust_level =
            (s.fuel > 0.0 && !s.landed && !s.crashed)
                ? std::clamp(throttle, 0.0, 1.0)
                : 0.0;

        draw_space(renderer, stars, cam);
        draw_terrain(renderer, sim.terrain(), cam, s);
        draw_lander(renderer, s, thrust_level, cam);
        if (s.crashed) {
            draw_debris(renderer, s, cam, seed, sim.terrain());
        }
        draw_hud(renderer, s, seed, sim.terrain(), throttle, cam);
        draw_overlay(renderer, s, paused);
        SDL_RenderPresent(renderer);

        ++frame;
        if (fps_cap > 0) {
            const Uint64 budget = 1000 / static_cast<Uint64>(fps_cap);
            const Uint64 spent = SDL_GetTicks() - now;
            if (spent < budget) {
                SDL_Delay(static_cast<Uint32>(budget - spent));
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
        "ticks=%llu state=%s score=%d seed=%llu\n",
        final_state.x, final_state.y, final_state.vx, final_state.vy,
        final_state.angle, final_state.fuel,
        static_cast<unsigned long long>(final_state.ticks),
        final_state.landed ? "landed" : (final_state.crashed ? "crashed"
                                                             : "flying"),
        final_state.score, static_cast<unsigned long long>(seed));

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
