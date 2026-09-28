// Lunar Lander: a playable game on SDL3 with jagged lunar terrain.
//
// This file is intentionally thin over lander::Simulation: all physics
// (fixed 1/120 s timestep, fuel, landing and crash rules, scoring,
// determinism) lives in include/lander/sim.hpp and lander::Terrain
// (include/lander/terrain.hpp) and is covered headlessly by
// tests/test_sim.cpp. The GUI only
//   (a) feeds keyboard input into the simulation,
//   (b) advances it with elapsed real time, and
//   (c) renders the world, lander, landing sites, and a HUD.
//
// World space: +x right, +y up, the lunar surface is the terrain height
// function, angle 0 = upright, positive angle = counter-clockwise.
// Screen space has a top-left origin, so every world point is y-flipped
// when drawn.

#include <SDL3/SDL.h>

#include "lander/camera.hpp"
#include "lander/sim.hpp"

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

constexpr double kStartX = 0.0;
constexpr double kStartY = 20.0;  // matches lander::State::y default

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
// can only rigidly move and scale the fixed world-space terrain, never change
// its shape. Coordinates stay floating point here; SDL quantizes to pixels.
Vec2 to_screen(double world_x, double world_y, const lander::Camera& cam) {
    return {kWindowWidth / 2.0 + (world_x - cam.x()) * cam.scale(),
            kWindowHeight / 2.0 - (world_y - cam.y()) * cam.scale()};
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
// Cosmetic background, generated deterministically from the game seed with a
// SplitMix64 stream that is deliberately separate from the simulation's own
// RNG (it must never influence physics).

struct Star {
    double x{};
    double y{};
    int size{};
    Uint8 bright{};
};

std::uint64_t splitmix64_next(std::uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

std::vector<Star> make_stars(std::uint64_t seed) {
    std::uint64_t state = seed * 0x9E3779B97F4A7C15ULL ^ 0xA5A4801A5A480193ULL;
    std::vector<Star> stars;
    stars.reserve(140);
    for (int i = 0; i < 140; ++i) {
        const double u1 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        const double u2 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        const double u3 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        Star s;
        s.x = u1 * kWindowWidth;
        s.y = u2 * kWindowHeight;
        s.size = u3 < 0.15 ? 2 : 1;
        s.bright = static_cast<Uint8>(90 + static_cast<int>(u3 * 140));
        stars.push_back(s);
    }
    return stars;
}

void draw_space(SDL_Renderer* renderer, const lander::Camera& cam,
                const std::vector<Star>& stars) {
    fill_rect(renderer, 0, 0, kWindowWidth, kWindowHeight,
              make_color(8, 10, 22));
    // Stars drift at 25% of the camera speed and wrap, so the sky extends
    // forever in every direction.
    for (const Star& star : stars) {
        double sx = std::fmod(star.x - cam.x() * cam.scale() * 0.25,
                              kWindowWidth);
        if (sx < 0) {
            sx += kWindowWidth;
        }
        double sy = std::fmod(star.y - cam.y() * cam.scale() * 0.25,
                              kWindowHeight);
        if (sy < 0) {
            sy += kWindowHeight;
        }
        SDL_Rect r{static_cast<int>(sx), static_cast<int>(sy), star.size,
                   star.size};
        fill_rect(renderer, r.x, r.y, r.w, r.h,
                  make_color(star.bright, star.bright,
                             std::min<Uint8>(255, star.bright + 20)));
    }
}

// ------------------------------------------------------------------ terrain

// Renders the actual collision surface: the terrain height function
// sampled on a fixed 1 m world-space lattice, closed into a polygon well
// below the window.
void draw_terrain(SDL_Renderer* renderer, const lander::Terrain& terrain,
                  const lander::Camera& cam) {
    const double scale = cam.scale();
    const double world_left = cam.x() - kWindowWidth / 2.0 / scale;
    const double world_right = cam.x() + kWindowWidth / 2.0 / scale;

    // The sample positions are anchored to the fixed 1 m world-space grid
    // (the same lattice the value noise is defined on), not to the moving
    // camera origin: the same world points are sampled every frame, so
    // camera motion and zoom only translate and uniformly scale the geometry
    // on screen, and the contour can never deform or wobble. The +-2 m
    // overhang keeps the polygon covering the window edges no matter where
    // the grid falls or how far in the camera is zoomed. World coordinates
    // stay floating point through to_screen; SDL does the final quantization
    // to pixels.
    std::vector<Vec2> surface;
    for (double x = std::floor(world_left - 2.0);
         x <= std::ceil(world_right + 2.0); x += 1.0) {
        surface.push_back(to_screen(x, terrain.height_at(x), cam));
    }
    if (surface.size() < 2) {
        return;
    }

    const Vec2 bottom_right{static_cast<double>(kWindowWidth) + 256.0,
                            static_cast<double>(kWindowHeight) + 512.0};
    const Vec2 bottom_left{-256.0, static_cast<double>(kWindowHeight) + 512.0};
    std::vector<Vec2> polygon = surface;
    polygon.push_back(bottom_right);
    polygon.push_back(bottom_left);

    // Regolith body, then a worn band just under the surface and a
    // lighter surface line on top.
    fill_poly(renderer, polygon, make_color(66, 70, 82));
    for (size_t i = 0; i + 1 < surface.size(); ++i) {
        draw_thick_line(renderer, {surface[i].x, surface[i].y + 9.0},
                        {surface[i + 1].x, surface[i + 1].y + 9.0}, 8.0,
                        make_color(58, 62, 74));
        draw_thick_line(renderer, surface[i], surface[i + 1], 2.5,
                        make_color(125, 130, 145));
    }

    // Distance ticks every 10 m, dropped from the surface.
    for (double xw = std::floor((world_left - 2.0) / 10.0) * 10.0;
         xw <= world_right + 2.0; xw += 10.0) {
        const Vec2 p = to_screen(xw, terrain.height_at(xw), cam);
        fill_rect(renderer, static_cast<int>(p.x), static_cast<int>(p.y), 1,
                  6, make_color(44, 47, 58));
    }

    // Landing sites: a bright slab on the flat section, a highlight edge,
    // and a faint vertical guide so the site is findable from altitude.
    for (const lander::Pad& pad : terrain.pads()) {
        const Vec2 left = to_screen(pad.x_min, pad.y, cam);
        const Vec2 right = to_screen(pad.x_max, pad.y, cam);
        if (left.x > kWindowWidth || right.x < 0) {
            continue;
        }
        const double pad_top_screen = to_screen(0.0, pad.y + 0.25, cam).y;
        fill_poly(renderer, {left, right,
                             {right.x, pad_top_screen},
                             {left.x, pad_top_screen}},
                  make_color(72, 210, 120));
        fill_rect(renderer, static_cast<int>(left.x),
                  static_cast<int>(pad_top_screen),
                  static_cast<int>(right.x - left.x), 2,
                  make_color(205, 255, 220));

        const double cx = (pad.x_min + pad.x_max) / 2.0;
        const Vec2 guide_top = to_screen(cx, pad.y + 14.0, cam);
        const Vec2 guide_bottom = to_screen(cx, pad.y + 0.25, cam);
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
                 bool thrusting, const lander::Camera& cam) {
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

    if (thrusting) {
        // Flame length flickers deterministically off the simulation tick
        // counter, so replays of the same state look identical.
        const double flick = 0.5 + 0.5 * std::sin(0.7 * s.ticks) +
                             0.3 * std::sin(1.3 * s.ticks + 1.0);
        const double len = 0.7 + 0.9 * flick;
        fill_poly(renderer, {local(-0.2, 0.18), local(0.0, 0.18 - len),
                             local(0.2, 0.18)},
                  make_color(255, 138, 38));
        fill_poly(renderer, {local(-0.1, 0.18),
                             local(0.0, 0.18 - len * 0.55), local(0.1, 0.18)},
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
    const double ground = terrain.height_at(s.x);
    for (int i = 0; i < 7; ++i) {
        const double u1 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        const double u2 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        const double u3 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        const double angle = u1 * 6.283185307179586;
        const double dist = 0.3 + 1.1 * u2;
        const Vec2 p = to_screen(s.x + std::cos(angle) * dist,
                                 ground + 0.05 + u3 * 0.25, cam);
        const int size = 2 + static_cast<int>(u3 * 3);
        fill_rect(renderer, static_cast<int>(p.x), static_cast<int>(p.y),
                  size, size, make_color(140, 90, 78));
    }
}

// --------------------------------------------------------------------- HUD

void draw_hud(SDL_Renderer* renderer, const lander::State& s,
              std::uint64_t seed, const lander::Terrain& terrain,
              const lander::Camera& cam) {
    const Color panel(12, 14, 26);
    const Color white(228, 233, 244);
    const Color dim(130, 138, 156);
    const Color green(120, 240, 160);
    const Color red(255, 92, 80);

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    fill_rect(renderer, 8, 8, 330, 232, panel, 160);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

    draw_text(renderer, "LUNAR LANDER", 18, 16, 3, white);
    char seed_line[48];
    std::snprintf(seed_line, sizeof seed_line, "SEED %llu",
                  static_cast<unsigned long long>(seed));
    draw_text(renderer, seed_line, 18, 46, 2, dim);

    // Altitude relative to the terrain directly below the lander.
    const double alt_m =
        std::max(0.0, s.y - terrain.height_at(s.x));
    const std::string alt = "ALT   " + fmt1(alt_m) + " M";
    const std::string vel =
        "VEL  " + fmt1(s.vx) + " " + fmt1(s.vy) + " M/S";
    const std::string ang = "ANG   " + fmt1(s.angle * 180.0 / 3.14159265358979) +
                            " DEG";
    const std::string fuel = "FUEL  " + fmt1(s.fuel) + " / 100";
    draw_text(renderer, alt, 18, 74, 2, white);
    draw_text(renderer, vel, 18, 94, 2, white);
    draw_text(renderer, ang, 18, 114, 2, white);
    draw_text(renderer, fuel, 18, 134, 2, white);

    // Fuel bar.
    fill_rect(renderer, 18, 158, 180, 12, make_color(40, 44, 60));
    const double frac =
        std::max(0.0, std::min(1.0, s.fuel / 100.0));
    const int bar_w = static_cast<int>(180 * frac);
    if (bar_w > 0) {
        fill_rect(renderer, 18, 158, bar_w, 12,
                  frac > 0.25 ? make_color(90, 200, 130) : red);
    }

    char score_line[32];
    std::snprintf(score_line, sizeof score_line, "SCORE %d", s.score);
    draw_text(renderer, score_line, 18, 182, 2, white);

    std::string status = "IN FLIGHT";
    Color status_color = white;
    if (s.landed) {
        status = "LANDED";
        status_color = green;
    } else if (s.crashed) {
        status = "CRASHED";
        status_color = red;
    }
    draw_text(renderer, status, 18, 208, 2, status_color);

    // Control help along the bottom.
    draw_text(renderer, "UP/SPACE THRUST   LEFT/RIGHT ROTATE", 18,
              kWindowHeight - 56, 1, dim);
    draw_text(renderer, "M CAMERA   WHEEL ZOOM   P PAUSE", 18,
              kWindowHeight - 40, 1, dim);
    draw_text(renderer, "R RETRY   N NEW SEED   ESC QUIT", 18,
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
        "  --help           show this message\n"
        "Controls: Up/Space/W thrust, Left/Right/A/D rotate, M camera\n"
        "mode (Auto/Manual), mouse wheel zoom (Manual), R retry same seed,\n"
        "N new seed, P pause, Esc/Q quit.\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::uint64_t seed = 0;
    bool have_seed = false;
    int max_frames = 0;
    int fps_cap = 0;
    std::string screenshot_path;

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
    sim.reset(seed);
    lander::Camera cam;
    // Frame the camera so the lander sits a fixed fraction down from the top
    // of the screen. The downward reach then scales with the zoom, so the
    // terrain below the lander stays in view at every scale (the lander is
    // never pushed off the top when zoomed in, and the ground is never lost
    // off the bottom when zoomed out).
    const double lander_top_fraction = cam.params().lander_top_fraction;
    const auto frame_target_y = [&](double lander_y, double scale) {
        return lander_y - (0.5 - lander_top_fraction) * (kWindowHeight / scale);
    };
    cam.snap(kStartX, frame_target_y(kStartY, cam.params().base_scale));
    std::vector<Star> stars = make_stars(seed);

    bool paused = false;
    bool running = true;
    int pending_wheel = 0;
    bool pending_cam_toggle = false;
    Uint64 prev_ticks = SDL_GetTicks();
    int frame = 0;

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
                        sim.reset(seed);
                        cam.snap(kStartX,
                                 frame_target_y(kStartY,
                                                cam.params().base_scale));
                        paused = false;
                        break;
                    case SDL_SCANCODE_N: {
                        seed = random_seed();
                        sim.reset(seed);
                        stars = make_stars(seed);
                        cam.snap(kStartX, kStartY);
                        paused = false;
                        break;
                    }
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
        lander::Input input;
        input.main_thrust =
            key(SDL_SCANCODE_UP) || key(SDL_SCANCODE_SPACE) ||
            key(SDL_SCANCODE_W);
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
        // The camera follows the lander and chooses its scale from the
        // lander's altitude above the terrain directly below it. It runs
        // even while paused so the mode toggle and wheel stay responsive;
        // with the lander frozen the position target is constant, so the
        // camera simply holds.
        const double altitude = s.y - sim.terrain().height_at(s.x);
        const double target_y = frame_target_y(s.y, cam.scale());
        cam.update(dt, s.x, target_y, altitude, pending_wheel,
                   pending_cam_toggle);
        pending_wheel = 0;
        pending_cam_toggle = false;

        const bool thrusting =
            input.main_thrust && s.fuel > 0.0 && !s.landed && !s.crashed;

        draw_space(renderer, cam, stars);
        draw_terrain(renderer, sim.terrain(), cam);
        draw_lander(renderer, s, thrusting, cam);
        if (s.crashed) {
            draw_debris(renderer, s, cam, seed, sim.terrain());
        }
        draw_hud(renderer, s, seed, sim.terrain(), cam);
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
