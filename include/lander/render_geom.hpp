#pragma once

// Pure presentation geometry shared by the SDL renderer (src/gui.cpp) and the
// headless geometry tests (tests/test_render_geom.cpp). Nothing here touches
// SDL: it only turns world-space body/terrain geometry into screen-space
// polygons and describes those polygons, so the same code can be exercised
// without a window.
//
// The two pieces that matter for the M05-R3 corrective round:
//   * body_surface_ring: one body's surface (and a thin inner edge) as a
//     contiguous, screen-space point ring. The local-view window is a fixed
//     arc centred on the ship; the system view is the full circle. The ring
//     is sampled at a bounded, zoom-independent count, so it stays finite and
//     well spaced at every zoom.
//   * thick_ring: a single closed annulus (a "thick curve") built from a
//     source ring by offsetting every vertex radially about a screen-space
//     centre. Because the offset is per-vertex and shared by both boundaries,
//     the result is one continuous polygon with no per-segment joints, so it
//     has no black seams and no per-segment overdraw at any zoom (M05-R3-18).

#include "lander/binary.hpp"
#include "lander/camera.hpp"

#include <cmath>
#include <vector>

namespace lander {

// World -> screen transform, identical to the renderer's. It depends only on
// the camera (centre, angle, scale, window size), so it is pure and shared.
inline Vec2 to_screen_point(double world_x, double world_y,
                            const Camera& cam) {
    const double dx = world_x - cam.center_x();
    const double dy = world_y - cam.center_y();
    const double c = std::cos(cam.angle());
    const double s = std::sin(cam.angle());
    const double local_x = dx * c + dy * s;
    const double local_y = -dx * s + dy * c;
    const double w = cam.params().window_width;
    const double h = cam.params().window_height;
    return {w / 2.0 + local_x * cam.scale(), h / 2.0 - local_y * cam.scale()};
}

// A body's rendered surface as two parallel screen-space rings: `outer` is
// the collision surface itself and `inner` sits a fixed 0.75 m inside it
// (the inner edge of the rendered rim). Both rings carry the same number of
// vertices, so they can be paired index-by-index.
struct SurfaceRing {
    std::vector<Vec2> outer;
    std::vector<Vec2> inner;
};

// Build the surface ring for one body. `ship` is the spacecraft world
// position (it centres the local-view arc). `full_body` selects the system
// view (whole circle) versus the local view (a fixed arc window).
inline SurfaceRing body_surface_ring(const Body& body, const Vec2& bpos,
                                     const Camera& cam, const Vec2& ship,
                                     bool full_body,
                                     double body_rotation) {
    const Terrain& terrain = body.terrain;
    const double scale = cam.scale();
    const double C = terrain.circumference();
    const double theta_ship =
        std::atan2(ship.y - bpos.y, ship.x - bpos.x);
    const double s_target = terrain.arc_at_angle(theta_ship - body_rotation);

    double half_angle = 0.0;
    double u_start = 0.0;
    double u_end = C;
    if (!full_body) {
        const double target_r =
            std::max(1.0, std::hypot(ship.x - bpos.x, ship.y - bpos.y));
        half_angle = (cam.params().window_width / 2.0 + 256.0) / scale /
                     target_r;
        half_angle = std::clamp(half_angle, 0.10, 0.75);
        const double half_arc = half_angle * terrain.reference_radius();
        u_start = s_target - half_arc;
        u_end = s_target + half_arc;
    }

    // Bounded, zoom-independent tessellation: the local window keeps a fixed
    // lattice; the full circle adapts to zoom but is clamped so the vertex
    // count (and hence the renderer's work) is bounded at every zoom.
    const int samples =
        full_body ? std::clamp(static_cast<int>(C * scale / 4.0), 64, 4096)
                  : 4096;

    SurfaceRing ring;
    ring.outer.reserve(64);
    ring.inner.reserve(64);
    auto push = [&](double u) {
        const double theta = terrain.angle_at_arc(u) + body_rotation;
        const double r = terrain.surface_radius_at_arc(u);
        ring.outer.push_back(to_screen_point(
            bpos.x + std::cos(theta) * r, bpos.y + std::sin(theta) * r, cam));
        ring.inner.push_back(to_screen_point(
            bpos.x + std::cos(theta) * (r - 0.75),
            bpos.y + std::sin(theta) * (r - 0.75), cam));
    };
    if (full_body) {
        for (int i = 0; i < samples; ++i) {
            push(i * C / samples);
        }
    } else {
        for (int i = static_cast<int>(std::floor(u_start / C * samples));
             i <= static_cast<int>(std::ceil(u_end / C * samples)); ++i) {
            push(i * C / samples);
        }
    }
    return ring;
}

// Build a closed "thick ring" (annulus) polygon from `ring`, offsetting every
// vertex radially about `centre` by `out_px` outward and `in_px` inward
// (screen pixels). The two boundaries are derived from the same vertices, so
// they share endpoints and the result is a single continuous polygon: filling
// it produces no inter-segment seams and no per-segment overdraw, at any
// zoom. Non-finite inputs yield an empty polygon (the fill then no-ops).
inline std::vector<Vec2> thick_ring(const std::vector<Vec2>& ring,
                                    const Vec2& centre, double out_px,
                                    double in_px) {
    const size_t n = ring.size();
    if (n < 2 || !std::isfinite(centre.x) || !std::isfinite(centre.y)) {
        return {};
    }
    std::vector<Vec2> outer(n);
    std::vector<Vec2> inner(n);
    for (size_t i = 0; i < n; ++i) {
        const Vec2& v = ring[i];
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
            return {};
        }
        const double dx = v.x - centre.x;
        const double dy = v.y - centre.y;
        const double len = std::hypot(dx, dy);
        if (len < 1.0e-9) {
            outer[i] = v;
            inner[i] = v;
            continue;
        }
        const double nx = dx / len;
        const double ny = dy / len;
        outer[i] = {v.x + nx * out_px, v.y + ny * out_px};
        inner[i] = {v.x - nx * in_px, v.y - ny * in_px};
    }

    std::vector<Vec2> polygon;
    polygon.reserve(2 * n);
    for (size_t i = 0; i < n; ++i) {
        polygon.push_back(outer[i]);
    }
    for (size_t i = n; i-- > 0;) {
        polygon.push_back(inner[i]);
    }
    return polygon;
}

}  // namespace lander
