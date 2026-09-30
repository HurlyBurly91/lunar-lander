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

// How much of one body's surface the renderer should fill. The decision is
// geometry-driven (M05-R3-23): a body that fits the viewport uses the full
// closed circle; a partial local patch is used only when its closure can be
// placed safely outside the visible viewport. It never depends on camera
// mode, reference-body index, or a call-site boolean.
enum class BodyRenderCoverage {
    kFull,
    kSurfacePatch,
};

inline bool point_outside_viewport(const Vec2& p, const Camera& cam,
                                   double margin = 0.0) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        return false;
    }
    const auto& q = cam.params();
    return p.x < -margin || p.x > q.window_width + margin ||
           p.y < -margin || p.y > q.window_height + margin;
}

inline bool segment_intersects_viewport(const Vec2& a, const Vec2& b,
                                        const Camera& cam,
                                        double margin = 0.0) {
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) ||
        !std::isfinite(b.y)) {
        return true;
    }
    const auto& q = cam.params();
    const double x0 = -margin;
    const double y0 = -margin;
    const double x1 = q.window_width + margin;
    const double y1 = q.window_height + margin;

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double p[4] = {-dx, dx, -dy, dy};
    const double qq[4] = {a.x - x0, x1 - a.x, a.y - y0, y1 - a.y};

    double t0 = 0.0;
    double t1 = 1.0;
    for (int i = 0; i < 4; ++i) {
        if (std::fabs(p[i]) < 1.0e-12) {
            if (qq[i] < 0.0) {
                return false;
            }
        } else {
            const double r = qq[i] / p[i];
            if (p[i] < 0.0) {
                if (r > t1) {
                    return false;
                }
                if (r > t0) {
                    t0 = r;
                }
            } else {
                if (r < t0) {
                    return false;
                }
                if (r < t1) {
                    t1 = r;
                }
            }
        }
    }
    return t0 <= t1;
}

// Choose an inward closure offset (screen pixels) for an open body-surface
// patch such that the inner closure arc, its chords, and the two radial side
// edges are all safely outside the visible viewport. Returns -1 when no safe
// patch closure exists, in which case the caller should draw the full body.
inline double patch_closure_offset(const std::vector<Vec2>& ring,
                                   const Vec2& centre, const Camera& cam) {
    constexpr double kClearance = 32.0;
    const size_t n = ring.size();
    if (n < 2 || !std::isfinite(centre.x) || !std::isfinite(centre.y)) {
        return -1.0;
    }

    const auto& q = cam.params();
    const double diag = std::hypot(q.window_width, q.window_height);
    double max_dist = 0.0;
    for (const auto& v : ring) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
            return -1.0;
        }
        max_dist = std::max(
            max_dist, std::hypot(v.x - centre.x, v.y - centre.y));
    }
    if (!std::isfinite(max_dist) || max_dist <= 1.0e-9) {
        return -1.0;
    }

    const double candidates[5] = {diag + 256.0, 2.0 * diag + 256.0,
                                  4.0 * diag + 256.0, 8.0 * diag + 256.0,
                                  max_dist};
    for (const double offset : candidates) {
        if (!std::isfinite(offset) || offset < 0.0) {
            continue;
        }

        bool ok = true;
        Vec2 first_inner{};
        Vec2 last_inner{};
        Vec2 prev_inner{};
        bool have_prev = false;
        for (size_t i = 0; i < n; ++i) {
            const Vec2& v = ring[i];
            const Vec2 to_centre{centre.x - v.x, centre.y - v.y};
            const double len = std::hypot(to_centre.x, to_centre.y);
            Vec2 inner;
            if (len < 1.0e-9) {
                inner = centre;
            } else {
                const double off = std::min(offset, len);
                inner = {v.x + to_centre.x / len * off,
                         v.y + to_centre.y / len * off};
            }
            if (!point_outside_viewport(inner, cam, kClearance)) {
                ok = false;
                break;
            }
            if (have_prev &&
                segment_intersects_viewport(prev_inner, inner, cam,
                                            kClearance)) {
                ok = false;
                break;
            }
            if (!have_prev) {
                first_inner = inner;
            }
            prev_inner = inner;
            last_inner = inner;
            have_prev = true;
        }
        if (!ok) {
            continue;
        }
        if (segment_intersects_viewport(ring.front(), first_inner, cam,
                                        kClearance) ||
            segment_intersects_viewport(ring.back(), last_inner, cam,
                                        kClearance)) {
            continue;
        }
        return offset;
    }
    return -1.0;
}

// Build the surface ring for one body. `ship` is the spacecraft world
// position (it centres the local-view arc). `coverage` selects the full
// circle versus a local arc window.
inline SurfaceRing body_surface_ring(const Body& body, const Vec2& bpos,
                                     const Camera& cam, const Vec2& ship,
                                     BodyRenderCoverage coverage,
                                     double body_rotation) {
    const Terrain& terrain = body.terrain;
    const double scale = cam.scale();
    const double C = terrain.circumference();
    const bool full_body = coverage == BodyRenderCoverage::kFull;
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

// The filled body polygon for a surface ring. A full body is the closed
// outer ring. A surface patch is closed by an inner concentric arc (not a
// fixed screen-bottom line); the offset is chosen by `patch_closure_offset`
// so the closure and its side edges are outside the viewport.
inline std::vector<Vec2> body_fill_polygon(const SurfaceRing& ring,
                                           const Vec2& centre,
                                           BodyRenderCoverage coverage,
                                           const Camera& cam) {
    const size_t n = ring.outer.size();
    if (n < 2) {
        return {};
    }
    if (coverage == BodyRenderCoverage::kFull) {
        return ring.outer;
    }

    const double offset = patch_closure_offset(ring.outer, centre, cam);
    if (offset < 0.0) {
        return ring.outer;
    }

    std::vector<Vec2> polygon = ring.outer;
    polygon.reserve(2 * n);
    for (size_t i = n; i-- > 0;) {
        const Vec2& v = ring.outer[i];
        const Vec2 to_centre{centre.x - v.x, centre.y - v.y};
        const double len = std::hypot(to_centre.x, to_centre.y);
        if (len < 1.0e-9) {
            polygon.push_back(centre);
        } else {
            const double off = std::min(offset, len);
            polygon.push_back({v.x + to_centre.x / len * off,
                               v.y + to_centre.y / len * off});
        }
    }
    return polygon;
}

// Decide, from geometry alone, whether a body should be drawn as the full
// closed circle or as a viewport-safe local surface patch (M05-R3-23..26).
inline BodyRenderCoverage body_render_coverage(const Body& body,
                                               const Vec2& bpos,
                                               const Camera& cam,
                                               const Vec2& ship,
                                               double body_rotation) {
    const auto& q = cam.params();
    const double projected = body.terrain.max_surface_radius() * cam.scale();
    if (std::isfinite(projected) &&
        projected + 16.0 <=
            std::min(q.window_width, q.window_height) * 0.5) {
        return BodyRenderCoverage::kFull;
    }

    const SurfaceRing ring = body_surface_ring(body, bpos, cam, ship,
                                               BodyRenderCoverage::kSurfacePatch,
                                               body_rotation);
    const Vec2 centre = to_screen_point(bpos.x, bpos.y, cam);
    if (patch_closure_offset(ring.outer, centre, cam) >= 0.0) {
        return BodyRenderCoverage::kSurfacePatch;
    }
    return BodyRenderCoverage::kFull;
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
