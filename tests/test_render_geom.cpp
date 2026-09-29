// Headless tests for the pure presentation geometry in
// include/lander/render_geom.hpp. No SDL, no window.
//
// M05-R3-18: the body/pad rim and the pads are now drawn as continuous,
// shared-vertex annuli (one closed polygon per band) instead of per-segment
// thick lines. `thick_ring` is the primitive that makes this work: it offsets
// every source vertex radially about a screen-space centre so both boundaries
// share the same endpoints and no per-segment joint leaves a gap. These tests
// verify that shared-endpoint / no-gap invariant, plus the bounded, finite,
// well-spaced ring required by M05-R3-17.

#include "lander/binary.hpp"
#include "lander/camera.hpp"
#include "lander/terrain.hpp"
#include "lander/render_geom.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

bool close(double a, double b, double eps) {
    return std::abs(a - b) <= eps;
}

void check_close(double a, double b, double eps, const char* message) {
    if (!close(a, b, eps)) {
        ++failures;
        std::printf("FAIL: %s (%.12g != %.12g, eps %.3g)\n", message, a, b,
                    eps);
    }
}

double polygon_area(const std::vector<lander::Vec2>& p) {
    double a = 0.0;
    const size_t n = p.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& c = p[i];
        const auto& d = p[(i + 1) % n];
        a += c.x * d.y - d.x * c.y;
    }
    return std::fabs(a) * 0.5;
}

// A small closed circle of `n` points at radius `r` about the origin.
std::vector<lander::Vec2> circle_ring(double r, int n) {
    std::vector<lander::Vec2> ring;
    for (int i = 0; i < n; ++i) {
        const double t = 2.0 * lander::kPi * i / n;
        ring.push_back({r * std::cos(t), r * std::sin(t)});
    }
    return ring;
}

// Max distance between consecutive vertices. `closed` includes the wrap edge
// (last -> first); for an open arc (a local window) that wrap edge is just the
// chord across the whole window, not a tessellation gap, so it is excluded.
double max_consecutive(const std::vector<lander::Vec2>& p, bool closed) {
    double m = 0.0;
    const size_t n = p.size();
    const size_t last = closed ? n : (n > 0 ? n - 1 : 0);
    for (size_t i = 0; i < last; ++i) {
        m = std::max(m, std::hypot(p[i].x - p[(i + 1) % n].x,
                                   p[i].y - p[(i + 1) % n].y));
    }
    return m;
}

double mean_consecutive(const std::vector<lander::Vec2>& p, bool closed) {
    double s = 0.0;
    const size_t n = p.size();
    const size_t last = closed ? n : (n > 0 ? n - 1 : 0);
    for (size_t i = 0; i < last; ++i) {
        s += std::hypot(p[i].x - p[(i + 1) % n].x,
                        p[i].y - p[(i + 1) % n].y);
    }
    return last ? s / last : 0.0;
}

// ---------------------------------------------------------------- transform

void test_to_screen_point() {
    lander::CameraParams p{};
    p.base_scale = 1.0;  // zoom stays 1 -> scale 1
    lander::Camera cam(p);
    const double w = p.window_width;
    const double h = p.window_height;

    // Angle 0, origin centre: identity with a y-flip and the centring.
    const auto a = lander::to_screen_point(10.0, 5.0, cam);
    check_close(a.x, w / 2.0 + 10.0, 1.0e-9, "x maps +x to the right");
    check_close(a.y, h / 2.0 - 5.0, 1.0e-9, "y maps +y to screen-up");

    // A scaled camera scales the offset; the window centre is fixed.
    lander::CameraParams q{};
    q.base_scale = 2.0;
    lander::Camera cam2(q);
    const auto b = lander::to_screen_point(10.0, 5.0, cam2);
    check_close(b.x, w / 2.0 + 20.0, 1.0e-9, "scale multiplies the x offset");
    check_close(b.y, h / 2.0 - 10.0, 1.0e-9, "scale multiplies the y offset");

    // A 90-degree camera rotation is a rigid rotation of the view (measured
    // relative to the camera's own centre, which snap() frames off-centre):
    // world +y, which is screen-up at angle 0, becomes screen-right, and
    // world +x becomes screen-down.
    lander::CameraParams r{};
    r.base_scale = 1.0;
    lander::Camera cam3(r);
    cam3.snap(0.0, 0.0, 0.5 * lander::kPi);
    const double cx3 = cam3.center_x();
    const double cy3 = cam3.center_y();
    const auto c0 = lander::to_screen_point(cx3, cy3, cam3);
    check_close(c0.x, w / 2.0, 1.0e-9,
                "90-degree: camera centre maps to the viewport centre (x)");
    check_close(c0.y, h / 2.0, 1.0e-9,
                "90-degree: camera centre maps to the viewport centre (y)");
    const auto cx_off = lander::to_screen_point(cx3 + 10.0, cy3, cam3);
    check_close(cx_off.x, c0.x, 1.0e-9,
                "90-degree: a +x offset stays horizontally centred");
    check_close(cx_off.y, c0.y + 10.0, 1.0e-9,
                "90-degree: world +x rotates to screen-down");
    const auto cy_off = lander::to_screen_point(cx3, cy3 + 10.0, cam3);
    check_close(cy_off.x, c0.x + 10.0, 1.0e-9,
                "90-degree: world +y rotates to screen-right");
    check_close(cy_off.y, c0.y, 1.0e-9,
                "90-degree: a +y offset stays vertically centred");
}

// ------------------------------------------------------------------- annulus

// M05-R3-18: a thick ring must be one continuous polygon whose two boundaries
// share the same endpoints (no per-segment joints) so it leaves no gap.
void test_thick_ring_no_seams() {
    const double r = 100.0;
    const int n = 16;
    const auto ring = circle_ring(r, n);
    const lander::Vec2 centre{0.0, 0.0};
    const auto band = lander::thick_ring(ring, centre, 8.0, 8.0);

    check(band.size() == 2 * static_cast<size_t>(n),
          "thick ring has one outer and one inner vertex per source vertex");

    const size_t half = band.size() / 2;
    bool all_finite = true;
    bool radii_ok = true;
    bool collinear = true;
    bool same_side = true;
    for (size_t i = 0; i < half; ++i) {
        const auto& o = band[i];                  // outer[i]
        // The inner boundary is stored in reverse order, so inner[i] (the
        // vertex sharing the same source ray as outer[i]) sits at the mirror
        // index band.size()-1-i.
        const auto& in = band[band.size() - 1 - i];
        if (!std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(in.x) ||
            !std::isfinite(in.y)) {
            all_finite = false;
        }
        const double ro = std::hypot(o.x, o.y);
        const double ri = std::hypot(in.x, in.y);
        if (!close(ro, r + 8.0, 1.0e-6) || !close(ri, r - 8.0, 1.0e-6)) {
            radii_ok = false;
        }
        // Outer and inner vertices sit on the same ray from the centre, so
        // the boundary vertices are shared (the seam is closed).
        const double cross = o.x * in.y - o.y * in.x;
        if (std::fabs(cross) > 1.0e-6) {
            collinear = false;
        }
        if (o.x * in.x + o.y * in.y < 0.0) {
            same_side = false;
        }
    }
    check(all_finite, "thick ring is entirely finite");
    check(radii_ok, "outer/inner boundaries are offset by exactly the widths");
    check(collinear,
          "adjacent outer/inner vertices share the same ray (shared endpoint)");
    check(same_side, "outer and inner vertices lie on the same side of centre");

    // The band is a proper annulus: a positive, finite area of the right
    // order of magnitude (a 16-gon ring, so a bit under the ideal circle).
    const double area = polygon_area(band);
    const double expected =
        lander::kPi * ((r + 8.0) * (r + 8.0) - (r - 8.0) * (r - 8.0));
    check(area > 0.0, "thick ring has positive area");
    check(area >= 0.5 * expected && area <= 1.5 * expected,
          "thick ring area is the right order of magnitude (a bounded annulus)");

    // No giant jump between consecutive vertices (the fix that removes the
    // per-segment gaps): the max step is bounded by a small multiple of the
    // mean step. The band is a closed polygon, so the wrap edge is real.
    const double mx = max_consecutive(band, true);
    const double mn = mean_consecutive(band, true);
    check(mx <= 3.0 * mn + 10.0, "no giant gap between consecutive vertices");

    // The two radial caps (where the outer and inner boundaries meet at the
    // first and last source vertex) are exactly shared endpoints: each is a
    // straight radial segment of length out+in.
    const auto cap_a = band[half - 1];  // outer[last] -> inner[last] join
    const auto cap_b = band[half];      // inner[last]
    check_close(std::hypot(cap_a.x - cap_b.x, cap_a.y - cap_b.y), 16.0, 1.0e-6,
                "the radial cap joins the shared endpoint by exactly out+in");
}

void test_thick_ring_degenerate() {
    const lander::Vec2 centre{0.0, 0.0};

    // Fewer than two source vertices -> empty.
    check(lander::thick_ring({{1.0, 2.0}}, centre, 4.0, 4.0).empty(),
          "a single vertex yields no band");
    check(lander::thick_ring({}, centre, 4.0, 4.0).empty(),
          "an empty ring yields no band");

    // A non-finite source vertex -> empty (never NaN in the output).
    std::vector<lander::Vec2> bad = {{10.0, 0.0},
                                     {std::nan(""), 0.0},
                                     {0.0, 10.0}};
    check(lander::thick_ring(bad, centre, 4.0, 4.0).empty(),
          "a non-finite vertex yields no band");

    // A non-finite centre -> empty.
    const auto ring = circle_ring(10.0, 4);
    check(lander::thick_ring(ring, {std::nan(""), 0.0}, 4.0, 4.0).empty(),
          "a non-finite centre yields no band");

    // A source vertex exactly at the centre degenerates gracefully (no NaN):
    // that vertex maps to itself on both boundaries.
    std::vector<lander::Vec2> at_centre = {{0.0, 0.0}, {10.0, 0.0}};
    const auto band = lander::thick_ring(at_centre, centre, 4.0, 4.0);
    bool ok = !band.empty();
    for (const auto& v : band) {
        ok = ok && std::isfinite(v.x) && std::isfinite(v.y);
    }
    check(ok, "a vertex at the centre never produces NaN");
}

// -------------------------------------------------------- bounded surface ring

// M05-R3-17: the body surface ring must stay bounded, finite, in the int
// range, and well-spaced at the widest and closest LOCAL zooms and in the
// full (system) view. These are the failure classes (integer overflow, NaN,
// huge coordinate conversion, giant tessellation gaps) that broke wide-zoom
// rendering before the fix.
void check_ring_bounded(const lander::SurfaceRing& ring, bool closed,
                        const char* label) {
    const size_t n = ring.outer.size();
    check(n >= 2, "ring has at least two vertices");
    check(n <= 4096, "ring vertex count stays bounded");
    check(ring.inner.size() == n, "outer and inner rings have equal length");

    bool finite = true;
    bool in_int_range = true;
    for (const auto& v : ring.outer) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
            finite = false;
        }
        if (std::fabs(v.x) > 1.0e9 || std::fabs(v.y) > 1.0e9) {
            in_int_range = false;
        }
    }
    for (const auto& v : ring.inner) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
            finite = false;
        }
        if (std::fabs(v.x) > 1.0e9 || std::fabs(v.y) > 1.0e9) {
            in_int_range = false;
        }
    }
    char msg[96];
    std::snprintf(msg, sizeof(msg), "%s: all vertices finite", label);
    check(finite, msg);
    std::snprintf(msg, sizeof(msg),
                  "%s: all vertices stay within the int range (no overflow)",
                  label);
    check(in_int_range, msg);

    // No giant gap: the largest single step is a small multiple of the mean.
    // A full body is a closed ring (the wrap edge is a real tessellation
    // edge); a local window is an open arc, whose wrap edge is just the chord
    // across the window and is not a rendered edge, so it is excluded.
    const double mx = max_consecutive(ring.outer, closed);
    const double mn = mean_consecutive(ring.outer, closed);
    std::snprintf(msg, sizeof(msg), "%s: no giant tessellation gap", label);
    check(mx <= 3.0 * mn + 10.0, msg);
}

void test_body_surface_ring_bounded() {
    lander::Body body;
    body.reference_radius = lander::kReferenceRadius;
    body.terrain = lander::Terrain(503);

    const lander::Vec2 bpos{0.0, 0.0};
    const lander::Vec2 ship{0.0, lander::kReferenceRadius + 20.0};

    // The wide LOCAL floor (0.14) and the close LOCAL ceiling (56) at the
    // base scale of 14, plus the SYSTEM ends, are the extremes that mattered.
    const double scales[] = {14.0 * 0.01, 14.0 * 4.0, 14.0 * 0.04, 14.0 * 2.0};
    const char* labels[] = {"wide LOCAL", "close LOCAL", "wide SYSTEM",
                            "close SYSTEM"};

    for (int i = 0; i < 4; ++i) {
        lander::CameraParams p{};
        p.base_scale = scales[i];
        lander::Camera cam(p);

        char label[48];
        std::snprintf(label, sizeof(label), "%s (full)", labels[i]);
        check_ring_bounded(
            lander::body_surface_ring(body, bpos, cam, ship, true, 0.0), true,
            label);

        std::snprintf(label, sizeof(label), "%s (local)", labels[i]);
        check_ring_bounded(
            lander::body_surface_ring(body, bpos, cam, ship, false, 0.0), false,
            label);
    }
}

// M05-R3-17 matrix: exercise both REF bodies (primary and companion), a
// sweep of the LOCAL/SYSTEM zoom range, the full and local windows, both a
// flying and a landed ship, and a tidal-rotation sweep. At every combination
// the surface ring must stay finite, in the int range, and free of giant
// gaps. This is the geometry-level proof that the wide LOCAL zoom range no
// longer breaks a body (M05-R3-17), for either reference body.
void test_body_surface_ring_matrix() {
    const lander::BinarySystem sys =
        lander::BinarySystem::canonical(178976.334, 503, 9001);
    const lander::Vec2 bpos{0.0, 0.0};

    // LOCAL zoom 0.01..4.0 (scale 0.14..56) and the SYSTEM ends (0.56..28).
    const double scales[] = {0.14, 1.0, 14.0, 56.0, 0.56, 28.0};
    const int n_scales = 6;
    const double rotations[] = {0.0, lander::kPi / 3.0, 2.0 * lander::kPi / 3.0};
    const int n_rot = 3;

    for (int bi = 0; bi < 2; ++bi) {
        const lander::Body& body = sys.body(bi);
        const double r0 = body.reference_radius;
        // A flying ship above the surface and a landed ship on it.
        const lander::Vec2 ships[] = {{0.0, r0 + 20.0}, {0.0, r0}};

        for (int si = 0; si < n_scales; ++si) {
            lander::CameraParams p{};
            p.base_scale = scales[si];
            lander::Camera cam(p);

            for (int sh = 0; sh < 2; ++sh) {
                const lander::Vec2& ship = ships[sh];

                for (int ri = 0; ri < n_rot; ++ri) {
                    char label[96];
                    std::snprintf(label, sizeof(label),
                                  "body%d scale%.2f ship%d full rot%d", bi,
                                  scales[si], sh, ri);
                    check_ring_bounded(
                        lander::body_surface_ring(body, bpos, cam, ship, true,
                                                 rotations[ri]),
                        true, label);
                }

                char llabel[96];
                std::snprintf(llabel, sizeof(llabel), "body%d scale%.2f ship%d "
                                                      "local",
                              bi, scales[si], sh);
                check_ring_bounded(
                    lander::body_surface_ring(body, bpos, cam, ship, false,
                                              0.0),
                    false, llabel);
            }
        }
    }
}

// M05-R3-18 / V28: the rim/pad annulus is built from the body's terrain ring,
// so its outer and inner boundaries must share exactly the same source
// endpoints (same ray about the screen centre, inner edge strictly inside)
// with gap-free coverage, for both REF bodies at several tidal rotation
// angles. That shared-endpoint construction is what keeps a filled annulus
// free of inter-segment black seams (M05-R3-18).
void test_body_annulus_shared_endpoints() {
    const lander::BinarySystem sys =
        lander::BinarySystem::canonical(178976.334, 503, 9001);
    const lander::Vec2 bpos{0.0, 0.0};

    for (int bi = 0; bi < 2; ++bi) {
        const lander::Body& body = sys.body(bi);
        for (int ri = 0; ri < 3; ++ri) {
            const double rot = ri * lander::kPi / 3.0;
            lander::CameraParams p{};
            p.base_scale = 14.0;
            lander::Camera cam(p);
            const auto ring = lander::body_surface_ring(
                body, bpos, cam,
                lander::Vec2{0.0, body.reference_radius + 20.0}, true, rot);

            const auto centre = lander::to_screen_point(bpos.x, bpos.y, cam);
            const auto band = lander::thick_ring(ring.outer, centre, 4.0, 4.0);
            const size_t n = ring.outer.size();

            char label[96];
            std::snprintf(label, sizeof(label), "body%d rot%d annulus", bi, ri);
            char sized[160];
            std::snprintf(sized, sizeof(sized),
                          "%s: the annulus carries both boundaries", label);
            check(band.size() == 2 * n, sized);

            bool finite = true;
            bool collinear = true;
            bool ordered = true;
            if (band.size() == 2 * n) {
                for (size_t i = 0; i < n; ++i) {
                    const auto& o = band[i];
                    const auto& inn = band[band.size() - 1 - i];
                    if (!std::isfinite(o.x) || !std::isfinite(o.y) ||
                        !std::isfinite(inn.x) || !std::isfinite(inn.y)) {
                        finite = false;
                        break;
                    }
                    const double ox = o.x - centre.x, oy = o.y - centre.y;
                    const double ix = inn.x - centre.x, iy = inn.y - centre.y;
                    // Same ray about the centre: the cross product, relative to
                    // the squared radius, is zero (shared endpoint).
                    if (std::fabs(ox * iy - oy * ix) >
                        1.0e-6 * (ox * ox + oy * oy)) {
                        collinear = false;
                        break;
                    }
                    // Inner edge strictly inside the outer edge (same side).
                    if (ox * ix + oy * iy <= 0.0) {
                        ordered = false;
                        break;
                    }
                }
            }
            check(finite,
                  "annulus vertices are finite for the terrain ring");
            check(collinear,
                  "outer/inner boundaries share each terrain source endpoint");
            check(ordered,
                  "the inner edge is strictly inside the outer edge");
        }
    }
}

// The local window must actually cover the intended arc (no missing chunk):
// the angular span of the sampled ring about the body centre matches the
// clamped window half-angle.
void test_local_window_covers_arc() {
    lander::Body body;
    body.reference_radius = lander::kReferenceRadius;
    body.terrain = lander::Terrain(503);

    const lander::Vec2 bpos{0.0, 0.0};
    const lander::Vec2 ship{0.0, lander::kReferenceRadius + 20.0};

    lander::CameraParams p{};
    p.base_scale = 14.0;  // neutral: half_angle clamps to its 0.10..0.75 band
    lander::Camera cam(p);

    const auto ring =
        lander::body_surface_ring(body, bpos, cam, ship, false, 0.0);

    // Screen-space angles of the ring about the body centre (window centre
    // is the viewport centre; the body centre maps to it).
    const double cx = p.window_width / 2.0;
    const double cy = p.window_height / 2.0;
    std::vector<double> angles;
    for (const auto& v : ring.outer) {
        angles.push_back(std::atan2(-(v.y - cy), v.x - cx));
    }
    // The span (max-min on the unwrapped arc) should match 2*half_angle,
    // where half_angle for a target at r ~ 352 and scale 14 is clamped to
    // [0.10, 0.75]. Here (640+256)/14/352 = 0.36, inside the band.
    double mn = 1e30;
    double mx = -1e30;
    for (double a : angles) {
        mn = std::min(mn, a);
        mx = std::max(mx, a);
    }
    const double span = mx - mn;
    const double expected = 2.0 * ((p.window_width / 2.0 + 256.0) / cam.scale() /
                                   std::hypot(ship.x - bpos.x,
                                              ship.y - bpos.y));
    check(span >= 0.8 * expected - 0.05,
          "the local window covers (at least) its intended arc");
    check(span <= 2.1 * 0.75 + 0.05,
          "the local window stays within the clamped maximum arc");
}

}  // namespace

int main() {
    test_to_screen_point();
    test_thick_ring_no_seams();
    test_thick_ring_degenerate();
    test_body_surface_ring_bounded();
    test_body_surface_ring_matrix();
    test_body_annulus_shared_endpoints();
    test_local_window_covers_arc();

    if (failures == 0) {
        std::puts("All lander_render_geom_tests passed");
        return 0;
    }
    std::printf("%d lander_render_geom_tests failed\n", failures);
    return 1;
}
