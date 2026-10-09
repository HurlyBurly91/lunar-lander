// M06 flight-computer tests: the shared ballistic predictor, the single
// maneuver node model, the planners, the bang-bang attitude controller, and
// the ordinary-Input node executor.
#include "lander/autopilot.hpp"
#include "lander/ballistic.hpp"
#include "lander/flight_computer.hpp"
#include "lander/sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

bool close(double a, double b, double eps) { return std::abs(a - b) <= eps; }

void check_close(double a, double b, double eps, const char* message) {
    if (!close(a, b, eps)) {
        ++failures;
        std::printf("FAIL: %s (%.12g != %.12g, eps %.3g)\n", message, a, b,
                    eps);
    }
}

void check_close_vec(lander::Vec2 a, lander::Vec2 b, double eps,
                     const char* message) {
    if (std::hypot(a.x - b.x, a.y - b.y) > eps) {
        ++failures;
        std::printf("FAIL: %s ((%.12g, %.12g) != (%.12g, %.12g), eps %.3g)\n",
                    message, a.x, a.y, b.x, b.y, eps);
    }
}

double vec_length(const lander::Vec2& v) { return std::hypot(v.x, v.y); }

// A flying ship placed relative to body i at ephemeris time 0.
lander::State state_relative(const lander::BinarySystem& bin, int i,
                             double arc, double altitude, double radial_v,
                             double tang_v, double angle_offset) {
    const lander::Body& body = bin.body(i);
    const lander::Vec2 pos = bin.position(i, 0.0);
    const lander::Vec2 vel = bin.velocity(i, 0.0);
    const double theta = body.terrain.angle_at_arc(arc);
    const double r = body.terrain.surface_radius_at_arc(arc) + altitude;
    const lander::Vec2 up{std::cos(theta), std::sin(theta)};
    const lander::Vec2 right{std::sin(theta), -std::cos(theta)};
    const double ox = r * up.x;
    const double oy = r * up.y;
    const double spin_x = -bin.omega() * oy;
    const double spin_y = bin.omega() * ox;
    lander::State s{};
    s.x = pos.x + r * up.x;
    s.y = pos.y + r * up.y;
    s.vx = vel.x + spin_x + radial_v * up.x + tang_v * right.x;
    s.vy = vel.y + spin_y + radial_v * up.y + tang_v * right.y;
    s.angle = theta - 0.5 * lander::kPi + angle_offset;
    s.fuel = 1000.0;
    s.landed = false;
    s.crashed = false;
    return s;
}

void test_snap_time_and_default_node() {
    const double dt = 1.0 / 120.0;
    check_close(lander::snap_time(0.0, dt), 0.0, 1e-12, "snap(0)");
    check_close(lander::snap_time(1.0, dt), 1.0, 1e-12, "snap(1 s)");
    check_close(lander::snap_time(1.004, dt), 1.0, 1e-9,
                "snap rounds to the nearest fixed step");

    const lander::ManeuverNode node =
        lander::default_node(10.0, 1, dt);
    check_close(node.time, 15.0, 1e-9, "default node is 5 s ahead");
    check(node.frame_body == 1, "default node keeps the requested frame body");
    check(node.dv_prograde == 0.0 && node.dv_radial == 0.0,
          "default node has zero delta-v");
}

void test_node_basis() {
    lander::Config cfg{};
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 503ULL, lander::companion_seed(503ULL));

    // Nominal frame: r = (100, 0) relative to body 0, v = (0, 10).
    {
        const lander::Vec2 bpos = bin.position(0, 0.0);
        const lander::Vec2 bvel = bin.velocity(0, 0.0);
        const lander::Vec2 p = bpos + lander::Vec2{100.0, 0.0};
        const lander::Vec2 v = bvel + lander::Vec2{0.0, 10.0};
        const auto basis = lander::compute_node_basis(bin, 0.0, 0, p, v);
        check_close_vec(basis.prograde, {0.0, 1.0}, 1e-12, "prograde");
        check_close_vec(basis.radial_out, {1.0, 0.0}, 1e-12, "radial out");
        check_close(basis.prograde.x * basis.radial_out.x +
                        basis.prograde.y * basis.radial_out.y,
                    0.0, 1e-12, "basis is orthogonal");
    }

    // Zero velocity: prograde falls back to the h >= 0 tangent of r.
    {
        const lander::Vec2 bpos = bin.position(0, 0.0);
        const lander::Vec2 bvel = bin.velocity(0, 0.0);
        const lander::Vec2 p = bpos + lander::Vec2{100.0, 0.0};
        const auto basis = lander::compute_node_basis(bin, 0.0, 0, p, bvel);
        check_close(vec_length(basis.prograde), 1.0, 1e-12,
                    "zero-velocity prograde is normalized");
        check_close(vec_length(basis.radial_out), 1.0, 1e-12,
                    "zero-velocity radial is normalized");
        check_close(basis.prograde.x * basis.radial_out.x +
                        basis.prograde.y * basis.radial_out.y,
                    0.0, 1e-12, "zero-velocity basis is orthogonal");
        check_close(basis.radial_out.x, 1.0, 1e-12,
                    "zero-velocity radial points outward");
    }

    // Fully degenerate: r and v are both zero.
    {
        const lander::Vec2 bpos = bin.position(0, 0.0);
        const lander::Vec2 bvel = bin.velocity(0, 0.0);
        const auto basis = lander::compute_node_basis(bin, 0.0, 0, bpos, bvel);
        check(std::isfinite(basis.prograde.x) && std::isfinite(basis.prograde.y) &&
                  std::isfinite(basis.radial_out.x) &&
                  std::isfinite(basis.radial_out.y),
              "degenerate basis is finite");
        check_close(vec_length(basis.prograde), 1.0, 1e-12,
                    "degenerate prograde is normalized");
        check_close(vec_length(basis.radial_out), 1.0, 1e-12,
                    "degenerate radial is normalized");
        check_close(basis.prograde.x * basis.radial_out.x +
                        basis.prograde.y * basis.radial_out.y,
                    0.0, 1e-12, "degenerate basis is orthogonal");
    }

    // Near-radial velocity: the radial candidate is tiny, but the fallback
    // must still produce a finite orthonormal frame; the exact radial-out
    // direction in this near-degenerate case is implementation-defined but
    // must remain perpendicular to prograde.
    {
        const lander::Vec2 bpos = bin.position(0, 0.0);
        const lander::Vec2 bvel = bin.velocity(0, 0.0);
        const lander::Vec2 p = bpos + lander::Vec2{100.0, 0.0};
        const lander::Vec2 v = bvel + lander::Vec2{10.0, 1.0e-6};
        const auto basis = lander::compute_node_basis(bin, 0.0, 0, p, v);
        check_close(vec_length(basis.prograde), 1.0, 1e-12,
                    "near-radial prograde is normalized");
        check_close(vec_length(basis.radial_out), 1.0, 1e-12,
                    "near-radial radial is normalized");
        check(std::isfinite(basis.radial_out.x) &&
                  std::isfinite(basis.radial_out.y),
              "near-radial basis is finite");
        check_close(basis.radial_out.x * basis.prograde.x +
                        basis.radial_out.y * basis.prograde.y,
                    0.0, 1e-6, "near-radial basis is orthogonal");
    }
}

void test_node_world_dv() {
    lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};
    lander::ManeuverNode node{};
    node.time = 42.0;
    node.frame_body = 0;
    node.dv_prograde = 2.0;
    node.dv_radial = 3.0;
    check_close_vec(lander::node_world_dv(node, basis), {3.0, 2.0}, 1e-12,
                    "world delta-v reconstruction");
}

void test_predictor_parity() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 503ULL, lander::companion_seed(503ULL));
    const lander::State start = state_relative(bin, 0, 100.0, 50.0, 1.0, 2.0,
                                               0.0);

    // A zero-input live simulation and the shared propagator agree step for
    // step over a short horizon.
    {
        lander::Simulation sim;
        sim.reset(503);
        sim.set_state(start);
        lander::BallisticState ball{
            {start.x, start.y}, {start.vx, start.vy}, 0.0};
        for (int i = 0; i < 120; ++i) {
            check(sim.step_once({}), "the parity probe stays in flight");
            ball = lander::step_ballistic(bin, ball, dt);
            if (i % 20 == 19) {
                check_close(sim.state().x, ball.p.x, 1e-9,
                            "parity x matches the shared propagator");
                check_close(sim.state().y, ball.p.y, 1e-9,
                            "parity y matches the shared propagator");
                check_close(sim.state().vx, ball.v.x, 1e-9,
                            "parity vx matches the shared propagator");
                check_close(sim.state().vy, ball.v.y, 1e-9,
                            "parity vy matches the shared propagator");
            }
        }
    }

    // Changing a body's mu changes the predicted path.
    {
        lander::Config cfg2{};
        cfg2.mu = cfg.mu * 1.1;
        const auto bin2 = lander::BinarySystem::canonical(
            cfg2.mu, 503ULL, lander::companion_seed(503ULL));
        lander::BallisticState a{
            {start.x, start.y}, {start.vx, start.vy}, 0.0};
        lander::BallisticState b{
            {start.x, start.y}, {start.vx, start.vy}, 0.0};
        a = lander::propagate_ballistic(bin, a, 240, dt);
        b = lander::propagate_ballistic(bin2, b, 240, dt);
        check(vec_length(a.p - b.p) > 1.0e-3,
              "changing a body's mu changes the predicted path");
    }
}

void test_node_impulse_prediction() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 503ULL, lander::companion_seed(503ULL));
    const lander::State start = state_relative(bin, 0, 100.0, 100.0, 0.5,
                                               12.0, 0.0);
    const double horizon = 2.0 * bin.period();

    const auto no_node = lander::predict_trajectory(
        bin, cfg, start, 0.0, 0, 1, std::nullopt, horizon, 100000);

    lander::ManeuverNode node{};
    node.time = lander::snap_time(10.0, dt);
    node.frame_body = 0;
    node.dv_prograde = 1.0;
    node.dv_radial = 0.5;

    // A very large `target_samples` forces stride 1, so the prediction keeps
    // every fixed-step sample and can be compared against the propagator.
    const auto with_node = lander::predict_trajectory(
        bin, cfg, start, 0.0, 0, 1, node, horizon, 100000);

    check(!no_node.pre.empty() && !with_node.pre.empty(),
          "both predictions produce a pre-node branch");
    check(with_node.pre.size() <= no_node.pre.size(),
          "the node prediction's pre branch ends at the node");
    for (size_t i = 0; i < with_node.pre.size(); ++i) {
        check_close_vec(with_node.pre[i], no_node.pre[i], 1e-9,
                        "the pre-node branch is unchanged until the node");
    }

    const int pre_steps =
        lander::ballistic_steps(0.0, node.time, dt);
    const lander::BallisticState pre_end =
        lander::propagate_ballistic(bin,
                                   {{start.x, start.y},
                                    {start.vx, start.vy},
                                    0.0},
                                   pre_steps, dt);
    check_close_vec(with_node.node_position, pre_end.p, 1e-9,
                    "the node position is the pre-burn endpoint");
    check(with_node.post.size() >= 2,
          "the post-node branch has at least two samples");
    check_close_vec(with_node.post.front(), pre_end.p, 1e-9,
                    "position is continuous across the node");

    const lander::Vec2 dv = lander::node_world_dv(node, with_node.basis);
    check_close_vec(with_node.dv_world, dv, 1e-12,
                    "the reported delta-v is the reconstructed world delta-v");
    check_close(with_node.total_dv, vec_length(dv), 1e-12,
                "the total delta-v magnitude is consistent");

    lander::BallisticState modified = pre_end;
    modified.v = modified.v + dv;
    const lander::BallisticState one_step =
        lander::step_ballistic(bin, modified, dt);
    check_close_vec(with_node.post[1], one_step.p, 1e-9,
                    "the post-node branch applies the ideal impulse");

    const lander::BallisticState plain_next =
        lander::step_ballistic(bin, pre_end, dt);
    check(vec_length(one_step.p - plain_next.p) > 1.0e-6,
          "the post-node branch differs from the no-node path");
}

// M06-R15 (V04): the PRE branch's final point, the POST branch's first point,
// and the node position all coincide at the node, i.e. the prediction joins
// PRE/POST at the node with no gap. This is the numerical basis for the
// on-scene node-edit overlay: the drawn PRE arc ends exactly where the NODE
// marker sits and the POST arc starts from that same point.
void test_pre_post_join() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 503ULL, lander::companion_seed(503ULL));
    const lander::State start = state_relative(bin, 0, 100.0, 100.0, 0.5,
                                               12.0, 0.0);
    const double horizon = 2.0 * bin.period();

    lander::ManeuverNode node{};
    node.time = lander::snap_time(10.0, dt);
    node.frame_body = 0;
    node.dv_prograde = 1.0;
    node.dv_radial = 0.5;

    const auto pred =
        lander::predict_trajectory(bin, cfg, start, 0.0, 0, 1, node, horizon,
                                   100000);

    check(!pred.pre.empty() && !pred.post.empty(),
          "the node prediction has both pre and post branches");
    check_close_vec(pred.pre.back(), pred.node_position, 1e-9,
                    "PRE ends at the node position");
    check_close_vec(pred.post.front(), pred.node_position, 1e-9,
                    "POST starts at the node position");
    check_close_vec(pred.pre.back(), pred.post.front(), 1e-9,
                    "PRE and POST join at the node (no gap)");
    check(node.time > 0.0 && node.time < horizon,
          "the node time is inside the horizon");
}

// M06-R16 (V01..V05): the single effective node-event epoch
// (prediction.node_time_effective = max(t0, snap(node->time, fixed_dt))) is
// the one anchor every node graphic may use. A FUTURE node reports the
// snapped scheduled epoch and its marker (node_position transformed at the
// effective epoch) sits exactly on the PRE/POST junction in every display
// frame. An OVERDUE node (node->time < t0) clamps to t0: the junction is the
// current ship position, the effective-epoch marker coincides with the live
// craft in every frame, and the stale raw-epoch transform demonstrably
// drifts away (the R15 defect). Delete + recreate produces a fresh future
// node with the future behavior again. Nothing here mutates the state, the
// node, or any subsystem.
void test_node_time_effective_overdue() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 503ULL, lander::companion_seed(503ULL));
    const lander::State start = state_relative(bin, 0, 100.0, 100.0, 0.5,
                                               12.0, 0.0);
    const double horizon = 2.0 * bin.period();
    const double t0 = 0.0;

    // The same inertial frame shift the GUI's node graphics apply (M06-R12):
    // identity for the world frame, otherwise wp - body(t_feat) + body(t_now).
    auto transform_at = [&](const lander::Vec2& wp, int frame_body,
                            double t_feat, double t_now) {
        if (frame_body < 0) return wp;
        return wp - bin.position(frame_body, t_feat) +
               bin.position(frame_body, t_now);
    };
    // F5..F8 display frames map to world (-1) and bodies 0..2.
    const int kFrames[] = {-1, 0, 1, 2};

    // 1) Fresh fixture: a FUTURE node.
    {
        lander::ManeuverNode node{};
        node.time = lander::snap_time(10.0, dt);
        node.frame_body = 0;
        node.dv_prograde = 1.0;
        node.dv_radial = 0.5;

        const auto pred = lander::predict_trajectory(
            bin, cfg, start, t0, 0, 1, node, horizon, 100000);

        check_close(pred.node_time_effective, node.time, 1e-9,
                    "future node: effective epoch == snapped scheduled epoch");
        check(pred.node_time_effective > t0,
              "future node: effective epoch is ahead of t0");
        check_close_vec(pred.pre.back(), pred.node_position, 1e-9,
                        "future node: PRE ends at the node");
        check_close_vec(pred.post.front(), pred.node_position, 1e-9,
                        "future node: POST starts at the node");
        for (int frame : kFrames) {
            const lander::Vec2 marker =
                transform_at(pred.node_position, frame,
                             pred.node_time_effective, t0);
            check_close_vec(
                marker,
                transform_at(pred.pre.back(), frame,
                             pred.node_time_effective, t0),
                1e-9, "future node: marker sits on the PRE/POST junction");
            check_close_vec(
                marker,
                transform_at(pred.post.front(), frame,
                             pred.node_time_effective, t0),
                1e-9, "future node: marker sits on the POST junction");
        }
        // The prediction never rewrites the scheduled node time.
        check_close(node.time, lander::snap_time(10.0, dt), 1e-12,
                    "future node: raw scheduled time is not overwritten");
    }

    // 2) Overdue node: the scheduled time is in the past (the R15 symptom:
    // the time advanced past the node).
    {
        lander::ManeuverNode node{};
        node.time = lander::snap_time(-10.0, dt);
        node.frame_body = 0;
        node.dv_prograde = 1.0;
        node.dv_radial = 0.5;

        const auto pred = lander::predict_trajectory(
            bin, cfg, start, t0, 0, 1, node, horizon, 100000);

        check_close(pred.node_time_effective, t0, 1e-12,
                    "overdue node: effective epoch clamps to t0");
        check(pred.node_time_effective > node.time,
              "overdue node: effective epoch is after the stale scheduled time");
        // With zero pre-steps the node position is the current ship position.
        check_close_vec(pred.node_position, lander::Vec2{start.x, start.y},
                        1e-9, "overdue node: junction is the current ship");
        check_close_vec(pred.pre.back(), pred.node_position, 1e-9,
                        "overdue node: PRE (single sample) is at the ship");
        check_close_vec(pred.post.front(), pred.node_position, 1e-9,
                        "overdue node: POST starts at the ship");
        for (int frame : kFrames) {
            const lander::Vec2 marker =
                transform_at(pred.node_position, frame,
                             pred.node_time_effective, t0);
            const lander::Vec2 ship =
                transform_at(lander::Vec2{start.x, start.y}, frame,
                             pred.node_time_effective, t0);
            check_close_vec(marker, ship, 1e-9,
                            "overdue node: marker sits on the live craft");
        }
        // The stale raw-epoch transform (the R15 defect) drifts from the
        // marker in at least one body frame: bodies move in 10 s, so the
        // shifted junction no longer lands where the marker does.
        bool any_drift = false;
        for (int frame : kFrames) {
            if (frame < 0) continue;  // world is the identity; no drift
            const lander::Vec2 stale =
                transform_at(pred.node_position, frame, node.time, t0);
            const lander::Vec2 marker =
                transform_at(pred.node_position, frame,
                             pred.node_time_effective, t0);
            if (vec_length(stale - marker) > 0.05) any_drift = true;
        }
        check(any_drift,
              "overdue node: raw-epoch transform drifts off the junction");
        check_close(node.time, lander::snap_time(-10.0, dt), 1e-12,
                    "overdue node: raw scheduled time is not overwritten");
    }

    // 3) Delete + recreate: a fresh default node is future again.
    {
        const lander::ManeuverNode fresh =
            lander::default_node(t0, 0, dt);
        check(fresh.time > t0, "recreated node is in the future");
        const auto pred = lander::predict_trajectory(
            bin, cfg, start, t0, 0, 1, fresh, horizon, 100000);
        check_close(pred.node_time_effective, fresh.time, 1e-9,
                    "recreated node: effective epoch == snapped scheduled");
        check_close_vec(pred.pre.back(), pred.node_position, 1e-9,
                        "recreated node: PRE ends at the node");
        check_close_vec(pred.post.front(), pred.node_position, 1e-9,
                        "recreated node: POST starts at the node");
    }

    // 4) No node at all: the effective epoch stays 0.0 (the no-node arc is
    // the whole branch; there is nothing to anchor).
    {
        const auto pred = lander::predict_trajectory(
            bin, cfg, start, t0, 0, 1, std::nullopt, horizon, 100000);
        check_close(pred.node_time_effective, 0.0, 1e-12,
                    "no node: effective epoch is 0.0");
    }

    // 5) No mutation: the ship state fed to the predictions is unchanged.
    {
        lander::State s2 = state_relative(bin, 0, 100.0, 100.0, 0.5, 12.0,
                                          0.0);
        (void)lander::predict_trajectory(bin, cfg, s2, t0, 0, 1,
                                         std::nullopt, horizon, 100000);
        check(s2 == start, "the ship state is unmutated by prediction");
    }
}

void test_trajectory_prediction() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 503ULL, lander::companion_seed(503ULL));
    const double horizon = 2.0 * bin.period();

    // A steep descent reports an impact using the body's future position and
    // tidal rotation.
    {
        const lander::State start =
            state_relative(bin, 0, 100.0, 10.0, -50.0, 0.0, 0.0);
        const auto pred = lander::predict_trajectory(
            bin, cfg, start, 0.0, 0, 1, std::nullopt, 5.0, 1024);
        check(pred.impact.valid, "a steep descent reports a terrain impact");
        check(pred.impact.body == 0, "the impact is on the primary");
        check(pred.impact.time > 0.0 && pred.impact.time < 5.0,
              "the impact time is inside the horizon");
        const lander::Vec2 bpos = bin.position(0, pred.impact.time);
        const double rho =
            std::hypot(pred.impact.position.x - bpos.x,
                       pred.impact.position.y - bpos.y);
        const double theta = std::atan2(pred.impact.position.y - bpos.y,
                                        pred.impact.position.x - bpos.x);
        const double surface = bin.body(0).terrain.surface_radius_at_arc(
            bin.body(0).terrain.arc_at_angle(theta -
                                             bin.body_rotation(
                                                 pred.impact.time)));
        check_close(rho, surface, 1.0,
                    "the impact sits on the future rotating surface");
    }

    // The closest-approach target is the actual moving surface point.
    {
        const lander::State start =
            state_relative(bin, 0, 0.0, 80.0, 0.0, 15.0, 0.0);
        const auto pred = lander::predict_trajectory(
            bin, cfg, start, 0.0, 0, 1, std::nullopt, horizon, 1024);
        check(pred.closest.valid, "closest approach is reported");
        const auto pad = bin.surface_point(
            1, bin.body(1).terrain.angle_at_arc(0.0),
            bin.body(1).terrain.surface_radius_at_arc(0.0),
            pred.closest.time);
        check_close_vec(pred.closest.target_position, pad.position, 1e-9,
                        "the closest-approach target tracks the moving pad");
        const double d = std::hypot(pred.closest.position.x - pad.position.x,
                                    pred.closest.position.y - pad.position.y);
        check_close(pred.closest.distance, d, 1e-9,
                    "the reported closest-approach distance is consistent");
    }

    // A visibly elliptical path produces numeric PE / AP extrema (or none,
    // but never NaNs) over the post-node branch.
    {
        const double r0 = bin.body(0).terrain.surface_radius_at_arc(0.0) + 200.0;
        const double v_circ = std::sqrt(bin.body(0).mu / r0);
        const lander::State start =
            state_relative(bin, 0, 0.0, 200.0, 0.0, 0.8 * v_circ, 0.0);
        lander::ManeuverNode node{};
        node.time = lander::snap_time(1.0, dt);
        node.frame_body = 0;
        const auto pred = lander::predict_trajectory(
            bin, cfg, start, 0.0, 0, 1, node, horizon, 1024);
        check(std::isfinite(pred.peri.distance) && std::isfinite(pred.apo.distance),
              "PE / AP readouts are finite");
        if (pred.peri.valid && pred.apo.valid) {
            check(pred.peri.distance < pred.apo.distance,
                  "PE is closer than AP when both exist");
        }
    }

    // Without a node there is no post-node branch and no PE / AP.
    {
        const lander::State start =
            state_relative(bin, 0, 0.0, 100.0, 0.0, 10.0, 0.0);
        const auto pred = lander::predict_trajectory(
            bin, cfg, start, 0.0, 0, 1, std::nullopt, horizon, 1024);
        check(pred.post.empty(), "no node means no post-node branch");
        check(!pred.peri.valid && !pred.apo.valid,
              "no node means no PE / AP readout");
    }
}

void test_planners() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 503ULL, lander::companion_seed(503ULL));

    // CIRCULARIZE removes the body-relative radial velocity at the node.
    {
        const lander::State start =
            state_relative(bin, 0, 0.0, 200.0, 3.0, 8.0, 0.0);
        lander::ManeuverNode at_now{};
        at_now.time = lander::snap_time(0.0, dt);
        at_now.frame_body = 0;
        const auto node = lander::plan_circularize(
            bin, cfg, start, 0.0, 0, at_now);
        check(node.has_value(), "CIRCULARIZE produced a node");
        if (node) {
            const int steps =
                lander::ballistic_steps(0.0, node->time, dt);
            const lander::BallisticState pre = lander::propagate_ballistic(
                bin, {{start.x, start.y}, {start.vx, start.vy}, 0.0}, steps,
                dt);
            const lander::Vec2 bpos = bin.position(0, node->time);
            const lander::Vec2 bvel = bin.velocity(0, node->time);
            const lander::Vec2 r = pre.p - bpos;
            const lander::Vec2 r_hat =
                r * (1.0 / std::hypot(r.x, r.y));
            const lander::Vec2 v_rel_pre = pre.v - bvel;
            const double radial_before =
                v_rel_pre.x * r_hat.x + v_rel_pre.y * r_hat.y;

            const auto basis =
                lander::compute_node_basis(bin, node->time, 0, pre.p, pre.v);
            const lander::Vec2 dv = lander::node_world_dv(*node, basis);
            const lander::Vec2 v_rel_after = (pre.v + dv) - bvel;
            const double radial_after =
                v_rel_after.x * r_hat.x + v_rel_after.y * r_hat.y;
            check(std::abs(radial_before) > 1.0,
                  "the CIRCULARIZE probe starts with radial velocity");
            check_close(radial_after, 0.0, 1.0e-6,
                        "CIRCULARIZE removes the radial velocity component");
        }
    }

    // TRANSFER TO OTHER MOON reuses the same solver as the T x3 initializer:
    // from the same state at the same time, both produce the same velocity.
    {
        const lander::State start =
            state_relative(bin, 0, 0.0, 100.0, 0.0, 0.0, 0.0);

        lander::Simulation sim;
        sim.reset(503);
        sim.set_state(start);
        const lander::State before = sim.state();
        const bool solved = sim.transfer();

        lander::ManeuverNode at_now{};
        at_now.time = lander::snap_time(0.0, dt);
        at_now.frame_body = 0;
        const auto planned =
            lander::plan_transfer(bin, cfg, start, 0.0, 0, at_now);

        check(solved == planned.has_value(),
              "the planner and T x3 agree on whether a solution exists");
        if (solved && planned) {
            const int steps =
                lander::ballistic_steps(0.0, at_now.time, dt);
            const lander::BallisticState pre = lander::propagate_ballistic(
                bin, {{start.x, start.y}, {start.vx, start.vy}, 0.0}, steps,
                dt);
            const auto basis =
                lander::compute_node_basis(bin, at_now.time, 0, pre.p, pre.v);
            const lander::Vec2 dv = lander::node_world_dv(*planned, basis);
            const lander::Vec2 planned_v = pre.v + dv;
            const lander::State& after = sim.state();
            check_close(planned_v.x, after.vx, 1.0e-6,
                        "the planner's transfer velocity matches T x3 (x)");
            check_close(planned_v.y, after.vy, 1.0e-6,
                        "the planner's transfer velocity matches T x3 (y)");
            check_close(before.x, after.x, 1e-12,
                        "T x3 still leaves the position untouched");
        }
    }

    // MATCH TARGET VELOCITY makes the ideal post-node world velocity equal to
    // the moving destination pad's world velocity at the node time.
    {
        const lander::State start =
            state_relative(bin, 0, 0.0, 150.0, 0.0, 10.0, 0.0);
        const auto node = lander::plan_match_target(
            bin, cfg, start, 0.0, 0, 1, std::nullopt);
        check(node.has_value(), "MATCH TARGET VELOCITY produced a node");
        if (node) {
            const int steps =
                lander::ballistic_steps(0.0, node->time, dt);
            const lander::BallisticState pre = lander::propagate_ballistic(
                bin, {{start.x, start.y}, {start.vx, start.vy}, 0.0}, steps,
                dt);
            const auto basis = lander::compute_node_basis(
                bin, node->time, node->frame_body, pre.p, pre.v);
            const lander::Vec2 dv = lander::node_world_dv(*node, basis);
            const auto pad = bin.surface_point(
                1, bin.body(1).terrain.angle_at_arc(0.0),
                bin.body(1).terrain.surface_radius_at_arc(0.0), node->time);
            check_close_vec(pre.v + dv, pad.velocity, 1.0e-9,
                            "the post-node velocity matches the moving pad");
        }
    }
}

void test_attitude_controller() {
    lander::Config cfg{};
    lander::State s{};
    s.fuel = 100.0;

    // Already pointing along the target direction: no rotation.
    {
        s.angle = 0.0;
        s.omega = 0.0;
        const auto in = lander::attitude_input(s, cfg,
                                               std::optional<lander::Vec2>(
                                                   {0.0, 1.0}),
                                               false, false);
        check(!in.rotate_left && !in.rotate_right,
              "no rotation when aligned with the target");
    }

    // Shortest-path rotation to the left and to the right.
    {
        s.angle = 0.0;
        s.omega = 0.0;
        const auto in_left = lander::attitude_input(
            s, cfg, std::optional<lander::Vec2>(
                       {1.0, 0.0}),
            false, false);
        check(in_left.rotate_left && !in_left.rotate_right,
              "a +x target rotates the shortest way (left from 0)");

        const auto in_right = lander::attitude_input(
            s, cfg, std::optional<lander::Vec2>(
                        {-1.0, 0.0}),
            false, false);
        check(in_right.rotate_right && !in_right.rotate_left,
              "a -x target rotates the shortest way (right from 0)");
    }

    // Across the 0 / 2pi wrap, the controller takes the short side.
    {
        s.angle = 2.0 * lander::kPi - 0.1;
        s.omega = 0.0;
        const auto in = lander::attitude_input(
            s, cfg, std::optional<lander::Vec2>(
                       {0.0, 1.0}),
            false, false);
        check(in.rotate_right && !in.rotate_left,
              "the wraparound target uses the shortest angular path");
    }

    // A high approach rate brakes before overshooting.
    {
        s.angle = 0.0;
        s.omega = 2.0;
        const auto in = lander::attitude_input(
            s, cfg, std::optional<lander::Vec2>(
                       {-1.0, 0.0}),
            false, false);
        check(in.rotate_left && !in.rotate_right,
              "a high rate on the wrong side brakes before overshoot");
    }

    // Undefined targets fail safely to no rotation.
    {
        s.angle = 0.0;
        s.omega = 0.5;
        const auto in = lander::attitude_input(s, cfg, std::nullopt, false,
                                               false);
        check(!in.rotate_left && !in.rotate_right,
              "an undefined target direction produces no rotation");
        const auto in_zero = lander::attitude_input(
            s, cfg, std::optional<lander::Vec2>(
                       {0.0, 0.0}),
            false, false);
        check(!in_zero.rotate_left && !in_zero.rotate_right,
              "a zero target direction produces no rotation");
    }

    // Manual rotation overrides the hold for that step.
    {
        s.angle = 0.0;
        s.omega = 0.0;
        const auto in = lander::attitude_input(
            s, cfg, std::optional<lander::Vec2>(
                       {-1.0, 0.0}),
            true, false);
        check(in.rotate_left && !in.rotate_right,
              "manual rotation overrides the attitude hold");
        check(in.main_throttle == 0.0 && !in.reaction_wheels,
              "the attitude controller never touches thrust or reaction "
              "wheels");
    }

    // The mode resolver produces the expected directions (and fails safely on
    // degenerate frames).
    {
        lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};
        const lander::Vec2 ship{10.0, 20.0};
        const lander::Vec2 target{30.0, 20.0};
        check_close_vec(
            *lander::attitude_target_direction(
                lander::AttitudeMode::Prograde, basis, ship, target, {}),
            {0.0, 1.0}, 1e-12, "PROGRADE direction");
        check_close_vec(
            *lander::attitude_target_direction(
                lander::AttitudeMode::Retrograde, basis, ship, target, {}),
            {0.0, -1.0}, 1e-12, "RETROGRADE direction");
        check_close_vec(
            *lander::attitude_target_direction(
                lander::AttitudeMode::RadialOut, basis, ship, target, {}),
            {1.0, 0.0}, 1e-12, "RADIAL OUT direction");
        check_close_vec(
            *lander::attitude_target_direction(
                lander::AttitudeMode::RadialIn, basis, ship, target, {}),
            {-1.0, 0.0}, 1e-12, "RADIAL IN direction");
        check_close_vec(
            *lander::attitude_target_direction(
                lander::AttitudeMode::Target, basis, ship, target, {}),
            {1.0, 0.0}, 1e-12, "TARGET direction");
        check_close_vec(
            *lander::attitude_target_direction(
                lander::AttitudeMode::AntiTarget, basis, ship, target, {}),
            {-1.0, 0.0}, 1e-12, "ANTI-TARGET direction");
        check_close_vec(
            *lander::attitude_target_direction(
                lander::AttitudeMode::Maneuver, basis, ship, target,
                {3.0, 4.0}),
            {0.6, 0.8}, 1e-12, "MANEUVER direction");
        check(lander::attitude_target_direction(
                   lander::AttitudeMode::Off, basis, ship, target, {})
                  .has_value() == false,
              "OFF has no target direction");
        check(lander::attitude_target_direction(
                   lander::AttitudeMode::Target, basis, ship, ship, {})
                  .has_value() == false,
              "TARGET on top of the target fails safely");
    }
}

void test_node_executor() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;

    // A 4 m/s node burns for exactly 1 s (120 fixed steps) at full throttle
    // and then completes with no latent output.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.frame_body = 0;
        node.dv_prograde = 4.0;
        node.dv_radial = 0.0;
        const lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

        lander::NodeExecutor exec;
        exec.arm(node, basis, node.time - 0.5, cfg);
        check(exec.state() == lander::ExecutorState::Align,
              "arming at ignition starts in ALIGN");
        check_close(exec.burn_time(), 1.0, 1e-12, "burn time is |dv| / a");
        check_close(exec.ignite_time(), node.time - 0.5, 1e-12,
                    "ignition is half a burn before the node");

        lander::State before{};
        before.fuel = 1000.0;
        double now = node.time - 0.5;
        int burn_steps = 0;
        for (int i = 0; i < 200 && exec.active(); ++i) {
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            lander::State after = before;
            after.fuel = std::max(
                0.0, before.fuel - (input.main_throttle > 0.0
                                       ? cfg.fuel_burn * dt
                                       : 0.0));
            now += dt;
            if (input.main_throttle > 0.0) {
                ++burn_steps;
            }
            exec.after_step(before, after, input, now, cfg);
            before = after;
        }
        check(exec.state() == lander::ExecutorState::Complete,
              "a fully fuelled node completes");
        check(burn_steps == 120, "the burn lasts exactly 120 fixed steps");
        check_close(vec_length(exec.dv_remaining()), 0.0, 1.0e-9,
                    "the tracked remaining delta-v reaches zero");
        const lander::Input idle =
            exec.make_input(before, now, cfg, false, false);
        check(!idle.rotate_left && !idle.rotate_right &&
                  idle.main_throttle == 0.0,
              "a completed executor leaves no latent output");
    }

    // Arming well before ignition begins aligning immediately (ALIGN,
    // throttle 0). Once aligned it holds in WAIT (throttle 0) until the
    // ignition time, then ignites into BURN.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_prograde = 4.0;  // vgo (0,4) -> aligned nose angle 0
        lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

        lander::NodeExecutor exec;
        exec.arm(node, basis, node.time - 5.0, cfg);
        check(exec.state() == lander::ExecutorState::Align,
              "arming before ignition begins aligning (ALIGN)");

        lander::State before{};
        before.fuel = 1000.0;
        before.angle = 0.0;  // already aligned with the VGO
        before.omega = 0.0;

        // First step: aligned while still pre-ignition -> hold in WAIT.
        double now = node.time - 5.0;
        {
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            check(input.main_throttle == 0.0, "no thrust before ignition");
            lander::State after = before;
            now += dt;
            exec.after_step(before, after, input, now, cfg);
        }
        check(exec.state() == lander::ExecutorState::Wait,
              "an aligned pre-ignition node holds in WAIT");

        // Holds with no thrust until the ignition time, then ignites.
        int thrust_steps = 0;
        for (int i = 0; i < 1200 && exec.active(); ++i) {
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            lander::State after = before;
            after.fuel = std::max(
                0.0, before.fuel - (input.main_throttle > 0.0
                                       ? cfg.fuel_burn * dt
                                       : 0.0));
            now += dt;
            if (input.main_throttle > 0.0) {
                ++thrust_steps;
            }
            exec.after_step(before, after, input, now, cfg);
            before = after;
            if (exec.state() == lander::ExecutorState::Burn) {
                break;
            }
        }
        check(exec.state() == lander::ExecutorState::Burn,
              "a held node ignites into BURN at the ignition time");
        check(thrust_steps == 0, "no thrust is emitted while held in WAIT");
    }

    // A misaligned node never forces a burn at the node time: it stays in
    // ALIGN (attitude only, no thrust) until the attitude actually lines up
    // with the VGO direction, then burns the finite window to completion.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_radial = 4.0;  // world +x VGO -> aligned nose angle is -pi/2
        lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

        lander::NodeExecutor exec;
        exec.arm(node, basis, node.time - 0.5, cfg);
        check(exec.state() == lander::ExecutorState::Align,
              "arming at ignition starts a node in ALIGN");

        lander::State before{};
        before.fuel = 1000.0;
        before.angle = 0.0;  // 90 deg off the VGO direction
        before.omega = 0.0;

        // Step well past the node time while misaligned: attitude only.
        double now = node.time - 0.5;
        int burn_steps = 0;
        for (int i = 0; i < 480; ++i) {
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            lander::State after = before;
            after.fuel = std::max(
                0.0, before.fuel - (input.main_throttle > 0.0
                                       ? cfg.fuel_burn * dt
                                       : 0.0));
            now += dt;
            if (input.main_throttle > 0.0) {
                ++burn_steps;
            }
            exec.after_step(before, after, input, now, cfg);
            before = after;
            if (!exec.active()) {
                break;
            }
        }
        check(burn_steps == 0,
              "a misaligned node emits no thrust past the node time");
        check(exec.state() == lander::ExecutorState::Align,
              "a misaligned node stays in ALIGN past the node time");
        check_close(vec_length(exec.dv_remaining()), 4.0, 1.0e-9,
                    "no delta-v is delivered while misaligned");

        // Once the attitude is actually aligned, the burn starts and the node
        // completes with the full delta-v delivered.
        before.angle = -0.5 * lander::kPi;
        before.omega = 0.0;
        int aligned_burn_steps = 0;
        for (int i = 0; i < 480 && exec.active(); ++i) {
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            lander::State after = before;
            after.fuel = std::max(
                0.0, before.fuel - (input.main_throttle > 0.0
                                       ? cfg.fuel_burn * dt
                                       : 0.0));
            now += dt;
            if (input.main_throttle > 0.0) {
                ++aligned_burn_steps;
            }
            exec.after_step(before, after, input, now, cfg);
            before = after;
        }
        check(exec.state() == lander::ExecutorState::Complete,
              "the node completes once aligned and burning");
        check(aligned_burn_steps == 120,
              "the aligned burn lasts exactly 120 fixed steps");
        check_close(vec_length(exec.dv_remaining()), 0.0, 1.0e-9,
                    "the aligned burn delivers the full node delta-v");
    }

    // Abort and fuel exhaustion leave no latent output.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_prograde = 4.0;
        lander::NodeExecutor exec;
        exec.arm(node, {{0.0, 1.0}, {1.0, 0.0}}, node.time - 0.5, cfg);
        exec.abort();
        check(exec.state() == lander::ExecutorState::Aborted,
              "abort moves an active executor to ABORTED");
        lander::State s{};
        s.fuel = 100.0;
        const lander::Input in =
            exec.make_input(s, node.time, cfg, false, false);
        check(in.main_throttle == 0.0 && !in.rotate_left && !in.rotate_right,
              "an aborted executor leaves no latent output");
    }
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_prograde = 4.0;
        lander::NodeExecutor exec;
        exec.arm(node, {{0.0, 1.0}, {1.0, 0.0}}, node.time - 0.5, cfg);

        lander::State before{};
        before.fuel = cfg.fuel_burn * dt;  // exactly one step of fuel
        const lander::Input input =
            exec.make_input(before, node.time - 0.5, cfg, false, false);
        lander::State after = before;
        after.fuel = 0.0;
        exec.after_step(before, after, input, node.time - 0.5 + dt, cfg);
        check(exec.state() == lander::ExecutorState::Incomplete,
              "running out of fuel mid-burn is INCOMPLETE");
    }
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_prograde = 4.0;
        lander::NodeExecutor exec;
        exec.arm(node, {{0.0, 1.0}, {1.0, 0.0}}, node.time - 0.5, cfg);

        lander::State before{};
        before.fuel = 1000.0;
        const lander::Input input =
            exec.make_input(before, node.time - 0.5, cfg, false, false);
        lander::State after = before;
        after.crashed = true;
        exec.after_step(before, after, input, node.time - 0.5 + dt, cfg);
        check(exec.state() == lander::ExecutorState::Aborted,
              "a crash aborts the executor");
    }
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_prograde = 4.0;
        lander::NodeExecutor exec;
        exec.arm(node, {{0.0, 1.0}, {1.0, 0.0}}, node.time - 0.5, cfg);

        lander::State before{};
        before.fuel = 1000.0;
        const lander::Input input =
            exec.make_input(before, node.time - 0.5, cfg, false, false);
        lander::State after = before;
        after.landed = true;
        exec.after_step(before, after, input, node.time - 0.5 + dt, cfg);
        check(exec.state() == lander::ExecutorState::Aborted,
              "landing mid-burn aborts the executor");
    }

    // A zero-delta-v node completes immediately.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        lander::NodeExecutor exec;
        exec.arm(node, {{0.0, 1.0}, {1.0, 0.0}}, 0.0, cfg);
        check(exec.state() == lander::ExecutorState::Complete,
              "a zero-delta-v node completes without burning");
    }
}

// M06-R22 (supersedes the R19 magnitude-gated rule; preserves the R19
// small-vector scenarios): continuous alignment safety during a burn — the
// exact failure observed in the M06-R18-H01 and M06-R21-H01 human gates,
// where the node executor spun the ship uncontrollably (STATE BURN / THR
// 1.00 while the thrust axis and the VGO ray were visibly separated, with
// the tracked VGO still large).
//
// Mechanism: each delivered impulse is applied along the previous step's
// nose, which lags the VGO direction. When the nose is materially off-axis,
// the impulse can rotate the remaining vector and feed the next step's
// pointing error. The hazard is not limited to the small-vector endgame;
// with a large enough initial misalignment or angular rate the VGO can grow
// while the craft spins. The safety is therefore full-range: thrust is
// emitted only while the nose is inside the 0.05 rad / 0.1 rad/s alignment
// band, and a misaligned burn step re-enters ALIGN with the VGO preserved.
//
// Both scenarios drive the real bang-bang law and the real semi-implicit
// attitude integration (no direct state mutation):
//   A: zero-gravity minimal geometry, armed at ignition with the exact
//      ignition state of the R19 failure (nose 0.0097 rad ahead of the VGO
//      direction, +0.01 rad/s);
//   B: the exact GUI node-executor fixture (seed 1005, the debug-fixture
//      orbit, the default node at 5.0 s with dv_prograde 0.5) through the
//      authoritative Simulation, driven exactly as gui.cpp does.
//
// The whole-run invariant both assert: the executor only ever emits main
// throttle while the ship is aligned with the VGO direction at the command
// step, regardless of VGO magnitude.
//
// The boundedness assertions separate the defect (an ever-growing spin while
// the engine fires, hundreds of fixed steps of thrust, the whole fuel tank
// consumed) from legitimate behaviour (a one-shot pre-ignition ALIGN
// rotation of up to a full turn or so at throttle 0, as in the existing
// misaligned-node test; the alignment gate itself bounds |omega| to 0.1
// rad/s at every burn re-entry).
//
// This test is the safety guard behind the R5-V08 closed-loop bound
// (tests/test_transfer_warm.cpp). The current 0.80 approach-ratio ceiling is
// only valid while this full-range guard stays mandatory: a future
// implementation that regains transfer margin by re-emitting off-axis thrust
// in a misaligned burn must fail here. Do not loosen the 0.05 rad /
// 0.1 rad/s band to make either the transfer test or this one pass.
 void test_node_executor_alignment_safety() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const double a = cfg.main_accel;

    // Read-only mirror of the executor's strict alignment band (the
    // NodeExecutor::aligned formula is private): the VGO direction within
    // 0.05 rad of the nose and |omega| within 0.1 rad/s; a zero remaining
    // vector is trivially aligned. This band bounds thrust at every VGO
    // magnitude (see the full-range invariant note above).
    auto aligned_mirror = [](const lander::Vec2& dv, const lander::State& s) {
        const double r = std::hypot(dv.x, dv.y);
        if (r < 1.0e-12) {
            return true;
        }
        const double desired = std::atan2(-dv.x, dv.y);
        double err =
            std::fmod(desired - s.angle + lander::kPi, lander::kTwoPi);
        if (err < 0.0) {
            err += lander::kTwoPi;
        }
        err -= lander::kPi;
        return std::abs(err) <= 0.05 && std::abs(s.omega) <= 0.1;
    };

    // A: zero gravity; the attitude and the executor's VGO accounting are
    // fully determined by (angle, omega, throttle), so this mirrors
    // integrate_flight for the burn dynamics exactly.
    {
        lander::ManeuverNode node{};
        node.time = 5.0;
        node.dv_prograde = 0.5;  // VGO (0, 0.5): the aligned nose angle is 0
        const lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

        lander::NodeExecutor exec;
        exec.arm(node, basis, 4.9375, cfg);  // armed at ignition
        check(exec.state() == lander::ExecutorState::Align,
              "arming at ignition starts in ALIGN");

        lander::State before{};
        before.fuel = 1000.0;
        before.angle = 0.0097;  // the nose leads the VGO direction
        before.omega = 0.01;    // (the observed ignition state)

        double now = 4.9375;
        int thrust_steps = 0;
        bool invariant_held = true;
        bool wait_after_ignition = false;
        double max_omega_thrusting = 0.0;
        double sweep = 0.0;
        for (int i = 0; i < 20000 && exec.active(); ++i) {
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            if (input.main_throttle > 0.0) {
                ++thrust_steps;
                max_omega_thrusting =
                    std::max(max_omega_thrusting, std::abs(before.omega));
                const lander::Vec2& dv = exec.dv_remaining();
                if (!aligned_mirror(dv, before)) {
                    invariant_held = false;
                }
            }
            lander::State after = before;
            lander::Vec2 acc{0.0, 0.0};
            if (input.main_throttle > 0.0) {
                acc.x = -a * input.main_throttle * std::sin(before.angle);
                acc.y = a * input.main_throttle * std::cos(before.angle);
            }
            if (input.rotate_left) {
                after.omega -= cfg.rotate_accel * dt;
            }
            if (input.rotate_right) {
                after.omega += cfg.rotate_accel * dt;
            }
            after.vx += acc.x * dt;
            after.vy += acc.y * dt;
            after.x += after.vx * dt;
            after.y += after.vy * dt;
            after.angle += after.omega * dt;
            after.fuel =
                std::max(0.0,
                         before.fuel -
                             (input.main_throttle > 0.0
                                  ? cfg.fuel_burn * dt
                                  : 0.0));
            now += dt;
            sweep += std::abs(after.omega) * dt;
            exec.after_step(before, after, input, now, cfg);
            if (now >= exec.ignite_time() &&
                exec.state() == lander::ExecutorState::Wait) {
                wait_after_ignition = true;
            }
            before = after;
        }
        check(exec.state() == lander::ExecutorState::Complete,
              "a small burn that loses alignment mid-burn completes "
              "instead of burning out");
        check(invariant_held,
              "thrust is only emitted while aligned with the VGO direction "
              "(full-range, whole run)");
        check(thrust_steps <= 60,
              "the burn stays bounded when alignment is lost and "
              "recovered");
        check(max_omega_thrusting <= 0.3,
              "no uncontrolled angular spin while the engine fires");
        check(sweep <= 10.0,
              "the whole run stays a bounded rotation, not a "
              "multi-turn spin");
        check(before.fuel > 900.0, "the burn does not drain the fuel tank");
        check(!wait_after_ignition,
              "after ignition the executor never returns to WAIT");
    }

    // B: the exact GUI node-executor fixture through the authoritative
    // Simulation, driven exactly as gui.cpp does.
    {
        lander::Simulation sim{};
        sim.reset(1005);
        const lander::Config& cfg2 = sim.config();
        const lander::BinarySystem& bin = sim.binary();
        const lander::Body& b = bin.body(0);
        const double r = b.terrain.max_surface_radius() + 20.0;
        const lander::Vec2 p0 = bin.position(0, 0.0);
        const lander::Vec2 v0 = bin.velocity(0, 0.0);
        const double speed = std::sqrt(b.mu / r);
        lander::State orbit{};
        orbit.x = p0.x;
        orbit.y = p0.y + r;
        orbit.vx = v0.x + speed;
        orbit.vy = v0.y;
        orbit.angle = 0.0;
        orbit.fuel = cfg2.fuel;
        sim.set_state(orbit);

        const double t0 = sim.sim_time();
        lander::ManeuverNode node =
            lander::default_node(t0, 0, cfg2.fixed_dt);
        node.dv_prograde = 0.5;  // the fixture's small visible burn
        const lander::State& st = sim.state();
        const double t =
            std::max(t0, lander::snap_time(node.time, cfg2.fixed_dt));
        const lander::BallisticState initial{{st.x, st.y}, {st.vx, st.vy},
                                             t0};
        const int steps = lander::ballistic_steps(t0, t, cfg2.fixed_dt);
        const lander::BallisticState pre = lander::propagate_ballistic(
            bin, initial, steps, cfg2.fixed_dt);
        const lander::NodeBasis basis = lander::compute_node_basis(
            bin, t, node.frame_body, pre.p, pre.v);

        lander::NodeExecutor exec;
        exec.arm(node, basis, t0, cfg2);

        int thrust_steps = 0;
        bool invariant_held = true;
        bool wait_after_ignition = false;
        double max_omega_thrusting = 0.0;
        double sweep = 0.0;
        for (int i = 0; i < 20000; ++i) {
            const lander::State before = sim.state();
            const double now = sim.sim_time();
            const lander::Input input =
                exec.make_input(before, now, cfg2, false, false);
            if (input.main_throttle > 0.0) {
                ++thrust_steps;
                max_omega_thrusting =
                    std::max(max_omega_thrusting, std::abs(before.omega));
                const lander::Vec2& dv = exec.dv_remaining();
                if (!aligned_mirror(dv, before)) {
                    invariant_held = false;
                }
            }
            sweep += std::abs(before.omega) * cfg2.fixed_dt;
            sim.step_once(input);
            exec.after_step(before, sim.state(), input, sim.sim_time(),
                            cfg2);
            if (sim.sim_time() >= exec.ignite_time() &&
                exec.state() == lander::ExecutorState::Wait) {
                wait_after_ignition = true;
            }
            if (!exec.active() || sim.state().crashed || sim.state().landed) {
                break;
            }
        }
        check(exec.state() == lander::ExecutorState::Complete,
              "the exact GUI fixture burn completes instead of spinning "
              "until fuel exhaustion");
        check(invariant_held,
              "thrust is only emitted while aligned with the VGO "
              "direction (GUI fixture, full-range, whole run)");
        check(thrust_steps <= 60, "the GUI fixture burn stays bounded");
        check(max_omega_thrusting <= 0.3,
              "no uncontrolled angular spin while the engine fires");
        check(sweep <= 10.0,
              "the whole run stays a bounded rotation, not a "
              "multi-turn spin");
        check(sim.state().fuel > 900.0,
              "the GUI fixture burn does not drain the fuel tank");
        check(!wait_after_ignition,
              "after ignition the executor never returns to WAIT");
        check(!sim.state().crashed && !sim.state().landed,
              "the fixture ship stays in the flight phase");
    }
}

// M06-R22 (R22-06 / V01): exact-failure regression for the full-range rule.
// The R19 magnitude-gated logic would have started this burn at full throttle
// because the tracked VGO is well above the small-vector threshold even though
// the nose is 0.5 rad off-axis. The full-range gate must cut thrust for the
// entire misaligned pre-ignition swing, preserve the VGO, align, and then
// complete a bounded physical burn.
void test_node_executor_full_range_alignment_safety() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const double a = cfg.main_accel;

    auto aligned_mirror = [](const lander::Vec2& dv, const lander::State& s) {
        const double r = std::hypot(dv.x, dv.y);
        if (r < 1.0e-12) {
            return true;
        }
        const double desired = std::atan2(-dv.x, dv.y);
        double err =
            std::fmod(desired - s.angle + lander::kPi, lander::kTwoPi);
        if (err < 0.0) {
            err += lander::kTwoPi;
        }
        err -= lander::kPi;
        return std::abs(err) <= 0.05 && std::abs(s.omega) <= 0.1;
    };

    lander::ManeuverNode node{};
    node.time = 5.0;
    node.dv_prograde = 4.0;  // large VGO: outside the R19 small-vector regime
    const lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

    lander::NodeExecutor exec;
    exec.arm(node, basis, 5.0, cfg, true);
    check(exec.state() == lander::ExecutorState::Burn,
          "a direct-burn arm starts in BURN even when materially "
          "misaligned");

    lander::State before{};
    before.fuel = 1000.0;
    before.angle = 0.5;  // 28.6 deg off the VGO direction
    before.omega = 0.0;

    const double vgo0 =
        std::hypot(exec.dv_remaining().x, exec.dv_remaining().y);
    double now = 5.0;
    int thrust_steps = 0;
    bool invariant_held = true;
    bool vgo_preserved = true;
    bool first_thrust = false;
    double max_omega_thrusting = 0.0;
    double max_err_thrusting = 0.0;
    double sweep = 0.0;
    for (int i = 0; i < 20000 && exec.active(); ++i) {
        const lander::Input input =
            exec.make_input(before, now, cfg, false, false);
        const lander::Vec2& dv = exec.dv_remaining();
        if (input.main_throttle > 0.0) {
            ++thrust_steps;
            first_thrust = true;
            max_omega_thrusting =
                std::max(max_omega_thrusting, std::abs(before.omega));
            const double desired = std::atan2(-dv.x, dv.y);
            double err =
                std::fmod(desired - before.angle + lander::kPi,
                          lander::kTwoPi);
            if (err < 0.0) {
                err += lander::kTwoPi;
            }
            err -= lander::kPi;
            max_err_thrusting = std::max(max_err_thrusting, std::abs(err));
            if (!aligned_mirror(dv, before)) {
                invariant_held = false;
            }
        } else if (!first_thrust &&
                   std::hypot(dv.x, dv.y) != vgo0) {
            vgo_preserved = false;
        }

        lander::State after = before;
        lander::Vec2 acc{0.0, 0.0};
        if (input.main_throttle > 0.0) {
            acc.x = -a * input.main_throttle * std::sin(before.angle);
            acc.y = a * input.main_throttle * std::cos(before.angle);
        }
        if (input.rotate_left) {
            after.omega -= cfg.rotate_accel * dt;
        }
        if (input.rotate_right) {
            after.omega += cfg.rotate_accel * dt;
        }
        after.vx += acc.x * dt;
        after.vy += acc.y * dt;
        after.x += after.vx * dt;
        after.y += after.vy * dt;
        after.angle += after.omega * dt;
        after.fuel = std::max(
            0.0,
            before.fuel -
                (input.main_throttle > 0.0 ? cfg.fuel_burn * dt : 0.0));
        sweep += std::abs(after.omega) * dt;
        now += dt;
        exec.after_step(before, after, input, now, cfg);
        before = after;
    }

    check(exec.state() == lander::ExecutorState::Complete,
          "a materially misaligned large-VGO burn completes instead of "
          "spinning");
    check(invariant_held,
          "full-range: thrust is only emitted while aligned with the VGO "
          "direction (large-VGO regression, whole run)");
    check(vgo_preserved,
          "the VGO is preserved through the zero-throttle alignment hold");
    check(thrust_steps <= 200,
          "the large-VGO burn stays bounded after the alignment hold");
    check(max_omega_thrusting <= 0.11,
          "no uncontrolled angular spin while the engine fires");
    check(max_err_thrusting <= 0.05 + 1.0e-9,
          "the thrust vector stays inside the alignment envelope");
    check(sweep <= 10.0,
          "the whole run stays a bounded rotation, not a multi-turn spin");
    check(before.fuel > 900.0,
          "the large-VGO burn does not drain the fuel tank");
}

// M06-R18 (R18-02 / V01..V05): the presentation layer of the node-executor
// isolation. The executor is driven exactly as in test_node_executor above
// (no executor change); every assertion is about the PRESENTATION quantities
// -- the actually-applied `Input.main_throttle` and the pure
// `presentation_thrust_level` mapping the drawn plume is sourced from -- plus
// the physicality of the burn in the authoritative simulation (the per-step
// velocity change beyond an identical zero-input reference run equals the
// applied thrust: no "magic force", no plume without engine).
void test_node_executor_presentation() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;

    // (A) Manual throttle 0, executor burning at ignition: the applied
    // input is full throttle and the presentation mapping reports a
    // non-zero plume that equals it.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.frame_body = 0;
        node.dv_prograde = 4.0;
        const lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

        lander::NodeExecutor exec;
        exec.arm(node, basis, node.time - 0.5, cfg);  // armed at ignition
        lander::State s{};
        s.fuel = 1000.0;
        s.angle = 0.0;  // aligned with the +y VGO
        const lander::Input input =
            exec.make_input(s, node.time - 0.5, cfg, false, false);
        check(input.main_throttle > 0.0,
              "A: an executor burn with manual throttle 0 applies main thrust");
        check_close(input.main_throttle, 1.0, 1e-12,
                    "A: the 4 m/s node ignites at full throttle");
        check_close(lander::presentation_thrust_level(s, input.main_throttle),
                    input.main_throttle, 1e-12,
                    "A: with manual 0 and applied > 0, the plume source "
                    "equals the applied input");
    }

    // (B) ALIGN and WAIT: no applied thrust, and the mapping reports no
    // plume.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_prograde = 4.0;
        const lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

        // Aligned before ignition: the first input already holds (throttle
        // 0) and the executor settles into WAIT.
        lander::NodeExecutor wait_exec;
        wait_exec.arm(node, basis, node.time - 5.0, cfg);
        lander::State ws{};
        ws.fuel = 1000.0;
        ws.angle = 0.0;
        const lander::Input wait_in =
            wait_exec.make_input(ws, node.time - 5.0, cfg, false, false);
        check(wait_in.main_throttle == 0.0, "B: WAIT applies no main thrust");
        check(lander::presentation_thrust_level(ws, wait_in.main_throttle) ==
                  0.0,
              "B: WAIT shows no plume");
        lander::State ws2 = ws;
        wait_exec.after_step(ws, ws2, wait_in, node.time - 5.0 + dt, cfg);
        check(wait_exec.state() == lander::ExecutorState::Wait,
              "B: the aligned pre-ignition node holds in WAIT");

        // Misaligned before ignition: attitude-only ALIGN, no thrust.
        lander::NodeExecutor align_exec;
        align_exec.arm(node, basis, node.time - 5.0, cfg);
        lander::State as{};
        as.fuel = 1000.0;
        as.angle = 0.9;
        const lander::Input align_in =
            align_exec.make_input(as, node.time - 5.0, cfg, false, false);
        check(align_in.main_throttle == 0.0, "B: ALIGN applies no main thrust");
        check(lander::presentation_thrust_level(as, align_in.main_throttle) ==
                  0.0,
              "B: ALIGN shows no plume");
        check(align_exec.state() == lander::ExecutorState::Align,
              "B: the misaligned node stays in ALIGN");
    }

    // (C) BURN in the authoritative simulation: the applied throttle is
    // positive, fuel decreases, the tracked VGO decreases, the plume source
    // equals the applied input at every burning step, and the per-step
    // velocity change beyond an identical zero-input reference run equals
    // the applied thrust (the engine, not magic).
    {
        const std::uint64_t seed = 503;
        const auto bin = lander::BinarySystem::canonical(
            cfg.mu, seed, lander::companion_seed(seed));
        lander::State start = state_relative(
            bin, 0, 100.0, 400.0, 0.0, 23.2, 0.0);
        start.angle = 0.0;  // nose along the +y VGO
        start.omega = 0.0;

        lander::Simulation sim;
        sim.reset(seed);
        sim.set_state(start);
        lander::Simulation ref;
        ref.reset(seed);
        ref.set_state(start);

        lander::ManeuverNode node{};
        node.time = 1.0;  // burn_time 1 s -> ignition at 0.5
        node.frame_body = 0;
        node.dv_prograde = 4.0;
        const lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};
        lander::NodeExecutor exec;
        exec.arm(node, basis, sim.sim_time(), cfg);

        double now = sim.sim_time();
        int wait_steps = 0;
        int burn_steps = 0;
        bool plume_matches = true;
        bool dv_matches = true;
        bool fuel_decreases = true;
        for (int i = 0; i < 120; ++i) {
            const lander::State before = sim.state();
            const lander::State ref_before = ref.state();
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            (void)sim.step_once(input);
            (void)ref.step_once(lander::Input{});
            now += dt;
            exec.after_step(before, sim.state(), input, now, cfg);

            const double expected_plume =
                lander::presentation_thrust_level(sim.state(),
                                                  input.main_throttle);
            if (std::abs(expected_plume - input.main_throttle) > 1e-12) {
                plume_matches = false;
            }
            if (input.main_throttle > 0.0) {
                ++burn_steps;
                if (sim.state().fuel >= before.fuel) {
                    fuel_decreases = false;
                }
                // Isolate the engine term: the identical reference run
                // (zero input, same gravity field) carries no thrust, so the
                // difference of the two per-step velocity changes is the
                // applied thrust along the pre-step nose (within the tiny
                // gravity-field difference between the diverging positions).
                const double dvx =
                    (sim.state().vx - before.vx) -
                    (ref.state().vx - ref_before.vx);
                const double dvy =
                    (sim.state().vy - before.vy) -
                    (ref.state().vy - ref_before.vy);
                const double ex =
                    -std::sin(before.angle) *
                    cfg.main_accel * input.main_throttle * dt;
                const double ey =
                    std::cos(before.angle) *
                    cfg.main_accel * input.main_throttle * dt;
                if (std::hypot(dvx - ex, dvy - ey) > 1e-3) {
                    dv_matches = false;
                }
            } else {
                ++wait_steps;
            }
        }
        check(wait_steps == 60, "C: the node waits 0.5 s before ignition");
        check(burn_steps == 60, "C: the first 0.5 s of the 1 s burn elapse");
        check(plume_matches, "C: the plume source equals the applied input "
                             "at every step");
        check(fuel_decreases, "C: burning consumes fuel in the authoritative "
                              "simulation");
        check(dv_matches, "C: the per-step delta-v matches the applied "
                          "thrust (no magic force)");
        check_close(vec_length(exec.dv_remaining()), 2.0, 1e-9,
                    "C: half the node delta-v is delivered at the node time");
        check(!sim.state().crashed && !sim.state().landed,
              "C: the run stays healthy");
    }

    // (D) The final partial step: 0 < applied < 1, and the plume source
    // equals the applied fraction. A 4.11 m/s node needs 123 full steps
    // plus one ~30 % partial step.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_prograde = 4.11;
        const lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

        lander::NodeExecutor exec;
        exec.arm(node, basis, node.time - 0.5, cfg);  // armed at ignition

        lander::State before{};
        before.fuel = 1000.0;
        before.angle = 0.0;
        double now = node.time - 0.5;
        int burn_steps = 0;
        double last_throttle = -1.0;
        bool partial_ok = true;
        for (int i = 0; i < 240 && exec.active(); ++i) {
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            lander::State after = before;
            after.fuel = std::max(
                0.0, before.fuel - (input.main_throttle > 0.0
                                       ? cfg.fuel_burn * dt
                                       : 0.0));
            now += dt;
            if (input.main_throttle > 0.0) {
                ++burn_steps;
                last_throttle = input.main_throttle;
                if (std::abs(
                        lander::presentation_thrust_level(
                            after, input.main_throttle) -
                        input.main_throttle) > 1e-12) {
                    partial_ok = false;
                }
            }
            exec.after_step(before, after, input, now, cfg);
            before = after;
        }
        check(exec.state() == lander::ExecutorState::Complete,
              "D: the partial-step node completes");
        check(burn_steps == 124, "D: 123 full steps plus one partial step");
        check(last_throttle > 0.0 && last_throttle < 1.0,
              "D: the final step is a 0 < applied < 1 partial throttle");
        check_close(last_throttle, 0.3, 1e-2,
                    "D: the final step burns ~30 % (the 0.01 m/s remainder)");
        check(partial_ok, "D: the plume source equals the applied fraction");
    }

    // (E) Terminal states leave no residual thrust, applied or rendered.
    {
        lander::ManeuverNode node{};
        node.time = 100.0;
        node.dv_prograde = 4.0;
        const lander::NodeBasis basis{{0.0, 1.0}, {1.0, 0.0}};

        // After COMPLETE, repeated inputs stay silent.
        lander::NodeExecutor exec;
        exec.arm(node, basis, node.time - 0.5, cfg);
        lander::State before{};
        before.fuel = 1000.0;
        before.angle = 0.0;
        double now = node.time - 0.5;
        while (exec.active()) {
            const lander::Input input =
                exec.make_input(before, now, cfg, false, false);
            lander::State after = before;
            after.fuel = std::max(
                0.0, before.fuel - (input.main_throttle > 0.0
                                       ? cfg.fuel_burn * dt
                                       : 0.0));
            now += dt;
            exec.after_step(before, after, input, now, cfg);
            before = after;
        }
        for (int i = 0; i < 3; ++i) {
            const lander::Input idle =
                exec.make_input(before, now + i * dt, cfg, false, false);
            check(idle.main_throttle == 0.0,
                  "E: a completed executor leaves no latent thrust");
            check(lander::presentation_thrust_level(before,
                                                    idle.main_throttle) == 0.0,
                  "E: a completed executor renders no plume");
        }

        // After ABORT mid-burn, the executor goes silent immediately.
        lander::NodeExecutor abort_exec;
        abort_exec.arm(node, basis, node.time - 0.5, cfg);
        lander::State ab{};
        ab.fuel = 1000.0;
        ab.angle = 0.0;
        const lander::Input first =
            abort_exec.make_input(ab, node.time - 0.5, cfg, false, false);
        lander::State ab2 = ab;
        ab2.fuel -= cfg.fuel_burn * dt;
        abort_exec.after_step(ab, ab2, first, node.time - 0.5 + dt, cfg);
        abort_exec.abort();
        for (int i = 0; i < 2; ++i) {
            const lander::Input in =
                abort_exec.make_input(ab2, node.time - 0.5 + dt + i * dt,
                                      cfg, false, false);
            check(in.main_throttle == 0.0,
                  "E: an aborted executor leaves no latent thrust");
            check(lander::presentation_thrust_level(ab2, in.main_throttle) ==
                      0.0,
                  "E: an aborted executor renders no plume");
        }

        // Suppression gates: even with a stale non-zero applied value, a
        // crashed or landed ship (or an empty tank) renders no plume.
        lander::State crashed{};
        crashed.fuel = 500.0;
        crashed.crashed = true;
        check(lander::presentation_thrust_level(crashed, 1.0) == 0.0,
              "E: a crashed ship renders no plume");
        lander::State landed{};
        landed.fuel = 500.0;
        landed.landed = true;
        check(lander::presentation_thrust_level(landed, 1.0) == 0.0,
              "E: a landed ship renders no plume");
        lander::State empty{};
        empty.fuel = 0.0;
        check(lander::presentation_thrust_level(empty, 1.0) == 0.0,
              "E: an empty tank renders no plume");
    }
}

void test_determinism() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 503;
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, seed, lander::companion_seed(seed));
    const lander::State start = state_relative(bin, 0, 100.0, 100.0, 0.0,
                                               12.0, 0.0);

    auto run = [&]() {
        lander::Simulation sim;
        sim.reset(seed);
        sim.set_state(start);

        lander::ManeuverNode node{};
        node.time = lander::snap_time(3.0, dt);
        node.frame_body = 0;
        node.dv_prograde = 1.0;
        node.dv_radial = 0.5;

        const auto basis = [&, &sim = sim]() {
            const int steps = lander::ballistic_steps(0.0, node.time, dt);
            const auto pre = lander::propagate_ballistic(
                bin, {{start.x, start.y}, {start.vx, start.vy}, 0.0}, steps,
                dt);
            return lander::compute_node_basis(bin, node.time, 0, pre.p,
                                              pre.v);
        }();

        lander::NodeExecutor exec;
        exec.arm(node, basis, sim.sim_time(), cfg);
        lander::AttitudeMode mode = lander::AttitudeMode::Prograde;

        for (int i = 0; i < 240; ++i) {
            const auto state = sim.state();
            const lander::Vec2 ship{state.x, state.y};
            auto target_dir = lander::attitude_target_direction(
                mode, basis, ship, {0.0, 0.0}, exec.dv_remaining());
            if (mode == lander::AttitudeMode::Maneuver && exec.active()) {
                const auto dir = exec.dv_remaining();
                target_dir = std::hypot(dir.x, dir.y) > 1.0e-9
                                 ? std::optional<lander::Vec2>(
                                       dir * (1.0 / std::hypot(dir.x, dir.y)))
                                 : std::nullopt;
            }
            lander::Input input = lander::attitude_input(
                state, cfg, target_dir, false, false);
            if (exec.active()) {
                const lander::Input exec_input =
                    exec.make_input(state, sim.sim_time(), cfg, false, false);
                input = exec_input;
            }
            const lander::State before = state;
            const bool alive = sim.step_once(input);
            exec.after_step(before, sim.state(), input, sim.sim_time(), cfg);
            (void)alive;
        }
        return std::make_pair(sim.state(), exec.state());
    };

    const auto a = run();
    const auto b = run();
    check(a.first == b.first, "repeated scripted runs produce the same state");
    check(a.second == b.second,
          "repeated scripted runs produce the same executor state");
}

}  // namespace

int main() {
    test_snap_time_and_default_node();
    test_node_basis();
    test_node_world_dv();
    test_predictor_parity();
    test_node_impulse_prediction();
    test_pre_post_join();
    test_node_time_effective_overdue();
    test_trajectory_prediction();
    test_planners();
    test_attitude_controller();
    test_node_executor();
    test_node_executor_alignment_safety();
    test_node_executor_full_range_alignment_safety();
    test_node_executor_presentation();
    test_determinism();

    if (failures == 0) {
        std::puts("All lander_flight_computer_tests passed");
        return 0;
    }
    std::printf("%d lander_flight_computer_tests failed\n", failures);
    return 1;
}
