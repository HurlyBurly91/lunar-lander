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
    test_trajectory_prediction();
    test_planners();
    test_attitude_controller();
    test_node_executor();
    test_determinism();

    if (failures == 0) {
        std::puts("All lander_flight_computer_tests passed");
        return 0;
    }
    std::printf("%d lander_flight_computer_tests failed\n", failures);
    return 1;
}
