#include "lander/flight_computer.hpp"

#include <algorithm>
#include <cmath>

namespace lander {

namespace {

double dot2(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }

double cross2(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }

double vec_length(const Vec2& a) { return std::hypot(a.x, a.y); }

Vec2 normalize_safe(const Vec2& a) {
    const double r = vec_length(a);
    if (r < 1.0e-12) {
        return {};
    }
    return {a.x / r, a.y / r};
}

bool finite_vec(const Vec2& a) {
    return std::isfinite(a.x) && std::isfinite(a.y);
}

bool finite_basis(const NodeBasis& b) {
    return finite_vec(b.prograde) && finite_vec(b.radial_out);
}

// The destination base pad is a point fixed on the destination body's rotating
// surface at body-local arc 0 (the M05 contract pad convention).
BinarySystem::SurfacePoint destination_pad(const BinarySystem& bin,
                                           int destination_body, double t) {
    const Terrain& ter = bin.body(destination_body).terrain;
    return bin.surface_point(destination_body, ter.angle_at_arc(0.0),
                             ter.surface_radius_at_arc(0.0), t);
}

bool inside_terrain(const BinarySystem& bin, int body, const Vec2& p,
                    double t) {
    const Terrain& ter = bin.body(body).terrain;
    const Vec2 bpos = bin.position(body, t);
    const double rx = p.x - bpos.x;
    const double ry = p.y - bpos.y;
    const double rho = std::hypot(rx, ry);
    if (rho < 1.0e-9) {
        return true;
    }
    const double theta = std::atan2(ry, rx);
    const double arc = ter.arc_at_angle(theta - bin.body_rotation(t));
    return rho <= ter.surface_radius_at_arc(arc);
}

struct RhoWindow {
    int count{0};
    double t_prev{0.0};
    double t_mid{0.0};
    double r_prev{0.0};
    double r_mid{0.0};
    Vec2 p_prev{};
    Vec2 p_mid{};

    void push(double t, double r, const Vec2& p, PeriAp& peri, PeriAp& apo,
              double threshold) {
        if (count == 0) {
            t_prev = t;
            r_prev = r;
            p_prev = p;
            count = 1;
            return;
        }
        if (count == 1) {
            t_mid = t;
            r_mid = r;
            p_mid = p;
            count = 2;
            return;
        }
        const double r_curr = r;
        if (r_mid < r_prev && r_mid < r_curr &&
            std::max(r_prev, r_curr) - r_mid >= threshold && !peri.valid) {
            peri = {true, r_mid, t_mid, p_mid};
        }
        if (r_mid > r_prev && r_mid > r_curr &&
            r_mid - std::min(r_prev, r_curr) >= threshold && !apo.valid) {
            apo = {true, r_mid, t_mid, p_mid};
        }
        t_prev = t_mid;
        r_prev = r_mid;
        p_prev = p_mid;
        t_mid = t;
        r_mid = r;
        p_mid = p;
    }
};

}  // namespace

double snap_time(double t, double dt) {
    if (dt <= 0.0) {
        return t;
    }
    const double steps = std::llround(t / dt);
    return steps * dt;
}

ManeuverNode default_node(double t0, int frame_body, double dt) {
    ManeuverNode node{};
    node.time = snap_time(t0 + 5.0, dt);
    node.frame_body = frame_body < 0 ? 0 : (frame_body > 1 ? 1 : frame_body);
    return node;
}

NodeBasis compute_node_basis(const BinarySystem& bin, double t,
                             int frame_body, const Vec2& p_pre,
                             const Vec2& v_pre) {
    const int index = frame_body < 0 ? 0 : (frame_body > 1 ? 1 : frame_body);
    const Vec2 bpos = bin.position(index, t);
    const Vec2 bvel = bin.velocity(index, t);
    const Vec2 r = p_pre - bpos;
    const Vec2 v = v_pre - bvel;
    const double h = cross2(r, v);

    NodeBasis basis{};

    Vec2 r_hat = normalize_safe(r);
    if (vec_length(r_hat) < 1.0e-12) {
        r_hat = {1.0, 0.0};
    }

    Vec2 prograde = normalize_safe(v);
    if (vec_length(prograde) < 1.0e-12) {
        prograde = normalize_safe(h >= 0.0 ? Vec2{-r.y, r.x}
                                           : Vec2{r.y, -r.x});
    }
    if (vec_length(prograde) < 1.0e-12) {
        prograde = normalize_safe(Vec2{-r_hat.y, r_hat.x});
    }
    if (vec_length(prograde) < 1.0e-12) {
        prograde = {1.0, 0.0};
    }

    const Vec2 radial_candidate =
        r_hat - Vec2{dot2(r_hat, prograde) * prograde.x,
                     dot2(r_hat, prograde) * prograde.y};
    Vec2 radial_out = normalize_safe(radial_candidate);
    if (vec_length(radial_out) < 1.0e-12) {
        radial_out = normalize_safe(Vec2{-prograde.y, prograde.x});
    }
    if (vec_length(radial_out) < 1.0e-12) {
        radial_out = {0.0, 1.0};
    }
    if (dot2(radial_out, r_hat) < 0.0) {
        radial_out = radial_out * -1.0;
    }

    basis.prograde = prograde;
    basis.radial_out = radial_out;
    return basis;
}

Vec2 node_world_dv(const ManeuverNode& node, const NodeBasis& basis) {
    const Vec2 pg = basis.prograde * node.dv_prograde;
    const Vec2 rd = basis.radial_out * node.dv_radial;
    return pg + rd;
}

TrajectoryPrediction predict_trajectory(
    const BinarySystem& bin, const Config& config, const State& start,
    double t0, int reference_body, int destination_body,
    const std::optional<ManeuverNode>& node, double horizon_seconds,
    int target_samples) {
    TrajectoryPrediction out{};
    const int ref = reference_body < 0 ? 0 : (reference_body > 1 ? 1 : reference_body);
    const int dest = destination_body < 0 ? 0 : (destination_body > 1 ? 1 : destination_body);
    const double dt = config.fixed_dt;
    const double t_end = t0 + horizon_seconds;

    if (start.crashed) {
        out.pre.push_back({start.x, start.y});
        out.timed.push_back({{start.x, start.y}, {start.vx, start.vy}, t0});
        return out;
    }
    if (start.landed) {
        // A landed ship is not in free flight; a zero-thrust ballistic
        // prediction would immediately intersect the surface it is resting on.
        // Report the ship's current surface position and stop.
        out.pre.push_back({start.x, start.y});
        out.timed.push_back({{start.x, start.y}, {start.vx, start.vy}, t0});
        const auto pad = destination_pad(bin, dest, t0);
        const double d = std::hypot(start.x - pad.position.x,
                                    start.y - pad.position.y);
        out.closest = {true, d, t0, {start.x, start.y}, pad.position};
        return out;
    }

    const BallisticState initial{
        {start.x, start.y}, {start.vx, start.vy}, t0};
    const double t_node =
        node ? std::max(t0, snap_time(node->time, dt)) : t_end;
    const int pre_steps = ballistic_steps(t0, t_node, dt);
    const int total_steps = ballistic_steps(t0, t_end, dt);
    const int stride = std::max(1, total_steps / std::max(1, target_samples));

    auto update_world = [&](const BallisticState& st) {
        if (!out.impact.valid) {
            for (int i = 0; i < 2; ++i) {
                if (inside_terrain(bin, i, st.p, st.t)) {
                    out.impact = {true, i, st.t, st.p};
                    break;
                }
            }
        }
        const auto pad = destination_pad(bin, dest, st.t);
        const double d = std::hypot(st.p.x - pad.position.x,
                                    st.p.y - pad.position.y);
        if (!out.closest.valid || d < out.closest.distance) {
            out.closest = {true, d, st.t, st.p, pad.position};
        }
    };

    auto add_pre_sample = [&](const BallisticState& st) {
        if (out.pre.empty() || out.pre.back() != st.p) {
            out.pre.push_back(st.p);
            out.timed.push_back({st.p, st.v, st.t});
        }
    };

    BallisticState cur = initial;
    update_world(cur);
    add_pre_sample(cur);

    bool impacted_before_node = false;
    for (int i = 0; i < pre_steps; ++i) {
        cur = step_ballistic(bin, cur, dt);
        update_world(cur);
        if (i % stride == 0 || i + 1 == pre_steps || out.impact.valid) {
            add_pre_sample(cur);
        }
        if (out.impact.valid) {
            impacted_before_node = true;
            break;
        }
    }

    if (!node) {
        out.node_position = cur.p;
        out.basis_valid = false;
        out.total_dv = 0.0;
        return out;
    }

    // The node state is the pure zero-thrust endpoint, even if the display
    // branch stopped early at an impact.
    const BallisticState pre_end =
        propagate_ballistic(bin, initial, pre_steps, dt);
    out.node_position = pre_end.p;
    out.basis = compute_node_basis(bin, t_node, node->frame_body, pre_end.p,
                                   pre_end.v);
    out.basis_valid = finite_basis(out.basis);
    out.dv_world = out.basis_valid ? node_world_dv(*node, out.basis) : Vec2{};
    out.total_dv = vec_length(out.dv_world);

    if (impacted_before_node) {
        return out;
    }

    BallisticState post = pre_end;
    post.v = post.v + out.dv_world;

    RhoWindow window{};
    const double significance = 0.25;
    auto track_post = [&](const BallisticState& st, bool record_sample) {
        update_world(st);
        if (record_sample && (out.post.empty() || out.post.back() != st.p)) {
            out.post.push_back(st.p);
            out.timed.push_back({st.p, st.v, st.t});
        }
        const Vec2 ref_pos = bin.position(ref, st.t);
        const double rho =
            std::hypot(st.p.x - ref_pos.x, st.p.y - ref_pos.y);
        window.push(st.t, rho, st.p, out.peri, out.apo, significance);
    };

    track_post(post, true);

    const int post_steps = ballistic_steps(t_node, t_end, dt);
    for (int i = 0; i < post_steps; ++i) {
        post = step_ballistic(bin, post, dt);
        track_post(post, i % stride == 0 || i + 1 == post_steps ||
                       out.impact.valid);
        if (out.impact.valid) {
            break;
        }
    }

    return out;
}

std::optional<ManeuverNode> plan_circularize(
    const BinarySystem& bin, const Config& config, const State& start,
    double t0, int reference_body,
    const std::optional<ManeuverNode>& existing) {
    const int ref = reference_body < 0 ? 0 : (reference_body > 1 ? 1 : reference_body);
    ManeuverNode node = existing ? *existing
                                 : default_node(t0, ref, config.fixed_dt);
    const int frame = node.frame_body < 0 ? 0
                     : (node.frame_body > 1 ? 1 : node.frame_body);
    node.frame_body = frame;
    const double t = std::max(t0, snap_time(node.time, config.fixed_dt));
    if (t < t0) {
        return std::nullopt;
    }

    const BallisticState initial{{start.x, start.y}, {start.vx, start.vy}, t0};
    const int steps = ballistic_steps(t0, t, config.fixed_dt);
    const BallisticState pre = propagate_ballistic(bin, initial, steps,
                                                   config.fixed_dt);

    const Vec2 bpos = bin.position(frame, t);
    const Vec2 bvel = bin.velocity(frame, t);
    const Vec2 r = pre.p - bpos;
    const Vec2 v = pre.v - bvel;
    const double rho = vec_length(r);
    const double mu = bin.body(frame).mu;
    if (rho < 1.0e-6 || mu <= 0.0) {
        return std::nullopt;
    }

    const double v_circ = std::sqrt(mu / rho);
    const double h = cross2(r, v);
    Vec2 tangent = normalize_safe(h >= 0.0 ? Vec2{-r.y, r.x}
                                           : Vec2{r.y, -r.x});
    if (vec_length(tangent) < 1.0e-12) {
        tangent = h >= 0.0 ? Vec2{0.0, 1.0} : Vec2{0.0, -1.0};
    }

    const Vec2 v_circ_vec = tangent * v_circ;
    const Vec2 v_desired = bvel + v_circ_vec;
    const Vec2 dv_world = v_desired - pre.v;

    const NodeBasis basis =
        compute_node_basis(bin, t, frame, pre.p, pre.v);
    node.time = t;
    node.dv_prograde = dot2(dv_world, basis.prograde);
    node.dv_radial = dot2(dv_world, basis.radial_out);
    return node;
}

std::optional<ManeuverNode> plan_transfer(
    const BinarySystem& bin, const Config& config, const State& start,
    double t0, int reference_body,
    const std::optional<ManeuverNode>& existing, TransferSolution* cache) {
    const int source = start.landed ? start.landed_body : reference_body;
    const int safe_source =
        source < 0 ? 0 : (source > 1 ? 1 : source);
    const int target = 1 - safe_source;

    ManeuverNode node = existing ? *existing
                                 : default_node(t0, safe_source,
                                                config.fixed_dt);
    const int frame = node.frame_body < 0 ? 0
                     : (node.frame_body > 1 ? 1 : node.frame_body);
    node.frame_body = frame;
    const double t = std::max(t0, snap_time(node.time, config.fixed_dt));

    const BallisticState initial{{start.x, start.y}, {start.vx, start.vy}, t0};
    const int steps = ballistic_steps(t0, t, config.fixed_dt);
    const BallisticState pre = propagate_ballistic(bin, initial, steps,
                                                   config.fixed_dt);

    // M06-R5: warm-first, cold-fallback transfer targeting from the predicted
    // pre-burn state. A valid cached solution for this route is re-aimed with
    // a bounded differential correction; any warm failure (or an empty /
    // mismatched cache) drops to the full coarse search, which reseeds the
    // cache. When no cache pointer is supplied this is exactly the original
    // pure-cold behaviour.
    Vec2 departure{};
    bool solved = false;
    if (cache && cache->valid && cache->source == safe_source &&
        cache->target == target) {
        const TransferSolution warm =
            solve_transfer_warm(bin, config.fixed_dt, pre.p, safe_source,
                                target, t, *cache);
        if (warm.valid) {
            *cache = warm;
            departure = warm.departure_velocity;
            solved = true;
        }
    }
    if (!solved) {
        if (!solve_transfer_velocity(bin, config.fixed_dt, pre.p, safe_source,
                                     target, t, departure, cache)) {
            return std::nullopt;
        }
    }

    const Vec2 dv_world = departure - pre.v;
    const NodeBasis basis =
        compute_node_basis(bin, t, frame, pre.p, pre.v);
    node.time = t;
    node.dv_prograde = dot2(dv_world, basis.prograde);
    node.dv_radial = dot2(dv_world, basis.radial_out);
    return node;
}

std::optional<ManeuverNode> plan_match_target(
    const BinarySystem& bin, const Config& config, const State& start,
    double t0, int reference_body, int destination_body,
    const std::optional<ManeuverNode>& existing) {
    const int ref = reference_body < 0 ? 0 : (reference_body > 1 ? 1 : reference_body);
    const int dest = destination_body < 0 ? 0 : (destination_body > 1 ? 1 : destination_body);

    ManeuverNode node = existing ? *existing
                                 : default_node(t0, ref, config.fixed_dt);
    if (!existing) {
        // Place a brand-new node near the predicted closest approach to the
        // moving destination pad.
        const TrajectoryPrediction probe = predict_trajectory(
            bin, config, start, t0, ref, dest, std::nullopt,
            2.0 * bin.period(), 1024);
        if (probe.closest.valid && probe.closest.time > t0) {
            node.time = snap_time(probe.closest.time, config.fixed_dt);
        }
    }
    const int frame = node.frame_body < 0 ? 0
                     : (node.frame_body > 1 ? 1 : node.frame_body);
    node.frame_body = frame;
    const double t = std::max(t0, snap_time(node.time, config.fixed_dt));

    const BallisticState initial{{start.x, start.y}, {start.vx, start.vy}, t0};
    const int steps = ballistic_steps(t0, t, config.fixed_dt);
    const BallisticState pre = propagate_ballistic(bin, initial, steps,
                                                   config.fixed_dt);

    const auto pad = destination_pad(bin, dest, t);
    const Vec2 dv_world = pad.velocity - pre.v;
    const NodeBasis basis =
        compute_node_basis(bin, t, frame, pre.p, pre.v);
    node.time = t;
    node.dv_prograde = dot2(dv_world, basis.prograde);
    node.dv_radial = dot2(dv_world, basis.radial_out);
    return node;
}

}  // namespace lander
