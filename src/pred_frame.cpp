// M06-R12: prediction reference-frame transform and AUTO orbit-reference
// classifier. See include/lander/pred_frame.hpp for the invariants. This file
// is pure analysis over world-space samples: it performs inertial frame shifts
// only (no rotation, no body-fixed frame) and runs a bounded orbit-reference
// classifier with hysteresis. It never touches the simulation or physics.

#include "lander/pred_frame.hpp"

#include <cmath>

#include "lander/terrain.hpp"  // kPi, kTwoPi

namespace lander {

namespace {

// Clamp a value into [lo, hi].
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Wrap an angle to (-pi, pi].
double wrap_pi(double a) {
    a = std::fmod(a + M_PI, kTwoPi);
    if (a < 0.0) a += kTwoPi;
    return a - M_PI;
}

// Internal target encoding for the hysteresis automaton: HOLD keeps the current
// committed segment; 0 / 1 are the two bodies; 2 is the world (transfer)
// segment.
constexpr int kHold = -1;
constexpr int kPrimary = 0;
constexpr int kCompanion = 1;
constexpr int kWorld = 2;

int target_of(RefSegment s) {
    return s == RefSegment::Primary ? kPrimary
         : s == RefSegment::Companion ? kCompanion
                                      : kWorld;
}
RefSegment segment_of(int t) {
    return t == kPrimary ? RefSegment::Primary
         : t == kCompanion ? RefSegment::Companion
                           : RefSegment::World;
}
bool is_body(int t) { return t == kPrimary || t == kCompanion; }

// Per-body classifier metrics for sample `i`, including the window statistics
// (E1-E2) and the raw bound/dominant/winding/ratio test inputs (E3).
BodyMetrics body_metrics(const std::vector<TimedTrajectorySample>& samples,
                         int i, int body, const BinarySystem& bin,
                         const ClassifierParams& p, double* available_frac) {
    const TimedTrajectorySample& s = samples[i];
    const double t = s.time;

    const Body& b = bin.body(body);
    const Vec2 bpos = bin.position(body, t);
    const Vec2 bvel = bin.velocity(body, t);

    const Vec2 r = s.position_world - bpos;
    const Vec2 vrel = s.velocity_world - bvel;
    double rho = std::hypot(r.x, r.y);
    if (rho < 1e-9) rho = 1e-9;

    // E3 raw tests: bound and dominant.
    const double v2 = vrel.x * vrel.x + vrel.y * vrel.y;
    BodyMetrics m;
    m.epsilon = 0.5 * v2 - b.mu / rho;
    const double a_self = b.mu / (rho * rho);
    // Tidal = the other body's field difference between the ship and body's
    // centre (the relative acceleration that a one-body orbit cannot absorb).
    const int other = 1 - body;
    const Vec2 g_ship = bin.gravity_from(other, s.position_world, t);
    const Vec2 g_body = bin.gravity_from(other, bpos, t);
    const double a_tidal = std::hypot(g_ship.x - g_body.x, g_ship.y - g_body.y);
    m.dominance = a_self / (a_tidal > 1e-9 ? a_tidal : 1e-9);

    // E1: analysis window W from the local (Keplerian) period at this sample.
    const double T_local = kTwoPi * std::sqrt((rho * rho * rho) / (b.mu > 1e-9 ? b.mu : 1e-9));
    const double W = clampd(p.window_scale * T_local, p.min_window, p.max_window);

    // E2: window statistics over [t, t + W], using samples available in the arc.
    const double t_end = t + W;
    const double T_end_arc = samples.back().time;
    *available_frac = clampd((T_end_arc - t) / (W > 1e-9 ? W : 1e-9), 0.0, 1.0);

    double prev_theta = 0.0;
    double net = 0.0;
    double sum_abs = 0.0;
    double rho_min = 1e300;
    double rho_max = 0.0;
    bool have_theta = false;
    for (size_t j = i; j < samples.size(); ++j) {
        if (samples[j].time > t_end) break;
        const Vec2 bp = bin.position(body, samples[j].time);
        const double rx = samples[j].position_world.x - bp.x;
        const double ry = samples[j].position_world.y - bp.y;
        const double rj = std::hypot(rx, ry);
        if (rj < rho_min) rho_min = rj;
        if (rj > rho_max) rho_max = rj;
        const double th = std::atan2(ry, rx);
        if (!have_theta) {
            prev_theta = th;
            have_theta = true;
        } else {
            const double d = wrap_pi(th - prev_theta);
            net += d;
            sum_abs += std::abs(d);
            prev_theta = th;
        }
    }

    m.delta_theta = net;
    m.angular_consistency = sum_abs > 1e-9 ? std::abs(net) / sum_abs : 0.0;
    m.radial_ratio = rho_min > 1e-9 ? rho_max / rho_min : 0.0;
    return m;
}

// E4: among the bodies that are raw candidates, pick the winner by (dominance,
// |delta_theta|), breaking ties toward the previous committed state, then body 0.
int pick_winner(const BodyMetrics* m, const bool* cand, int state_body) {
    int best = -1;
    for (int b = 0; b < 2; ++b) {
        if (!cand[b]) continue;
        if (best < 0) {
            best = b;
            continue;
        }
        const bool close_d = std::abs(m[b].dominance - m[best].dominance) < 1e-9;
        const bool close_t = std::abs(m[b].delta_theta - m[best].delta_theta) < 1e-9;
        bool take_b = false;
        if (m[b].dominance > m[best].dominance) take_b = true;
        else if (close_d && m[b].delta_theta > m[best].delta_theta) take_b = true;
        else if (close_d && close_t) {
            // Tie: prefer the previous committed state, then the lower index.
            take_b = (b == state_body) && (best != state_body);
        }
        if (take_b) best = b;
    }
    return best;
}

}  // namespace

const char* pred_frame_name(PredFrame frame) {
    switch (frame) {
        case PredFrame::Primary: return "PRIMARY";
        case PredFrame::Companion: return "COMPANION";
        case PredFrame::Auto: return "AUTO";
        case PredFrame::World: default: return "WORLD";
    }
}

const char* ref_segment_name(RefSegment s) {
    switch (s) {
        case RefSegment::Primary: return "PRIMARY";
        case RefSegment::Companion: return "COMPANION";
        case RefSegment::World: default: return "WORLD";
    }
}

int ref_segment_body(RefSegment s) {
    return s == RefSegment::Primary ? 0 : s == RefSegment::Companion ? 1 : -1;
}

FrameSample transform_to_frame(const TimedTrajectorySample& s,
                               const BinarySystem& bin, PredFrame frame) {
    // Auto is a label, not a frame: the caller resolves it to a segment first.
    // Treat it as World (identity) here.
    if (frame == PredFrame::Auto) frame = PredFrame::World;
    if (frame == PredFrame::World) {
        return FrameSample{s.position_world, s.velocity_world, s.time};
    }
    const int body = (frame == PredFrame::Primary) ? 0 : 1;
    const Vec2 bpos = bin.position(body, s.time);
    const Vec2 bvel = bin.velocity(body, s.time);
    return FrameSample{s.position_world - bpos, s.velocity_world - bvel, s.time};
}

void classify_auto(const std::vector<TimedTrajectorySample>& samples,
                   const BinarySystem& bin, const ClassifierParams& p,
                   RefSegment prev_state, AutoClassifyResult& out) {
    const int n = (int)samples.size();
    out.segments.assign(n, RefSegment::World);
    out.metrics.resize(n);
    if (n == 0) return;

    int state = target_of(prev_state);
    int pending = kHold;
    int pending_count = 0;
    double pending_since = -1e300;

    for (int i = 0; i < n; ++i) {
        double avail[2];
        const BodyMetrics m0 = body_metrics(samples, i, 0, bin, p, &avail[0]);
        const BodyMetrics m1 = body_metrics(samples, i, 1, bin, p, &avail[1]);
        BodyMetrics m[2] = {m0, m1};

        // E3 raw candidate per body (also requires the window to be usable).
        bool cand[2] = {false, false};
        for (int b = 0; b < 2; ++b) {
            cand[b] = avail[b] >= p.min_available_frac &&
                      m[b].epsilon < 0.0 &&
                      m[b].dominance >= p.dominance_min &&
                      std::abs(m[b].delta_theta) >= p.delta_theta_min &&
                      m[b].angular_consistency >= p.angular_consistency_min &&
                      m[b].radial_ratio <= p.radial_ratio_max;
        }

        const bool window_ok = avail[0] >= p.min_available_frac ||
                               avail[1] >= p.min_available_frac;

        SampleMetrics& rec = out.metrics[i];
        rec.body[0] = m[0];
        rec.body[1] = m[1];
        rec.window_ok = window_ok;

        int target = kHold;
        if (window_ok) {
            const int winner = pick_winner(m, cand, state == kHold ? -1
                                                                  : (is_body(state) ? state : -1));
            if (winner >= 0) {
                rec.raw = segment_of(winner);
                target = winner;
            } else {
                rec.raw = RefSegment::World;
                target = kWorld;
            }
        } else {
            rec.raw = RefSegment::World;
            target = kHold;
        }

        // E5 hysteresis.
        if (target != kHold) {
            if (target != state) {
                if (pending != target) {
                    pending = target;
                    pending_count = 1;
                    pending_since = samples[i].time;
                } else {
                    pending_count += 1;
                }
                if (pending_count >= p.confirm_count &&
                    (samples[i].time - pending_since) >= p.confirm_seconds) {
                    if (is_body(target) && is_body(state)) {
                        // Never a direct body->body: release to World first,
                        // then keep accumulating toward the target for the
                        // second (World->body) leg.
                        state = kWorld;
                        pending_count = 0;
                        pending_since = samples[i].time;
                        // pending stays == target (a body) for the second leg.
                    } else {
                        state = target;
                        pending = kHold;
                        pending_count = 0;
                        pending_since = -1e300;
                    }
                }
            } else {
                // Arrived at (or staying at) the current segment.
                pending = kHold;
                pending_count = 0;
                pending_since = -1e300;
            }
        }
        // target == kHold: freeze state and pending.
        out.segments[i] = segment_of(state);
    }
}

}  // namespace lander
