#include "lander/ballistic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace lander {

namespace {

// M06-R5: a zero-thrust-propagation counter for the transfer solvers. `propagate`
// is the multi-step integrator both the cold solver and the warm differential
// correction use, so this counts full trajectory propagations. It is a
// single-threaded telemetry hook (the game and the tests invoke a solve one at
// a time); it never affects the solver's result, only its measurement.
static int g_propagation_count = 0;

// M05-R3-16: the transfer solver's clearance shell above a surface.
constexpr double kTransferClearance = 15.0;

// M05-R3-16: how far outside the actual terrain the transfer arc must
// stay at each step, relative to the collision test the simulation itself
// runs. The game re-checks terrain one fixed step after the solver's
// sample, and the bodies rotate in between, so the margin keeps the
// solver's guarantee valid against the game's test.
constexpr double kTransferTerrainMargin = 1.0;

// M05-R3-16: propagate (p, v) forward by exactly `steps` fixed steps of
// size `dt` with the same semi-implicit Euler scheme and the same
// two-body inverse-square field as Simulation::integrate_flight (gravity
// sampled at each step's start time), without thrust, spin, fuel, or
// collision. Pure: the inputs are unchanged.
Vec2 propagate(const BinarySystem& bin, const Vec2& p0, const Vec2& v0,
                double t0, int steps, double dt) {
    ++g_propagation_count;
    Vec2 p = p0;
    Vec2 v = v0;
    double t = t0;
    for (int i = 0; i < steps; ++i) {
        const Vec2 a = bin.gravity(p, t);
        v = v + a * dt;
        p = p + v * dt;
        t += dt;
    }
    return p;
}

// M05-R3-16: whether the unthrust arc from (p0, v0) stays outside both
// bodies' actual terrain at every fixed step. This mirrors the collision
// test the simulation applies each step (tidal-lock rotation included),
// with a margin. Pure: no state is mutated.
bool transfer_arc_clear(const BinarySystem& bin, const Vec2& p0,
                         const Vec2& v0, double t0, int steps, double dt,
                         int source) {
    Vec2 p = p0;
    Vec2 v = v0;
    double t = t0;
    for (int i = 0; i < steps; ++i) {
        const Vec2 a = bin.gravity(p, t);
        v = v + a * dt;
        p = p + v * dt;
        t += dt;
        for (int b = 0; b < 3; ++b) {
            const Vec2 bp = bin.position(b, t);
            const double rx = p.x - bp.x;
            const double ry = p.y - bp.y;
            const double rho = std::hypot(rx, ry);
            if (rho < 1.0e-9) {
                return false;
            }
            const double theta = std::atan2(ry, rx);
            const Terrain& ter = bin.body(b).terrain;
            const double surface =
                ter.surface_radius_at_arc(
                    ter.arc_at_angle(theta - bin.body_rotation(b, t)));
            if (b == source) {
                // The craft departs from this body's exact surface point (no
                // clearance shell, M05-R3-20). The game's contact test crashes
                // on any below-surface reading, so the departure must be
                // outward enough to stay above the local terrain at every step
                // from the first one on.
                if (rho < surface) {
                    if (getenv("LL_TRANSFER_DEBUG")) {
                        std::fprintf(
                            stderr,
                            "  arc-clear REJECT src b=%d i=%d rho=%.3f "
                            "surf=%.3f\n",
                            b, i, rho, surface);
                    }
                    return false;
                }
            } else {
                // The other body: keep the conservative clearance margin so
                // the approach never grazes its terrain.
                if (rho - surface < kTransferTerrainMargin) {
                    if (getenv("LL_TRANSFER_DEBUG")) {
                        std::fprintf(
                            stderr,
                            "  arc-clear REJECT oth b=%d i=%d rho=%.3f "
                            "surf=%.3f margin=%.3f\n",
                            b, i, rho, surface, rho - surface);
                    }
                    return false;
                }
            }
        }
    }
    return true;
}

}  // namespace

int ballistic_steps(double t0, double t1, double dt) {
    if (dt <= 0.0 || t1 <= t0) {
        return 0;
    }
    return static_cast<int>(std::llround((t1 - t0) / dt));
}

BallisticState step_ballistic(const BinarySystem& bin,
                              const BallisticState& state, double dt) {
    const Vec2 a = bin.gravity(state.p, state.t);
    BallisticState out = state;
    out.v = state.v + a * dt;
    out.p = state.p + out.v * dt;
    out.t = state.t + dt;
    return out;
}

BallisticState propagate_ballistic(const BinarySystem& bin,
                                   const BallisticState& state, int steps,
                                   double dt) {
    BallisticState out = state;
    for (int i = 0; i < steps; ++i) {
        out = step_ballistic(bin, out, dt);
    }
    return out;
}

std::vector<BallisticState> predict_zero_thrust(const BinarySystem& bin,
                                                const BallisticState& start,
                                                double end_t, double dt,
                                                int target_samples) {
    std::vector<BallisticState> out;
    const int total = ballistic_steps(start.t, end_t, dt);
    out.push_back(start);
    if (total <= 0 || dt <= 0.0) {
        return out;
    }
    const int wanted = std::max(1, target_samples);
    const int sample_every = std::max(1, total / wanted);
    BallisticState current = start;
    for (int i = 0; i < total; ++i) {
        current = step_ballistic(bin, current, dt);
        if (i == total - 1 || i % sample_every == 0) {
            out.push_back(current);
        }
    }
    return out;
}

bool solve_transfer_velocity(const BinarySystem& bin, double dt,
                              const Vec2& x0, int source, int target, double t0,
                              Vec2& v_out, TransferSolution* out_solution) {
    if (dt <= 0.0) {
        return false;
    }
    const Body& tgt = bin.body(target);

    // M05-R3-19: staged deterministic solver with fixed work bounds,
    // independent of player input and of the wall clock. The binary is
    // analytic, so a coarse integration step locates the most promising
    // terminal-miss basin, a medium step polishes each basin (polar scan +
    // damped Newton), and a final pass at the authoritative fixed step
    // re-polishes the best candidates and validates the miss, the speed
    // bound, and terrain clearance before any state is changed.
    constexpr double kFlightFractions[] = {0.15, 0.20, 0.25, 0.30, 0.40,
                                           0.50};
    constexpr double kCoarseDt = 0.5;
    // Medium integration step for the refine stage: coarse enough to keep each
    // propagation cheap, fine enough that the converged velocity lands within
    // ~a metre of the authoritative minimum, which the final full-step Newton
    // then polishes. Using the full 1/120 step here would make the refine (the
    // most propagations) dominate the solve time.
    constexpr double kMediumDt = 0.1;
    constexpr int kCoarseSpeeds = 13;  // 10..34 m/s by 2
    constexpr double kCoarseSpeedMin = 10.0;
    constexpr double kCoarseSpeedStep = 2.0;
    constexpr int kCoarseDirs = 24;  // 15-degree steps
    constexpr int kTopBasins = 3;
    constexpr double kCoarseAccept = 300.0;
    constexpr int kRefineSpeedRadius = 2;  // +/-2 m/s by 1
    constexpr int kRefineAngRadius = 3;    // +/-15 degrees by 5
    // Newton-iteration caps. Characterized over the M06-R13 48-phase 0->1
    // sweep (plus the full transfer test suite): (2,2) reproduces the exact
    // accepted solution quality of the original (5,4) -- identical 5.88 m
    // max achieved miss, all arcs terrain-clear -- while shaving the
    // redundant over-polish iterations, cutting ~3% of propagations and
    // ~23% of worst-case solve wall time. Lowering further (1,1) thins the
    // margin to 6.53 m and (0,0) stops converging, so (2,2) is the fastest
    // pair that keeps the current quality without trading correctness.
    constexpr int kNewtonMax = 2;
    constexpr int kFinalNewtonMax = 2;
    constexpr double kNewtonStep = 0.25;
    constexpr double kMaxSpeed = 60.0;
    // Arrival-shell proximity tolerance (m). Re-tuned upward from 5.0 because
    // the velocity-only arc now originates at the surface (0 clearance) rather
    // than the old 15 m departure shell, which shifts the outward basin's
    // minimum to ~5.8 m. The hard safety gate remains transfer_arc_clear's
    // 1 m terrain margin over every step, so a looser proximity is safe: any
    // arc that clips terrain is rejected there regardless of this bound.
    constexpr double kAcceptMiss = 8.0;

    struct Candidate {
        double miss{};
        double speed{};
        double fraction{};
        Vec2 v0{};
    };
    // M06-R6 lower-energy handoff: keep one bounded candidate per flight-time
    // fraction (the existing bounded candidate set) instead of the global
    // top-2 by miss, so the final pass can weigh every flight time and choose
    // the lowest-energy valid arrival rather than the first miss to clear.
    constexpr int kFracs = static_cast<int>(
        sizeof(kFlightFractions) / sizeof(kFlightFractions[0]));
    Candidate best[kFracs]{};
    for (int k = 0; k < kFracs; ++k) {
        best[k].miss = 1.0e30;
    }
    auto consider = [&](const Candidate& c) {
        for (int k = 0; k < kFracs; ++k) {
            if (std::fabs(kFlightFractions[k] - c.fraction) < 1.0e-9) {
                if (c.miss < best[k].miss) {
                    best[k] = c;
                }
                return;
            }
        }
    };
    // Arrival shell for a candidate flight time: a small clearance shell
    // above the target's surface on the side the departure is coming from,
    // so the final approach stays outside the terrain.
    auto goal_at = [&](double t1, Vec2& goal_out) {
        const Vec2 s1 = bin.position(source, t1);
        const Vec2 g1 = bin.position(target, t1);
        const double dg = std::hypot(s1.x - g1.x, s1.y - g1.y);
        if (dg < 1.0e-9) {
            return false;
        }
        const Vec2 approach{(s1.x - g1.x) / dg, (s1.y - g1.y) / dg};
        const double r_arr =
            tgt.terrain.max_surface_radius() + kTransferClearance;
        goal_out = {g1.x + approach.x * r_arr, g1.y + approach.y * r_arr};
        return true;
    };
    // M05-R3-20: penalty (per metre) for a candidate's FIRST-step dip back
    // into the departure body's terrain, added to the Newton's terminal
    // objective. The game's contact test crashes on any below-surface reading,
    // so a velocity that reaches the target but initially falls into the body
    // is a crash; this steers the polish toward an outward departure. It is a
    // cheap single-step objective hint only; transfer_arc_clear is the
    // authoritative full-arc clearance gate (it re-checks every step).
    constexpr double kDivePenalty = 100.0;
    auto first_step_dive = [&](const Vec2& v) {
        const Vec2 a0 = bin.gravity(x0, t0);
        const Vec2 v1 = v + a0 * dt;
        const Vec2 p1 = x0 + v1 * dt;
        const double t1s = t0 + dt;
        const Vec2 sp = bin.position(source, t1s);
        const double rx = p1.x - sp.x;
        const double ry = p1.y - sp.y;
        const double rho1 = std::hypot(rx, ry);
        if (rho1 < 1.0e-9) {
            return 1.0e30;
        }
        const double theta = std::atan2(ry, rx);
        const Terrain& ter = bin.body(source).terrain;
        const double surface = ter.surface_radius_at_arc(
            ter.arc_at_angle(theta - bin.body_rotation(source, t1s)));
        return std::max(0.0, surface - rho1);
    };

    for (double fraction : kFlightFractions) {
        const int steps =
            static_cast<int>(std::lround(fraction * bin.period() / dt));
        if (steps < 10) {
            continue;
        }
        const double t1 = t0 + steps * dt;
        Vec2 x_goal{};
        if (!goal_at(t1, x_goal)) {
            continue;
        }
        // Medium-step count for the refine stage (see kMediumDt).
        const int msteps =
            std::max(8, static_cast<int>(std::lround((t1 - t0) / kMediumDt)));

        // Coarse stage: a coarse integration step makes each propagation
        // cheap; the terminal miss is used only to locate the basins.
        const int csteps =
            std::max(8, static_cast<int>(std::lround((t1 - t0) / kCoarseDt)));
        auto miss_c = [&](const Vec2& v) {
            const Vec2 f = propagate(bin, x0, v, t0, csteps, kCoarseDt);
            const double m = std::hypot(f.x - x_goal.x, f.y - x_goal.y);
            return std::isfinite(m) ? m : 1.0e30;
        };
        struct Coarse {
            double miss{};
            double speed{};
            double ang{};
        };
        Coarse top[kTopBasins]{};
        for (int k = 0; k < kTopBasins; ++k) {
            top[k].miss = 1.0e30;
        }
        for (int si = 0; si < kCoarseSpeeds; ++si) {
            const double speed = kCoarseSpeedMin + kCoarseSpeedStep * si;
            for (int di = 0; di < kCoarseDirs; ++di) {
                const double ang = kTwoPi * di / kCoarseDirs;
                const double m =
                    miss_c(Vec2{speed * std::cos(ang), speed * std::sin(ang)});
                for (int k = 0; k < kTopBasins; ++k) {
                    if (m < top[k].miss) {
                        for (int j = kTopBasins - 1; j > k; --j) {
                            top[j] = top[j - 1];
                        }
                        top[k] = {m, speed, ang};
                        break;
                    }
                }
            }
        }
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(stderr, "[T] frac=%.2f coarse_top=%.3f\n", fraction,
                         top[0].miss);
        }
        if (top[0].miss > kCoarseAccept) {
            continue;  // no basin at this flight time
        }

        // Refine stage: polish each promising basin (polar scan + damped
        // Newton) at the authoritative fixed step, so the converged
        // velocity is already valid against the simulation's own
        // integration. Only the top basins per flight time are refined,
        // which bounds the work.
        for (int k = 0; k < kTopBasins; ++k) {
            if (top[k].miss > 1000.0) {
                continue;  // no refine window can reach such a candidate
            }
            double m = top[k].miss;
            Vec2 v0{top[k].speed * std::cos(top[k].ang),
                    top[k].speed * std::sin(top[k].ang)};
            auto miss_m = [&](const Vec2& v) {
                const Vec2 f =
                    propagate(bin, x0, v, t0, msteps, kMediumDt);
                double mm = std::hypot(f.x - x_goal.x, f.y - x_goal.y);
                mm += kDivePenalty * first_step_dive(v);
                return std::isfinite(mm) ? mm : 1.0e30;
            };
            for (int si = -kRefineSpeedRadius; si <= kRefineSpeedRadius;
                 ++si) {
                const double speed = top[k].speed + 1.0 * si;
                if (speed < 1.0e-9) {
                    continue;
                }
                for (int ai = -kRefineAngRadius; ai <= kRefineAngRadius;
                     ++ai) {
                    const double ang = top[k].ang + kTwoPi * ai / 36.0;
                    const Vec2 v{speed * std::cos(ang), speed * std::sin(ang)};
                    const double mm = miss_m(v);
                    if (mm < m) {
                        m = mm;
                        v0 = v;
                    }
                }
            }
            // Damped Newton polish on the terminal miss: central-difference
            // 2x2 Jacobian, with the step halved whenever it fails to
             // improve the miss.
            for (int iter = 0; iter < kNewtonMax && m >= kAcceptMiss;
                  ++iter) {
                const Vec2 fpx =
                    propagate(bin, x0, v0 + Vec2{kNewtonStep, 0}, t0,
                              msteps, kMediumDt);
                const Vec2 fmx =
                    propagate(bin, x0, v0 - Vec2{kNewtonStep, 0}, t0,
                              msteps, kMediumDt);
                const Vec2 fpy =
                    propagate(bin, x0, v0 + Vec2{0, kNewtonStep}, t0,
                              msteps, kMediumDt);
                const Vec2 fmy =
                    propagate(bin, x0, v0 - Vec2{0, kNewtonStep}, t0,
                              msteps, kMediumDt);
                const double j00 = (fpx.x - fmx.x) / (2.0 * kNewtonStep);
                const double j10 = (fpx.y - fmx.y) / (2.0 * kNewtonStep);
                const double j01 = (fpy.x - fmy.x) / (2.0 * kNewtonStep);
                const double j11 = (fpy.y - fmy.y) / (2.0 * kNewtonStep);
                const double det = j00 * j11 - j01 * j10;
                if (std::abs(det) < 1.0e-9) {
                    break;  // singular or degenerate Jacobian
                }
                const Vec2 f =
                    propagate(bin, x0, v0, t0, msteps, kMediumDt);
                const double fx = f.x - x_goal.x;
                const double fy = f.y - x_goal.y;
                double dvx = (-fx * j11 + fy * j01) / det;
                double dvy = (-fy * j00 + fx * j10) / det;
                bool improved = false;
                for (int damp = 0; damp < 6; ++damp) {
                    const Vec2 trial{v0.x + dvx, v0.y + dvy};
                    const double mt = miss_m(trial);
                    if (mt < m) {
                        v0 = trial;
                        m = mt;
                        improved = true;
                        break;
                    }
                    dvx *= 0.5;
                    dvy *= 0.5;
                }
                if (!improved) {
                    break;
                }
            }
            const double speed = std::hypot(v0.x, v0.y);
            if (getenv("LL_TRANSFER_DEBUG")) {
                const Vec2 fdbg = propagate(bin, x0, v0, t0, steps, dt);
                const double tterm =
                    std::hypot(fdbg.x - x_goal.x, fdbg.y - x_goal.y);
                const Vec2 spc = bin.position(source, t0);
                double ux = x0.x - spc.x, uy = x0.y - spc.y;
                const double ul = std::hypot(ux, uy);
                ux /= ul;
                uy /= ul;
                std::fprintf(
                    stderr,
                    "  [refine] frac=%.2f speed=%.3f term=%.3f "
                    "dive=%.4f radial=%.3f m_pen=%.3f\n",
                    fraction, speed, tterm, first_step_dive(v0),
                    v0.x * ux + v0.y * uy, m);
            }
            if (m >= kAcceptMiss || speed < 1.0e-9 || speed > kMaxSpeed) {
                continue;  // not plausible: reject, never teleport
            }
            consider(Candidate{m, speed, fraction, v0});
        }
    }

    if (best[0].miss >= 1.0e30) {
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(
                stderr,
                 "[T] src=%d tgt=%d done have_solution=0 (no basin)\n",
                  source, target);
         }
        return false;  // no plausible solution: the state is untouched
    }

    // Final pass at the authoritative fixed step: re-polish each bounded
    // flight-time candidate, apply the hard gates (miss, speed bound, terrain
    // clearance), and choose among the survivors. M06-R6 lower-energy
    // handoff: rather than accepting the first candidate whose miss clears,
    // rank the survivors by predicted target-relative arrival speed and accept
    // the lowest, so the terminal de-orbit has the least approach energy to
    // bleed off. A coarse/medium basin that does not survive the
    // authoritative step is discarded. This is the source of truth.
    struct Valid {
        Candidate cand{};
        Vec2 v0{};  // clearing departure velocity
        int steps{0};
        double t1{0.0};
        Vec2 goal{};
        double arrival_rel{0.0};
    };
    Valid valids[kFracs]{};
    int n_valid = 0;
    for (int k = 0; k < kFracs; ++k) {
        const Candidate& c = best[k];
        if (c.miss >= 1.0e30) {
            continue;  // no candidate at this flight time
        }
        const int steps =
            static_cast<int>(std::lround(c.fraction * bin.period() / dt));
        const double t1 = t0 + steps * dt;
        Vec2 x_goal{};
        if (!goal_at(t1, x_goal)) {
            continue;
        }
        auto miss_f = [&](const Vec2& v) {
            const Vec2 f = propagate(bin, x0, v, t0, steps, dt);
            double mm = std::hypot(f.x - x_goal.x, f.y - x_goal.y);
            mm += kDivePenalty * first_step_dive(v);
            return std::isfinite(mm) ? mm : 1.0e30;
        };
        // Cheap terminal-only miss, used to pre-filter escape-search candidates
        // before paying for a full terrain clearance check.
        auto term_f = [&](const Vec2& v) {
            const Vec2 f = propagate(bin, x0, v, t0, steps, dt);
            return std::hypot(f.x - x_goal.x, f.y - x_goal.y);
        };
        Vec2 v0 = c.v0;
        double m = miss_f(v0);
        for (int iter = 0; iter < kFinalNewtonMax && m >= kAcceptMiss;
             ++iter) {
            const Vec2 fpx =
                propagate(bin, x0, v0 + Vec2{kNewtonStep, 0}, t0, steps, dt);
            const Vec2 fmx =
                propagate(bin, x0, v0 - Vec2{kNewtonStep, 0}, t0, steps, dt);
            const Vec2 fpy =
                propagate(bin, x0, v0 + Vec2{0, kNewtonStep}, t0, steps, dt);
            const Vec2 fmy =
                propagate(bin, x0, v0 - Vec2{0, kNewtonStep}, t0, steps, dt);
            const double j00 = (fpx.x - fmx.x) / (2.0 * kNewtonStep);
            const double j10 = (fpx.y - fmx.y) / (2.0 * kNewtonStep);
            const double j01 = (fpy.x - fmy.x) / (2.0 * kNewtonStep);
            const double j11 = (fpy.y - fmy.y) / (2.0 * kNewtonStep);
            const double det = j00 * j11 - j01 * j10;
            if (std::abs(det) < 1.0e-9) {
                break;  // singular or degenerate Jacobian
            }
            const Vec2 f = propagate(bin, x0, v0, t0, steps, dt);
            const double fx = f.x - x_goal.x;
            const double fy = f.y - x_goal.y;
            double dvx = (-fx * j11 + fy * j01) / det;
            double dvy = (-fy * j00 + fx * j10) / det;
            bool improved = false;
            for (int damp = 0; damp < 6; ++damp) {
                const Vec2 trial{v0.x + dvx, v0.y + dvy};
                const double mt = miss_f(trial);
                if (mt < m) {
                    v0 = trial;
                    m = mt;
                    improved = true;
                    break;
                }
                dvx *= 0.5;
                dvy *= 0.5;
            }
            if (!improved) {
                break;
            }
        }
        const double speed = std::hypot(v0.x, v0.y);
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(stderr, "[T] final frac=%.2f m=%.3f speed=%.3f\n",
                         c.fraction, m, speed);
        }
        if (m >= kAcceptMiss || speed < 1.0e-9 || speed > kMaxSpeed) {
            if (getenv("LL_TRANSFER_DEBUG")) {
                std::fprintf(stderr, "  reject: miss/speed\n");
            }
            continue;
        }
        // The arc must stay outside both bodies' actual terrain at every
        // flight step, by the same test the simulation applies (tidal
        // rotation included). This forces a landed craft to depart outward
        // enough to clear its own surface, and keeps the final approach from
        // grazing the target's terrain.
        // Determine the clearing departure velocity for this flight time: the
        // polished arc, or a small neighbourhood search if it grazes terrain
         // (typically the target's surface on the final approach).
        Vec2 clearing = v0;
        bool clearing_ok =
            transfer_arc_clear(bin, x0, clearing, t0, steps, dt, source);
        if (!clearing_ok) {
            const double a0 = std::atan2(v0.y, v0.x);
            for (int pass = 0; pass < 2 && !clearing_ok; ++pass) {
                const double sf = (pass == 0 ? 0.03 : 0.07);  // speed span
                const double da =
                    (pass == 0 ? 1.0 : 3.0) * kTwoPi / 360.0;  // deg
                for (int is = -1; is <= 1 && !clearing_ok; ++is) {
                    for (int ia = -1; ia <= 1 && !clearing_ok; ++ia) {
                        const double s = speed * (1.0 + is * sf);
                        if (s < 1.0e-9 || s > kMaxSpeed) {
                            continue;
                        }
                        const double ang = a0 + ia * da;
                        const Vec2 v2{s * std::cos(ang), s * std::sin(ang)};
                        if (term_f(v2) >= kAcceptMiss) {
                            continue;  // no longer reaches the arrival shell
                        }
                        if (!transfer_arc_clear(bin, x0, v2, t0, steps, dt,
                                                source)) {
                            continue;  // still grazes terrain
                        }
                        clearing = v2;
                        clearing_ok = true;
                    }
                }
            }
        }
        if (!clearing_ok) {
            if (getenv("LL_TRANSFER_DEBUG")) {
                std::fprintf(stderr,
                             "  [final] frac=%.2f arc not clear -> discard\n",
                             c.fraction);
            }
            continue;
        }
        // Predicted target-relative arrival speed for the clearing arc: the
        // endpoint velocity relative to the target body's inertial velocity at
        // the arrival epoch.
        const BallisticState fs = propagate_ballistic(
            bin, BallisticState{x0.x, x0.y, clearing.x, clearing.y, t0},
            steps, dt);
        const Vec2 tv = bin.velocity(target, fs.t);
        const double arrival_rel =
            std::hypot(fs.v.x - tv.x, fs.v.y - tv.y);
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(stderr,
                         "  [final] frac=%.2f miss=%.3f TOF=%.3f dep=%.3f "
                         "arrival_rel=%.3f\n",
                         c.fraction, m, t1 - t0, speed, arrival_rel);
        }
        valids[n_valid++] =
            Valid{c, clearing, steps, t1, x_goal, arrival_rel};
    }
    if (n_valid == 0) {
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(
                stderr,
                 "[T] src=%d tgt=%d done have_solution=0 (no clear arc)\n",
                  source, target);
         }
        return false;  // no plausible solution: the state is untouched
    }

    // M06-R6 lower-energy handoff: among the survivors prefer the lowest
    // predicted target-relative arrival speed; ties fall back to the lower
    // terminal miss, then the lower flight time, for determinism. A linear
    // O(K) scan over the bounded candidate set (a small fixed K) keeps the
    // work bounded, matching the existing search's complexity.
    int win_i = 0;
    for (int i = 1; i < n_valid; ++i) {
        if (valids[i].arrival_rel < valids[win_i].arrival_rel ||
            (valids[i].arrival_rel == valids[win_i].arrival_rel &&
             valids[i].cand.miss < valids[win_i].cand.miss) ||
            (valids[i].arrival_rel == valids[win_i].arrival_rel &&
             valids[i].cand.miss == valids[win_i].cand.miss &&
             valids[i].cand.fraction < valids[win_i].cand.fraction)) {
            win_i = i;
        }
    }
    const Valid& win = valids[win_i];
    if (getenv("LL_TRANSFER_DEBUG")) {
        // Report the old (first-valid-by-miss) choice vs the new (lowest
        // arrival speed) choice for the same case, plus the propagation cost.
        Valid old = win;
        for (int i = 0; i < n_valid; ++i) {
            if (valids[i].cand.miss < old.cand.miss) {
                old = valids[i];
            }
        }
        std::fprintf(
            stderr,
            "[T] src=%d tgt=%d done have_solution=1 | old(miss) frac=%.2f "
            "arr=%.3f | new(arrival) frac=%.2f arr=%.3f prop=%d\n",
            source, target, old.cand.fraction, old.arrival_rel,
            win.cand.fraction, win.arrival_rel,
            ballistic_propagation_count());
    }

    const Vec2 chosen_v0 = win.v0;
    v_out = chosen_v0;

    // M06-R5: optionally hand the accepted solution to the caller as a warm
    // cache entry, including the predicted target-relative arrival speed the
    // terminal de-orbit reads. The miss is recomputed at the authoritative
     // step from the chosen velocity against the accepted arrival shell.
    if (out_solution) {
        const Vec2 f = propagate(bin, x0, chosen_v0, t0, win.steps, dt);
        out_solution->valid = true;
        out_solution->source = source;
        out_solution->target = target;
        out_solution->solve_epoch = t0;
        out_solution->departure_state = x0;
        out_solution->departure_velocity = chosen_v0;
        out_solution->time_of_flight = win.t1 - t0;
        out_solution->arrival_epoch = win.t1;
        out_solution->achieved_miss =
            std::hypot(f.x - win.goal.x, f.y - win.goal.y);
        out_solution->fraction = win.cand.fraction;
        out_solution->arrival_rel_speed = win.arrival_rel;
    }
    return true;
}

NewtonCorrectionResult differential_correction(
    const BinarySystem& bin, double dt, const Vec2& x0, double t0, int steps,
    const Vec2& goal, Vec2 v0, int max_iters, double accept_miss,
    double newton_step) {
    NewtonCorrectionResult r{};
    r.v0 = v0;
    if (steps < 1) {
        return r;
    }
    // Terminal error F(v0) = propagate(x0, v0, t0, steps, dt) - goal, in norm.
    // The plain error (no cold-solver dive penalty): terrain clearance is the
    // caller's authoritative gate, so the correction need not chase it.
    auto miss = [&](const Vec2& v) {
        const Vec2 f = propagate(bin, x0, v, t0, steps, dt);
        const double m = std::hypot(f.x - goal.x, f.y - goal.y);
        return std::isfinite(m) ? m : 1.0e30;
    };
    double m = miss(v0);
    r.final_miss = m;
    if (m <= accept_miss) {
        r.converged = true;
        return r;  // the seed already meets the tolerance
    }
    constexpr double kDetFloor = 1.0e-9;  // deterministic singular-J threshold
    for (int iter = 0; iter < max_iters; ++iter) {
        r.iterations = iter + 1;
        // Central-difference 2x2 Jacobian J = dF/dv0.
        const double h = newton_step;
        const Vec2 fpx = propagate(bin, x0, v0 + Vec2{h, 0.0}, t0, steps, dt);
        const Vec2 fmx = propagate(bin, x0, v0 - Vec2{h, 0.0}, t0, steps, dt);
        const Vec2 fpy = propagate(bin, x0, v0 + Vec2{0.0, h}, t0, steps, dt);
        const Vec2 fmy = propagate(bin, x0, v0 - Vec2{0.0, h}, t0, steps, dt);
        const double j00 = (fpx.x - fmx.x) / (2.0 * h);
        const double j10 = (fpx.y - fmx.y) / (2.0 * h);
        const double j01 = (fpy.x - fmy.x) / (2.0 * h);
        const double j11 = (fpy.y - fmy.y) / (2.0 * h);
        const double det = j00 * j11 - j01 * j10;
        if (std::abs(det) < kDetFloor) {
            r.singular_jacobian = true;  // stop deterministically; caller falls back
            return r;
        }
        const Vec2 f = propagate(bin, x0, v0, t0, steps, dt);
        const double fx = f.x - goal.x;
        const double fy = f.y - goal.y;
        // Solve J·delta_v = -F by Cramer's rule.
        double dvx = (-fx * j11 + fy * j01) / det;
        double dvy = (-fy * j00 + fx * j10) / det;
        // Bounded damping / backtracking: lambda in {1, 1/2, 1/4, ...}.
        bool improved = false;
        for (int damp = 0; damp < 6; ++damp) {
            const Vec2 trial{v0.x + dvx, v0.y + dvy};
            const double mt = miss(trial);
            if (mt < m) {
                v0 = trial;
                m = mt;
                r.v0 = v0;
                r.final_miss = mt;
                improved = true;
                break;
            }
            dvx *= 0.5;
            dvy *= 0.5;
        }
        if (m <= accept_miss) {
            r.converged = true;
            return r;
        }
        if (!improved) {
            break;  // bounded: no progress, stop (never iterate to convergence)
        }
    }
    return r;
}

TransferSolution solve_transfer_warm(const BinarySystem& bin, double dt,
                                     const Vec2& x0, int source, int target,
                                     double t0, const TransferSolution& prev) {
    TransferSolution none{};  // valid=false
    if (dt <= 0.0 || !prev.valid ||
        prev.source != source || prev.target != target) {
        return none;
    }

    // The arrival shell for a candidate flight time: a clearance shell above
    // the target's surface on the side the departure is coming from (the same
    // geometry the cold solver uses, so warm and cold are directly comparable).
    const Body& tgt = bin.body(target);
    auto goal_at = [&](double t1, Vec2& goal_out) {
        const Vec2 s1 = bin.position(source, t1);
        const Vec2 g1 = bin.position(target, t1);
        const double dg = std::hypot(s1.x - g1.x, s1.y - g1.y);
        if (dg < 1.0e-9) {
            return false;
        }
        const Vec2 approach{(s1.x - g1.x) / dg, (s1.y - g1.y) / dg};
        const double r_arr =
            tgt.terrain.max_surface_radius() + kTransferClearance;
        goal_out = {g1.x + approach.x * r_arr, g1.y + approach.y * r_arr};
        return true;
    };

    // Shift the previous solution to the current epoch: keep the same flight
    // duration (snapped to the fixed-step grid) so the arrival shell moves with
    // the binary, and seed the departure velocity from the previous solution.
    const int steps =
        std::max(10, static_cast<int>(std::lround(prev.fraction * bin.period() / dt)));
    const double t1 = t0 + steps * dt;
    Vec2 x_goal{};
    if (!goal_at(t1, x_goal)) {
        return none;
    }
    Vec2 v0 = prev.departure_velocity;

    // Bounded differential correction (M06-R5-02) — no coarse grid (R5-03).
    constexpr int kWarmNewtonMax = 8;  // small fixed iteration maximum
    constexpr double kAcceptMiss = 8.0;  // matches the cold solver's bound
    constexpr double kNewtonStep = 0.25;  // matches the cold solver's step size
    constexpr double kMaxSpeed = 60.0;
    const NewtonCorrectionResult corr =
        differential_correction(bin, dt, x0, t0, steps, x_goal, v0,
                                kWarmNewtonMax, kAcceptMiss, kNewtonStep);
    v0 = corr.v0;
    const double speed = std::hypot(v0.x, v0.y);
    if (corr.final_miss >= kAcceptMiss || speed < 1.0e-9 || speed > kMaxSpeed) {
        return none;  // bounded failure -> caller falls back to a cold solve
    }

    // Authoritative validation (M06-R5-P04): the full 1/120 arc must clear
    // both bodies' terrain (the same gate the cold solver uses).
    if (!transfer_arc_clear(bin, x0, v0, t0, steps, dt, source)) {
        return none;
    }

    TransferSolution out{};
    out.valid = true;
    out.source = source;
    out.target = target;
    out.solve_epoch = t0;
    out.departure_state = x0;
    out.departure_velocity = v0;
    out.time_of_flight = steps * dt;
    out.arrival_epoch = t1;
    out.achieved_miss = corr.final_miss;
    out.fraction = prev.fraction;
    out.newton_iterations = corr.iterations;
    return out;
}

void ballistic_reset_propagation_count() {
    g_propagation_count = 0;
}

int ballistic_propagation_count() {
    return g_propagation_count;
}

}  // namespace lander
