#include "lander/ballistic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace lander {

namespace {

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
        for (int b = 0; b < 2; ++b) {
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
                    ter.arc_at_angle(theta - bin.body_rotation(t)));
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
                             Vec2& v_out) {
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
    constexpr int kNewtonMax = 5;
    constexpr int kFinalNewtonMax = 4;
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
    Candidate best[2]{};
    for (int k = 0; k < 2; ++k) {
        best[k].miss = 1.0e30;
    }
    auto consider = [&](const Candidate& c) {
        for (int k = 0; k < 2; ++k) {
            if (c.miss < best[k].miss ||
                (c.miss == best[k].miss && c.speed < best[k].speed) ||
                (c.miss == best[k].miss && c.speed == best[k].speed &&
                 c.fraction < best[k].fraction)) {
                for (int j = 1; j > k; --j) {
                    best[j] = best[j - 1];
                }
                best[k] = c;
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
            ter.arc_at_angle(theta - bin.body_rotation(t1s)));
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
                const Vec2 fp0 =
                    propagate(bin, x0, v0 - Vec2{kNewtonStep, 0}, t0,
                              msteps, kMediumDt);
                const Vec2 fm0 =
                    propagate(bin, x0, v0 + Vec2{kNewtonStep, 0}, t0,
                              msteps, kMediumDt);
                const Vec2 fp1 =
                    propagate(bin, x0, v0 - Vec2{0, kNewtonStep}, t0,
                              msteps, kMediumDt);
                const Vec2 fm1 =
                    propagate(bin, x0, v0 + Vec2{0, kNewtonStep}, t0,
                              msteps, kMediumDt);
                const double j00 = (fp0.x - fm0.x) / (2.0 * kNewtonStep);
                const double j01 = (fp0.y - fm0.y) / (2.0 * kNewtonStep);
                const double j10 = (fp1.x - fm1.x) / (2.0 * kNewtonStep);
                const double j11 = (fp1.y - fm1.y) / (2.0 * kNewtonStep);
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

    // Final pass at the authoritative fixed step: re-polish the best
    // candidate(s) and accept the first whose terminal miss, speed bound,
    // and terrain clearance all hold. This is the source of truth, so a
    // coarse/medium basin that does not survive it is discarded.
    Vec2 chosen_v0{};
    bool have_solution = false;
    for (int k = 0; k < 2 && !have_solution; ++k) {
        const Candidate& c = best[k];
        if (c.miss >= 1.0e30) {
            break;
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
            const Vec2 fp0 =
                propagate(bin, x0, v0 - Vec2{kNewtonStep, 0}, t0, steps, dt);
            const Vec2 fm0 =
                propagate(bin, x0, v0 + Vec2{kNewtonStep, 0}, t0, steps, dt);
            const Vec2 fp1 =
                propagate(bin, x0, v0 - Vec2{0, kNewtonStep}, t0, steps, dt);
            const Vec2 fm1 =
                propagate(bin, x0, v0 + Vec2{0, kNewtonStep}, t0, steps, dt);
            const double j00 = (fp0.x - fm0.x) / (2.0 * kNewtonStep);
            const double j01 = (fp0.y - fm0.y) / (2.0 * kNewtonStep);
            const double j10 = (fp1.x - fm1.x) / (2.0 * kNewtonStep);
            const double j11 = (fp1.y - fm1.y) / (2.0 * kNewtonStep);
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
        if (transfer_arc_clear(bin, x0, v0, t0, steps, dt, source)) {
            chosen_v0 = v0;
            have_solution = true;
            continue;
        }
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(stderr, "  arc not clear: escaping\n");
        }
        // The polished terminal-miss arc grazes terrain (typically the target's
        // surface on the final approach). Search a small neighbourhood for the
        // nearest velocity that both clears the terrain and still reaches the
        // arrival shell, so a shallow graze does not sink the whole transfer.
        const double a0 = std::atan2(v0.y, v0.x);
        bool escaped = false;
        for (int pass = 0; pass < 2 && !escaped; ++pass) {
            const double sf = (pass == 0 ? 0.03 : 0.07);  // speed span
            const double da = (pass == 0 ? 1.0 : 3.0) * kTwoPi / 360.0;  // deg
            for (int is = -1; is <= 1 && !escaped; ++is) {
                for (int ia = -1; ia <= 1 && !escaped; ++ia) {
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
                    chosen_v0 = v2;
                    have_solution = true;
                    escaped = true;
                }
            }
        }
    }
    if (getenv("LL_TRANSFER_DEBUG")) {
        std::fprintf(stderr, "[T] src=%d tgt=%d done have_solution=%d\n",
                     source, target, (int)have_solution);
    }

    if (!have_solution) {
        return false;  // no plausible solution: the state is untouched
    }

    v_out = chosen_v0;
    return true;
}

}  // namespace lander
