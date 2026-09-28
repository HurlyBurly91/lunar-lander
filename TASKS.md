# Active Task

Milestone: M04
Request: M04-R1, M04-R2
State: COMPLETE

## Bootstrap note

The durable execution ledger was introduced after substantial M04
implementation had already occurred.

Do not fabricate historical request IDs for earlier work.

Earlier M04 implementation and verification will be summarized in the final
M04 record at closeout.

M04-R1 begins with the currently unresolved human-observed presentation issues.

## User requirements

- [x] M04-R1-01 Keep the player lander at an exact screen-space anchor through
  camera rotation, translation, and zoom.
  Source: USER
  Required invariant:
      screen_x(lander) = 0.5 * window_width
      screen_y(lander) = lander_top_fraction * window_height
  Files: include/lander/camera.hpp, src/gui.cpp
  Evidence: include/lander/camera.hpp::Camera::update sets focus exactly
  (target - up*framing_offset, no follow smoothing); the offset's 1/scale factor
  cancels to_screen's scale factor, so the anchor is exact at every angle and
  zoom. Final visual confirmation is tracked by M04-R1-H01.

- [x] M04-R1-02 Remove accumulated horizontal/local-tangent camera lag during
  orbit.
  Source: USER
  Notes: generic follow smoothing must not move the player away from its
  intended anchor.
  Files: include/lander/camera.hpp
  Evidence: Camera::update no longer smooths the focus through the rotating
  local frame (follow_rate is no longer applied to the player anchor), so a
  full revolution holds the exact anchor (see M04-R1-V01).

- [x] M04-R1-03 Add render interpolation between authoritative fixed simulation
  states.
  Source: USER
  Required basis:
      alpha = simulation_accumulator / fixed_dt
  Files: include/lander/sim.hpp, src/sim.cpp, src/gui.cpp
  Evidence: lander::interpolated_state(previous, current, alpha, snap_to_current)
  and the gui main loop compute alpha = accumulator / fixed_dt.

- [x] M04-R1-04 Interpolate render x/y and attitude consistently, including
  shortest-path angle interpolation across angular wrap.
  Source: USER
  Files: src/sim.cpp
  Evidence: interpolated_state lerps x/y and interpolates attitude with
  shortest-angle (wrap-aware) math.

- [x] M04-R1-05 Drive both camera presentation and lander rendering from the
  same interpolated presentation state.
  Source: USER
  Files: src/gui.cpp
  Evidence: the main loop builds one render_state and feeds it to cam.update,
  draw_terrain, and draw_lander, while physics/collision/HUD keep the
  authoritative fixed-step state.

- [x] M04-R1-06 Replace whole-millisecond GUI frame timing with SDL3
  high-resolution monotonic timing.
  Source: USER
  Preferred API: SDL_GetTicksNS or appropriate SDL3 equivalent.
  Files: src/gui.cpp
  Evidence: frame delta, the FPS frame budget, and random_seed now use
  SDL_GetTicksNS.

- [x] M04-R1-07 Handle landed/crashed terminal transitions without rendering an
  invalid interpolated state.
  Source: USER
  Files: src/sim.cpp
  Evidence: interpolated_state snaps to the authoritative state when
  landed/crashed (no blending into an invalid intermediate).

- [x] M04-R1-08 Make the starfield a deterministic fixed screen-space
  background.
  Source: USER
  Required invariant:
      star_screen_position = function(seed, star_index, viewport_size)
  Files: include/lander/starfield.hpp
  Evidence: star_screen_pos returns the seed/viewport-derived position and
  ignores all camera arguments (this supersedes the earlier inertial
  camera-rotation starfield behavior).

- [x] M04-R1-09 Star screen position must be independent of camera translation,
  zoom, and rotation.
  Source: USER
  Files: include/lander/starfield.hpp, tests/test_starfield.cpp
  Evidence: tests/test_starfield.cpp asserts identical star screen positions
  across camera x/y, scale, and angle. ctest lander_starfield_tests passed.

## Preserve / constraints

- [x] M04-R1-P01 Preserve inverse-square radial gravity and existing genuine
  free-flight orbital physics.
  Source: USER
  Evidence: gravity a = -mu*r/|r|^3 unchanged; test_radial_gravity and
  test_orbit_stays_bounded pass.

- [x] M04-R1-P02 Preserve the current moon constants:
      reference radius = 332.384 m
      mu = 178976.334 m^3/s^2
      nominal reference circular period ~= 90 s
      nominal reference circular speed ~= 23.205 m/s
  Source: USER
  Evidence: constants unchanged; test_reference_values passes.

- [x] M04-R1-P03 Do not change the orbital integrator merely to hide
  presentation stutter.
  Source: USER
  Evidence: fixed_dt remains 1/120 and the semi-implicit Euler integrator is
  unchanged; only presentation interpolation was added.

- [x] M04-R1-P04 Preserve production O circularize behavior with no continuing
  hidden stabilization.
  Source: USER
  Evidence: circularize() unchanged; test_circularize_state and
  test_circularize_orbit pass.

- [x] M04-R1-P05 Preserve F refuel and the current 1000-unit M04 testing fuel.
  Source: USER
  Evidence: refuel() and Config.fuel = 1000 unchanged;
  test_refuel_only_changes_fuel passes.

- [x] M04-R1-P06 Preserve corrected moon fill, terrain generation, collision,
  and local-frame world-geometry orientation.
  Source: USER
  Evidence: terrain.cpp and the terrain/collision paths are untouched by
  M04-R1; existing terrain/landing/collision tests pass.

- [x] M04-R1-P07 Preserve throttle and HUD behavior.
  Source: USER
  Evidence: throttle and HUD code are untouched by M04-R1.

- [x] M04-R1-P08 Do not add velocity look-ahead in M04.
  Source: USER
  Evidence: the camera uses the exact current target only; no look-ahead term.

- [x] M04-R1-P09 Do not begin M05.
  Source: USER
  Evidence: only M04 presentation files changed (see git status --short).

## Automated verification

- [x] M04-R1-V01 Add a camera regression that moves a synthetic target through
  at least one full 360-degree revolution and verifies the target remains at
  the exact intended screen anchor.
  Files: tests/test_camera.cpp
  Evidence: test_full_revolution_anchor_and_zoom drives 1200 steps over a full
  revolution, checking the exact anchor (1e-6) at each step. ctest
  lander_camera_tests passed.

- [x] M04-R1-V02 Verify anchor preservation across MANUAL zoom from 0.2X
  through 4.0X.
  Files: tests/test_camera.cpp
  Evidence: test_full_revolution_anchor_and_zoom sweeps manual zoom from 1.0X
  up to the 4.0X clamp and back down to the 0.2X clamp, checking the exact
  anchor at every step. ctest lander_camera_tests passed.

- [x] M04-R1-V03 Verify anchor preservation during AUTO zoom transitions.
  Files: tests/test_camera.cpp
  Evidence: test_full_revolution_anchor_and_zoom and
  test_auto_hysteresis_and_zoom drive AUTO overview/landing transitions and
  check the anchor. ctest lander_camera_tests passed.

- [x] M04-R1-V04 Add a fixed-step orbital diagnostic recording radius, angular
  position, and angular increment and verify smooth monotonic authoritative
  physics progression.
  Files: tests/test_sim.cpp
  Evidence: test_orbit_presentation_is_smooth records per fixed-step
  radius/angular position/increment for an unpowered circular orbit and checks
  monotonic, bounded progression. lander_tests passed:
  physics steps=15221 mean_dtheta=0.000412818290943
  abs_min=0.00041264791594 abs_max=0.000412988753876.

- [x] M04-R1-V05 Verify interpolated presentation at representative render
  rates:
      60 Hz
      90 Hz
      120 Hz
      144 Hz
  with no repeated/jumped render positions caused by fixed-step cadence.
  Files: tests/test_sim.cpp
  Evidence: test_orbit_presentation_is_smooth replays one orbit at each rate and
  checks interpolated angular progression has no repeat or jump. lander_tests
  passed (per-rate mean/abs min/abs max all positive and tightly bounded):
  60 Hz mean=0.000825636581971 min=0.0008252958319 max=0.000825977507736
  90 Hz mean=0.000550424387938 min=0.000550197221264 max=0.00055065167183
  120 Hz mean=0.00041281829095 min=0.00041264791594 max=0.000412988753876
  144 Hz mean=0.000344015242453 min=0.000343873263284 max=0.000344157294896

- [x] M04-R1-V06 Verify star screen positions are identical when camera x/y,
  scale, and angle change.
  Files: tests/test_starfield.cpp
  Evidence: test_starfield asserts invariance across camera x/y, scale, and
  angle. ctest lander_starfield_tests passed.

- [x] M04-R1-V07 Preserve existing orbital radius and period regression tests
  without loosening their tolerances merely to obtain a pass.
  Files: tests/test_sim.cpp
  Evidence: test_orbit_stays_bounded and test_circularize_orbit are unchanged and
  passing with identical measured values to the pre-M04-R1 baseline:
  r0=417.174915020 v_analytic=20.712795196 T_analytic=126.549182365
  T_measured=126.550000000 min_r=417.088647413 max_r=417.261254043 (cw and ccw).

- [x] M04-R1-V08 Build successfully:
      cmake --build build
  Evidence: cmake --build build -j succeeded in this session (no errors).

- [x] M04-R1-V09 Pass complete test suite:
      ctest --test-dir build --output-on-failure
  Evidence: ctest reported 3/3 tests passed (lander_tests, lander_camera_tests,
  lander_starfield_tests), 0 failed.

- [x] M04-R1-V10 Pass:
      git diff --check
  Evidence: git diff --check produced no output (no whitespace errors).

- [x] M04-R1-V11 Pass normal and orbit headless smoke runs.
  Files: src/gui.cpp
  Evidence: SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy lander_gui
  --seed 1234 --frames 30 --fps 60 exited 0 (state=flying); and
  --orbit-demo --frames 300 --fps 60 exited 0 (state=flying, ticks=597).

## Human verification

- [x] M04-R1-H01 User confirms the lander remains correctly anchored during
  orbit and at high/low manual zoom.
  Source: USER (confirmed 2026-09-28)

- [x] M04-R1-H02 User confirms circular-orbit presentation is visually smooth
  rather than stuttery.
  Source: USER (confirmed 2026-09-28)

- [x] M04-R1-H03 User confirms stars remain fixed on screen and no longer trace
  circles as the local-frame world camera rotates.
  Source: USER (confirmed 2026-09-28)

## Derived implementation tasks

- [x] M04-R1-D01 Replace the alternating +/- manual-zoom check with a true
  monotonic 1.0X -> 4.0X -> 0.2X sweep that checks the anchor at each step, so
  V02 is exercised across the full supported range.
  Source: DERIVED
  Files: tests/test_camera.cpp
  Evidence: test_full_revolution_anchor_and_zoom now sweeps to the 4.0X clamp
  and down to the 0.2X clamp; ctest lander_camera_tests passed.

## Evidence / notes

Existing orbital evidence (unchanged before and after M04-R1):

    r0 = 417.174915020 m
    v_analytic = 20.712795196 m/s
    T_analytic = 126.549182365 s
    T_measured = 126.550000000 s
    min_r = 417.088647413 m
    max_r = 417.261254043 m

The same bounded-orbit result is reported clockwise and counter-clockwise, which
is evidence that the observed GUI stutter is presentation/timing related rather
than a failure of the underlying orbital physics.

M04-R1 session output (lander_tests, --orbit-demo source values):

    orbit-presentation physics steps=15221
        mean_dtheta=0.000412818290943 abs_min=0.00041264791594
        abs_max=0.000412988753876
    orbit-presentation render fps=60  frames=7611
        mean=0.000825636581971 min=0.0008252958319 max=0.000825977507736
    orbit-presentation render fps=90  frames=11416
        mean=0.000550424387938 min=0.000550197221264 max=0.00055065167183
    orbit-presentation render fps=120 frames=15221
        mean=0.00041281829095 min=0.00041264791594 max=0.000412988753876
    orbit-presentation render fps=144 frames=18265
        mean=0.000344015242453 min=0.000343873263284 max=0.000344157294896

Screenshots generated for human inspection (not read back by the model):

    /tmp/opencode/lunar_lander_m04_r1_spawn.ppm   (normal spawn, seed 1234)
    /tmp/opencode/lunar_lander_m04_r1_orbit.ppm   (orbit-demo, ~20s, seed 1234;
                                                  camera rotated, stars fixed)

## Request M04-R2 (flame animation: continuous presentation time)

Source: USER (2026-09-28, follow-up after M04-R1 human review)

Human observation:
- The lander/body motion is now substantially smoother, but the engine flame
  itself still visibly stutters/flickers in discrete steps, especially while
  the craft rotates.
- The flame animation was driven from the integer simulation `ticks` counter
  (an intentional cosmetic flicker). After interpolated rendering was added,
  that left the flame quantized to fixed physics steps.
- Treat this as a presentation-only follow-up. Do not interpret the cosmetic
  flicker as evidence that the orbital integrator is unstable.

### User requirements

- [x] M04-R2-01 Do not drive the flame animation directly from the integer
  simulation `ticks` counter.
  Source: USER
  Files: src/gui.cpp
  Evidence: draw_lander no longer uses s.ticks for the flame; the 0.7*ticks /
  1.3*ticks sine terms were removed and replaced by lander::flame_length().

- [x] M04-R2-02 Use continuous presentation/render time for the flame
  animation.
  Source: USER
  Files: src/gui.cpp
  Evidence: a double flame_clock (seconds) accumulates real frame dt each frame
  (frozen while paused, reset on start_mission) and is passed to draw_lander as
  flame_time.

- [x] M04-R2-03 The flame animation must remain visually smooth at 60, 90,
  120, and 144 Hz.
  Source: USER
  Evidence: flame length is a continuous function of real-time flame_clock with
  7 Hz / 11 Hz sine components (both below the 30 Hz Nyquist limit at 60 Hz
  sampling, so no aliasing), independent of refresh cadence.

- [x] M04-R2-04 Rotating the craft must not make the flame appear to jump
  between discrete positions because of tick-based length changes.
  Source: USER
  Evidence: flame_length depends only on thrust_level and the continuous clock,
  never on state.angle or ticks; rotation only changes flame direction, so
  length varies continuously through rotation.

### Preserve / constraints

- [x] M04-R2-P01 Preserve throttle-controlled flame magnitude (full throttle
  gives the original flame extent, low throttle gives a short puff; length
  scales with the throttle level).
  Source: USER
  Evidence: flame_length = thrust_level * (0.7 + 0.9*flame_flick(t)); the
  full-throttle extent range [0.43, 1.87] matches the original mapping. test
  checks 0.0 -> 0 and 0.25 < 1.0 at equal time.

- [x] M04-R2-P02 Preserve the cosmetic flame variation (the flame still varies
  in length over time; it must not become a static fixed-length line).
  Source: USER
  Evidence: flame_flick(t) is a non-constant sum of sines; test asserts the
  one-second length range exceeds 0.3 and that it varies within a single tick.

- [x] M04-R2-P03 The flame animation must not affect physics, fuel consumption,
  thrust, collision, replay state, or authoritative simulation determinism. It
  is a purely cosmetic render effect.
  Source: USER
  Evidence: flame_flick/flame_length are pure functions in the lander namespace
  never called by Simulation::step_fixed/advance; the only caller is
  draw_lander (render). Headless smoke shows ticks=237 and fuel=1000 identical
  to the pre-change baseline.

### Automated verification

- [ ] M04-R2-V01 Add a focused presentation test asserting the flame length is
  a continuous, smooth function of presentation time and is NOT constant within
  a single physics step (i.e., not tick-quantized); also assert it is bounded,
  varies over time, and that the throttle still scales its magnitude.
  Files: tests/test_sim.cpp
  Evidence: test_flame_animation_continuous checks (1) variation within one
  1/120 s tick window, (2) sub-0.2 jumps across 1 ms sub-steps, (3) 1 s range
  > 0.3, (4) non-negativity, and (5) throttle scaling (0.0 -> 0, 0.25 < 1.0,
  full > 0.4). lander_tests passed.

- [x] M04-R2-V02 Rebuild succeeds:
      cmake --build build
  Evidence: cmake --build build -j succeeded (sim.cpp, gui.cpp, test_sim.cpp
  recompiled; no errors).

- [x] M04-R2-V03 Pass complete test suite:
      ctest --test-dir build --output-on-failure
  Evidence: ctest reported 3/3 tests passed (lander_tests, lander_camera_tests,
  lander_starfield_tests), 0 failed.

- [x] M04-R2-V04 Pass:
      git diff --check
  Evidence: git diff --check produced no output (exit 0).

- [x] M04-R2-V05 Pass normal and orbit headless smoke runs (confirms the flame
  change did not break the render loop or state machine).
  Files: src/gui.cpp
  Evidence: SDL_VIDEODRIVER=dummy lander_gui --seed 1 --frames 120 exited 0
  (state=flying, ticks=237); and --orbit-demo --frames 120 exited 0
  (state=flying, ticks=237).

### Human verification

- [x] M04-R2-H01 User confirms the engine flame animates smoothly (no discrete
  step flicker), including while the craft rotates, at their display refresh
  rate (60/90/120/144 Hz).
  Source: USER (confirmed 2026-09-28)

### Derived implementation tasks

- [ ] M04-R2-D01 Introduce pure `lander::flame_flick(t)` /
  `lander::flame_length(thrust, t)` helpers driven by a continuous presentation
  clock (seconds) in the GUI, and use them in `draw_lander` in place of the
  integer-tick flicker.
  Source: DERIVED
  Files: include/lander/sim.hpp, src/sim.cpp, src/gui.cpp
  Evidence: flame_flick/flame_length defined in sim.cpp (7 Hz / 11 Hz, magnitude
  preserved); gui.cpp advances a continuous flame_clock and passes it to
  draw_lander.
