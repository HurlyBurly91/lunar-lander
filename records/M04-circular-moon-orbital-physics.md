# M04 — Circular moon and orbital physics

Date: 2026-09-28
Status: COMPLETE

## Goal

Replace the flat M01/M02/M03 world with a small closed circular moon and real
2-D radial inverse-square gravity so sustained ~90-second orbital flight is
practical, while keeping the same controls, terrain feel, throttle, camera
modes, and HUD. This is the first curved-moon milestone.

## Final moon constants

    surface gravity (at reference radius):  1.62 m/s^2
    reference moon radius:                 332.384 m
    reference circumference:              2088.43 m   (= 2*pi*332.384)
    gravitational parameter mu:          178976.334 m^3/s^2
    reference circular speed:             23.205 m/s
    reference circular period:             90 s (nominal near-surface)
    fixed simulation step:                1/120 s (120 Hz)

Source: `include/lander/sim.hpp` (`Config`), `include/lander/terrain.hpp`
(`kReferenceRadius`, `kReferenceCircumference`, `kSpawnAngle`).

## Coordinate system and local frame

- Moon-centred global inertial 2-D Cartesian frame; the moon centre is `(0,0)`.
- Lander position/velocity are global/inertial: `(x, y)` and `(vx, vy)`.
- Attitude stays inertial (`angle 0` -> thrust axis +Y); the physical lander is
  NOT auto-rotated by travel around the body.
- Spawn is at the "north pole" (`kSpawnAngle = pi/2`), so local outward ~= +Y
  and the original visual orientation is preserved.
- Local frame used by the camera and the landing checks, with
  `theta = atan2(y, x)`:
      local up    = radial outward   (camera angle_ = theta - 0.5*pi)
      local right = tangent
  so screen up is always local radial outward and screen right is local
  tangent anywhere around the moon.

## Gravity

- Inverse-square radial: `a = -mu * r / |r|^3` (`src/sim.cpp`). Points at the
  moon centre everywhere; ~1.62 m/s^2 at the reference radius.
- No constant-magnitude approximation, no velocity look-ahead, and no hidden
  stabilisation.

## Terrain and landing sites

- The deterministic M01 height field is wrapped around the reference
  circumference. The surface is a radius over a wrapped arc coordinate
  `s in [0, 2*pi*R)`, via `Terrain::arc_at_angle` / `angle_at_arc`
  (`terrain.hpp`).
- Continuous and seamless at the wrap boundary; the identical geometry is used
  by rendering and collision; it is fixed in moon/world coordinates (never
  regenerated from camera position).
- Landing sites are short constant-radius arcs (`Pad`: `center_arc`,
  `half_width` in metres of surface arc, `radius`, `multiplier`), deterministic
  from the seed, part of the real collision surface, and safe across the
  longitude seam.

## Landing / crash rules (local components)

- On ground contact the checks use LOCAL radial/tangential velocity
  (`LocalVelocity`) and LOCAL attitude relative to the surface outward
  direction, not global `vx/vy` or global angle 0.
- safe pad + safe radial speed + safe tangential speed + safe local attitude ->
  landed; otherwise crashed. The existing M01 gameplay thresholds were
  preserved.

## O circularize / F refuel

- O (circularize): from the current radius sets a pure-tangential circular
  orbit (`v = -sqrt(mu/r)` along the tangent) with zero radial velocity. It is
  a developer/test helper that uses the real simulation; there is no hidden
  autopilot and no gameplay orbit mode.
- F (refuel): restores fuel to the configured maximum and changes nothing else.
- Fuel was raised to a **temporary 1000 units** (`Config.fuel`) for M04 testing
  so sustained orbits and long flights are possible during verification. This
  is a temporary M04 testing value, not a final gameplay balance decision.

## Rendering

- Corrected moon fill: the disc interior is filled (regolith, surface line,
  pads, worn band) so the curved world reads as a body rather than an empty
  field with a floating line.
- `draw_terrain` samples the fixed moon/world-space surface; the camera applies
  only a rigid transform and never alters terrain geometry.
- HUD: altitude is local terrain-relative (`radial_distance - surface_radius`);
  velocity is shown as local `TANGENTIAL / RADIAL` components; throttle, fuel,
  and scoring are preserved.

## Camera (M04-R1 exact anchoring)

- The camera follows the lander in the global frame and rotates its local basis
  with `atan2(y, x)` so screen up is always local radial outward.
- M04-R1 fix: the player-follow anchor is set EXACTLY — `focus = target - up *
  framing_offset()` with NO follow smoothing applied in the rotating local
  frame. The earlier generic `follow_rate` smoothing of the focus accumulated
  horizontal/lateral drift over an orbit; removing it makes the lander hold the
  exact screen anchor (`0.30` from the top) at every angle and zoom.
- `framing_offset() = (0.5 - lander_top_fraction) * (window_height / scale())`;
  the `1/scale` factor cancels `to_screen`'s scale, so the anchor is exact at
  all zooms. `M` toggles AUTO/MANUAL; the wheel zooms in MANUAL (0.20–4.0);
  AUTO eases 0.40<->1.40 with 18/25 m hysteresis.

## Simulation / timing (M04-R1)

- The authoritative simulation stays fixed-step at 120 Hz (semi-implicit
  Euler), unchanged. Physics, collision, fuel, and scoring run on the
  authoritative state.
- M04-R1 added presentation-only render interpolation:
    `lander::interpolated_state(prev, curr, alpha, snap)` (`src/sim.cpp`)
  interpolates x/y in polar form and attitude by shortest-angle (wrap-aware)
  lerp, and snaps to the authoritative state when landed/crashed.
- GUI frame timing moved to high-resolution monotonic `SDL_GetTicksNS`; the
  main loop keeps an accumulator, computes `alpha = accumulator / fixed_dt`,
  and feeds ONE interpolated `render_state` to the camera, terrain, and lander
  (physics keeps the authoritative state).

## Starfield (M04-R1 decision)

- The starfield is a deterministic FIXED SCREEN-SPACE background:
    star_screen_position = f(seed, star_index, viewport_size)
  (`include/lander/starfield.hpp`). It does not depend on camera position,
  zoom, rotation, or the lander, so the rotating local-frame world camera no
  longer makes the stars trace circles. This intentionally favors stable game
  presentation over a physically exact inertial celestial sphere.

## Flame animation (M04-R2 decision)

- The engine flame was driven from the integer simulation tick counter, which
  quantized it to 120 Hz fixed steps and stuttered (especially during rotation)
  once interpolated rendering landed.
- M04-R2 made it a continuous presentation-time effect:
    `lander::flame_flick(t)` / `lander::flame_length(thrust, t)` (`src/sim.cpp`)
  driven by a GUI `flame_clock` (seconds) that advances with real frame time
  (frozen while paused, reset on restart). The components are 7 Hz and 11 Hz,
  both below the 30 Hz Nyquist limit at 60 Hz sampling, so there is no aliasing.
  Throttle still scales the magnitude (full -> original extent, range
  ~[0.43, 1.87]; low -> short puff). It is purely cosmetic: it never touches
  physics, fuel, thrust, collision, or deterministic replay (the only caller is
  `draw_lander`).

## Requirements -> implementation -> evidence

M04-R1 (presentation follow-up, USER 2026-09-28):

- M04-R1-01 exact screen-space anchor -> `Camera::update` exact focus
  (`camera.hpp`) -> `test_full_revolution_anchor_and_zoom` (`test_camera.cpp`)
- M04-R1-02 no accumulated orbit lag -> `follow_rate` no longer applied to the
  player anchor (`camera.hpp`) -> full-revolution test
- M04-R1-03/04/05 render interpolation, polar x/y + shortest-angle, one
  `render_state` -> `interpolated_state` (`sim.cpp`), `gui.cpp` main loop ->
  `test_orbit_presentation_is_smooth` (`test_sim.cpp`)
- M04-R1-06 high-resolution timing -> `SDL_GetTicksNS` (`gui.cpp`)
- M04-R1-07 terminal snap -> `interpolated_state` snap (`sim.cpp`)
- M04-R1-08/09 fixed screen-space starfield -> `star_screen_pos`
  (`starfield.hpp`) -> `test_starfield.cpp`
- M04-R1-P01..P09 (inverse-square gravity, moon constants, integrator, O
  circularize, F refuel, terrain/HUD, no look-ahead, no M05) -> verified by the
  unchanged existing suite and by byte-identical headless final states.

M04-R2 (flame, USER 2026-09-28):

- M04-R2-01..04 (not tick-driven, continuous time, smooth at 60/90/120/144 Hz,
  no rotation jumps) -> `flame_flick`/`flame_length` (`sim.cpp`) + `flame_clock`
  (`gui.cpp`) -> `test_flame_animation_continuous` (`test_sim.cpp`)
- M04-R2-P01..P03 (throttle magnitude preserved, still varies over time,
  presentation-only) -> `test_flame_animation_continuous` and unchanged
  headless ticks/fuel.

## Automated verification

    cmake --build build
    ctest --test-dir build --output-on-failure   # 3/3 passed (lander_tests,
                                                # lander_camera_tests,
                                                # lander_starfield_tests)
    git diff --check                              # clean

Headless GUI smoke (SDL dummy video), all exit 0 with the final state flying
(simulation is presentation-independent and deterministic):

    SDL_VIDEODRIVER=dummy ./build/lander_gui --seed 1 --frames 120
    # final: ... ticks=237 state=flying
    SDL_VIDEODRIVER=dummy ./build/lander_gui --seed 1 --frames 120 --orbit-demo
    # final: ... ticks=237 state=flying

## Orbital measurements (terrain-clearing circular orbit, `--orbit-demo` basis)

    r0 = 417.174915020 m
    v_analytic = 20.712795196 m/s
    T_analytic = 126.549182365 s
    T_measured = 126.550000000 s
    min_r = 417.088647413 m
    max_r = 417.261254043 m

The measured period matches the analytic Kepler period to ~0.001 s over a full
revolution, and the orbital radius stays bounded within ~0.17 m. The same
result is reported for both clockwise and counter-clockwise orbits, confirming
the integrator is stable and that the earlier GUI stutter was a
presentation/timing issue rather than a physics failure. (The orbit sits above
the nominal 90 s reference scale because it must clear real terrain:
`r0 = 417 m` > the 332.384 m reference radius, so `v = sqrt(mu/r)` is lower and
`T = 2*pi*sqrt(r^3/mu)` longer than the nominal 23.205 m/s / 90 s near-surface
values.)

## Human verification (all confirmed 2026-09-28)

- M04-R1-H01: lander stays correctly anchored through orbit and manual zoom. PASS
- M04-R1-H02: circular-orbit presentation is visually smooth. PASS
- M04-R1-H03: stars remain fixed in screen space. PASS
- M04-R2-H01: engine flame animates smoothly (no step flicker), including while
  the craft rotates, at 60/90/120/144 Hz. PASS

The earlier M04 curved-world checks (world visibly curves, no visible terrain
seam, landing at multiple longitudes, local up sensible around the body,
~90-second orbit gameplay scale, and deorbit using the same physics) were
confirmed during M04 development.

## Follow-ups (out of scope for M04)

- M05 and everything after (terrain variety, missions, cargo, mining, economy,
  etc.) are not started, per AGENTS.md.
- The 1000-unit fuel value is a temporary M04 testing value; revisit fuel
  balance when gameplay tuning resumes.
