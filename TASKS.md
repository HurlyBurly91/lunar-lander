# Active Task

Milestone: M04
Request: M04-R1
State: ACTIVE

## Bootstrap note

The durable execution ledger was introduced after substantial M04
implementation had already occurred.

Do not fabricate historical request IDs for earlier work.

Earlier M04 implementation and verification will be summarized in the final
M04 record at closeout.

M04-R1 begins with the currently unresolved human-observed presentation issues.

## User requirements

- [ ] M04-R1-01 Keep the player lander at an exact screen-space anchor through
  camera rotation, translation, and zoom.
  Source: USER
  Required invariant:
      screen_x(lander) = 0.5 * window_width
      screen_y(lander) = lander_top_fraction * window_height

- [ ] M04-R1-02 Remove accumulated horizontal/local-tangent camera lag during
  orbit.
  Source: USER
  Notes: generic follow smoothing must not move the player away from its
  intended anchor.

- [ ] M04-R1-03 Add render interpolation between authoritative fixed simulation
  states.
  Source: USER
  Required basis:
      alpha = simulation_accumulator / fixed_dt

- [ ] M04-R1-04 Interpolate render x/y and attitude consistently, including
  shortest-path angle interpolation across angular wrap.
  Source: USER

- [ ] M04-R1-05 Drive both camera presentation and lander rendering from the
  same interpolated presentation state.
  Source: USER

- [ ] M04-R1-06 Replace whole-millisecond GUI frame timing with SDL3
  high-resolution monotonic timing.
  Source: USER
  Preferred API: SDL_GetTicksNS or appropriate SDL3 equivalent.

- [ ] M04-R1-07 Handle landed/crashed terminal transitions without rendering an
  invalid interpolated state.
  Source: USER

- [ ] M04-R1-08 Make the starfield a deterministic fixed screen-space
  background.
  Source: USER
  Required invariant:
      star_screen_position = function(seed, star_index, viewport_size)

- [ ] M04-R1-09 Star screen position must be independent of camera translation,
  zoom, and rotation.
  Source: USER

## Preserve / constraints

- [ ] M04-R1-P01 Preserve inverse-square radial gravity and existing genuine
  free-flight orbital physics.
  Source: USER

- [ ] M04-R1-P02 Preserve the current moon constants:
      reference radius = 332.384 m
      mu = 178976.334 m^3/s^2
      nominal reference circular period ~= 90 s
      nominal reference circular speed ~= 23.205 m/s
  Source: USER

- [ ] M04-R1-P03 Do not change the orbital integrator merely to hide
  presentation stutter.
  Source: USER

- [ ] M04-R1-P04 Preserve production O circularize behavior with no continuing
  hidden stabilization.
  Source: USER

- [ ] M04-R1-P05 Preserve F refuel and the current 1000-unit M04 testing fuel.
  Source: USER

- [ ] M04-R1-P06 Preserve corrected moon fill, terrain generation, collision,
  and local-frame world-geometry orientation.
  Source: USER

- [ ] M04-R1-P07 Preserve throttle and HUD behavior.
  Source: USER

- [ ] M04-R1-P08 Do not add velocity look-ahead in M04.
  Source: USER

- [ ] M04-R1-P09 Do not begin M05.
  Source: USER

## Automated verification

- [ ] M04-R1-V01 Add a camera regression that moves a synthetic target through
  at least one full 360-degree revolution and verifies the target remains at
  the exact intended screen anchor.

- [ ] M04-R1-V02 Verify anchor preservation across MANUAL zoom from 0.2X
  through 4.0X.

- [ ] M04-R1-V03 Verify anchor preservation during AUTO zoom transitions.

- [ ] M04-R1-V04 Add a fixed-step orbital diagnostic recording radius, angular
  position, and angular increment and verify smooth monotonic authoritative
  physics progression.

- [ ] M04-R1-V05 Verify interpolated presentation at representative render
  rates:
      60 Hz
      90 Hz
      120 Hz
      144 Hz
  with no repeated/jumped render positions caused by fixed-step cadence.

- [ ] M04-R1-V06 Verify star screen positions are identical when camera x/y,
  scale, and angle change.

- [ ] M04-R1-V07 Preserve existing orbital radius and period regression tests
  without loosening their tolerances merely to obtain a pass.

- [ ] M04-R1-V08 Build successfully:
      cmake --build build

- [ ] M04-R1-V09 Pass complete test suite:
      ctest --test-dir build --output-on-failure

- [ ] M04-R1-V10 Pass:
      git diff --check

- [ ] M04-R1-V11 Pass normal and orbit headless smoke runs.

## Human verification

- [ ] M04-R1-H01 User confirms the lander remains correctly anchored during
  orbit and at high/low manual zoom.

- [ ] M04-R1-H02 User confirms circular-orbit presentation is visually smooth
  rather than stuttery.

- [ ] M04-R1-H03 User confirms stars remain fixed on screen and no longer trace
  circles as the local-frame world camera rotates.

## Derived implementation tasks

None yet. Add D-items here if implementation reveals necessary work not already
represented by the user requirements.

## Evidence / notes

Existing orbital evidence before M04-R1:

    r0 = 417.174915020 m
    v_analytic = 20.712795196 m/s
    T_analytic = 126.549182365 s
    T_measured = 126.550000000 s
    min_r = 417.088647413 m
    max_r = 417.261254043 m

The same bounded-orbit result was reported clockwise and counter-clockwise.

These results are evidence that the observed GUI stutter is currently more
likely presentation/timing related than a failure of the underlying orbital
physics.
