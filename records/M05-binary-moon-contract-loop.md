# M05 — Binary moon and first contract loop

Date: 2026-09-30
Status: COMPLETE

## Goal

Turn the M04 circular-moon flight simulation into the first complete playable
game loop by extending the world from one moon to a compact two-body system and
adding a minimal repeating contract between the two bodies.

M05 shipped:

- a smaller companion moon in a real circular barycentric binary orbit with the
  M04 primary
- simultaneous two-body gravity acting on the spacecraft
- body-relative collision, navigation, orbit initialization, landing, and
  takeoff on both bodies
- tidal locking of both bodies to the binary line of centres
- a system-scale camera view for wide-binary navigation
- a repeating primary <-> companion contract loop
- the camera/control polish needed to make the binary playable

No ECS, missions, cargo, mining, economy, or M06 work was introduced.

## Final system constants

Primary (the unchanged M04 canonical body):

    reference radius R0:             332.384 m
    intrinsic surface gravity g0:    1.62 m/s^2
    gravitational parameter mu0:     178976.334 m^3/s^2
    nominal surface circular speed:  23.2048 m/s
    nominal surface circular period: 90.0 s

Companion (derived from the canonical scaling law, not hand-tuned):

    radius scale s:                  1/9
    reference radius R1:             36.9316 m
    intrinsic surface gravity g1:    1.62 m/s^2
    gravitational parameter mu1:     mu0 / 81 ~= 2209.584 m^3/s^2
    nominal surface circular speed:  7.7349 m/s
    nominal surface circular period: 30.0 s

Binary ephemeris:

    fixed centre-to-centre separation D: 600.0 m
    mu_system:                           mu0 + mu1
    omega:                               sqrt(mu_system / D^3)
    binary period:                       ~= 216.94 s
    primary barycentric radius:          D / 82 ~= 7.317 m
    companion barycentric radius:        81 D / 82 ~= 592.683 m

Both bodies are tidally locked to the binary line of centres (rotation zero at
t = 0, ~216.94 s period). Their terrain and pads rotate rigidly with the body,
landed attachment and takeoff inherit the full surface-point velocity, landing
is evaluated against the moving surface point, and the contract destination
pad is a moving target.

World metres remain ordinary simulation metres; there is no fake distance
scale, fast-travel coordinate transform, or hidden velocity scaling.

## Shipped gameplay and controls

- Throttle: Up/W increases, Down/S decreases, X cuts.
- Rotation: Left/A and Right/D (visually natural mapping to the simulation's
  rotate_left/rotate_right inputs).
- Camera: `M` toggles LOCAL AUTO / LOCAL MANUAL; from SYSTEM, `M` enters LOCAL
  MANUAL directly. `V` performs the normal system-view save/restore round
  trip. The mouse wheel zooms in LOCAL MANUAL.
- Reaction wheels:
  - `E` is a discrete GUI toggle (`RW OFF` <-> `RW ON`).
  - `Shift+E` is a transient hold-to-damp control: it arms reaction-wheel
    damping only while held and does not change the stored `E` toggle.
  - Manual rotation takes priority over both for that step; a crash disables
    both; retry/new mission/new seed reset the stored toggle to OFF.
  - The HUD shows `RW HOLD`, otherwise `RW ON` / `RW OFF`.
- Circularize: `O x3` CW, `Shift+O x3` CCW (triple-tap guarded).
- Debug initializers: `B x3` body-synchronous orbit, `T x3` velocity-only
  ballistic inter-body transfer (both triple-tap guarded).
- `G` toggles the navigation/gravity overlay; `F` refuels; `P` pauses;
  `N x3` rolls a new seed; `R x3` retries the same seed.
- The SYSTEM view shows the whole binary, the moving contract destination,
  offscreen navigation cues, and a minimum-size oriented ship marker with a
  deterministic throttle-scaled thrust plume.
- The HUD reports explicit reference-frame values (REF/JOB, ALT, VRAD, VTAN,
  ATT, SPIN, ORB, destination DIST/VREL/RANGE) and the reaction-wheel state.

## Request groups and final outcome

- M05-R1: the initial binary-moon / first-contract-loop implementation
  (two-body gravity, companion derivation, body-relative navigation and
  landing, SYSTEM view, contract loop). Human playtest produced M05-R2.
- M05-R2: camera/navigation follow-up (reference-body camera stability,
  SYSTEM offscreen destination navigation, oriented minimum-size ship marker).
  Human playtest produced M05-R3.
- M05-R3: camera/HUD/control fixes and tidal locking, then three corrective
  rounds:
  - M05-R3-01..09: LOCAL AUTO readability floor, smooth SYSTEM wheel zoom,
    explicit HUD reference frames, reaction-wheel damping, navigation/gravity
    overlay, `O` / `Shift+O` CW/CCW semantics, and tidal locking.
  - M05-R3-10..16: triple-tap guarded `R x3` retry, wider LOCAL manual zoom,
    inertial starfield, SYSTEM no-auto-pan, adaptive zoom formatting, `B x3`
    body-synchronous orbit initializer, and `T x3` transfer initializer.
  - M05-R3-17..22: corrective fixes after human re-test (wide-LOCAL rendering,
    terrain seam, non-blocking transfer solver, velocity-only transfer
    semantics, starfield no-regression rule).
  - M05-R3-23..30: final geometry-driven body-render coverage, viewport-safe
    local-patch closure, removal of visible radial tick/seam artifacts, and
    identical treatment of both bodies. H17/H22 passed on the `1f3eae3`
    corrected build.
- M05-R4: final camera/control presentation polish:
  - `M` from SYSTEM enters LOCAL MANUAL through `Camera::enter_manual()` while
    `V` keeps its save/restore semantics.
  - the minimum-size SYSTEM marker gained a deterministic throttle-scaled
    thrust plume (`marker_plume()`), with the full-model flame unchanged.
  - `E` became a discrete reaction-wheel toggle with manual-rotation priority,
    crash suppression, reset on new mission/seed/retry, and an
    `RW ON` / `RW OFF` HUD indicator.
- M05-R5: `Shift+E` became a transient hold-to-damp reaction-wheel control
  while `E` remains the discrete toggle. The hold is computed each frame from
  the polled keyboard and modifier state, is composed with the stored toggle
  into `Input.reaction_wheels`, and is shown as `RW HOLD` in the HUD.

## Requirements -> implementation -> evidence (final rounds)

M05-R4 (all confirmed):

- M05-R4-01 SYSTEM -> `M` enters LOCAL MANUAL
  -> `include/lander/camera.hpp::Camera::enter_manual()` + `src/gui.cpp`
  -> `tests/test_camera.cpp::test_m_from_system_enters_manual`,
  `test_m_from_system_keeps_v_restore_semantics`,
  `test_m_from_system_does_not_modify_simulation`.
- M05-R4-02 SYSTEM minimum-marker thrust indication
  -> `include/lander/camera.hpp::marker_plume()` + `src/gui.cpp::draw_lander()`
  -> `tests/test_render_geom.cpp::test_marker_plume_geometry`.
- M05-R4-03 `E` reaction-wheel toggle with HUD state
  -> `include/lander/guarded_actions.hpp::ReactionWheelToggle` + `src/gui.cpp`
  -> `tests/test_sim.cpp::test_reaction_wheel_toggle` and the unchanged
  `test_reaction_wheel_damping`.

M05-R5 (all confirmed):

- M05-R5-01..04 (`Shift+E` hold, `E` remains toggle, manual/crash priority,
  HUD/legend documentation)
  -> `ReactionWheelToggle::press(repeat, shift)` and
  `ReactionWheelToggle::input(manual, active, hold)` in
  `include/lander/guarded_actions.hpp`; per-frame `Shift+E` composition in
  `src/gui.cpp`; `RW HOLD` / legend / usage updates
  -> extended `tests/test_sim.cpp::test_reaction_wheel_toggle`.

## Automated verification

Final M05-R5 build (and re-run at closeout):

    cmake --build build
    ctest --test-dir build --output-on-failure   # 6/6 passed
    git diff --check                             # clean

The six test targets are `lander_tests`, `lander_binary_tests`,
`lander_camera_tests`, `lander_starfield_tests`,
`lander_guarded_actions_tests`, and `lander_render_geom_tests`.

Headless GUI smoke (`SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy`), all exit
0 with sane final state:

    ./build/lander_gui --seed 7 --frames 3 --fps 60
    ./build/lander_gui --system-view --seed 7 --frames 3 --fps 60
    ./build/lander_gui --orbit-demo --system-view --seed 7 --frames 5 --fps 60

No image files were read by the model (text-only constraint); graphical
behavior was verified through process exit status, logs, simulation state,
tests, and headless input behavior.

## Human verification

- All M05-R1, M05-R2, and M05-R3 human-verification items passed in the final
  M05 playtest on 2026-09-30 (the earlier H17/H22 re-test had already passed on
  the `1f3eae3` corrected build).
- M05-R4-H01..H03 and M05-R5-H01..H02 were confirmed by the user on 2026-09-30
  ("all check"):
  - SYSTEM -> `M` enters LOCAL MANUAL with sensible zoom and radial-down
    orientation
  - the minimum-size SYSTEM marker shows clear, correctly directed thrust
    without cluttering navigation cues
  - `E` works naturally as a reaction-wheel toggle with visible RW state and
    manual-rotation priority
  - `Shift+E` acts as a temporary hold-to-damp control and does not change the
    stored `E` toggle
  - the HUD/legend feedback for `RW HOLD` vs `RW ON`/`RW OFF` is unambiguous

## Closeout

- `STATUS.md` is set to `COMPLETE`.
- The active `TASKS.md` ledger is marked `COMPLETE`; all M05 request groups
  have verified user, preservation, automated-verification, and
  human-verification entries.
- This record is the compressed retrospective provenance for M05.
- No M06 work has been started.
