# M06 Subsystem-Isolation Debug Harness (`--debug-subsystem`)

This is a developer-facing reference for the `--debug-subsystem <name>` option in
`lander_gui`. It is a read-only diagnostic harness: it lets a developer bring up
the game with **exactly one** M06 flight-computer subsystem isolated, so that
subsystem's inputs and outputs can be watched in the window without the visual
noise of the other five M06 surfaces.

It is **not** a gameplay feature and it **never** changes how the game plays.

## What it does

With a selector active:

- An expanded panel (top-right, where the flight computer normally is) shows a
  **common minimum readout** plus a full **mode-specific** section with the
  live values that describe the isolated subsystem's behaviour (see below).
- The unrelated surfaces are hidden: the nav / circular-orbit overlay, the
  predicted-trajectory + LIVE-prediction legend, the normal flight-computer
  panel, and the contract banner. Three deliberate exceptions:
  - `ui` keeps **every** normal player-facing surface (nav overlay, the
    zero-thrust/LIVE prediction + legend, the flight-computer panel, the
    contract banner, the HUD) and adds only a minimal one-line debug header
    below the HUD, so the normal surfaces can be judged without the other
    subsystems' panels.
  - `node-edit` also keeps the planned-arc overlay (the node and its pre/post
    prediction branches) so the edited node stays visible while editing.
  - `predictor` keeps the **predicted-trajectory overlay** (the COAST / PLAN
    arc with its PE / AP / closest-approach / impact markers, the powered LIVE
    projection, and the three-kind legend) but NOT the nav / orbit field, so the
    projected path — the mode's primary observable — is visible while reading
    the predictor panel. F2 / F3 / F4 re-point the live predictor's policy, so
    the visible projection follows the selected kind.
- A deterministic, **startup-only** fixture places the ship in a starting
  condition that exercises the selected subsystem, and arms exactly that one
  subsystem. From that point the subsystem runs through the normal simulation
  and control paths — the harness applies no hidden forces and never mutates
  the simulation directly. The panels are strictly read-only: every value they
  show is either taken from the subsystem's existing public state or computed
  cheaply from it, and no panel value is ever fed back into the simulation or
  a controller.

Without a selector (absent, or `none`), the rendering and initialization paths
are unchanged: normal gameplay is byte-for-byte what it was before the harness
existed (verified by byte-identical headless final states for absent vs
`none`).

## The modes

| selector              | isolates                                                    | startup fixture                                                        |
|-----------------------|-------------------------------------------------------------|------------------------------------------------------------------------|
| `manual`              | the pilot (thrust / attitude / cutoff) as the reference case | ship landed on the primary pad                                          |
| `predictor`           | the shifted receding-horizon live predictor                 | clean circular orbit around the primary                                 |
| `attitude`            | the PD attitude controller (VGO / node / coast)             | clean orbit with a prograde attitude demand                             |
| `node-edit`           | the single maneuver node + planner (no burn)                | clean orbit with a small planned prograde node (not executed)          |
| `node-executor`       | the one-shot VGO node executor                              | clean orbit, the node armed for execution, maneuver attitude active     |
| `transfer-cold`       | the one-shot COLD inter-moon transfer solve                 | clean orbit; a COLD 0->1 solve runs once and is displayed (no live loop)|
| `transfer-warm`       | the two-level WARM transfer midcourse (seeded from COLD)    | clean orbit; COLD seeds the cache, the midcourse is engaged (bounded rate)|
| `autoland-primary`    | the powered-landing autopilot landing on the primary        | orbiting the primary, landing armed toward the primary pad             |
| `autoland-companion`  | the powered-landing autopilot landing on the companion      | orbiting the companion, landing armed toward the companion pad         |
| `autoland-cross`      | the powered-landing autopilot on the cross-body (V14) route | orbiting the primary, landing armed toward the companion (cross route)  |
| `ui`                  | camera / navigation / contract / HUD / controls (no sim logic)| ship landed on the primary pad                                       |

`none` is accepted and runs ordinary gameplay with no debug surface.

## The common readout

Every mode shows the same baseline, derived from the live simulation only:

```
DEBUG: <mode-name>
<one-line description of what the mode isolates>
T <sim time s>
BODY <reference body>   TARGET <contract target body>
STATE <FLIGHT|LANDED|CRASHED>   X <x>   Y <y>
ALT <altitude m>   VR <radial m/s>   VT <tangential m/s>
V-REL <relative speed m/s> (vs target)
```

Each mode then appends a `[ <mode-name> ]` section with the full live values
for the isolated subsystem:

- **manual** — throttle, reaction-wheel state and hold, the composed per-step
  rotate input, surface radial / tangential velocity, angle and spin, the
  clear flag, and the pilot control hints (this is the reference case: the
  pilot is the "subsystem").
- **predictor** — the current prediction policy (COAST / LIVE / PLAN, switchable
  with F2 / F3 / F4), the cache last operation and rebuild / shift /
  invalidation counters, the horizon and sample count, the primed / terminal
  flags, the current policy signature and whether it changed (which forces a
  cold rebuild), the predicted contact, perihelion / apohelion and minimum
  terrain clearance over the sample ring, the closest approach to the
  destination pad, and the measured advance cost against the per-frame
  rebuild budget. The projected trajectory itself is drawn on the map (the
  same COAST / PLAN arc, powered LIVE projection, and legend as normal
  gameplay), so the path can be read alongside the panel's numbers; F2 / F3 /
  F4 change which policy the live predictor projects.
- **attitude** — the requested mode (1-8), target vs actual nose angle,
  angle error and spin, the stop angle and whether the stop condition is met,
  the commanded wheel direction from the last step input, and the 1-8 key
  legend.
- **node-edit** — the node's frame / time / prograde + radial delta-v, the
  total delta-v, the node's world position, the predicted pre/post branch
  endpoints, perihelion / apohelion, and the plan kind (n/a: the planner does
  not store it), plus the create / plan / execute key hints.
- **node-executor** — the executor state (Wait / Align / Burn / Complete /
  Aborted / Incomplete), the node and its ignite / burn times (and whether the
  burn is running late), the remaining / total / delivered delta-v, the
  throttle, the remaining fuel, and the final result, plus the key hints.
- **transfer-cold** — the one-shot COLD solve: validity, achieved miss, time
  of flight, departure and arrival states, terrain clearance at both ends,
  the propagation count and wall-clock solve time, and a note that the raw
  terrain surface is the known TFD-1 input (surfaced, never corrected).
- **transfer-warm** — the two-level midcourse: active state, the COLD seed,
  the cache, the WARM Newton iteration count and whether a cold fallback
  occurred, the propagation cost of the last re-plan (and total), the
  achieved miss before / after the last re-plan against the miss tolerance,
  and the re-plan / retarget counts and cadence, plus the same TFD note.
- **autoland-primary / autoland-companion / autoland-cross** — the armed
  target and source, the phase (cross additionally shows the full
  ASCEND > TRANSFER > CAPTURE > DE-ORBIT > BRAKE > APPROACH > DESCENT >
  TOUCHDOWN sequence with the current phase bracketed), the target-relative
  speed and radial / tangential velocity, the held command acceleration
  (magnitude and nose angle) and throttle with `t_go`, the wanted / actual
  attitude and error, the active guidance path (high-energy VGO / gentle
  gravity feedforward / ZEM-ZEV terminal), the terminal law state (or, for
  cross, the terminal handoff gate: feasibility, `t_go`, peak acceleration,
  initial radial), and the result (in flight / touchdown / crashed).

## Using it

```
./build/lander_gui --debug-subsystem predictor
./build/lander_gui --debug-subsystem transfer-warm
./build/lander_gui --debug-subsystem autoland-cross
./build/lander_gui            # no selector: normal gameplay, unchanged
```

A selector that is not one of the names above (and is not empty) is a hard
error reported before the window opens, with the usage text.

Every mode keeps the normal piloting controls (throttle, rotate, X cutoff, P
pause, V system view, G nav overlay, mouse-wheel zoom, N x3 / R x3 re-seed /
retry, Shift+O / O circularize, E / Shift+E reaction wheels). In addition, the
modes expose these harness-relevant controls:

- **predictor** — F2 / F3 / F4 switch the projected arc's prediction policy
  between COAST (zero-thrust ballistic), LIVE (the pilot's current throttle
  and attitude demand), and PLAN (the planned maneuver node, when one is
  set). Only the projected arc changes; the ship and its controls are
  untouched.
- **attitude** — 1-8 set the attitude demand (1 OFF, 2 PROGRADE, 3 RETRO,
  4 RADIAL OUT, 5 RADIAL IN, 6 TARGET, 7 ANTI-TARGET, 8 MANEUVER).
- **node-edit / node-executor** — I / Z plan and engage the normal node and
  midcourse flows; Return arms the one-shot executor on the planned node;
  Shift+Return (or X) aborts the executor / midcourse.
- **transfer-warm** — Z engages the midcourse (after a COLD plan exists),
  X aborts it; any manual throttle input reclaims control from it, as in
  normal play.
- **autoland-*** — 9 arms the landing autopilot on the reference body
  (the fixture arms it at startup); X aborts it; manual throttle reclaims
  control.

## Determinism

Each mode re-seeds the simulation with a fixed per-mode seed (independent of
`--seed`) so the same mode always starts from the same terrain, contract, and
placed state:

```
none=0  manual=1001  predictor=1002  attitude=1003  node-edit=1004
node-executor=1005  transfer-cold=1006  transfer-warm=1007
autoland-primary=1008  autoland-companion=1009  autoland-cross=1010  ui=1011
```

## Scope and constraints (why it must stay small)

The harness is a **DEBUG** tool with hard limits:

- It only isolates and displays. It does **not** fix, harden, or change the
  behavior of the subsystems it shows. If a subsystem has a latent defect
  (for example TFD-1 or TFD-2), the harness will faithfully surface it; it will
  not correct it.
- It never changes canonical physics or `fixed_dt`.
- It never weakens, deletes, or relaxes tests.
- It is startup-only: the fixture runs once, at scenario start; afterwards the
  armed subsystem runs through the ordinary per-frame control and simulation
  loop.
- Only one mode is active at a time; the panel favors a low-rate / event-driven
  readout rather than per-frame log spam.

## Where it lives

- `include/lander/debug_subsystem.hpp` / `src/debug_subsystem.cpp` — the
  `lander::debug_subsystem` module: the `DebugSubsystem` selector, the
  selector parser / names / descriptions, the common readout, the per-mode
  seeds, and the `setup_debug_scenario` fixture.
- `src/gui.cpp` — the `--debug-subsystem` argument, the pre-`SDL_Init`
  validation, the startup fixture call, the `DebugPanelCtx` gui-local
  observation context (the last composed per-step input, the prediction
  cache, the predictor's current policy / signature / cost, and the WARM
  re-plan observations), and the expanded `draw_debug_subsystem_panel` plus
  the per-mode surface suppression when a mode is active.
- `include/lander/landing.hpp` — small read-only accessors on
  `LandingAutopilot` (`target_body()`, `source_body()`, `high_energy()`,
  `target_disturbed()`, `command()`, `terminal_preview()`) so the panel can
  display the armed target, active law, held command, and terminal preview
  without storing any new state in the subsystem.
- `tests/test_debug_subsystem.cpp` — headless coverage of the selector, the
  seed table, the common readout, and the one-subsystem-arming / determinism of
  every fixture.

## Related canonical documents

The subsystems the harness isolates are governed by:

    docs/flight-guidance-computational-rate-tiers.md
    docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md
    docs/flight-guidance-intermoon-transfer-differential-correction-warm-starting-and-bounded-replanning.md
    docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md
    docs/physics-model-gravity.md
