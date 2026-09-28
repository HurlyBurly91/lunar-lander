# Lunar Lander

## Goal

Build a Lunar Lander game incrementally from the existing working SDL3
simulation.

The project should remain playable after every milestone.

The long-term direction is:

1. establish recognizable classic Lunar Lander gameplay,
2. then extend the world into a circular moon,
3. eventually support near-surface orbital flight with an approximately
   90-second nominal low circular orbit.

Do not implement later stages early.

## Current baseline — M00

The current working game provides:

- SDL3 window and renderer
- keyboard thrust and rotation
- fixed-timestep lander simulation
- fuel
- landing/crash states
- HUD
- restart/new seed
- deterministic simulation behavior
- flat ground and a landing pad

This is the working baseline and must remain playable.

## Milestone sequence

### M00 — Playable SDL3 baseline

Status: complete/baseline.

### M01 — Classic lunar terrain

Replace the flat infinite ground with deterministic jagged lunar terrain and
real flat landing sites.

This milestone should make the game visually and mechanically recognizable as
a Lunar Lander game without changing the world to a circular moon.

Later milestones are intentionally not specified in implementation detail yet.

### M02 — Contextual camera and manual zoom

Add an automatic overview/landing camera plus a player-selectable manual
mouse-wheel zoom mode.

This milestone changes presentation only. Curved-moon physics remains later
work.

### M03 — Throttle and camera polish

Add continuous main-engine throttle, correct zoom anchoring so rapid manual
zoom preserves lander framing, and make the stellar background independent of
local camera translation and zoom.

This is the final control/camera polish milestone before curved-moon work.

### M04 — Circular moon and orbital physics

Replace the flat world with a closed circular moon using inverse-square radial
gravity and seamless wrapped terrain.

Target a roughly 90-second nominal near-surface circular-orbit scale so orbital
flight is useful within short mobile-game sessions.

### M05 — Binary moon and first contract loop

Status: DEFINED, NOT ACTIVE until M04 closeout.

Use the M04 primary moon as the canonical gravitational calibration body.

Add a 1/9-radius companion derived from the same intrinsic-surface-gravity
scaling law, giving it a nominal ~30-second reference low circular orbit.

Place the two bodies in a deliberately compact 600 m centre-to-centre circular
binary and model spacecraft motion in one global inertial frame with gravity
from both bodies.

Add body-relative landing/takeoff, a system-scale camera view, and the first
repeating contract loop between a base on each moon.

Specification:

milestones/M05-binary-moon-contract-loop.md

ECS conversion and modular spacecraft remain later work.
