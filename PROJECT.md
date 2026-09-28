# Lunar Lander

## Goal

Build a Lunar Lander game incrementally from the existing working SDL3
simulation.

The project should remain playable after every milestone.

The long-term direction is:

1. establish recognizable classic Lunar Lander gameplay,
2. then extend the world into a circular moon,
3. eventually support near-surface orbital flight with an approximately
   10-minute low circular orbit.

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
