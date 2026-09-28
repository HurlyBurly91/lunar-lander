# Status

## Current milestone

M03 — Throttle and camera polish

State: NOT STARTED

Specification:

milestones/M03-throttle-camera-polish.md

## Prior milestone

M02 — Contextual camera and manual zoom

State: COMPLETE

M02 added:

- AUTO overview / landing camera
- overview zoom 0.40
- landing zoom 1.40
- 18/25 m altitude hysteresis
- MANUAL camera mode
- mouse-wheel zoom
- unified world-to-screen camera transform

Human verification accepted the AUTO overview scale and transition behavior.

Remaining issues discovered during human play testing are now M03:

- binary main thrust makes precise altitude control difficult
- rapid manual wheel zoom can lose lander framing
- starfield incorrectly moves with camera translation and zoom
