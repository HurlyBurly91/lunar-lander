# Status

## Current milestone

M06 — Flight computer and maneuver planning

State: AWAITING HUMAN VERIFICATION

Phase: `M06-R11` (M06 HARDENING — Pass 1: trustworthy diagnostics). Implementation
+ automated verification are **complete**; only the human visual pass
(`M06-R11-H01`) remains open. `M06-R12` (prediction reference-frame architecture)
is the next queued bounded group (registered in `TASKS.md`, not yet started). This
bounded pass made the debug **readouts** tell
the truth and changed NO predictor / controller physics:
- the debug font now renders every diagnostic character (lowercase a-z ->
  uppercase; `[`/`]` now drawn) via `include/lander/debug_font.hpp`;
- the common readout altitude is now **body-relative and signed** (selected
  reference body's terrain + tidal rotation, via `altitude_at`) with an explicit
  `REF <body>(#n)` / `TGT <body>(#n)` identity;
- diagnostic meanings / units are relabelled (manual SURF VR/VT = body-centre
  relative; V-REL = target body-centre relative; predictor PE/AP -> `MIN R` /
  `MAX R`; clearance -> `CLR-PT` reference-point via `altitude_at`; contact `T+`
  -> ETA with separate absolute time; horizon -> steps + seconds; rolling vs
  long COAST/PLAN distinguished; a `WORLD/INERTIAL` frame label on every
  world-frame quantity; explicit unavailable states);
- the amber closest-approach square now carries an in-scene `CP <body> T<+eta>s`
  label;
- the debug orbit fixture (`place_in_orbit`) now uses the selected body's own
  terrain / `mu` (was the primary terrain + `cfg.mu`).
This is **Pass 1** of the bounded post-M06 hardening: it added NO frozen-COAST
validator, changed NO guidance / transfer / collision / canonical physics, and
made NO commit. **PRED-01..08 and SIM-COLL-01 all remain OPEN** (recorded in
`docs/m06-predictor-physics-issues.md`; Pass 1's PRED-01 label and PRED-06
relabel partially improve symptoms but do not close those issues). The M06
predictor subsystem remains NOT accepted; the two transfer modes remain subject
to the known TFD-1/TFD-2. The underlying harness is the M06-R8
subsystem-isolation debug tool (`--debug-subsystem <name>`, 11 modes; M06-R9
kept the predicted-trajectory overlay visible in predictor mode; M06-R10 was the
record-only precursor) — implemented and evidenced in `TASKS.md` (M06-R11, then
M06-R10, M06-R9, M06-R8).

Phase detail (M06-R8 harness): a bounded subsystem-isolation debug
harness. `./build/lander_gui --debug-subsystem <name>` launches the game
showing exactly ONE M06 subsystem at a time (11 modes: manual, predictor,
attitude, node-edit, node-executor, transfer-cold, transfer-warm,
autoland-primary, autoland-companion, autoland-cross, ui), with unrelated
flight-computer / debug readouts hidden and a common minimum flight readout
always shown. This is a DEBUG pass, NOT a feature or hardening pass: it must
not fix subsystem defects, change canonical physics / `fixed_dt`, weaken
tests, start M07, or commit. A debug scenario is a startup-only fixture (fixed
per-mode seed, reproducible); once begun the subsystem runs through the normal
simulation / control paths (no runtime teleportation or hidden forces). With
no selector, normal gameplay is byte-for-byte unchanged (verified: headless
final-state line is byte-identical for absent vs `none`). The `transfer-cold`
/ `transfer-warm` modes are meant to EXPOSE the deferred TFD-1/TFD-2 defects,
not fix them. M06-R8 (all R8-01..R8-16 and V01..V06) is implemented and
evidenced in `TASKS.md`. The M06-R9 follow-up — keeping the predicted-trajectory
overlay (the projected path: COAST / PLAN arc + PE / AP / closest-approach
markers, the powered LIVE projection, and the kind legend) visible in
`--debug-subsystem predictor`, the mode's primary graphical observable — is
implemented and evidenced there; it reuses the existing normal-gameplay drawing
calls (no new rendering) and leaves every other mode and normal gameplay
unchanged. The harness is available for the user to inspect one subsystem at a
time. `M06-R7-H01` (the consolidated M06 playtest) stays OPEN as the acceptance
gate; this tool supports that playtest one subsystem at a time. M06-R6 / M06-R7
remain code-complete (see `TASKS.md`).

Test status (re-verified for M06-R11 Pass 1): full build clean; ctest is
**9/11** — the only 2 failing targets are the documented KNOWN POST-M06
transfer-subsystem defects (TFD-1 `lander_tests`, TFD-2
`lander_transfer_warm_tests`), unchanged from the baseline and not part of this
pass. **No new failures.** The four new `lander_debug_subsystem_tests` M06-R11
cases (font coverage; signed body-relative altitude; primary + companion debug
orbit-fixture radius/speed/position/velocity; `reference_label`/`target_label`
identities) all pass, on top of the pre-existing R8/R9/R10 debug-subsystem
tests (selector, seeds, common readout, fixture signatures, determinism,
landing getters). All M06-core targets (landing, flight-computer, predictor,
binary, GUI smoke) remain green. See "M06 known post-M06 transfer-subsystem
defects" in `TASKS.md`.

Open human-verification: `M06-R11-H01` (the M06-R11 Pass 1 visual pass — run
each corrected debug mode and confirm the labels / units now read correctly; see
the launch + visual checklist reported at closeout) is OPEN and gates this
pass. The M06 **predictor subsystem is NOT accepted** (findings PRED-01..08 +
SIM-COLL-01 remain OPEN in the ledger; Pass 1 only corrected the diagnostic
readouts). `M06-R7-H01` (the consolidated M06 playtest) remains the overall
M06 acceptance gate and stays OPEN; the `manual` mode is a provisional pass
(harness only — SIM-COLL-01 supersedes any collision/contact acceptance). The
remaining debug subsystems may be inspected independently of the predictor
deferral (the two transfer modes remain subject to the known TFD-1/TFD-2).
M06-R11 (this Pass 1 diagnostic snapshot) is now committed and pushed to
`origin` at the human-verification blocker at the user's request (2026-10-04) so
the game can be shown / reviewed; M06 as a whole is NOT closed / accepted and the
remaining human items (`M06-R11-H01`, `M06-R7-H01`) stay open. `M06-R12`
(prediction reference-frame architecture) is the next queued bounded group
(registered in `TASKS.md`; not started). `M07` is not active.

Specification:

milestones/M06-flight-computer-and-maneuver-planning.md

Execution ledger:

TASKS.md

## Prior milestone

M05 — Binary moon and first contract loop

State: COMPLETE

Record:

records/M05-binary-moon-contract-loop.md

M05 shipped the compact two-body moon system, simultaneous two-body gravity,
body-relative landing/takeoff on moving tidally-locked bodies, the system-scale
camera, and the first repeating primary <-> companion contract loop.

Specification:

milestones/M05-binary-moon-contract-loop.md

## Prior milestone

M04 — Circular moon and orbital physics

State: COMPLETE

Record:

records/M04-circular-moon-orbital-physics.md

M04 shipped a closed circular moon with radial inverse-square gravity, wrapped
circular terrain and landing sites, local-frame landing checks, O circularize /
F refuel, a corrected moon fill, and a usable local-frame camera. The M04-R1
presentation follow-up (exact player camera anchor, fixed-step render
interpolation, high-resolution timing, fixed screen-space starfield) and the
M04-R2 continuous presentation-time flame animation are complete. All automated
checks pass and all human verification items (M04-R1-H01/H02/H03, M04-R2-H01)
were confirmed on 2026-09-28.

Specification:

milestones/M04-circular-moon-orbital-physics.md

## Prior milestone

M03 — Throttle and camera polish

State: COMPLETE
