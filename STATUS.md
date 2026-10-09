# Status

```yaml
Milestone: M06
State: ACTIVE
Phase: HUMAN_VERIFICATION
Active-Request: M06-R23
Previous: M05
Spec: milestones/M06-flight-computer-and-maneuver-planning.md
Ledger: TASKS.md
Experience-Retrieval: ENABLED
```

This file is intentionally minimal. Current execution detail, unresolved
human gates, evidence, blockers, and supersession links live in `TASKS.md`.
Historical pre-migration status is preserved in
`records/M06-pre-experience-STATUS.snapshot.md`.

M06-R23 is active in `HUMAN_VERIFICATION`. The `D05` retarget root-cause fix
(valid but unaccepted cache must not be reported as an accepted route) is in
place, and the 2026-10-09 automated follow-up work is complete: the
terminal-completion check now runs as an O(1) per-step arrival-shell test in
`TransferMidcourse::after_step` (R23-04 scenarios pass); `D02`'s headless
retarget-lifecycle trace verifies the 120 s window (584 generations = 428
holds + 156 retargets; 1445 burn-boundary-guard short-circuits; 1 epoch
change; clean completion at 73.81 s; no post-completion activity); `D06`
decoupled the presentation (display/banner/numeric values republish at most
every 2.5 s, banner shown 3.0 s, replaced route kept on screen faded for
8 s, camera zoom held after the initial fit); and the R23-05 full-encounter
oracle was reworked (WARM completes at 73.81 s, closest approach 42.1 m,
minimum clearance 4.49 m, 14 burns, fuel 96/1000, no crash; the COLD baseline
never completes). The verification battery returns 12/12 ctest (V14-C did not
reproduce). This follow-up work is uncommitted on top of provisional
checkpoint `f1c2449`; that commit is not human acceptance.

The three unresolved human gates remain: `M06-R21-H01` (FAILED on
2026-10-09), `M06-R22-H01` (FAILED on 2026-10-09), and `M06-R23-H01`
(awaiting human review, including the presentation-stability observation).
Schema-2 migration remains a metadata/evidence-ledger change only.
