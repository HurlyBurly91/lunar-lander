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

M06-R23 core fix (`D05` bounded retarget persistence / terminal completion) is
implemented and the automated verification is green; awaiting human
re-verification of `M06-R21-H01`, `M06-R22-H01`, and `M06-R23-H01`. `M06-R23-D02`
and `M06-R23-D06` are deferred follow-ups. The implementation was included in provisional checkpoint `f1c2449`; that commit is not human acceptance.

Schema-2 migration is a metadata/evidence-ledger change only. M06-R21-H01,
M06-R22-H01, and M06-R23-H01 remain unresolved after the migration; deferred
R23 presentation and retarget investigations remain open.
