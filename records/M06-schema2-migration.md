# M06 schema-1 to schema-2 migration provenance

- Baseline Lunar Lander commit: `f1c24495c7cf623a0a2111c2d30438cb81c11212`.
- Source ledger: `records/M06-schema1-live-TASKS.snapshot.md` (byte-exact
  pre-migration `TASKS.md`, Git blob `1f062adac1b87b906cc7ed4f2795101af77cc0d0`).
- Previous framework: 1.0.1 / schema 1, source `eec7768`.
- Target: schema 2 with provenance-bound migration compatibility.
- Scope: state/evidence metadata only; NO physics, simulation, game source,
  tests, canonical algorithms, existing human decisions, or M07 changes.

## Preservation decisions

All 186 explicitly declared schema-1 live task IDs remain present in the
migrated ledger with their original identifiers and task descriptions. The
original complete ledger is preserved separately without modifying prior
historical records.

The 10 legacy preservation constraints in R18, R19, and R20 that had no
checkbox state are conservatively marked OPEN; this does not claim they were
violated or unimplemented. Their original status-free declarations remain in
the source snapshot.

The following formerly checked requirements or constraints were demoted to
IN_PROGRESS pending actual evidence or human acceptance:

```text
M06-R21-P04
M06-R21-P05
M06-R21-F01-01
M06-R22-05
M06-R22-08
M06-R22-09
M06-R22-10
M06-R23-02
M06-R23-03
```

Legacy `[H]` markers on non-H tasks were changed to OPEN; the original
human-review obligations remain linked to the existing R21/R22 H gates.
All three previously accepted H tasks (R18-H01, R19-H01, R20-H01) retain
their recorded accepted decisions. No unresolved H gate has been accepted.

Historical automated `[x]` results now carry `Evidence-Mode:
HISTORICAL_RECORDED`, the frozen evidence source, and explicit limitations.
This preserves the old assertion without inventing the exact command,
repository fingerprint, or a fresh test run. Historical PASS is not
current-independent verification. New schema-2 verification must use
normal command, oracle, result, and repository-state fields.

Material requirement-to-evidence relationships were reconstructed from the
specific previously recorded test descriptions. They must not be interpreted
as extending a test beyond its original described scope. All local links are
bidirectional. Inherited H gates from R7/R11/R12 reference exact requirements
in the immutable pre-experience ledger through `Coverage-Source` rather than
copying archived task groups back into the bounded live ledger.

The sole typed legacy diagnostic is the pre-existing R18 plume-source
observation, moved without changing its meaning to R18-D01. R18-01 retains
the original statement as a Finding.

The pre-migration header disagreement (`STATUS.md` HUMAN_VERIFICATION,
`TASKS.md` IMPLEMENTATION) was reconciled to HUMAN_VERIFICATION, consistent
with the current status pointer and open R21/R22/R23 human gates. This is
not a declaration that every automated item or deferred presentation
follow-up has completed.

## Gates and follow-up

`M06-R21-H01`, `M06-R22-H01`, and `M06-R23-H01` remain FAILED /
AWAITING_HUMAN as applicable. Inherited M06-R11-H01, R12-H01, R12-H02,
and R7-H01 remain awaiting human verification. R23-D02/D06 and remaining
R23 verification/presentation work are not closed by this migration.

No autoland-primary or new milestone work is authorized by this state
migration. A clean schema validator proves structural consistency only,
not simulation correctness or human acceptance.
