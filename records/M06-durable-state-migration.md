# M06 durable-state migration — experience-augmented architecture

Date: 2026-10-06

Source project commit: `508344fbfafb81f74d92964111d60bb9de954147`

Reference architecture: `HurlyBurly91/durable-state-machine` at commit
`6b3ebc329de5355f6a06c3725438001999786cc8`, specifically
`DURABLE_STATE_MACHINE_EXPERIENCE_AUGMENTED.md` and
`templates/experience-augmented/`.

## Purpose

Migrate the existing Lunar Lander durable-state machinery in place without
changing application behavior or replacing project-specific truth.

## Preserved project truth

- M06 remains the active milestone.
- Current execution remains M06-R18 with implementation and automated
  verification complete and human gate `M06-R18-H01` unresolved.
- Older unresolved human gates `M06-R11-H01`, `M06-R12-H01`,
  `M06-R12-H02`, and `M06-R7-H01` remain unresolved.
- Existing stable request/requirement/task IDs retain their historical meanings.
- M06-R13, M06-R14, and M06-R15/R16/R17 accepted checkpoints and commit
  provenance remain available.
- The sole current ctest red, M06-R6-V14 / V14-C cross-body soft-land, remains
  visible and unresolved.
- PRED-01..PRED-08, SIM-COLL-01, TFD-1, TFD-2, and deferred camera/UI zoom
  polish remain open/deferred; nothing was inferred complete.
- Existing milestone specifications, canonical domain docs, historical records,
  source/document markers, Git rules, text-only-model restrictions, and SDL3
  policy remain intact.
- `PROJECT.md` was deliberately left unchanged because it already represented
  product/roadmap truth rather than execution bookkeeping.

## Architectural changes

- `STATUS.md` is now a minimal pointer using separate `State: ACTIVE` and
  `Phase: HUMAN_VERIFICATION`; task-level waiting is represented with `[H]`.
- `TASKS.md` is now bounded to resume-critical live execution state. Exact
  pre-migration contents are preserved in immutable snapshot records and Git
  history rather than remaining in an ever-growing active ledger.
- `AGENTS.md` now uses semantic investigative transactions: routine captures,
  samples, analyzer runs, and parameter trials stay under an existing request /
  derived task until a durable decision boundary changes.
- Durable checkpointing is based on reconstruction cost, not context growth or
  each inner-loop observation.
- `records/` remains historical provenance rather than a per-trial laboratory
  notebook.
- Added the optional/advisory `experiences/` precedent-memory layer and minimal
  retrieval telemetry. Both canonical JSONL stores begin empty; migration does
  not fabricate precedent.
- Experience retrieval is experimentally enabled in `STATUS.md`, but is not
  required to reconstruct execution and can be disabled without restructuring
  the baseline durable-state system.
- `RUN_PROMPT.txt` now reconstructs current state, loads canonical docs
  selectively, reuses IDs within bounded investigation, stores exhaustive
  measurements outside durable prose, checkpoints semantic decisions, and
  retrieves experiences only at useful strategic boundaries.

## Exact pre-migration snapshots

- `records/M06-pre-experience-STATUS.snapshot.md`
- `records/M06-pre-experience-TASKS.snapshot.md`

These snapshots are preservation artifacts, not alternate active ledgers.

## Validation

- No application source, tests, CMake, physics, controller, renderer, or
  gameplay behavior was changed by this migration.
- Current milestone identity, M06-R18 active request, and all unresolved human
  gates were preserved.
- Stable historical IDs remain recoverable verbatim from the snapshot and Git
  history; no ID was renumbered or repurposed.
- Project-specific canonical docs and source-to-document preservation policy
  remain intact.
- Existing completed milestone records were not modified.
- Baseline resume remains possible with `experiences/` ignored or experience
  retrieval disabled.
- `experiences/experiences.jsonl` and `experiences/retrievals.jsonl` are
  empty at migration.
