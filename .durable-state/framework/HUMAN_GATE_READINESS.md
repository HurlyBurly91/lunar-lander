# Human-Gate Readiness

Status: **normative for the experience-augmented framework**

`HUMAN_VERIFICATION` is an established execution state, not a statement of
future intent. Enter it only when the next executable gate is genuinely owned by
a human and every machine-owned prerequisite for that gate is complete.

The failure this rule prevents is:

```text
human verification is the eventual destination
    !=
repository is ready for human verification now
```

A verified implementation may still require checkpointing, publication,
artifact preparation, environment setup, or another automated precondition
before the human can inspect the exact intended target. Those steps remain
machine-owned work.

## Required readiness conditions

When `Phase: HUMAN_VERIFICATION`:

1. A specific pending `[H]` task is selected.
2. `Active-Request` identifies the request group containing that selected gate.
3. Every transitive `Requires:` and `Blocked-By:` prerequisite of the selected
   gate is `VERIFIED` or `SUPERSEDED`.
4. No unresolved `Dxx` or `Vxx` task remains in the active request unless it is
   explicitly downstream of the selected human gate.
5. The artifact, environment, build, commit, or deployed/published revision the
   human will inspect is durably identified.
6. Repository-specific publication or synchronization requirements are complete
   when the human check depends on committed, pushed, deployed, or otherwise
   externally materialized work.
7. `STATUS.md`, `TASKS.md`, repository state, and the actual next action all
   agree that human judgment is next.

If any condition is false, remain in the phase that describes the outstanding
machine work:

```text
IMPLEMENTATION
AUTOMATED_VERIFICATION
FOLLOW_UP
```

Do not add a generic `PUBLICATION` phase merely to represent this boundary.
Publication/checkpoint work is an ordinary `Dxx`/`Vxx` prerequisite whose
completion makes the human gate ready.

## Selecting the gate

Write the selected gate in `STATUS.md`:

```yaml
State: ACTIVE
Phase: HUMAN_VERIFICATION
Active-Request: M18-R1
Next-Gate: M18-R1-H01
```

For compatibility, the validator may infer the selected gate when exactly one
pending `[H]` task exists. When multiple pending human gates exist,
`Next-Gate` is required.

`Next-Gate` must reference an existing pending `[H]` task. It may not point to:

- an open or completed requirement;
- a derived or automated-verification task;
- an already accepted human task;
- an archived task that is not live in the current ledger;
- a future gate whose prerequisites are unresolved.

`Active-Request` is the request currently being executed. Therefore, during
`HUMAN_VERIFICATION`, it must match the selected gate's `Mxx-Rn` request group.
The previously completed engineering follow-up may remain in records and task
history, but it is no longer the active request once the human gate is ready.

## Modelling checkpoint and publication prerequisites

When a human must test committed, pushed, deployed, packaged, or otherwise
published work, model that boundary explicitly. Prefer an automated verification
task because it records actual commands and repository state:

```markdown
- [x] M18-R13-V02 publish and verify the human-test checkpoint
  Covers:
    - M18-R13-P02
  Gate: INTEGRATION
  Command: <commit/push/deploy and independent equality checks>
  Oracle: integration-or-end-to-end
  Expected: the exact human-test revision is durably available
  Result: PASS - <actual revision and remote/deployment equality>
  Repository-State: HEAD=<commit>; WORKTREE=CLEAN
  Limitations:
    - does not establish the human-visible acceptance result

- [H] M18-R1-H01 inspect the exact published target
  Requires:
    - M18-R13-V02
  Covers:
    - M18-R1-18
  Gate: HUMAN
```

A `Dxx` checkpoint task may also be used, but a consequential publication claim
should normally have a `Vxx` oracle recording the actual revision and external
state. Do not mark a publication prerequisite complete based only on intended
commands or prose such as “publication finishing.”

## Downstream work

Machine work that explicitly depends on the selected human decision may remain
open:

```markdown
- [ ] M18-R1-D04 implement accepted visual feedback
  Requires:
    - M18-R1-H01
```

This is downstream work and does not make the gate unready. Unresolved work with
no dependency on the selected gate is pre-gate work and prevents entry into
`HUMAN_VERIFICATION`.

## Human feedback re-entry

If the human gate fails or creates new implementation work:

```text
ACTIVE / HUMAN_VERIFICATION
    -> persist the new request or follow-up
    -> FOLLOW_UP / IMPLEMENTATION
    -> AUTOMATED_VERIFICATION
    -> satisfy checkpoint/publication prerequisites
    -> HUMAN_VERIFICATION
```

Do not leave the project in `HUMAN_VERIFICATION` while machine-owned correction,
verification, checkpointing, or publication is actively occurring.

## Deterministic validation

Schema-2 framework 1.1.3 adds these checks:

```text
E131  no unambiguous pending human gate is selected
E132  Next-Gate does not reference a pending [H] task
E133  Active-Request does not match the selected gate's request
E134  selected gate has an unresolved transitive prerequisite
E135  active request contains unresolved pre-gate D/V work
E136  selected gate prerequisite graph contains a cycle
```

The validator deliberately does not infer whether a Git push, deployment, or
external publication is required. Repository policy must express that
requirement as a task and link it through `Requires:`. Once expressed, the
validator enforces the dependency closure.

## Core principle

At every layer, state names describe established reality:

```text
transition intent
    !=
transition completed

human gate planned
    !=
human gate ready
```

Advance durable phase only after the relevant postconditions are established.
