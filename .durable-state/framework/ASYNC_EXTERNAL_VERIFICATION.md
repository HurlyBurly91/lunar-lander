# Asynchronous External Verification

Status: **normative for the experience-augmented framework**

External verification may continue independently after the coding agent has
submitted work. Examples include hosted CI, remote build farms, deployment
checks, hardware tests, external evaluators, and provider-side validation.

The framework distinguishes:

```text
verification is running elsewhere
    !=
the coding agent must remain alive and poll it
```

Keep the existing milestone phase `ACTIVE / AUTOMATED_VERIFICATION` and the
existing `Vxx` task states. Do not add a waiting phase, a duplicate ledger, or a
scheduler to the durable-state framework.

## When to suspend instead of poll

A local command owned by the current tool process may run synchronously when
that is the appropriate verification mechanism. An external run is different:
it already has an independent identity and continues without the agent.

After starting or discovering a required external run, one immediate bounded
status query is reasonable to capture its identity and current state. If it is
still non-terminal and completion is not imminent, do not hold a long-running
agent/tool call solely to sleep and poll. Persist the wait contract, checkpoint
state, and yield execution.

## Pending external-verification contract

Represent a live external wait on the existing `Vxx` task:

```markdown
- [~] M04-R3-V03 required hosted CI
  Covers:
    - M04-R3-01
  Gate: INTEGRATION
  Verification-Mode: EXTERNAL_ASYNC
  External-Provider: github-actions
  External-Run-ID:
    - 38014888880
    - 38014888798
  Target-Identity: commit:f79f4f76eba98ca45a66cb33aafb7014476b866a
  Required-Checks:
    - durable-state-schema-2
    - tests
  External-Status: IN_PROGRESS
  Resume-Mechanism: SCHEDULED
  Command: gh run view <recorded-run-id> --json status,conclusion,headSha,jobs
  Oracle: integration-or-end-to-end
  Expected: every recorded required check reaches a successful terminal result for the exact target identity
  Suspension-Checkpoint: HEAD=f79f4f76eba98ca45a66cb33aafb7014476b866a; WORKTREE=CLEAN
  Timeout-Policy: recheck after the expected provider duration; report timeout rather than infer success
  Limitations:
    - pending external execution is not verification evidence
```

Required pending fields are:

```text
Verification-Mode: EXTERNAL_ASYNC
External-Provider
External-Run-ID
Target-Identity
Required-Checks
External-Status: QUEUED | IN_PROGRESS
Resume-Mechanism: MANUAL | SCHEDULED | EVENT
Command
Oracle
Expected
Suspension-Checkpoint
Limitations
```

`Timeout-Policy` is recommended. A project may use a deadline, a bounded retry
cadence, escalation, or an explicit justified no-deadline policy.

The external ID must be the provider's stable execution identity, not a vague
reference such as “latest CI.” `Target-Identity` must identify the immutable
commit, artifact digest, deployment, build, or equivalent object actually being
tested. `Required-Checks` names the complete set whose result controls the gate.

`Suspension-Checkpoint` uses the same repository-state syntax as verification
evidence:

```text
HEAD=<commit>; WORKTREE=CLEAN
HEAD=<commit>; DIFF-SHA256=<material-worktree fingerprint>
```

The pending contract is durable coordination state, not a PASS result. Do not
record `Result: PASS`, `External-Conclusion`, or mark the task `[x]` while the
external run is queued or in progress.

## Suspension boundary

Before yielding execution:

1. verify that the external run targets the intended revision/artifact;
2. persist provider, exact run ID, target identity, required checks, query
   command, expected terminal condition, resume mechanism, limitations, and
   suspension checkpoint;
3. keep the milestone in `ACTIVE / AUTOMATED_VERIFICATION`;
4. keep the external `Vxx` task `[~] IN_PROGRESS`;
5. link any later human gate or closeout dependency through `Requires:`;
6. run deterministic validation;
7. checkpoint and publish durable state when repository policy requires it;
8. release the coding agent.

Scheduling belongs to an external harness such as a timer, webhook receiver,
queue, or agent scheduler. The durable-state repository defines what must be
resumed and how to validate it; it does not keep a worker alive.

## Resume and applicability

On resume:

1. reconstruct repository state normally;
2. locate the exact `[~]` `EXTERNAL_ASYNC` verification task;
3. query the recorded provider execution by its exact ID;
4. verify the returned repository/project, target identity, workflow/check
   identity, and required-check set;
5. treat duplicate resumes or duplicate completion notifications idempotently;
6. if still non-terminal, update only material wait metadata when useful,
   checkpoint if needed, and yield again;
7. if terminal, preserve the actual conclusion and continue according to the
   rules below.

Do not silently substitute a rerun, a newer branch run, or another commit. If a
provider creates a replacement execution, record the new ID explicitly and
explain which run supersedes or complements the earlier one.

At-least-once scheduler or webhook delivery must be safe. Reprocessing the same
terminal event may confirm existing evidence but must not duplicate state
transitions, commits, deployments, or human notifications.

## Successful completion

A successful external result becomes ordinary verified evidence:

```markdown
- [x] M04-R3-V03 required hosted CI
  ...pending identity fields retained...
  External-Status: COMPLETED
  External-Conclusion: SUCCESS
  Command: gh run view <recorded-run-id> --json status,conclusion,headSha,jobs
  Oracle: integration-or-end-to-end
  Expected: every recorded required check reaches a successful terminal result for the exact target identity
  Result: PASS <actual checks and conclusions>
  Repository-State: HEAD=<applicable commit>; WORKTREE=CLEAN
  Limitations:
    - <what the external checks do not establish>
```

Before marking `[x]`, independently confirm that the provider result applies to
the recorded target and that every required check has a permitted successful
conclusion. A merely completed run is not necessarily successful.

## Failure, cancellation, timeout, or staleness

For a terminal non-success:

- retain `External-Status: COMPLETED`;
- record `External-Conclusion: FAILURE | CANCELLED | TIMED_OUT |
  ACTION_REQUIRED | NEUTRAL | SKIPPED | STALE` as actually reported and
  interpreted by project policy;
- record `Result: FAIL ...` or another truthful non-PASS result;
- do not mark the `Vxx` task `[x]`;
- preserve logs/artifact references needed to diagnose the failure;
- return to `FOLLOW_UP` or `IMPLEMENTATION` when correction is required;
- persist a new derived repair task when the failure creates distinct work;
- rerun or replace external verification only through an explicit new or
  updated execution identity.

Missing, inaccessible, contradictory, or mismatched provider evidence is
indeterminate, never passing.

## Human-gate and closeout interaction

Required external checks are machine-owned prerequisites. A human gate that
depends on them must name the external `Vxx` task in `Requires:`. The framework
must not enter `HUMAN_VERIFICATION`, complete a milestone, merge, deploy, or
claim release readiness while required external verification remains pending or
non-successful.

## Scope boundary

The framework owns:

```text
pending-wait representation
identity and applicability requirements
checkpoint/yield semantics
resume validation
success/failure state transitions
human-gate and closeout dependencies
```

The project owns test selection, CI topology, job parallelization, caching, and
provider-specific success policy. The orchestration layer owns timers, webhooks,
queues, credentials, notifications, and process resumption.
