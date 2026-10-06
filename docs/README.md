# Canonical Domain State

This directory stores **current cross-milestone domain truth**: durable rules and facts describing how the system presently works.

Suitable content includes:

- domain or gameplay rules;
- protocols and data formats;
- algorithm definitions;
- subsystem behavior;
- architecture facts and invariants;
- domain terminology and semantics.

Do not use this directory for active task progress, chronological implementation history, milestone closeout records, speculative future plans, or experience/precedent records.

Canonical documents are loaded on demand. Do not read the entire `docs/` tree merely because it exists.

A canonical document may be discovered from the active milestone, `TASKS.md`, the domain being modified, or directly from source code.

Source code may bind an implementation region to its canonical document:

```text
// BEGIN CANONICAL ALGORITHM: <descriptive name>
// Reference: docs/<document>.md

... implementation ...

// END CANONICAL ALGORITHM: <descriptive name>
```

Use the host language's native comment syntax when `//` is not valid, while preserving the three labels.

When working inside a marked region:

1. read the referenced document before changing the implementation;
2. preserve its documented algorithm and invariants unless the current requirement explicitly changes them;
3. move the markers and reference with the implementation if it is moved or decomposed;
4. update the referenced canonical document in the same work when the requirement changes the canonical behavior;
5. do not load unrelated documents;
6. do not remove or weaken the reference because surrounding architecture changes.

Milestones may also declare only the canonical documents relevant to that milestone:

```text
READ:
    docs/...

MAY MODIFY:
    docs/...

MUST PRESERVE:
    docs/...
```

This is not a central registry of all guidance documents.

If a milestone intentionally changes canonical domain behavior, reconcile the affected canonical document before milestone closeout. Otherwise existing canonical rules remain preserved.

## Lunar Lander current canonical documents

The project currently binds gravity/orbital physics and the M06 flight-guidance
algorithms to the named `docs/*.md` files already present in this directory.
Those documents remain authoritative and were not rewritten by the durable-state
migration.
