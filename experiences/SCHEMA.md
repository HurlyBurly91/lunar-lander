# Experience Schema

Canonical experience store:

```text
experiences/experiences.jsonl
```

Canonical retrieval telemetry:

```text
experiences/retrievals.jsonl
```

Both are JSON Lines: one JSON object per line.

The experience store contains **selected evaluated precedents**, not raw conversation history and not every execution event.

## Experience object

Recommended fields:

```json
{
  "id": "E0001",
  "kind": "episode",
  "domain": ["rendering"],
  "tags": ["human-verification", "presentation"],
  "origin": {
    "milestone": "M01",
    "requests": ["M01-R2"]
  },
  "context": {
    "phase": "HUMAN_VERIFICATION",
    "observation": [
      "automated proxy passed",
      "human-visible requirement failed"
    ]
  },
  "decision": {
    "type": "requirement_refinement",
    "summary": "Validate the observable requirement rather than the internal proxy."
  },
  "action": {
    "summary": "Changed implementation and verification to target observable behavior."
  },
  "outcome": {
    "status": "SUCCESS_WITH_FOLLOWUP",
    "observations": [
      "observable behavior improved"
    ]
  },
  "lesson": {
    "prefer": [
      "verify user-visible acceptance criteria directly where practical"
    ],
    "avoid": [
      "assuming an internal proxy guarantees presentation quality"
    ]
  },
  "applies_when": [
    "automated proxy succeeds while human-visible requirement fails"
  ],
  "evidence": {
    "tasks": ["M01-R1-H01", "M01-R2-01", "M01-R2-H01"],
    "source": ["automated-test", "human-verification"]
  },
  "confidence": {
    "level": "high",
    "basis": ["explicit human observation", "automated regression evidence"]
  },
  "status": "active"
}
```

## Stable experience IDs

Use:

```text
E0001
E0002
E0003
...
```

Do not encode milestone identity into the primary experience ID.

Link chronology through `origin`.

IDs never silently change meaning.

## Kinds

Initial kinds:

```text
episode
heuristic
```

An `episode` is a specific evaluated case.

A `heuristic` is a generalized pattern supported by unusually strong or repeated evidence.

Do not create a separate heuristic subsystem initially.

## Outcome vocabulary

Recommended values:

```text
SUCCESS
SUCCESS_WITH_FOLLOWUP
PARTIAL_SUCCESS
FAILURE
INCONCLUSIVE
SUPERSEDED
```

## Experience status

Use:

```text
active
stale
superseded
```

Superseded experiences remain in the store for provenance but should be excluded from normal retrieval.

Stale experiences may still be useful but should normally rank below directly applicable active ones.

## Confidence

Use coarse evidence quality:

```text
low
medium
high
```

Confidence is not a calibrated probability.

## Retention test

Before appending an experience ask:

```text
Did something materially informative happen?
        ↓ yes
Was the outcome evaluated?
        ↓ yes
Is there plausible future reuse value?
        ↓ yes
retain
```

Otherwise do not add a precedent.

## Retrieval telemetry object

Example:

```json
{
  "retrieval_id": "X0001",
  "milestone": "M02",
  "request": "M02-R3",
  "decision": "choose render smoothing strategy",
  "retrieved": ["E0002", "E0007"],
  "used": ["E0002"],
  "assessment": {
    "E0002": "helpful",
    "E0007": "neutral"
  },
  "decision_outcome": "SUCCESS"
}
```

The exact telemetry schema may evolve.

Preserve these concepts:

```text
retrieved
used
helpful / neutral / misleading
resulting decision outcome
```

Telemetry is diagnostic and non-authoritative.
