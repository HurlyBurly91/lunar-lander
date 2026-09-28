# Lunar Lander Harness A/B

Six independent repositories exist:

    codex/m01
    codex/m02
    codex/m03

    opencode/m01
    opencode/m02
    opencode/m03

Run the same local model/settings for every trial.

Recommended order:

    M03
    M02
    M01

M03 is most likely to expose harness-loop differences quickly.

Give each agent exactly the contents of RUN_PROMPT.txt.

Do not allow either agent to inspect parent directories.

After each run, evaluate externally:

    ./evaluator/run.sh codex m03
    ./evaluator/run.sh opencode m03

Record:

- pass/fail
- wall-clock runtime
- model input tokens
- model output tokens
- tool calls
- build/test attempts
- final diff size
- whether the agent declared success correctly

Run Codex and OpenCode sequentially, not simultaneously.

If both perform similarly on a task, do not waste time repeating it.

If one task separates them materially, repeat only that task two more times per
harness from fresh seed copies.
