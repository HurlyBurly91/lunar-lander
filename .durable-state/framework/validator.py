#!/usr/bin/env python3
"""Deterministic validator for durable-state schema 2.

The validator intentionally checks repository-resident state and evidence
relationships only. It does not execute project commands or decide whether a
human judgment is correct.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import stat
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

SUPPORTED_SCHEMA_VERSION = "2"

MILESTONE_RE = re.compile(r"^M\d+$")
REQUEST_RE = re.compile(r"^(M\d+-R\d+)$")
TASK_ID_PATTERN = r"M\d+-R\d+(?:-F\d+)?-(?:(?:P|D|V|H)\d+|\d+(?:-\d+)*)"
TASK_ID_RE = re.compile(rf"^{TASK_ID_PATTERN}$")
TASK_REF_RE = re.compile(rf"\b({TASK_ID_PATTERN})\b")
REQUEST_HEADING_RE = re.compile(r"^##\s+(M\d+-R\d+)\s+(?:—|-)\s+(.+?)\s*$")
FOLLOWUP_HEADING_RE = re.compile(r"^###\s+(M\d+-R\d+-F\d+)\s+(.+?)\s*$")
TASK_LINE_RE = re.compile(
    rf"^\s*-\s+\[([ ~?Hx-])\]\s+({TASK_ID_PATTERN})(?:\s+(.*?))?\s*$"
)
FIELD_RE = re.compile(r"^\s*(?:-\s+)?([A-Za-z][A-Za-z0-9-]*):\s*(.*?)\s*$")
LIST_ITEM_RE = re.compile(r"^\s*-\s+(.+?)\s*$")
HEADER_FIELD_RE = re.compile(r"^([A-Za-z][A-Za-z0-9-]*):\s*(.*?)\s*$")

ALLOWED_MILESTONE_STATES = {"NOT_STARTED", "ACTIVE", "BLOCKED", "COMPLETE"}
ALLOWED_PHASES = {
    "IMPLEMENTATION",
    "AUTOMATED_VERIFICATION",
    "HUMAN_VERIFICATION",
    "FOLLOW_UP",
}
ALLOWED_GATES = {
    "FUNCTIONAL",
    "CONSTRAINT",
    "INVARIANT",
    "INTEGRATION",
    "PRESENTATION",
    "HUMAN",
}
ALLOWED_ORACLES = {
    "independent-existing-regression",
    "property-or-invariant",
    "integration-or-end-to-end",
    "static-analysis",
    "same-change-generated-test",
    "external-reference-comparison",
    "human-observation",
}
ALLOWED_CONCLUSION_STATUSES = {
    "OBSERVED",
    "DERIVED",
    "VERIFIED",
    "HUMAN_ACCEPTED",
    "UNKNOWN",
}
REFERENCE_FIELDS = {
    "Requires",
    "Blocked-By",
    "Supersedes",
    "Superseded-By",
    "Verified-By",
    "Covers",
    "Conclusion-Evidence",
}
CONCLUSION_FIELDS = {
    "Conclusion",
    "Conclusion-Status",
    "Conclusion-Scope",
    "Conclusion-Evidence",
    "Conclusion-Limitations",
    "Conclusion-Recheck-On",
}
STRONG_NEGATIVE_RE = re.compile(
    r"\b(impossible|cannot|never|exhausted|no\s+solution|does\s+not\s+work|"
    r"cannot\s+originate|ruled\s+out)\b",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class FieldValue:
    value: str
    line: int


@dataclass
class Task:
    task_id: str
    marker: str
    title: str
    request: str
    line: int
    fields: dict[str, list[FieldValue]] = field(default_factory=dict)

    @property
    def kind(self) -> str:
        suffix = self.task_id.rsplit("-", 1)[-1]
        return suffix[0] if suffix[0] in "PDVH" else "R"

    def values(self, name: str) -> list[FieldValue]:
        return self.fields.get(name, [])

    def strings(self, name: str) -> list[str]:
        return [item.value for item in self.values(name)]

    def first(self, name: str) -> FieldValue | None:
        values = self.values(name)
        return values[0] if values else None


@dataclass(frozen=True)
class Diagnostic:
    severity: str
    code: str
    path: str
    line: int
    message: str

    def to_dict(self) -> dict[str, object]:
        return {
            "severity": self.severity,
            "code": self.code,
            "path": self.path,
            "line": self.line,
            "message": self.message,
        }


@dataclass
class ParsedLedger:
    path: Path
    header: dict[str, FieldValue]
    requests: dict[str, int]
    tasks: list[Task]


class Validator:
    def __init__(
        self,
        root: Path,
        *,
        strict: bool = False,
        target_schema: str | None = None,
    ) -> None:
        self.root = root.resolve()
        self.strict = strict
        self.target_schema = target_schema
        self.diagnostics: list[Diagnostic] = []
        self._git_head: str | None = None
        self._material_dirty: bool | None = None
        self._worktree_fingerprints: dict[str, str] = {}
        self._archive_cache: dict[str, str] = {}

    def add(
        self,
        severity: str,
        code: str,
        path: Path | str,
        line: int,
        message: str,
    ) -> None:
        try:
            display_path = str(Path(path).resolve().relative_to(self.root))
        except (ValueError, OSError):
            display_path = str(path)
        self.diagnostics.append(
            Diagnostic(severity=severity, code=code, path=display_path, line=line, message=message)
        )

    def error(self, code: str, path: Path | str, line: int, message: str) -> None:
        self.add("ERROR", code, path, line, message)

    def warning(self, code: str, path: Path | str, line: int, message: str) -> None:
        self.add("WARNING", code, path, line, message)

    def validate(self) -> list[Diagnostic]:
        self._validate_manifest_schema()

        status_path = self.root / "STATUS.md"
        tasks_path = self.root / "TASKS.md"
        status_header = self._parse_yaml_header(status_path)
        ledger = self._parse_tasks(tasks_path)

        if status_header is None or ledger is None:
            return self._sorted_diagnostics()

        self._validate_headers(status_path, status_header, ledger)
        task_by_id = self._validate_task_identity(ledger)
        self._validate_references(ledger, task_by_id)
        self._validate_evidence_coverage(ledger, task_by_id)
        self._validate_verification_tasks(ledger, task_by_id)
        self._validate_conclusions(ledger, task_by_id)
        self._validate_supersession(ledger, task_by_id)
        self._validate_file_bindings(ledger)
        self._validate_completion(ledger)

        return self._sorted_diagnostics()

    def _sorted_diagnostics(self) -> list[Diagnostic]:
        return sorted(
            self.diagnostics,
            key=lambda item: (item.path, item.line, item.severity, item.code, item.message),
        )

    def failed(self) -> bool:
        if any(item.severity == "ERROR" for item in self.diagnostics):
            return True
        return self.strict and any(item.severity == "WARNING" for item in self.diagnostics)

    def _validate_manifest_schema(self) -> None:
        manifest = self.root / ".durable-state" / "MANIFEST"
        expected = self.target_schema or SUPPORTED_SCHEMA_VERSION
        if not manifest.exists():
            return
        values: dict[str, tuple[str, int]] = {}
        try:
            lines = manifest.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            self.error("E001", manifest, 1, f"cannot read manifest: {exc}")
            return
        for line_number, raw in enumerate(lines, start=1):
            if not raw or raw.lstrip().startswith("#") or "=" not in raw:
                continue
            key, value = raw.split("=", 1)
            key = key.strip()
            value = value.strip()
            if key in values:
                self.error("E002", manifest, line_number, f"duplicate manifest field {key}")
            values[key] = (value, line_number)
        schema = values.get("SCHEMA_VERSION")
        if schema is None:
            self.error("E003", manifest, 1, "manifest is missing SCHEMA_VERSION")
        elif schema[0] != expected and self.target_schema is None:
            self.error(
                "E004",
                manifest,
                schema[1],
                f"installed schema {schema[0]} is incompatible with validator schema {expected}",
            )

    def _parse_yaml_header(self, path: Path) -> dict[str, FieldValue] | None:
        if not path.is_file():
            self.error("E010", path, 1, "required file is missing")
            return None
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as exc:
            self.error("E011", path, 1, f"cannot read UTF-8 file: {exc}")
            return None

        in_yaml = False
        found_yaml = False
        header: dict[str, FieldValue] = {}
        for line_number, raw in enumerate(lines, start=1):
            stripped = raw.strip()
            if not in_yaml:
                if stripped == "```yaml":
                    in_yaml = True
                    found_yaml = True
                continue
            if stripped == "```":
                break
            match = HEADER_FIELD_RE.match(stripped)
            if not match:
                if stripped:
                    self.error("E012", path, line_number, "invalid YAML-header line")
                continue
            key, value = match.groups()
            if key in header:
                self.error("E013", path, line_number, f"duplicate header field {key}")
            header[key] = FieldValue(value=value, line=line_number)

        if not found_yaml:
            self.error("E014", path, 1, "missing leading ```yaml state header")
            return None
        return header

    def _parse_tasks(self, path: Path) -> ParsedLedger | None:
        header = self._parse_yaml_header(path)
        if header is None:
            return None
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as exc:
            self.error("E015", path, 1, f"cannot read UTF-8 file: {exc}")
            return None

        requests: dict[str, int] = {}
        tasks: list[Task] = []
        current_request: str | None = None
        current_task: Task | None = None
        current_list_field: str | None = None
        in_inherited_section = False
        in_fence = False

        for line_number, raw in enumerate(lines, start=1):
            stripped = raw.strip()
            if stripped.startswith("```"):
                in_fence = not in_fence
                continue
            if in_fence:
                continue

            request_match = REQUEST_HEADING_RE.match(raw) or FOLLOWUP_HEADING_RE.match(raw)
            if request_match:
                current_request = request_match.group(1)
                in_inherited_section = False
                if current_request in requests:
                    self.error(
                        "E020", path, line_number, f"duplicate request heading {current_request}"
                    )
                requests[current_request] = line_number
                current_task = None
                current_list_field = None
                continue

            if raw.startswith("## "):
                in_inherited_section = raw.strip() == "## Inherited unresolved human gates"
                current_request = None
                current_task = None
                current_list_field = None
                continue

            task_match = TASK_LINE_RE.match(raw)
            if task_match:
                marker, task_id, title = task_match.groups()
                request = task_id.rsplit("-", 1)[0]
                current_task = Task(
                    task_id=task_id,
                    marker=marker,
                    title=(title or "").strip(),
                    request=request,
                    line=line_number,
                )
                tasks.append(current_task)
                current_list_field = None
                if current_request is None and not (in_inherited_section and current_task.kind == "H"):
                    self.error("E021", path, line_number, f"task {task_id} is outside a request group")
                elif current_request is not None and current_request != request and not (
                    current_request.startswith(request + "-F")
                ):
                    self.error(
                        "E022",
                        path,
                        line_number,
                        f"task {task_id} is under request {current_request}",
                    )
                continue

            if current_task is None:
                continue

            field_match = FIELD_RE.match(raw)
            if field_match:
                key, value = field_match.groups()
                current_list_field = key
                if value:
                    current_task.fields.setdefault(key, []).append(
                        FieldValue(value=value, line=line_number)
                    )
                else:
                    current_task.fields.setdefault(key, [])
                continue

            item_match = LIST_ITEM_RE.match(raw)
            if item_match and current_list_field:
                current_task.fields.setdefault(current_list_field, []).append(
                    FieldValue(value=item_match.group(1), line=line_number)
                )
                continue

            if stripped and not raw.startswith((" ", "\t")):
                current_task = None
                current_list_field = None

        return ParsedLedger(path=path, header=header, requests=requests, tasks=tasks)

    @staticmethod
    def _placeholder(value: str) -> bool:
        normalized = value.strip()
        return (
            not normalized
            or ("<" in normalized and ">" in normalized)
            or normalized.upper() in {"TBD", "TODO", "PLACEHOLDER"}
        )

    @staticmethod
    def _kind_from_id(task_id: str) -> str:
        suffix = task_id.rsplit("-", 1)[-1]
        return suffix[0] if suffix[0] in "PDVH" else "R"

    def _required_header(
        self, path: Path, header: dict[str, FieldValue], key: str
    ) -> FieldValue | None:
        item = header.get(key)
        if item is None:
            self.error("E030", path, 1, f"missing required header field {key}")
            return None
        if self._placeholder(item.value):
            self.error("E031", path, item.line, f"header field {key} is unresolved")
        return item

    def _validate_headers(
        self,
        status_path: Path,
        status: dict[str, FieldValue],
        ledger: ParsedLedger,
    ) -> None:
        tasks_path = ledger.path
        for key in ("Milestone", "State"):
            self._required_header(status_path, status, key)
            self._required_header(tasks_path, ledger.header, key)

        milestone = status.get("Milestone")
        state = status.get("State")
        if milestone and not MILESTONE_RE.fullmatch(milestone.value):
            self.error("E032", status_path, milestone.line, "Milestone must match M<number>")
        if state and state.value not in ALLOWED_MILESTONE_STATES:
            self.error(
                "E033",
                status_path,
                state.line,
                f"invalid milestone State {state.value!r}",
            )

        for key in ("Milestone", "State", "Phase", "Active-Request"):
            left = status.get(key)
            right = ledger.header.get(key)
            if (left is None) != (right is None):
                path = status_path if left is None else tasks_path
                line = 1 if left is None and right is None else (right.line if left is None else left.line)
                self.error("E034", path, line, f"{key} must appear in both STATUS.md and TASKS.md")
            elif left and right and left.value != right.value:
                self.error(
                    "E035",
                    tasks_path,
                    right.line,
                    f"{key}={right.value!r} disagrees with STATUS.md value {left.value!r}",
                )

        state_value = state.value if state else None
        phase = status.get("Phase")
        active_request = status.get("Active-Request")
        if state_value in {"ACTIVE", "BLOCKED"}:
            if phase is None:
                self.error("E036", status_path, 1, f"{state_value} state requires Phase")
            elif phase.value not in ALLOWED_PHASES:
                self.error("E037", status_path, phase.line, f"invalid Phase {phase.value!r}")
            if active_request is None:
                self.error("E038", status_path, 1, f"{state_value} state requires Active-Request")
            elif not REQUEST_RE.fullmatch(active_request.value):
                self.error(
                    "E039",
                    status_path,
                    active_request.line,
                    "Active-Request must match M<number>-R<number>",
                )
            elif active_request.value not in ledger.requests:
                self.error(
                    "E040",
                    status_path,
                    active_request.line,
                    f"Active-Request {active_request.value} has no request heading in TASKS.md",
                )
        elif state_value in {"NOT_STARTED", "COMPLETE"}:
            if phase is not None:
                self.warning(
                    "W030",
                    status_path,
                    phase.line,
                    f"{state_value} state normally omits Phase",
                )
            if active_request is not None:
                self.warning(
                    "W031",
                    status_path,
                    active_request.line,
                    f"{state_value} state normally omits Active-Request",
                )

        if milestone and active_request and not active_request.value.startswith(milestone.value + "-"):
            self.error(
                "E041",
                status_path,
                active_request.line,
                f"Active-Request {active_request.value} does not belong to {milestone.value}",
            )

        for key in ("Spec", "Ledger"):
            item = status.get(key)
            if not item or self._placeholder(item.value):
                continue
            target = self.root / item.value
            if not target.exists():
                self.error("E042", status_path, item.line, f"{key} target does not exist: {item.value}")

    def _validate_task_identity(self, ledger: ParsedLedger) -> dict[str, Task]:
        task_by_id: dict[str, Task] = {}
        milestone = ledger.header.get("Milestone")
        state = ledger.header.get("State")

        for request, line in ledger.requests.items():
            if milestone and not request.startswith(milestone.value + "-"):
                self.error(
                    "E050",
                    ledger.path,
                    line,
                    f"request {request} does not belong to active milestone {milestone.value}",
                )

        for task in ledger.tasks:
            if task.task_id in task_by_id:
                self.error(
                    "E051",
                    ledger.path,
                    task.line,
                    f"duplicate task ID {task.task_id}; first defined on line {task_by_id[task.task_id].line}",
                )
            else:
                task_by_id[task.task_id] = task

            if task.request not in ledger.requests and not (
                task.kind == "H" and task.values("Coverage-Source")
            ):
                self.error(
                    "E052", ledger.path, task.line, f"task {task.task_id} has no matching request heading"
                )
            if milestone and not task.task_id.startswith(milestone.value + "-"):
                self.error(
                    "E053",
                    ledger.path,
                    task.line,
                    f"task {task.task_id} does not belong to active milestone {milestone.value}",
                )
            if self._placeholder(task.title):
                self.error("E054", ledger.path, task.line, f"task {task.task_id} has no concrete title")
            if task.marker == "H" and task.kind != "H":
                self.error(
                    "E055",
                    ledger.path,
                    task.line,
                    "[H] AWAITING_HUMAN is valid only for H-namespace tasks",
                )
            if task.marker == "-" and not task.values("Superseded-By"):
                self.error(
                    "E056",
                    ledger.path,
                    task.line,
                    f"superseded task {task.task_id} requires Superseded-By",
                )
            if task.marker == "?" and not task.values("Blocked-By"):
                self.error(
                    "E057",
                    ledger.path,
                    task.line,
                    f"blocked task {task.task_id} requires Blocked-By",
                )

        if state and state.value == "NOT_STARTED" and ledger.tasks:
            self.error("E058", ledger.path, ledger.tasks[0].line, "NOT_STARTED ledger must not contain live tasks")
        if state and state.value == "BLOCKED" and not any(task.marker == "?" for task in ledger.tasks):
            self.error("E059", ledger.path, state.line, "BLOCKED milestone has no blocked task")

        return task_by_id

    def _refs(self, values: Iterable[FieldValue]) -> list[tuple[str, int]]:
        found: list[tuple[str, int]] = []
        for item in values:
            for match in TASK_REF_RE.finditer(item.value):
                found.append((match.group(1), item.line))
        return found

    def _archived_task_found(
        self, source: FieldValue | None, task_id: str, *, marker: str | None = None
    ) -> bool:
        """Check an exact task declaration in an existing local archive."""
        if source is None or not source.value.startswith("records/"):
            return False
        path = (self.root / source.value).resolve()
        try:
            path.relative_to((self.root / "records").resolve())
        except ValueError:
            return False
        if not path.is_file():
            return False
        key = str(path)
        if key not in self._archive_cache:
            try:
                self._archive_cache[key] = path.read_text(encoding="utf-8")
            except (OSError, UnicodeError):
                return False
        mark = re.escape(marker) if marker is not None else r"[ ~?Hx-]"
        pattern = rf"(?m)^\s*-\s*\[{mark}\]\s+{re.escape(task_id)}(?=\s|$)"
        return re.search(pattern, self._archive_cache[key]) is not None

    def _validate_references(self, ledger: ParsedLedger, task_by_id: dict[str, Task]) -> None:
        for task in ledger.tasks:
            for field_name in REFERENCE_FIELDS:
                for ref, line in self._refs(task.values(field_name)):
                    if ref not in task_by_id:
                        if field_name == "Covers" and task.kind == "H" and self._archived_task_found(
                            task.first("Coverage-Source"), ref
                        ):
                            continue
                        self.error(
                            "E060",
                            ledger.path,
                            line,
                            f"{task.task_id} {field_name} references unknown task {ref}",
                        )
                    if ref == task.task_id:
                        self.error(
                            "E061",
                            ledger.path,
                            line,
                            f"{task.task_id} may not reference itself through {field_name}",
                        )

    def _validate_evidence_coverage(
        self, ledger: ParsedLedger, task_by_id: dict[str, Task]
    ) -> None:
        for task in ledger.tasks:
            if task.kind in {"R", "P"}:
                verified_by = self._refs(task.values("Verified-By"))
                if task.marker == "x" and not verified_by:
                    self.error(
                        "E070",
                        ledger.path,
                        task.line,
                        f"verified requirement {task.task_id} requires Verified-By evidence",
                    )
                for target_id, line in verified_by:
                    target = task_by_id.get(target_id)
                    if target is None:
                        continue
                    if target.kind not in {"V", "H"}:
                        self.error(
                            "E071",
                            ledger.path,
                            line,
                            f"Verified-By target {target_id} is not a V/H evidence task",
                        )
                        continue
                    covers = {ref for ref, _ in self._refs(target.values("Covers"))}
                    if task.task_id not in covers:
                        self.error(
                            "E072",
                            ledger.path,
                            line,
                            f"{target_id} does not list {task.task_id} in Covers",
                        )
                    if task.marker == "x" and target.marker != "x":
                        self.error(
                            "E073",
                            ledger.path,
                            line,
                            f"verified requirement {task.task_id} depends on unresolved evidence {target_id}",
                        )
            elif task.values("Verified-By"):
                self.warning(
                    "W070",
                    ledger.path,
                    task.values("Verified-By")[0].line,
                    f"Verified-By is intended for R/P requirements, not {task.kind} tasks",
                )

            if task.kind == "H" and task.values("Coverage-Source"):
                archive = task.first("Coverage-Source")
                if not any(
                    ref not in task_by_id and self._archived_task_found(archive, ref)
                    for ref, _ in self._refs(task.values("Covers"))
                ):
                    self.error("E077", ledger.path, task.line,
                               f"{task.task_id} Coverage-Source requires an exact archived requirement")

            if task.kind in {"V", "H"}:
                covers = self._refs(task.values("Covers"))
                if not covers:
                    self.error(
                        "E074",
                        ledger.path,
                        task.line,
                        f"evidence task {task.task_id} requires Covers",
                    )
                for target_id, line in covers:
                    target = task_by_id.get(target_id)
                    if target is None:
                        continue
                    if target.kind not in {"R", "P"}:
                        self.error(
                            "E075",
                            ledger.path,
                            line,
                            f"Covers target {target_id} is not an R/P requirement",
                        )
                        continue
                    reverse = {ref for ref, _ in self._refs(target.values("Verified-By"))}
                    if task.task_id not in reverse:
                        self.error(
                            "E076",
                            ledger.path,
                            line,
                            f"{target_id} does not list {task.task_id} in Verified-By",
                        )
            elif task.values("Covers"):
                self.warning(
                    "W071",
                    ledger.path,
                    task.values("Covers")[0].line,
                    f"Covers is intended for V/H evidence tasks, not {task.kind} tasks",
                )

    def _require_task_field(
        self,
        ledger: ParsedLedger,
        task: Task,
        name: str,
        code: str,
    ) -> list[FieldValue]:
        values = task.values(name)
        if not values:
            self.error(code, ledger.path, task.line, f"{task.task_id} requires {name}")
            return []
        for item in values:
            if self._placeholder(item.value) or item.value.upper() == "UNKNOWN":
                self.error(code, ledger.path, item.line, f"{task.task_id} has unresolved {name}")
        return values

    def _validate_verification_tasks(
        self, ledger: ParsedLedger, task_by_id: dict[str, Task]
    ) -> None:
        del task_by_id
        for task in ledger.tasks:
            if task.kind not in {"V", "H"}:
                continue

            gate_values = self._require_task_field(ledger, task, "Gate", "E080")
            gate = gate_values[0].value if gate_values else None
            if gate and gate not in ALLOWED_GATES:
                self.error("E081", ledger.path, gate_values[0].line, f"invalid Gate {gate!r}")
            if task.kind == "H" and gate and gate != "HUMAN":
                self.error("E082", ledger.path, gate_values[0].line, "H task Gate must be HUMAN")
            if task.kind == "V" and gate == "HUMAN":
                self.error("E083", ledger.path, gate_values[0].line, "V task Gate may not be HUMAN")

            if task.kind == "V":
                oracle_values = task.values("Oracle")
                if oracle_values:
                    oracle = oracle_values[0].value
                    if oracle not in ALLOWED_ORACLES:
                        self.error(
                            "E084", ledger.path, oracle_values[0].line, f"invalid Oracle {oracle!r}"
                        )
                if task.marker == "x":
                    mode = task.first("Evidence-Mode")
                    if mode is not None and mode.value == "HISTORICAL_RECORDED":
                        source_values = self._require_task_field(
                            ledger, task, "Evidence-Source", "E097"
                        )
                        if source_values and not self._archived_task_found(
                            source_values[0], task.task_id, marker="x"
                        ):
                            self.error(
                                "E098", ledger.path, source_values[0].line,
                                f"{task.task_id} is not recorded as verified in its archival source",
                            )
                        result_values = self._require_task_field(
                            ledger, task, "Result", "E088"
                        )
                        self._require_task_field(ledger, task, "Limitations", "E096")
                        if result_values and not result_values[0].value.upper().startswith("PASS"):
                            self.error(
                                "E090", ledger.path, result_values[0].line,
                                f"{task.task_id} historical result must begin PASS",
                            )
                    else:
                        if mode is not None:
                            self.error(
                                "E099", ledger.path, mode.line,
                                f"unsupported Evidence-Mode {mode.value!r}",
                            )
                        self._require_task_field(ledger, task, "Command", "E085")
                        self._require_task_field(ledger, task, "Oracle", "E086")
                        self._require_task_field(ledger, task, "Expected", "E087")
                        result_values = self._require_task_field(ledger, task, "Result", "E088")
                        repository_values = self._require_task_field(
                            ledger, task, "Repository-State", "E089"
                        )
                        self._require_task_field(ledger, task, "Limitations", "E096")
                        if result_values and not result_values[0].value.upper().startswith("PASS"):
                            self.error(
                                "E090", ledger.path, result_values[0].line,
                                f"verified V task {task.task_id} must record a PASS result",
                            )
                        if repository_values:
                            self._validate_repository_state_value(
                                ledger.path, repository_values[0], task.task_id
                            )
                elif any(
                    item.value.upper().startswith("PASS") for item in task.values("Result")
                ):
                    self.warning(
                        "W080",
                        ledger.path,
                        task.values("Result")[0].line,
                        f"{task.task_id} records PASS but is not [x] VERIFIED",
                    )

            if task.kind == "H":
                if task.marker == "x":
                    decision_values = self._require_task_field(
                        ledger, task, "Human-Decision", "E091"
                    )
                    self._require_task_field(ledger, task, "Decision-Source", "E092")
                    if decision_values and decision_values[0].value != "ACCEPTED":
                        self.error(
                            "E093",
                            ledger.path,
                            decision_values[0].line,
                            "verified H task must record Human-Decision: ACCEPTED",
                        )
                elif any(item.value == "ACCEPTED" for item in task.values("Human-Decision")):
                    self.error(
                        "E094",
                        ledger.path,
                        task.values("Human-Decision")[0].line,
                        f"{task.task_id} records acceptance but is not [x] VERIFIED",
                    )

    def _validate_repository_state_value(
        self, path: Path, value: FieldValue, task_id: str
    ) -> None:
        text = value.value.strip()
        clean_match = re.fullmatch(
            r"HEAD=([0-9a-fA-F]{7,40});\s*WORKTREE=CLEAN", text
        )
        dirty_match = re.fullmatch(
            r"HEAD=([0-9a-fA-F]{7,40});\s*DIFF-SHA256=([0-9a-fA-F]{64})", text
        )
        if not clean_match and not dirty_match:
            self.error(
                "E095",
                path,
                value.line,
                f"{task_id} Repository-State must be HEAD=<sha>; WORKTREE=CLEAN or HEAD=<sha>; DIFF-SHA256=<sha256>",
            )
            return

        match = clean_match or dirty_match
        assert match is not None
        recorded_head = self.resolve_git_ref(match.group(1))
        if not recorded_head:
            self.warning(
                "W081",
                path,
                value.line,
                f"{task_id} evidence HEAD {match.group(1).lower()} is unavailable in this worktree",
            )
            return

        if clean_match:
            if self.material_changed_since(recorded_head):
                self.warning(
                    "W082",
                    path,
                    value.line,
                    f"{task_id} evidence is stale: material files changed after recorded clean state {recorded_head}",
                )
            return

        assert dirty_match is not None
        current_diff = self.worktree_fingerprint(recorded_head)
        recorded_diff = dirty_match.group(2).lower()
        if current_diff and current_diff != recorded_diff:
            self.warning(
                "W083",
                path,
                value.line,
                f"{task_id} material-worktree fingerprint is stale",
            )

    def _validate_conclusions(self, ledger: ParsedLedger, task_by_id: dict[str, Task]) -> None:
        for task in ledger.tasks:
            has_conclusion = any(task.values(name) for name in CONCLUSION_FIELDS)
            if not has_conclusion:
                continue
            if task.kind != "D":
                self.error(
                    "E100",
                    ledger.path,
                    task.line,
                    f"decision-shaping conclusions belong on D tasks, not {task.kind} tasks",
                )
            for name in CONCLUSION_FIELDS:
                self._require_task_field(ledger, task, name, "E101")

            status_item = task.first("Conclusion-Status")
            if status_item and status_item.value not in ALLOWED_CONCLUSION_STATUSES:
                self.error(
                    "E102",
                    ledger.path,
                    status_item.line,
                    f"invalid Conclusion-Status {status_item.value!r}",
                )
                continue

            statement = task.first("Conclusion")
            if (
                statement
                and status_item
                and status_item.value not in {"VERIFIED", "HUMAN_ACCEPTED"}
                and STRONG_NEGATIVE_RE.search(statement.value)
            ):
                self.warning(
                    "W100",
                    ledger.path,
                    statement.line,
                    "strong negative conclusion is not independently verified; bound it to the observed scope",
                )

            evidence_refs = self._refs(task.values("Conclusion-Evidence"))
            if not status_item:
                continue
            if status_item.value == "VERIFIED":
                resolved = [task_by_id[ref] for ref, _ in evidence_refs if ref in task_by_id]
                if not any(item.kind in {"V", "H"} and item.marker == "x" for item in resolved):
                    self.error(
                        "E103",
                        ledger.path,
                        status_item.line,
                        "VERIFIED conclusion requires at least one verified V/H task in Conclusion-Evidence",
                    )
            elif status_item.value == "HUMAN_ACCEPTED":
                resolved = [task_by_id[ref] for ref, _ in evidence_refs if ref in task_by_id]
                if not any(item.kind == "H" and item.marker == "x" for item in resolved):
                    self.error(
                        "E104",
                        ledger.path,
                        status_item.line,
                        "HUMAN_ACCEPTED conclusion requires a verified H task in Conclusion-Evidence",
                    )

    def _validate_supersession(self, ledger: ParsedLedger, task_by_id: dict[str, Task]) -> None:
        for task in ledger.tasks:
            supersedes = {ref for ref, _ in self._refs(task.values("Supersedes"))}
            superseded_by = {ref for ref, _ in self._refs(task.values("Superseded-By"))}
            for old_id in supersedes:
                old = task_by_id.get(old_id)
                if old is None:
                    continue
                reverse = {ref for ref, _ in self._refs(old.values("Superseded-By"))}
                if task.task_id not in reverse:
                    self.error(
                        "E110",
                        ledger.path,
                        task.line,
                        f"{task.task_id} supersedes {old_id}, but the reverse Superseded-By link is missing",
                    )
            for new_id in superseded_by:
                new = task_by_id.get(new_id)
                if new is None:
                    continue
                reverse = {ref for ref, _ in self._refs(new.values("Supersedes"))}
                if task.task_id not in reverse:
                    self.error(
                        "E111",
                        ledger.path,
                        task.line,
                        f"{task.task_id} is superseded by {new_id}, but the reverse Supersedes link is missing",
                    )
            if task.marker == "-" and not superseded_by:
                self.error(
                    "E112",
                    ledger.path,
                    task.line,
                    f"superseded task {task.task_id} lacks a task-ID Superseded-By link",
                )

    def _validate_file_bindings(self, ledger: ParsedLedger) -> None:
        for task in ledger.tasks:
            for item in task.values("Doc"):
                value = item.value.split("#", 1)[0].strip()
                if self._placeholder(value):
                    continue
                if value.startswith("docs/") and not (self.root / value).exists():
                    self.error(
                        "E120",
                        ledger.path,
                        item.line,
                        f"{task.task_id} references missing canonical document {value}",
                    )
            if task.marker != "x":
                continue
            for item in task.values("Implementation"):
                value = item.value.strip()
                if self._placeholder(value):
                    continue
                path_part = re.split(r"::|#", value, maxsplit=1)[0]
                if "/" in path_part and not (self.root / path_part).exists():
                    self.error(
                        "E121",
                        ledger.path,
                        item.line,
                        f"{task.task_id} references missing implementation path {path_part}",
                    )

    def _validate_completion(self, ledger: ParsedLedger) -> None:
        state = ledger.header.get("State")
        phase = ledger.header.get("Phase")
        if state and state.value == "COMPLETE":
            unresolved = [task for task in ledger.tasks if task.marker not in {"x", "-"}]
            for task in unresolved:
                self.error(
                    "E130",
                    ledger.path,
                    task.line,
                    f"COMPLETE milestone contains unresolved task {task.task_id}",
                )
        if phase and phase.value == "HUMAN_VERIFICATION":
            if not any(task.kind == "H" and task.marker == "H" for task in ledger.tasks):
                self.warning(
                    "W130",
                    ledger.path,
                    phase.line,
                    "HUMAN_VERIFICATION phase has no [H] human-verification task",
                )

    MATERIAL_EXCLUDES = (
        "STATUS.md",
        "TASKS.md",
        "records/**",
        "experiences/retrievals.jsonl",
        ".durable-state/**",
    )

    def _run_git(self, args: Sequence[str]) -> bytes | None:
        try:
            return subprocess.check_output(
                ["git", "-C", str(self.root), *args], stderr=subprocess.DEVNULL
            )
        except (OSError, subprocess.CalledProcessError):
            return None

    def git_head(self) -> str | None:
        if self._git_head is None:
            output = self._run_git(["rev-parse", "HEAD"])
            self._git_head = output.decode("ascii").strip().lower() if output else ""
        return self._git_head or None

    def resolve_git_ref(self, ref: str) -> str | None:
        output = self._run_git(["rev-parse", "--verify", f"{ref}^{{commit}}"])
        return output.decode("ascii").strip().lower() if output else None

    def _material_pathspec(self) -> list[str]:
        return ["--", ".", *[f":(exclude){item}" for item in self.MATERIAL_EXCLUDES]]

    def _material_diff(self, base_ref: str) -> bytes | None:
        return self._run_git(
            ["diff", "--binary", "--no-ext-diff", base_ref, *self._material_pathspec()]
        )

    def _material_untracked_paths(self) -> list[bytes] | None:
        output = self._run_git(["ls-files", "--others", "--exclude-standard", "-z"])
        if output is None:
            return None
        excluded_exact = {"STATUS.md", "TASKS.md", "experiences/retrievals.jsonl"}
        paths: list[bytes] = []
        for encoded in output.split(b"\0"):
            if not encoded:
                continue
            decoded = encoded.decode("utf-8", errors="surrogateescape")
            if (
                decoded in excluded_exact
                or decoded.startswith("records/")
                or decoded.startswith(".durable-state/")
            ):
                continue
            paths.append(encoded)
        return sorted(paths)

    def material_changed_since(self, base_ref: str) -> bool | None:
        diff = self._material_diff(base_ref)
        untracked = self._material_untracked_paths()
        if diff is None or untracked is None:
            return None
        return bool(diff or untracked)

    def material_dirty(self) -> bool | None:
        if self._material_dirty is None:
            head = self.git_head()
            if not head:
                return None
            changed = self.material_changed_since(head)
            if changed is None:
                return None
            self._material_dirty = changed
        return self._material_dirty

    def worktree_fingerprint(self, base_ref: str) -> str | None:
        resolved = self.resolve_git_ref(base_ref)
        if not resolved:
            return None
        if resolved in self._worktree_fingerprints:
            return self._worktree_fingerprints[resolved]
        digest = hashlib.sha256()
        digest.update(b"durable-state-material-worktree-v1\0")
        digest.update(resolved.encode("ascii"))
        digest.update(b"\0")
        diff = self._material_diff(resolved)
        if diff is None:
            return None
        digest.update(diff)
        digest.update(b"\0UNTRACKED\0")
        paths = self._material_untracked_paths()
        if paths is None:
            return None
        for encoded_path in paths:
            try:
                relative = encoded_path.decode("utf-8", errors="surrogateescape")
                full_path = self.root / relative
                info = full_path.lstat()
            except OSError:
                return None
            digest.update(encoded_path)
            digest.update(b"\0")
            if stat.S_ISLNK(info.st_mode):
                digest.update(b"SYMLINK\0")
                digest.update(os.readlink(full_path).encode("utf-8", errors="surrogateescape"))
            elif stat.S_ISREG(info.st_mode):
                digest.update(b"FILE\0")
                try:
                    with full_path.open("rb") as handle:
                        while chunk := handle.read(1024 * 1024):
                            digest.update(chunk)
                except OSError:
                    return None
            else:
                digest.update(f"MODE={info.st_mode:o}".encode("ascii"))
            digest.update(b"\0")
        value = digest.hexdigest()
        self._worktree_fingerprints[resolved] = value
        return value

    def current_repository_state(self) -> str:
        head = self.git_head()
        if not head:
            raise RuntimeError(f"{self.root} is not a readable Git worktree")
        dirty = self.material_dirty()
        if dirty is False:
            return f"HEAD={head}; WORKTREE=CLEAN"
        fingerprint = self.worktree_fingerprint(head)
        if not fingerprint:
            raise RuntimeError("could not calculate material-worktree fingerprint")
        return f"HEAD={head}; DIFF-SHA256={fingerprint}"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate durable-state schema 2 without executing project commands."
    )
    parser.add_argument("directory", nargs="?", default=".", help="project directory")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="treat warnings, including stale repository evidence, as failure",
    )
    parser.add_argument("--json", action="store_true", help="emit machine-readable diagnostics")
    parser.add_argument(
        "--target-schema",
        choices=[SUPPORTED_SCHEMA_VERSION],
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--print-repository-state",
        action="store_true",
        help="print the current repository-state value and exit",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    root = Path(args.directory)
    if not root.is_dir():
        print(f"durable-state validator: directory does not exist: {root}", file=sys.stderr)
        return 2

    validator = Validator(root, strict=args.strict, target_schema=args.target_schema)
    if args.print_repository_state:
        try:
            print(validator.current_repository_state())
        except RuntimeError as exc:
            print(f"durable-state validator: {exc}", file=sys.stderr)
            return 2
        return 0

    diagnostics = validator.validate()
    if args.json:
        payload = {
            "schema_version": SUPPORTED_SCHEMA_VERSION,
            "project": str(validator.root),
            "valid": not validator.failed(),
            "strict": args.strict,
            "errors": sum(item.severity == "ERROR" for item in diagnostics),
            "warnings": sum(item.severity == "WARNING" for item in diagnostics),
            "diagnostics": [item.to_dict() for item in diagnostics],
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        for item in diagnostics:
            location = f"{item.path}:{item.line}" if item.line else item.path
            print(f"{item.severity} {item.code} {location}: {item.message}")
        errors = sum(item.severity == "ERROR" for item in diagnostics)
        warnings = sum(item.severity == "WARNING" for item in diagnostics)
        status = "VALID" if not validator.failed() else "INVALID"
        print(f"{status}: {errors} error(s), {warnings} warning(s)")

    return 1 if validator.failed() else 0


if __name__ == "__main__":
    raise SystemExit(main())
