#!/usr/bin/env python3
"""Reduce environment-owned sim-time logs and compare repeated runs."""

import argparse
import csv
import difflib
import glob
import io
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


OBSERVATION_RE = re.compile(r"SIM_OBS\s+(?P<node>[a-z_]+)\s+(?P<fields>.*)")
FIELD_RE = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>\S+)")
CAN_DUMP_RE = re.compile(
    r"^\s*\([^)]*\)\s+\S+\s+(?P<identifier>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)")
CLOCK_RECORD_RE = re.compile(
    r"clock:\s*\n\s*sec:\s*(-?\d+)\s*\n\s*nanosec:\s*(\d+)",
    re.MULTILINE,
)
RECORD_SEPARATOR_RE = re.compile(r"(?m)^\s*---\s*$")
SIM_STEP_MARKER_ID = 0x7FF
CONTROLLER_COMMAND_IDS = range(0x578, 0x57E)
STATIC_STEP_RE = re.compile(
    r"Simulation stepping owned by the environment: step=(?P<step>\d+) ms")
ADAPTATION_COMPLETE_RE = re.compile(
    r"SIM_STEP adaptation complete: step=(?P<step>\d+) ms timeout=(?P<timeout>\d+) ms "
    r"switch_ms=(?P<switch>\d+)")
ADAPTATION_ID_RE = re.compile(
    r"SIM_STEP adaptation id=(?P<id>0x[0-9A-Fa-f]+) arrivals=\d+ class=(?P<cls>\S+) "
    r"period_ms=(?P<period>\d+) anchor_ms=(?P<anchor>\d+)")
REPLAY_RE = re.compile(r"Simulation stepping owned by the environment: open-loop replay of ")
REPLAY_SCHEDULE_RE = re.compile(
    r"SIM_STEP replay schedule: step_ms=(?P<step>\d+) switch_ms=(?P<switch>\d+)")
ROS_CAPTURE_PREFIX = "ros:"

Observation = Tuple[str, Dict[str, str]]
COMPANION_SUFFIXES = {
    "runtime": "-runtime.txt",
    "clock": "-clock.txt",
    "can": "-can.txt",
    "steps": "-steps.csv",
}


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


class RunData:
    def __init__(self, text: str, companion_text: Optional[Dict[str, str]] = None) -> None:
        self.text = text
        self.companion_text = companion_text or {}
        self.runtime_text = self.companion_text.get("runtime", "")
        self.observations: List[Observation] = []
        self.can_capture_records: List[str] = []
        self.can_capture_by_step: Dict[int, List[str]] = {}
        self.can_marker_steps: List[int] = []
        self.can_marker_invalid = 0
        self.clock_ms: List[int] = []
        self.step_rows: List[Dict[str, str]] = []
        static_match = STATIC_STEP_RE.search(text)
        adaptation_match = ADAPTATION_COMPLETE_RE.search(text)
        self.adaptation_switch_ms = 0
        self.environment_step_ms = None
        if static_match:
            self.environment_step_ms = int(static_match.group("step"))
        if adaptation_match:
            self.environment_step_ms = int(adaptation_match.group("step"))
            self.adaptation_switch_ms = int(adaptation_match.group("switch"))
        self.adaptation_expected = "adaptation on" in text or adaptation_match is not None
        self.replay = REPLAY_RE.search(text) is not None
        replay_schedule = REPLAY_SCHEDULE_RE.search(text)
        if replay_schedule:
            self.environment_step_ms = int(replay_schedule.group("step"))
            self.adaptation_switch_ms = int(replay_schedule.group("switch"))
        self.ros_records: Dict[str, List[str]] = {
            name[len(ROS_CAPTURE_PREFIX):]: [
                record.strip() for record in RECORD_SEPARATOR_RE.split(captured)
                if record.strip()]
            for name, captured in self.companion_text.items()
            if name.startswith(ROS_CAPTURE_PREFIX)
        }
        self.adaptation_ids = sorted(
            (m.group("id").upper(), m.group("cls"), int(m.group("period")), int(m.group("anchor")))
            for m in ADAPTATION_ID_RE.finditer(text))
        steps_text = self.companion_text.get("steps", "")
        if steps_text:
            self.step_rows = list(csv.DictReader(io.StringIO(steps_text)))

        for line in text.splitlines():
            observation = OBSERVATION_RE.search(line)
            if observation:
                fields = dict(FIELD_RE.findall(observation.group("fields")))
                self.observations.append((observation.group("node"), fields))

        clock_text = self.companion_text.get("clock", "")
        self.clock_ms = [
            int(seconds) * 1000 + int(nanoseconds) // 1000000
            for seconds, nanoseconds in CLOCK_RECORD_RE.findall(clock_text)
        ]

        pending_capture_frames: List[str] = []
        for line in self.companion_text.get("can", "").splitlines():
            match = CAN_DUMP_RE.match(line)
            if not match:
                continue
            identifier = int(match.group("identifier"), 16)
            data = match.group("data").upper()
            frame = "%X#%s" % (identifier, data)
            self.can_capture_records.append(frame)
            if identifier != SIM_STEP_MARKER_ID:
                pending_capture_frames.append(frame)
                continue

            try:
                marker_data = bytes.fromhex(data)
            except ValueError:
                marker_data = b""
            step = int.from_bytes(marker_data, byteorder="little") if len(marker_data) == 8 else 0
            if step == 0:
                self.can_marker_invalid += 1
                pending_capture_frames = []
                continue
            self.can_marker_steps.append(step)
            step_frames = self.can_capture_by_step.setdefault(step, [])
            step_frames.extend(pending_capture_frames)
            step_frames.append(frame)
            pending_capture_frames = []

    @property
    def all_text(self) -> str:
        return self.text + "\n" + self.runtime_text


def load_run(path_text: str) -> RunData:
    if path_text == "-":
        return RunData(sys.stdin.read())

    path = Path(path_text)
    if path.is_dir():
        log_paths = sorted(path.rglob("*.log"))
        if len(log_paths) == 1:
            return load_run(str(log_paths[0]))
        contents = [read_text(log_path) for log_path in log_paths]
        return RunData("\n".join(contents))

    companion_text = {}
    for name, suffix in COMPANION_SUFFIXES.items():
        companion_path = path.with_name(path.stem + suffix)
        if companion_path.exists():
            companion_text[name] = read_text(companion_path)
    for ros_path in sorted(path.parent.glob(glob.escape(path.stem) + "-ros-*.txt")):
        topic = ros_path.name[len(path.stem) + len("-ros-"):-len(".txt")]
        companion_text[ROS_CAPTURE_PREFIX + topic] = read_text(ros_path)
    return RunData(read_text(path), companion_text)


def latest_summary(data: RunData, node: str) -> Dict[str, str]:
    summaries = [
        fields
        for observed_node, fields in data.observations
        if observed_node == node and fields.get("summary") == "1"
    ]
    return summaries[-1] if summaries else {}


def as_int(fields: Dict[str, str], key: str):
    try:
        return int(fields[key])
    except (KeyError, TypeError, ValueError):
        return None


def format_table(rows: Sequence[Tuple[str, str, str]]) -> None:
    print("CHECK | STATUS | DETAILS")
    print("----- | ------ | -------")
    for name, status, details in rows:
        print("%s | %s | %s" % (name, status, details))


def _step_row_int(row: Dict[str, str], key: str) -> Optional[int]:
    try:
        return int(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def expected_increment(t_ms: int, step_ms: int, switch_ms: int) -> int:
    """Clock increment after t_ms: 1 ms during adaptation, the derived step after the switch."""
    return 1 if t_ms < switch_ms else step_ms


def step_index(t_ms: int, step_ms: int, switch_ms: int) -> int:
    if t_ms <= switch_ms:
        return t_ms
    return switch_ms + (t_ms - switch_ms) // step_ms


def check_step_records(rows: Sequence[Dict[str, str]], step_ms: int,
                       switch_ms: int = 0) -> List[Tuple[str, str, str]]:
    """F.7 logical-time and command-attribution checks on the bridge step records."""
    if not rows:
        return [("step records", "FAIL", "sim_steps.csv is missing (logging.sim_steps)")]

    out_of_order = 0
    bad_time = 0
    bad_timeout_flag = 0
    missing_arrival = 0
    arrival_after_close = 0
    malformed = 0
    previous_step = 0
    previous_t = 0
    for row in rows:
        step = _step_row_int(row, "step")
        t_ms = _step_row_int(row, "t_ms")
        wait_us = _step_row_int(row, "wait_us")
        timed_out = _step_row_int(row, "timed_out")
        if None in (step, t_ms, wait_us, timed_out):
            malformed += 1
            continue
        if step != previous_step + 1:
            out_of_order += 1
        if t_ms - previous_t != expected_increment(previous_t, step_ms, switch_ms):
            bad_time += 1
        previous_step, previous_t = step, t_ms
        due = [item for item in row.get("due", "").split("|") if item]
        missing = [item for item in row.get("missing", "").split("|") if item]
        arrivals = {}
        for item in row.get("arrivals_us", "").split("|"):
            if ":" in item:
                identifier, latency = item.split(":", 1)
                try:
                    arrivals[identifier] = int(latency)
                except ValueError:
                    malformed += 1
        if bool(timed_out) != bool(missing):
            bad_timeout_flag += 1
        if not timed_out and any(identifier not in arrivals for identifier in due):
            missing_arrival += 1
        if any(latency > wait_us for latency in arrivals.values()):
            arrival_after_close += 1

    complete = not (out_of_order or bad_time or malformed)
    rows_out = [
        ("step records complete and consecutive", "PASS" if complete else "FAIL",
         "%d records; %d out of order, %d time increments off the expected step (%d ms after %d ms), %d malformed" % (
             len(rows), out_of_order, bad_time, step_ms, switch_ms, malformed)),
        ("step closure consistent with due IDs",
         "PASS" if not (bad_timeout_flag or missing_arrival) else "FAIL",
         "%d timeout flag mismatches, %d complete steps with a due ID lacking an arrival" % (
             bad_timeout_flag, missing_arrival)),
        ("no frame merged into a closed step", "PASS" if not arrival_after_close else "FAIL",
         "%d steps with an arrival later than the close" % arrival_after_close),
    ]
    timeouts = sum(1 for row in rows if row.get("timed_out") == "1")
    late = sum(_step_row_int(row, "late") or 0 for row in rows)
    early = sum(_step_row_int(row, "early") or 0 for row in rows)
    duplicates = sum(_step_row_int(row, "duplicates") or 0 for row in rows)
    waits = sorted(_step_row_int(row, "wait_us") or 0 for row in rows if row.get("timed_out") != "1")
    if waits:
        percentile = lambda fraction: waits[min(len(waits) - 1, int(fraction * len(waits)))]
        latency = "wait_us p50=%d p95=%d p99=%d max=%d" % (
            percentile(0.5), percentile(0.95), percentile(0.99), waits[-1])
    else:
        latency = "no completed steps"
    rows_out.append(("step timeouts, duplicates, late and early frames", "INFO",
                     "%d timeouts, %d duplicates, %d late, %d early; %s" % (
                         timeouts, duplicates, late, early, latency)))
    return rows_out


def reduce_run(data: RunData, expected_step_ms: int,
               minimum_sim_ms: int) -> List[Tuple[str, str, str]]:
    step_ms = data.environment_step_ms or expected_step_ms
    rows: List[Tuple[str, str, str]] = []
    mode_count = len(re.findall(r"Simulation clock mode enabled", data.text))
    required_mode_count = 1 if data.replay else 2
    rows.append(("sim mode enabled on bridge and controller",
                 "PASS" if mode_count >= required_mode_count else "FAIL",
                 "%d enabled startup messages" % mode_count))
    started = "SIM_STEP environment-owned stepping started." in data.text
    switch_ms = data.adaptation_switch_ms
    rows.append(("environment-owned stepping", "PASS" if started else "FAIL",
                 "step=%s ms; stepping %s" % (step_ms, "started" if started else "did not start")))
    if data.adaptation_expected:
        scheduled = [item for item in data.adaptation_ids if item[1] == "scheduled"]
        rows.append(("adaptation period completed",
                     "PASS" if data.adaptation_switch_ms and scheduled else "FAIL",
                     "switch at %d ms, step %s ms; scheduled %s; unscheduled %s" % (
                         switch_ms, step_ms,
                         ", ".join("%s@%d" % (i[0], i[2]) for i in scheduled) or "none",
                         ", ".join(i[0] for i in data.adaptation_ids if i[1] != "scheduled") or "none")))
    if data.replay:
        rows.append(("controller command path", "PASS",
                     "open-loop replay; command intake from CAN is disabled"))
    elif "Raptor DBW node is used." in data.text:
        rows.append(("Raptor DBW path", "PASS", "controller selected Raptor DBW (variant V3)"))
    elif "Direct CAN communication is enabled" in data.text:
        rows.append(("direct CAN path", "PASS", "controller selected direct CAN (variant V1)"))
    else:
        rows.append(("controller command path", "FAIL", "controller path was not identified"))

    if data.runtime_text:
        true_count = len(re.findall(r"Boolean value is: True", data.runtime_text))
        rows.append(("use_sim_time parameters", "PASS" if true_count >= required_mode_count else "FAIL",
                     "%d true Boolean values in runtime artifact" % true_count))
    else:
        rows.append(("use_sim_time parameters", "FAIL", "runtime artifact is missing"))

    bridge = latest_summary(data, "bridge")
    steps_run = None
    if bridge:
        sim_ms = as_int(bridge, "sim_time_ms")
        cumulative = as_int(bridge, "cumulative_substeps")
        mismatches = as_int(bridge, "substep_mismatches")
        published = as_int(bridge, "clock_published")
        markers = as_int(bridge, "step_markers_sent")
        time_mismatches = as_int(bridge, "env_time_mismatches")
        closed = as_int(bridge, "env_steps_closed")
        if None not in (sim_ms, cumulative, mismatches):
            steps_run = step_index(sim_ms, step_ms, switch_ms)
            status = "PASS" if ((sim_ms <= switch_ms or (sim_ms - switch_ms) % step_ms == 0) and
                                cumulative == sim_ms and mismatches == 0) else "FAIL"
            rows.append(("exact sub-step count per step", status,
                         "%d sub-steps over %d steps (1 ms until %d ms, then %d ms); %d mismatches" % (
                             cumulative, steps_run, switch_ms, step_ms, mismatches)))
            if published is not None:
                status = "PASS" if published in (steps_run, steps_run + 1) else "FAIL"
                rows.append(("one release per step", status,
                             "%d clock publications / %d steps plus initial clock" % (
                                 published, steps_run)))
            if markers is not None and data.can_marker_steps:
                rows.append(("step markers sent per step",
                             "PASS" if markers == steps_run else "FAIL",
                             "%d markers / %d steps" % (markers, steps_run)))
            if closed is not None:
                rows.append(("closed steps",
                             "PASS" if closed in (steps_run, steps_run - 1) else "FAIL",
                             "%d closed / %d steps (one may be open at shutdown)" % (
                                 closed, steps_run)))
        else:
            rows.append(("exact sub-step count per step", "FAIL", "summary fields incomplete"))
        rows.append(("coordinator time equals published time",
                     "PASS" if time_mismatches == 0 else "FAIL",
                     "env_time_mismatches=%s" % time_mismatches))
        rows.append(("minimum logical runtime",
                     "PASS" if sim_ms is not None and sim_ms >= minimum_sim_ms else "FAIL",
                     "sim_time_ms=%s (minimum %d)" % (sim_ms, minimum_sim_ms)))
        rtf = bridge.get("realtime_factor")
        if rtf is not None:
            rows.append(("real-time factor", "INFO", "realtime_factor=%s env_wall_ms=%s wait_wall_ms=%s" % (
                rtf, bridge.get("env_wall_ms"), bridge.get("wait_wall_ms"))))
        summary_fields = ["env_timeouts", "env_max_consecutive_timeouts", "env_duplicates",
                          "env_late", "env_early", "env_rephased"]
        rows.append(("coordinator totals", "INFO", ", ".join(
            "%s=%s" % (name, bridge.get(name)) for name in summary_fields)))
        if data.replay:
            done, _, total = bridge.get("env_replay_steps", "").partition("/")
            complete = ("SIM_STEP replay complete" in data.text and done.isdigit() and
                        done == total)
            rows.append(("replay ran every recorded step", "PASS" if complete else "FAIL",
                         "env_replay_steps=%s" % bridge.get("env_replay_steps")))
    else:
        rows.append(("exact sub-step count per step", "FAIL", "bridge summary missing"))
        rows.append(("minimum logical runtime", "FAIL", "bridge summary missing"))

    if data.clock_ms:
        intervals = [b - a for a, b in zip(data.clock_ms, data.clock_ms[1:])]
        ok = (data.clock_ms[0] == 0 and bool(intervals) and
              all(i == expected_increment(t, step_ms, switch_ms)
                  for t, i in zip(data.clock_ms, intervals)))
        rows.append(("logical time strictly monotonic in exact steps", "PASS" if ok else "FAIL",
                     "%d clock samples from %d ms, 1 ms increments until %d ms then %d ms" % (
                         len(data.clock_ms), data.clock_ms[0], switch_ms, step_ms)))
    else:
        rows.append(("logical time strictly monotonic in exact steps", "FAIL",
                     "clock capture is missing"))

    rows.extend(check_step_records(data.step_rows, step_ms, switch_ms))
    if data.clock_ms and data.step_rows:
        step_times = [_step_row_int(row, "t_ms") for row in data.step_rows]
        captured = data.clock_ms[1:]
        common = min(len(captured), len(step_times))
        rows.append(("clock capture matches step records",
                     "PASS" if common and captured[:common] == step_times[:common] else "FAIL",
                     "%d common steps" % common))

    if data.can_marker_steps:
        steps = data.can_marker_steps
        contiguous = steps[0] == 1 and all(b == a + 1 for a, b in zip(steps, steps[1:]))
        rows.append(("captured CAN step markers",
                     "PASS" if contiguous and not data.can_marker_invalid else "FAIL",
                     "%d markers from %d to %d, %d malformed" % (
                         len(steps), steps[0], steps[-1], data.can_marker_invalid)))
    else:
        rows.append(("captured CAN step markers", "WARN", "step markers are absent"))
    if data.ros_records:
        rows.append(("captured ROS outputs", "INFO", ", ".join(
            "%s=%d" % item for item in ((topic, len(records))
                                       for topic, records in sorted(data.ros_records.items())))))
    return rows


def _split_marker_group(frames: Sequence[str]) -> Tuple[List[str], List[str]]:
    controller, environment = [], []
    for frame in frames:
        identifier = int(frame.split("#", 1)[0], 16)
        if identifier == SIM_STEP_MARKER_ID:
            continue
        (controller if identifier in CONTROLLER_COMMAND_IDS else environment).append(frame)
    return sorted(controller), environment


def compare_runs(first: RunData, second: RunData, expected_step_ms: int,
                 step_count: Optional[int], max_diff_lines: int) -> bool:
    """Required: identical logical clock. Informational: closed-loop repeat of the team stack."""
    step_ms = first.environment_step_ms or expected_step_ms
    switch_ms = first.adaptation_switch_ms
    target = common_clock_window(first, second, None, step_count)
    failed = compare_clock_window(first, second, target, step_ms, switch_ms)
    if first.adaptation_expected or second.adaptation_expected:
        same = first.adaptation_ids == second.adaptation_ids and (
            first.environment_step_ms == second.environment_step_ms and
            first.adaptation_switch_ms == second.adaptation_switch_ms)
        print("derived schedule (class, period, anchor per ID, step, switch): INFO (%s)" % (
            "identical" if same else "differs"))

    all_steps = [step_index(t, step_ms, switch_ms) for t in target]
    if first.replay and second.replay:
        return compare_replay_outputs(first, second, all_steps, max_diff_lines) or failed

    steps = [s for s in all_steps if first.can_capture_by_step.get(s) and second.can_capture_by_step.get(s)]
    if not steps:
        print("closed-loop CAN comparison: SKIP (no common marker-grouped captures)")
        return failed
    timeouts = [sum(1 for row in run.step_rows if row.get("timed_out") == "1")
                for run in (first, second)]
    note = "; inconclusive, runs had timeouts %s" % timeouts if any(timeouts) else ""
    kind = "recording-vs-replay" if first.replay or second.replay else "closed-loop"
    for label, index in (("controller command frames", 0), ("environment CAN outputs", 1)):
        differing = [s for s in steps
                     if _split_marker_group(first.can_capture_by_step[s])[index] !=
                     _split_marker_group(second.can_capture_by_step[s])[index]]
        if differing:
            print("%s %s by step: INFO (%d of %d steps differ, first at step %d%s)" % (
                kind, label, len(differing), len(steps), differing[0], note))
        else:
            print("%s %s by step: INFO (identical over %d steps%s)" % (
                kind, label, len(steps), note))
    return failed


def compare_replay_outputs(first: RunData, second: RunData, steps: Sequence[int],
                           max_diff_lines: int) -> bool:
    """F.7 environment determinism: two replays of one command sequence must match exactly."""
    failed = False
    missing = [s for s in steps if not first.can_capture_by_step.get(s) or
               not second.can_capture_by_step.get(s)]
    if not steps or missing:
        print("replay CAN outputs by step: FAIL (%d steps, %d without a marker-grouped capture, "
              "first at step %s)" % (len(steps), len(missing), missing[0] if missing else "-"))
        failed = True
    else:
        for label, index in (("environment CAN outputs", 1), ("controller-ID CAN frames", 0)):
            first_frames = [(s, f) for s in steps for f in
                            _split_marker_group(first.can_capture_by_step[s])[index]]
            second_frames = [(s, f) for s in steps for f in
                             _split_marker_group(second.can_capture_by_step[s])[index]]
            failed |= compare_records(
                "replay %s by step" % label,
                ["step=%d %s" % item for item in first_frames],
                ["step=%d %s" % item for item in second_frames], max_diff_lines)
    topics = sorted(set(first.ros_records) | set(second.ros_records))
    if not topics:
        print("replay ROS outputs: WARN (no ROS captures)")
    for topic in topics:
        one, two = first.ros_records.get(topic), second.ros_records.get(topic)
        if not one or not two:
            print("replay ROS output %s: FAIL (missing capture; run-a=%s run-b=%s)" % (
                topic, len(one or []), len(two or [])))
            failed = True
        elif one == two:
            print("replay ROS output %s: PASS (%d messages identical)" % (topic, len(one)))
        else:
            index = next((i for i, pair in enumerate(zip(one, two)) if pair[0] != pair[1]),
                         min(len(one), len(two)))
            print("replay ROS output %s: FAIL (%d vs %d messages, first difference at message %d)" % (
                topic, len(one), len(two), index))
            failed = True
    return failed


def ordered_unique(values: Sequence[int]) -> List[int]:
    seen = set()
    result = []
    for value in values:
        if value not in seen:
            seen.add(value)
            result.append(value)
    return result


def common_clock_window(first: RunData, second: RunData, start_sim_ms: Optional[int], count: Optional[int]) -> List[int]:
    second_times = set(ordered_unique(second.clock_ms))
    common = [value for value in ordered_unique(first.clock_ms)
              if value > 0 and value in second_times]
    if start_sim_ms is not None:
        common = [value for value in common if value >= start_sim_ms]
    if count is not None:
        common = common[:count]
    return common


def clock_window_values(data: RunData, target: Sequence[int]) -> List[int]:
    available = set(data.clock_ms)
    return [value for value in target if value in available]


def compare_records(label: str, first: Sequence[str], second: Sequence[str], limit: int) -> bool:
    if first == second:
        print("%s: PASS (%d records identical)" % (label, len(first)))
        return False

    print("%s: FAIL (%d records vs %d)" % (label, len(first), len(second)))
    diff = list(difflib.unified_diff(first, second, fromfile="run-a", tofile="run-b", lineterm=""))
    for line in diff[:limit]:
        print(line)
    if len(diff) > limit:
        print("... %d additional diff lines omitted" % (len(diff) - limit))
    return True


def compare_clock_window(first: RunData, second: RunData, target: Sequence[int], expected_step_ms: int,
                         switch_ms: int = 0) -> bool:
    first_values = clock_window_values(first, target)
    second_values = clock_window_values(second, target)
    missing = len(target) - min(len(first_values), len(second_values))
    first_intervals = [current - previous for previous, current in zip(first_values, first_values[1:])]
    second_intervals = [current - previous for previous, current in zip(second_values, second_values[1:])]
    complete = bool(target) and not missing and len(first_values) == len(second_values)
    expected_intervals = [expected_increment(t, expected_step_ms, switch_ms) for t in target[:-1]]
    interval_ok = (
        len(first_intervals) == max(0, len(target) - 1) and
        len(second_intervals) == max(0, len(target) - 1) and
        first_intervals == expected_intervals and second_intervals == expected_intervals
    )
    if complete and interval_ok and first_values == second_values:
        print("logical clock records: PASS (%d common steps, %d..%d ms)" %
              (len(target), target[0], target[-1]))
        return False
    print("logical clock records: FAIL (common steps=%d, run-a=%d, run-b=%d)" %
          (len(target), len(first_values), len(second_values)))
    print("clock interval details: run-a=%s run-b=%s expected=%d" %
          (first_intervals[:3], second_intervals[:3], expected_step_ms))
    return True


def main() -> int:
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument(
        "run", help="combined docker log file, directory of .log files, or - for stdin")
    argument_parser.add_argument("--compare", metavar="RUN", help="second run to compare with the first")
    argument_parser.add_argument("--expected-step-ms", type=int, default=10,
                                 help="step duration when the log does not report it (default: 10)")
    argument_parser.add_argument("--minimum-sim-ms", type=int, default=3000,
                                 help="minimum logical runtime for an individual run (default: 3000)")
    argument_parser.add_argument(
        "--steps", type=int,
        help="number of common positive logical-clock steps to compare")
    argument_parser.add_argument("--max-diff-lines", type=int, default=40,
                                 help="maximum comparison diff lines to print (default: 40)")
    arguments = argument_parser.parse_args()

    if arguments.steps is not None and arguments.steps <= 0:
        argument_parser.error("--steps must be positive")
    if arguments.expected_step_ms <= 0:
        argument_parser.error("--expected-step-ms must be positive")

    first_run = load_run(arguments.run)
    first_rows = reduce_run(first_run, arguments.expected_step_ms, arguments.minimum_sim_ms)
    print("SIM-TIME RUN: %s" % arguments.run)
    format_table(first_rows)

    comparison_failed = False
    if arguments.compare:
        second_run = load_run(arguments.compare)
        second_rows = reduce_run(second_run, arguments.expected_step_ms, arguments.minimum_sim_ms)
        print("\nSIM-TIME COMPARISON RUN: %s" % arguments.compare)
        format_table(second_rows)
        print("\nDETERMINISM COMPARISON")
        comparison_failed = compare_runs(
            first_run, second_run, arguments.expected_step_ms,
            arguments.steps, arguments.max_diff_lines)
        comparison_failed |= any(status == "FAIL" for _, status, _ in second_rows)

    return 1 if comparison_failed or any(status == "FAIL" for _, status, _ in first_rows) else 0


if __name__ == "__main__":
    sys.exit(main())