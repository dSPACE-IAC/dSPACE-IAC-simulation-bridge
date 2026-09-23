#!/usr/bin/env python3
"""Reduce sim-time logs and compare repeated runs over a logical-clock window."""

import argparse
import difflib
import math
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


OBSERVATION_RE = re.compile(r"SIM_OBS\s+(?P<node>[a-z_]+)\s+(?P<fields>.*)")
FIELD_RE = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>\S+)")
CAN_FRAME_RE = re.compile(r"\bsend:\s+(0x[0-9A-Fa-f]+)\s+\[(\d+)\]")
CAN_BYTE_RE = re.compile(r"\bsend:\s+([0-9A-Fa-f]{2})\s*$")
CAN_DUMP_RE = re.compile(r"^\s*\([^)]*\)\s+\S+\s+(?P<frame>[0-9A-Fa-f]+#[0-9A-Fa-f]*)")
CLOCK_RECORD_RE = re.compile(
    r"clock:\s*\n\s*sec:\s*(-?\d+)\s*\n\s*nanosec:\s*(\d+)",
    re.MULTILINE,
)
HANDSHAKE_RE = re.compile(r"^data:\s*(\d+)\s*$", re.MULTILINE)
DEBUG_STEERING_RE = re.compile(r"^output_steering:\s*(\S+)\s*$", re.MULTILINE)
DEBUG_TIMING_RE = re.compile(r"^(vel_pid_dt|acc_pid_dt|steering_dt):\s*(\S+)\s*$", re.MULTILINE)
DEBUG_VELOCITY_RE = re.compile(
    r"^(desired_velocity|current_velocity|error_velocity):\s*(\S+)\s*$", re.MULTILINE)
DEBUG_RECORD_SEPARATOR_RE = re.compile(r"(?m)^\s*---\s*$")

Observation = Tuple[str, Dict[str, str]]
COMPANION_SUFFIXES = {
    "runtime": "-runtime.txt",
    "clock": "-clock.txt",
    "handshake": "-handshake.txt",
    "debug": "-debug.txt",
    "can": "-can.txt",
}


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


class RunData:
    def __init__(self, text: str, companion_text: Optional[Dict[str, str]] = None) -> None:
        self.text = text
        self.companion_text = companion_text or {}
        self.runtime_text = self.companion_text.get("runtime", "")
        self.observations: List[Observation] = []
        self.controller_can_records: List[str] = []
        self.can_capture_records: List[str] = []
        self.clock_ms: List[int] = []
        self.handshakes: List[int] = []
        self.debug_steering: List[str] = []
        self.debug_timing: List[str] = []
        self.debug_velocity_records: List[Dict[str, str]] = []

        for line in text.splitlines():
            observation = OBSERVATION_RE.search(line)
            if observation:
                fields = dict(FIELD_RE.findall(observation.group("fields")))
                self.observations.append((observation.group("node"), fields))

            can_out = re.search(r"can_out::(\S+)", line)
            if can_out:
                self.controller_can_records.append("can_out::" + can_out.group(1))
                continue
            can_frame = CAN_FRAME_RE.search(line)
            if can_frame:
                self.controller_can_records.append("send: %s [%s]" % can_frame.groups())
                continue
            can_byte = CAN_BYTE_RE.search(line)
            if can_byte:
                self.controller_can_records.append("send: " + can_byte.group(1).upper())

        clock_text = self.companion_text.get("clock", "")
        self.clock_ms = [
            int(seconds) * 1000 + int(nanoseconds) // 1000000
            for seconds, nanoseconds in CLOCK_RECORD_RE.findall(clock_text)
        ]
        self.handshakes = [int(value) for value in HANDSHAKE_RE.findall(
            self.companion_text.get("handshake", ""))]
        debug_text = self.companion_text.get("debug", "")
        self.debug_steering = DEBUG_STEERING_RE.findall(debug_text)
        self.debug_timing = DEBUG_TIMING_RE.findall(debug_text)
        self.debug_velocity_records = [
            fields for fields in (
                dict(DEBUG_VELOCITY_RE.findall(record))
                for record in DEBUG_RECORD_SEPARATOR_RE.split(debug_text)
            ) if fields
        ]
        self.can_capture_records = [
            match.group("frame").upper()
            for line in self.companion_text.get("can", "").splitlines()
            if (match := CAN_DUMP_RE.match(line))
        ]

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


def check_debug_velocity_consistency(records: Sequence[Dict[str, str]]) -> Tuple[str, str]:
    if not records:
        return "FAIL", "debug capture is missing"

    checked_records = 0
    mismatches = 0
    incomplete_records = 0
    for fields in records:
        try:
            desired_velocity = float(fields["desired_velocity"])
            current_velocity = float(fields["current_velocity"])
            error_velocity = float(fields["error_velocity"])
        except (KeyError, ValueError):
            incomplete_records += 1
            continue

        checked_records += 1
        if not math.isclose(
            error_velocity,
            desired_velocity - current_velocity,
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            mismatches += 1

    status = "PASS" if not mismatches and not incomplete_records else "FAIL"
    details = "%d records checked; %d mismatches, %d incomplete" % (
        checked_records, mismatches, incomplete_records)
    return status, details


def format_table(rows: Sequence[Tuple[str, str, str]]) -> None:
    print("CHECK | STATUS | DETAILS")
    print("----- | ------ | -------")
    for name, status, details in rows:
        print("%s | %s | %s" % (name, status, details))


def reduce_run(data: RunData, expected_substeps: int, minimum_sim_ms: int) -> List[Tuple[str, str, str]]:
    rows: List[Tuple[str, str, str]] = []
    mode_count = len(re.findall(r"Simulation clock mode enabled", data.text))
    rows.append(("sim mode enabled on bridge and controller",
                 "PASS" if mode_count >= 2 else "FAIL",
                 "%d enabled startup messages" % mode_count))
    rows.append(("direct CAN path",
                 "PASS" if "Direct CAN communication is enabled" in data.text else "FAIL",
                 "controller selected direct CAN"))

    if data.runtime_text:
        true_parameter_count = len(re.findall(r"Boolean value is: True", data.runtime_text))
        rows.append(("use_sim_time parameters",
                     "PASS" if true_parameter_count >= 2 else "FAIL",
                     "%d true Boolean values in runtime artifact" % true_parameter_count))
    else:
        rows.append(("use_sim_time parameters", "FAIL", "runtime artifact is missing"))

    bridge_handshake_records = [
        fields for node, fields in data.observations
        if node == "bridge" and ("handshake_received" in fields or fields.get("summary") == "1")
    ]
    bridge_clock_records = [
        fields for node, fields in data.observations
        if node == "bridge" and "clock_published" in fields
    ]
    controller_records = [
        fields for node, fields in data.observations
        if node == "controller" and "clock_received" in fields
    ]
    rows.append(("bridge handshake observations", "PASS" if bridge_handshake_records else "FAIL",
                 "%d sampled records" % len(bridge_handshake_records)))
    rows.append(("bridge clock observations", "PASS" if bridge_clock_records else "FAIL",
                 "%d sampled records" % len(bridge_clock_records)))
    rows.append(("controller clock observations", "PASS" if controller_records else "FAIL",
                 "%d sampled records" % len(controller_records)))

    bridge = latest_summary(data, "bridge")
    controller = latest_summary(data, "controller")

    if controller:
        clock_received = as_int(controller, "clock_received")
        control_invocations = as_int(controller, "control_invocations")
        zero_clock_messages = as_int(controller, "zero_clock_messages")
        handshakes_sent = as_int(controller, "handshakes_sent")
        if None not in (clock_received, control_invocations, zero_clock_messages):
            expected_controls = clock_received - zero_clock_messages
            status = "PASS" if control_invocations == expected_controls else "FAIL"
            rows.append(("one control cycle per nonzero clock", status,
                         "%d controls / %d eligible clocks" %
                         (control_invocations, expected_controls)))
        else:
            rows.append(("one control cycle per nonzero clock", "FAIL", "summary fields incomplete"))
        if None not in (clock_received, handshakes_sent):
            status = "PASS" if handshakes_sent == clock_received else "FAIL"
            rows.append(("one sim_time_increase per clock", status,
                         "%d handshakes / %d clocks" % (handshakes_sent, clock_received)))
        else:
            rows.append(("one sim_time_increase per clock", "FAIL", "summary fields incomplete"))
    else:
        rows.append(("one control cycle per nonzero clock", "FAIL", "controller summary missing"))
        rows.append(("one sim_time_increase per clock", "FAIL", "controller summary missing"))

    if bridge:
        handshakes_received = as_int(bridge, "handshakes_received")
        requested_substeps = as_int(bridge, "requested_substeps")
        cumulative_substeps = as_int(bridge, "cumulative_substeps")
        clock_published = as_int(bridge, "clock_published")
        non_ten_handshakes = as_int(bridge, "non_ten_handshakes")
        substep_mismatches = as_int(bridge, "substep_mismatches")
        if None not in (handshakes_received, requested_substeps, cumulative_substeps,
                        non_ten_handshakes, substep_mismatches):
            status = "PASS" if (
                requested_substeps == handshakes_received * expected_substeps and
                cumulative_substeps == requested_substeps and
                non_ten_handshakes == 0 and
                substep_mismatches == 0
            ) else "FAIL"
            rows.append(("ten V-ESI substeps per handshake", status,
                         "%d requested / %d completed over %d handshakes" %
                         (requested_substeps, cumulative_substeps, handshakes_received)))
        else:
            rows.append(("ten V-ESI substeps per handshake", "FAIL", "summary fields incomplete"))
        if None not in (clock_published, handshakes_received):
            valid_publication_counts = (handshakes_received, handshakes_received + 1)
            status = "PASS" if clock_published in valid_publication_counts else "FAIL"
            if clock_published == handshakes_received + 1:
                detail = "%d publications / %d handshakes including initial clock" % (
                    clock_published, handshakes_received)
            elif clock_published == handshakes_received:
                detail = "%d publications / %d handshakes; shutdown snapshot may omit final callback" % (
                    clock_published, handshakes_received)
            else:
                detail = "%d publications / %d handshakes; expected equal or equal plus initial clock" % (
                    clock_published, handshakes_received)
            rows.append(("one clock publication per handshake", status, detail))
        else:
            rows.append(("one clock publication per handshake", "FAIL", "summary fields incomplete"))
        simulated_ms = as_int(bridge, "sim_time_ms")
        rows.append(("minimum logical runtime", "PASS" if simulated_ms is not None and simulated_ms >= minimum_sim_ms else "FAIL",
                     "sim_time_ms=%s (minimum %d)" % (simulated_ms, minimum_sim_ms)))
    else:
        rows.append(("ten V-ESI substeps per handshake", "FAIL", "bridge summary missing"))
        rows.append(("one clock publication per handshake", "FAIL", "bridge summary missing"))
        rows.append(("minimum logical runtime", "FAIL", "bridge summary missing"))

    if data.clock_ms:
        clock_intervals = [current - previous for previous, current in zip(data.clock_ms, data.clock_ms[1:])]
        status = "PASS" if len(clock_intervals) > 0 and all(
            interval == expected_substeps for interval in clock_intervals) else "FAIL"
        rows.append(("captured clock advances per handshake", status,
                     "%d samples, %d ms intervals" % (len(data.clock_ms), expected_substeps)))
    else:
        rows.append(("captured clock advances per handshake", "FAIL", "clock capture is missing"))

    wall_timer_message = "Use Wall Clock (system clock)." in data.text
    rows.append(("no wall-clock acquisition path in sim mode",
                 "PASS" if not wall_timer_message else "FAIL",
                 "wall-clock startup message %s" % ("found" if wall_timer_message else "not found")))
    periodic_timer_messages = re.findall(
        r"publish_intervals\.(pure_pursuit_timer|long_control_timer|control_timer|state_machine_timer):",
        data.text)
    rows.append(("no periodic controller timers in sim mode",
                 "PASS" if not periodic_timer_messages else "FAIL",
                 "%d periodic timer messages" % len(periodic_timer_messages)))
    executor_message = "Spinning with single-threaded executor (deterministic sim mode)."
    rows.append(("single-threaded sim executor", "PASS" if executor_message in data.text else "FAIL",
                 "deterministic executor startup message %s" %
                 ("found" if executor_message in data.text else "not found")))

    if data.handshakes:
        handshake_status = "PASS" if all(value == expected_substeps for value in data.handshakes) else "FAIL"
        rows.append(("captured handshake values", handshake_status,
                     "checked %d topic samples" % len(data.handshakes)))
    else:
        rows.append(("captured handshake values", "FAIL", "handshake capture is missing"))

    if data.debug_timing:
        timing_values = []
        for _, value in data.debug_timing:
            try:
                timing_values.append(float(value))
            except ValueError:
                pass
        timing_status = "PASS" if timing_values and all(abs(value - 0.01) <= 1e-9 for value in timing_values) else "FAIL"
        rows.append(("controller debug timing", timing_status,
                     "checked %d PID timing fields against 0.01 s" % len(timing_values)))
    else:
        rows.append(("controller debug timing", "FAIL", "debug capture is missing"))

    velocity_status, velocity_details = check_debug_velocity_consistency(
        data.debug_velocity_records)
    rows.append(("debug velocity snapshot consistency", velocity_status, velocity_details))

    if data.controller_can_records:
        rows.append(("controller CAN output records captured", "PASS",
                     "%d normalized records" % len(data.controller_can_records)))
    else:
        rows.append(("controller CAN output records captured", "WARN",
                     "enable logging.sent_can_frames for payload comparison"))
    if data.can_capture_records:
        rows.append(("CAN capture records captured", "PASS",
                     "%d normalized frames" % len(data.can_capture_records)))
    else:
        rows.append(("CAN capture records captured", "WARN",
                     "candump artifact is unavailable; CAN comparison will be skipped"))
    return rows


def normalized_summary_records(data: RunData) -> List[str]:
    records = []
    for node in ("bridge", "controller"):
        fields = latest_summary(data, node)
        if fields:
            comparable_fields = {
                key: value for key, value in fields.items()
                if key != "clock_published"
            }
            records.append("%s %s" % (
                node, " ".join("%s=%s" % item for item in sorted(comparable_fields.items()))))
    return records


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


def compare_clock_window(first: RunData, second: RunData, target: Sequence[int], expected_step_ms: int) -> bool:
    first_values = clock_window_values(first, target)
    second_values = clock_window_values(second, target)
    missing = len(target) - min(len(first_values), len(second_values))
    first_intervals = [current - previous for previous, current in zip(first_values, first_values[1:])]
    second_intervals = [current - previous for previous, current in zip(second_values, second_values[1:])]
    complete = bool(target) and not missing and len(first_values) == len(second_values)
    interval_ok = (
        len(first_intervals) == max(0, len(target) - 1) and
        len(second_intervals) == max(0, len(target) - 1) and
        all(interval == expected_step_ms for interval in first_intervals + second_intervals)
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


def fixed_prefix(values: Sequence[str], count: int) -> Sequence[str]:
    return values[:count]


def compare_fixed_capture(label: str, first: Sequence[str], second: Sequence[str], count: int,
                          limit: int) -> bool:
    if len(first) < count or len(second) < count:
        print("%s: FAIL (need %d records; run-a=%d run-b=%d)" %
              (label, count, len(first), len(second)))
        return True
    return compare_records(label, fixed_prefix(first, count), fixed_prefix(second, count), limit)


def compare_runs(first: RunData, second: RunData, expected_substeps: int,
                 handshake_count: Optional[int], start_sim_ms: Optional[int],
                 max_diff_lines: int) -> bool:
    comparison_failed = False
    comparison_failed |= compare_records(
        "summary counters",
        normalized_summary_records(first),
        normalized_summary_records(second),
        max_diff_lines)

    target = common_clock_window(first, second, start_sim_ms, handshake_count)
    requested_count = handshake_count if handshake_count is not None else len(target)
    if handshake_count is not None and len(target) < handshake_count:
        print("logical clock window: FAIL (requested %d common steps, found %d)" %
              (handshake_count, len(target)))
        comparison_failed = True
    comparison_failed |= compare_clock_window(first, second, target, expected_substeps)

    if handshake_count is None:
        print("fixed-window captures: WARN (pass --handshake-count to compare topic and payload prefixes)")
        print("payload comparisons: SKIP (unbounded captures are not logically aligned)")
        return comparison_failed

    comparison_failed |= compare_fixed_capture(
        "handshake topic values", [str(value) for value in first.handshakes],
        [str(value) for value in second.handshakes], requested_count, max_diff_lines)
    comparison_failed |= compare_fixed_capture(
        "controller steering records", first.debug_steering, second.debug_steering,
        requested_count, max_diff_lines)
    if first.controller_can_records and second.controller_can_records:
        comparison_failed |= compare_records(
            "controller CAN output records", first.controller_can_records,
            second.controller_can_records, max_diff_lines)
    else:
        print("controller CAN output records: WARN (service CAN logging missing in one or both runs)")
    if first.can_capture_records and second.can_capture_records:
        comparison_failed |= compare_records(
            "CAN capture records (bounded run)", first.can_capture_records,
            second.can_capture_records, max_diff_lines)
    else:
        print("CAN capture records (bounded run): WARN (candump artifact missing in one or both runs)")
    return comparison_failed


def main() -> int:
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument(
        "run", help="combined docker log file, directory of .log files, or - for stdin")
    argument_parser.add_argument("--compare", metavar="RUN", help="second run to compare with the first")
    argument_parser.add_argument("--expected-substeps", type=int, default=10,
                                 help="expected substeps in each controller handshake (default: 10)")
    argument_parser.add_argument("--minimum-sim-ms", type=int, default=3000,
                                 help="minimum logical runtime for an individual run (default: 3000)")
    argument_parser.add_argument(
        "--handshake-count", "--steps", dest="handshake_count", type=int,
        help="number of common positive logical-clock steps to compare")
    argument_parser.add_argument(
        "--start-sim-ms", type=int,
        help="first common logical time to compare; default is the first common nonzero time")
    argument_parser.add_argument("--max-diff-lines", type=int, default=40,
                                 help="maximum comparison diff lines to print (default: 40)")
    arguments = argument_parser.parse_args()

    if arguments.handshake_count is not None and arguments.handshake_count <= 0:
        argument_parser.error("--handshake-count must be positive")
    if arguments.expected_substeps <= 0:
        argument_parser.error("--expected-substeps must be positive")

    first_run = load_run(arguments.run)
    first_rows = reduce_run(first_run, arguments.expected_substeps, arguments.minimum_sim_ms)
    print("SIM-TIME RUN: %s" % arguments.run)
    format_table(first_rows)

    comparison_failed = False
    if arguments.compare:
        second_run = load_run(arguments.compare)
        second_rows = reduce_run(second_run, arguments.expected_substeps, arguments.minimum_sim_ms)
        print("\nSIM-TIME COMPARISON RUN: %s" % arguments.compare)
        format_table(second_rows)
        print("\nDETERMINISM COMPARISON")
        comparison_failed = compare_runs(
            first_run, second_run, arguments.expected_substeps,
            arguments.handshake_count, arguments.start_sim_ms, arguments.max_diff_lines)
        comparison_failed |= any(status == "FAIL" for _, status, _ in second_rows)

    return 1 if comparison_failed or any(status == "FAIL" for _, status, _ in first_rows) else 0


if __name__ == "__main__":
    sys.exit(main())