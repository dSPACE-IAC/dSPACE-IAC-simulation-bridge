import contextlib
import io
import unittest

from sim_time_observability import (
    RunData,
    check_step_records,
    compare_runs,
    reduce_run,
)

STEP_HEADER = ("step,t_ms,release_ns,close_ns,wait_us,timed_out,due,missing,accepted,"
               "duplicates,late,early,arrivals_us")


def step_row(step, timed_out=0, due="1400", missing="", arrivals="1400:500", wait_us=600):
    return "%d,%d,0,0,%d,%d,%s,%s,1,0,0,0,%s" % (
        step, step * 10, wait_us, timed_out, due, missing, arrivals)


def make_step_rows(text):
    return RunData("", {"steps": text}).step_rows


class SimTimeObservabilityTest(unittest.TestCase):
    def test_run_data_groups_can_by_marker_step(self):
        can_capture = "\n".join((
            "(1.000000) can0 130#AA",
            "(1.000001) can0 7FF#0100000000000000",
            "(1.000002) can0 131#BB",
            "(1.000003) can0 7FF#0200000000000000",
        ))

        data = RunData("", {"can": can_capture})

        self.assertEqual(data.can_marker_steps, [1, 2])
        self.assertEqual(data.can_capture_by_step[1], ["130#AA", "7FF#0100000000000000"])
        self.assertEqual(data.can_capture_by_step[2], ["131#BB", "7FF#0200000000000000"])

    def test_reducer_classifies_raptor_and_direct_can_paths(self):
        raptor_rows = {
            name: status
            for name, status, _ in reduce_run(
                RunData("Raptor DBW node is used. Direct CAN communication is disabled"), 10, 0)
        }
        self.assertEqual(raptor_rows["Raptor DBW path"], "PASS")
        self.assertNotIn("direct CAN path", raptor_rows)

        direct_rows = {
            name: status
            for name, status, _ in reduce_run(
                RunData("Direct CAN communication is enabled"), 10, 0)
        }
        self.assertEqual(direct_rows["direct CAN path"], "PASS")
    def test_step_records_accept_complete_and_timed_out_steps(self):
        rows = make_step_rows("\n".join((
            STEP_HEADER,
            step_row(1, timed_out=1, missing="1400", arrivals="", wait_us=20000),
            step_row(2),
            step_row(3, due="", arrivals="", wait_us=0),
        )))

        status = {name: s for name, s, _ in check_step_records(rows, 10)}

        self.assertEqual(status["step records complete and consecutive"], "PASS")
        self.assertEqual(status["step closure consistent with due IDs"], "PASS")
        self.assertEqual(status["no frame merged into a closed step"], "PASS")

    def test_step_records_fail_on_gap_missing_arrival_and_late_merge(self):
        gap = make_step_rows("\n".join((STEP_HEADER, step_row(1), step_row(3))))
        status = {name: s for name, s, _ in check_step_records(gap, 10)}
        self.assertEqual(status["step records complete and consecutive"], "FAIL")

        unarrived = make_step_rows("\n".join((STEP_HEADER, step_row(1, arrivals=""))))
        status = {name: s for name, s, _ in check_step_records(unarrived, 10)}
        self.assertEqual(status["step closure consistent with due IDs"], "FAIL")

        merged = make_step_rows("\n".join((
            STEP_HEADER, step_row(1, arrivals="1400:900", wait_us=600))))
        status = {name: s for name, s, _ in check_step_records(merged, 10)}
        self.assertEqual(status["no frame merged into a closed step"], "FAIL")

    def test_step_records_expect_one_ms_steps_during_adaptation(self):
        rows = make_step_rows("\n".join((
            STEP_HEADER,
            "1,1,0,0,0,0,,,0,0,0,0,",
            "2,2,0,0,0,0,,,0,0,0,0,",
            "3,12,0,0,0,0,,,0,0,0,0,",
        )))
        status = {name: s for name, s, _ in check_step_records(rows, 10, 2)}
        self.assertEqual(status["step records complete and consecutive"], "PASS")
        status = {name: s for name, s, _ in check_step_records(rows, 10, 0)}
        self.assertEqual(status["step records complete and consecutive"], "FAIL")

    def test_environment_reducer_parses_adaptation(self):
        log = "\n".join((
            "Simulation stepping owned by the environment: adaptation on (3.000 s, rtf 1.000)",
            "SIM_STEP adaptation id=0x578 arrivals=300 class=scheduled period_ms=10 anchor_ms=10 first_due_ms=3010",
            "SIM_STEP adaptation complete: step=10 ms timeout=10 ms switch_ms=3000 window_ms=3000 scheduled=1 unscheduled=0",
        ))
        data = RunData(log)
        self.assertEqual((data.environment_step_ms, data.adaptation_switch_ms), (10, 3000))
        self.assertEqual(data.adaptation_ids, [("0X578", "scheduled", 10, 10)])
        status = {name: s for name, s, _ in reduce_run(data, 10, 0)}
        self.assertEqual(status["adaptation period completed"], "PASS")

    def test_environment_reducer_checks_clock_and_markers(self):
        log = "\n".join((
            "Simulation clock mode enabled",
            "Simulation stepping owned by the environment: step=10 ms timeout=20 ms",
            "SIM_STEP environment-owned stepping started.",
            "Direct CAN communication is enabled",
        ))
        good = RunData(log, {
            "clock": "\n".join(
                "clock:\n  sec: 0\n  nanosec: %d\n---" % (i * 10000000) for i in range(4)),
            "steps": "\n".join((STEP_HEADER, step_row(1), step_row(2), step_row(3))),
            "can": "\n".join("(1.0) can0 7FF#%02X00000000000000" % i for i in (1, 2, 3)),
        })
        status = {name: s for name, s, _ in reduce_run(good, 10, 0)}
        self.assertEqual(status["logical time strictly monotonic in exact steps"], "PASS")
        self.assertEqual(status["clock capture matches step records"], "PASS")
        self.assertEqual(status["captured CAN step markers"], "PASS")

        skipped = RunData(log, {
            "clock": "\n".join(
                "clock:\n  sec: 0\n  nanosec: %d\n---" % t for t in (0, 10000000, 30000000)),
            "can": "\n".join("(1.0) can0 7FF#%02X00000000000000" % i for i in (1, 3)),
        })
        status = {name: s for name, s, _ in reduce_run(skipped, 10, 0)}
        self.assertEqual(status["logical time strictly monotonic in exact steps"], "FAIL")
        self.assertEqual(status["captured CAN step markers"], "FAIL")

    def test_environment_comparison_is_informational_for_closed_loop(self):
        log = "Simulation stepping owned by the environment: step=10 ms"
        clock = "\n".join(
            "clock:\n  sec: 0\n  nanosec: %d\n---" % (i * 10000000) for i in range(3))
        first = RunData(log, {"clock": clock, "can": "\n".join((
            "(1.0) can0 57A#01", "(1.0) can0 4B0#00", "(1.0) can0 7FF#0100000000000000",
            "(2.0) can0 57A#01", "(2.0) can0 4B0#00", "(2.0) can0 7FF#0200000000000000"))})
        second = RunData(log, {"clock": clock, "can": "\n".join((
            "(1.0) can0 57A#01", "(1.0) can0 4B0#00", "(1.0) can0 7FF#0100000000000000",
            "(2.0) can0 57A#02", "(2.0) can0 4B0#00", "(2.0) can0 7FF#0200000000000000"))})
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            failed = compare_runs(first, second, 10, None, 20)
        self.assertFalse(failed)
        self.assertIn("controller command frames by step: INFO (1 of 2 steps differ, first at step 2",
                      output.getvalue())
        self.assertIn("environment CAN outputs by step: INFO (identical over 2 steps",
                      output.getvalue())

    def test_replay_run_uses_recorded_schedule_and_needs_no_stack(self):
        log = "\n".join((
            "Simulation clock mode enabled",
            "Simulation stepping owned by the environment: open-loop replay of 3 steps (12 ms) from x",
            "SIM_STEP replay schedule: step_ms=10 switch_ms=2",
            "SIM_STEP environment-owned stepping started.",
            "SIM_STEP replay complete: 3 steps, sim_time_ms=12.",
            "SIM_OBS bridge summary=1 sim_time_ms=12 cumulative_substeps=12 substep_mismatches=0 "
            "env_time_mismatches=0 env_replay_steps=3/3",
        ))
        data = RunData(log, {"clock": "\n".join(
            "clock:\n  sec: 0\n  nanosec: %d\n---" % (t * 1000000) for t in (0, 1, 2, 12))})
        self.assertTrue(data.replay)
        self.assertEqual((data.environment_step_ms, data.adaptation_switch_ms), (10, 2))
        status = {name: s for name, s, _ in reduce_run(data, 10, 0)}
        self.assertEqual(status["sim mode enabled on bridge and controller"], "PASS")
        self.assertEqual(status["controller command path"], "PASS")
        self.assertEqual(status["replay ran every recorded step"], "PASS")
        self.assertEqual(status["logical time strictly monotonic in exact steps"], "PASS")

        partial = RunData(log.replace("3/3", "2/3"))
        status = {name: s for name, s, _ in reduce_run(partial, 10, 0)}
        self.assertEqual(status["replay ran every recorded step"], "FAIL")

    def test_replay_pair_requires_identical_environment_outputs(self):
        log = "\n".join((
            "Simulation stepping owned by the environment: open-loop replay of 2 steps (20 ms) from x",
            "SIM_STEP replay schedule: step_ms=10 switch_ms=0",
        ))
        clock = "\n".join(
            "clock:\n  sec: 0\n  nanosec: %d\n---" % (i * 10000000) for i in range(3))

        def make(second_frame, ros):
            can = "\n".join((
                "(1.0) can0 4B0#00", "(1.0) can0 7FF#0100000000000000",
                "(2.0) can0 4B0#%s" % second_frame, "(2.0) can0 7FF#0200000000000000"))
            return RunData(log, {"clock": clock, "can": can, "ros:ins": ros})

        def compare(first, second):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                return compare_runs(first, second, 10, None, 20), output.getvalue()

        failed, text = compare(make("01", "a: 1\n---\na: 2\n"), make("01", "a: 1\n---\na: 2\n"))
        self.assertFalse(failed)
        self.assertIn("replay environment CAN outputs by step: PASS", text)
        self.assertIn("replay ROS output ins: PASS (2 messages identical)", text)

        failed, text = compare(make("01", "a: 1\n---\na: 2\n"), make("02", "a: 1\n---\na: 2\n"))
        self.assertTrue(failed)
        self.assertIn("replay environment CAN outputs by step: FAIL", text)

        failed, text = compare(make("01", "a: 1\n---\na: 2\n"), make("01", "a: 1\n---\na: 3\n"))
        self.assertTrue(failed)
        self.assertIn("replay ROS output ins: FAIL (2 vs 2 messages, first difference at message 1)",
                      text)


if __name__ == "__main__":
    unittest.main()