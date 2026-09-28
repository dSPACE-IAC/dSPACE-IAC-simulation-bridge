import contextlib
import io
import unittest

from sim_time_observability import (
    RunData,
    check_debug_timing,
    compare_step_capture,
    reduce_run,
)


def make_debug_record(step, timing):
    return "\n".join((
        "sim_step: %d" % step,
        "output_steering: %d.0" % step,
        "vel_pid_dt: %s" % timing,
        "acc_pid_dt: %s" % timing,
        "steering_dt: %s" % timing,
    ))


class SimTimeObservabilityTest(unittest.TestCase):
    def test_run_data_groups_debug_and_can_by_step(self):
        debug = "\n---\n".join((make_debug_record(2, "0.01"), make_debug_record(1, "0.01")))
        controller_log = "\n".join((
            "[INFO] can_out::steering_cmd sim_step=2",
            "[INFO] send: 0x123 [2]",
            "[INFO] send: AA",
            "[INFO] send: BB",
        ))
        can_capture = "\n".join((
            "(1.000000) can0 130#AA",
            "(1.000001) can0 7FF#0100000000000000",
            "(1.000002) can0 131#BB",
            "(1.000003) can0 7FF#0200000000000000",
        ))

        data = RunData(controller_log, {"debug": debug, "can": can_capture})

        self.assertEqual(data.debug_steering_by_step, {2: ["2.0"], 1: ["1.0"]})
        self.assertEqual(
            data.controller_can_by_step[2],
            ["can_out::steering_cmd | send: 0x123 [2] | send: AA | send: BB"],
        )
        self.assertEqual(data.can_marker_steps, [1, 2])
        self.assertEqual(data.can_capture_by_step[1], ["130#AA", "7FF#0100000000000000"])
        self.assertEqual(data.can_capture_by_step[2], ["131#BB", "7FF#0200000000000000"])

    def test_keyed_comparison_uses_requested_step_order(self):
        first = {2: ["steer=2"], 1: ["steer=1"]}
        second = {1: ["steer=1"], 2: ["steer=2"]}
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            failed = compare_step_capture("steering", first, second, [1, 2], 20)
        self.assertFalse(failed)
        self.assertIn("PASS", output.getvalue())

    def test_timing_warmup_is_reported_separately(self):
        records = [
            {"vel_pid_dt": "0.0", "acc_pid_dt": "0.0", "steering_dt": "0.0"},
            {"vel_pid_dt": "0.0", "acc_pid_dt": "0.0", "steering_dt": "0.0"},
            {"vel_pid_dt": "0.01", "acc_pid_dt": "0.01", "steering_dt": "0.01"},
            {"vel_pid_dt": "0.03", "acc_pid_dt": "0.03", "steering_dt": "0.03"},
            {"vel_pid_dt": "0.01", "acc_pid_dt": "0.01", "steering_dt": "0.01"},
            {"vel_pid_dt": "0.01", "acc_pid_dt": "0.01", "steering_dt": "0.01"},
        ]

        rows = {name: (status, details) for name, status, details in check_debug_timing(records)}

        self.assertEqual(rows["controller debug timing warm-up"][0], "INFO")
        self.assertIn("4 startup records excluded", rows["controller debug timing warm-up"][1])
        self.assertEqual(rows["controller debug timing"][0], "PASS")

    def test_non_warmup_timing_mismatch_fails(self):
        records = [
            {"vel_pid_dt": "0.01", "acc_pid_dt": "0.01", "steering_dt": "0.01"}
            for _ in range(4)
        ]
        records.append({"vel_pid_dt": "0.01", "acc_pid_dt": "0.02", "steering_dt": "0.01"})

        rows = {name: status for name, status, _ in check_debug_timing(records)}

        self.assertEqual(rows["controller debug timing"], "FAIL")

    def test_reducer_classifies_raptor_and_direct_can_paths(self):
        raptor_rows = {
            name: status
            for name, status, _ in reduce_run(
                RunData("Raptor DBW node is used. Direct CAN communication is disabled"),
                expected_substeps=10,
                minimum_sim_ms=0,
            )
        }
        self.assertEqual(raptor_rows["Raptor DBW path"], "PASS")
        self.assertNotIn("direct CAN path", raptor_rows)
        self.assertEqual(raptor_rows["controller CAN output records captured"], "INFO")

        direct_rows = {
            name: status
            for name, status, _ in reduce_run(
                RunData("Direct CAN communication is enabled"),
                expected_substeps=10,
                minimum_sim_ms=0,
            )
        }
        self.assertEqual(direct_rows["direct CAN path"], "PASS")


if __name__ == "__main__":
    unittest.main()