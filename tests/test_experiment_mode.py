from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from dataclasses import replace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

from PySide6.QtWidgets import QApplication

from arm_console import experiment_page
from arm_console.models import AccelerationSample
from arm_console.v1_protocol import (AccFrame, AckFrame, ExperimentInfoFrame, InvalidFrame,
                                     MotionFrame, V1Decoder, V1Encoder)
from test_protocol_v1 import online_controller

app = QApplication.instance() or QApplication([])


class ExperimentModeTests(unittest.TestCase):
    def test_command_and_capability_protocol(self):
        self.assertEqual(V1Encoder.encode("GET", 11, "EXPERIMENT"), b"GET,11,EXPERIMENT\n")
        self.assertEqual(V1Encoder.encode("XMOVE", 12, 20000, 350, 100),
                         b"XMOVE,12,20000,350,100\n")
        decoder = V1Decoder()
        self.assertEqual(decoder.feed(b"EXPERIMENT,11,1,20,2000\n", 0),
                         [ExperimentInfoFrame(11, 1, 20, 2000)])
        self.assertIsInstance(decoder.feed(b"EXPERIMENT,11,1,20,3000\n", 0)[0], InvalidFrame)

    def test_travel_targets_and_ratio(self):
        self.assertEqual([experiment_page.target_for_travel(x) for x in
                          experiment_page.TRAVELS.values()], [66700, 43300, 20000])
        self.assertEqual(experiment_page.ratio_ramp_ms(100, 250, 50), 200)

    def test_old_firmware_or_mismatched_calibration_cannot_start_experiment(self):
        controller, link = online_controller()
        try:
            self.assertIsNone(controller.request_experiment_move(66700, 25, 100))
            controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
            controller.cal = replace(controller.cal, max_extension_mdeg=60000)
            self.assertIsNone(controller.request_experiment_move(66700, 25, 100))
            controller.cal = replace(controller.cal, max_extension_mdeg=90000)
            self.assertIsNone(controller.request_experiment_move(66700, 33, 100))
            self.assertFalse(any(payload.startswith(b"XMOVE,") for payload, _ in link.sent))
        finally:
            controller.shutdown()

    def test_one_run_records_actual_wire_window_and_signed_mean(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.latest_state = replace(controller.latest_state, pos_mdeg=160000,
                                          target_mdeg=160000)
        (ROOT / "runtime").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                page._start("出臂")
                self.assertIsNotNone(page.active)
                run = page.active
                payload = [p for p, _ in link.sent if p.startswith(b"XMOVE,")][-1]
                self.assertEqual(payload, f"XMOVE,{run.req_id},66700,125,100\n".encode())
                sent_at = time.monotonic()
                page.on_wire_written(payload, True, sent_at)
                for index, (x, y, z) in enumerate(((-0.2, 0.8, 1.0), (0.4, -0.1, 0.5))):
                    sample = AccelerationSample(1.0 + index, x, y, z, "g", "test", "MCU")
                    page.on_acc(AccFrame(index, index * 50, sample), sent_at + 0.1 + index * 0.05)
                page.on_motion(MotionFrame(run.req_id, "END", 200, 66700, 66700, "NONE"), sent_at + 0.3)
                page._on_result(run.req_id, "XMOVE", "DONE")
                saved = json.loads(experiment_page.DATA_PATH.read_text(encoding="utf-8"))
                self.assertEqual(len(saved), 1)
                self.assertEqual(saved[0]["sample_count"], 2)
                self.assertAlmostEqual(saved[0]["axis_stats"]["x"]["peak_abs_g"], 0.4)
                self.assertAlmostEqual(saved[0]["axis_stats"]["x"]["mean_g"], 0.1)
                self.assertAlmostEqual(saved[0]["axis_stats"]["y"]["mean_g"], 0.35)
                self.assertAlmostEqual(saved[0]["abs_axis_sum_stats"]["peak_g"], 2.0)
                self.assertAlmostEqual(saved[0]["abs_axis_sum_stats"]["mean_g"], 1.5)
                self.assertEqual(page.table.item(0, 7).text(), "2.000/1.500")
                self.assertIsNotNone(saved[0]["duration_s"])
                page._delete(saved[0]["id"])
                self.assertEqual(json.loads(experiment_page.DATA_PATH.read_text(encoding="utf-8")), [])
            finally:
                page._timer.stop()
                controller.shutdown()

    def test_return_moves_without_collecting_or_saving_experiment_data(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.latest_state = replace(controller.latest_state, pos_mdeg=66700,
                                          target_mdeg=66700)
        (ROOT / "runtime").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                self.assertEqual(page.return_button.text(), "回收到折叠位")
                page._start("回收")
                run = page.active
                self.assertIsNotNone(run)
                self.assertFalse(run.record)
                payload = [p for p, _ in link.sent if p.startswith(b"XMOVE,")][-1]
                self.assertEqual(payload, f"XMOVE,{run.req_id},160000,125,100\n".encode())
                sent_at = time.monotonic()
                page.on_wire_written(payload, True, sent_at)
                page.on_acc(AccFrame(1, 50, AccelerationSample(1.0, -0.2, 0.8, 1.0,
                                                               "g", "test", "MCU")), sent_at + 0.1)
                self.assertEqual(run.samples, [])
                page.on_motion(MotionFrame(run.req_id, "END", 200, 160000, 160000, "NONE"), sent_at + 0.3)
                page._on_result(run.req_id, "XMOVE", "DONE")
                self.assertIsNone(page.active)
                self.assertEqual(page.records, [])
                self.assertFalse(experiment_page.DATA_PATH.exists())
                self.assertIn("未写入实验记录", page.status.text())
            finally:
                page._timer.stop()
                controller.shutdown()

    def test_experiment_near_target_settle_uses_stop_and_marks_record_non_exact(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.latest_state = replace(controller.latest_state, pos_mdeg=160000,
                                          target_mdeg=160000)
        (ROOT / "runtime").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                page.travel.setCurrentIndex(2)  # Long target +20°, actual fold-to-target sweep is 140°.
                page._start("出臂")
                run = page.active
                self.assertIsNotNone(run)
                payload = [p for p, _ in link.sent if p.startswith(b"XMOVE,")][-1]
                self.assertEqual(payload, f"XMOVE,{run.req_id},20000,125,100\n".encode())
                page.on_wire_written(payload, True, time.monotonic())
                controller.handle_frame(AckFrame(run.req_id, "XMOVE", "ACCEPTED"))
                base = controller.latest_state.t_ms
                # A stop far from target must never count as arrival.
                for offset in (250, 750, 1500):
                    controller.handle_frame(replace(controller.latest_state, t_ms=base+offset,
                        motion="MOVING", pos_mdeg=50000, target_mdeg=20000))
                self.assertFalse(any(p.startswith(b"STOP,") for p, _ in link.sent))
                for offset in (1800, 2200, 2850):
                    controller.handle_frame(replace(controller.latest_state, t_ms=base+offset,
                        motion="MOVING", pos_mdeg=22800, target_mdeg=20000))
                stop = [p for p, _ in link.sent if p.startswith(b"STOP,")][-1]
                stop_req = int(stop.split(b",")[1])
                controller.handle_frame(AckFrame(stop_req, "STOP", "APPLIED"))
                self.assertIsNotNone(page.active)  # ACK alone is not completion.
                query_id = controller._move_settle.stop_state_query_id
                controller.handle_frame(replace(controller.latest_state, t_ms=base+2900,
                    motion="IDLE", pos_mdeg=22800, target_mdeg=20000, reply_to=query_id,
                    fault_code="NONE"))
                self.assertIsNone(page.active)
                self.assertEqual(page.records[-1]["terminal"], "SETTLED")
                self.assertFalse(page.records[-1]["comparable"])
                self.assertEqual(page.records[-1]["end_pos_mdeg"], 22800)
                self.assertIn("2.8°", page.records[-1]["result"])
            finally:
                page._timer.stop()
                controller.shutdown()

    def test_fault_recovery_waits_for_fresh_state_without_motion(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        page = experiment_page.ExperimentPage(controller)
        try:
            fault = replace(controller.latest_state, motion="FAULT", fault_code="MOTION_TIMEOUT",
                            pos_mdeg=22800, target_mdeg=20000)
            controller.handle_frame(fault)
            self.assertTrue(page.recover_button.isEnabled())
            before = len(link.sent)
            page.recover_button.click()
            stop = link.sent[-1][0]
            self.assertTrue(stop.startswith(b"STOP,"))
            self.assertFalse(page.recover_button.isEnabled())
            controller.handle_frame(AckFrame(int(stop.split(b",")[1]), "STOP", "APPLIED"))
            self.assertIn("等待新鲜 STATE", page.status.text())
            query_id = controller._query_ids["STATE"]
            controller.handle_frame(replace(fault, t_ms=fault.t_ms+100, motion="IDLE",
                                            fault_code="NONE", reply_to=query_id))
            self.assertIn("故障恢复已确认", page.status.text())
            self.assertFalse(page.recover_button.isEnabled())
            self.assertTrue(all(p.startswith((b"STOP,", b"GET,")) for p, _ in link.sent[before:]))
        finally:
            page._timer.stop()
            controller.shutdown()


if __name__ == "__main__":
    unittest.main()
