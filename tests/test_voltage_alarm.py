from __future__ import annotations

import os
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

from PySide6.QtWidgets import QApplication

from arm_console.models import ConnectionState
from arm_console.window import MainWindow, VoltageAlarmGate

app = QApplication.instance() or QApplication([])


class VoltageAlarmTests(unittest.TestCase):
    def test_transient_sag_is_ignored_and_sustained_low_voltage_alerts_once(self):
        gate = VoltageAlarmGate()
        self.assertFalse(gate.observe(10_300, valid=True, received_at=0.0))
        self.assertFalse(gate.observe(10_600, valid=True, received_at=0.5))
        self.assertFalse(gate.observe(10_450, valid=True, received_at=1.0))
        self.assertFalse(gate.observe(10_400, valid=True, received_at=2.0))
        self.assertTrue(gate.observe(10_350, valid=True, received_at=3.0))
        self.assertFalse(gate.observe(10_300, valid=True, received_at=4.0))
        self.assertFalse(gate.observe(10_700, valid=True, received_at=5.0))
        self.assertFalse(gate.observe(10_300, valid=True, received_at=6.0))
        self.assertFalse(gate.observe(10_800, valid=True, received_at=7.0))
        self.assertFalse(gate.observe(10_300, valid=True, received_at=8.0))
        self.assertFalse(gate.observe(10_250, valid=True, received_at=9.0))
        self.assertTrue(gate.observe(10_200, valid=True, received_at=10.0))

    def test_invalid_feedback_and_gaps_break_low_voltage_streak(self):
        gate = VoltageAlarmGate()
        self.assertFalse(gate.observe(10_500, valid=True, received_at=0.0))
        self.assertFalse(gate.observe(None, valid=False, received_at=1.0))
        self.assertFalse(gate.observe(10_500, valid=True, received_at=2.0))
        self.assertFalse(gate.observe(10_400, valid=True, received_at=5.0))
        self.assertFalse(gate.observe(10_300, valid=True, received_at=6.0))
        self.assertTrue(gate.observe(10_200, valid=True, received_at=7.0))

    def test_real_fresh_state_shows_nonmodal_warning_without_repeating(self):
        window = MainWindow()
        try:
            window.serial.state = ConnectionState.OPEN
            window.controller.link_online = True
            with patch("arm_console.window.QMessageBox") as box:
                base = time.monotonic()
                for t_ms, voltage in ((1, 10_500), (2, 10_450), (3, 10_400), (4, 10_350)):
                    now = base + (t_ms - 1) * 0.5
                    window.controller._last_state_received_at = now
                    window.controller.latest_state = SimpleNamespace(
                        t_ms=t_ms, hx8_online=True, voltage_mv=voltage)
                    with patch("arm_console.window.time.monotonic", return_value=now):
                        window._check_low_voltage()
                box.assert_not_called()  # four rapid frames are still a short sag

                # The warning is owned by MainWindow and must keep its
                # accumulation window when the user opens experiment mode.
                window.switch_page(3)
                self.assertEqual(window.pages.currentIndex(), 3)
                window.controller._last_state_received_at = base + 2.0
                window.controller.latest_state = SimpleNamespace(t_ms=5, hx8_online=True, voltage_mv=10_300)
                with patch("arm_console.window.time.monotonic", return_value=window.controller._last_state_received_at):
                    window._check_low_voltage()
                self.assertEqual(box.call_count, 1)
                self.assertFalse(box.return_value.setModal.call_args.args[0])
                self.assertIn("10.60 V", box.return_value.setText.call_args.args[0])
                window._check_low_voltage()
                self.assertEqual(box.call_count, 1)
                window.controller.simulated = True
                window.controller.latest_state = SimpleNamespace(t_ms=6, hx8_online=True, voltage_mv=10_100)
                window._check_low_voltage()
                self.assertEqual(box.call_count, 1)
        finally:
            window.controller.latest_state = None
            window.serial.state = ConnectionState.DISCONNECTED
            window.close()


if __name__ == "__main__":
    unittest.main()
