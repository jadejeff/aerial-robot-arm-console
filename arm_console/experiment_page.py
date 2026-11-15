"""Measured one-shot arm experiments. All acceleration figures are in g, gravity included."""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import json
from pathlib import Path
import statistics
import time
import uuid

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QComboBox, QFormLayout, QHBoxLayout, QLabel,
                               QPushButton, QSpinBox, QTableWidget, QTableWidgetItem,
                               QVBoxLayout, QWidget, QHeaderView)

from .v1_protocol import AccFrame, MotionFrame
from .v1_state import ExperimentController, FOLD_TARGET_MDEG, WORKING_ZERO_MDEG


MAX_TRAVEL_DEG = 70.0
STRUCTURE_SPEEDS = {"低速": 25, "中速": 45, "高速": 70}
TRAVELS = {"短": round(MAX_TRAVEL_DEG / 3, 1),
           "中": round(MAX_TRAVEL_DEG * 2 / 3, 1), "长": MAX_TRAVEL_DEG}
DATA_PATH = Path(__file__).resolve().parent.parent / "runtime" / "experiment_records.json"


def target_for_travel(travel_deg: float) -> int:
    return WORKING_ZERO_MDEG - round(travel_deg * 10) * 100


def ratio_ramp_ms(distance_deg: float, speed_dps: int, ratio_percent: int) -> int:
    """Each ramp's acceleration distance is half of v*t; both ramps total q*D."""
    raw = round((ratio_percent / 100) * distance_deg / speed_dps * 1000 / 20) * 20
    if not 20 <= raw <= 2000:
        raise ValueError("此行程／速度对应的加减速时间超出固件 20–2000 ms；请改用时间模式")
    return raw


@dataclass
class ActiveRun:
    req_id: int
    params: dict
    queued_at: float
    record: bool = True
    tx_at: float | None = None
    tx_wall: str | None = None
    end_at: float | None = None
    end_mcu_ms: int | None = None
    end_pos_mdeg: int | None = None
    samples: list[tuple[float, AccFrame]] = field(default_factory=list)
    terminal: str | None = None


class ExperimentPage(QWidget):
    def __init__(self, controller: ExperimentController):
        super().__init__()
        self.controller = controller
        self.records: list[dict] = []
        self.active: ActiveRun | None = None
        self._recovery_req_id: int | None = None
        self._recovery_query_id: int | None = None
        self._recovery_requested_at: float | None = None
        self._load()
        root = QVBoxLayout(self)
        root.addWidget(QLabel("实验模式｜仅记录出臂；回收只执行动作；补偿臂挡位仅用于记录，不改变控制信号"))
        root.addWidget(QLabel("起点：折叠位；短／中／长相对正常位前伸 23.3°／46.7°／70°。结构测试加减速各 100 ms。"))
        form = QFormLayout()
        self.kind = QComboBox()
        self.kind.addItems(("结构测试", "信号测试"))
        form.addRow("实验类别", self.kind)
        self.gear = QComboBox()
        self.gear.addItems(("1 最短", "2 较短", "3 中间", "4 较长", "5 最长"))
        form.addRow("补偿臂挡位（记录用）", self.gear)
        self.travel = QComboBox()
        for label, degrees in TRAVELS.items():
            target = target_for_travel(degrees)
            self.travel.addItem(f"{label} · 相对正常位 {degrees:.1f}° · 折叠起点约 {(FOLD_TARGET_MDEG-target)/1000:.1f}°（目标 +{target/1000:.1f}°）",
                                (label, degrees))
        form.addRow("行程", self.travel)
        self.structure_speed = QComboBox()
        for name, percent in STRUCTURE_SPEEDS.items():
            self.structure_speed.addItem(f"{name} · {percent}%", percent)
        form.addRow("结构测试速度", self.structure_speed)
        self.signal_speed = QSpinBox()
        self.signal_speed.setRange(5, 70)
        self.signal_speed.setSingleStep(5)
        self.signal_speed.setValue(35)
        self.signal_speed.setSuffix(" %")
        form.addRow("信号测试速度（步长 5%）", self.signal_speed)
        self.ramp_mode = QComboBox()
        self.ramp_mode.addItems(("加减速时间", "加减速行程占比"))
        form.addRow("信号调节方式（二选一）", self.ramp_mode)
        self.ramp_time = QSpinBox()
        self.ramp_time.setRange(20, 2000)
        self.ramp_time.setSingleStep(20)
        self.ramp_time.setValue(100)
        self.ramp_time.setSuffix(" ms / 段")
        form.addRow("加速／减速时间", self.ramp_time)
        self.ramp_ratio = QSpinBox()
        self.ramp_ratio.setRange(10, 100)
        self.ramp_ratio.setSingleStep(10)
        self.ramp_ratio.setValue(30)
        self.ramp_ratio.setSuffix(" %")
        form.addRow("两段合计占行程", self.ramp_ratio)
        root.addLayout(form)
        self.firmware_label = QLabel()
        root.addWidget(self.firmware_label)
        buttons = QHBoxLayout()
        self.extend_button = QPushButton("出臂并记录")
        self.return_button = QPushButton("回收到折叠位")
        self.recover_button = QPushButton("故障恢复")
        self.recover_button.setToolTip("停止并保持，等待新鲜 STATE 确认故障清除；不自动运动或操作夹爪")
        buttons.addWidget(self.extend_button)
        buttons.addWidget(self.return_button)
        buttons.addWidget(self.recover_button)
        buttons.addStretch()
        root.addLayout(buttons)
        self.fault_status = QLabel("设备故障：等待状态反馈")
        root.addWidget(self.fault_status)
        self.status = QLabel("等待真实设备反馈")
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        root.addWidget(QLabel("出臂记录（g，含重力；X/Y/Z 均值保留方向；|X|+|Y|+|Z| 按帧求和后统计峰/均）"))
        self.table = QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(("时间", "类别／挡位", "动作", "目标／速度／斜坡", "X 峰/均", "Y 峰/均", "Z 峰/均", "|X|+|Y|+|Z| 峰/均", "结果／样本", "删除"))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table, 1)
        self.kind.currentIndexChanged.connect(self._sync)
        self.ramp_mode.currentIndexChanged.connect(self._sync)
        self.extend_button.clicked.connect(lambda: self._start("出臂"))
        self.return_button.clicked.connect(lambda: self._start("回收"))
        self.recover_button.clicked.connect(self._recover)
        controller.command_result.connect(self._on_result)
        controller.state_changed.connect(self._sync)
        controller.device_restarted.connect(self._device_restart)
        self._timer = QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self._check_timeout)
        self._timer.start()
        self._render()
        self._sync()

    def _load(self) -> None:
        try:
            loaded = json.loads(DATA_PATH.read_text(encoding="utf-8"))
            self.records = loaded if isinstance(loaded, list) else []
        except FileNotFoundError:
            self.records = []
        except (ValueError, OSError):
            self.records = []

    def _save(self) -> None:
        DATA_PATH.parent.mkdir(parents=True, exist_ok=True)
        temp = DATA_PATH.with_suffix(".tmp")
        temp.write_text(json.dumps(self.records, ensure_ascii=False, indent=2), encoding="utf-8")
        temp.replace(DATA_PATH)

    def _sync(self) -> None:
        if self.active is not None and not self.controller.link_online:
            self._finish("链路断开，动作状态未确认", time.monotonic())
            return
        if self._recovery_req_id is not None and not self.controller.link_online:
            self._recovery_req_id = None
            self._recovery_query_id = None
            self._recovery_requested_at = None
            self.status.setText("故障恢复中链路断开，恢复结果未确认")
        structure = self.kind.currentIndex() == 0
        self.gear.setEnabled(structure)
        self.structure_speed.setEnabled(structure)
        self.signal_speed.setEnabled(not structure)
        self.ramp_mode.setEnabled(not structure)
        self.ramp_time.setEnabled(not structure and self.ramp_mode.currentIndex() == 0)
        self.ramp_ratio.setEnabled(not structure and self.ramp_mode.currentIndex() == 1)
        supported = self.controller.experiment_info is not None
        self.firmware_label.setText("实验固件 XMOVE：已确认" if supported else
                                    "实验固件 XMOVE：未确认。请烧录含实验命令的固件并重连；旧固件不能调节加减速。")
        state = self.controller.latest_state
        if not self.controller.link_online or state is None:
            self.fault_status.setText("设备故障：未知／等待在线反馈")
        elif state.fault_code == "NONE":
            self.fault_status.setText("设备故障：无")
        else:
            position = f"{state.pos_mdeg/1000:.1f}°" if state.pos_mdeg is not None else "未知"
            target = f"{state.target_mdeg/1000:.1f}°" if state.target_mdeg is not None else "未知"
            self.fault_status.setText(f"设备故障：{state.fault_code} · 实际 {position} · 目标 {target}")
        if (self._recovery_query_id is not None and state is not None and
                state.reply_to >= self._recovery_query_id):
            self.status.setText("故障恢复已确认：设备报告无故障" if state.fault_code == "NONE" and
                                state.motion == "IDLE" else
                                f"故障恢复未完成：设备仍报告 {state.fault_code}／{state.motion}")
            self._recovery_req_id = None
            self._recovery_query_id = None
            self._recovery_requested_at = None
        self.recover_button.setEnabled(bool(not self.controller.simulated and self.controller.link_online and
            self.controller.mode == "MANUAL" and state is not None and state.hx8_online and
            state.fault_code != "NONE" and self._recovery_req_id is None and
            not any(p.verb == "STOP" for p in self.controller._pending.values())))
        ready = (supported and not self.controller.simulated and self.controller.link_online and
                 self.controller.info is not None and self.controller.cal is not None and
                 self.controller.mode == "MANUAL" and state is not None and state.hx8_online and
                 state.imu_online and state.imu_hz >= 60 and state.pos_mdeg is not None and
                 state.motion == "IDLE" and state.fault_code == "NONE" and not self.controller._pending and
                 self.active is None)
        self.extend_button.setEnabled(bool(ready and abs(state.pos_mdeg-FOLD_TARGET_MDEG) <= 5000))
        self.return_button.setEnabled(bool(ready and state.pos_mdeg < FOLD_TARGET_MDEG-5000 and
                                           state.pos_mdeg >= target_for_travel(70)-10000))

    def _recover(self) -> None:
        if not self.controller.request_recover():
            self.status.setText(f"故障恢复未提交：{self.controller.command_status}")
            return
        if self.active is not None:
            self._finish("故障恢复中止采样，动作未完成", time.monotonic())
        self._recovery_req_id = self.controller._next_req - 1
        self._recovery_query_id = None
        self._recovery_requested_at = time.monotonic()
        self.status.setText("已发送停止并保持；等待设备 ACK 和新鲜 STATE 确认故障是否清除")
        self._sync()

    def _start(self, action: str) -> None:
        if self.active is not None:
            return
        state = self.controller.latest_state
        info = self.controller.info
        if state is None or state.pos_mdeg is None or info is None:
            return
        name, travel = self.travel.currentData()
        target = target_for_travel(travel) if action == "出臂" else FOLD_TARGET_MDEG
        if action == "出臂" and abs(state.pos_mdeg-FOLD_TARGET_MDEG) > 5000:
            self.status.setText("出臂必须先回到折叠位")
            return
        speed_percent = (self.structure_speed.currentData() if self.kind.currentIndex() == 0
                         else self.signal_speed.value())
        if speed_percent % 5:
            self.status.setText("速度只能按 5% 的档位设置")
            return
        if self.kind.currentIndex() == 0:
            ramp_ms = 100
            ramp_input = "固定 100 ms"
        elif self.ramp_mode.currentIndex() == 0:
            ramp_ms = self.ramp_time.value()
            if ramp_ms % 20:
                self.status.setText("每段加减速时间只能按 20 ms 的档位设置")
                return
            ramp_input = f"时间 {ramp_ms} ms"
        else:
            try:
                if self.ramp_ratio.value() % 10:
                    raise ValueError("加减速行程占比只能按 10% 的档位设置")
                speed_dps = max(1, round(info.max_speed_dps * speed_percent / 100))
                ramp_ms = ratio_ramp_ms(abs(target-state.pos_mdeg)/1000, speed_dps, self.ramp_ratio.value())
            except ValueError as exc:
                self.status.setText(str(exc))
                return
            ramp_input = f"占比 {self.ramp_ratio.value()}% → 每段 {ramp_ms} ms"
        params = {"test": self.kind.currentText(), "gear": self.gear.currentText() if self.kind.currentIndex()==0 else None,
                  "action": action, "travel": name, "travel_deg": travel, "start_mdeg": state.pos_mdeg,
                  "target_mdeg": target, "commanded_delta_deg": abs(target-state.pos_mdeg)/1000,
                  "speed_percent": speed_percent,
                  "speed_dps": round(info.max_speed_dps*speed_percent/100), "ramp_ms": ramp_ms,
                  "ramp_input": ramp_input, "device_boot_id": info.boot_id}
        req = self.controller.request_experiment_move(target, speed_percent, ramp_ms)
        if req is None:
            self.status.setText(self.controller.command_status)
            return
        self.active = ActiveRun(req, params, time.monotonic(), record=action == "出臂")
        self.status.setText(f"请求 {req} 已排队；等待实际串口写出和设备结束回执")
        self._sync()

    def on_wire_written(self, payload: bytes, success: bool, written_at: float) -> None:
        run = self.active
        if run is None or not payload.startswith(f"XMOVE,{run.req_id},".encode()):
            return
        if not success:
            self._finish("串口写入失败", written_at)
            return
        if run.tx_at is None:  # retries do not reset the measurement window
            run.tx_at = written_at
            if run.record:
                run.tx_wall = datetime.now().astimezone().isoformat(timespec="milliseconds")
            self.status.setText(f"请求 {run.req_id} 已写出，" + ("正在采样" if run.record else "正在回收（不记录）"))

    def on_acc(self, frame: AccFrame, received_at: float) -> None:
        run = self.active
        if (run is not None and run.record and run.tx_at is not None and
                received_at >= run.tx_at and run.terminal is None):
            run.samples.append((received_at, frame))

    def on_motion(self, frame: MotionFrame, received_at: float) -> None:
        run = self.active
        if run is None or frame.req_id != run.req_id:
            return
        if frame.stage in ("END", "ABORT") and run.end_at is None:
            run.end_at = received_at
            run.end_mcu_ms = frame.t_ms
            run.end_pos_mdeg = frame.pos_mdeg
            run.terminal = frame.stage
            if frame.stage == "ABORT":
                error = (abs(frame.pos_mdeg - frame.target_mdeg) / 1000
                         if frame.pos_mdeg is not None and frame.target_mdeg is not None else None)
                detail = (f"；实际 {frame.pos_mdeg / 1000:.1f}°，距目标 {error:.1f}°"
                          if error is not None else "")
                self._finish(f"中止：{frame.reason}{detail}", received_at)

    def _on_result(self, req_id: int, verb: str, result: str) -> None:
        if req_id == self._recovery_req_id and verb == "STOP":
            if result == "APPLIED":
                self._recovery_query_id = self.controller._query_ids.get("STATE")
                self.status.setText("STOP 已确认；等待新鲜 STATE 核对故障")
            else:
                self.status.setText(f"故障恢复失败：{result}")
                self._recovery_req_id = None
                self._recovery_query_id = None
                self._recovery_requested_at = None
            self._sync()
            return
        if self.active is not None and self.active.req_id == req_id:
            if result == "DONE":
                outcome = "完成" if self.active.end_at is not None else "完成，但未收到动作结束帧"
            elif result == "SETTLED":
                state = self.controller.latest_state
                self.active.end_at = time.monotonic()
                self.active.end_mcu_ms = state.t_ms if state is not None else None
                self.active.end_pos_mdeg = state.pos_mdeg if state is not None else None
                self.active.terminal = "SETTLED"
                error = (abs(state.pos_mdeg-self.active.params["target_mdeg"])/1000
                         if state is not None and state.pos_mdeg is not None else None)
                outcome = (f"近目标稳定保持：误差 {error:.1f}°（非精确到位）" if error is not None
                           else "近目标稳定保持（非精确到位）")
            else:
                outcome = f"失败：{result}"
            self._finish(outcome, time.monotonic())

    def _check_timeout(self) -> None:
        if (self._recovery_req_id is not None and self._recovery_requested_at is not None and
                time.monotonic()-self._recovery_requested_at > 5):
            self.status.setText("故障恢复超时：未收到完整回执；请检查设备 STATE")
            self._recovery_req_id = None
            self._recovery_query_id = None
            self._recovery_requested_at = None
            self._sync()
        if self.active is not None and time.monotonic()-self.active.queued_at > 15:
            self._finish("回执超时，动作状态未确认", time.monotonic())

    def _device_restart(self) -> None:
        if self.active is not None:
            self._finish("设备重启，动作状态未确认", time.monotonic())

    def _finish(self, result: str, now: float) -> None:
        run = self.active
        if run is None:
            return
        self.active = None
        if not run.record:
            self.status.setText(f"请求 {run.req_id}：回收{result}；未写入实验记录")
            self._sync()
            return
        end = run.end_at or now
        samples = [f for t, f in run.samples if run.tx_at is not None and t <= end]
        values = [[getattr(f.sample, axis) for f in samples] for axis in ("ax", "ay", "az")]
        stats = [{"peak_abs_g": max(map(abs, v)), "mean_g": statistics.fmean(v),
                  "mean_abs_g": statistics.fmean(map(abs, v))} if v else None for v in values]
        abs_axis_sums = [sum(abs(component) for component in components) for components in zip(*values)]
        abs_axis_sum_stats = ({"peak_g": max(abs_axis_sums), "mean_g": statistics.fmean(abs_axis_sums)}
                              if abs_axis_sums else None)
        record = {"id": str(uuid.uuid4()), "recorded_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                  "req_id": run.req_id, **run.params, "tx_at": run.tx_wall,
                  "duration_s": round(end-run.tx_at, 3) if run.tx_at is not None and run.end_at is not None else None,
                  "end_mcu_ms": run.end_mcu_ms, "end_pos_mdeg": run.end_pos_mdeg,
                  "actual_delta_deg": (abs(run.end_pos_mdeg-run.params["start_mdeg"])/1000
                                       if run.end_pos_mdeg is not None else None),
                  "comparable": result == "完成", "terminal": run.terminal,
                  "sample_count": len(samples),
                  "first_acc_mcu_ms": samples[0].sample_ms if samples else None,
                  "last_acc_mcu_ms": samples[-1].sample_ms if samples else None,
                  "axis_stats": dict(zip(("x", "y", "z"), stats)),
                  "abs_axis_sum_stats": abs_axis_sum_stats, "result": result,
                  "unit": "g including gravity",
                  "window": ("PC monotonic TX to verified STOP/STATE RX" if run.terminal == "SETTLED"
                             else "PC monotonic TX to MCU terminal RX")}
        self.records.append(record)
        try:
            self._save()
        except OSError as exc:
            self.status.setText(f"请求 {run.req_id}：{result}；记录保存失败：{exc}")
            self._render()
            self._sync()
            return
        self._render()
        self.status.setText(f"请求 {run.req_id}：{result}；采集 {len(samples)} 帧，已保存至 {DATA_PATH}")
        self._sync()

    def _delete(self, record_id: str) -> None:
        self.records = [r for r in self.records if r.get("id") != record_id]
        self._save()
        self._render()

    def _render(self) -> None:
        self.table.setRowCount(len(self.records))
        for row, record in enumerate(self.records):
            axes = record.get("axis_stats", {})
            def fmt(axis: str) -> str:
                stat = axes.get(axis)
                if not stat:
                    return "无有效样本"
                return f"{stat['peak_abs_g']:.3f}/{stat.get('mean_g', stat['mean_abs_g']):+.3f}"
            aggregate = record.get("abs_axis_sum_stats")
            aggregate_text = (f"{aggregate['peak_g']:.3f}/{aggregate['mean_g']:.3f}" if aggregate else
                              "旧记录未计算" if "abs_axis_sum_stats" not in record and record.get("sample_count", 0) else
                              "无有效样本")
            values = (record.get("recorded_at", "")[:19].replace("T", " "),
                      f"{record.get('test','')} {record.get('gear') or ''}", record.get("action", ""),
                      f"{record.get('target_mdeg',0)/1000:.1f}° / {record.get('speed_percent')}% / {record.get('ramp_ms')}ms",
                      fmt("x"), fmt("y"), fmt("z"), aggregate_text,
                      f"{record.get('result','')} / {record.get('sample_count',0)}")
            for col, value in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(value))
            button = QPushButton("删除")
            button.clicked.connect(lambda checked=False, rid=record.get("id"): self._delete(rid))
            self.table.setCellWidget(row, 9, button)
