from __future__ import annotations

import time

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSpinBox,
    QScrollArea, QSlider, QVBoxLayout, QWidget,
)

from .models import VisionResult
from .black_blocks import BlackBlockConfig
from .state import ExperimentController
from .v1_state import (
    FOLD_TARGET_MDEG, GRIP_INSTALL_ANGLE_MDEG, GRIP_TEST_ANGLE_MDEG,
    MANUAL_EXTENSION_LIMIT_MDEG, WORKING_ZERO_MDEG,
)
from .vision import VisionCanvas


SPEED_MIN, SPEED_MAX = 0.0, 100.0  # 最大实际速度由INFO给出
MANUAL_SPEED_MAX = 70.0  # 模式一演示上限；视觉抓取页保持独立范围
MANUAL_ANGLE_MAX = 70.0  # 模式一界面上限；协议保护范围保持不变
ANGLE_MIN, ANGLE_MAX = 0.0, MANUAL_EXTENSION_LIMIT_MDEG / 1000


class ManualPage(QWidget):
    def __init__(self, controller: ExperimentController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.grab_flow = None
        self._manual_grab_started = False
        self._show_recovery_status = False
        root = QVBoxLayout(self)
        root.setSpacing(5)
        title = QLabel("手动模式")
        title.setObjectName("pageTitle")
        self.recover_button = QPushButton("故障恢复")
        self.recover_button.setToolTip("发送停止并保持，检查设备状态；不自动归零或操作夹爪")
        self.recover_button.setEnabled(False)
        title_row = QHBoxLayout()
        title_row.addWidget(title)
        title_row.addStretch()
        title_row.addWidget(self.recover_button)
        root.addLayout(title_row)
        grid = QGridLayout()
        self.speed_slider, self.speed_spin = self._add_control(grid, 0, "出臂速度", SPEED_MIN, MANUAL_SPEED_MAX, 50.0, "%（归一化）")
        self.angle_slider, self.angle_spin = self._add_control(
            grid, 1, "出臂角度（相对正常位）", ANGLE_MIN, MANUAL_ANGLE_MAX, 30.0, "°"
        )
        self.angle_spin.setToolTip(
            f"相对 +{WORKING_ZERO_MDEG / 1000:g}°正常位的出臂行程；绝对目标按正常位加出臂方向换算"
        )
        root.addLayout(grid)
        buttons = QHBoxLayout()
        self.start_button = QPushButton("启动出臂")
        self.start_button.setObjectName("primaryButton")
        self.grab_button = QPushButton("按当前参数抓取")
        self.zero_button = QPushButton("归零")
        self.zero_button.setToolTip(
            f"物理标定零点仍是0°；当前正常／视觉工作位为向折叠方向的绝对+{WORKING_ZERO_MDEG / 1000:g}°"
        )
        self.open_button = QPushButton("夹爪张开")
        self.close_button = QPushButton("夹爪闭合")
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.grab_button)
        buttons.addWidget(self.zero_button)
        buttons.addWidget(self.open_button)
        buttons.addWidget(self.close_button)
        buttons.addStretch()
        root.addLayout(buttons)
        self.parameters = QLabel()
        self.operation = QLabel("最近操作：尚未提交")
        self.actual = QLabel("机械臂实际状态：未知／等待反馈")
        self.position_feedback = QLabel("HX8角度：目标未知 · 实际未知 · 误差未知")
        self.voltage_label = QLabel("HX8 电压：未知")
        self.fault_label = QLabel("故障：等待设备反馈")
        status_grid = QGridLayout()
        status_grid.setHorizontalSpacing(12)
        status_grid.setVerticalSpacing(3)
        status_grid.setColumnStretch(0, 1)
        status_grid.setColumnStretch(1, 1)
        for label in (self.parameters, self.operation, self.actual, self.position_feedback,
                      self.voltage_label, self.fault_label):
            label.setWordWrap(True)
        status_grid.addWidget(self.parameters, 0, 0)
        status_grid.addWidget(self.operation, 0, 1)
        status_grid.addWidget(self.actual, 1, 0)
        status_grid.addWidget(self.position_feedback, 1, 1)
        status_grid.addWidget(self.voltage_label, 2, 0)
        status_grid.addWidget(self.fault_label, 2, 1)
        root.addLayout(status_grid)
        self.video_canvas = VisionCanvas(empty_text="视频未打开\n请选择图传设备并点击打开视频")
        root.addWidget(self.video_canvas, 1)
        self.speed_spin.valueChanged.connect(self._update_parameters)
        self.angle_spin.valueChanged.connect(self._update_parameters)
        self.start_button.clicked.connect(self._start)
        self.grab_button.clicked.connect(self._grab)
        self.zero_button.clicked.connect(self._zero)
        self.open_button.clicked.connect(lambda: self._grip("OPEN"))
        self.close_button.clicked.connect(lambda: self._grip("CLOSE"))
        self.recover_button.clicked.connect(self._recover)
        controller.state_changed.connect(self._sync_state)
        self._update_parameters()

    def _add_control(self, grid, row, label, minimum, maximum, initial, unit):
        grid.addWidget(QLabel(label), row, 0)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(round(minimum * 10), round(maximum * 10))
        slider.setValue(round(initial * 10))
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(1)
        spin.setSingleStep(0.1)
        spin.setValue(initial)
        spin.setSuffix(" " + unit)
        slider.valueChanged.connect(lambda value: spin.setValue(value / 10))
        spin.valueChanged.connect(lambda value: slider.setValue(round(value * 10)))
        grid.addWidget(slider, row, 1)
        grid.addWidget(spin, row, 2)
        return slider, spin

    def _update_parameters(self) -> None:
        sign = self.controller.cal.extension_sign if self.controller.cal and self.controller.cal.extension_sign else -1
        target = WORKING_ZERO_MDEG / 1000 + sign * self.angle_spin.value()
        maximum = self.controller.info.max_speed_dps if self.controller.info else None
        speed = (f"{round(maximum * self.speed_spin.value() / 100)}°/s（INFO 上限 {maximum}°/s）"
                 if maximum else "等待 INFO 速度上限")
        actual = self.controller.latest_state.pos_mdeg if self.controller.latest_state else None
        start = ("折叠位" if actual is not None and abs(actual - FOLD_TARGET_MDEG) <= 5_000 else
                 "正常位" if actual is not None and abs(actual - WORKING_ZERO_MDEG) <= 5_000 else "待确认")
        self.parameters.setText(
            f"待发送：速度 {self.speed_spin.value():.1f}% ≈ {speed} · 起点 {start} · "
            f"相对正常位 {self.angle_spin.value():.1f}° · 绝对目标 {target:.1f}°")

    def _start(self) -> None:
        self._manual_grab_started = False
        self._show_recovery_status = False
        speed, angle = self.speed_spin.value(), self.angle_spin.value()
        self.controller.request_extend(speed, angle)
        self.operation.setText(f"最近操作：出臂请求（速度 {speed:.1f}%，角度 {angle:.1f}°） · {self.controller.command_status}")

    def _grab(self) -> None:
        self._show_recovery_status = False
        speed, angle = self.speed_spin.value(), self.angle_spin.value()
        if self.grab_flow is None:
            self.operation.setText("最近操作：抓取流程未连接")
            return
        if self.grab_flow.start_manual_grab(speed, angle):
            self._manual_grab_started = True
            self.operation.setText(f"最近操作：抓取已提交 · 出臂 {speed:g}%／{angle:g}°，回到起点 30%")
        else:
            self.operation.setText(f"最近操作：抓取未启动 · {self.controller.command_status}")

    def _zero(self) -> None:
        self._manual_grab_started = False
        self._show_recovery_status = False
        self.controller.request_home()
        self.operation.setText(f"最近操作：归零 · {self.controller.command_status}")

    def _grip(self, action: str) -> None:
        self._manual_grab_started = False
        self._show_recovery_status = False
        self.controller.request_grip(action)
        self.operation.setText(f"最近操作：夹爪{'张开' if action == 'OPEN' else '闭合'} · {self.controller.command_status}")

    def _recover(self) -> None:
        self._manual_grab_started = False
        self._show_recovery_status = True
        self.controller.request_recover()
        self.operation.setText(f"最近操作：故障恢复 · {self.controller.command_status}")

    def _sync_state(self) -> None:
        self._update_parameters()
        self.actual.setText(f"机械臂执行状态：{self.controller.execution_status}")
        idle = (not self.controller.control_busy and not self.controller.grip_ramp_active and
                (self.grab_flow is None or not self.grab_flow.running))
        self.start_button.setEnabled(idle)
        self.grab_button.setEnabled(idle and (self.grab_flow is None or not self.grab_flow.running))
        self.zero_button.setEnabled(idle)
        self.open_button.setEnabled(idle)
        self.close_button.setEnabled(idle)
        c = self.controller
        if self._manual_grab_started and self.grab_flow is not None:
            self.operation.setText(f"最近操作：手动抓取 · {self.grab_flow.flow_label.text()}")
        if self._show_recovery_status:
            self.operation.setText(f"最近操作：故障恢复 · {c.command_status}；以设备故障反馈为准")
        s = c.latest_state
        online = not c.simulated and c.link_online and s is not None
        if online and s.target_mdeg is not None and s.pos_mdeg is not None:
            error = (s.target_mdeg - s.pos_mdeg) / 1000
            self.position_feedback.setText(
                f"HX8角度（设备反馈）：目标 {s.target_mdeg / 1000:.1f}° · 实际 {s.pos_mdeg / 1000:.1f}° · 误差 {error:+.1f}°")
        else:
            self.position_feedback.setText("HX8角度：目标未知 · 实际未知 · 误差未知")
        self.voltage_label.setText(
            f"HX8 电压：{s.voltage_mv / 1000:.2f} V" if online and s.hx8_online and s.voltage_mv is not None
            else "HX8 电压：未知／无有效反馈")
        descriptions = {
            "MOTION_TIMEOUT": "运动超时，未在规定时间内满足到位条件",
            "HX8_OFFLINE": "HX8反馈离线", "FEEDBACK_STALE": "MCU中止：HX8反馈健康检查未通过；需检查原始反馈时间、电压和状态位",
            "IMU_OFFLINE": "IMU离线", "IMU_RATE_LOW": "IMU有效采样率不足",
            "NOT_READY": "执行器未就绪", "FAULT": "设备故障，请检查反馈和机构",
            "LINK_LOST": "控制会话失联", "STOPPED": "动作已中止",
        }
        if not online:
            self.fault_label.setText("故障：未知／等待设备在线反馈")
        elif s.fault_code == "NONE":
            self.fault_label.setText("故障：无（设备反馈）")
        else:
            detail = descriptions.get(s.fault_code, "设备报告故障")
            self.fault_label.setText(f"故障：{s.fault_code} · {detail}")
        self.recover_button.setEnabled(bool(online and c.mode == "MANUAL" and s.hx8_online
            and s.fault_code != "NONE" and not any(p.verb == "STOP" for p in c._pending.values())))


class VisionPage(QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        root = QVBoxLayout(self)
        heading = QLabel(title)
        heading.setObjectName("pageTitle")
        root.addWidget(heading)
        self.canvas = VisionCanvas()
        root.addWidget(self.canvas, 1)
        self.source_label = QLabel("视觉源：未接入")
        self.target_label = QLabel("目标：未检测（无数据源）")
        self.coords_label = QLabel("图像坐标：—")
        self.coords_label.setWordWrap(True)
        root.addWidget(self.source_label)
        root.addWidget(self.target_label)
        root.addWidget(self.coords_label)
        self.result: VisionResult | None = None

    def set_frame(self, image, result: VisionResult) -> None:
        self.canvas.set_frame(image, result)
        self.result = result
        self.source_label.setText(f"视觉源：{result.source}{'（模拟）' if result.simulated else ''}")
        if result.black_blocks is not None:
            block = result.black_blocks[0] if result.black_blocks else None
            self.target_label.setText(
                f"黑色物块：检测到 1 个（最大合格轮廓） · {'已对齐' if result.black_block_aligned else '普通'}"
                if block else "未检测到黑色物块")
            self.coords_label.setText(
                f"视觉重心（原图像素质心）：({round(block.center_x)}, {round(block.center_y)}) px · "
                f"轮廓面积：{block.area_px:.0f} px²" if block else "图像坐标：—")
            return
        if result.detection is None:
            self.target_label.setText("目标：未检测到" if result.simulated else "目标：识别算法未接入")
            self.coords_label.setText("图像坐标：—")
        else:
            d = result.detection
            self.target_label.setText(f"目标：已检测 · 编号 {d.target_id} · 类别 {d.category} · 置信度 {d.confidence:.1%}")
            self.coords_label.setText(f"图像坐标：u={d.u:.1f} px, v={d.v:.1f} px")

    def clear_source(self) -> None:
        self.canvas.clear()
        self.result = None
        self.source_label.setText("视觉源：未接入")
        self.target_label.setText("目标：未检测（无数据源）")
        self.coords_label.setText("图像坐标：—")


class VisionDemoPage(VisionPage):
    parameters_changed = Signal()

    def __init__(self, parent=None):
        super().__init__("视觉演示模式", parent)
        root = self.layout()
        details = QWidget()
        details_layout = QVBoxLayout(details)
        details_layout.setContentsMargins(4, 4, 4, 4)
        for widget in (self.source_label, self.target_label, self.coords_label):
            root.removeWidget(widget)
        controls = QGridLayout()
        self.gray_threshold = QSpinBox()
        self.gray_threshold.setRange(0, 255)
        self.gray_threshold.setValue(70)
        self.min_area = QSpinBox()
        self.min_area.setRange(1, 100000)
        self.min_area.setValue(250)
        self.max_area_percent = QDoubleSpinBox()
        self.max_area_percent.setRange(0.1, 100.0)
        self.max_area_percent.setDecimals(1)
        self.max_area_percent.setValue(25.0)
        self.max_area_percent.setSuffix(" %")
        self.alignment_tolerance = QSpinBox()
        self.alignment_tolerance.setRange(0, 20)
        self.alignment_tolerance.setValue(5)
        self.alignment_tolerance.setSuffix(" px")
        for control in (self.gray_threshold, self.min_area, self.max_area_percent,
                        self.alignment_tolerance):
            control.setKeyboardTracking(False)
            control.valueChanged.connect(lambda _value: self.parameters_changed.emit())
        for column, (label, control) in enumerate((("黑色阈值 (0–255)", self.gray_threshold),
                                                   ("最小面积 (px²)", self.min_area),
                                                   ("最大画面占比", self.max_area_percent))):
            controls.addWidget(QLabel(label), 0, column)
            controls.addWidget(control, 1, column)
        controls.addWidget(QLabel("对齐容差（原图像素）"), 2, 0)
        controls.addWidget(self.alignment_tolerance, 3, 0)
        details_layout.addLayout(controls)
        self.tracking_label = QLabel("视觉跟踪：SEARCH · 等待视频帧")
        for widget in (self.source_label, self.target_label, self.coords_label):
            details_layout.addWidget(widget)
        details_layout.addWidget(QLabel("仅展示图传与识别叠加；此页面不提交机械臂运动命令。"))
        self.details_scroll = QScrollArea()
        self.details_scroll.setWidgetResizable(True)
        self.details_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.details_scroll.setMinimumHeight(105)
        self.details_scroll.setMaximumHeight(135)
        self.details_scroll.setWidget(details)
        root.addWidget(self.details_scroll)
        root.addWidget(self.tracking_label)

    def set_frame(self, image, result: VisionResult) -> None:
        super().set_frame(image, result)
        if result.tracking_phase == "SEARCH":
            self.tracking_label.setText("视觉跟踪：SEARCH · 中央 50% 区域搜索 · 连续漏检 0 帧")
        elif result.tracking_phase == "TRACK" and result.black_blocks:
            self.tracking_label.setText("视觉跟踪：TRACK · 已锁定 · 连续漏检 0 帧")
        elif result.tracking_phase == "TRACK":
            self.tracking_label.setText(
                f"视觉跟踪：TRACK · 暂时丢失 · 连续漏检 {result.tracking_misses}/{result.tracking_miss_limit} 帧"
                + (" · 下一帧恢复 SEARCH" if result.tracking_misses >= result.tracking_miss_limit else ""))
        else:
            self.tracking_label.setText("视觉跟踪：等待识别")

    def clear_source(self) -> None:
        super().clear_source()
        self.tracking_label.setText("视觉跟踪：SEARCH · 等待视频帧")

    def detector_config(self) -> BlackBlockConfig:
        return BlackBlockConfig(self.gray_threshold.value(), self.min_area.value(),
                                self.max_area_percent.value())

    def show_pending_frame(self, image, source: str, frame_id: int) -> None:
        result = VisionResult(image.width(), image.height(), None, source, False,
                              frame_id=frame_id, roi_overlay=True)
        self.set_frame(image, result)
        self.target_label.setText("参数已更新，正在识别最新画面")
        self.coords_label.setText("图像坐标：—")


class VisionGrabPage(VisionPage):
    REGION_ANGLES = (30, 50, 70, 90, 110)  # bottom to top, travel from configured normal position
    STABLE_FRAMES = 8
    CENTER_TOLERANCE_PX = 12
    SETTLE_TOLERANCE_MDEG = 5_000
    PICK_RETURN_TOLERANCE_MDEG = 10_000  # Manual PICK only; never claims precise arrival.
    PICK_STILL_DELTA_MDEG = 1_000
    PICK_STILL_MIN_MS = 1_000
    SETTLE_DELTA_MDEG = 300
    SETTLE_SAMPLES = 3
    SETTLE_MIN_MS = 350
    SETTLE_FEEDBACK_TIMEOUT_SECONDS = 2.0
    RETURN_SETTLE_SECONDS = 0.45  # MCU resets its 300 ms HX8 stability gate on STOP.
    RETURN_DEADLINE_SECONDS = 5.0
    RETURN_MAX_ATTEMPTS = 4  # Retry only after an explicit NOT_READY rejection.
    PICK_REPLY_TIMEOUT_SECONDS = 12.0  # MCU motion deadline is 10 seconds.
    def __init__(self, controller: ExperimentController, parent=None):
        super().__init__("视觉抓取模式", parent)
        self.controller = controller
        self.running = False
        self.stage = 0
        self.request_generated = False
        self.command_sent = False
        self.actual_execution = "未知／等待反馈"
        self.grab_result = "未知／暂不可确认"
        self._start_submission_count = 0
        self._candidate_region = None
        self._stable_frames = 0
        self._active_req = None
        self._pick_started_at: float | None = None
        self._phase = "idle"
        self._settle_target = None
        self._settle_last = None
        self._settle_time = None
        self._settle_count = 0
        self._settle_anchor = None
        self._settle_since_ms = None
        self._pick_close_applied = False
        self._manual_pick = False
        self._settle_stop_query_id = None
        self._settle_wait_since = None
        self._return_wait_since = None
        self._return_ready_after = None
        self._return_state_after = None
        self._return_close_req = None
        self._return_attempts = 0
        self._return_target_mdeg: int | None = None
        self._action_speed_percent = 30.0
        self._return_speed_percent = 60.0
        self._manual_timed_grab = False
        self._manual_travel_seconds: float | None = None
        self._manual_close_deadline: float | None = None
        self._manual_done_at: float | None = None
        self._manual_pending_pick: tuple[float, float] | None = None
        self._return_confirm_after: float | None = None
        self._return_confirm_device_ms: int | None = None
        controls = QGridLayout()
        self.speed_slider, self.speed_spin = ManualPage._add_control(
            self, controls, 0, "出臂速度", 0.1, 100.0, 30.0, "%")
        self.layout().addLayout(controls)
        row = QHBoxLayout()
        self.start_button = QPushButton("启动视觉抓取")
        self.start_button.setObjectName("primaryButton")
        self.stop_button = QPushButton("停止新决策")
        self.stop_button.setEnabled(False)
        row.addWidget(self.start_button)
        row.addWidget(self.stop_button)
        row.addStretch()
        self.layout().addLayout(row)
        self.flow_label = QLabel("流程：待启动")
        self.request_label = QLabel("抓取请求：无")
        self.command_label = QLabel("控制命令：未发送")
        self.actual_label = QLabel("机械臂实际执行：未知／等待反馈")
        self.result_label = QLabel("抓取结果：未知／暂不可确认")
        details = QWidget()
        details_layout = QVBoxLayout(details)
        details_layout.setContentsMargins(4, 2, 4, 2)
        for widget in (self.source_label, self.target_label, self.coords_label):
            self.layout().removeWidget(widget)
            details_layout.addWidget(widget)
        for widget in (self.flow_label, self.request_label, self.command_label, self.actual_label, self.result_label):
            details_layout.addWidget(widget)
        self.details_scroll = QScrollArea()
        self.details_scroll.setWidgetResizable(True)
        self.details_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.details_scroll.setMinimumHeight(85)
        self.details_scroll.setMaximumHeight(105)
        self.details_scroll.setWidget(details)
        self.layout().addWidget(self.details_scroll)
        self.timer = QTimer(self)
        self.timer.setInterval(700)
        self.timer.timeout.connect(self._advance_simulation)
        self.feedback_timer = QTimer(self)
        self.feedback_timer.setInterval(250)
        self.feedback_timer.timeout.connect(self._poll_motion)
        self.start_button.clicked.connect(self.start_flow)
        self.stop_button.clicked.connect(self.stop_flow)
        controller.state_changed.connect(self._sync_execution)
        controller.command_result.connect(self._on_command_result)
        controller.motion_event.connect(self._on_motion_event)

    def start_flow(self) -> None:
        if self.running:
            return
        self._action_speed_percent = self.speed_spin.value()
        self._return_speed_percent = 60.0
        if not self.controller.simulated and self.controller.mode != "MANUAL":
            if not self.controller.request_mode("MANUAL"):
                self.flow_label.setText(f"流程：{self.controller.command_status}")
                return
        elif not self.controller.simulated and not self.controller._guard(manual=True, motion=True):
            self.flow_label.setText(f"流程：{self.controller.command_status}")
            return
        self.running = True
        self._phase = "mode" if not self.controller.simulated and self.controller.mode != "MANUAL" else "watch"
        self._candidate_region = None
        self._stable_frames = 0
        self._active_req = None
        self._pick_started_at = None
        self._settle_last = None
        self._settle_time = None
        self._settle_count = 0
        self._settle_anchor = None
        self._settle_since_ms = None
        self._settle_stop_query_id = None
        self._settle_wait_since = None
        self._return_wait_since = None
        self._return_ready_after = None
        self._return_state_after = None
        self._return_close_req = None
        self._return_attempts = 0
        self._return_target_mdeg = None
        self.stage = 0
        self._manual_timed_grab = False
        self._manual_travel_seconds = None
        self._manual_close_deadline = None
        self._manual_done_at = None
        self._manual_pending_pick = None
        self._pick_close_applied = False
        self._manual_pick = False
        self._return_confirm_after = None
        self._return_confirm_device_ms = None
        self.request_generated = False
        self.command_sent = False
        self.actual_execution = "未知／等待反馈"
        self.grab_result = "未知／暂不可确认"
        self._start_submission_count = self.controller.submitted_requests
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.flow_label.setText("流程：等待手动模式回执" if self._phase == "mode" else "流程：等待中心目标稳定")
        self.request_label.setText("抓取请求：无")
        self.command_label.setText("控制命令：未发送")
        self.actual_label.setText("机械臂实际执行：未知／等待反馈")
        self.result_label.setText("抓取结果：未知／暂不可确认")
        if self.controller.simulated:
            self.timer.start()
            self.controller.event.emit("模拟", "视觉抓取", "模拟流程已启动")
        else:
            self.controller.event.emit("视觉", "视觉抓取", "已开启；等待中心目标连续稳定")

    def start_manual_grab(self, speed: float, angle: float) -> bool:
        """Run the existing grab state machine with a manual parameter snapshot."""
        if self.running:
            return False
        if not self.controller.simulated and not self.controller._guard(manual=True, motion=True):
            return False
        self.start_flow()
        if not self.running:
            return False
        self._action_speed_percent = speed
        self._return_speed_percent = 30.0
        self._manual_timed_grab = False
        self._manual_pick = True
        if self.controller.simulated:
            self.request_label.setText(f"模拟手动抓取：出臂 {speed:g}%／{angle:g}°，回到起点 30%")
            return True
        # The previous PICK leaves the jaw closed. SERVO is accepted only
        # while HX8 is idle, so apply OPEN first; its physical movement can
        # overlap the following extension, without claiming position feedback.
        state = self.controller.latest_state
        if state is None or state.grip_cmd_mdeg != GRIP_TEST_ANGLE_MDEG["OPEN"]:
            if not self.controller.request_grip("OPEN"):
                self.stop_flow(send_stop=False)
                return False
            self._manual_pending_pick = (speed, angle)
            self._phase = "manual_open"
            self._active_req = self.controller._next_req - 1
            self.flow_label.setText("流程：张开夹爪PWM已提交；收到设备回执后立即出臂")
            self.request_label.setText(f"抓取请求：出臂 {speed:g}%／{angle:g}°；等待开爪回执")
            return True
        if self._submit_grab_motion(speed, angle, f"手动抓取：相对行程 {angle:g}°，速度 {speed:g}%"):
            return True
        self.stop_flow(send_stop=False)
        return False

    def _submit_grab_motion(self, speed: float, angle: float, label: str) -> bool:
        if not self.controller.request_extend(speed, angle, host_settle=False, linked_pick=True):
            self.flow_label.setText(f"流程：出臂请求被拦截：{self.controller.command_status}")
            return False
        self._phase = "pick_extend"
        self._active_req = self.controller._next_req - 1
        self._pick_started_at = time.monotonic()
        self._return_target_mdeg = self.controller.last_extend_start_mdeg
        self._settle_target = WORKING_ZERO_MDEG + self.controller.cal.extension_sign * round(angle * 1000)
        self._settle_last = None
        self._settle_time = None
        self._settle_count = 0
        self._settle_anchor = None
        self._settle_since_ms = None
        self._pick_close_applied = False
        if self._manual_timed_grab:
            state = self.controller.latest_state
            if state is None or state.pos_mdeg is None or self.controller.info is None:
                return False
            speed_dps = max(1, round(self.controller.info.max_speed_dps * speed / 100))
            self._manual_travel_seconds = abs(self._settle_target - state.pos_mdeg) / (speed_dps * 1000)
            # The actual START event resets the clock after the queued write.
            self._manual_close_deadline = time.monotonic() + self._manual_travel_seconds
            self.controller.event.emit(
                "控制", "手动抓取闭爪时序",
                f"按本次行程和 {speed_dps}°/s 估算 {self._manual_travel_seconds:.3f}s；"
                "到时仍须确认 HX8 已停止、接近目标且无故障")
        self.feedback_timer.start()
        self.request_generated = True
        self.request_label.setText(f"抓取请求：{label}")
        self.flow_label.setText("流程：出臂中；MCU按实际位置在目标前4°应用闭爪PWM")
        return True

    def _on_motion_event(self, frame) -> None:
        if (self.running and self._phase == "pick_extend" and
                frame.req_id == self._active_req and frame.stage == "CLOSE_APPLIED"):
            self._pick_close_applied = True
        if (self.running and self._manual_timed_grab and self._phase == "extend" and
                frame.req_id == self._active_req and frame.stage == "START" and
                self._manual_travel_seconds is not None):
            self._manual_close_deadline = time.monotonic() + self._manual_travel_seconds

    def _close_or_wait_for_manual_time(self) -> None:
        if self._manual_timed_grab:
            self._phase = "timed_wait"
            self._active_req = None
            self._manual_done_at = time.monotonic()
            self.feedback_timer.start()
            self.controller.query("STATE")
            self.flow_label.setText("流程：HX8已停止，等待估算闭爪时刻和新鲜位置反馈")
            self._sync_execution()
        elif self.controller.request_grip("CLOSE"):
            self._phase = "close"
            self._active_req = self.controller._next_req - 1
            self.flow_label.setText("流程：HX8到位，等待闭爪PWM回执")
        else:
            self.stop_flow(send_stop=False)
            self.flow_label.setText(f"流程：闭爪被拦截：{self.controller.command_status}")

    def observe(self, result: VisionResult) -> None:
        if not self.running or self.controller.simulated or self._phase != "watch":
            return
        block = result.black_blocks[0] if result.black_blocks else None
        region = None
        if (block is not None and abs(block.center_x - result.width / 2) <= self.CENTER_TOLERANCE_PX
                and 0 <= block.center_y < result.height):
            region = 5 - min(4, int(block.center_y * 5 / result.height))
        self._stable_frames = self._stable_frames + 1 if region is not None and region == self._candidate_region else (1 if region else 0)
        self._candidate_region = region
        self.flow_label.setText(f"流程：区域{region}稳定 {self._stable_frames}/{self.STABLE_FRAMES} 帧" if region else "流程：等待中心目标")
        if self._stable_frames < self.STABLE_FRAMES:
            return
        angle = self.REGION_ANGLES[region - 1]
        if round(angle * 1000) > MANUAL_EXTENSION_LIMIT_MDEG:
            self.flow_label.setText(
                f"流程：区域{region}需出臂{angle}°，超过实测{ANGLE_MAX:g}°上限；未发送动作，请重新标定该区域")
            self._stable_frames = 0
            return
        if not self._submit_grab_motion(self._action_speed_percent, angle,
                                        f"区域{region}，相对行程{angle}°，速度{self._action_speed_percent:g}%"):
            self.flow_label.setText(f"流程：出臂请求被拦截：{self.controller.command_status}")
            self._candidate_region = None
            self._stable_frames = 0
            return

    def _on_command_result(self, req: int, verb: str, result: str) -> None:
        if not self.running or req != self._active_req:
            return
        if self._phase == "manual_open" and verb == "SERVO" and result == "APPLIED":
            snapshot = self._manual_pending_pick
            self._manual_pending_pick = None
            self._active_req = None
            self._phase = "manual_starting"
            if snapshot is not None and self._submit_grab_motion(
                    snapshot[0], snapshot[1],
                    f"手动抓取：相对行程 {snapshot[1]:g}°，速度 {snapshot[0]:g}%"):
                return
            reason = self.controller.command_status
            self.stop_flow(send_stop=False)
            self.flow_label.setText(f"流程：开爪后出臂未启动：{reason}")
            return
        if self._phase == "pick_extend" and verb == "PICK" and result == "DONE":
            self.feedback_timer.stop()
            now = time.monotonic()
            self._phase = "return_wait"
            self._active_req = None
            self._return_wait_since = now
            self._return_ready_after = now + self.RETURN_SETTLE_SECONDS
            self._return_state_after = now
            self._return_close_req = req
            self._return_attempts = 0
            self.feedback_timer.start()
            self.controller.query("STATE")
            self.flow_label.setText("流程：MCU已应用闭爪PWM并到位，等待新鲜状态后回位")
            return
        if self._phase == "extend" and verb == "MOVE" and result == "DONE":
            self.feedback_timer.stop()
            self._close_or_wait_for_manual_time()
            return
        elif self._phase == "settle_stop" and verb == "STOP" and result == "APPLIED":
            self._phase = "settle_wait"
            self._active_req = None
            self._settle_wait_since = time.monotonic()
            self.flow_label.setText("流程：已保持位置，等待无故障的空闲反馈")
            self.controller.query("STATE")
            self._settle_stop_query_id = self.controller._query_ids.get("STATE")
            self.feedback_timer.start()
            return
        elif self._phase == "pick_settle_stop" and verb == "STOP" and result == "APPLIED":
            self._phase = "pick_settle_wait"
            self._active_req = None
            self._settle_wait_since = time.monotonic()
            self.controller.query("STATE")
            self._settle_stop_query_id = self.controller._query_ids.get("STATE")
            self.feedback_timer.start()
            self.flow_label.setText("流程：闭爪后已停止并保持，等待无故障的空闲反馈后回位")
            return
        elif self._phase == "close" and verb == "SERVO" and result == "APPLIED":
            now = time.monotonic()
            self._phase = "return_wait"
            self._active_req = None
            self._return_wait_since = now
            self._return_ready_after = now + self.RETURN_SETTLE_SECONDS
            self._return_state_after = now
            self._return_close_req = req
            self._return_attempts = 0
            self.feedback_timer.start()
            self.controller.query("STATE")
            self.flow_label.setText("流程：闭爪PWM已应用，等待HX8稳定后回正")
            return
        elif self._phase == "return" and verb == "MOVE" and result == "NOT_READY":
            # STOP clears the MCU's 300 ms stability gate. An explicit ERR
            # proves this MOVE did not start, so a new request is safe later.
            now = time.monotonic()
            self._phase = "return_wait"
            self._active_req = None
            self._return_ready_after = now + self.RETURN_SETTLE_SECONDS
            self._return_state_after = now
            self.feedback_timer.start()
            self.controller.query("STATE")
            self.flow_label.setText("流程：HX8尚未稳定，等待新反馈后重试回正")
            return
        elif self._phase == "return" and verb == "MOVE" and result in ("DONE", "SETTLED"):
            # DONE belongs to the return MOVE, but the cached STATE may still
            # describe the extension endpoint. Keep controls locked until a
            # fresh idle position confirms which preset the next PICK starts at.
            self._phase = "return_confirm"
            self._active_req = None
            self._return_confirm_after = time.monotonic()
            self._return_confirm_device_ms = (self.controller.latest_state.t_ms
                                              if self.controller.latest_state is not None else None)
            self.controller.query("STATE")
            self.feedback_timer.start()
            self.flow_label.setText("流程：回位动作已结束；等待新鲜位置反馈后允许再次抓取")
            return
        self.stop_flow(send_stop=False)
        self._phase = "aborted"
        self.flow_label.setText(f"流程：中止，{result}")
        self.controller.state_changed.emit()

    def _poll_motion(self) -> None:
        if not self.running or self._phase not in ("extend", "pick_extend", "pick_settle_stop", "settle_wait", "pick_settle_wait", "timed_wait", "return_wait", "return_confirm") or self.controller.simulated:
            self.feedback_timer.stop()
            return
        self.controller.query("STATE")
        if self._phase in ("pick_extend", "pick_settle_stop", "settle_wait", "pick_settle_wait", "timed_wait", "return_wait", "return_confirm"):
            self._sync_execution()  # Also enforce the deadline if no STATE arrives.

    def _try_close_after_hold(self) -> None:
        state = self.controller.latest_state
        if (state is None or state.motion != "IDLE" or state.fault_code != "NONE" or
                not state.hx8_online or state.pos_mdeg is None or self._settle_target is None or
                abs(state.pos_mdeg - self._settle_target) > self.SETTLE_TOLERANCE_MDEG or
                self.controller._pending or not self.controller.link_online or
                self._settle_stop_query_id is None or state.reply_to < self._settle_stop_query_id):
            return
        self.feedback_timer.stop()
        self._close_or_wait_for_manual_time()

    def _sync_execution(self) -> None:
        if not self.running or self.controller.simulated:
            return
        if self._phase == "return_confirm":
            state = self.controller.latest_state
            received = self.controller._last_state_received_at
            now = time.monotonic()
            if not self.controller.link_online or (state is not None and state.fault_code != "NONE"):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：回位后的设备反馈异常，未开放再次抓取")
                return
            if (self._return_confirm_after is not None and received is not None and
                    received >= self._return_confirm_after and now - received < 1.0 and
                    state is not None and state.motion == "IDLE" and state.hx8_online and
                    (self._return_confirm_device_ms is None or state.t_ms > self._return_confirm_device_ms) and
                    state.pos_mdeg is not None and self._return_target_mdeg is not None and
                    abs(state.pos_mdeg - self._return_target_mdeg) <= self.SETTLE_TOLERANCE_MDEG):
                self.feedback_timer.stop()
                self._phase = "finished"
                self.running = False
                self.start_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                position = "折叠位置" if self._return_target_mdeg == FOLD_TARGET_MDEG else "正常位置"
                self.flow_label.setText(f"流程：已返回{position}；闭爪PWM保持，可按当前参数再次抓取")
                self.result_label.setText("抓取结果：未知／无夹持反馈")
                self.controller.state_changed.emit()
            elif self._return_confirm_after is not None and now - self._return_confirm_after > 2.0:
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：未取得回位后的新鲜位置反馈；请查看设备状态")
            return
        if self._phase == "pick_extend":
            state = self.controller.latest_state
            if state is not None and state.fault_code != "NONE":
                self.stop_flow(send_stop=False)
                self.flow_label.setText(f"流程：MCU故障 {state.fault_code}，抓取已中止；不会发送回程")
                return
            if not self.controller.link_online:
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：设备离线，抓取状态未知；不会继续自动动作")
                return
            received = self.controller._last_state_received_at
            if (self._manual_pick and self._pick_close_applied and
                    state is not None and state.pos_mdeg is not None and
                    self._settle_target is not None and received is not None and
                    time.monotonic() - received < 0.75 and state.t_ms != self._settle_time and
                    state.motion == "MOVING" and state.hx8_online and
                    state.grip_cmd_mdeg == GRIP_TEST_ANGLE_MDEG["CLOSE"] and
                    self._active_req is not None and state.last_req_id >= self._active_req):
                near = abs(state.pos_mdeg - self._settle_target) <= self.PICK_RETURN_TOLERANCE_MDEG
                if (near and self._settle_anchor is not None and
                        abs(state.pos_mdeg - self._settle_anchor) <= self.PICK_STILL_DELTA_MDEG):
                    self._settle_count += 1
                else:
                    self._settle_count = 1 if near else 0
                    self._settle_anchor = state.pos_mdeg if near else None
                    self._settle_since_ms = state.t_ms if near else None
                self._settle_time = state.t_ms
                if (self._settle_count >= self.SETTLE_SAMPLES and self._settle_since_ms is not None and
                        state.t_ms - self._settle_since_ms >= self.PICK_STILL_MIN_MS):
                    pick_req = self._active_req
                    self._phase = "pick_settle_stop"
                    self._return_close_req = pick_req
                    self._settle_wait_since = time.monotonic()
                    self.feedback_timer.stop()
                    if self.controller.request_stop():
                        self._active_req = self.controller._next_req - 1
                        self.flow_label.setText(
                            f"流程：闭爪PWM已应用，HX8距目标{abs(state.pos_mdeg - self._settle_target) / 1000:.1f}°且位置稳定；停止后回位")
                    else:
                        self.stop_flow(send_stop=False)
                        self.flow_label.setText(f"流程：无法停止并保持：{self.controller.command_status}")
                    return
            if (self._pick_started_at is not None and
                    time.monotonic() - self._pick_started_at > self.PICK_REPLY_TIMEOUT_SECONDS):
                self.stop_flow(send_stop=True)
                self.flow_label.setText("流程：抓取终态超时，已请求停止并保持；请查看设备反馈")
            return
        if self._phase == "pick_settle_stop":
            if (not self.controller.link_online or
                    (self._settle_wait_since is not None and
                     time.monotonic() - self._settle_wait_since > self.SETTLE_FEEDBACK_TIMEOUT_SECONDS)):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：停止请求未确认，未自动回位；请查看设备状态")
            return
        if self._phase == "timed_wait":
            state = self.controller.latest_state
            received = self.controller._last_state_received_at
            now = time.monotonic()
            if (state is not None and state.fault_code != "NONE") or not self.controller.link_online:
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：闭爪前设备故障或离线，已取消闭爪与回位")
                return
            if (self._manual_close_deadline is not None and self._manual_done_at is not None and
                    now - max(self._manual_close_deadline, self._manual_done_at) >
                    self.SETTLE_FEEDBACK_TIMEOUT_SECONDS):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：未取得新鲜的近目标空闲反馈，未闭爪")
                return
            if (self._manual_close_deadline is not None and now >= self._manual_close_deadline and
                    self._manual_done_at is not None and received is not None and
                    received > self._manual_done_at and now - received < 1.0 and
                    state is not None and state.motion == "IDLE" and state.hx8_online and
                    state.pos_mdeg is not None and self._settle_target is not None and
                    abs(state.pos_mdeg - self._settle_target) <= self.SETTLE_TOLERANCE_MDEG and
                    not self.controller._pending):
                self.feedback_timer.stop()
                if self.controller.request_grip("CLOSE"):
                    self._phase = "close"
                    self._active_req = self.controller._next_req - 1
                    self.flow_label.setText("流程：估算时刻已到、HX8近目标空闲，等待闭爪PWM回执")
                else:
                    self.stop_flow(send_stop=False)
                    self.flow_label.setText(f"流程：闭爪被拦截：{self.controller.command_status}")
            return
        if self._phase == "settle_wait":
            if (self._settle_wait_since is not None and
                    time.monotonic() - self._settle_wait_since > self.SETTLE_FEEDBACK_TIMEOUT_SECONDS):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：STOP后未取得新鲜空闲反馈，未闭爪")
                return
            self._try_close_after_hold()
            return
        if self._phase == "pick_settle_wait":
            state = self.controller.latest_state
            received = self.controller._last_state_received_at
            now = time.monotonic()
            if (not self.controller.link_online or state is None or state.fault_code != "NONE" or
                    self._settle_wait_since is None or
                    now - self._settle_wait_since > self.SETTLE_FEEDBACK_TIMEOUT_SECONDS):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：闭爪后停止反馈异常，未自动回位；请查看设备状态")
                return
            if (received is not None and received >= self._settle_wait_since and now - received < 1.0 and
                    state.motion == "IDLE" and state.hx8_online and state.pos_mdeg is not None and
                    self._settle_target is not None and
                    abs(state.pos_mdeg - self._settle_target) <= self.PICK_RETURN_TOLERANCE_MDEG and
                    state.grip_cmd_mdeg == GRIP_TEST_ANGLE_MDEG["CLOSE"] and
                    self._settle_stop_query_id is not None and
                    state.reply_to >= self._settle_stop_query_id and not self.controller._pending):
                self._phase = "return_wait"
                self._return_wait_since = now
                self._return_ready_after = now + self.RETURN_SETTLE_SECONDS
                self._return_state_after = now
                self._return_attempts = 0
                self.controller.query("STATE")
                self.flow_label.setText("流程：闭爪后近目标稳定保持，等待HX8就绪后回位")
            return
        if self._phase == "extend":
            state = self.controller.latest_state
            received = self.controller._last_state_received_at
            if state is not None and state.fault_code != "NONE":
                self.stop_flow(send_stop=False)
                self.flow_label.setText(f"流程：设备故障 {state.fault_code}，未闭爪")
                return
            if (state is not None and state.pos_mdeg is not None and received is not None and
                    self._settle_target is not None and state.t_ms != self._settle_time and
                    time.monotonic() - received < 0.75 and state.motion == "MOVING" and
                    state.hx8_online):
                near = abs(state.pos_mdeg - self._settle_target) <= self.SETTLE_TOLERANCE_MDEG
                if (near and self._settle_anchor is not None and
                        abs(state.pos_mdeg - self._settle_anchor) <= self.SETTLE_DELTA_MDEG):
                    self._settle_count += 1
                else:
                    self._settle_count = 1 if near else 0
                    self._settle_anchor = state.pos_mdeg if near else None
                    self._settle_since_ms = state.t_ms if near else None
                self._settle_last = state.pos_mdeg
                self._settle_time = state.t_ms
                if (self._settle_count >= self.SETTLE_SAMPLES and self._settle_since_ms is not None and
                        state.t_ms - self._settle_since_ms >= self.SETTLE_MIN_MS):
                    self.feedback_timer.stop()
                    if self.controller.request_stop():
                        self._phase = "settle_stop"
                        self._active_req = self.controller._next_req - 1
                        self.flow_label.setText("流程：接近目标且位置稳定，正在停止并保持")
                    else:
                        self.stop_flow(send_stop=False)
                        self.flow_label.setText(f"流程：保持失败：{self.controller.command_status}")
                    return
        if self._phase == "return_wait":
            state = self.controller.latest_state
            now = time.monotonic()
            if (self._return_wait_since is None or
                    now - self._return_wait_since > self.RETURN_DEADLINE_SECONDS or
                    self._return_attempts >= self.RETURN_MAX_ATTEMPTS):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：回位等待超时；闭爪PWM保持，请检查HX8并手动选择正常／折叠位置")
                return
            if state is not None and state.fault_code != "NONE":
                self.feedback_timer.stop()
                self.stop_flow(send_stop=False)
                self.flow_label.setText(f"流程：闭爪后设备故障 {state.fault_code}，回程未发送")
                self.controller.event.emit("视觉", "回程中止", self.flow_label.text())
                return
            if (state is not None and state.motion == "IDLE" and state.fault_code == "NONE"
                    and not self.controller._pending and self.controller.link_online and
                    self.controller._last_state_received_at is not None and
                    self._return_ready_after is not None and now >= self._return_ready_after and
                    self._return_state_after is not None and
                    self.controller._last_state_received_at > self._return_state_after and
                    now - self.controller._last_state_received_at < 1.0 and
                    state.grip_cmd_mdeg == GRIP_TEST_ANGLE_MDEG["CLOSE"]):
                # Set the phase before _submit emits state_changed, preventing
                # reentrant callbacks from submitting a second MOVE.
                self._phase = "return"
                if self.controller.request_visual_return(self._return_close_req,
                                                         target_mdeg=self._return_target_mdeg,
                                                         speed_percent=self._return_speed_percent):
                    self._return_attempts += 1
                    self._phase = "return"
                    self._active_req = self.controller._next_req - 1
                    position = "折叠位置" if self._return_target_mdeg == FOLD_TARGET_MDEG else "正常位置"
                    self.flow_label.setText(f"流程：闭爪PWM已应用，HX8返回{position}中")
                    self.command_label.setText(f"控制命令：已提交回程 MOVE +{self._return_target_mdeg / 1000:g}°")
                else:
                    self.stop_flow(send_stop=False)
                    self.flow_label.setText(f"流程：返回被拦截：{self.controller.command_status}")
            return
        if self._phase == "mode" and self.controller.mode == "MANUAL" and not self.controller._pending:
            self._phase = "watch"
            self.flow_label.setText("流程：等待中心目标稳定")
        self.actual_label.setText(f"机械臂实际执行：{self.controller.execution_status}")
        if (self.controller.last_request is not None and
                self.controller.last_request.action == "grab" and
                self.controller.submitted_requests > self._start_submission_count):
            self.request_label.setText("抓取请求：已产生一次")
            self.command_label.setText(f"控制命令：{self.controller.command_status}")
        if self._phase in ("watch", "extend", "pick_extend", "close", "return"):
            return
        if self.controller.ready_for_pick:
            self.flow_label.setText("流程：准备完成，等待视觉算法提供候选目标和单次触发")
        elif "闭爪命令" in self.controller.execution_status:
            self.flow_label.setText("流程：闭爪命令已应用，HX8继续出臂")
        elif "中止" in self.controller.execution_status:
            self.flow_label.setText("流程：已中止")

    def stop_flow(self, *, send_stop: bool = True) -> None:
        if not self.running:
            return
        self.timer.stop()
        self.feedback_timer.stop()
        self.running = False
        self._phase = "idle"
        self._active_req = None
        self._pick_started_at = None
        self._candidate_region = None
        self._stable_frames = 0
        self._return_wait_since = None
        self._return_ready_after = None
        self._return_state_after = None
        self._return_close_req = None
        self._return_attempts = 0
        self._return_target_mdeg = None
        self._manual_timed_grab = False
        self._manual_travel_seconds = None
        self._manual_close_deadline = None
        self._manual_done_at = None
        self._manual_pending_pick = None
        self._pick_close_applied = False
        self._manual_pick = False
        self._return_confirm_after = None
        self._return_confirm_device_ms = None
        self.controller.set_visual_target_enabled(False)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.flow_label.setText("流程：已停止新决策")
        if send_stop and not self.controller.simulated:
            self.controller.request_stop()
        self.controller.event.emit("模拟" if self.controller.simulated else "视觉", "停止视觉抓取", "后续自动决策已停止；设备执行状态以回执为准")
        self.controller.state_changed.emit()

    def _advance_simulation(self) -> None:
        if not self.running or not self.controller.simulated:
            return
        self.stage += 1
        if self.stage == 1:
            self.flow_label.setText("流程：已识别模拟目标")
        elif self.stage == 2:
            self.flow_label.setText("流程：模拟目标满足示意条件")
        elif self.stage == 3:
            self.request_generated = True
            self.request_label.setText("抓取请求：已生成（模拟）")
            self.command_sent = self.controller.request_grab()
            self.command_label.setText("控制命令：模拟提交，未写入串口" if self.command_sent else "控制命令：未提交")
            self.flow_label.setText("流程：模拟抓取请求已提交")
        else:
            self.actual_execution = "模拟执行结束（非设备反馈）"
            self.grab_result = "模拟成功（非真实抓取结果）"
            self.actual_label.setText(f"机械臂实际执行：{self.actual_execution}")
            self.result_label.setText(f"抓取结果：{self.grab_result}")
            self.flow_label.setText("流程：模拟演示结束")
            self.timer.stop()
            self.running = False
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.controller.event.emit("模拟", "视觉抓取", self.grab_result)

    def simulation_disabled(self) -> None:
        self.stop_flow()
        self.actual_execution = "未知／等待反馈"
        self.grab_result = "未知／暂不可确认"
        self.actual_label.setText("机械臂实际执行：未知／等待反馈")
        self.result_label.setText("抓取结果：未知／暂不可确认")
        self.command_label.setText("控制命令：未发送")
        self.request_label.setText("抓取请求：无")
