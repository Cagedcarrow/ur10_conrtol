#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import queue
import signal
import subprocess
import sys
import threading
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


PREFLIGHT_REQUIRED_MARKERS = {
    "ROBOT_PING_OK",
    "JOINT_STATES_OK",
    "SCALED_CONTROLLER_ACTIVE",
    "EXEC_ACTION_ONLINE",
    "SPEED_SCALING_NONZERO",
    "RTDE_OK",
}

PREFLIGHT_FAILURE_MARKERS = {
    "ROBOT_PING_FAIL",
    "JOINT_STATES_MISSING",
    "SCALED_CONTROLLER_NOT_ACTIVE",
    "EXEC_ACTION_MISSING",
    "SPEED_SCALING_ZERO_OR_MISSING",
    "SPEED_SCALING_TOPIC_MISSING",
    "RTDE_OVERFLOW",
}

PREFLIGHT_IDLE_STYLE = (
    "QPushButton { background-color: #808080; color: white; font-weight: 600; }"
)
PREFLIGHT_PASS_STYLE = (
    "QPushButton { background-color: #2e7d32; color: white; font-weight: 600; }"
)
PREFLIGHT_FAIL_STYLE = (
    "QPushButton { background-color: #c62828; color: white; font-weight: 600; }"
)

DIAGNOSTIC_BUTTONS = (
    ("network", "网络/路由"),
    ("primary", "Primary配置"),
    ("ports", "机器人端口"),
    ("realtime", "30003关节流"),
    ("residuals", "残余/冲突"),
    ("all", "完整通信"),
)


def _child_env() -> dict[str, str]:
    env = os.environ.copy()
    env.pop("QT_PLUGIN_PATH", None)
    env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
    return env


class ManagedProcess:
    def __init__(self, name: str, command: str, log_queue: "queue.Queue[str]") -> None:
        self.name = name
        self.command = command
        self.log_queue = log_queue
        self.proc: Optional[subprocess.Popen[str]] = None

    def is_running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self) -> None:
        if self.is_running():
            self.log_queue.put(f"[{self.name}] already running")
            return
        self.log_queue.put(f"[{self.name}] START {self.command}")
        self.proc = subprocess.Popen(
            ["bash", "-lc", self.command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=_child_env(),
            preexec_fn=os.setsid,
        )
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        assert self.proc is not None
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            self.log_queue.put(f"[{self.name}] {line.rstrip()}")
        code = self.proc.wait()
        self.log_queue.put(f"[{self.name}] EXIT code={code}")

    def terminate(self) -> None:
        if not self.is_running() or self.proc is None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except Exception:
            self.proc.terminate()


class MainWindow(QMainWindow):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.setWindowTitle("UR10 实机控制界面")
        self.resize(1100, 720)

        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.driver_process: Optional[ManagedProcess] = None
        self.moveit_process: Optional[ManagedProcess] = None
        self.full_stack_process: Optional[ManagedProcess] = None
        self.cleanup_process: Optional[ManagedProcess] = None
        self.workspace = Path(args.workspace)
        self.preflight_seen_markers: set[str] = set()
        self.preflight_failed = False
        self.preflight_running = False
        self.diagnostic_buttons: dict[str, QPushButton] = {}

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        net_group = QGroupBox("网络配置")
        net_layout = QGridLayout(net_group)
        self.robot_ip = QLineEdit(args.robot_ip)
        self.reverse_ip = QLineEdit(args.reverse_ip)
        net_layout.addWidget(QLabel("机械臂 IP"), 0, 0)
        net_layout.addWidget(self.robot_ip, 0, 1)
        net_layout.addWidget(QLabel("本机回连 IP"), 1, 0)
        net_layout.addWidget(self.reverse_ip, 1, 1)
        root.addWidget(net_group)

        button_row = QHBoxLayout()
        self.start_driver_button = QPushButton("启动驱动")
        self.preflight_button = QPushButton("执行预检")
        self.start_moveit_button = QPushButton("启动 MoveIt + RViz")
        self.start_full_stack_button = QPushButton("启动完整链路")
        self.cleanup_button = QPushButton("清理残余进程")

        for button in (
            self.start_driver_button,
            self.preflight_button,
            self.start_moveit_button,
            self.start_full_stack_button,
            self.cleanup_button,
        ):
            button_row.addWidget(button)
        self.set_preflight_button_idle()
        root.addLayout(button_row)

        diag_group = QGroupBox("详细检测")
        diag_layout = QHBoxLayout(diag_group)
        for mode, label in DIAGNOSTIC_BUTTONS:
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, selected=mode: self.run_diagnostic(selected))
            diag_layout.addWidget(button)
            self.diagnostic_buttons[mode] = button
            self.set_diagnostic_button_idle(mode)
        root.addWidget(diag_group)

        self.status_label = QLabel("空闲")
        root.addWidget(self.status_label)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        root.addWidget(self.log_view)

        self.start_driver_button.clicked.connect(self.start_driver)
        self.preflight_button.clicked.connect(self.run_preflight)
        self.start_moveit_button.clicked.connect(self.start_moveit_rviz)
        self.start_full_stack_button.clicked.connect(self.start_full_stack)
        self.cleanup_button.clicked.connect(self.cleanup)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.flush_logs)
        self.timer.start(100)

    def ros_env_prefix(self) -> str:
        return (
            "source /opt/ros/humble/setup.bash && "
            "source /home/liuxiaopeng/ws_moveit2/install/setup.bash && "
            f"source {self.workspace / 'install' / 'setup.bash'} && "
        )

    def driver_command(self) -> str:
        return (
            self.ros_env_prefix()
            + "ros2 launch ur10_real_control_ros2 ur_driver_only.launch.py "
            + f"robot_ip:={self.robot_ip.text().strip()} "
            + f"reverse_ip:={self.reverse_ip.text().strip()} "
            + "ur_type:=ur10 headless_mode:=false launch_dashboard_client:=false"
        )

    def moveit_command(self) -> str:
        return (
            self.ros_env_prefix()
            + "ros2 launch ur10_real_control_ros2 ur10_real_control.launch.py "
            + f"robot_ip:={self.robot_ip.text().strip()} "
            + f"reverse_ip:={self.reverse_ip.text().strip()} "
            + "ur_type:=ur10 launch_driver:=false launch_rviz:=true launch_rsp:=false"
        )

    def full_stack_command(self) -> str:
        return (
            self.ros_env_prefix()
            + "ros2 launch ur10_real_control_ros2 ur10_real_control.launch.py "
            + f"robot_ip:={self.robot_ip.text().strip()} "
            + f"reverse_ip:={self.reverse_ip.text().strip()} "
            + "ur_type:=ur10 launch_driver:=true launch_rviz:=true launch_rsp:=false launch_dashboard_client:=false"
        )

    def ensure_idle(self, allowed_running: Optional[set[str]] = None) -> bool:
        allowed_running = allowed_running or set()
        active = [
            proc.name
            for proc in (
                self.driver_process,
                self.moveit_process,
                self.full_stack_process,
                self.cleanup_process,
            )
            if proc is not None and proc.is_running() and proc.name not in allowed_running
        ]
        if active:
            self.log_queue.put(f"[gui] running processes exist: {', '.join(active)}")
            return False
        return True

    def set_preflight_button_idle(self) -> None:
        self.preflight_button.setText("执行预检")
        self.preflight_button.setStyleSheet(PREFLIGHT_IDLE_STYLE)

    def set_preflight_button_pass(self) -> None:
        self.preflight_button.setText("预检通过")
        self.preflight_button.setStyleSheet(PREFLIGHT_PASS_STYLE)

    def set_preflight_button_fail(self) -> None:
        self.preflight_button.setText("预检失败")
        self.preflight_button.setStyleSheet(PREFLIGHT_FAIL_STYLE)

    def set_preflight_button_running(self) -> None:
        self.preflight_button.setText("预检中")
        self.preflight_button.setStyleSheet(PREFLIGHT_IDLE_STYLE)

    def set_diagnostic_button_idle(self, mode: str) -> None:
        label = self.diagnostic_label(mode)
        self.diagnostic_buttons[mode].setText(label)
        self.diagnostic_buttons[mode].setStyleSheet(PREFLIGHT_IDLE_STYLE)

    def set_diagnostic_button_running(self, mode: str) -> None:
        label = self.diagnostic_label(mode)
        self.diagnostic_buttons[mode].setText(f"{label} 检测中")
        self.diagnostic_buttons[mode].setStyleSheet(PREFLIGHT_IDLE_STYLE)

    def set_diagnostic_button_pass(self, mode: str) -> None:
        label = self.diagnostic_label(mode)
        self.diagnostic_buttons[mode].setText(f"{label} 通过")
        self.diagnostic_buttons[mode].setStyleSheet(PREFLIGHT_PASS_STYLE)

    def set_diagnostic_button_fail(self, mode: str) -> None:
        label = self.diagnostic_label(mode)
        self.diagnostic_buttons[mode].setText(f"{label} 失败")
        self.diagnostic_buttons[mode].setStyleSheet(PREFLIGHT_FAIL_STYLE)

    def diagnostic_label(self, mode: str) -> str:
        return dict(DIAGNOSTIC_BUTTONS).get(mode, mode)

    def start_driver(self) -> None:
        if not self.ensure_idle():
            return
        self.driver_process = ManagedProcess("driver", self.driver_command(), self.log_queue)
        self.driver_process.start()
        self.status_label.setText("正在启动驱动")

    def start_moveit_rviz(self) -> None:
        if not self.ensure_idle(allowed_running={"driver"}):
            return
        self.moveit_process = ManagedProcess("moveit_rviz", self.moveit_command(), self.log_queue)
        self.moveit_process.start()
        self.status_label.setText("正在启动 MoveIt + RViz")

    def start_full_stack(self) -> None:
        if not self.ensure_idle():
            return
        self.full_stack_process = ManagedProcess("full_stack", self.full_stack_command(), self.log_queue)
        self.full_stack_process.start()
        self.status_label.setText("正在启动完整链路")

    def run_preflight(self) -> None:
        if not self.ensure_idle(allowed_running={"driver", "moveit_rviz", "full_stack"}):
            return
        self.preflight_seen_markers.clear()
        self.preflight_failed = False
        self.preflight_running = True
        self.set_preflight_button_running()
        command = (
            self.ros_env_prefix()
            + f"ROBOT_IP={self.robot_ip.text().strip()} ros2 run ur10_real_control_ros2 check_real_ur10_ready.sh {self.robot_ip.text().strip()}"
        )
        ManagedProcess("preflight", command, self.log_queue).start()
        self.status_label.setText("正在执行预检")

    def diagnostic_command(self, mode: str) -> str:
        script = self.workspace / "ur10_real_control_ros2" / "scripts" / "ur10_comm_diagnostics.py"
        return (
            f"python3 {script} {mode} "
            + f"--robot-ip {self.robot_ip.text().strip()} "
            + f"--reverse-ip {self.reverse_ip.text().strip()} "
            + "--timeout 3"
        )

    def run_diagnostic(self, mode: str) -> None:
        if not self.ensure_idle(allowed_running={"driver", "moveit_rviz", "full_stack"}):
            return
        self.set_diagnostic_button_running(mode)
        ManagedProcess(f"diag_{mode}", self.diagnostic_command(mode), self.log_queue).start()
        self.status_label.setText(f"正在执行详细检测：{self.diagnostic_label(mode)}")

    def update_preflight_state_from_log(self, line: str) -> None:
        if not line.startswith("[preflight] "):
            return
        message = line.removeprefix("[preflight] ").strip()
        if message in PREFLIGHT_REQUIRED_MARKERS:
            self.preflight_seen_markers.add(message)
        if message in PREFLIGHT_FAILURE_MARKERS or "ERROR" in message or "Error" in message:
            self.preflight_failed = True
        if message.startswith("EXIT code="):
            self.preflight_running = False
            if self.preflight_failed or self.preflight_seen_markers != PREFLIGHT_REQUIRED_MARKERS:
                self.set_preflight_button_fail()
                missing = sorted(PREFLIGHT_REQUIRED_MARKERS - self.preflight_seen_markers)
                if missing:
                    self.log_queue.put(f"[gui] preflight missing markers: {', '.join(missing)}")
            else:
                self.set_preflight_button_pass()

    def update_diagnostic_state_from_log(self, line: str) -> None:
        for mode, _label in DIAGNOSTIC_BUTTONS:
            prefix = f"[diag_{mode}] "
            if not line.startswith(prefix):
                continue
            message = line.removeprefix(prefix).strip()
            if message == "DIAG_OK":
                self.set_diagnostic_button_pass(mode)
            elif message == "DIAG_FAIL":
                self.set_diagnostic_button_fail(mode)
            elif message.startswith("EXIT code=") and not message.endswith("=0"):
                self.set_diagnostic_button_fail(mode)
            return

    def cleanup(self) -> None:
        if self.cleanup_process is not None and self.cleanup_process.is_running():
            self.log_queue.put("[gui] cleanup already running")
            return
        self.set_control_buttons_enabled(False)
        for proc in (self.driver_process, self.moveit_process, self.full_stack_process):
            if proc is not None:
                proc.terminate()
        command = self.ros_env_prefix() + "ros2 run ur10_real_control_ros2 cleanup_real_control_processes.sh"
        self.cleanup_process = ManagedProcess("cleanup", command, self.log_queue)
        self.cleanup_process.start()
        self.status_label.setText("正在清理残余进程")

    def set_control_buttons_enabled(self, enabled: bool) -> None:
        for button in (
            self.start_driver_button,
            self.preflight_button,
            self.start_moveit_button,
            self.start_full_stack_button,
        ):
            button.setEnabled(enabled)
        for button in self.diagnostic_buttons.values():
            button.setEnabled(enabled)

    def update_cleanup_state_from_log(self, line: str) -> None:
        if not line.startswith("[cleanup] EXIT code="):
            return
        self.set_control_buttons_enabled(True)
        self.status_label.setText("清理完成")

    def flush_logs(self) -> None:
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.log_view.append(line)
            self.update_preflight_state_from_log(line)
            self.update_diagnostic_state_from_log(line)
            self.update_cleanup_state_from_log(line)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="10.160.9.21")
    parser.add_argument("--reverse-ip", default="10.160.9.10")
    parser.add_argument("--workspace", default="/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model")
    args, _ = parser.parse_known_args()
    return args


def main() -> int:
    args = parse_args()
    app = QApplication(sys.argv)
    window = MainWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
