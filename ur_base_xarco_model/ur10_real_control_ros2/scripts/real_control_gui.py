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
        self.setWindowTitle("UR10 Real Control GUI")
        self.resize(1100, 720)

        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.driver_process: Optional[ManagedProcess] = None
        self.moveit_process: Optional[ManagedProcess] = None
        self.full_stack_process: Optional[ManagedProcess] = None
        self.workspace = Path(args.workspace)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        net_group = QGroupBox("Network")
        net_layout = QGridLayout(net_group)
        self.robot_ip = QLineEdit(args.robot_ip)
        self.reverse_ip = QLineEdit(args.reverse_ip)
        net_layout.addWidget(QLabel("Robot IP"), 0, 0)
        net_layout.addWidget(self.robot_ip, 0, 1)
        net_layout.addWidget(QLabel("Reverse IP"), 1, 0)
        net_layout.addWidget(self.reverse_ip, 1, 1)
        root.addWidget(net_group)

        button_row = QHBoxLayout()
        self.start_driver_button = QPushButton("Start Driver")
        self.preflight_button = QPushButton("Run Preflight")
        self.start_moveit_button = QPushButton("Start MoveIt + RViz")
        self.start_full_stack_button = QPushButton("Start Full Stack")
        self.cleanup_button = QPushButton("Cleanup Processes")

        for button in (
            self.start_driver_button,
            self.preflight_button,
            self.start_moveit_button,
            self.start_full_stack_button,
            self.cleanup_button,
        ):
            button_row.addWidget(button)
        root.addLayout(button_row)

        self.status_label = QLabel("Idle")
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
            + "ur_type:=ur10 headless_mode:=false launch_dashboard_client:=true"
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
            + "ur_type:=ur10 launch_driver:=true launch_rviz:=true launch_rsp:=false"
        )

    def ensure_idle(self) -> bool:
        active = [
            proc.name
            for proc in (self.driver_process, self.moveit_process, self.full_stack_process)
            if proc is not None and proc.is_running()
        ]
        if active:
            self.log_queue.put(f"[gui] running processes exist: {', '.join(active)}")
            return False
        return True

    def start_driver(self) -> None:
        if not self.ensure_idle():
            return
        self.driver_process = ManagedProcess("driver", self.driver_command(), self.log_queue)
        self.driver_process.start()
        self.status_label.setText("Driver launching")

    def start_moveit_rviz(self) -> None:
        if not self.ensure_idle():
            return
        self.moveit_process = ManagedProcess("moveit_rviz", self.moveit_command(), self.log_queue)
        self.moveit_process.start()
        self.status_label.setText("MoveIt + RViz launching")

    def start_full_stack(self) -> None:
        if not self.ensure_idle():
            return
        self.full_stack_process = ManagedProcess("full_stack", self.full_stack_command(), self.log_queue)
        self.full_stack_process.start()
        self.status_label.setText("Full stack launching")

    def run_preflight(self) -> None:
        command = (
            self.ros_env_prefix()
            + f"ROBOT_IP={self.robot_ip.text().strip()} ros2 run ur10_real_control_ros2 check_real_ur10_ready.sh {self.robot_ip.text().strip()}"
        )
        ManagedProcess("preflight", command, self.log_queue).start()
        self.status_label.setText("Preflight running")

    def cleanup(self) -> None:
        for proc in (self.driver_process, self.moveit_process, self.full_stack_process):
            if proc is not None:
                proc.terminate()
        command = self.ros_env_prefix() + "ros2 run ur10_real_control_ros2 cleanup_real_control_processes.sh"
        ManagedProcess("cleanup", command, self.log_queue).start()
        self.status_label.setText("Cleanup triggered")

    def flush_logs(self) -> None:
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.log_view.append(line)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="10.160.9.21")
    parser.add_argument("--reverse-ip", default="10.160.9.100")
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
