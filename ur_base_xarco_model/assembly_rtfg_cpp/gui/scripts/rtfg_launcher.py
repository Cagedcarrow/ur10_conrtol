#!/usr/bin/env python3
"""
RTFG 控制面板 — 一键环境准备 + MATLAB 风格操作流程

所有环境准备工作（清理旧进程、启动求解器、编译代码）均可通过按钮执行，
无需记忆复杂命令。

流程:
  ① 环境准备与验证 — 清理旧进程 → 启动求解器 → 检查服务 → 加载配置
  ② RViz2 启动 — 拉起 RViz2 可视化窗口
  ③ 移动到入泥姿态点 — 求解入泥点 IK 并将机械臂移动到轨迹起始位置
  ④ 开始拟合并播放 — 运行轨迹拟合求解，完成后自动播放拟合轨迹
  ⑤ 返回初始姿态 — 播放完毕后回到 URDF 初始位姿

与 MATLAB main_realtime_trajectory_fit_gui.m 行为对齐:
  - 初始姿态 = URDF 加载后的关节角度
  - "移动到入泥姿态点" = MATLAB "移动到轨迹起始点" (approach start)
  - "开始拟合并播放" = 拟合 + 自动播放 (MATLAB 的 "尖端轨迹拟合" + "开始运行")
  - 所有按钮在流程中保持可点击，非一次性

默认以仿真模式运行 (--sim), 无需连接真实机械臂。
使用 --real 可切换到真实硬件模式。

Usage:
    python3 rtfg_launcher.py [--sim | --real]
"""

import os
import sys
import subprocess
import signal
import time
import datetime
import math
import shlex

from PyQt5.QtCore import (
    Qt, QThread, pyqtSignal, QTimer
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTextEdit, QProgressBar, QFrame,
    QSizePolicy
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

APP_NAME = "RTFG 控制面板"
APP_VERSION = "v3.0"
ROS_DISTRO = "humble"
SIMULATION_MODE = True  # default to simulation; override with --real

# Workspace root (two levels up from scripts/)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_GUI_DIR = os.path.dirname(_SCRIPT_DIR)
_PROJECT_ROOT = os.path.abspath(os.path.join(_GUI_DIR, "..", ".."))

CLEARANCE_THRESHOLD = 0.002  # 2 mm

BUTTON_STYLE = """
    QPushButton {
        background-color: #21262d;
        color: #e6edf3;
        border: 1px solid #30363d;
        border-radius: 6px;
        padding: 12px 24px;
        font-size: 14px;
        font-weight: 500;
        min-height: 20px;
    }
    QPushButton:hover {
        background-color: #30363d;
        border-color: #6e7681;
    }
    QPushButton:pressed {
        background-color: #388bfd26;
    }
    QPushButton:disabled {
        opacity: 0.4;
        color: #484f58;
        border-color: #21262d;
    }
"""

BUTTON_PRIMARY_STYLE = """
    QPushButton {
        background-color: #1f6feb;
        color: #ffffff;
        border: 1px solid #1f6feb;
        border-radius: 6px;
        padding: 12px 24px;
        font-size: 14px;
        font-weight: 500;
        min-height: 20px;
    }
    QPushButton:hover {
        background-color: #388bfd;
        border-color: #388bfd;
    }
    QPushButton:pressed {
        background-color: #1a5dc4;
    }
    QPushButton:disabled {
        background-color: #21262d;
        color: #484f58;
        border: 1px solid #30363d;
    }
"""

BUTTON_SUCCESS_STYLE = """
    QPushButton {
        background-color: #238636;
        color: #ffffff;
        border: 1px solid #238636;
        border-radius: 6px;
        padding: 12px 24px;
        font-size: 14px;
        font-weight: 500;
        min-height: 20px;
    }
    QPushButton:hover {
        background-color: #2ea043;
        border-color: #2ea043;
    }
    QPushButton:pressed {
        background-color: #1b6b2a;
    }
    QPushButton:disabled {
        background-color: #21262d;
        color: #484f58;
        border: 1px solid #30363d;
    }
"""

BUTTON_DANGER_STYLE = """
    QPushButton {
        background-color: #da3633;
        color: #ffffff;
        border: 1px solid #da3633;
        border-radius: 6px;
        padding: 12px 24px;
        font-size: 14px;
        font-weight: 500;
        min-height: 20px;
    }
    QPushButton:hover {
        background-color: #f85149;
        border-color: #f85149;
    }
    QPushButton:pressed {
        background-color: #b62324;
    }
    QPushButton:disabled {
        background-color: #21262d;
        color: #484f58;
        border: 1px solid #30363d;
    }
"""

# ---------------------------------------------------------------------------
# Dark theme palette
# ---------------------------------------------------------------------------

DARK_PALETTE = """
    QMainWindow { background-color: #0d1117; }
    QWidget { background-color: #0d1117; color: #e6edf3; font-family: "Segoe UI", "Noto Sans", sans-serif; }
    QLabel { color: #e6edf3; }
    QTextEdit {
        background-color: #161b22;
        color: #e6edf3;
        border: 1px solid #30363d;
        border-radius: 4px;
        font-family: "Cascadia Code", "Fira Code", "SF Mono", monospace;
        font-size: 12px;
        padding: 8px;
    }
    QProgressBar {
        background-color: #21262d;
        border: 1px solid #30363d;
        border-radius: 4px;
        text-align: center;
        color: #e6edf3;
        font-size: 11px;
    }
    QProgressBar::chunk {
        background-color: #3fb950;
        border-radius: 3px;
    }
"""

# ---------------------------------------------------------------------------
# Worker thread for async service calls
# ---------------------------------------------------------------------------

class ServiceWorker(QThread):
    """Run a ROS2 service call in a background thread and emit result."""

    finished = pyqtSignal(dict)   # {success: bool, message: str, ...}
    progress = pyqtSignal(str)    # progress message

    def __init__(self, task_name, task_fn, *args, **kwargs):
        super().__init__()
        self.task_name = task_name
        self.task_fn = task_fn
        self.args = args
        self.kwargs = kwargs

    def run(self):
        try:
            self.progress.emit(f"[{self.task_name}] 正在执行...")
            result = self.task_fn(*self.args, **self.kwargs)
            self.finished.emit(result)
        except Exception as e:
            self.finished.emit({"success": False, "message": str(e), "error": str(e)})


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class RtfgControlPanel(QMainWindow):
    """RTFG MATLAB-style control panel with reusable buttons."""

    def __init__(self):
        super().__init__()
        sim_tag = " [仿真]" if SIMULATION_MODE else " [真实硬件]"
        self.setWindowTitle(f"{APP_NAME} {APP_VERSION}{sim_tag}")
        self.setMinimumSize(580, 680)
        self.resize(600, 720)

        # State
        self._ros_client = None
        self._rviz2_process = None
        self._solver_process = None  # background solver node (managed by env check)
        self._worker = None
        self._start_time = None
        self._config_params = {}
        self._playback_count = 0
        self._is_playing = False  # True during playback (prevents re-click)
        # Flags tracking which steps have been completed at least once
        self._env_ok = False
        self._rviz_ok = False
        self._moved_to_dip = False

        self._init_ui()
        # ROS2 初始化延迟到按钮点击时执行，避免 rclpy 与 Qt 构造冲突导致段错误

    # ------------------------------------------------------------------
    # UI setup
    # ------------------------------------------------------------------

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)

        # ---- Title ----
        title = QLabel(f"<b>{APP_NAME}</b>  <span style='color:#8b949e;'>{APP_VERSION}</span>")
        title.setStyleSheet("font-size: 18px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        sim_subtitle = "仿真模式 · 一键环境准备 + MATLAB 风格流程 (按钮可持续点击)"
        subtitle = QLabel(sim_subtitle)
        subtitle.setStyleSheet("color: #8b949e; font-size: 12px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(8)

        # ---- Step indicator ----
        self._step_labels = []
        step_row = QHBoxLayout()
        step_row.setSpacing(8)
        step_names = ["① 环境准备验证", "② RViz2 启动", "③ 移动到入泥点",
                       "④ 拟合并播放", "⑤ 返回初始姿态"]
        for i, name in enumerate(step_names):
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(
                "color: #484f58; font-size: 11px; padding: 4px 6px; "
                "border: 1px solid #21262d; border-radius: 4px;"
            )
            lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            step_row.addWidget(lbl)
            self._step_labels.append(lbl)
        layout.addLayout(step_row)

        layout.addSpacing(4)

        # ---- Separator ----
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("background-color: #30363d;")
        sep.setFixedHeight(1)
        layout.addWidget(sep)

        # ---- Buttons ----
        btn_layout = QVBoxLayout()
        btn_layout.setSpacing(8)

        self.btn_env = QPushButton(
            "① 环境准备与验证 — 清理旧进程 → 启动求解器 → 检查服务 → 加载配置")
        self.btn_env.setStyleSheet(BUTTON_STYLE)
        self.btn_env.clicked.connect(self._on_env_check)
        btn_layout.addWidget(self.btn_env)

        self.btn_rviz = QPushButton("② RViz2 启动 — 拉起可视化窗口")
        self.btn_rviz.setStyleSheet(BUTTON_PRIMARY_STYLE)
        self.btn_rviz.setEnabled(False)
        self.btn_rviz.clicked.connect(self._on_rviz_launch)
        btn_layout.addWidget(self.btn_rviz)

        self.btn_move_dip = QPushButton("③ 移动到入泥姿态点 — IK 求解并移动至轨迹起始点")
        self.btn_move_dip.setStyleSheet(BUTTON_STYLE)
        self.btn_move_dip.setEnabled(False)
        self.btn_move_dip.clicked.connect(self._on_move_to_dip)
        btn_layout.addWidget(self.btn_move_dip)

        self.btn_fit_play = QPushButton("④ 开始拟合并播放 — 计算轨迹 → 自动播放")
        self.btn_fit_play.setStyleSheet(BUTTON_SUCCESS_STYLE)
        self.btn_fit_play.setEnabled(False)
        self.btn_fit_play.clicked.connect(self._on_fit_and_play)
        btn_layout.addWidget(self.btn_fit_play)

        self.btn_return_home = QPushButton("⑤ 返回初始姿态 — 回到 URDF 初始位姿")
        self.btn_return_home.setStyleSheet(BUTTON_DANGER_STYLE)
        self.btn_return_home.setEnabled(False)
        self.btn_return_home.clicked.connect(self._on_return_home)
        btn_layout.addWidget(self.btn_return_home)

        layout.addLayout(btn_layout)

        # ---- Progress bar ----
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setFixedHeight(20)
        self.progress.setVisible(False)
        layout.addWidget(self.progress)

        # ---- Status ----
        self.status_label = QLabel("就绪 — 点击 ① 环境准备与验证 一键启动")
        self.status_label.setStyleSheet("color: #8b949e; font-size: 13px; padding: 4px 0;")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # ---- Log area ----
        log_label = QLabel("运行日志")
        log_label.setStyleSheet("color: #8b949e; font-size: 11px; font-weight: 600;")
        layout.addWidget(log_label)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(180)
        layout.addWidget(self.log)

    # ------------------------------------------------------------------
    # ROS2 init
    # ------------------------------------------------------------------

    def _init_ros(self):
        mode_label = "仿真模式" if SIMULATION_MODE else "真实硬件模式"
        self._log(f"正在初始化 ROS2 客户端 ({mode_label})...")
        try:
            if "ROS_DISTRO" not in os.environ:
                ros_setup = f"/opt/ros/{ROS_DISTRO}/setup.bash"
                self._log(f"警告: ROS_DISTRO 未设置, 请先 source {ros_setup}")

            gui_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
            if gui_dir not in sys.path:
                sys.path.insert(0, gui_dir)

            from rtfg_gui.ros_client import RosClient
            self._ros_client = RosClient(simulation_mode=SIMULATION_MODE)
            self._log(f"✅ ROS2 客户端初始化成功 ({mode_label})")
        except Exception as e:
            self._log(f"❌ ROS2 初始化失败: {e}")
            self._ros_client = None

    # ------------------------------------------------------------------
    # Button handlers
    # ------------------------------------------------------------------

    def _on_env_check(self):
        """① 环境准备与验证 — 清理 → 启动求解器 → 检查服务 → 加载配置。"""
        self.btn_env.setEnabled(False)
        self._set_step_active(0)
        self.status_label.setText("环境准备中...")
        self._log("=" * 50)
        self._log("① 环境准备与验证 — 开始")
        self._log("   步骤 1: 清理旧进程")
        self._log("   步骤 2: 启动求解器节点")
        self._log("   步骤 3: 等待服务就绪")
        self._log("   步骤 4: 加载配置参数")

        # 延迟初始化 ROS2（从按钮点击触发，而非构造函数）
        if self._ros_client is None:
            self._log("正在初始化 ROS2 客户端...")
            self._init_ros()
            if self._ros_client is None:
                self._step_failed(0, "ROS2 客户端初始化失败")
                self.btn_env.setEnabled(True)
                return

        self._run_async("环境验证", self._do_env_setup)

    def _do_env_setup(self):
        """Kill old solver, start new one, wait for services, load config."""
        # --- Step 1: Kill old solver node ---
        self._kill_solver()
        try:
            subprocess.run(["pkill", "-f", "rtfg_solver_node"],
                           stderr=subprocess.DEVNULL, timeout=5)
            time.sleep(0.5)  # let process die
        except Exception:
            pass

        # --- Step 2: Start new solver node ---
        env = os.environ.copy()
        cmd = ["ros2", "run", "assembly_rtfg_cuda", "rtfg_solver_node"]
        try:
            self._solver_process = subprocess.Popen(
                cmd, env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except FileNotFoundError:
            return {"success": False,
                    "message": "ros2 命令未找到, ROS2 环境可能未正确加载"}
        except Exception as e:
            return {"success": False,
                    "message": f"启动求解器失败: {e}"}

        # --- Step 3: Wait for service to become available ---
        # Re-init ROS client for fresh connection
        if self._ros_client is not None:
            try:
                self._ros_client.destroy()
            except Exception:
                pass
            self._ros_client = None
        self._init_ros()
        if self._ros_client is None:
            return {"success": False,
                    "message": "ROS2 客户端初始化失败"}

        # Poll for load_config service
        from assembly_rtfg_cuda.srv import LoadConfig
        temp_cli = self._ros_client._node.create_client(
            LoadConfig, "/rtfg/load_config")

        found = False
        for attempt in range(30):  # up to 30 seconds
            if temp_cli.wait_for_service(timeout_sec=1.0):
                found = True
                break
            if attempt % 5 == 4:
                self._log(f"   等待求解器就绪... ({attempt + 1}/30)")
        if not found:
            return {"success": False,
                    "message": "求解器节点未能在 30 秒内就绪, 请检查日志"}

        # --- Step 4: Load config ---
        try:
            config = self._ros_client.load_config()
        except Exception as e:
            return {"success": False,
                    "message": f"load_config 调用失败: {e}"}

        if not config.get("success"):
            return {"success": False,
                    "message": f"load_config 失败: {config.get('message')}"}

        initial_q = list(config.get("initial_q", []))
        self._config_params = {
            "left_wall_offset": config.get("left_wall_offset", 0.0),
            "mud_height": config.get("mud_height", 0.0),
            "approach_len": config.get("approach_len", 0.0),
            "theta_deg": config.get("theta_deg", 0.0),
            "depth": config.get("depth", 0.0),
            "x_plane": config.get("x_plane", 0.0),
            "pose_x": config.get("pose_x", 0.0),
            "pose_y": config.get("pose_y", 0.0),
            "pose_z": config.get("pose_z", 0.0),
            "roll_deg": config.get("roll_deg", 0.0),
            "pitch_deg": config.get("pitch_deg", 0.0),
            "yaw_deg": config.get("yaw_deg", 0.0),
            "initial_q": initial_q,
            "current_q": initial_q,  # start from URDF initial pose
            "clearance_threshold": CLEARANCE_THRESHOLD,
        }

        topics = self._ros_client.get_topics()
        return {
            "success": True,
            "message": "环境准备与验证通过",
            "initial_q": config.get("initial_q", []),
            "topics": topics,
            "params": self._config_params,
        }

    def _on_env_check_done(self, result):
        if result.get("success"):
            self._env_ok = True
            self._log(f"✅ 环境验证通过!")
            params = result.get("params", {})
            initial_q = result.get("initial_q", [])
            self._log(f"   求解参数: left_wall_offset={params.get('left_wall_offset', '?'):.4f}, "
                      f"pose_x={params.get('pose_x', '?'):.3f}, "
                      f"yaw={params.get('yaw_deg', '?'):.1f}°")
            self._log(f"   URDF 初始姿态 (initial_q): {[f'{v:.3f}' for v in initial_q]}")
            self._log(f"   发现 {len(result.get('topics', []))} 个 /rtfg/ topic")
            self._set_step_completed(0)
            self.btn_rviz.setEnabled(True)
            self.btn_move_dip.setEnabled(True)
            self.btn_rviz.setFocus()
            self.status_label.setText(
                "环境就绪 — 点击 ② RViz2 启动 或 ③ 移动到入泥姿态点")
        else:
            self._log(f"❌ 环境准备失败: {result.get('message', '未知错误')}")
            self._step_failed(0, result.get("message", "未知错误"))
            self.btn_env.setEnabled(True)

    # ------------------------------------------------------------------

    def _on_rviz_launch(self):
        """② RViz2 启动 — 拉起 RViz2 可视化窗口。"""
        self.btn_rviz.setEnabled(False)
        self._set_step_active(1)
        self.status_label.setText("正在启动 RViz2...")
        self._log("② RViz2 启动 — 开始")

        config_path = self._find_rviz_config()
        if config_path is None:
            self._log("❌ 未找到 RViz2 配置文件")
            self._step_failed(1, "rtfg_display.rviz 未找到")
            self.btn_rviz.setEnabled(True)
            return

        self._log(f"   配置文件: {config_path}")
        self._kill_rviz2()

        try:
            cmd = ["ros2", "run", "rviz2", "rviz2", "-d", config_path]
            self._log(f"   启动命令: {' '.join(cmd)}")
            self._rviz2_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            time.sleep(2)
            poll = self._rviz2_process.poll()
            if poll is not None:
                self._log(f"❌ RViz2 进程立即退出, 退出码: {poll}")
                self._step_failed(1, f"RViz2 退出码 {poll}")
                self.btn_rviz.setEnabled(True)
                return

            self._rviz_ok = True
            self._log(f"✅ RViz2 启动成功 (PID {self._rviz2_process.pid})")
            self._set_step_completed(1)
            self.btn_rviz.setEnabled(True)  # remains clickable
            self.btn_move_dip.setEnabled(True)
            self.btn_move_dip.setFocus()
            self.status_label.setText("RViz2 已启动 — 可点击 ③ 移动到入泥姿态点")

        except FileNotFoundError:
            self._log("❌ ros2 命令未找到, 请确保已 source ROS2 环境")
            self._step_failed(1, "ros2 命令未找到")
            self.btn_rviz.setEnabled(True)
        except Exception as e:
            self._log(f"❌ RViz2 启动失败: {e}")
            self._step_failed(1, str(e))
            self.btn_rviz.setEnabled(True)

    # ------------------------------------------------------------------

    def _on_move_to_dip(self):
        """③ 移动到入泥姿态点 — 求解入泥点 IK，移动至轨迹起始点。

        对应 MATLAB main_realtime_trajectory_fit_gui.m 中的 "移动到轨迹起始点"。
        """
        self.btn_move_dip.setEnabled(False)
        self._set_step_active(2)
        self.status_label.setText("正在求解入泥点 IK 并移动...")
        self._log("③ 移动到入泥姿态点 — 开始")
        self._log("   对应 MATLAB '移动到轨迹起始点': 求解第一个目标位姿的 IK")

        self._run_async("移动到入泥姿态点", self._ros_client.move_to_start)

    def _on_move_to_dip_done(self, result):
        if result.get("success"):
            self._moved_to_dip = True
            self._log("✅ 已移动到入泥姿态点 (轨迹起始点)")
            self._set_step_completed(2)
            self.btn_move_dip.setEnabled(True)  # can click again after returning
            self.btn_fit_play.setEnabled(True)
            self.btn_fit_play.setFocus()
            self.status_label.setText("已就位 — 点击 ④ 开始拟合并播放")
        else:
            self._log(f"❌ 移动到入泥姿态点失败: {result.get('message', '未知错误')}")
            self._step_failed(2, result.get("message", "未知错误"))
            self.btn_move_dip.setEnabled(True)

    # ------------------------------------------------------------------

    def _on_fit_and_play(self):
        """④ 开始拟合并播放 — 计算轨迹，完成后自动播放。

        对应 MATLAB 的 "尖端轨迹拟合" + "开始运行" 两个步骤合一。
        """
        self.btn_fit_play.setEnabled(False)
        self._set_step_active(3)
        self.status_label.setText("正在计算轨迹 (约 10-15 秒, 请耐心等待)...")
        self._log("④ 开始拟合并播放 — 开始")
        self._log("   步骤一: 轨迹拟合求解 (MATLAB '尖端轨迹拟合')")
        params = self._config_params
        self._log(f"   left_wall_offset={params.get('left_wall_offset', 0):.4f}, "
                  f"pose_x={params.get('pose_x', 0):.3f}, "
                  f"yaw={params.get('yaw_deg', 0):.1f}°, "
                  f"theta={params.get('theta_deg', 0):.1f}°")

        self.progress.setVisible(True)
        self.progress.setRange(0, 0)  # indeterminate
        self._start_time = time.time()

        self._elapsed_timer = QTimer()
        self._elapsed_timer.timeout.connect(self._update_elapsed)
        self._elapsed_timer.start(2000)

        self._run_async("计算轨迹", self._ros_client.fit_preview, self._config_params)

    def _update_elapsed(self):
        if self._start_time:
            elapsed = time.time() - self._start_time
            self.status_label.setText(
                f"正在计算轨迹... 已耗时 {elapsed:.0f} 秒 (约 10-15 秒)")

    def _on_fit_done(self, result):
        """Called when fit_preview completes — then auto-starts playback."""
        self.progress.setVisible(False)
        if hasattr(self, '_elapsed_timer'):
            self._elapsed_timer.stop()

        elapsed = time.time() - self._start_time if self._start_time else 0

        # Check if trajectory was solved (even with collision warnings)
        anchor_count = result.get('anchor_count', 0)
        if anchor_count <= 0:
            self._log(f"❌ 轨迹计算失败 (耗时 {elapsed:.0f} 秒): "
                      f"{result.get('message', '未知错误')}")
            self._step_failed(3, result.get("message", "未知错误"))
            self.btn_fit_play.setEnabled(True)
            return

        # --- Fit succeeded, log results ---
        has_warnings = not result.get("success", True)
        prefix = "⚠️" if has_warnings else "✅"
        self._log(f"{prefix} 轨迹计算完成! (耗时 {elapsed:.0f} 秒)")
        pb_count = result.get('playback_count', 0)
        self._playback_count = pb_count
        self._log(f"   锚点数: {anchor_count}")
        self._log(f"   播放点数: {pb_count}")
        self._log(f"   IK 耗时: {result.get('timing_ik_total_s', '?'):.2f} s")
        self._log(f"   碰撞检测耗时: {result.get('timing_collision_total_s', '?'):.2f} s")
        self._log(f"   最大播放步长: {result.get('max_playback_joint_step_deg', '?'):.2f}°")
        self._log(f"   最小 clearance: {result.get('min_tool_basin_clearance', 0) * 1000:.2f} mm")

        if has_warnings:
            self._log(f"   ⚠️ 碰撞审计发现违规, 但轨迹已缓存可执行")
            self._log(f"   碰撞点数: {len(result.get('collision_objects', []))}")

        self._set_step_completed(3)

        # --- Step 2: Auto-play the trajectory ---
        self._log("   步骤二: 自动播放拟合轨迹 (MATLAB '开始运行')")
        self._log(f"   播放 {pb_count} 个点, 估算时长 ~{pb_count * 0.025:.0f} 秒")
        self.status_label.setText(f"轨迹计算完成 — 正在自动播放 ({pb_count} 点)...")

        self._run_async("执行轨迹", self._do_play_and_wait, pb_count)

    def _do_play_and_wait(self, playback_count):
        """Execute cached trajectory and wait for playback to finish."""
        # Call execute_cached to start playback
        exec_result = self._ros_client.execute_cached()
        if not exec_result.get("success"):
            return {
                "success": False,
                "message": f"启动播放失败: {exec_result.get('message')}",
                "playback_count": playback_count,
            }

        # Wait for estimated playback duration
        # 50 Hz playback = 0.02s per point, plus 2s overhead
        wait_sec = playback_count * 0.025 + 2.0
        self._is_playing = True
        time.sleep(wait_sec)
        self._is_playing = False

        return {
            "success": True,
            "message": f"播放完成 ({playback_count} 点, 约 {wait_sec:.0f} 秒)",
            "playback_count": playback_count,
        }

    def _on_play_done(self, result):
        """Called when auto-playback completes."""
        if result.get("success"):
            self._log(f"✅ {result.get('message', '播放完成')}")
            self._set_step_completed(3)  # keep step 4 highlighted as completed
            self.status_label.setText("✅ 播放完成 — 可点击 ⑤ 返回初始姿态 或 ③ 重新移动到入泥点")
            # Enable return-home AND re-enable fit-play so user can repeat
            self.btn_return_home.setEnabled(True)
            self.btn_fit_play.setEnabled(True)   # can fit and play again
            self.btn_move_dip.setEnabled(True)   # can move to dip again
            self.btn_return_home.setFocus()
        else:
            self._log(f"❌ 播放失败: {result.get('message', '未知错误')}")
            self._step_failed(3, result.get("message", "未知错误"))
            self.btn_fit_play.setEnabled(True)

    # ------------------------------------------------------------------

    def _on_return_home(self):
        """⑤ 返回初始姿态 — 回到 URDF 初始位姿。"""
        self.btn_return_home.setEnabled(False)
        self._set_step_active(4)
        self.status_label.setText("正在返回初始姿态...")
        self._log("⑤ 返回初始姿态 — 回到 URDF 初始位姿")
        self._log("   对应 MATLAB 加载 URDF 后的初始关节角度")

        self._run_async("返回初始姿态", self._ros_client.move_to_home)

    def _on_return_home_done(self, result):
        if result.get("success"):
            self._log("✅ 已返回初始姿态 (URDF 初始位姿)")
            self._set_step_completed(4)
            self.status_label.setText("已回到初始姿态 — 可点击 ③ 重新移动到入泥点")
            self.btn_return_home.setEnabled(True)  # stays clickable
            self.btn_move_dip.setEnabled(True)     # can move to dip again
            self.btn_fit_play.setEnabled(True)     # can fit again
        else:
            self._log(f"❌ 返回初始姿态失败: {result.get('message', '未知错误')}")
            self._step_failed(4, result.get("message", "未知错误"))
            self.btn_return_home.setEnabled(True)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _run_async(self, task_name, task_fn, *args, **kwargs):
        """Run a task in background thread."""
        self._worker = ServiceWorker(task_name, task_fn, *args, **kwargs)
        self._worker.progress.connect(self._log)

        handlers = {
            "环境验证": self._on_env_check_done,
            "移动到入泥姿态点": self._on_move_to_dip_done,
            "计算轨迹": self._on_fit_done,
            "执行轨迹": self._on_play_done,
            "返回初始姿态": self._on_return_home_done,
        }
        handler = handlers.get(task_name)
        if handler:
            self._worker.finished.connect(handler)
        self._worker.finished.connect(lambda r: self._log_result(r, task_name))
        self._worker.start()

    def _log_result(self, result, task_name):
        if not result.get("success"):
            self._log(f"[{task_name}] 失败: {result.get('message', 'unknown')}")

    def _log(self, text):
        """线程安全的日志追加（仅使用 append，避免 QTextCursor 跨线程崩溃）。"""
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.log.append(f"[{ts}] {text}")

    def _set_step_active(self, step):
        """Highlight step indicator for currently active step."""
        for i, lbl in enumerate(self._step_labels):
            if i == step:
                lbl.setStyleSheet(
                    "color: #58a6ff; font-size: 11px; padding: 4px 6px; "
                    "background-color: #1f6feb22; border: 1px solid #1f6feb; border-radius: 4px;"
                )
            elif self._is_step_completed(i):
                lbl.setStyleSheet(
                    "color: #3fb950; font-size: 11px; padding: 4px 6px; "
                    "background-color: #23863622; border: 1px solid #238636; border-radius: 4px;"
                )
            else:
                lbl.setStyleSheet(
                    "color: #484f58; font-size: 11px; padding: 4px 6px; "
                    "border: 1px solid #21262d; border-radius: 4px;"
                )

    def _set_step_completed(self, step):
        """Mark step indicator as completed (green)."""
        self._step_labels[step].setStyleSheet(
            "color: #3fb950; font-size: 11px; padding: 4px 6px; "
            "background-color: #23863622; border: 1px solid #238636; border-radius: 4px;"
        )

    def _is_step_completed(self, step):
        """Check if a step indicator shows completed style."""
        lbl = self._step_labels[step]
        return "3fb950" in lbl.styleSheet()

    def _step_failed(self, step, msg):
        self._log(f"❌ 步骤 {step + 1} 失败: {msg}")
        self.status_label.setText(f"步骤 {step + 1} 失败 — {msg}")
        self._set_step_active(step)
        if 0 <= step < len(self._step_labels):
            self._step_labels[step].setStyleSheet(
                "color: #f85149; font-size: 11px; padding: 4px 6px; "
                "background-color: #f8514922; border: 1px solid #f85149; border-radius: 4px;"
            )

    def _find_rviz_config(self):
        """Locate rtfg_display.rviz."""
        candidates = []
        this_dir = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.join(this_dir, "..", "config", "rtfg_display.rviz"))

        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("assembly_rtfg_cuda")
            candidates.append(os.path.join(share, "config", "rtfg_display.rviz"))
            candidates.append(os.path.join(share, "rviz", "rtfg_view.rviz"))
        except Exception:
            pass

        ws = os.environ.get("COLCON_PREFIX_PATH", "").split(":")[0] if "COLCON_PREFIX_PATH" in os.environ else ""
        if ws:
            candidates.append(os.path.join(ws, "share", "assembly_rtfg_cuda", "rviz", "rtfg_view.rviz"))

        for c in candidates:
            c = os.path.normpath(os.path.abspath(c))
            if os.path.isfile(c):
                return c

        self._log(f"   搜索路径: {candidates}")
        return None

    def _kill_rviz2(self):
        if self._rviz2_process is not None:
            try:
                os.killpg(os.getpgid(self._rviz2_process.pid), signal.SIGTERM)
                self._rviz2_process.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._rviz2_process.pid), signal.SIGKILL)
                except Exception:
                    pass
            self._rviz2_process = None

    def _kill_solver(self):
        """Kill the solver node process if we started it."""
        if self._solver_process is not None:
            try:
                os.killpg(os.getpgid(self._solver_process.pid), signal.SIGTERM)
                self._solver_process.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._solver_process.pid), signal.SIGKILL)
                except Exception:
                    pass
            self._solver_process = None

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event):
        self._log("正在关闭...")
        self._log("正在停止求解器节点...")
        self._kill_solver()
        self._kill_rviz2()
        if self._ros_client is not None:
            try:
                self._ros_client.destroy()
            except Exception:
                pass
        self._log("已关闭")
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _check_ros2_sourced():
    """Return True if the current process has ROS2 humble properly sourced."""
    if os.environ.get("ROS_DISTRO") != ROS_DISTRO:
        return False
    ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    return f"/opt/ros/{ROS_DISTRO}/lib" in ld_path


def _launch_via_ros2_shell():
    """Restart via a shell script that sources ROS2 before launching Python.

    On Linux, glibc's dynamic linker reads LD_LIBRARY_PATH at PROCESS START
    and caches it for dlopen. The *only* reliable way to make rclpy's C
    extensions find ROS2 shared libraries is to start the Python process
    with the correct LD_LIBRARY_PATH from the very beginning.

    Instead of using os.execve() (which can leave Qt/X11 state in an
    inconsistent condition), we write a one-shot shell script that sources
    ROS2, then replaces itself with the Python interpreter via 'exec'.
    The current process exits cleanly; the child process inherits the
    terminal/display and starts the GUI afresh with the correct environment.
    """
    ros_setup = f"/opt/ros/{ROS_DISTRO}/setup.bash"
    workspace_setup = os.path.join(_PROJECT_ROOT, "install", "setup.bash")

    if not os.path.isfile(ros_setup):
        print(f"[WARN] ROS2 setup not found: {ros_setup}")
        return False

    # Build the shell launcher script
    lines = [
        "#!/bin/bash",
        f"source {ros_setup}",
    ]
    if os.path.isfile(workspace_setup):
        lines.append(f"source {workspace_setup}")
    # exec replaces the shell with the Python process,
    # preserving the freshly sourced environment.
    lines.append(f'exec {shlex.join([sys.executable] + sys.argv)}')
    launcher_path = os.path.join(_SCRIPT_DIR, ".ros2_launcher.sh")
    try:
        with open(launcher_path, "w") as f:
            f.write("\n".join(lines) + "\n")
        os.chmod(launcher_path, 0o755)
    except Exception as e:
        print(f"[WARN] Failed to write launcher script: {e}")
        return False

    print(f"[INFO] Restarting with ROS2 {ROS_DISTRO} environment ...")
    if os.path.isfile(workspace_setup):
        print(f"[INFO]   Workspace overlay: {_PROJECT_ROOT}")

    try:
        subprocess.Popen([launcher_path],
                         stdin=sys.stdin,
                         stdout=sys.stdout,
                         stderr=sys.stderr,
                         preexec_fn=os.setsid)
    except Exception as e:
        print(f"[ERROR] Failed to launch ROS2 wrapper: {e}")
        return False

    # Exit the old process immediately (no Qt has been created yet).
    os._exit(0)


def main():
    global SIMULATION_MODE

    if "--real" in sys.argv:
        SIMULATION_MODE = False
        print("[INFO] 真实硬件模式 (--real): 将实际控制机械臂")
    elif "--sim" in sys.argv:
        SIMULATION_MODE = True
        print("[INFO] 仿真模式 (--sim): 跳过实际运动，RViz2 动画显示")
    else:
        print("[INFO] 默认仿真模式: 用 --real 切换到真实硬件模式")

    if not _check_ros2_sourced():
        _launch_via_ros2_shell()
        # _launch_via_ros2_shell() calls os._exit(0) on success;
        # if we reach here, ROS2 was already sourced or launch failed silently.
        if not _check_ros2_sourced():
            print("[WARN] ROS2 环境未完整加载，服务调用可能失败。")
            print("[WARN] 如果遇到问题，请手动执行:")
            print(f"[WARN]   source /opt/ros/{ROS_DISTRO}/setup.bash")
            print(f"[WARN]   source {_PROJECT_ROOT}/install/setup.bash")

    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_PALETTE)
    app.setApplicationName(APP_NAME)

    window = RtfgControlPanel()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
