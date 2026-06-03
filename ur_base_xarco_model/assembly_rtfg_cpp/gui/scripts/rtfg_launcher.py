#!/usr/bin/env python3
"""
RTFG 控制面板 — PyQt5 5-Step Control Panel

5 步操作流程:
  ① 环境验证 — 检查 ROS2 求解器服务是否就绪
  ② RViz2 启动 — 拉起 RViz2 可视化窗口
  ③ 移动到起始点 — 机器人移动到初始关节位姿 (仿真模式: 仅记录目标关节角)
  ④ 计算轨迹 — 运行轨迹拟合求解 (约 5 分钟)
  ⑤ 开始运动 — 执行拟合后的轨迹 (仿真模式: 跳过实际运动)

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
APP_VERSION = "v2.2"
ROS_DISTRO = "humble"
SIMULATION_MODE = True  # default to simulation; override with --real

# Workspace root (two levels up from scripts/)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_GUI_DIR = os.path.dirname(_SCRIPT_DIR)
_PROJECT_ROOT = os.path.abspath(os.path.join(_GUI_DIR, "..", ".."))

# Parameters are loaded from solver config at runtime (Step 1).
# They come from environment_runtime_config.yaml — tuned for the actual URDF setup.
# The user does NOT need to adjust them; they are proven to work (benchmark verified).
CLEARANCE_THRESHOLD = 0.002  # 2 mm — configurable but rarely changed

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
    """RTFG 5-step control panel main window."""

    def __init__(self):
        super().__init__()
        sim_tag = " [仿真]" if SIMULATION_MODE else " [真实硬件]"
        self.setWindowTitle(f"{APP_NAME} {APP_VERSION}{sim_tag}")
        self.setMinimumSize(560, 640)
        self.resize(580, 680)

        # State
        self._current_step = 0
        self._ros_client = None
        self._rviz2_process = None
        self._fit_success = False
        self._worker = None
        self._start_time = None
        self._config_params = {}  # populated by Step 1 (load_config)

        self._init_ui()
        self._init_ros()

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

        sim_subtitle = "仿真模式 · 5 步操作流程 (步骤 ③⑤ 跳过实际运动)"
        subtitle = QLabel(sim_subtitle)
        subtitle.setStyleSheet("color: #8b949e; font-size: 12px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(8)

        # ---- Step indicator ----
        self._step_labels = []
        step_row = QHBoxLayout()
        step_row.setSpacing(8)
        step_names = ["① 环境验证", "② RViz2 启动", "③ 移动到起始点", "④ 计算轨迹", "⑤ 开始运动"]
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

        self.btn_env = QPushButton("① 环境验证 — 检查 ROS2 求解器服务")
        self.btn_env.setStyleSheet(BUTTON_STYLE)
        self.btn_env.clicked.connect(self._on_step1)
        btn_layout.addWidget(self.btn_env)

        self.btn_rviz = QPushButton("② RViz2 启动 — 拉起可视化窗口")
        self.btn_rviz.setStyleSheet(BUTTON_PRIMARY_STYLE)
        self.btn_rviz.setEnabled(False)
        self.btn_rviz.clicked.connect(self._on_step2)
        btn_layout.addWidget(self.btn_rviz)

        self.btn_move = QPushButton("③ 移动到起始点 — 记录初始关节位姿 [仿真]")
        self.btn_move.setStyleSheet(BUTTON_STYLE)
        self.btn_move.setEnabled(False)
        self.btn_move.clicked.connect(self._on_step3)
        btn_layout.addWidget(self.btn_move)

        self.btn_fit = QPushButton("④ 计算轨迹 — 运行轨迹拟合求解 (~5 分钟)")
        self.btn_fit.setStyleSheet(BUTTON_PRIMARY_STYLE)
        self.btn_fit.setEnabled(False)
        self.btn_fit.clicked.connect(self._on_step4)
        btn_layout.addWidget(self.btn_fit)

        self.btn_exec = QPushButton("⑤ 开始运动 — 确认轨迹已缓存 [仿真]")
        self.btn_exec.setStyleSheet(BUTTON_SUCCESS_STYLE)
        self.btn_exec.setEnabled(False)
        self.btn_exec.clicked.connect(self._on_step5)
        btn_layout.addWidget(self.btn_exec)

        layout.addLayout(btn_layout)

        # ---- Progress bar ----
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setFixedHeight(20)
        self.progress.setVisible(False)
        layout.addWidget(self.progress)

        # ---- Status ----
        self.status_label = QLabel("就绪 — 点击 ① 环境验证 开始")
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
            # Ensure ROS2 environment
            if "ROS_DISTRO" not in os.environ:
                ros_setup = f"/opt/ros/{ROS_DISTRO}/setup.bash"
                self._log(f"警告: ROS_DISTRO 未设置, 请先 source {ros_setup}")

            # Add parent dir (gui/) to sys.path so we can import rtfg_gui
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
    # Step 1: 环境验证
    # ------------------------------------------------------------------

    def _on_step1(self):
        self.btn_env.setEnabled(False)
        self._set_step_active(0)
        self.status_label.setText("正在验证 ROS2 环境...")
        self._log("=" * 50)
        self._log("① 环境验证 — 开始")

        if self._ros_client is None:
            self._log("❌ ROS2 客户端未初始化, 尝试重新初始化...")
            self._init_ros()
            if self._ros_client is None:
                self._step_failed(0, "ROS2 客户端初始化失败")
                return

        self._run_async(
            "环境验证",
            self._do_env_check,
        )

    def _do_env_check(self):
        """Check ROS2 services are available and load config parameters."""
        config = self._ros_client.load_config()
        if not config.get("success"):
            return {"success": False, "message": f"load_config 失败: {config.get('message')}"}

        # Store all config parameters for use in Step 4
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
            "current_q": [],
            "clearance_threshold": CLEARANCE_THRESHOLD,
        }

        topics = self._ros_client.get_topics()
        return {
            "success": True,
            "message": "环境验证通过",
            "initial_q": config.get("initial_q", []),
            "topics": topics,
            "params": self._config_params,
        }

    def _on_step1_done(self, result):
        if result.get("success"):
            self._log(f"✅ 环境验证通过!")
            params = result.get("params", {})
            self._log(f"   求解参数: left_wall_offset={params.get('left_wall_offset', '?'):.4f}, "
                      f"pose_x={params.get('pose_x', '?'):.3f}, "
                      f"yaw={params.get('yaw_deg', '?'):.1f}°")
            self._log(f"   initial_q: {[f'{v:.3f}' for v in result.get('initial_q', [])]}")
            self._log(f"   发现 {len(result.get('topics', []))} 个 /rtfg/ topic")
            self._complete_step(0)
            self.btn_rviz.setEnabled(True)
            self.btn_rviz.setFocus()
            self.status_label.setText("环境验证通过 — 点击 ② RViz2 启动")
        else:
            self._step_failed(0, result.get("message", "未知错误"))
            self.btn_env.setEnabled(True)

    # ------------------------------------------------------------------
    # Step 2: RViz2 启动
    # ------------------------------------------------------------------

    def _on_step2(self):
        self.btn_rviz.setEnabled(False)
        self._set_step_active(1)
        self.status_label.setText("正在启动 RViz2...")
        self._log("② RViz2 启动 — 开始")

        # Find rviz config
        config_path = self._find_rviz_config()
        if config_path is None:
            self._log("❌ 未找到 RViz2 配置文件")
            self._step_failed(1, "rtfg_display.rviz 未找到")
            self.btn_rviz.setEnabled(True)
            return

        self._log(f"   配置文件: {config_path}")

        # Kill existing RViz2
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

            self._log(f"✅ RViz2 启动成功 (PID {self._rviz2_process.pid})")
            self._complete_step(1)
            self.btn_move.setEnabled(True)
            self.btn_move.setFocus()
            self.status_label.setText("RViz2 已启动 — 点击 ③ 移动到起始点")

        except FileNotFoundError:
            self._log("❌ ros2 命令未找到, 请确保已 source ROS2 环境")
            self._step_failed(1, "ros2 命令未找到")
            self.btn_rviz.setEnabled(True)
        except Exception as e:
            self._log(f"❌ RViz2 启动失败: {e}")
            self._step_failed(1, str(e))
            self.btn_rviz.setEnabled(True)

    def _find_rviz_config(self):
        """Locate rtfg_display.rviz."""
        candidates = []

        # Source tree
        this_dir = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.join(this_dir, "..", "config", "rtfg_display.rviz"))

        # Installed share
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("assembly_rtfg_cpp")
            candidates.append(os.path.join(share, "config", "rtfg_display.rviz"))
            candidates.append(os.path.join(share, "rviz", "rtfg_view.rviz"))
        except Exception:
            pass

        # Workspace install
        ws = os.environ.get("COLCON_PREFIX_PATH", "").split(":")[0] if "COLCON_PREFIX_PATH" in os.environ else ""
        if ws:
            candidates.append(os.path.join(ws, "share", "assembly_rtfg_cpp", "rviz", "rtfg_view.rviz"))

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

    # ------------------------------------------------------------------
    # Step 3: 移动到起始点
    # ------------------------------------------------------------------

    def _on_step3(self):
        self.btn_move.setEnabled(False)
        self._set_step_active(2)
        self.status_label.setText("正在移动到起始点...")
        self._log("③ 移动到起始点 — 开始")

        self._run_async("移动到起始点", self._ros_client.move_to_start)

    def _on_step3_done(self, result):
        if result.get("success"):
            self._log("✅ 已移动到起始点")
            self._complete_step(2)
            self.btn_fit.setEnabled(True)
            self.btn_fit.setFocus()
            self.status_label.setText("已就位 — 点击 ④ 计算轨迹 (~5 分钟)")
        else:
            self._step_failed(2, result.get("message", "未知错误"))
            self.btn_move.setEnabled(True)

    # ------------------------------------------------------------------
    # Step 4: 计算轨迹
    # ------------------------------------------------------------------

    def _on_step4(self):
        self.btn_fit.setEnabled(False)
        self._set_step_active(3)
        self.status_label.setText("正在计算轨迹 (约 5 分钟, 请耐心等待)...")
        self._log("④ 计算轨迹 — 开始 (使用环境验证加载的配置参数)")
        params = self._config_params
        self._log(f"   left_wall_offset={params.get('left_wall_offset', 0):.4f}, "
                  f"pose_x={params.get('pose_x', 0):.3f}, "
                  f"yaw={params.get('yaw_deg', 0):.1f}°, "
                  f"theta={params.get('theta_deg', 0):.1f}°")

        self.progress.setVisible(True)
        self.progress.setRange(0, 0)  # indeterminate
        self._start_time = time.time()

        # Start a timer to update elapsed time
        self._elapsed_timer = QTimer()
        self._elapsed_timer.timeout.connect(self._update_elapsed)
        self._elapsed_timer.start(2000)

        self._run_async(
            "计算轨迹",
            self._ros_client.fit_preview,
            self._config_params,
        )

    def _update_elapsed(self):
        if self._start_time:
            elapsed = time.time() - self._start_time
            self.status_label.setText(
                f"正在计算轨迹... 已耗时 {elapsed:.0f} 秒 (约 5 分钟)")

    def _on_step4_done(self, result):
        self.progress.setVisible(False)
        if hasattr(self, '_elapsed_timer'):
            self._elapsed_timer.stop()

        elapsed = time.time() - self._start_time if self._start_time else 0

        anchor_count = result.get('anchor_count', 0)
        if anchor_count > 0:
            # Trajectory was solved (even if collision audit found warnings).
            # The solver node always caches the trajectory, so execute_cached will work.
            self._fit_success = True
            has_warnings = not result.get("success", False)
            prefix = "⚠️" if has_warnings else "✅"
            self._log(f"{prefix} 轨迹计算完成! (耗时 {elapsed:.0f} 秒)")
            self._log(f"   锚点数: {anchor_count}")
            self._log(f"   播放点数: {result.get('playback_count', '?')}")
            self._log(f"   IK 耗时: {result.get('timing_ik_total_s', '?')} s")
            self._log(f"   碰撞检测耗时: {result.get('timing_collision_total_s', '?')} s")
            self._log(f"   最小 clearance: {result.get('min_tool_basin_clearance', '?')} m")

            if has_warnings:
                self._log(f"   ⚠️ 碰撞审计发现违规, 请查看上方碰撞报告后决定是否执行")
                self._log(f"   碰撞点数: {len(result.get('collision_objects', []))}")
                collision_types = set(result.get('collision_types', []))
                self._log(f"   碰撞类型: {', '.join(sorted(collision_types)) if collision_types else 'none'}")

            self._complete_step(3)
            self.btn_exec.setEnabled(True)
            self.btn_exec.setFocus()
            self.status_label.setText(
                "⚠️ 轨迹完成 (有碰撞警告) — 点击 ⑤ 开始运动" if has_warnings
                else "轨迹计算完成 — 点击 ⑤ 开始运动")
        else:
            self._fit_success = False
            self._log(f"❌ 轨迹计算失败 (耗时 {elapsed:.0f} 秒): {result.get('message', '未知错误')}")
            self._step_failed(3, result.get("message", "未知错误"))
            self.btn_fit.setEnabled(True)

    # ------------------------------------------------------------------
    # Step 5: 开始运动
    # ------------------------------------------------------------------

    def _on_step5(self):
        self.btn_exec.setEnabled(False)
        self._set_step_active(4)
        self.status_label.setText("正在执行轨迹...")
        self._log("⑤ 开始运动 — 执行缓存轨迹")

        self._run_async("执行轨迹", self._ros_client.execute_cached)

    def _on_step5_done(self, result):
        if result.get("success"):
            self._log("✅ 运动指令已发送!")
            self._complete_step(4)
            self.status_label.setText("✅ 全部完成! 5 步流程执行完毕")
        else:
            self._log(f"❌ 执行失败: {result.get('message', '未知错误')}")
            self._step_failed(4, result.get("message", "未知错误"))
            self.btn_exec.setEnabled(True)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _run_async(self, task_name, task_fn, *args, **kwargs):
        """Run a task in background thread."""
        self._worker = ServiceWorker(task_name, task_fn, *args, **kwargs)
        self._worker.progress.connect(self._log)

        # Connect finished signal based on task name
        handlers = {
            "环境验证": self._on_step1_done,
            "移动到起始点": self._on_step3_done,
            "计算轨迹": self._on_step4_done,
            "执行轨迹": self._on_step5_done,
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
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.log.append(f"[{ts}] {text}")
        # Auto-scroll to bottom
        cursor = self.log.textCursor()
        cursor.movePosition(cursor.End)
        self.log.setTextCursor(cursor)

    def _set_step_active(self, step):
        """Highlight step indicator."""
        for i, lbl in enumerate(self._step_labels):
            if i == step:
                lbl.setStyleSheet(
                    "color: #58a6ff; font-size: 11px; padding: 4px 6px; "
                    "background-color: #1f6feb22; border: 1px solid #1f6feb; border-radius: 4px;"
                )
            elif i < self._current_step:
                lbl.setStyleSheet(
                    "color: #3fb950; font-size: 11px; padding: 4px 6px; "
                    "background-color: #23863622; border: 1px solid #238636; border-radius: 4px;"
                )
            else:
                lbl.setStyleSheet(
                    "color: #484f58; font-size: 11px; padding: 4px 6px; "
                    "border: 1px solid #21262d; border-radius: 4px;"
                )

    def _complete_step(self, step):
        self._current_step = step + 1
        self._set_step_active(step)

    def _step_failed(self, step, msg):
        self._log(f"❌ 步骤 {step + 1} 失败: {msg}")
        self.status_label.setText(f"步骤 {step + 1} 失败 — {msg}")
        self._set_step_active(step)
        # Highlight failed step in red
        if 0 <= step < len(self._step_labels):
            self._step_labels[step].setStyleSheet(
                "color: #f85149; font-size: 11px; padding: 4px 6px; "
                "background-color: #f8514922; border: 1px solid #f85149; border-radius: 4px;"
            )

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event):
        self._log("正在关闭...")
        self._kill_rviz2()
        if self._ros_client is not None:
            try:
                self._ros_client.destroy()
            except Exception:
                pass
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _source_ros2():
    """Source ROS2 setup scripts and restart this process with the correct env.

    Key insight: on Linux, glibc's dynamic linker reads LD_LIBRARY_PATH at
    PROCESS START and caches it for dlopen. Setting os.environ['LD_LIBRARY_PATH']
    after Python has started does NOT affect subsequent dlopen calls (including
    those made by Python's import mechanism when loading C extensions).

    The only reliable fix is to restart this Python process with the full
    environment that 'source setup.bash' would provide.  We use os.execve()
    to re-exec the current script with the sourced environment.
    """
    # Guard: if ROS_DISTRO is already set AND /opt/ros/humble/lib is on
    # LD_LIBRARY_PATH, we're already in a sourced environment — skip.
    if os.environ.get("ROS_DISTRO") == ROS_DISTRO:
        ld = os.environ.get("LD_LIBRARY_PATH", "")
        if f"/opt/ros/{ROS_DISTRO}/lib" in ld:
            return  # environment already correct

    ros_setup = f"/opt/ros/{ROS_DISTRO}/setup.bash"
    workspace_setup = os.path.join(_PROJECT_ROOT, "install", "setup.bash")

    if not os.path.isfile(ros_setup):
        print(f"[WARN] ROS2 setup not found: {ros_setup}")
        return

    # Build the source chain and dump the resulting environment
    cmd = f"source {ros_setup}"
    if os.path.isfile(workspace_setup):
        cmd += f" && source {workspace_setup}"
    cmd += " && env -0"  # null-delimited for safety

    try:
        raw = subprocess.check_output(
            ["bash", "-c", cmd],
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except Exception as e:
        print(f"[WARN] Failed to source ROS2: {e}")
        return

    # Parse all env vars into a dict
    new_env = {}
    for entry in raw.split(b"\x00"):
        entry = entry.decode("utf-8", errors="replace")
        if "=" not in entry:
            continue
        k, v = entry.split("=", 1)
        new_env[k] = v

    # Preserve USER and HOME (may be missing in stripped environments)
    for key in ("HOME", "USER"):
        if key not in new_env and key in os.environ:
            new_env[key] = os.environ[key]

    print(f"[INFO] Restarting with ROS2 {ROS_DISTRO} environment ...")
    print(f"[INFO]   LD_LIBRARY_PATH includes /opt/ros/{ROS_DISTRO}/lib")
    if os.path.isfile(workspace_setup):
        print(f"[INFO]   Workspace overlay: {_PROJECT_ROOT}")

    # Restart this Python process with the new environment.
    # os.execve replaces the current process without forking.
    os.execve(sys.executable, [sys.executable] + sys.argv, new_env)
    # (Never reaches here on success)


def main():
    global SIMULATION_MODE

    # Parse command-line arguments
    if "--real" in sys.argv:
        SIMULATION_MODE = False
        print("[INFO] 真实硬件模式 (--real): move_to_start 和 execute_cached 将实际控制机械臂")
    elif "--sim" in sys.argv:
        SIMULATION_MODE = True
        print("[INFO] 仿真模式 (--sim): move_to_start 和 execute_cached 将跳过实际运动")
    else:
        print("[INFO] 默认仿真模式: 用 --real 切换到真实硬件模式")

    # Always source ROS2 to ensure rclpy and friends are importable,
    # even when running from IDE or desktop without terminal sourcing.
    _source_ros2()

    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_PALETTE)
    app.setApplicationName(APP_NAME)

    window = RtfgControlPanel()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
