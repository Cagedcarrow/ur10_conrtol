#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# One-click startup script for RTFG PyQt5 Control Panel
#
# Usage:
#   chmod +x rtfg_one_click.sh
#   ./rtfg_one_click.sh [--sim|--real]
#
# Default: --sim (simulation mode, no real robot needed).
#   --sim  仿真模式 — move_to_start 和 execute_cached 跳过实际运动
#   --real 真实硬件模式 — 需要 UR10 真机 + trajectory controller 运行
#
# This script sources the ROS2 environment, starts rtfg_solver_node in the
# background, launches the PyQt5 control panel, and cleans up on exit.
# ---------------------------------------------------------------------------

set -euo pipefail

LAUNCH_FLAG="${1:---sim}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GUI_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_DIR="$(cd "$GUI_DIR/../.." && pwd)"

if [ "$LAUNCH_FLAG" = "--real" ]; then
  MODE_LABEL="真实硬件模式"
else
  MODE_LABEL="仿真模式"
fi

echo "============================================"
echo " RTFG 控制面板 — 一键启动 ($MODE_LABEL)"
echo "============================================"

# ---------------------------------------------------------------------------
# Source ROS2 environment
# ---------------------------------------------------------------------------
if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "[1/4] Sourcing ROS2 Humble..."
    source "/opt/ros/humble/setup.bash"
  elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "[1/4] Sourcing ROS2 Jazzy..."
    source "/opt/ros/jazzy/setup.bash"
  elif [ -f "/opt/ros/rolling/setup.bash" ]; then
    echo "[1/4] Sourcing ROS2 Rolling..."
    source "/opt/ros/rolling/setup.bash"
  else
    echo "ERROR: ROS2 not found under /opt/ros/. Please source your ROS2 setup manually."
    exit 1
  fi
else
  echo "[1/4] ROS2 already sourced (distro: $ROS_DISTRO)"
fi

# ---------------------------------------------------------------------------
# Source workspace overlay
# ---------------------------------------------------------------------------
if [ -f "$PROJECT_DIR/install/setup.bash" ]; then
  echo "[2/4] Sourcing workspace overlay at $PROJECT_DIR/install/setup.bash..."
  source "$PROJECT_DIR/install/setup.bash"
else
  echo "[2/4] No workspace overlay found — assuming packages are already discoverable."
fi

# ---------------------------------------------------------------------------
# Kill any existing solver node
# ---------------------------------------------------------------------------
echo "[3/4] Cleaning up old processes..."
pkill -f "rtfg_solver_node" 2>/dev/null || true
sleep 1

# ---------------------------------------------------------------------------
# Start solver node in background
# ---------------------------------------------------------------------------
echo "[3/4] Starting rtfg_solver_node in background..."
PKG_SHARE=$(ros2 pkg prefix assembly_rtfg_cpp 2>/dev/null || echo "")
if [ -n "$PKG_SHARE" ]; then
  CONFIG_PATH="$PKG_SHARE/share/assembly_rtfg_cpp/config/environment_runtime_config.yaml"
  URDF_PATH="$PKG_SHARE/share/assembly_rtfg_cpp/urdf/assembly_rtfg_solver.urdf"
else
  CONFIG_PATH="$PROJECT_DIR/src/assembly_rtfg_cpp/config/environment_runtime_config.yaml"
  URDF_PATH="$PROJECT_DIR/src/assembly_rtfg_cpp/urdf/assembly_rtfg_solver.urdf"
fi

ros2 run assembly_rtfg_cpp rtfg_solver_node \
  --ros-args \
  -p config_path:="$CONFIG_PATH" \
  -p solver_urdf_path:="$URDF_PATH" \
  -p solver_backend:=kdl \
  -p solver_mode:=full \
  -p clearance_threshold:=0.002 &

SOLVER_PID=$!
echo "      Solver PID: $SOLVER_PID"

# Wait for solver services to be ready
echo "[3/4] Waiting for solver services..."
for i in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q "/rtfg/load_config"; then
    echo "      /rtfg/load_config is ready!"
    break
  fi
  sleep 1
done

# ---------------------------------------------------------------------------
# Launch PyQt5 control panel
# ---------------------------------------------------------------------------
echo "[4/4] Launching PyQt5 control panel ($MODE_LABEL)..."
python3 "$SCRIPT_DIR/rtfg_launcher.py" "$LAUNCH_FLAG"

# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------
echo ""
echo "============================================"
echo " Shutting down..."
echo "============================================"
kill $SOLVER_PID 2>/dev/null || true
wait $SOLVER_PID 2>/dev/null || true
echo "Done."
