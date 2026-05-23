#!/usr/bin/env bash
set -u

PATTERNS=(
  "ros2 launch ur10_real_control_ros2 ur10_real_control.launch.py"
  "ros2 launch ur10_real_control_ros2 ur_driver_only.launch.py"
  "ros2 launch ur10_real_control_ros2 real_control_gui.launch.py"
  "ur10_real_control_gui"
  "ur_ros2_control_node"
  "dashboard_client"
  "controller_stopper"
  "robot_state_helper"
  "urscript_interface"
  "trajectory_until_node"
  "move_group"
  "rviz2"
  "robot_state_publisher"
)

echo "=== CLEANUP START ==="
for pattern in "${PATTERNS[@]}"; do
  hits="$(pgrep -a -f "${pattern}" || true)"
  if [ -n "${hits}" ]; then
    echo "[MATCH] ${pattern}"
    echo "${hits}"
    pkill -TERM -f "${pattern}" || true
  fi
done

sleep 2

for pattern in "${PATTERNS[@]}"; do
  hits="$(pgrep -a -f "${pattern}" || true)"
  if [ -n "${hits}" ]; then
    echo "[KILL] ${pattern}"
    echo "${hits}"
    pkill -KILL -f "${pattern}" || true
  fi
done

echo "=== CLEANUP END ==="
