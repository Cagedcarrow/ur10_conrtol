#!/usr/bin/env bash
set -u

ROBOT_IP="${1:-${ROBOT_IP:-10.160.9.21}}"

strip_ansi() {
  sed -r 's/\x1B\[[0-9;]*[mK]//g'
}

echo "=== UR10 REAL READY CHECK START ==="
echo "ROBOT_IP=${ROBOT_IP}"

if timeout 3s ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
  echo "ROBOT_PING_OK"
else
  echo "ROBOT_PING_FAIL"
fi

echo "=== PORT LISTENERS 50001-50004 ==="
ss -lntp | grep -E ':5000(1|2|3|4)' || echo "NO_5000X_LISTENER"

echo "=== JOINT STATES ==="
ros2 topic info /joint_states -v || true
if JS_ONCE="$(timeout 5s ros2 topic echo /joint_states --once 2>/dev/null)"; then
  echo "${JS_ONCE}"
  echo "JOINT_STATES_OK"
else
  echo "JOINT_STATES_MISSING"
fi

NODE_LIST="$(timeout 5s ros2 node list 2>/dev/null || true)"
CTRL_LIST="$(timeout 5s ros2 control list_controllers 2>/dev/null || true)"
CTRL_LIST_CLEAN="$(printf '%s\n' "${CTRL_LIST}" | strip_ansi)"
ACTION_LIST="$(timeout 5s ros2 action list 2>/dev/null || true)"
SPEED_TOPIC="$(timeout 5s ros2 topic list 2>/dev/null | grep -E '/speed_scaling_state_broadcaster/speed_scaling$|/speed_scaling$' | head -1 || true)"

echo "=== ROS NODES ==="
echo "${NODE_LIST}"
echo "=== CONTROLLERS ==="
echo "${CTRL_LIST}"
echo "=== ACTIONS ==="
echo "${ACTION_LIST}" | grep follow_joint_trajectory || true

if echo "${CTRL_LIST_CLEAN}" | grep -Eq '^scaled_joint_trajectory_controller[[:space:]].*[[:space:]]active[[:space:]]*$'; then
  echo "SCALED_CONTROLLER_ACTIVE"
else
  echo "SCALED_CONTROLLER_NOT_ACTIVE"
fi

if echo "${ACTION_LIST}" | grep -qx '/scaled_joint_trajectory_controller/follow_joint_trajectory'; then
  echo "EXEC_ACTION_ONLINE"
else
  echo "EXEC_ACTION_MISSING"
fi

if [ -n "${SPEED_TOPIC}" ]; then
  echo "SPEED_SCALING_TOPIC=${SPEED_TOPIC}"
  SPEED_MSG="$(timeout 5s ros2 topic echo "${SPEED_TOPIC}" --once 2>/dev/null || true)"
  echo "${SPEED_MSG}"
  SPEED_VALUE="$(printf '%s\n' "${SPEED_MSG}" | awk '/data:/ {print $2; exit}')"
  if awk "BEGIN {exit !(${SPEED_VALUE:-0} > 0.01)}"; then
    echo "SPEED_SCALING_NONZERO"
  else
    echo "SPEED_SCALING_ZERO_OR_MISSING"
  fi
else
  echo "SPEED_SCALING_TOPIC_MISSING"
fi

echo "=== POSSIBLE RTDE CONFLICT PROCESSES ==="
pgrep -a -f 'ur_rtde|rtde_control|rtde_receive|ExternalControl|external_control|urx|RoboDK|python.*rtde' || echo "NO_EXTRA_RTDE_CLIENT"

echo "=== RECENT RTDE OVERFLOW CHECK ==="
RTDE_HITS="$(
  find "${HOME}/.ros/log" -maxdepth 3 -type f -name '*.log' -mmin -30 -print0 2>/dev/null \
    | xargs -0 -r grep -IhE 'Pipeline producer overflowed|RTDE Data Pipeline|RTDE.*overflowed|overflowed.*RTDE' 2>/dev/null \
    | tail -n 10
)"
if [ -n "${RTDE_HITS}" ]; then
  echo "${RTDE_HITS}"
  echo "RTDE_OVERFLOW"
else
  echo "RTDE_OK"
fi

echo "=== UR10 REAL READY CHECK END ==="
