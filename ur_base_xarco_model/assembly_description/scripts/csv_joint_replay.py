#!/usr/bin/env python3
import csv
import math
import time
from bisect import bisect_right
from typing import List, Optional, Tuple

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class CsvJointReplay(Node):
    def __init__(self):
        super().__init__('csv_joint_replay')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('publish_topic', '/joint_states')
        self.declare_parameter('time_column', 'Time')
        self.declare_parameter('publish_hz', 125.0)
        self.declare_parameter('output_mode', 'trajectory_action')
        self.declare_parameter('follow_joint_trajectory_action', '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('wait_action_timeout_sec', 30.0)
        self.declare_parameter('send_full_trajectory', True)
        self.declare_parameter('start_delay_sec', 0.2)

        self._joint_names = [
            'ur10_shoulder_pan',
            'ur10_shoulder_lift',
            'ur10_elbow',
            'ur10_wrist_1',
            'ur10_wrist_2',
            'ur10_wrist_3',
        ]
        self._joint_columns = ['Act_q0', 'Act_q1', 'Act_q2', 'Act_q3', 'Act_q4', 'Act_q5']
        self._joint_velocity_columns = ['Act_qd0', 'Act_qd1', 'Act_qd2', 'Act_qd3', 'Act_qd4', 'Act_qd5']

        csv_path = str(self.get_parameter('csv_path').value)
        if not csv_path:
            raise RuntimeError('Parameter "csv_path" is empty.')

        self._speed_scale = max(float(self.get_parameter('speed_scale').value), 1e-6)
        self._loop = bool(self.get_parameter('loop').value)
        self._publish_topic = str(self.get_parameter('publish_topic').value)
        self._time_column = str(self.get_parameter('time_column').value)
        self._publish_hz = max(float(self.get_parameter('publish_hz').value), 1.0)
        self._publish_period = 1.0 / self._publish_hz
        self._output_mode = str(self.get_parameter('output_mode').value).strip().lower()
        self._traj_action = str(self.get_parameter('follow_joint_trajectory_action').value)
        self._wait_action_timeout_sec = max(float(self.get_parameter('wait_action_timeout_sec').value), 0.1)
        self._send_full_trajectory = bool(self.get_parameter('send_full_trajectory').value)
        self._start_delay_sec = max(float(self.get_parameter('start_delay_sec').value), 0.0)
        self._joint_map = {}

        self._traj = self._load_csv(csv_path, self._time_column)
        if not self._traj:
            raise RuntimeError(f'No valid rows loaded from: {csv_path}')

        self._duration = self._traj[-1][0]
        self._times = [p[0] for p in self._traj]
        self._finished = False
        self._goal_active = False
        self._pending_retry: Optional[float] = None

        if self._output_mode == 'trajectory_action':
            self._action_client = ActionClient(self, FollowJointTrajectory, self._traj_action)
            self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
            self._wait_and_send_trajectory()
            self.create_timer(0.5, self._action_monitor_tick)
        elif self._output_mode == 'joint_states':
            self._pub = self.create_publisher(JointState, self._publish_topic, 10)
            self._start_mono = time.monotonic()
            self.create_timer(self._publish_period, self._tick)
        else:
            raise RuntimeError(f'Unsupported output_mode="{self._output_mode}", expected "trajectory_action" or "joint_states".')

        self.get_logger().info(
            f'Loaded {len(self._traj)} points from "{csv_path}", '
            f'speed_scale={self._speed_scale}, loop={self._loop}, mode={self._output_mode}'
        )

    def _load_csv(self, csv_path: str, time_column: str) -> List[Tuple[float, List[float], List[float]]]:
        points: List[Tuple[float, List[float], List[float]]] = []
        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            has_velocity = all(c in (reader.fieldnames or []) for c in self._joint_velocity_columns)
            required = [time_column] + self._joint_columns
            missing = [c for c in required if c not in (reader.fieldnames or [])]
            if missing:
                raise RuntimeError(f'Missing required CSV columns: {missing}')

            for row_idx, row in enumerate(reader, start=2):
                try:
                    t = float(row[time_column])
                    q = [float(row[c]) for c in self._joint_columns]
                    qd = [float(row[c]) for c in self._joint_velocity_columns] if has_velocity else []
                    if not math.isfinite(t) or any(not math.isfinite(v) for v in q) or any(not math.isfinite(v) for v in qd):
                        continue
                    points.append((t, q, qd))
                except ValueError:
                    self.get_logger().warn(f'Skip invalid row {row_idx}')
                    continue

        points.sort(key=lambda x: x[0])
        if points:
            t0 = points[0][0]
            points = [(t - t0, q, qd) for t, q, qd in points]
            # Ensure strictly increasing time for trajectory controllers.
            min_dt = 1e-4
            normalized: List[Tuple[float, List[float], List[float]]] = [points[0]]
            last_t = points[0][0]
            for t, q, qd in points[1:]:
                if t <= last_t:
                    t = last_t + min_dt
                normalized.append((t, q, qd))
                last_t = t
            points = normalized
        return points

    def _interpolate_state(self, t: float) -> Tuple[List[float], List[float]]:
        if t <= self._times[0]:
            return list(self._traj[0][1]), list(self._traj[0][2])
        if t >= self._times[-1]:
            return list(self._traj[-1][1]), list(self._traj[-1][2])

        right = bisect_right(self._times, t)
        i0 = max(0, right - 1)
        i1 = min(len(self._traj) - 1, right)
        t0, q0, qd0 = self._traj[i0]
        t1, q1, qd1 = self._traj[i1]
        dt = max(t1 - t0, 1e-9)
        alpha = (t - t0) / dt
        q = [a + alpha * (b - a) for a, b in zip(q0, q1)]
        qd = [a + alpha * (b - a) for a, b in zip(qd0, qd1)]
        return q, qd

    def _publish_state(self, q: List[float], qd: List[float]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = q
        msg.velocity = qd
        self._pub.publish(msg)

    def _to_duration(self, seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_map[name] = float(pos)

    def _build_goal(self) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._joint_names)
        if not self._send_full_trajectory:
            self.get_logger().warn(
                'send_full_trajectory=false is not supported in v1; falling back to full trajectory.'
            )
        source = self._traj
        if all(name in self._joint_map for name in self._joint_names):
            pt0 = JointTrajectoryPoint()
            pt0.positions = [self._joint_map[name] for name in self._joint_names]
            pt0.time_from_start = self._to_duration(0.0)
            goal.trajectory.points.append(pt0)
        else:
            self.get_logger().warn(
                'Current /joint_states not complete yet; sending trajectory without warm-start point.'
            )
        for t, q, qd in source:
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            if qd:
                pt.velocities = list(qd)
            pt.time_from_start = self._to_duration(self._start_delay_sec + (t / self._speed_scale))
            goal.trajectory.points.append(pt)
        return goal

    def _wait_and_send_trajectory(self):
        if not self._action_client.wait_for_server(timeout_sec=self._wait_action_timeout_sec):
            self.get_logger().error(
                f'Action server unavailable: {self._traj_action}. '
                f'Will retry periodically.'
            )
            self._pending_retry = time.monotonic() + 2.0
            return
        self._send_trajectory()

    def _send_trajectory(self):
        if self._goal_active:
            return
        goal = self._build_goal()
        self.get_logger().info(
            f'Sending trajectory with {len(goal.trajectory.points)} points to {self._traj_action}'
        )
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        self._goal_active = True

    def _on_goal_response(self, future):
        self._goal_active = False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected.')
            self._pending_retry = time.monotonic() + 2.0
            return
        self.get_logger().info('Trajectory goal accepted.')
        self._goal_active = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_active = False
        result = future.result()
        if result is None:
            self.get_logger().error('Trajectory result is None.')
            self._pending_retry = time.monotonic() + 2.0
            return
        error_code = int(result.result.error_code)
        if error_code == 0:
            self.get_logger().info('Trajectory replay finished.')
            if self._loop:
                self._pending_retry = time.monotonic() + 0.1
            else:
                self._finished = True
            return
        self.get_logger().error(
            f'Trajectory execution failed with error_code={error_code}.'
        )
        self._pending_retry = time.monotonic() + 2.0

    def _action_monitor_tick(self):
        if self._finished:
            raise SystemExit
        if self._pending_retry is not None and time.monotonic() >= self._pending_retry:
            self._pending_retry = None
            self._wait_and_send_trajectory()

    def _tick(self):
        elapsed = (time.monotonic() - self._start_mono) * self._speed_scale
        if self._loop and self._duration > 0.0:
            replay_t = elapsed % self._duration
        else:
            replay_t = min(elapsed, self._duration)

        q, qd = self._interpolate_state(replay_t)
        self._publish_state(q, qd)

        if not self._loop and elapsed >= self._duration:
            self.get_logger().info('Replay finished.')
            raise SystemExit


def main():
    rclpy.init()
    node = CsvJointReplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
