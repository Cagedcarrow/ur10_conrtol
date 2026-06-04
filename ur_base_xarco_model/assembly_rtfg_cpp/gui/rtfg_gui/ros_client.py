"""ROS2 service call wrapper for assembly_rtfg_cuda solver services.

Provides synchronous wrappers for the three main solver services
(load_config, fit_preview, execute_cached) plus a move_to_start method.

Supports --sim mode where robot-dependent steps (move_to_start, execute_cached)
are mocked so the full 5-step GUI flow can be tested without hardware.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from assembly_rtfg_cuda.srv import LoadConfig, FitPreview, ExecuteCached
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, Pose
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory


_JOINT_NAMES = [
    "ur10_shoulder_pan", "ur10_shoulder_lift", "ur10_elbow",
    "ur10_wrist_1", "ur10_wrist_2", "ur10_wrist_3"]


def _joint_trajectory_to_dict(jt_msg):
    """Convert a JointTrajectory message to a JSON-serializable dict."""
    points_out = []
    for pt in jt_msg.points:
        points_out.append({
            "positions": list(pt.positions),
            "velocities": list(pt.velocities) if pt.velocities else [],
            "accelerations": list(pt.accelerations) if pt.accelerations else [],
            "effort": list(pt.effort) if pt.effort else [],
            "time_from_start": {
                "sec": pt.time_from_start.sec,
                "nanosec": pt.time_from_start.nanosec,
            },
        })
    return {
        "joint_names": list(jt_msg.joint_names),
        "points": points_out,
    }


def _pose_array_to_dict(pa_msg):
    """Convert a PoseArray message to a JSON-serializable dict."""
    poses_out = []
    for p in pa_msg.poses:
        poses_out.append({
            "position": {
                "x": p.position.x,
                "y": p.position.y,
                "z": p.position.z,
            },
            "orientation": {
                "x": p.orientation.x,
                "y": p.orientation.y,
                "z": p.orientation.z,
                "w": p.orientation.w,
            },
        })
    return {
        "header": {
            "frame_id": pa_msg.header.frame_id,
            "stamp": {
                "sec": pa_msg.header.stamp.sec,
                "nanosec": pa_msg.header.stamp.nanosec,
            },
        },
        "poses": poses_out,
    }


class RosClient:
    """ROS2 service client for rtfg_solver_node services.

    Creates a background-spinning ROS2 node and wraps the three solver
    services (load_config, fit_preview, execute_cached) with timeout
    and error handling.

    Parameters
    ----------
    simulation_mode : bool
        If True, mock move_to_start and execute_cached so the full
        5-step flow completes without real robot hardware.
    """

    def __init__(self, simulation_mode=False):
        self.simulation_mode = simulation_mode

        if not rclpy.ok():
            rclpy.init()
        self._node = Node("rtfg_gui_client")

        # Service clients
        self._load_cli = self._node.create_client(LoadConfig, "/rtfg/load_config")
        self._fit_cli = self._node.create_client(FitPreview, "/rtfg/fit_preview")
        self._execute_cli = self._node.create_client(
            ExecuteCached, "/rtfg/execute_cached"
        )

        # Action client for move_to_start (lazily initialised)
        self._traj_action_cli = None

        self._clis = [self._load_cli, self._fit_cli, self._execute_cli]
        self._logger = self._node.get_logger()

    # ------------------------------------------------------------------
    # Service helpers
    # ------------------------------------------------------------------

    def _get_traj_action_client(self):
        """Return (creating if needed) the FollowJointTrajectory action client."""
        if self._traj_action_cli is None:
            self._traj_action_cli = ActionClient(
                self._node,
                FollowJointTrajectory,
                "/joint_trajectory_controller/follow_joint_trajectory")
        return self._traj_action_cli

    def _wait_for_service(self, cli, timeout_s=5.0):
        """Block until the given service is available, or raise on timeout."""
        if not cli.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(
                f"Service {cli.srv_name} not available after {timeout_s}s"
            )

    def _call_service(self, cli, request, timeout_s=30.0):
        """Call a service synchronously with timeout.

        Returns the response, or raises on failure.
        """
        self._wait_for_service(cli, timeout_s=min(timeout_s, 5.0))
        future = cli.call_async(request)
        # Note: rclpy Humble spin_until_future_complete may return None;
        # always check future.result() to determine success.
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_s
        )
        resp = future.result()
        if resp is None:
            raise TimeoutError(
                f"Service call {cli.srv_name} timed out after {timeout_s}s"
            )
        if future.exception() is not None:
            raise RuntimeError(
                f"Service call {cli.srv_name} failed: {future.exception()}"
            )
        return resp

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def load_config(self, yaml_path=""):
        """Call /rtfg/load_config and return all parameters as a dict.

        Returns
        -------
        dict with keys: success, message, left_wall_offset, mud_height,
        approach_len, theta_deg, depth, x_plane, pose_x, pose_y, pose_z,
        roll_deg, pitch_deg, yaw_deg, initial_q.
        """
        req = LoadConfig.Request()
        req.yaml_path = yaml_path
        resp = self._call_service(self._load_cli, req, timeout_s=10.0)
        return {
            "success": resp.success,
            "message": resp.message,
            "left_wall_offset": resp.left_wall_offset,
            "mud_height": resp.mud_height,
            "approach_len": resp.approach_len,
            "theta_deg": resp.theta_deg,
            "depth": resp.depth,
            "x_plane": resp.x_plane,
            "pose_x": resp.pose_x,
            "pose_y": resp.pose_y,
            "pose_z": resp.pose_z,
            "roll_deg": resp.roll_deg,
            "pitch_deg": resp.pitch_deg,
            "yaw_deg": resp.yaw_deg,
            "initial_q": list(resp.initial_q),
        }

    def fit_preview(self, params, timeout_s=600.0):
        """Call /rtfg/fit_preview with a params dict.

        Parameters
        ----------
        params : dict
            Expected keys: left_wall_offset, mud_height, approach_len,
            theta_deg, depth, x_plane, pose_x, pose_y, pose_z,
            roll_deg, pitch_deg, yaw_deg, current_q, clearance_threshold.
        timeout_s : float
            Service call timeout in seconds (default 600 s, ~10 min for full solve).

        Returns
        -------
        dict with keys: success, message, trajectory, timing_*, anchor_count, ...
        """
        req = FitPreview.Request()
        req.left_wall_offset = params.get("left_wall_offset", 0.0)
        req.mud_height = params.get("mud_height", 0.0)
        req.approach_len = params.get("approach_len", 0.0)
        req.theta_deg = params.get("theta_deg", 0.0)
        req.depth = params.get("depth", 0.0)
        req.x_plane = params.get("x_plane", 0.0)
        req.pose_x = params.get("pose_x", 0.0)
        req.pose_y = params.get("pose_y", 0.0)
        req.pose_z = params.get("pose_z", 0.0)
        req.roll_deg = params.get("roll_deg", 0.0)
        req.pitch_deg = params.get("pitch_deg", 0.0)
        req.yaw_deg = params.get("yaw_deg", 0.0)
        req.current_q = params.get("current_q", [])
        req.clearance_threshold = params.get("clearance_threshold", 0.002)

        resp = self._call_service(self._fit_cli, req, timeout_s=timeout_s)

        return {
            "success": resp.success,
            "message": resp.message,
            "trajectory": _joint_trajectory_to_dict(resp.trajectory),
            "target_tcp_poses": _pose_array_to_dict(resp.target_tcp_poses),
            "actual_tcp_poses": _pose_array_to_dict(resp.actual_tcp_poses),
            "collision_points_xyz": list(resp.collision_points_xyz),
            "collision_types": list(resp.collision_types),
            "collision_objects": list(resp.collision_objects),
            "collision_segments": list(resp.collision_segments),
            "min_self_clearance": resp.min_self_clearance,
            "min_tool_body_clearance": resp.min_tool_body_clearance,
            "min_tool_basin_clearance": resp.min_tool_basin_clearance,
            "min_self_object": resp.min_self_object,
            "min_tool_body_object": resp.min_tool_body_object,
            "min_tool_basin_object": resp.min_tool_basin_object,
            "anchor_count": resp.anchor_count,
            "playback_count": resp.playback_count,
            "max_target_rotation_delta_deg": resp.max_target_rotation_delta_deg,
            "max_actual_rotation_delta_deg": resp.max_actual_rotation_delta_deg,
            "max_anchor_joint_step_deg": resp.max_anchor_joint_step_deg,
            "max_playback_joint_step_deg": resp.max_playback_joint_step_deg,
            "timing_total_wall_s": resp.timing_total_wall_s,
            "timing_ik_total_s": resp.timing_ik_total_s,
            "timing_collision_total_s": resp.timing_collision_total_s,
            "timing_avg_per_pose_s": resp.timing_avg_per_pose_s,
        }

    def execute_cached(self):
        """Call /rtfg/execute_cached to execute the last cached trajectory.

        In simulation mode, the solver node's onExecuteCached will detect
        that no real robot controller is available and fall back to publishing
        /joint_states, which animates the arm in RViz2 via robot_state_publisher.

        Returns
        -------
        dict with keys: success, message.
        """
        if self.simulation_mode:
            self._logger.info(
                "[SIM] execute_cached — 仿真模式: solver节点将使用 "
                "joint_state回放模式在RViz2中显示运动")

        req = ExecuteCached.Request()
        req.execute = True
        resp = self._call_service(self._execute_cli, req, timeout_s=10.0)
        return {"success": resp.success, "message": resp.message}

    def move_to_start(self):
        """Move robot to the first trajectory target (入泥点/approach start).

        Calls /rtfg/move_to_start service. The solver node solves IK for the
        first target pose (matching MATLAB's "移动到轨迹起始点" behavior) and
        publishes /joint_states for smooth animation in RViz2.

        Returns
        -------
        dict with keys: success, message.
        """
        self._logger.info("move_to_start: moving to trajectory start (入泥点)")
        try:
            from std_srvs.srv import Trigger
            cli = self._node.create_client(Trigger, "/rtfg/move_to_start")
            if not cli.wait_for_service(timeout_sec=5.0):
                return {"success": False,
                        "message": "move_to_start service not available"}
            req = Trigger.Request()
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=30.0)
            resp = future.result()
            if resp is not None and resp.success:
                return {"success": True, "message": resp.message}
            return {"success": False,
                    "message": resp.message if resp else "no response"}
        except Exception as e:
            self._logger.warn(f"move_to_start failed: {e}")
            return {"success": False, "message": str(e)}

    def move_to_home(self):
        """Move robot back to the URDF initial pose (初始姿态).

        Calls /rtfg/move_to_home service. The solver node moves to
        initial_q (loaded from URDF) via joint-state playback.

        Returns
        -------
        dict with keys: success, message.
        """
        self._logger.info("move_to_home: returning to initial URDF pose")
        try:
            from std_srvs.srv import Trigger
            cli = self._node.create_client(Trigger, "/rtfg/move_to_home")
            if not cli.wait_for_service(timeout_sec=5.0):
                return {"success": False,
                        "message": "move_to_home service not available"}
            req = Trigger.Request()
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
            resp = future.result()
            if resp is not None and resp.success:
                return {"success": True, "message": resp.message}
            return {"success": False,
                    "message": resp.message if resp else "no response"}
        except Exception as e:
            self._logger.warn(f"move_to_home failed: {e}")
            return {"success": False, "message": str(e)}

    def get_topics(self):
        """Return a list of /rtfg/ topic names known to the node."""
        topics = []
        for name, _ in self._node.get_topic_names_and_types():
            if name.startswith("/rtfg/"):
                topics.append(name)
        return sorted(topics)

    def destroy(self):
        """Cleanly shut down the ROS2 node."""
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
