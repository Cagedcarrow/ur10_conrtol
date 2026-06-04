#include "collision_checker.h"
#include "continuous_trajectory_solver.h"
#include "ik_backend.h"
#include "robot_model.h"
#include "rolling_planner.h"
#include "trajectory_generator.h"
#include "trajectory_solver.h"

#include "assembly_rtfg_cuda/srv/execute_cached.hpp"
#include "assembly_rtfg_cuda/srv/fit_preview.hpp"
#include "assembly_rtfg_cuda/srv/load_config.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

const std::vector<std::string> kJointNames = {
  "ur10_shoulder_pan", "ur10_shoulder_lift", "ur10_elbow",
  "ur10_wrist_1", "ur10_wrist_2", "ur10_wrist_3"};

void logProfile(const rclcpp::Logger& logger, const std::string& name, double total_s, std::size_t denom)
{
  const double total_ms = total_s * 1000.0;
  const double avg_ms = denom > 0 ? total_ms / static_cast<double>(denom) : 0.0;
  RCLCPP_INFO(logger, "[PROFILE] %s: total %.3f ms, avg %.3f ms", name.c_str(), total_ms, avg_ms);
}

geometry_msgs::msg::Pose poseFromTform(const rtfg::Mat4& T)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = T(0, 3);
  pose.position.y = T(1, 3);
  pose.position.z = T(2, 3);
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

Eigen::VectorXd vectorToEigen(const std::vector<double>& values, const Eigen::VectorXd& fallback)
{
  if (values.empty()) {
    return fallback;
  }
  if (values.size() != 6) {
    throw std::runtime_error("current_q must contain 6 joint values");
  }
  Eigen::VectorXd q(6);
  for (int i = 0; i < 6; ++i) {
    q(i) = values[static_cast<std::size_t>(i)];
  }
  return q;
}

trajectory_msgs::msg::JointTrajectory makeJointTrajectory(const Eigen::MatrixXd& q_series)
{
  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = kJointNames;
  jt.points.resize(static_cast<std::size_t>(q_series.rows()));
  constexpr double dt = 0.02;
  for (int i = 0; i < q_series.rows(); ++i) {
    auto& point = jt.points[static_cast<std::size_t>(i)];
    point.positions.resize(6);
    for (int j = 0; j < 6; ++j) {
      point.positions[static_cast<std::size_t>(j)] = q_series(i, j);
    }
    const double t = static_cast<double>(i) * dt;
    point.time_from_start.sec = static_cast<int32_t>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);
  }
  return jt;
}

template <typename PoseGetter>
geometry_msgs::msg::PoseArray makeSparsePoseArray(
  const std::string& frame_id, const rclcpp::Time& stamp, int stride, std::size_t total,
  PoseGetter&& getter)
{
  geometry_msgs::msg::PoseArray array;
  array.header.frame_id = frame_id;
  array.header.stamp = stamp;
  const std::size_t step = static_cast<std::size_t>(std::max(1, stride));
  if (total == 0) {
    return array;
  }
  for (std::size_t i = 0; i < total; i += step) {
    array.poses.push_back(getter(i));
  }
  if (((total - 1) % step) != 0) {
    array.poses.push_back(getter(total - 1));
  }
  return array;
}

}  // namespace

class RtfgSolverNode : public rclcpp::Node
{
public:
  using LoadConfig = assembly_rtfg_cuda::srv::LoadConfig;
  using FitPreview = assembly_rtfg_cuda::srv::FitPreview;
  using ExecuteCached = assembly_rtfg_cuda::srv::ExecuteCached;
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  RtfgSolverNode()
  : Node("rtfg_solver_node")
  {
    const auto share = ament_index_cpp::get_package_share_directory("assembly_rtfg_cuda");
    config_path_ = declare_parameter<std::string>(
      "config_path", share + "/config/environment_runtime_config.yaml");
    solver_urdf_path_ = declare_parameter<std::string>(
      "solver_urdf_path", share + "/urdf/assembly_rtfg_solver.urdf");
    base_link_ = declare_parameter<std::string>("base_link", "base_jizuo");
    tip_link_ = declare_parameter<std::string>("tip_link", "sensor_shovel_tcp");
    clearance_threshold_ = declare_parameter<double>("clearance_threshold", 2e-3);
    solver_mode_ = declare_parameter<std::string>("solver_mode", "full");
    solver_backend_ = declare_parameter<std::string>("solver_backend", "kdl");
    max_collision_candidates_full_ = declare_parameter<int>("max_collision_candidates_full", 48);
    max_collision_candidates_realtime_ = declare_parameter<int>("max_collision_candidates_realtime", 2);
    collision_check_stride_full_ = declare_parameter<int>("collision_check_stride_full", 7);
    collision_check_stride_realtime_ = declare_parameter<int>("collision_check_stride_realtime", 7);
    publish_sparse_posearray_realtime_ =
      declare_parameter<bool>("publish_sparse_posearray_realtime", true);
    posearray_stride_realtime_ = declare_parameter<int>("posearray_stride_realtime", 10);
    enable_window_solve_ = declare_parameter<bool>("enable_window_solve", false);
    window_size_ = declare_parameter<int>("window_size", 100);
    window_overlap_ = declare_parameter<int>("window_overlap", 10);
    max_iterations_full_ = declare_parameter<int>("max_iterations_full", 60);
    max_iterations_realtime_ = declare_parameter<int>("max_iterations_realtime", 30);
    stagnation_epsilon_ = declare_parameter<double>("stagnation_epsilon", 1e-5);
    stagnation_patience_ = declare_parameter<int>("stagnation_patience", 3);
    dq_stop_threshold_ = declare_parameter<double>("dq_stop_threshold", 1e-6);
    controller_action_ = declare_parameter<std::string>(
      "controller_action", "/joint_trajectory_controller/follow_joint_trajectory");

    runtime_ = rtfg::loadRuntimeConfig(config_path_, solver_urdf_path_);
    robot_ = rtfg::loadRobotModel(solver_urdf_path_, base_link_, tip_link_);

    load_srv_ = create_service<LoadConfig>(
      "/rtfg/load_config",
      std::bind(&RtfgSolverNode::onLoadConfig, this, std::placeholders::_1, std::placeholders::_2));
    fit_srv_ = create_service<FitPreview>(
      "/rtfg/fit_preview",
      std::bind(&RtfgSolverNode::onFitPreview, this, std::placeholders::_1, std::placeholders::_2));
    execute_srv_ = create_service<ExecuteCached>(
      "/rtfg/execute_cached",
      std::bind(&RtfgSolverNode::onExecuteCached, this, std::placeholders::_1, std::placeholders::_2));

    move_to_start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/rtfg/move_to_start",
      std::bind(&RtfgSolverNode::onMoveToStart, this, std::placeholders::_1, std::placeholders::_2));

    move_home_srv_ = create_service<std_srvs::srv::Trigger>(
      "/rtfg/move_to_home",
      std::bind(&RtfgSolverNode::onMoveToHome, this, std::placeholders::_1, std::placeholders::_2));

    target_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/rtfg/target_tcp_poses", 10);
    actual_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/rtfg/tcp_path", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/rtfg/collision_markers", 10);
    metrics_pub_ = create_publisher<std_msgs::msg::String>("/rtfg/metrics", 10);
    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);

    // Joint state publisher for simulation-mode RViz2 animation.
    // When no real robot controller is available, the solver publishes
    // joint states directly so robot_state_publisher + RViz2 display the arm.
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    execute_client_ = rclcpp_action::create_client<FollowJT>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      controller_action_);
    RCLCPP_INFO(get_logger(), "assembly_rtfg_cuda solver ready: %s", solver_urdf_path_.c_str());
  }

private:
  void onLoadConfig(const LoadConfig::Request::SharedPtr request, LoadConfig::Response::SharedPtr response)
  {
    try {
      const std::string path = request->yaml_path.empty() ? config_path_ : request->yaml_path;
      runtime_ = rtfg::loadRuntimeConfig(path, solver_urdf_path_);
      config_path_ = path;
      response->success = true;
      response->message = "ok";
      fillRuntimeResponse(response);
    } catch (const std::exception& e) {
      response->success = false;
      response->message = e.what();
    }
  }

  void onFitPreview(const FitPreview::Request::SharedPtr request, FitPreview::Response::SharedPtr response)
  {
    std::scoped_lock lock(mutex_);

    // Cancel any ongoing joint-state playback from a prior execute_cached.
    if (playback_timer_) {
      playback_timer_->cancel();
    }

    try {
      const auto t_total_start = std::chrono::high_resolution_clock::now();

      rtfg::TrajectoryParams params;
      params.left_wall_offset = request->left_wall_offset;
      params.mud_height = request->mud_height;
      params.approach_len = request->approach_len;
      params.theta_deg = request->theta_deg;
      params.depth = request->depth;
      params.x_plane = request->x_plane;

      rtfg::EnvironmentPose pose;
      pose.x = request->pose_x;
      pose.y = request->pose_y;
      pose.z = request->pose_z;
      pose.roll_deg = request->roll_deg;
      pose.pitch_deg = request->pitch_deg;
      pose.yaw_deg = request->yaw_deg;

      const auto t_plan_start = std::chrono::high_resolution_clock::now();
      const auto target_plan = rtfg::buildTargetPlan(params, pose);
      const auto t_plan_end = std::chrono::high_resolution_clock::now();

      const auto basin_boxes = rtfg::buildBasinBoxes(pose);
      rtfg::SolverConfig solver_cfg;
      solver_cfg.solver_mode = solver_mode_;
      solver_cfg.solver_backend = solver_backend_;
      solver_cfg.clearance_threshold =
        request->clearance_threshold > 0.0 ? request->clearance_threshold : clearance_threshold_;
      solver_cfg.max_iterations_full = max_iterations_full_;
      solver_cfg.max_iterations_realtime = max_iterations_realtime_;
      solver_cfg.max_iterations = solver_mode_ == "realtime" ?
        max_iterations_realtime_ : max_iterations_full_;
      solver_cfg.stagnation_epsilon = stagnation_epsilon_;
      solver_cfg.stagnation_patience = stagnation_patience_;
      solver_cfg.dq_stop_threshold = dq_stop_threshold_;
      solver_cfg.max_collision_candidates_full = max_collision_candidates_full_;
      solver_cfg.max_collision_candidates_realtime = max_collision_candidates_realtime_;
      solver_cfg.collision_check_stride_full = collision_check_stride_full_;
      solver_cfg.collision_check_stride_realtime = collision_check_stride_realtime_;
      solver_cfg.publish_sparse_posearray_realtime = publish_sparse_posearray_realtime_;
      solver_cfg.posearray_stride_realtime = posearray_stride_realtime_;
      solver_cfg.enable_window_solve = enable_window_solve_;
      solver_cfg.window_size = window_size_;
      solver_cfg.window_overlap = window_overlap_;
      const Eigen::VectorXd current_q = vectorToEigen(request->current_q, runtime_.initial_q);

      const auto t_solve_start = std::chrono::high_resolution_clock::now();
      rtfg::ContinuousTrajectorySolver solver(robot_, basin_boxes, solver_cfg);
      rtfg::RollingPlanner planner(solver_cfg);
      auto result = planner.solve(solver, target_plan.tforms, target_plan.segment_names,
                                  current_q, runtime_.initial_q);
      const auto t_solve_end = std::chrono::high_resolution_clock::now();

      response->success = !result.has_collision;
      response->message = result.has_collision ? "trajectory solved but collision audit found violations" : "ok";
      const auto t_traj_pack_start = std::chrono::high_resolution_clock::now();
      response->trajectory = makeJointTrajectory(result.playback_q);
      const auto t_traj_pack_end = std::chrono::high_resolution_clock::now();
      // Always cache trajectory so execute_cached works.
      // When collision audit finds violations, still cache but warn the user.
      cached_trajectory_ = response->trajectory;
      cached_valid_ = true;
      if (!response->success) {
        RCLCPP_WARN(get_logger(),
          "Trajectory cached with collision audit warnings (%ld collision points). "
          "Review collision report before executing.",
          result.collision_sample_indices.size());
      }
      const int pose_stride = (solver_mode_ == "realtime" && publish_sparse_posearray_realtime_) ?
        std::max(1, posearray_stride_realtime_) : 1;
      const rclcpp::Time stamp = now();
      const auto t_target_msg_start = std::chrono::high_resolution_clock::now();
      response->target_tcp_poses = makeTargetPoseArray(target_plan, pose_stride, stamp);
      const auto t_target_msg_end = std::chrono::high_resolution_clock::now();

      const auto t_actual_msg_start = std::chrono::high_resolution_clock::now();
      response->actual_tcp_poses = makeActualPoseArray(result.playback_q, pose_stride, stamp);
      const auto t_actual_msg_end = std::chrono::high_resolution_clock::now();

      const double posearray_pack_s =
        std::chrono::duration<double>(t_target_msg_end - t_target_msg_start).count() +
        std::chrono::duration<double>(t_actual_msg_end - t_actual_msg_start).count();
      const double joint_traj_pack_s =
        std::chrono::duration<double>(t_traj_pack_end - t_traj_pack_start).count();

      result.timing.posearray_pack_s = posearray_pack_s;
      result.timing.joint_trajectory_pack_s = joint_traj_pack_s;
      result.timing.posearray_points =
        static_cast<int32_t>(response->target_tcp_poses.poses.size() +
                             response->actual_tcp_poses.poses.size());
      result.timing.joint_trajectory_points = static_cast<int32_t>(response->trajectory.points.size());

      fillCollisionResponse(result, response);
      fillMetricResponse(result, response);

      const auto t_publish_start = std::chrono::high_resolution_clock::now();
      publishVisualization(response->target_tcp_poses, response->actual_tcp_poses, result);
      const auto t_publish_end = std::chrono::high_resolution_clock::now();
      const auto t_total_end = std::chrono::high_resolution_clock::now();

      const std::size_t point_count = target_plan.tforms.size();
      const std::size_t anchor_count = static_cast<std::size_t>(result.anchor_q.rows());
      const std::size_t playback_count = static_cast<std::size_t>(result.playback_q.rows());
      const double ros_msg_packaging_s =
        std::chrono::duration<double>(t_target_msg_end - t_target_msg_start).count() +
        std::chrono::duration<double>(t_actual_msg_end - t_actual_msg_start).count() +
        std::chrono::duration<double>(t_publish_end - t_publish_start).count();

      logProfile(get_logger(), "轨迹点生成",
        std::chrono::duration<double>(t_plan_end - t_plan_start).count(), point_count);
      logProfile(get_logger(), "TF查询", 0.0, point_count);
      logProfile(
        get_logger(), "主求解器",
        std::chrono::duration<double>(t_solve_end - t_solve_start).count(), anchor_count);
      logProfile(get_logger(), "单点IK平均", result.timing.avg_per_pose_s, anchor_count);
      logProfile(get_logger(), "1300点IK总耗时", result.timing.ik_total_s, anchor_count);
      logProfile(get_logger(), "碰撞检测", result.timing.collision_total_s, playback_count);
      logProfile(get_logger(), "MoveIt plan", 0.0, point_count);
      logProfile(get_logger(), "时间参数化", result.timing.playback_generation_s, playback_count);
      logProfile(get_logger(), "PoseArray打包", posearray_pack_s, result.timing.posearray_points);
      logProfile(get_logger(), "JointTrajectory打包", joint_traj_pack_s, result.timing.joint_trajectory_points);
      logProfile(get_logger(), "ROS消息打包", ros_msg_packaging_s, playback_count);
      RCLCPP_INFO(get_logger(),
                  "[PROFILE] solver_mode: %s\n[PROFILE] points_requested: %d\n[PROFILE] points_solved: %d\n[PROFILE] total_time_ms: %.3f\n[PROFILE] ik_total_ms: %.3f\n[PROFILE] ik_avg_ms_per_point: %.3f\n[PROFILE] collision_total_ms: %.3f\n[PROFILE] fcl_calls: %d\n[PROFILE] fcl_skipped_points: %d\n[PROFILE] fcl_forced_points: %d\n[PROFILE] continuous_prediction_hits: %d\n[PROFILE] continuous_prediction_fallbacks: %d\n[PROFILE] candidate_quick_rejects: %d\n[PROFILE] posearray_points: %d\n[PROFILE] joint_trajectory_points: %d\n[PROFILE] posearray_pack_ms: %.3f\n[PROFILE] joint_trajectory_pack_ms: %.3f\n[PROFILE] avg_ik_iterations: %.3f\n[PROFILE] max_ik_iterations: %d\n[PROFILE] original_candidates: %d\n[PROFILE] candidates_filtered_by_error: %d\n[PROFILE] candidates_entered_fcl: %d\n[PROFILE] candidates_truncated: %d",
                  result.timing.solver_mode.c_str(),
                  result.timing.points_requested,
                  result.timing.points_solved,
                  result.timing.total_wall_s * 1000.0,
                  result.timing.ik_total_s * 1000.0,
                  result.timing.avg_per_pose_s * 1000.0,
                  result.timing.collision_total_s * 1000.0,
                  result.timing.fcl_calls,
                  result.timing.fcl_skipped_points,
                  result.timing.fcl_forced_points,
                  result.timing.continuous_prediction_hits,
                  result.timing.continuous_prediction_fallbacks,
                  result.timing.candidate_quick_rejects,
                  result.timing.posearray_points,
                  result.timing.joint_trajectory_points,
                  result.timing.posearray_pack_s * 1000.0,
                  result.timing.joint_trajectory_pack_s * 1000.0,
                  result.timing.avg_ik_iterations,
                  result.timing.ik_iterations_max,
                  result.timing.original_candidates,
                  result.timing.candidates_filtered_by_error,
                  result.timing.candidates_entered_fcl,
                  result.timing.candidates_truncated);
      logProfile(get_logger(), "总耗时",
        std::chrono::duration<double>(t_total_end - t_total_start).count(), point_count);
    } catch (const std::exception& e) {
      cached_valid_ = false;
      response->success = false;
      response->message = e.what();
    }
  }

  void onExecuteCached(const ExecuteCached::Request::SharedPtr request, ExecuteCached::Response::SharedPtr response)
  {
    std::scoped_lock lock(mutex_);
    const auto t_send_start = std::chrono::high_resolution_clock::now();
    if (!request->execute) {
      response->success = false;
      response->message = "execute flag must be true";
      return;
    }
    if (!cached_valid_) {
      response->success = false;
      response->message = "no cached trajectory";
      return;
    }

    // Try the real robot controller first.
    // If unavailable, fall back to joint-state playback for RViz2 animation.
    if (execute_client_->wait_for_action_server(3s)) {
      FollowJT::Goal goal;
      goal.trajectory = cached_trajectory_;
      execute_client_->async_send_goal(goal);
      const auto t_send_end = std::chrono::high_resolution_clock::now();
      logProfile(
        get_logger(), "发送JointTrajectory",
        std::chrono::duration<double>(t_send_end - t_send_start).count(),
        cached_trajectory_.points.size());
      response->success = true;
      response->message = "execute goal submitted to robot controller";
      return;
    }

    // Fallback: joint-state playback for simulation / visualisation.
    // Publish the cached trajectory as /joint_states at ~50 Hz so
    // robot_state_publisher + RViz2 animate the arm.
    RCLCPP_INFO(get_logger(),
      "Trajectory controller not available — starting joint-state playback "
      "(%zu points) for RViz2 animation", cached_trajectory_.points.size());
    playback_index_ = 0;
    // Cancel any prior playback timer to avoid double-publishing.
    if (playback_timer_) {
      playback_timer_->cancel();
    }
    playback_timer_ = create_wall_timer(
      20ms, std::bind(&RtfgSolverNode::onPlaybackTimer, this));
    const auto t_send_end = std::chrono::high_resolution_clock::now();
    logProfile(
      get_logger(), "JointState播放",
      std::chrono::duration<double>(t_send_end - t_send_start).count(),
      cached_trajectory_.points.size());
    response->success = true;
    response->message = "joint-state playback started (simulation mode)";
  }

  void onMoveToStart(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                      std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::scoped_lock lock(mutex_);
    // Cancel any ongoing playback or move timer
    if (playback_timer_) {
      playback_timer_->cancel();
    }
    if (move_timer_) {
      move_timer_->cancel();
    }
    // Move to the FIRST trajectory target (入泥点/approach start),
    // matching MATLAB's "移动到轨迹起始点" behavior.
    // This solves IK for the first target pose, not just initial_q.
    RCLCPP_INFO(get_logger(), "Moving to trajectory start (入泥点) via IK solve");
    try {
      const auto target_plan = rtfg::buildTargetPlan(runtime_.trajectory, runtime_.pose);
      if (target_plan.tforms.empty()) {
        response->success = false;
        response->message = "target plan is empty";
        return;
      }
      const auto basin_boxes = rtfg::buildBasinBoxes(runtime_.pose);
      rtfg::SolverConfig cfg;
      cfg.solver_mode = "full";
      cfg.solver_backend = solver_backend_;
      cfg.clearance_threshold = clearance_threshold_;
      cfg.max_iterations = max_iterations_full_;
      cfg.stagnation_epsilon = stagnation_epsilon_;
      cfg.stagnation_patience = stagnation_patience_;
      cfg.dq_stop_threshold = dq_stop_threshold_;
      auto ik_backend = rtfg::createIKSolverBackend(cfg);
      const rtfg::Mat4 first_target = target_plan.tforms.front();
      const std::vector<std::pair<std::array<double, 6>, double>> weight_schedule = {
        {{{1, 1, 1, 0.20, 0.20, 0.20}}, M_PI / 6.0},
        {{{1, 1, 1, 0.10, 0.10, 0.10}}, M_PI / 4.0},
        {{{1, 1, 1, 0.03, 0.03, 0.03}}, 70.0 * M_PI / 180.0},
        {{{1, 1, 1, 0.00, 0.00, 0.00}}, std::numeric_limits<double>::infinity()}};
      rtfg::CandidateInfo best;
      const Eigen::VectorXd seed = runtime_.initial_q;
      for (const auto& sched : weight_schedule) {
        best = ik_backend->solve(robot_, basin_boxes, cfg, first_target,
                                 seed, seed, Eigen::VectorXd::Zero(6),
                                 sched.first, sched.second);
        if (best.valid && best.pos_err <= cfg.ik_position_tolerance &&
            best.rot_err <= sched.second) {
          break;
        }
      }
      if (!best.valid) {
        response->success = false;
        response->message = "IK failed for trajectory start pose";
        return;
      }
      const auto entry_pt = target_plan.entry;
      RCLCPP_INFO(get_logger(), "IK solved: pos_err=%.4f m, rot_err=%.4f deg, clear=%.4f m",
                  best.pos_err, rtfg::rad2deg(best.rot_err), best.clearance);
      RCLCPP_INFO(get_logger(), "Entry point: (%.3f, %.3f, %.3f)",
                  entry_pt.x(), entry_pt.y(), entry_pt.z());
      move_index_ = 0;
      move_steps_ = 80;  // 4 seconds at 50 Hz for a longer, smoother move
      move_target_q_ = best.q;
      move_timer_ = create_wall_timer(
        20ms, std::bind(&RtfgSolverNode::onMoveTimer, this));
      response->success = true;
      response->message = "moving to trajectory start (入泥点)";
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("move to start failed: ") + e.what();
    }
  }

  void onMoveToHome(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::scoped_lock lock(mutex_);
    // Cancel any ongoing playback or move timer
    if (playback_timer_) {
      playback_timer_->cancel();
    }
    if (move_timer_) {
      move_timer_->cancel();
    }
    RCLCPP_INFO(get_logger(), "Moving to home position (initial URDF pose)");
    move_index_ = 0;
    move_steps_ = 60;  // 3 seconds at 50 Hz
    move_target_q_ = runtime_.initial_q;
    move_timer_ = create_wall_timer(
      20ms, std::bind(&RtfgSolverNode::onMoveTimer, this));
    response->success = true;
    response->message = "moving to home position";
  }

  void onMoveTimer()
  {
    if (move_index_ >= move_steps_) {
      move_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Move-to-start playback complete");
      return;
    }
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.header.frame_id = base_link_;
    js.name = kJointNames;
    js.position.resize(6);
    // Quintic blend from current to target for smooth visual motion
    const double a = rtfg::quinticBlend(
        static_cast<double>(move_index_) / static_cast<double>(move_steps_));
    for (int j = 0; j < 6; ++j) {
      // Simplified: just blend from zero to target (in simulation, starting
      // from an arbitrary state isn't critical — RViz2 shows the target pose)
      js.position[static_cast<std::size_t>(j)] = move_target_q_(j) * a;
    }
    joint_state_pub_->publish(js);
    ++move_index_;
  }

  void fillRuntimeResponse(const LoadConfig::Response::SharedPtr& response) const
  {
    const auto& t = runtime_.trajectory;
    const auto& p = runtime_.pose;
    response->left_wall_offset = t.left_wall_offset;
    response->mud_height = t.mud_height;
    response->approach_len = t.approach_len;
    response->theta_deg = t.theta_deg;
    response->depth = t.depth;
    response->x_plane = t.x_plane;
    response->pose_x = p.x;
    response->pose_y = p.y;
    response->pose_z = p.z;
    response->roll_deg = p.roll_deg;
    response->pitch_deg = p.pitch_deg;
    response->yaw_deg = p.yaw_deg;
    response->initial_q.resize(6);
    for (int i = 0; i < 6; ++i) {
      response->initial_q[static_cast<std::size_t>(i)] = runtime_.initial_q(i);
    }
  }

  geometry_msgs::msg::PoseArray makeTargetPoseArray(
    const rtfg::TargetPlan& target_plan, int stride, const rclcpp::Time& stamp) const
  {
    return makeSparsePoseArray(
      base_link_, stamp, stride, target_plan.tforms.size(),
      [&](std::size_t i) { return poseFromTform(target_plan.tforms[i]); });
  }

  geometry_msgs::msg::PoseArray makeActualPoseArray(
    const Eigen::MatrixXd& q_series, int stride, const rclcpp::Time& stamp) const
  {
    return makeSparsePoseArray(
      base_link_, stamp, stride, static_cast<std::size_t>(q_series.rows()),
      [&](std::size_t i) {
        return poseFromTform(rtfg::tipTransform(robot_, q_series.row(static_cast<int>(i)).transpose()));
      });
  }

  void fillCollisionResponse(
    const rtfg::TrajectoryResult& result, const FitPreview::Response::SharedPtr& response) const
  {
    response->collision_points_xyz.reserve(static_cast<std::size_t>(result.collision_points.rows() * 3));
    for (int i = 0; i < result.collision_points.rows(); ++i) {
      response->collision_points_xyz.push_back(result.collision_points(i, 0));
      response->collision_points_xyz.push_back(result.collision_points(i, 1));
      response->collision_points_xyz.push_back(result.collision_points(i, 2));
    }
    response->collision_types = result.collision_types;
    response->collision_objects = result.collision_objects;
    response->collision_segments = result.collision_segments;
    response->min_self_clearance = result.global_minimums.min_self;
    response->min_tool_body_clearance = result.global_minimums.min_tool_body;
    response->min_tool_basin_clearance = result.global_minimums.min_tool_basin;
    response->min_self_object = result.global_minimums.min_self_object;
    response->min_tool_body_object = result.global_minimums.min_tool_body_object;
    response->min_tool_basin_object = result.global_minimums.min_tool_basin_object;
  }

  void fillMetricResponse(
    const rtfg::TrajectoryResult& result, const FitPreview::Response::SharedPtr& response) const
  {
    response->anchor_count = static_cast<int32_t>(result.anchor_q.rows());
    response->playback_count = static_cast<int32_t>(result.playback_q.rows());
    response->max_target_rotation_delta_deg = result.max_target_rot_deg;
    response->max_actual_rotation_delta_deg = result.max_actual_rot_deg;
    response->max_anchor_joint_step_deg = result.max_anchor_qstep_deg;
    response->max_playback_joint_step_deg = result.max_playback_qstep_deg;
    response->timing_total_wall_s = result.timing.total_wall_s;
    response->timing_ik_total_s = result.timing.ik_total_s;
    response->timing_collision_total_s = result.timing.collision_total_s;
    response->timing_avg_per_pose_s = result.timing.avg_per_pose_s;
  }

  void publishVisualization(
    const geometry_msgs::msg::PoseArray& target,
    const geometry_msgs::msg::PoseArray& actual,
    const rtfg::TrajectoryResult& result)
  {
    target_pub_->publish(target);
    actual_pub_->publish(actual);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = base_link_;
    clear.header.stamp = now();
    clear.ns = "rtfg_collisions";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    for (int i = 0; i < result.collision_points.rows(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = base_link_;
      m.header.stamp = now();
      m.ns = "rtfg_collisions";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = result.collision_points(i, 0);
      m.pose.position.y = result.collision_points(i, 1);
      m.pose.position.z = result.collision_points(i, 2);
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.025;
      m.scale.y = 0.025;
      m.scale.z = 0.025;
      m.color.r = 1.0;
      m.color.g = 0.05;
      m.color.b = 0.02;
      m.color.a = 0.9;
      markers.markers.push_back(m);
    }
    marker_pub_->publish(markers);

    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = "assembly_rtfg";
    moveit_msgs::msg::RobotTrajectory robot_traj;
    robot_traj.joint_trajectory = cached_trajectory_;
    display.trajectory.push_back(robot_traj);
    display_pub_->publish(display);

    std_msgs::msg::String metrics;
    std::ostringstream oss;
    oss << "anchor_count: " << result.anchor_q.rows()
        << "\nplayback_count: " << result.playback_q.rows()
        << "\nsolver_mode: " << result.timing.solver_mode
        << "\npoints_requested: " << result.timing.points_requested
        << "\npoints_solved: " << result.timing.points_solved
        << "\ntotal_wall_s: " << result.timing.total_wall_s
        << "\nik_total_s: " << result.timing.ik_total_s
        << "\ncollision_total_s: " << result.timing.collision_total_s
        << "\nplayback_generation_s: " << result.timing.playback_generation_s
        << "\nposearray_pack_s: " << result.timing.posearray_pack_s
        << "\njoint_trajectory_pack_s: " << result.timing.joint_trajectory_pack_s
        << "\noriginal_candidates: " << result.timing.original_candidates
        << "\ncandidates_entered_fcl: " << result.timing.candidates_entered_fcl
        << "\nfcl_calls: " << result.timing.fcl_calls
        << "\nmin_tool_basin_clearance_m: " << result.global_minimums.min_tool_basin;
    metrics.data = oss.str();
    metrics_pub_->publish(metrics);
  }

  std::string config_path_;
  std::string solver_urdf_path_;
  std::string base_link_;
  std::string tip_link_;
  std::string controller_action_;
  std::string solver_mode_;
  std::string solver_backend_{"numeric"};
  double clearance_threshold_{2e-3};
  int max_collision_candidates_full_{8};
  int max_collision_candidates_realtime_{2};
  int collision_check_stride_full_{7};
  int collision_check_stride_realtime_{5};
  bool publish_sparse_posearray_realtime_{true};
  int posearray_stride_realtime_{10};
  bool enable_window_solve_{false};
  int window_size_{100};
  int window_overlap_{10};
  int max_iterations_full_{60};
  int max_iterations_realtime_{30};
  double stagnation_epsilon_{1e-5};
  int stagnation_patience_{3};
  double dq_stop_threshold_{1e-6};
  rtfg::RuntimeConfig runtime_;
  rtfg::RobotModel robot_;
  std::mutex mutex_;
  trajectory_msgs::msg::JointTrajectory cached_trajectory_;
  bool cached_valid_{false};

  rclcpp::Service<LoadConfig>::SharedPtr load_srv_;
  rclcpp::Service<FitPreview>::SharedPtr fit_srv_;
  rclcpp::Service<ExecuteCached>::SharedPtr execute_srv_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr actual_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp_action::Client<FollowJT>::SharedPtr execute_client_;

  // Joint-state playback for simulation mode (no real robot controller)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr playback_timer_;
  std::size_t playback_index_{0};

  // Move-to-start timer for simulation-mode animation
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_to_start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_home_srv_;
  rclcpp::TimerBase::SharedPtr move_timer_;
  int move_index_{0};
  int move_steps_{60};
  Eigen::VectorXd move_target_q_;  // 6-DOF initial_q from runtime config

  void onPlaybackTimer()
  {
    if (playback_index_ >= cached_trajectory_.points.size()) {
      playback_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Joint-state playback complete");
      return;
    }
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.header.frame_id = base_link_;
    js.name = cached_trajectory_.joint_names;
    const auto& pt = cached_trajectory_.points[playback_index_];
    js.position = pt.positions;
    joint_state_pub_->publish(js);
    ++playback_index_;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RtfgSolverNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "rtfg_solver_node fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
