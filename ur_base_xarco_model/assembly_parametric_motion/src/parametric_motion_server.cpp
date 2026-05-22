#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include "assembly_parametric_motion/srv/execute_cached_plan.hpp"
#include "assembly_parametric_motion/srv/manage_experiment_config.hpp"
#include "assembly_parametric_motion/srv/preview_plan.hpp"
#include "assembly_parametric_motion/srv/validate_plan.hpp"

using namespace std::chrono_literals;

namespace
{
struct Limits
{
  double min_depth_mm{20.0};
  double max_depth_mm{40.0};
  double min_speed{0.2};
  double max_speed{1.0};
  double min_angle_deg{20.0};
  double max_angle_deg{60.0};
};

struct ExperimentEntry
{
  std::string id;
  double depth_mm{30.0};
  double speed{0.55};
  double angle_deg{35.0};
  std::string notes;
};

struct ComputeOutput
{
  bool success{false};
  std::string message;
  double fraction{0.0};
  double path_length_m{0.0};
  double planning_time_s{0.0};
  double velocity_scaling{0.1};
  double acceleration_scaling{0.08};
  double estimated_mass_g{0.0};
  moveit_msgs::msg::RobotTrajectory trajectory;
};

bool sync_move_group_parameters(const rclcpp::Node::SharedPtr & node, double timeout_sec)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    RCLCPP_WARN(node->get_logger(), "Timed out waiting for /move_group parameter service.");
    return false;
  }

  const std::vector<std::string> prefixes = {
    "robot_description",
    "robot_description_semantic",
    "robot_description_kinematics",
    "robot_description_planning",
  };

  std::set<std::string> names{
    "robot_description",
    "robot_description_semantic",
  };
  auto list_result = client->list_parameters(prefixes, 10);
  names.insert(list_result.names.begin(), list_result.names.end());

  if (names.empty()) {
    return false;
  }

  auto params = client->get_parameters(std::vector<std::string>(names.begin(), names.end()));
  std::size_t synced = 0;
  for (const auto & p : params) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      continue;
    }
    if (!node->has_parameter(p.get_name())) {
      node->declare_parameter(p.get_name(), p.get_parameter_value());
    } else {
      node->set_parameter(p);
    }
    ++synced;
  }
  RCLCPP_INFO(node->get_logger(), "Synced %zu parameters from /move_group.", synced);
  return synced > 0;
}

double clampv(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}
}  // namespace

class ParametricMotionServer : public rclcpp::Node
{
public:
  ParametricMotionServer()
  : Node("parametric_motion_server")
  {
    std::string default_config_path;
    try {
      default_config_path =
        ament_index_cpp::get_package_share_directory("assembly_description") +
        "/config/parametric_experiment.yaml";
    } catch (...) {
      default_config_path =
        "/root/ur10_ws/src/ur_base_xarco_model/assembly_description/config/parametric_experiment.yaml";
    }

    config_path_ = declare_parameter<std::string>("config_path", default_config_path);
    planning_group_ = declare_parameter<std::string>("planning_group", "assembly_manipulator");
    eef_link_ = declare_parameter<std::string>("eef_link", "sensor_shovel_shovel_tcp");
    cartesian_fraction_min_ = declare_parameter<double>("cartesian_fraction_min", 0.90);
    min_velocity_scaling_ = declare_parameter<double>("min_velocity_scaling", 0.05);
    max_velocity_scaling_ = declare_parameter<double>("max_velocity_scaling", 0.35);
    execute_timeout_s_ = declare_parameter<double>("execute_timeout_s", 120.0);

    manage_srv_ = create_service<assembly_parametric_motion::srv::ManageExperimentConfig>(
      "/assembly/config/load_save",
      std::bind(&ParametricMotionServer::on_manage_config, this, std::placeholders::_1, std::placeholders::_2));
    validate_srv_ = create_service<assembly_parametric_motion::srv::ValidatePlan>(
      "/assembly/plan/validate",
      std::bind(&ParametricMotionServer::on_validate, this, std::placeholders::_1, std::placeholders::_2));
    preview_srv_ = create_service<assembly_parametric_motion::srv::PreviewPlan>(
      "/assembly/plan/preview",
      std::bind(&ParametricMotionServer::on_preview, this, std::placeholders::_1, std::placeholders::_2));
    execute_srv_ = create_service<assembly_parametric_motion::srv::ExecuteCachedPlan>(
      "/assembly/plan/execute",
      std::bind(&ParametricMotionServer::on_execute, this, std::placeholders::_1, std::placeholders::_2));

    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
  }

  bool initialize()
  {
    if (!load_config(config_path_)) {
      RCLCPP_WARN(get_logger(), "Failed to load config at startup: %s", config_path_.c_str());
    }

    execute_client_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
      shared_from_this(), "/execute_trajectory");
    init_timer_ = create_wall_timer(
      2s,
      std::bind(&ParametricMotionServer::ensure_move_group_ready, this));
    ensure_move_group_ready();
    return true;
  }

private:
  void fill_config_response(
    assembly_parametric_motion::srv::ManageExperimentConfig::Response::SharedPtr response) const
  {
    response->experiment_ids.clear();
    for (const auto & kv : experiments_) {
      response->experiment_ids.push_back(kv.first);
    }
    response->active_experiment_id = active_experiment_id_;
    auto it = experiments_.find(active_experiment_id_);
    if (it != experiments_.end()) {
      response->penetration_depth_mm = it->second.depth_mm;
      response->speed_setting = it->second.speed;
      response->entry_angle_deg = it->second.angle_deg;
    }
    response->min_depth_mm = limits_.min_depth_mm;
    response->max_depth_mm = limits_.max_depth_mm;
    response->min_speed = limits_.min_speed;
    response->max_speed = limits_.max_speed;
    response->min_angle_deg = limits_.min_angle_deg;
    response->max_angle_deg = limits_.max_angle_deg;
  }

  bool load_config(const std::string & path)
  {
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "YAML load failed (%s): %s", path.c_str(), e.what());
      return false;
    }

    if (!root["experiments"] || !root["experiments"].IsSequence()) {
      RCLCPP_ERROR(get_logger(), "Config missing experiments sequence: %s", path.c_str());
      return false;
    }

    if (root["limits"]) {
      auto lim = root["limits"];
      if (lim["depth_mm"]) {
        auto depth = lim["depth_mm"];
        limits_.min_depth_mm = depth["min"] ? depth["min"].as<double>() : limits_.min_depth_mm;
        limits_.max_depth_mm = depth["max"] ? depth["max"].as<double>() : limits_.max_depth_mm;
      } else {
        limits_.min_depth_mm = lim["min_depth_mm"] ? lim["min_depth_mm"].as<double>() : limits_.min_depth_mm;
        limits_.max_depth_mm = lim["max_depth_mm"] ? lim["max_depth_mm"].as<double>() : limits_.max_depth_mm;
      }
      if (lim["speed"]) {
        auto speed = lim["speed"];
        limits_.min_speed = speed["min"] ? speed["min"].as<double>() : limits_.min_speed;
        limits_.max_speed = speed["max"] ? speed["max"].as<double>() : limits_.max_speed;
      } else {
        limits_.min_speed = lim["min_speed"] ? lim["min_speed"].as<double>() : limits_.min_speed;
        limits_.max_speed = lim["max_speed"] ? lim["max_speed"].as<double>() : limits_.max_speed;
      }
      if (lim["angle_deg"]) {
        auto angle = lim["angle_deg"];
        limits_.min_angle_deg = angle["min"] ? angle["min"].as<double>() : limits_.min_angle_deg;
        limits_.max_angle_deg = angle["max"] ? angle["max"].as<double>() : limits_.max_angle_deg;
      } else {
        limits_.min_angle_deg = lim["min_angle_deg"] ? lim["min_angle_deg"].as<double>() : limits_.min_angle_deg;
        limits_.max_angle_deg = lim["max_angle_deg"] ? lim["max_angle_deg"].as<double>() : limits_.max_angle_deg;
      }
    }
    if (root["planning"] && root["planning"]["thresholds"] &&
      root["planning"]["thresholds"]["cartesian_fraction_min"])
    {
      cartesian_fraction_min_ = root["planning"]["thresholds"]["cartesian_fraction_min"].as<double>();
    }
    if (root["planning"] && root["planning"]["frame"]) {
      eef_link_ = root["planning"]["frame"].as<std::string>();
    }

    experiments_.clear();
    for (const auto & node : root["experiments"]) {
      ExperimentEntry entry;
      entry.id = node["id"].as<std::string>();
      entry.depth_mm = node["penetration_depth_mm"].as<double>();
      entry.speed = node["speed_setting"].as<double>();
      entry.angle_deg = node["entry_angle_deg"].as<double>();
      entry.notes = node["notes"] ? node["notes"].as<std::string>() : "";
      experiments_[entry.id] = entry;
    }
    if (root["active_experiment_id"]) {
      active_experiment_id_ = root["active_experiment_id"].as<std::string>();
    }
    if (experiments_.find(active_experiment_id_) == experiments_.end() && !experiments_.empty()) {
      active_experiment_id_ = experiments_.begin()->first;
    }
    validated_request_ = std::nullopt;
    cached_valid_ = false;
    config_path_ = path;
    return true;
  }

  bool save_config(const std::string & path)
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "active_experiment_id" << YAML::Value << active_experiment_id_;
    out << YAML::Key << "limits" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "depth_mm" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << limits_.min_depth_mm;
    out << YAML::Key << "max" << YAML::Value << limits_.max_depth_mm;
    out << YAML::EndMap;
    out << YAML::Key << "speed" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << limits_.min_speed;
    out << YAML::Key << "max" << YAML::Value << limits_.max_speed;
    out << YAML::EndMap;
    out << YAML::Key << "angle_deg" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << limits_.min_angle_deg;
    out << YAML::Key << "max" << YAML::Value << limits_.max_angle_deg;
    out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::Key << "planning" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "frame" << YAML::Value << eef_link_;
    out << YAML::Key << "thresholds" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "cartesian_fraction_min" << YAML::Value << cartesian_fraction_min_;
    out << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::Key << "experiments" << YAML::Value << YAML::BeginSeq;
    for (const auto & kv : experiments_) {
      out << YAML::BeginMap;
      out << YAML::Key << "id" << YAML::Value << kv.second.id;
      out << YAML::Key << "penetration_depth_mm" << YAML::Value << kv.second.depth_mm;
      out << YAML::Key << "speed_setting" << YAML::Value << kv.second.speed;
      out << YAML::Key << "entry_angle_deg" << YAML::Value << kv.second.angle_deg;
      out << YAML::Key << "notes" << YAML::Value << kv.second.notes;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream ofs(path, std::ios::out | std::ios::trunc);
    if (!ofs.good()) {
      return false;
    }
    ofs << out.c_str();
    return true;
  }

  bool validate_input(double depth_mm, double speed, double angle_deg, std::string & message) const
  {
    if (depth_mm < limits_.min_depth_mm || depth_mm > limits_.max_depth_mm) {
      message = "penetration_depth_mm out of range";
      return false;
    }
    if (speed < limits_.min_speed || speed > limits_.max_speed) {
      message = "speed_setting out of range";
      return false;
    }
    if (angle_deg < limits_.min_angle_deg || angle_deg > limits_.max_angle_deg) {
      message = "entry_angle_deg out of range";
      return false;
    }
    if (eef_link_ != "sensor_shovel_shovel_tcp") {
      message = "planning.frame must be sensor_shovel_shovel_tcp";
      return false;
    }
    return true;
  }

  double estimate_mass(double speed, double depth_mm, double angle_deg) const
  {
    // Quadratic response surface model from experiment_data_recorder/data_extend.
    const double y =
      -621.736111 +
      218.808619 * speed +
      49.185807 * depth_mm +
      13.806745 * angle_deg -
      728.146570 * speed * speed +
      15.913510 * speed * depth_mm +
      3.023693 * speed * angle_deg -
      0.972549 * depth_mm * depth_mm +
      0.009497 * depth_mm * angle_deg -
      0.224407 * angle_deg * angle_deg;
    return y;
  }

  std::vector<geometry_msgs::msg::Pose> build_waypoints(double depth_mm, double angle_deg, double & path_length)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    path_length = 0.0;

    geometry_msgs::msg::Pose current = move_group_->getCurrentPose(eef_link_).pose;
    tf2::Quaternion q;
    tf2::fromMsg(current.orientation, q);
    q.normalize();
    const tf2::Vector3 x_axis = tf2::quatRotate(q, tf2::Vector3(1.0, 0.0, 0.0));
    const tf2::Vector3 y_axis = tf2::quatRotate(q, tf2::Vector3(0.0, 1.0, 0.0));
    const tf2::Vector3 z_axis = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, 1.0));

    const double angle_rad = angle_deg * M_PI / 180.0;
    tf2::Vector3 cut_dir = std::cos(angle_rad) * x_axis + std::sin(angle_rad) * y_axis;
    if (cut_dir.length2() < 1e-12) {
      cut_dir = x_axis;
    }
    cut_dir.normalize();

    const double depth_m = depth_mm * 0.001;
    const tf2::Vector3 p0(current.position.x, current.position.y, current.position.z);
    const double cut_length = 0.06 + 0.001 * depth_mm;
    const tf2::Vector3 p1 = p0 + 0.02 * z_axis - 0.03 * cut_dir;   // approach
    const tf2::Vector3 p2 = p0 - 0.003 * z_axis;                    // contact
    const tf2::Vector3 p3 = p2 - depth_m * z_axis;                  // penetrate
    const tf2::Vector3 p4 = p3 + cut_length * cut_dir;              // cut
    const tf2::Vector3 p5 = p4 + 0.05 * z_axis - 0.03 * cut_dir;    // lift + retreat

    std::array<tf2::Vector3, 6> pts{p0, p1, p2, p3, p4, p5};
    for (std::size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::msg::Pose pose = current;
      pose.position.x = pts[i].x();
      pose.position.y = pts[i].y();
      pose.position.z = pts[i].z();
      waypoints.push_back(pose);
      if (i > 0) {
        path_length += (pts[i] - pts[i - 1]).length();
      }
    }
    return waypoints;
  }

  std::pair<double, double> speed_to_scaling(double speed) const
  {
    const double alpha = clampv((speed - limits_.min_speed) / std::max(1e-9, limits_.max_speed - limits_.min_speed), 0.0, 1.0);
    const double vel = min_velocity_scaling_ + alpha * (max_velocity_scaling_ - min_velocity_scaling_);
    const double acc = std::min(0.30, 0.80 * vel);
    return {vel, acc};
  }

  ComputeOutput compute_plan(double depth_mm, double speed, double angle_deg)
  {
    ComputeOutput out;
    if (!move_group_ready_ || !move_group_) {
      out.success = false;
      out.message = "move_group not ready";
      return out;
    }
    out.estimated_mass_g = estimate_mass(speed, depth_mm, angle_deg);
    std::string msg;
    if (!validate_input(depth_mm, speed, angle_deg, msg)) {
      out.success = false;
      out.message = msg;
      return out;
    }

    auto [vel, acc] = speed_to_scaling(speed);
    out.velocity_scaling = vel;
    out.acceleration_scaling = acc;
    move_group_->setMaxVelocityScalingFactor(vel);
    move_group_->setMaxAccelerationScalingFactor(acc);
    move_group_->setStartStateToCurrentState();

    double path_length = 0.0;
    auto waypoints = build_waypoints(depth_mm, angle_deg, path_length);
    out.path_length_m = path_length;

    auto t0 = std::chrono::steady_clock::now();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, true);
    auto t1 = std::chrono::steady_clock::now();

    out.fraction = fraction;
    out.planning_time_s = std::chrono::duration<double>(t1 - t0).count();
    out.trajectory = trajectory;
    if (fraction < cartesian_fraction_min_) {
      out.success = false;
      out.message = "cartesian path fraction below threshold";
      return out;
    }

    if (trajectory.joint_trajectory.points.empty()) {
      out.success = false;
      out.message = "empty trajectory generated";
      return out;
    }
    out.success = true;
    out.message = "ok";
    return out;
  }

  void on_manage_config(
    const assembly_parametric_motion::srv::ManageExperimentConfig::Request::SharedPtr request,
    assembly_parametric_motion::srv::ManageExperimentConfig::Response::SharedPtr response)
  {
    std::scoped_lock lock(mutex_);
    const std::string path = request->yaml_path.empty() ? config_path_ : request->yaml_path;
    try {
      if (request->command == "load") {
        if (!load_config(path)) {
          response->success = false;
          response->message = "failed to load yaml";
          return;
        }
      } else if (request->command == "set_active") {
        if (experiments_.find(request->active_experiment_id) == experiments_.end()) {
          response->success = false;
          response->message = "active experiment id not found";
          return;
        }
        active_experiment_id_ = request->active_experiment_id;
      } else if (request->command == "save") {
        if (request->active_experiment_id.empty()) {
          response->success = false;
          response->message = "active_experiment_id is empty";
          return;
        }
        ExperimentEntry & entry = experiments_[request->active_experiment_id];
        entry.id = request->active_experiment_id;
        entry.depth_mm = request->penetration_depth_mm;
        entry.speed = request->speed_setting;
        entry.angle_deg = request->entry_angle_deg;
        entry.notes = request->notes;
        active_experiment_id_ = request->active_experiment_id;
        if (!save_config(path)) {
          response->success = false;
          response->message = "failed to save yaml";
          return;
        }
        config_path_ = path;
      } else {
        response->success = false;
        response->message = "unknown command";
        return;
      }
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("exception: ") + e.what();
      return;
    }

    response->success = true;
    response->message = "ok";
    fill_config_response(response);
  }

  void on_validate(
    const assembly_parametric_motion::srv::ValidatePlan::Request::SharedPtr request,
    assembly_parametric_motion::srv::ValidatePlan::Response::SharedPtr response)
  {
    std::scoped_lock lock(mutex_);
    const auto out = compute_plan(
      request->penetration_depth_mm,
      request->speed_setting,
      request->entry_angle_deg);
    response->success = out.success;
    response->message = out.message;
    response->velocity_scaling = out.velocity_scaling;
    response->acceleration_scaling = out.acceleration_scaling;
    response->estimated_mass_g = out.estimated_mass_g;
    response->cartesian_fraction = out.fraction;
    if (out.success) {
      validated_request_ = RequestedParams{
        request->penetration_depth_mm,
        request->speed_setting,
        request->entry_angle_deg};
      cached_valid_ = false;
    }
  }

  void on_preview(
    const assembly_parametric_motion::srv::PreviewPlan::Request::SharedPtr request,
    assembly_parametric_motion::srv::PreviewPlan::Response::SharedPtr response)
  {
    std::scoped_lock lock(mutex_);
    if (!validated_request_) {
      response->success = false;
      response->message = "validate must pass before preview";
      cached_valid_ = false;
      return;
    }

    const RequestedParams req{
      request->penetration_depth_mm,
      request->speed_setting,
      request->entry_angle_deg};
    if (!matches_validated(req, *validated_request_)) {
      response->success = false;
      response->message = "preview params differ from last validated params";
      cached_valid_ = false;
      return;
    }

    const auto out = compute_plan(
      request->penetration_depth_mm,
      request->speed_setting,
      request->entry_angle_deg);
    response->success = out.success;
    response->message = out.message;
    response->cartesian_fraction = out.fraction;
    response->path_length_m = out.path_length_m;
    response->planning_time_s = out.planning_time_s;

    if (!out.success) {
      cached_valid_ = false;
      return;
    }
    cached_trajectory_ = out.trajectory;
    cached_valid_ = true;

    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = "assembly";
    moveit::core::RobotStatePtr state = move_group_->getCurrentState(5.0);
    if (state) {
      moveit::core::robotStateToRobotStateMsg(*state, display.trajectory_start);
    }
    display.trajectory.push_back(cached_trajectory_);
    display_pub_->publish(display);
  }

  void on_execute(
    const assembly_parametric_motion::srv::ExecuteCachedPlan::Request::SharedPtr request,
    assembly_parametric_motion::srv::ExecuteCachedPlan::Response::SharedPtr response)
  {
    std::scoped_lock lock(mutex_);
    if (!request->execute) {
      response->success = false;
      response->message = "execute flag must be true";
      return;
    }
    if (!cached_valid_) {
      response->success = false;
      response->message = "no cached preview trajectory";
      return;
    }
    if (!execute_client_->wait_for_action_server(5s)) {
      response->success = false;
      response->message = "execute_trajectory action unavailable";
      return;
    }

    moveit_msgs::action::ExecuteTrajectory::Goal goal;
    goal.trajectory = cached_trajectory_;
    auto goal_future = execute_client_->async_send_goal(goal);
    if (goal_future.wait_for(5s) != std::future_status::ready) {
      response->success = false;
      response->message = "timeout sending execute goal";
      return;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      response->success = false;
      response->message = "execute goal rejected";
      return;
    }

    auto result_future = execute_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::duration<double>(execute_timeout_s_)) != std::future_status::ready) {
      response->success = false;
      response->message = "timeout waiting execution result";
      return;
    }
    auto wrapped = result_future.get();
    if (!wrapped.result || wrapped.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      response->success = false;
      response->message = "execution failed";
      return;
    }
    response->success = true;
    response->message = "execution success";
    cached_valid_ = false;
  }

  void ensure_move_group_ready()
  {
    if (move_group_ready_) {
      if (init_timer_) {
        init_timer_->cancel();
      }
      return;
    }
    const bool synced = sync_move_group_parameters(shared_from_this(), 2.0);
    if (!synced || !has_parameter("robot_description") || !has_parameter("robot_description_semantic")) {
      return;
    }
    try {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
      move_group_->setPoseReferenceFrame("base_jizuo");
      move_group_->setPlanningTime(10.0);
      move_group_->setNumPlanningAttempts(5);
      move_group_->setEndEffectorLink(eef_link_);
      move_group_->startStateMonitor(5.0);
      move_group_ready_ = true;
      if (init_timer_) {
        init_timer_->cancel();
      }
      RCLCPP_INFO(get_logger(), "Parametric motion server ready.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "MoveGroupInterface init failed, will retry: %s", e.what());
    }
  }

  struct RequestedParams
  {
    double depth_mm;
    double speed;
    double angle_deg;
  };

  bool matches_validated(const RequestedParams & lhs, const RequestedParams & rhs) const
  {
    constexpr double eps = 1e-9;
    return
      std::abs(lhs.depth_mm - rhs.depth_mm) < eps &&
      std::abs(lhs.speed - rhs.speed) < eps &&
      std::abs(lhs.angle_deg - rhs.angle_deg) < eps;
  }

  std::string config_path_;
  std::string planning_group_;
  std::string eef_link_;
  double cartesian_fraction_min_{0.90};
  double min_velocity_scaling_{0.05};
  double max_velocity_scaling_{0.35};
  double execute_timeout_s_{120.0};

  Limits limits_{};
  std::map<std::string, ExperimentEntry> experiments_;
  std::string active_experiment_id_{"EXP_001"};

  std::mutex mutex_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  bool move_group_ready_{false};
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr execute_client_;

  moveit_msgs::msg::RobotTrajectory cached_trajectory_;
  bool cached_valid_{false};
  std::optional<RequestedParams> validated_request_;

  rclcpp::Service<assembly_parametric_motion::srv::ManageExperimentConfig>::SharedPtr manage_srv_;
  rclcpp::Service<assembly_parametric_motion::srv::ValidatePlan>::SharedPtr validate_srv_;
  rclcpp::Service<assembly_parametric_motion::srv::PreviewPlan>::SharedPtr preview_srv_;
  rclcpp::Service<assembly_parametric_motion::srv::ExecuteCachedPlan>::SharedPtr execute_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParametricMotionServer>();
  if (!node->initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Initialization failed.");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
