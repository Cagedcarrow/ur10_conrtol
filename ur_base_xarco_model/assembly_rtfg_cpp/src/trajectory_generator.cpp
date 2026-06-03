#include "trajectory_generator.h"
#include "utils.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>

namespace rtfg {
namespace {

constexpr double kDegToRad = M_PI / 180.0;

struct BasinGeom {
  Vec3 world_origin{-0.135, 0.0, 0.25};
  double outer_length_x = 0.37;
  double outer_width_y = 0.50;
  double outer_height_z = 0.18;
  double wall_thickness = 0.003;
  double inner_length_x = outer_length_x - 2.0 * wall_thickness;
  double inner_width_y = outer_width_y - 2.0 * wall_thickness;
  double inner_height_z = outer_height_z;
  double inner_x_min = world_origin.x() - inner_length_x / 2.0;
  double inner_x_max = world_origin.x() + inner_length_x / 2.0;
  double inner_y_min = world_origin.y() - inner_width_y / 2.0;
  double inner_y_max = world_origin.y() + inner_width_y / 2.0;
  double floor_z = world_origin.z();
  double rim_z = world_origin.z() + inner_height_z;
  double x_safety_margin = 0.01;
  double y_safety_margin = 0.01;
  double z_motion_max = rim_z - 0.01;
};

double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
}

Vec3 normalize(const Vec3& v, const Vec3& fallback)
{
  if (v.norm() < 1e-9) {
    return fallback.normalized();
  }
  return v.normalized();
}

Eigen::Matrix3d poseRotation(const EnvironmentPose& pose)
{
  return rpyToRot(pose.roll_deg * kDegToRad, pose.pitch_deg * kDegToRad, pose.yaw_deg * kDegToRad);
}

Mat4 poseTform(const EnvironmentPose& pose)
{
  return rtToTform(poseRotation(pose), Vec3(pose.x, pose.y, pose.z));
}

Vec3 transformPoint(const Mat4& T, const Vec3& p)
{
  return T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
}

double yamlScalar(const YAML::Node& node, const std::string& key, double fallback)
{
  return node[key] ? node[key].as<double>() : fallback;
}

Eigen::VectorXd readInitialQ(const std::string& urdf_path)
{
  std::ifstream in(urdf_path);
  if (!in) {
    throw std::runtime_error("cannot open URDF for initial_q: " + urdf_path);
  }
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const std::regex control_re("<ros2_control[^>]*>[\\s\\S]*?</ros2_control>");
  std::smatch control_match;
  if (!std::regex_search(text, control_match, control_re)) {
    throw std::runtime_error("ros2_control block not found in " + urdf_path);
  }
  const std::string control_text = control_match[0].str();
  const std::vector<std::string> joints = {
    "ur10_shoulder_pan", "ur10_shoulder_lift", "ur10_elbow",
    "ur10_wrist_1", "ur10_wrist_2", "ur10_wrist_3"};
  Eigen::VectorXd q(6);
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const std::regex re(
      "<joint\\s+name=\"" + joints[i] + "\"[\\s\\S]*?<state_interface\\s+name=\"position\">\\s*"
      "<param\\s+name=\"initial_value\">([^<]+)</param>");
    std::smatch match;
    if (!std::regex_search(control_text, match, re)) {
      throw std::runtime_error("initial_value not found for " + joints[i]);
    }
    q(static_cast<int>(i)) = std::stod(match[1].str());
  }
  return q;
}

std::vector<Vec3> computeTangents(const std::vector<Vec3>& points)
{
  std::vector<Vec3> tangents(points.size(), Vec3(0.0, 1.0, 0.0));
  if (points.size() < 2) {
    return tangents;
  }
  tangents.front() = normalize(points[1] - points[0], Vec3(0.0, 1.0, 0.0));
  tangents.back() = normalize(points.back() - points[points.size() - 2], Vec3(0.0, 1.0, 0.0));
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    tangents[i] = normalize(points[i + 1] - points[i - 1], Vec3(0.0, 1.0, 0.0));
  }
  return tangents;
}

Eigen::Matrix3d buildTrackRotation(const Vec3& tangent)
{
  const Vec3 world_up(0.0, 0.0, 1.0);
  const Vec3 x_axis = -normalize(tangent, Vec3(1.0, 0.0, 0.0));
  Vec3 z_axis = world_up - world_up.dot(x_axis) * x_axis;
  z_axis = normalize(z_axis, world_up);
  if (z_axis.z() < 0.0) {
    z_axis = -z_axis;
  }
  Vec3 y_axis = normalize(z_axis.cross(x_axis), Vec3(0.0, 1.0, 0.0));
  z_axis = normalize(x_axis.cross(y_axis), world_up);
  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;
  return R;
}

Eigen::Matrix3d buildLiftRotation(const Vec3& heading)
{
  const Vec3 z_axis(0.0, 0.0, 1.0);
  const Vec3 x_axis = -normalize(heading, Vec3(1.0, 0.0, 0.0));
  Vec3 y_axis = normalize(z_axis.cross(x_axis), Vec3(0.0, 1.0, 0.0));
  Vec3 x_fixed = normalize(y_axis.cross(z_axis), Vec3(-1.0, 0.0, 0.0));
  Eigen::Matrix3d R;
  R.col(0) = x_fixed;
  R.col(1) = y_axis;
  R.col(2) = z_axis;
  return R;
}

void appendPose(TargetPlan& plan, const Vec3& point, const Eigen::Matrix3d& R, const std::string& segment)
{
  plan.points.push_back(point);
  plan.tforms.push_back(rtToTform(R, point));
  plan.segment_names.push_back(segment);
  plan.z_axes.push_back(R.col(2));
}

}  // namespace

RuntimeConfig loadRuntimeConfig(const std::string& yaml_path, const std::string& solver_urdf_path)
{
  RuntimeConfig cfg;
  cfg.initial_q = readInitialQ(solver_urdf_path);
  YAML::Node root = YAML::LoadFile(yaml_path);
  YAML::Node params = root["trajectory"] && root["trajectory"]["parameters"] ?
    root["trajectory"]["parameters"] : root["parameters"];
  if (params) {
    cfg.trajectory.left_wall_offset = yamlScalar(params, "left_wall_offset", cfg.trajectory.left_wall_offset);
    cfg.trajectory.mud_height = yamlScalar(params, "mud_height", cfg.trajectory.mud_height);
    cfg.trajectory.approach_len = yamlScalar(params, "approach_len", cfg.trajectory.approach_len);
    cfg.trajectory.theta_deg = yamlScalar(params, "theta_deg", cfg.trajectory.theta_deg);
    cfg.trajectory.depth = yamlScalar(params, "depth", cfg.trajectory.depth);
    cfg.trajectory.x_plane = yamlScalar(params, "x_plane", cfg.trajectory.x_plane);
  }
  YAML::Node pose = root["environment_pose"] && root["environment_pose"]["pose"] ?
    root["environment_pose"]["pose"] : root["pose"];
  if (pose) {
    cfg.pose.x = yamlScalar(pose, "x", cfg.pose.x);
    cfg.pose.y = yamlScalar(pose, "y", cfg.pose.y);
    cfg.pose.z = yamlScalar(pose, "z", cfg.pose.z);
    cfg.pose.roll_deg = yamlScalar(pose, "roll_deg", cfg.pose.roll_deg);
    cfg.pose.pitch_deg = yamlScalar(pose, "pitch_deg", cfg.pose.pitch_deg);
    cfg.pose.yaw_deg = yamlScalar(pose, "yaw_deg", cfg.pose.yaw_deg);
  }
  cfg.trajectory = clampTrajectoryParams(cfg.trajectory);
  cfg.pose = clampEnvironmentPose(cfg.pose);
  return cfg;
}

TrajectoryParams clampTrajectoryParams(const TrajectoryParams& params)
{
  BasinGeom basin;
  TrajectoryParams out = params;
  out.left_wall_offset = clamp(out.left_wall_offset, basin.y_safety_margin, basin.inner_width_y - basin.y_safety_margin);
  out.mud_height = clamp(out.mud_height, 0.0, basin.inner_height_z);
  out.approach_len = clamp(out.approach_len, 0.0, 0.30);
  out.theta_deg = clamp(out.theta_deg, -60.0, -5.0);
  out.depth = clamp(out.depth, 0.01, 0.10);
  out.x_plane = clamp(out.x_plane, basin.inner_x_min + basin.x_safety_margin, basin.inner_x_max - basin.x_safety_margin);
  return out;
}

EnvironmentPose clampEnvironmentPose(const EnvironmentPose& pose)
{
  return pose;
}

TargetPlan buildTargetPlan(const TrajectoryParams& params_in, const EnvironmentPose& pose)
{
  const auto params = clampTrajectoryParams(params_in);
  BasinGeom basin;
  const double y_entry = basin.inner_y_min + params.left_wall_offset;
  const double z_mud = basin.floor_z + params.mud_height;
  const double theta = params.theta_deg * kDegToRad;
  const double depth_scale = 1.0 - std::cos(std::abs(theta));
  if (depth_scale < 1e-6) {
    throw std::runtime_error("theta_deg too small to derive arc radius");
  }
  const double radius = params.depth / depth_scale;
  const Eigen::Vector2d dir_entry(std::cos(theta), std::sin(theta));
  const Eigen::Vector2d p_entry(y_entry, z_mud);
  const Eigen::Vector2d p_approach = p_entry - params.approach_len * dir_entry;
  const double a1 = theta - M_PI / 2.0;
  const double a2 = -M_PI / 2.0;
  const Eigen::Vector2d center = p_entry - radius * Eigen::Vector2d(std::cos(a1), std::sin(a1));

  std::vector<Vec3> seg0;
  std::vector<Vec3> seg1;
  std::vector<Vec3> seg2;
  constexpr int n0 = 30;
  constexpr int n_arc = 140;
  constexpr int n_lift = 90;
  for (int i = 0; i < n0; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n0 - 1);
    const Eigen::Vector2d p = (1.0 - t) * p_approach + t * p_entry;
    seg0.emplace_back(params.x_plane, p.x(), p.y());
  }
  for (int i = 0; i < n_arc; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_arc - 1);
    const double a = (1.0 - t) * a1 + t * a2;
    seg1.emplace_back(params.x_plane, center.x() + radius * std::cos(a), center.y() + radius * std::sin(a));
  }
  const Vec3 p_arc_end = seg1.back();
  const double lift_len = std::max(0.0, std::min(basin.z_motion_max - p_arc_end.z(), std::max(0.06, 1.20 * radius)));
  for (int i = 0; i < n_lift; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_lift - 1);
    seg2.emplace_back(params.x_plane, p_arc_end.y(), p_arc_end.z() + t * lift_len);
  }

  std::vector<Vec3> all;
  all.insert(all.end(), seg0.begin(), seg0.end() - 1);
  all.insert(all.end(), seg1.begin(), seg1.end());
  all.insert(all.end(), seg2.begin() + 1, seg2.end());

  const auto tangents = computeTangents(all);
  const Mat4 T_pose = poseTform(pose);
  TargetPlan plan;
  plan.arc_radius = radius;
  plan.vertical_penetration = z_mud - p_arc_end.z();
  plan.approach_start = transformPoint(T_pose, seg0.front());
  plan.entry = transformPoint(T_pose, Vec3(params.x_plane, p_entry.x(), p_entry.y()));
  plan.arc_end = transformPoint(T_pose, p_arc_end);

  const std::size_t seg0_end = seg0.size() - 1;
  const std::size_t seg1_start = seg0_end;
  const std::size_t seg1_end = seg0_end + seg1.size() - 1;
  Vec3 lift_heading = Vec3::Zero();
  for (std::size_t i = seg1_start; i <= seg1_end && i < tangents.size(); ++i) {
    Vec3 proj = tangents[i];
    proj.z() = 0.0;
    if (proj.norm() > 1e-9) {
      lift_heading += proj;
    }
  }
  lift_heading = normalize(lift_heading, Vec3(0.0, 1.0, 0.0));
  const Eigen::Matrix3d lift_R = buildLiftRotation(lift_heading);
  const Eigen::Matrix3d shared_start_R = buildTrackRotation(tangents[seg1_end]);
  const double rot_delta = rotationDistance(shared_start_R, lift_R);
  const int transition_steps = std::min(std::max(8, static_cast<int>(std::ceil(rot_delta / (3.0 * kDegToRad))) + 1), 18);

  for (std::size_t i = 0; i < all.size(); ++i) {
    Eigen::Matrix3d R = buildTrackRotation(tangents[i]);
    std::string segment = "斜线";
    if (i >= seg1_start) {
      segment = "圆弧";
    }
    if (i <= seg1_end) {
      appendPose(plan, transformPoint(T_pose, all[i]), T_pose.block<3, 3>(0, 0) * R, segment);
    } else {
      break;
    }
  }
  const Vec3 shared_point = transformPoint(T_pose, all[seg1_end]);
  for (int i = 0; i < transition_steps; ++i) {
    const double alpha = quinticBlend(static_cast<double>(i) / static_cast<double>(std::max(transition_steps - 1, 1)));
    const Eigen::Matrix3d R = slerpRotation(shared_start_R, lift_R, alpha);
    appendPose(plan, shared_point, T_pose.block<3, 3>(0, 0) * R, "出泥");
  }
  for (std::size_t i = seg1_end + 1; i < all.size(); ++i) {
    appendPose(plan, transformPoint(T_pose, all[i]), T_pose.block<3, 3>(0, 0) * lift_R, "出泥");
  }
  return plan;
}

std::vector<BasinBox> buildBasinBoxes(const EnvironmentPose& pose)
{
  const Mat4 T = poseTform(pose);
  struct BoxSpec {
    const char* name;
    Vec3 size;
    Vec3 offset;
  };
  const BoxSpec specs[] = {
    {"block_basin_bottom", Vec3(0.37, 0.50, 0.003), Vec3(-0.135, 0.0, 0.25)},
    {"block_basin_front_wall", Vec3(0.37, 0.003, 0.18), Vec3(-0.135, 0.2485, 0.34)},
    {"block_basin_back_wall", Vec3(0.37, 0.003, 0.18), Vec3(-0.135, -0.2485, 0.34)},
    {"block_basin_left_wall", Vec3(0.003, 0.494, 0.18), Vec3(-0.3185, 0.0, 0.34)},
    {"block_basin_right_wall", Vec3(0.003, 0.494, 0.18), Vec3(0.0485, 0.0, 0.34)}};
  std::vector<BasinBox> boxes;
  boxes.reserve(std::size(specs));
  for (const auto& spec : specs) {
    BasinBox box;
    box.name = spec.name;
    box.size = spec.size;
    box.pose = T * rtToTform(Eigen::Matrix3d::Identity(), spec.offset);
    boxes.push_back(box);
  }
  return boxes;
}

}  // namespace rtfg
