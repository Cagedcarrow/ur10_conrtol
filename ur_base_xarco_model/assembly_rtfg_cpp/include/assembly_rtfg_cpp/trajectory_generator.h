#pragma once

#include "types.h"

#include <string>
#include <vector>

namespace rtfg {

struct EnvironmentPose {
  double x = -1.83;
  double y = 0.0;
  double z = 0.0;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 180.0;
};

struct TrajectoryParams {
  double left_wall_offset = 0.195334;
  double mud_height = 0.108;
  double approach_len = 0.2349;
  double theta_deg = -30.0;
  double depth = 0.05212;
  double x_plane = -0.005312;
};

struct RuntimeConfig {
  TrajectoryParams trajectory;
  EnvironmentPose pose;
  Eigen::VectorXd initial_q = Eigen::VectorXd::Zero(6);
};

struct TargetPlan {
  std::vector<Vec3> points;
  std::vector<Mat4> tforms;
  std::vector<std::string> segment_names;
  std::vector<Vec3> z_axes;
  Vec3 approach_start = Vec3::Zero();
  Vec3 entry = Vec3::Zero();
  Vec3 arc_end = Vec3::Zero();
  double arc_radius = 0.0;
  double vertical_penetration = 0.0;
};

RuntimeConfig loadRuntimeConfig(const std::string& yaml_path, const std::string& solver_urdf_path);

TrajectoryParams clampTrajectoryParams(const TrajectoryParams& params);
EnvironmentPose clampEnvironmentPose(const EnvironmentPose& pose);
TargetPlan buildTargetPlan(const TrajectoryParams& params, const EnvironmentPose& pose);
std::vector<BasinBox> buildBasinBoxes(const EnvironmentPose& pose);

}  // namespace rtfg
