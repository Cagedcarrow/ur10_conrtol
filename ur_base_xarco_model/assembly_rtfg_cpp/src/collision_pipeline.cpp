#include "collision_pipeline.h"

#include <algorithm>

namespace rtfg {

namespace {

bool isMountingBase(const std::string& link_name) {
  return link_name == "base_jizuo" ||
         link_name == "base_jizuo_base_ur10_with_dizuo" ||
         link_name == "ur10";
}

}  // namespace

CollisionPipeline::CollisionPipeline(const RobotModel& robot,
                                     const std::vector<BasinBox>& basin_boxes,
                                     const SolverConfig& cfg)
  : robot_(robot), basin_boxes_(basin_boxes), cfg_(cfg) {}

int CollisionPipeline::collisionStride() const {
  return cfg_.solver_mode == "realtime" ? cfg_.collision_check_stride_realtime
                                        : cfg_.collision_check_stride_full;
}

bool CollisionPipeline::shouldRunPreciseCheck(int point_idx, bool keypoint) const {
  const int stride = collisionStride();
  return stride <= 1 || keypoint || point_idx == 0 || ((point_idx % stride) == 0);
}

QuickCheckResult CollisionPipeline::quickPlaybackCheck(const Eigen::VectorXd& q,
                                                       const Eigen::VectorXd& q_prev) const {
  QuickCheckResult result;
  const Eigen::VectorXd dq = wrapJointDelta(q - q_prev);
  if (dq.norm() > 1.35) {
    result.ok = false;
    result.reason = "joint_delta";
    return result;
  }
  const auto poses = forwardKinematics(robot_, q);
  const auto& tcp = poses.at(robot_.tip_link);
  const double z = tcp(2, 3);
  if (z < -0.05 || z > 1.50) {
    result.ok = false;
    result.reason = "workspace_z";
    return result;
  }
  return result;
}

CollisionSummary CollisionPipeline::preciseCheck(const Eigen::VectorXd& q) const {
  return evaluateConfiguration(robot_, basin_boxes_, cfg_, q);
}

}  // namespace rtfg
