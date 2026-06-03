#pragma once

#include "collision_checker.h"
#include "robot_model.h"
#include "utils.h"

#include <string>

namespace rtfg {

struct QuickCheckResult {
  bool ok = true;
  std::string reason;
};

class CollisionPipeline {
public:
  CollisionPipeline(const RobotModel& robot,
                    const std::vector<BasinBox>& basin_boxes,
                    const SolverConfig& cfg);

  int collisionStride() const;
  bool shouldRunPreciseCheck(int point_idx, bool keypoint) const;
  QuickCheckResult quickPlaybackCheck(const Eigen::VectorXd& q,
                                      const Eigen::VectorXd& q_prev) const;
  CollisionSummary preciseCheck(const Eigen::VectorXd& q) const;

private:
  const RobotModel& robot_;
  const std::vector<BasinBox>& basin_boxes_;
  const SolverConfig& cfg_;
};

}  // namespace rtfg
