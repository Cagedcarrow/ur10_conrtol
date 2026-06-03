#pragma once

#include "trajectory_solver.h"

namespace rtfg {

class ContinuousTrajectorySolver {
public:
  ContinuousTrajectorySolver(const RobotModel& robot,
                             const std::vector<BasinBox>& basin_boxes,
                             SolverConfig cfg);

  TrajectoryResult solve(const std::vector<Mat4>& target_tforms,
                         const std::vector<std::string>& segment_names,
                         const Eigen::VectorXd& current_q,
                         const Eigen::VectorXd& home_q) const;

  const SolverConfig& config() const { return cfg_; }

private:
  const RobotModel& robot_;
  const std::vector<BasinBox>& basin_boxes_;
  SolverConfig cfg_;
};

}  // namespace rtfg
