#include "continuous_trajectory_solver.h"

#include <utility>

namespace rtfg {

ContinuousTrajectorySolver::ContinuousTrajectorySolver(const RobotModel& robot,
                                                       const std::vector<BasinBox>& basin_boxes,
                                                       SolverConfig cfg)
  : robot_(robot), basin_boxes_(basin_boxes), cfg_(std::move(cfg)) {}

TrajectoryResult ContinuousTrajectorySolver::solve(const std::vector<Mat4>& target_tforms,
                                                   const std::vector<std::string>& segment_names,
                                                   const Eigen::VectorXd& current_q,
                                                   const Eigen::VectorXd& home_q) const {
  return solveTrajectory(robot_, basin_boxes_, cfg_, target_tforms, segment_names, current_q,
                         home_q);
}

}  // namespace rtfg
