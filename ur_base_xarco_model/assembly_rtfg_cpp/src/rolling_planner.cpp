#include "rolling_planner.h"

#include <algorithm>

namespace rtfg {

RollingPlanner::RollingPlanner(SolverConfig cfg) : cfg_(std::move(cfg)) {}

TrajectoryResult RollingPlanner::solve(const ContinuousTrajectorySolver& solver,
                                       const std::vector<Mat4>& target_tforms,
                                       const std::vector<std::string>& segment_names,
                                       const Eigen::VectorXd& current_q,
                                       const Eigen::VectorXd& home_q) const {
  if (!cfg_.enable_window_solve || target_tforms.empty()) {
    return solver.solve(target_tforms, segment_names, current_q, home_q);
  }
  const int window = std::max(1, std::min(cfg_.window_size,
                                          static_cast<int>(target_tforms.size())));
  std::vector<Mat4> subset(target_tforms.begin(), target_tforms.begin() + window);
  std::vector<std::string> names(segment_names.begin(),
                                 segment_names.begin() + std::min<int>(window, segment_names.size()));
  return solver.solve(subset, names, current_q, home_q);
}

}  // namespace rtfg
