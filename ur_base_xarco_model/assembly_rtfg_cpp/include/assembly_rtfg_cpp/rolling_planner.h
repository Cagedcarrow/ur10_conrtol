#pragma once

#include "continuous_trajectory_solver.h"

namespace rtfg {

class RollingPlanner {
public:
  explicit RollingPlanner(SolverConfig cfg);

  bool enabled() const { return cfg_.enable_window_solve; }
  int windowSize() const { return cfg_.window_size; }
  int windowOverlap() const { return cfg_.window_overlap; }

  TrajectoryResult solve(const ContinuousTrajectorySolver& solver,
                         const std::vector<Mat4>& target_tforms,
                         const std::vector<std::string>& segment_names,
                         const Eigen::VectorXd& current_q,
                         const Eigen::VectorXd& home_q) const;

private:
  SolverConfig cfg_;
};

}  // namespace rtfg
