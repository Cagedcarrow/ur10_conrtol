#pragma once

#include "types.h"
#include "utils.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace rtfg {

// Full trajectory solver: anchor IK → playback generation → collision audit.
// Uses KDL LMA solver as primary backend with custom DLS as fallback.
TrajectoryResult solveTrajectory(const RobotModel& robot,
                                 const std::vector<BasinBox>& basin_boxes,
                                 const SolverConfig& cfg,
                                 const std::vector<Mat4>& target_tforms,
                                 const std::vector<std::string>& segment_names,
                                 const Eigen::VectorXd& current_q,
                                 const Eigen::VectorXd& home_q);

}  // namespace rtfg
