#pragma once

#include "types.h"

#include <Eigen/Dense>

#include <vector>

namespace rtfg {

// Evaluate collision state for a given joint configuration using pre-loaded
// collision geometries from RobotModel.
// Checks self-collision, tool-body, and tool-basin collisions using FCL.
CollisionSummary evaluateConfiguration(const RobotModel& robot,
                                       const std::vector<BasinBox>& basin_boxes,
                                       const SolverConfig& cfg, const Eigen::VectorXd& q);

}  // namespace rtfg
