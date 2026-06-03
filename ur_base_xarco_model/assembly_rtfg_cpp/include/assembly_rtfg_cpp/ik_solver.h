#pragma once

#include "types.h"

#include <Eigen/Dense>

#include <array>
#include <vector>

namespace rtfg {

// Solve IK using KDL Levenberg-Marquardt solver (analytical Jacobian, O(N)).
// Returns a CandidateInfo; check .valid to see if solution is usable.
// This is the primary solver — faster and more robust than the custom DLS.
CandidateInfo solveSinglePoseKdl(const RobotModel& robot,
                                  const std::vector<BasinBox>& basin_boxes,
                                  const SolverConfig& cfg,
                                  const Mat4& target, const Eigen::VectorXd& seed,
                                  const Eigen::VectorXd& q_prev,
                                  const Eigen::VectorXd& dq_prev,
                                  const std::array<double, 6>& weights,
                                  double orient_limit);

// Solve IK for a single target pose using damped least squares with multi-seed strategy.
// Returns the best safe candidate; throws if no valid solution found.
CandidateInfo solveSinglePose(const RobotModel& robot,
                              const std::vector<BasinBox>& basin_boxes, const SolverConfig& cfg,
                              const Mat4& target, const Eigen::VectorXd& seed,
                              const Eigen::VectorXd& q_prev, const Eigen::VectorXd& dq_prev,
                              const std::array<double, 6>& weights, double orient_limit);

// Build local seed list from previous config and home config, with ±2π wraps on joints 5,6.
std::vector<Eigen::VectorXd> buildSeedList(const Eigen::VectorXd& q_prev,
                                           const Eigen::VectorXd& home_q,
                                           const RobotModel& robot);

// Build global random seed list for fallback when local seeds fail.
std::vector<Eigen::VectorXd> buildGlobalSeedList(const RobotModel& robot);

}  // namespace rtfg
