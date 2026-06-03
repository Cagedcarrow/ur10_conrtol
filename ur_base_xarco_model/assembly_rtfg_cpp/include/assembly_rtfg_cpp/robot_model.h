#pragma once

#include "types.h"

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declare urdf types
namespace urdf {
struct Pose;
}

namespace rtfg {

// Load robot model from URDF file, tracing the kinematic chain from base_link to tip_link.
RobotModel loadRobotModel(const std::string& urdf_path, const std::string& base_link,
                          const std::string& tip_link);

// Compute link poses for a given joint configuration.
std::unordered_map<std::string, Mat4> forwardKinematics(const RobotModel& robot,
                                                        const Eigen::VectorXd& q);

// Get the TCP (tip link) transform for a given joint configuration.
Mat4 tipTransform(const RobotModel& robot, const Eigen::VectorXd& q);

// Compute 6x6 geometric Jacobian at the TCP for the current configuration.
Mat6 numericJacobian(const RobotModel& robot, const Eigen::VectorXd& q);

// Clamp joint angles to robot limits.
Eigen::VectorXd clampToLimits(const RobotModel& robot, const Eigen::VectorXd& q);

// Clamp to limits with 10% rebound away from the limit when clamping occurs.
// This prevents prediction seeds from getting stuck at the joint boundary.
Eigen::VectorXd clampToLimitsWithRebound(const RobotModel& robot, const Eigen::VectorXd& q);

// Align joint angles to be closest to the reference configuration (handle ±2π wraps).
// This is the C++ equivalent of MATLAB's alignEquivalentConfiguration().
Eigen::VectorXd alignToReference(const RobotModel& robot, const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& q_ref);

// Convert URDF pose to 4x4 homogeneous transform.
Mat4 urdfPoseToTform(const urdf::Pose& pose);

}  // namespace rtfg
