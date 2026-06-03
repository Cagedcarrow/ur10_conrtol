#pragma once

#include "ik_solver.h"

#include <memory>
#include <string>

namespace rtfg {

class IKSolverBase {
public:
  virtual ~IKSolverBase() = default;
  virtual std::string name() const = 0;
  virtual CandidateInfo solve(const RobotModel& robot,
                              const std::vector<BasinBox>& basin_boxes,
                              const SolverConfig& cfg,
                              const Mat4& target, const Eigen::VectorXd& seed,
                              const Eigen::VectorXd& q_prev,
                              const Eigen::VectorXd& dq_prev,
                              const std::array<double, 6>& weights,
                              double orient_limit) const = 0;
};

class CurrentNumericIK final : public IKSolverBase {
public:
  std::string name() const override { return "numeric"; }
  CandidateInfo solve(const RobotModel& robot,
                      const std::vector<BasinBox>& basin_boxes,
                      const SolverConfig& cfg,
                      const Mat4& target, const Eigen::VectorXd& seed,
                      const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev,
                      const std::array<double, 6>& weights,
                      double orient_limit) const override;
};

class KdlLMAIK final : public IKSolverBase {
public:
  std::string name() const override { return "kdl"; }
  CandidateInfo solve(const RobotModel& robot,
                      const std::vector<BasinBox>& basin_boxes,
                      const SolverConfig& cfg,
                      const Mat4& target, const Eigen::VectorXd& seed,
                      const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev,
                      const std::array<double, 6>& weights,
                      double orient_limit) const override;
};

class TracIKSolver final : public IKSolverBase {
public:
  std::string name() const override { return "tracik"; }
  CandidateInfo solve(const RobotModel& robot,
                      const std::vector<BasinBox>& basin_boxes,
                      const SolverConfig& cfg,
                      const Mat4& target, const Eigen::VectorXd& seed,
                      const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev,
                      const std::array<double, 6>& weights,
                      double orient_limit) const override;
};

class IKFastSolver final : public IKSolverBase {
public:
  std::string name() const override { return "ikfast"; }
  CandidateInfo solve(const RobotModel& robot,
                      const std::vector<BasinBox>& basin_boxes,
                      const SolverConfig& cfg,
                      const Mat4& target, const Eigen::VectorXd& seed,
                      const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev,
                      const std::array<double, 6>& weights,
                      double orient_limit) const override;
};

class URKinematicsSolver final : public IKSolverBase {
public:
  std::string name() const override { return "ur_kinematics"; }
  CandidateInfo solve(const RobotModel& robot,
                      const std::vector<BasinBox>& basin_boxes,
                      const SolverConfig& cfg,
                      const Mat4& target, const Eigen::VectorXd& seed,
                      const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev,
                      const std::array<double, 6>& weights,
                      double orient_limit) const override;
};

std::unique_ptr<IKSolverBase> createIKSolverBackend(const SolverConfig& cfg);

}  // namespace rtfg
