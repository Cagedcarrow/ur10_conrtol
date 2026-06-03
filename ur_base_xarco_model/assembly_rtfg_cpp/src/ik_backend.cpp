#include "ik_backend.h"

#include <stdexcept>

namespace rtfg {

namespace {

CandidateInfo unsupportedBackend(const std::string& name) {
  throw std::runtime_error("IK backend '" + name + "' is not implemented yet; use numeric");
}

}  // namespace

CandidateInfo CurrentNumericIK::solve(const RobotModel& robot,
                                      const std::vector<BasinBox>& basin_boxes,
                                      const SolverConfig& cfg,
                                      const Mat4& target, const Eigen::VectorXd& seed,
                                      const Eigen::VectorXd& q_prev,
                                      const Eigen::VectorXd& dq_prev,
                                      const std::array<double, 6>& weights,
                                      double orient_limit) const {
  return solveSinglePose(robot, basin_boxes, cfg, target, seed, q_prev, dq_prev, weights,
                         orient_limit);
}

CandidateInfo KdlLMAIK::solve(const RobotModel& robot,
                              const std::vector<BasinBox>& basin_boxes,
                              const SolverConfig& cfg,
                              const Mat4& target, const Eigen::VectorXd& seed,
                              const Eigen::VectorXd& q_prev,
                              const Eigen::VectorXd& dq_prev,
                              const std::array<double, 6>& weights,
                              double orient_limit) const {
  return solveSinglePoseKdl(robot, basin_boxes, cfg, target, seed, q_prev, dq_prev, weights,
                            orient_limit);
}

CandidateInfo TracIKSolver::solve(const RobotModel&, const std::vector<BasinBox>&,
                                  const SolverConfig&, const Mat4&, const Eigen::VectorXd&,
                                  const Eigen::VectorXd&, const Eigen::VectorXd&,
                                  const std::array<double, 6>&, double) const {
  return unsupportedBackend(name());
}

CandidateInfo IKFastSolver::solve(const RobotModel&, const std::vector<BasinBox>&,
                                  const SolverConfig&, const Mat4&, const Eigen::VectorXd&,
                                  const Eigen::VectorXd&, const Eigen::VectorXd&,
                                  const std::array<double, 6>&, double) const {
  return unsupportedBackend(name());
}

CandidateInfo URKinematicsSolver::solve(const RobotModel&, const std::vector<BasinBox>&,
                                        const SolverConfig&, const Mat4&, const Eigen::VectorXd&,
                                        const Eigen::VectorXd&, const Eigen::VectorXd&,
                                        const std::array<double, 6>&, double) const {
  return unsupportedBackend(name());
}

std::unique_ptr<IKSolverBase> createIKSolverBackend(const SolverConfig& cfg) {
  if (cfg.solver_backend == "numeric" || cfg.solver_backend.empty()) {
    return std::make_unique<CurrentNumericIK>();
  }
  if (cfg.solver_backend == "kdl") {
    return std::make_unique<KdlLMAIK>();
  }
  if (cfg.solver_backend == "tracik") {
    return std::make_unique<TracIKSolver>();
  }
  if (cfg.solver_backend == "ikfast") {
    return std::make_unique<IKFastSolver>();
  }
  if (cfg.solver_backend == "ur_kinematics") {
    return std::make_unique<URKinematicsSolver>();
  }
  throw std::runtime_error("Unsupported ik backend: " + cfg.solver_backend);
}

}  // namespace rtfg
