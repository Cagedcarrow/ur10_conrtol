#include "ik_solver.h"
#include "collision_checker.h"
#include "robot_model.h"
#include "utils.h"

#include <Eigen/Dense>

#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace rtfg {

CandidateInfo solveSinglePose(const RobotModel& robot,
                              const std::vector<BasinBox>& basin_boxes, const SolverConfig& cfg,
                              const Mat4& target, const Eigen::VectorXd& seed,
                              const Eigen::VectorXd& q_prev, const Eigen::VectorXd& dq_prev,
                              const std::array<double, 6>& weights, double orient_limit) {
  Eigen::VectorXd q = clampToLimits(robot, seed);
  Eigen::VectorXd q_ref = q_prev + dq_prev;

  CandidateInfo cand;
  double prev_err_norm = std::numeric_limits<double>::infinity();
  int stagnate_count = 0;

  for (int iter = 0; iter < cfg.max_iterations; ++iter) {
    Mat4 T = tipTransform(robot, q);
    Vec6 err = poseError(T, target);

    Vec6 weighted = err;
    for (int k = 0; k < 6; ++k) {
      weighted(k) *= weights[k];
    }

    double pos_err = err.head<3>().norm();
    double rot_err = err.tail<3>().norm();

    if (pos_err <= cfg.ik_position_tolerance && rot_err <= orient_limit) {
      cand.iterations_used = iter + 1;
      break;
    }

    Mat6 J = numericJacobian(robot, q);
    for (int r = 0; r < 6; ++r) {
      J.row(r) *= weights[r];
    }

    // Adaptive damping: higher when far from target, lower when close
    double lambda = cfg.lambda;
    if (pos_err > 0.1 || rot_err > 0.1) {
      lambda = std::max(1e-3, 5e-3 * (pos_err / 0.05));
      lambda = std::min(lambda, 0.1);
    } else {
      lambda = 5e-4 + 5e-3 * (pos_err / 0.05);
    }

    Eigen::MatrixXd H = J.transpose() * J + lambda * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd g = J.transpose() * weighted;
    Eigen::VectorXd dq = H.ldlt().solve(g);

    double step_norm = dq.norm();
    if (step_norm > 0.45) {
      dq *= (0.45 / step_norm);
    }
    if (step_norm <= cfg.dq_stop_threshold) {
      cand.iterations_used = iter + 1;
      break;
    }

    q = clampToLimits(robot, q + dq);
    q = alignToReference(robot, q, q_ref);

    // Check convergence rate; break early if stagnating
    double err_norm = weighted.norm();
    if (prev_err_norm < std::numeric_limits<double>::infinity() &&
        std::abs(prev_err_norm - err_norm) <= cfg.stagnation_epsilon) {
      stagnate_count++;
      if (stagnate_count >= cfg.stagnation_patience) {
        cand.iterations_used = iter + 1;
        break;
      }
    } else {
      stagnate_count = 0;
    }
    prev_err_norm = err_norm;
    cand.iterations_used = iter + 1;
  }

  Mat4 T = tipTransform(robot, q);
  Vec6 err = poseError(T, target);

  cand.q = q;
  cand.pos_err = err.head<3>().norm();
  cand.rot_err = err.tail<3>().norm();
  cand.clearance = std::numeric_limits<double>::infinity();  // deferred — caller checks
  cand.cost = continuityCost(q, q_prev, dq_prev);
  cand.joint_cost = cand.cost;
  cand.valid = true;
  if (cand.iterations_used <= 0) {
    cand.iterations_used = cfg.max_iterations;
  }
  return cand;
}

std::vector<Eigen::VectorXd> buildSeedList(const Eigen::VectorXd& q_prev,
                                           const Eigen::VectorXd& home_q,
                                           const RobotModel& robot) {
  std::vector<Eigen::VectorXd> seeds;
  seeds.push_back(q_prev);
  seeds.push_back(home_q);
  seeds.push_back(Eigen::VectorXd::Zero(q_prev.size()));

  std::array<double, 3> wraps = {-2.0 * M_PI, 0.0, 2.0 * M_PI};
  std::vector<Eigen::VectorXd> expanded = seeds;
  for (const auto& q0 : seeds) {
    for (double w5 : wraps) {
      for (double w6 : wraps) {
        if (w5 == 0.0 && w6 == 0.0) continue;
        Eigen::VectorXd q = q0;
        if (q.size() >= 5) q(4) += w5;
        if (q.size() >= 6) q(5) += w6;
        expanded.push_back(clampToLimits(robot, q));
      }
    }
  }

  std::vector<Eigen::VectorXd> uniq;
  for (const auto& q : expanded) {
    bool duplicate = false;
    for (const auto& u : uniq) {
      if ((wrapJointDelta(q - u)).norm() < 1e-9) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) uniq.push_back(q);
  }
  return uniq;
}

std::vector<Eigen::VectorXd> buildGlobalSeedList(const RobotModel& robot) {
  std::vector<Eigen::VectorXd> seeds;
  Eigen::VectorXd qmid(6);
  for (const auto& seg : robot.segments) {
    if (!seg.movable) continue;
    qmid(seg.q_index) = 0.5 * (seg.lower + seg.upper);
  }
  seeds.push_back(qmid);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  for (int s = 0; s < 96; ++s) {
    Eigen::VectorXd q(6);
    for (const auto& seg : robot.segments) {
      if (!seg.movable) continue;
      q(seg.q_index) = seg.lower + (seg.upper - seg.lower) * unif(rng);
    }
    seeds.push_back(q);
  }
  return seeds;
}

// --- KDL LMA backend ---

namespace {

Mat4 kdlFrameToMat4(const KDL::Frame& f) {
  Mat4 T = Mat4::Identity();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) T(r, c) = f.M(r, c);
  T(0, 3) = f.p(0);
  T(1, 3) = f.p(1);
  T(2, 3) = f.p(2);
  return T;
}

}  // namespace

CandidateInfo solveSinglePoseKdl(const RobotModel& robot,
                                  const std::vector<BasinBox>& basin_boxes,
                                  const SolverConfig& cfg,
                                  const Mat4& target, const Eigen::VectorXd& seed,
                                  const Eigen::VectorXd& q_prev,
                                  const Eigen::VectorXd& dq_prev,
                                  const std::array<double, 6>& weights,
                                  double orient_limit) {
  CandidateInfo cand;
  if (!robot.kdl_chain || robot.kdl_chain->getNrOfJoints() != 6) {
    cand.valid = false;
    cand.failure_reason = "KDL chain not available";
    return cand;
  }

  // Transform target TCP → target wrist_3
  Mat4 target_wrist3 = target * robot.T_wrist3_to_tcp.inverse();

  // Convert to KDL Frame
  KDL::Frame target_frame;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) target_frame.M(r, c) = target_wrist3(r, c);
  target_frame.p(0) = target_wrist3(0, 3);
  target_frame.p(1) = target_wrist3(1, 3);
  target_frame.p(2) = target_wrist3(2, 3);

  // Convert seed to KDL JntArray
  KDL::JntArray q_init(6);
  Eigen::VectorXd seed_clamped = clampToLimits(robot, seed);
  for (int i = 0; i < 6; ++i) q_init(i) = seed_clamped(i);

  // Build weight vector for KDL LMA
  // KDL's L vector: sqrt of task-space weights. Higher = more important.
  // Our weights: [pos, pos, pos, rot, rot, rot] where 1 = important, 0 = ignore.
  // Scale: pos weight 1 → L=1, rot weight 0.03 → L=sqrt(0.03)≈0.173
  Eigen::Matrix<double, 6, 1> L;
  for (int k = 0; k < 6; ++k) {
    L(k) = std::sqrt(std::max(weights[k], 1e-6));
  }

  KDL::ChainIkSolverPos_LMA solver(*robot.kdl_chain, L, 1e-6, 200, 1e-7);
  KDL::JntArray q_out(6);
  int rc = solver.CartToJnt(q_init, target_frame, q_out);

  if (rc < 0) {
    cand.valid = false;
    cand.failure_reason = "KDL_LMA:" + std::string(solver.strError(rc));
    return cand;
  }

  // Convert result back to Eigen, align to reference
  Eigen::VectorXd q_kdl(6);
  for (int i = 0; i < 6; ++i) q_kdl(i) = q_out(i);
  q_kdl = clampToLimits(robot, q_kdl);
  q_kdl = alignToReference(robot, q_kdl, q_prev + dq_prev);

  // Evaluate quality (collision deferred to caller)
  Mat4 T_tcp = tipTransform(robot, q_kdl);
  Vec6 err = poseError(T_tcp, target);

  cand.q = q_kdl;
  cand.pos_err = err.head<3>().norm();
  cand.rot_err = err.tail<3>().norm();
  cand.clearance = std::numeric_limits<double>::infinity();  // deferred
  cand.cost = continuityCost(q_kdl, q_prev, dq_prev);
  cand.joint_cost = cand.cost;
  cand.iterations_used = 1;
  cand.valid = true;
  return cand;
}

}  // namespace rtfg
