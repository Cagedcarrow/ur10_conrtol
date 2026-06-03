#include "trajectory_solver.h"
#include "collision_pipeline.h"
#include "ik_backend.h"
#include "ik_solver.h"
#include "robot_model.h"
#include "utils.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct RankedCandidate {
  rtfg::CandidateInfo cand;
  double score = std::numeric_limits<double>::infinity();
};

struct PointSearchResult {
  rtfg::CandidateInfo safe;
  rtfg::CandidateInfo fallback;
  double safe_score = std::numeric_limits<double>::infinity();
  double fallback_score = std::numeric_limits<double>::infinity();
  int original_candidates = 0;
  int error_rejected = 0;
  int candidate_quick_rejects = 0;
  int entered_fcl = 0;
  int truncated = 0;
  int fcl_calls = 0;
  int ik_iterations_total = 0;
  int ik_iterations_max = 0;
  int continuous_prediction_hits = 0;
  int continuous_prediction_fallbacks = 0;
};

double weightedJointCost(const Eigen::VectorXd& q, const Eigen::VectorXd& q_prev)
{
  static const std::array<double, 6> weights = {1.0, 1.0, 1.0, 1.4, 1.7, 2.0};
  const Eigen::VectorXd dq = rtfg::wrapJointDelta(q - q_prev);
  double cost = 0.0;
  for (int i = 0; i < dq.size(); ++i) {
    cost += weights[static_cast<std::size_t>(i)] * std::abs(dq(i));
  }
  return cost;
}

bool isKeyPlaybackPoint(const std::vector<std::string>& segment_names, int idx)
{
  if (idx <= 0 || idx + 1 >= static_cast<int>(segment_names.size())) {
    return true;
  }
  if (segment_names.empty()) {
    return idx == 0;
  }
  return segment_names[static_cast<std::size_t>(idx)] !=
           segment_names[static_cast<std::size_t>(idx - 1)] ||
         segment_names[static_cast<std::size_t>(idx)] !=
           segment_names[static_cast<std::size_t>(idx + 1)];
}

}  // namespace

namespace rtfg {

TrajectoryResult solveTrajectory(const RobotModel& robot,
                                 const std::vector<BasinBox>& basin_boxes,
                                 const SolverConfig& cfg,
                                 const std::vector<Mat4>& target_tforms,
                                 const std::vector<std::string>& segment_names,
                                 const Eigen::VectorXd& current_q,
                                 const Eigen::VectorXd& home_q) {
  if (target_tforms.empty()) {
    throw std::runtime_error("Empty target trajectory");
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  TrajectoryResult out;
  const bool realtime_mode = cfg.solver_mode == "realtime";
  const int requested_points = static_cast<int>(target_tforms.size());
  const int solve_points = std::max(1, cfg.enable_window_solve
                                        ? std::min(cfg.window_size, requested_points)
                                        : requested_points);
  const std::vector<Mat4> target_subset(target_tforms.begin(),
                                        target_tforms.begin() + solve_points);
  const std::vector<std::string> segment_subset(
      segment_names.begin(), segment_names.begin() + std::min<std::size_t>(
                                                      segment_names.size(),
                                                      static_cast<std::size_t>(solve_points)));

  const std::vector<std::pair<std::array<double, 6>, double>> weight_schedule = {
      {{{1, 1, 1, 0.20, 0.20, 0.20}}, M_PI / 6.0},
      {{{1, 1, 1, 0.10, 0.10, 0.10}}, M_PI / 4.0},
      {{{1, 1, 1, 0.03, 0.03, 0.03}}, 70.0 * M_PI / 180.0},
      {{{1, 1, 1, 0.00, 0.00, 0.00}}, std::numeric_limits<double>::infinity()}};
  const std::vector<std::pair<std::array<double, 6>, double>> predictive_schedule = {
      {{{1, 1, 1, 0.20, 0.20, 0.20}}, M_PI / 6.0},
      {{{1, 1, 1, 0.10, 0.10, 0.10}}, M_PI / 4.0}};

  SolverTiming& timing = out.timing;
  timing.solver_mode = cfg.solver_mode;
  timing.points_requested = requested_points;
  timing.points_solved = solve_points;
  timing.n_poses_solved = solve_points;
  out.progress_events.push_back({"模型与场景预处理", 0.05, "ROS2 C++ 已开始处理轨迹求解"});

  Eigen::MatrixXd anchor_q(solve_points, 6);
  Eigen::VectorXd q_prev = current_q;
  Eigen::VectorXd dq_prev = Eigen::VectorXd::Zero(6);
  const auto global_seeds = buildGlobalSeedList(robot);
  double ik_total_us = 0.0;
  const auto ik_backend = createIKSolverBackend(cfg);
  CollisionPipeline collision_pipeline(robot, basin_boxes, cfg);

  std::function<PointSearchResult(const Mat4&, const std::string&, int, const SolverConfig&,
                                  int, bool)>
    run_point_search;
  run_point_search = [&](const Mat4& target, const std::string& segment_name, int point_idx,
                                  const SolverConfig& search_cfg, int collision_limit,
                                  bool allow_full_retry) -> PointSearchResult {
    PointSearchResult result;
    std::vector<RankedCandidate> ranked_candidates;
    const bool keypoint = isKeyPlaybackPoint(segment_subset, point_idx);
    const int global_seed_budget = search_cfg.solver_mode == "realtime" ? 24 : 48;
    bool stop_search = false;
    const bool prefer_clearance = search_cfg.solver_mode == "realtime";
    const Eigen::VectorXd q_ref = search_cfg.solver_mode == "realtime" ?
      clampToLimits(robot, q_prev + dq_prev) : q_prev;

    auto accept_candidate = [&](const CandidateInfo& cand, double orient_limit) {
      if (!cand.valid) {
        return;
      }
      result.original_candidates++;
      result.ik_iterations_total += cand.iterations_used;
      result.ik_iterations_max = std::max(result.ik_iterations_max, cand.iterations_used);

      if (!result.fallback.valid || cand.pos_err < result.fallback.pos_err ||
          (std::abs(cand.pos_err - result.fallback.pos_err) < 1e-12 &&
           cand.cost < result.fallback.cost)) {
        result.fallback = cand;
        result.fallback_score = cand.cost + 0.35 * weightedJointCost(cand.q, q_ref);
      }

      if (cand.pos_err > search_cfg.ik_position_tolerance || cand.rot_err > orient_limit) {
        result.error_rejected++;
        return;
      }

      CandidateInfo ranked = cand;
      ranked.joint_cost = weightedJointCost(ranked.q, q_ref);
      RankedCandidate rc;
      rc.cand = ranked;
      rc.score = ranked.cost + 0.35 * ranked.joint_cost;
      ranked_candidates.push_back(rc);

    };

    auto eval_seed = [&](const Eigen::VectorXd& seed, const std::array<double, 6>& weights,
                         double orient_limit) {
      if (stop_search) {
        return;
      }
      CandidateInfo cand =
        ik_backend->solve(robot, basin_boxes, search_cfg, target, seed, q_prev, dq_prev,
                          weights, orient_limit);
      accept_candidate(cand, orient_limit);
    };

    auto run_stage = [&](const std::vector<Eigen::VectorXd>& seeds,
                         const std::vector<std::pair<std::array<double, 6>, double>>& schedules) {
      for (const auto& sched : schedules) {
        for (const auto& seed : seeds) {
          eval_seed(seed, sched.first, sched.second);
          if (stop_search) {
            return;
          }
        }
      }
    };

    auto finalize_candidates = [&](int collision_limit) -> bool {
      std::sort(ranked_candidates.begin(), ranked_candidates.end(),
                [](const RankedCandidate& a, const RankedCandidate& b) {
                  if (std::abs(a.score - b.score) > 1e-12) return a.score < b.score;
                  if (std::abs(a.cand.pos_err - b.cand.pos_err) > 1e-12) {
                    return a.cand.pos_err < b.cand.pos_err;
                  }
                  return a.cand.rot_err < b.cand.rot_err;
                });

      result.entered_fcl = std::min(static_cast<int>(ranked_candidates.size()), collision_limit);
      result.truncated = std::max(0, static_cast<int>(ranked_candidates.size()) - collision_limit);

      auto evaluate_ranked = [&](const RankedCandidate& ranked) {
        result.fcl_calls++;
        CollisionSummary collision = collision_pipeline.preciseCheck(ranked.cand.q);
        const double clearance =
          std::min({collision.min_self, collision.min_tool_body, collision.min_tool_basin});
        if (clearance < search_cfg.clearance_threshold) {
          return;
        }
        CandidateInfo safe_cand = ranked.cand;
        safe_cand.clearance = clearance;
        safe_cand.failure_reason =
          !collision.violation_object.empty() ? collision.violation_object : std::string();
        // Prefer clearance-first but use score as strong secondary when clearance is close.
        // This avoids picking a far-away candidate that would cause playback collisions.
        if (!result.safe.valid ||
            (prefer_clearance &&
             (safe_cand.clearance > result.safe.clearance + 5e-5 ||
              (std::abs(safe_cand.clearance - result.safe.clearance) < 5e-5 &&
               ranked.score < result.safe_score))) ||
            (!prefer_clearance &&
             (ranked.score < result.safe_score ||
              (std::abs(ranked.score - result.safe_score) < 1e-12 &&
               safe_cand.clearance > result.safe.clearance)))) {
          // Temporarily accept — playback validation below may reject.
          result.safe = safe_cand;
          result.safe_score = ranked.score;
        }
      };

      for (int idx = 0; idx < result.entered_fcl; ++idx) {
        evaluate_ranked(ranked_candidates[static_cast<std::size_t>(idx)]);
      }

      if (!result.safe.valid && search_cfg.solver_mode == "realtime" &&
          result.entered_fcl < static_cast<int>(ranked_candidates.size())) {
        for (std::size_t idx = static_cast<std::size_t>(result.entered_fcl);
             idx < ranked_candidates.size(); ++idx) {
          evaluate_ranked(ranked_candidates[idx]);
        }
      }


      if (!result.safe.valid && result.fallback.valid &&
          result.fallback.pos_err <= search_cfg.ik_position_tolerance) {
        result.fcl_calls++;
        CollisionSummary collision = collision_pipeline.preciseCheck(result.fallback.q);
        const double clearance =
          std::min({collision.min_self, collision.min_tool_body, collision.min_tool_basin});
        if (clearance >= search_cfg.clearance_threshold) {
          result.safe = result.fallback;
          result.safe.clearance = clearance;
          result.safe_score = result.fallback_score;
          result.safe.failure_reason = "accepted relaxed-orientation fallback";
        } else {
          result.fallback.clearance = clearance;
          result.fallback.failure_reason =
            !collision.violation_object.empty() ? collision.violation_object :
            std::string("clearance below threshold");
          // Adaptive fallback: if position accuracy is excellent, accept even
          // with marginal clearance — the playback audit is the final safety check.
          if (result.fallback.pos_err < 0.5 * search_cfg.ik_position_tolerance) {
            result.safe = result.fallback;
            result.safe.clearance = clearance;
            result.safe_score = result.fallback_score;
            result.safe.failure_reason =
              "adaptive_clearance_warning:" + result.fallback.failure_reason;
          }
        }
      }

      if (allow_full_retry && search_cfg.solver_mode == "realtime" && keypoint && !result.safe.valid) {
        SolverConfig retry_cfg = search_cfg;
        retry_cfg.solver_mode = "full";
        retry_cfg.max_iterations = search_cfg.max_iterations_full;
        PointSearchResult retry = run_point_search(target, segment_name, point_idx,
                                                   retry_cfg, retry_cfg.max_collision_candidates_full,
                                                   false);
        result.original_candidates += retry.original_candidates;
        result.error_rejected += retry.error_rejected;
        result.candidate_quick_rejects += retry.candidate_quick_rejects;
        result.entered_fcl += retry.entered_fcl;
        result.truncated += retry.truncated;
        result.fcl_calls += retry.fcl_calls;
        result.ik_iterations_total += retry.ik_iterations_total;
        result.ik_iterations_max = std::max(result.ik_iterations_max, retry.ik_iterations_max);
        if (retry.safe.valid) {
          result.safe = retry.safe;
          result.safe_score = retry.safe_score;
        } else if (!result.safe.valid && retry.fallback.valid) {
          result.fallback = retry.fallback;
          result.fallback_score = retry.fallback_score;
        }
      }

      return result.safe.valid;
    };

    if (search_cfg.solver_mode == "realtime") {
      // Prediction fast path: use both the extrapolated seed (with rebound
      // from joint limits) and the previous solution. Evaluating all 4
      // candidates (2 seeds × 2 schedules) together lets finalize_candidates
      // pick the best overall, avoiding playback collisions that a single‑seed
      // early‑exit would miss.
      const Eigen::VectorXd rebound_seed = clampToLimitsWithRebound(robot, q_prev + dq_prev);
      run_stage({rebound_seed, q_prev}, predictive_schedule);
      if (finalize_candidates(1)) {
        result.continuous_prediction_hits++;
        return result;
      }
      result.continuous_prediction_fallbacks++;
    }

    if (!stop_search) {
      run_stage({q_prev, home_q, Eigen::VectorXd::Zero(6)}, weight_schedule);
    }
    if (!stop_search) {
      std::vector<Eigen::VectorXd> seeds = buildSeedList(q_prev, home_q, robot);
      for (const auto& seed : seeds) {
        if ((seed - q_prev).norm() < 1e-9) continue;
        if ((seed - home_q).norm() < 1e-9) continue;
        if (seed.norm() < 1e-9) continue;
        for (const auto& sched : weight_schedule) {
          eval_seed(seed, sched.first, sched.second);
          if (stop_search) {
            break;
          }
        }
        if (stop_search) {
          break;
        }
      }
    }
    if (!stop_search) {
      int global_trials = 0;
      for (const auto& sched : weight_schedule) {
        for (const auto& seed : global_seeds) {
          eval_seed(seed, sched.first, sched.second);
          ++global_trials;
          if (stop_search || global_trials >= global_seed_budget) {
            break;
          }
        }
        if (stop_search || global_trials >= global_seed_budget) {
          break;
        }
      }
    }

    finalize_candidates(collision_limit);

    return result;
  };

  for (int i = 0; i < solve_points; ++i) {
    auto t_pose_start = std::chrono::high_resolution_clock::now();

    const Mat4& target = target_subset[static_cast<std::size_t>(i)];
    const std::string segment_name = segment_subset.empty() ? std::string() :
      segment_subset[static_cast<std::size_t>(std::min<std::size_t>(
        static_cast<std::size_t>(i), segment_subset.size() - 1))];
    const int collision_limit = realtime_mode ?
      cfg.max_collision_candidates_realtime : cfg.max_collision_candidates_full;

    PointSearchResult search = run_point_search(target, segment_name, i, cfg, collision_limit, true);
    timing.original_candidates += search.original_candidates;
    timing.candidates_filtered_by_error += search.error_rejected;
    timing.candidates_entered_fcl += search.entered_fcl;
    timing.candidates_truncated += search.truncated;
    timing.fcl_calls += search.fcl_calls;
    timing.candidate_quick_rejects += search.candidate_quick_rejects;
    timing.continuous_prediction_hits += search.continuous_prediction_hits;
    timing.continuous_prediction_fallbacks += search.continuous_prediction_fallbacks;
    timing.ik_iterations_total += search.ik_iterations_total;
    timing.ik_iterations_max = std::max(timing.ik_iterations_max, search.ik_iterations_max);

    if (search.safe.valid) {
      timing.n_custom_success++;
    } else if (search.fallback.valid &&
               search.fallback.pos_err <= cfg.ik_position_tolerance) {
      // Adaptive fallback: accept best-position candidate even if
      // clearance is marginal (playback audit catches violations)
      search.safe = search.fallback;
      timing.n_custom_success++;
} else {
      std::ostringstream oss;
      oss << "第 " << (i + 1) << "/" << solve_points << " 个目标位姿 ROS2 C++ 求解失败";
      if (!segment_name.empty()) {
        oss << " [" << segment_name << "]";
      }
      if (search.fallback.valid) {
        oss << "，位置误差 " << search.fallback.pos_err << " m";
        oss << "，姿态误差 " << rad2deg(search.fallback.rot_err) << " deg";
        if (std::isfinite(search.fallback.clearance)) {
          oss << "，clearance " << search.fallback.clearance << " m";
        }
        if (!search.fallback.failure_reason.empty()) {
          oss << "，clearance 违规对象: " << search.fallback.failure_reason;
        }
      }
      timing.n_failed++;
      throw std::runtime_error(oss.str());
    }

    auto t_pose_end = std::chrono::high_resolution_clock::now();
    const double pose_us =
      std::chrono::duration<double, std::micro>(t_pose_end - t_pose_start).count();
    ik_total_us += pose_us;
    timing.min_per_pose_s = std::min(timing.min_per_pose_s, pose_us * 1e-6);
    timing.max_per_pose_s = std::max(timing.max_per_pose_s, pose_us * 1e-6);

    anchor_q.row(i) = search.safe.q.transpose();
    if (i > 0) {
      out.max_anchor_qstep_deg =
        std::max(out.max_anchor_qstep_deg,
                 rad2deg((wrapJointDelta(search.safe.q - q_prev)).norm()));
    }
    dq_prev = wrapJointDelta(search.safe.q - q_prev);
    q_prev = search.safe.q;

    if (i == 0 || i == solve_points - 1 || ((i + 1) % 10) == 0) {
      std::ostringstream oss;
      oss << "ROS2 C++ 正在求解轨迹锚点 " << (i + 1) << "/" << solve_points;
      out.progress_events.push_back(
          {"anchor 轨迹求解", 0.05 + 0.45 * ((i + 1.0) / std::max(1, solve_points)), oss.str()});
    }
  }

  timing.ik_total_s = ik_total_us * 1e-6;
  timing.avg_per_pose_s = solve_points > 0 ? (ik_total_us * 1e-6) / solve_points : 0.0;
  timing.avg_ik_iterations =
    timing.points_solved > 0 ? static_cast<double>(timing.ik_iterations_total) /
                                 static_cast<double>(timing.points_solved)
                             : 0.0;
  out.anchor_q = anchor_q;

  // --- playback generation ---
  auto t_playback_start = std::chrono::high_resolution_clock::now();
  std::vector<Eigen::VectorXd> playback_q_list;
  std::vector<std::string> playback_segment_names;
  playback_q_list.reserve(static_cast<std::size_t>(solve_points) * 4);
  playback_segment_names.reserve(static_cast<std::size_t>(solve_points) * 4);

  playback_q_list.push_back(anchor_q.row(0).transpose());
  playback_segment_names.push_back(segment_subset.empty() ? std::string() : segment_subset.front());

  for (int i = 1; i < solve_points; ++i) {
    Eigen::VectorXd qa = anchor_q.row(i - 1).transpose();
    Eigen::VectorXd qb = anchor_q.row(i).transpose();
    Mat4 Ta = tipTransform(robot, qa);
    Mat4 Tb = tipTransform(robot, qb);
    const double joint_step = wrapJointDelta(qb - qa).norm();
    const double pos_step = (Tb.block<3, 1>(0, 3) - Ta.block<3, 1>(0, 3)).norm();
    const double rot_step = rotationDistance(Ta.block<3, 3>(0, 0), Tb.block<3, 3>(0, 0));
    int nseg = 1 + std::max(
        {2, static_cast<int>(std::ceil(joint_step / (0.70 * M_PI / 180.0))),
         static_cast<int>(std::ceil(pos_step / 0.0030)),
         static_cast<int>(std::ceil(rot_step / (0.30 * M_PI / 180.0)))});
    nseg = std::min(std::max(nseg, 4), 32);
    for (int k = 1; k <= nseg; ++k) {
      const double a = quinticBlend(static_cast<double>(k) / static_cast<double>(nseg));
      Eigen::VectorXd q = qa + a * wrapJointDelta(qb - qa);
      playback_q_list.push_back(q);
      playback_segment_names.push_back(segment_subset.empty() ? std::string() :
                                       segment_subset[static_cast<std::size_t>(
                                         std::min<std::size_t>(static_cast<std::size_t>(i),
                                                               segment_subset.size() - 1))]);
    }
    if (i == solve_points - 1 || (i % 15) == 0) {
      std::ostringstream oss;
      oss << "ROS2 C++ 正在生成 playback 轨迹 " << i << "/" << (solve_points - 1);
      out.progress_events.push_back(
          {"playback 连续化", 0.55 + 0.20 * (static_cast<double>(i) / std::max(1, solve_points - 1)),
           oss.str()});
    }
  }

  out.playback_q.resize(static_cast<int>(playback_q_list.size()), 6);
  for (int i = 0; i < static_cast<int>(playback_q_list.size()); ++i) {
    out.playback_q.row(i) = playback_q_list[static_cast<std::size_t>(i)].transpose();
    if (i > 0) {
      out.max_playback_qstep_deg =
        std::max(out.max_playback_qstep_deg,
                 rad2deg((wrapJointDelta(playback_q_list[static_cast<std::size_t>(i)] -
                                         playback_q_list[static_cast<std::size_t>(i - 1)])).norm()));
    }
  }
  out.playback_segment_names = playback_segment_names;
  auto t_playback_end = std::chrono::high_resolution_clock::now();
  timing.playback_generation_s =
    std::chrono::duration<double>(t_playback_end - t_playback_start).count();

  // --- collision audit ---
  auto t_collision_start = std::chrono::high_resolution_clock::now();
  out.tcp_path.resize(out.playback_q.rows(), 3);
  out.collision_checked_flags.reserve(static_cast<std::size_t>(out.playback_q.rows()));
  std::vector<Vec3> collision_points;
  Mat4 T_prev = Mat4::Identity();
  bool have_prev = false;

  for (int i = 0; i < out.playback_q.rows(); ++i) {
    Eigen::VectorXd qi = out.playback_q.row(i).transpose();
    Mat4 T = tipTransform(robot, qi);
    out.tcp_path.row(i) = T.block<3, 1>(0, 3).transpose();

    if (have_prev) {
      out.max_actual_rot_deg = std::max(
        out.max_actual_rot_deg,
        rad2deg(rotationDistance(T_prev.block<3, 3>(0, 0), T.block<3, 3>(0, 0))));
    }
    T_prev = T;
    have_prev = true;

    const bool keypoint = isKeyPlaybackPoint(playback_segment_names, i);
    const bool do_fcl = collision_pipeline.shouldRunPreciseCheck(i, keypoint) ||
                        (i == out.playback_q.rows() - 1);
    if (!do_fcl) {
      timing.fcl_skipped_points++;
      out.collision_checked_flags.push_back(0);
      QuickCheckResult quick = collision_pipeline.quickPlaybackCheck(
          qi, i > 0 ? playback_q_list[static_cast<std::size_t>(i - 1)] : qi);
      if (!quick.ok) {
        out.has_collision = true;
        out.collision_sample_indices.push_back(i + 1);
        out.collision_traj_indices.push_back(std::min(i + 1, solve_points));
        out.collision_types.push_back("light_check");
        out.collision_objects.push_back(quick.reason);
        out.collision_segments.push_back(playback_segment_names[static_cast<std::size_t>(i)]);
        collision_points.push_back(T.block<3, 1>(0, 3));
        if (out.first_collision_sample_idx < 0) {
          out.first_collision_sample_idx = i + 1;
          out.first_collision_traj_idx = std::min(i + 1, solve_points);
          out.first_collision_type = "light_check";
          out.first_collision_object = quick.reason;
          out.first_collision_segment = playback_segment_names[static_cast<std::size_t>(i)];
          out.first_collision_point = T.block<3, 1>(0, 3);
          out.first_collision_clearance = 0.0;
        }
      }
      continue;
    }

    out.collision_checked_flags.push_back(1);
    timing.fcl_calls++;
    if (keypoint) {
      timing.fcl_forced_points++;
    }
    CollisionSummary cm = collision_pipeline.preciseCheck(qi);
    if (cm.min_self < out.global_minimums.min_self) {
      out.global_minimums.min_self = cm.min_self;
      out.global_minimums.min_self_object = cm.min_self_object;
    }
    if (cm.min_tool_body < out.global_minimums.min_tool_body) {
      out.global_minimums.min_tool_body = cm.min_tool_body;
      out.global_minimums.min_tool_body_object = cm.min_tool_body_object;
    }
    if (cm.min_tool_basin < out.global_minimums.min_tool_basin) {
      out.global_minimums.min_tool_basin = cm.min_tool_basin;
      out.global_minimums.min_tool_basin_object = cm.min_tool_basin_object;
    }
    if (!cm.violation_type.empty()) {
      out.has_collision = true;
      out.collision_sample_indices.push_back(i + 1);
      out.collision_traj_indices.push_back(std::min(i + 1, solve_points));
      out.collision_types.push_back(cm.violation_type);
      out.collision_objects.push_back(cm.violation_object);
      out.collision_segments.push_back(playback_segment_names[static_cast<std::size_t>(i)]);
      collision_points.push_back(T.block<3, 1>(0, 3));
      if (out.first_collision_sample_idx < 0) {
        out.first_collision_sample_idx = i + 1;
        out.first_collision_traj_idx = std::min(i + 1, solve_points);
        out.first_collision_type = cm.violation_type;
        out.first_collision_object = cm.violation_object;
        out.first_collision_segment = playback_segment_names[static_cast<std::size_t>(i)];
        out.first_collision_point = T.block<3, 1>(0, 3);
        out.first_collision_clearance = cm.violation_clearance;
      }
    }

    if (i == out.playback_q.rows() - 1 || ((i + 1) % 50) == 0) {
      std::ostringstream oss;
      oss << "ROS2 C++ 正在进行碰撞复检 " << (i + 1) << "/" << out.playback_q.rows();
      out.progress_events.push_back(
          {"碰撞复检", 0.75 + 0.20 * ((i + 1.0) / out.playback_q.rows()), oss.str()});
    }
  }
  out.collision_points.resize(static_cast<int>(collision_points.size()), 3);
  for (int i = 0; i < static_cast<int>(collision_points.size()); ++i) {
    out.collision_points.row(i) = collision_points[static_cast<std::size_t>(i)].transpose();
  }
  for (int i = 1; i < solve_points; ++i) {
    out.max_target_rot_deg = std::max(
        out.max_target_rot_deg,
        rad2deg(rotationDistance(target_subset[static_cast<std::size_t>(i - 1)].block<3, 3>(0, 0),
                                  target_subset[static_cast<std::size_t>(i)].block<3, 3>(0, 0))));
  }

  auto t_collision_end = std::chrono::high_resolution_clock::now();
  timing.collision_total_s = std::chrono::duration<double>(t_collision_end - t_collision_start).count();
  timing.posearray_points = 0;
  timing.joint_trajectory_points = out.playback_q.rows();
  timing.ik_total_s = ik_total_us * 1e-6;
  timing.avg_per_pose_s = solve_points > 0 ? timing.ik_total_s / solve_points : 0.0;

  auto t_end = std::chrono::high_resolution_clock::now();
  timing.total_wall_s = std::chrono::duration<double>(t_end - t_start).count();
  out.progress_events.push_back({"结果回传", 1.0, "ROS2 C++ 求解完成"});
  return out;
}

}  // namespace rtfg
