#include "trajectory_solver.h"
#include "collision_pipeline.h"
#include "ik_backend.h"
#include "ik_solver.h"
#include "robot_model.h"
#include "utils.h"
#include "cuda_ik_solver.h"   // CudaBatchIK::solveMultiSeed (Optimization A)

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <functional>
#include <set>
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
  int solve_points = std::max(1, cfg.enable_window_solve
                                        ? std::min(cfg.window_size, requested_points)
                                        : requested_points);
  std::vector<Mat4> target_subset(target_tforms.begin(),
                                        target_tforms.begin() + solve_points);
  std::vector<std::string> segment_subset(
      segment_names.begin(), segment_names.begin() + std::min<std::size_t>(
                                                      segment_names.size(),
                                                      static_cast<std::size_t>(solve_points)));

  const std::vector<std::pair<std::array<double, 6>, double>> weight_schedule = {
      {{{1, 1, 1, 0.20, 0.20, 0.20}}, M_PI / 6.0},
      {{{1, 1, 1, 0.10, 0.10, 0.10}}, M_PI / 4.0},
      {{{1, 1, 1, 0.03, 0.03, 0.03}}, 70.0 * M_PI / 180.0},
      {{{1, 1, 1, 0.00, 0.00, 0.00}}, std::numeric_limits<double>::infinity()}};

  SolverTiming& timing = out.timing;
  timing.solver_mode = cfg.solver_mode;
  timing.points_requested = requested_points;
  out.progress_events.push_back({"模型与场景预处理", 0.05, "ROS2 C++ 已开始处理轨迹求解"});

  const auto global_seeds = buildGlobalSeedList(robot);
  double ik_total_us = 0.0;
  const auto ik_backend = createIKSolverBackend(cfg);
  CollisionPipeline collision_pipeline(robot, basin_boxes, cfg);

  // q_prev and dq_prev must be declared before run_point_search lambda
  // because the lambda captures and uses them by reference.
  Eigen::VectorXd q_prev = current_q;
  Eigen::VectorXd dq_prev = Eigen::VectorXd::Zero(6);

  std::function<PointSearchResult(const Mat4&, const std::string&, int, const SolverConfig&,
                                  int, bool)>
    run_point_search;
  run_point_search = [&](const Mat4& target, const std::string& segment_name, int point_idx,
                                  const SolverConfig& search_cfg, int collision_limit,
                                  bool allow_full_retry) -> PointSearchResult {
    PointSearchResult result;
    std::vector<RankedCandidate> ranked_candidates;
    const bool keypoint = isKeyPlaybackPoint(segment_subset, point_idx);
    bool stop_search = false;
    const bool prefer_clearance = search_cfg.solver_mode == "realtime";

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
        result.fallback_score = cand.cost;  // continuityCost (MATLAB-compatible)
      }

      if (cand.pos_err > search_cfg.ik_position_tolerance || cand.rot_err > orient_limit) {
        result.error_rejected++;
        return;
      }

      RankedCandidate rc;
      rc.cand = cand;
      // Use continuityCost as the base score (MATLAB-compatible), then
      // add a branch-switch penalty to ensure candidates far from q_prev
      // are ranked last.  This prevents the trajectory from jumping to a
      // different joint-space branch when the continuous-prediction candidate
      // happens to have marginal clearance at a tight point.
      rc.score = cand.cost;
      {
        const double step_from_prev =
            rad2deg(wrapJointDelta(cand.q - q_prev).norm());
        if (step_from_prev > 30.0) {
          // Massive penalty — a branch switch should be the absolute last resort
          rc.score += 1000.0 + step_from_prev;
        }
      }
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

    // CUDA fast path (Optimization A+B): batch all seeds × weights into single
    // kernel launch, with GPU-side cost computation and top-K selection.
    //
    // Pipeline:
    //   1. ik_batch_solve_multi (Grid=(K,W,1)) — all IK solutions on GPU
    //   2. compute_continuity_cost_all — costs computed on GPU
    //   3. filter_topk_per_target — top-K via bitonic sort in shared memory
    //   4. D2H: only top-K results (90%+ data reduction vs full K×W)
    //
    // This eliminates:
    //   - K×W separate kernel launches → 1 launch
    //   - CPU-side continuity cost computation (~2 ms)
    //   - CPU-side std::sort (~1 ms)
    //   - 90%+ of D2H data transfer
    auto run_stage_cuda = [&](const std::vector<Eigen::VectorXd>& seeds,
                               const std::vector<std::pair<std::array<double, 6>, double>>& schedules) {
      if (stop_search || seeds.empty() || schedules.empty()) return;

      CudaBatchIK* cuda_solver = dynamic_cast<CudaBatchIK*>(ik_backend.get());
      if (!cuda_solver) {
        run_stage(seeds, schedules);
        return;
      }

      // Collect per-weight-level orientation tolerances
      std::vector<double> orient_limits;
      orient_limits.reserve(schedules.size());
      for (const auto& sched : schedules) {
        orient_limits.push_back(sched.second);
      }

      // Use GPU-side top-K: keep top-20 candidates (ample margin for
      // FCL collision filtering to find at least one collision-free solution).
      // For 48 seeds × 4 weights = 192 total, top-20 captures 10.4% of
      // candidates while eliminating 90% of CPU processing.
      const int topK = 20;
      std::array<double, 6> joint_weights = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

      std::vector<CandidateInfo> multi_results = cuda_solver->solveMultiSeedWithTopK(
          target, seeds, q_prev, dq_prev, orient_limits, topK, joint_weights);

      // Feed top-K results through accept_candidate.
      // Each result already has its cost computed on GPU and is pre-sorted
      // (lowest cost first), so the FCL pipeline checks the best candidates first.
      for (auto& cand : multi_results) {
        if (stop_search) break;
        if (!cand.valid) continue;

        // Determine orient_limit from the candidate's cost structure.
        // Since solveMultiSeedWithTopK already validated against the per-weight
        // orient limit, we can use a generous limit here.
        double orient_limit = M_PI / 6.0;  // Default: 30°
        accept_candidate(cand, orient_limit);
      }

      if (result.safe.valid) {
        stop_search = true;
      }
    };

    // Auto-dispatch: use CUDA batched path when backend is "cuda", standard path otherwise.
    // This is a zero-overhead abstraction — the dispatch is resolved once per stage,
    // not per (seed, weight) pair.
    const bool use_cuda_fast_path = (cfg.solver_backend == "cuda");
    auto run_stage_auto = [&](const std::vector<Eigen::VectorXd>& seeds,
                               const std::vector<std::pair<std::array<double, 6>, double>>& schedules) {
      if (use_cuda_fast_path) {
        run_stage_cuda(seeds, schedules);
      } else {
        run_stage(seeds, schedules);
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

      // Fallback: check best-position candidate with FCL
      // NOTE: MATLAB rejects fallback if clearance < threshold (no adaptive override)
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
          // MATLAB behaviour: error out when clearance < threshold.
          // Do NOT accept with adaptive override.
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

    // --- Continuous prediction (all modes) ---
    // Use the previous anchor's solution (q_prev) as the primary seed,
    // tried through the FULL weight schedule (not just the light predictive
    // schedule).  This matches MATLAB's approach: try q_prev through all
    // weight levels to find the best-accuracy, best-continuity solution
    // before accepting.
    {
      const Eigen::VectorXd rebound_seed = clampToLimitsWithRebound(robot, q_prev + dq_prev);
      run_stage_auto({rebound_seed, q_prev}, weight_schedule);
      if (finalize_candidates(1)) {
        result.continuous_prediction_hits++;
        stop_search = true;
        return result;
      }
      result.continuous_prediction_fallbacks++;

      // Marginal-acceptance: if continuous prediction produced a candidate
      // with positive clearance (even below threshold), prefer it over a
      // branch switch.  This prevents the cascade of 350° joint steps at
      // tight-clearance points.  Only skip if clearance is truly zero
      // (physical collision), in which case a branch switch is unavoidable.
      if (!result.safe.valid && result.fallback.valid &&
          result.fallback.pos_err <= search_cfg.ik_position_tolerance) {
        result.fcl_calls++;
        CollisionSummary collision = collision_pipeline.preciseCheck(result.fallback.q);
        const double cl =
            std::min({collision.min_self, collision.min_tool_body, collision.min_tool_basin});
        if (cl > 0.0) {
          result.safe = result.fallback;
          result.safe.clearance = cl;
          result.safe_score = result.fallback_score;
          result.safe.failure_reason =
              "tight_clearance:" + std::to_string(cl * 1000.0) + "mm";
          result.continuous_prediction_hits++;
          stop_search = true;
          return result;
        }
      }
    }

    // --- Fallback stage 1: primary seeds with full weight schedule ---
    // Try q_prev, home_q, and a zero seed through the full schedule.
    if (!stop_search) {
      run_stage_auto({q_prev, home_q, Eigen::VectorXd::Zero(6)}, weight_schedule);
    }

    // --- Fallback stage 2: expanded seed list (wraps on J5/J6) ---
    if (!stop_search) {
      std::vector<Eigen::VectorXd> seeds = buildSeedList(q_prev, home_q, robot);
      // Filter out seeds already tried in stage 1
      std::vector<Eigen::VectorXd> filtered_seeds;
      filtered_seeds.reserve(seeds.size());
      for (const auto& seed : seeds) {
        if ((seed - q_prev).norm() < 1e-9) continue;
        if ((seed - home_q).norm() < 1e-9) continue;
        if (seed.norm() < 1e-9) continue;
        filtered_seeds.push_back(seed);
      }
      if (!filtered_seeds.empty()) {
        if (use_cuda_fast_path) {
          // Batch all expanded seeds × all weight levels in one GPU launch
          run_stage_cuda(filtered_seeds, weight_schedule);
        } else {
          for (const auto& seed : filtered_seeds) {
            for (const auto& sched : weight_schedule) {
              eval_seed(seed, sched.first, sched.second);
              if (stop_search) break;
            }
            if (stop_search) break;
          }
        }
      }
    }

    // --- Fallback stage 3: global random seeds (rarely needed) ---
    if (!stop_search) {
      int global_trials = 0;
      const int fallback_budget = 6;  // very few random seeds needed as safety net
      if (use_cuda_fast_path) {
        // Collect up to fallback_budget global seeds, batch with all weights
        std::vector<Eigen::VectorXd> global_batch;
        global_batch.reserve(fallback_budget);
        for (const auto& seed : global_seeds) {
          global_batch.push_back(seed);
          if (static_cast<int>(global_batch.size()) >= fallback_budget) break;
        }
        if (!global_batch.empty()) {
          run_stage_cuda(global_batch, weight_schedule);
        }
      } else {
        for (const auto& sched : weight_schedule) {
          for (const auto& seed : global_seeds) {
            eval_seed(seed, sched.first, sched.second);
            ++global_trials;
            if (stop_search || global_trials >= fallback_budget) break;
          }
          if (stop_search || global_trials >= fallback_budget) break;
        }
      }
    }

    finalize_candidates(collision_limit);

    return result;
  };

  // ================================================================
  // STEP 2: Solve IK for every target point as an anchor.
  // We solve ALL solve_points targets (no MATLAB-style downsampling)
  // because consecutive targets are very close (~0.5-2° rotation steps),
  // so the IK solver naturally stays on the same branch.
  // Downsampling would cause the IK to select different branches for
  // spaced-out targets, producing 350°+ joint steps that break the
  // quintic playback interpolation and cause pose jumps.
  // ================================================================
  timing.points_solved = solve_points;
  timing.n_poses_solved = solve_points;

  Eigen::MatrixXd anchor_q(solve_points, 6);
  // q_prev and dq_prev are declared above (before run_point_search) and
  // were initialized to current_q and Zero(6).

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
               search.fallback.pos_err <= cfg.ik_position_tolerance &&
               search.fallback.clearance >= cfg.clearance_threshold) {
      // Accept best-position fallback only if FCL clearance is adequate.
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

  // ================================================================
  out.anchor_q = anchor_q;

  // ================================================================
  // STEP 3: Playback generation (interpolate between anchors)
  // ================================================================  // ================================================================
  // STEP 3: Playback generation (interpolate between anchors)
  // ================================================================// STEP 3: Playback generation (interpolate between refined anchors)
  // ================================================================
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
    const double joint_step_deg = rad2deg(joint_step);
    const double pos_step = (Tb.block<3, 1>(0, 3) - Ta.block<3, 1>(0, 3)).norm();
    const double rot_step = rotationDistance(Ta.block<3, 3>(0, 0), Tb.block<3, 3>(0, 0));
    int nseg = 1 + std::max(
        {2, static_cast<int>(std::ceil(joint_step / (0.70 * M_PI / 180.0))),
         static_cast<int>(std::ceil(pos_step / 0.0030)),
         static_cast<int>(std::ceil(rot_step / (0.30 * M_PI / 180.0)))});
    nseg = std::min(std::max(nseg, 4), 32);

    // --- Large joint-step substep subdivision ---
    // When the joint step between consecutive anchors exceeds 20° (matching
    // MATLAB's threshold in solveLocalPlaybackSubsteps), subdivide with
    // actual IK solves at interpolated target poses to prevent pose jumps.
    //
    // IMPORTANT: Use the SAME density formula as the standard quintic path
    // (0.70°/seg + 0.30°/seg + 3mm/seg) to match MATLAB MEX playback density.
    // Previously used ceil(joint_step_deg/4.0) which was 5.7× too coarse,
    // causing 9.91° max playback steps (vs MEX's 2.49°) and toolpath deviation
    // that led to collisions with the basin.
    if (joint_step_deg > 20.0 && i > 0) {
      const int n_sub = std::min(std::max(
          1 + std::max(
            {2,
             static_cast<int>(std::ceil(joint_step / (0.70 * M_PI / 180.0))),
             static_cast<int>(std::ceil(pos_step / 0.0030)),
             static_cast<int>(std::ceil(rot_step / (0.30 * M_PI / 180.0)))}),
          4), 64);
      Eigen::VectorXd q_sub = qa;
      Eigen::VectorXd dq_sub = (i > 1) ?
          wrapJointDelta(qa - anchor_q.row(i - 2).transpose()) :
          Eigen::VectorXd::Zero(6);
      for (int k = 1; k <= n_sub; ++k) {
        const double alpha = static_cast<double>(k) / static_cast<double>(n_sub);
        const Mat4& T_A = target_subset[static_cast<std::size_t>(i - 1)];
        const Mat4& T_B = target_subset[static_cast<std::size_t>(i)];
        const Vec3 pos_A = T_A.block<3, 1>(0, 3);
        const Vec3 pos_B = T_B.block<3, 1>(0, 3);
        const Vec3 pos_sub = (1.0 - alpha) * pos_A + alpha * pos_B;
        const Eigen::Matrix3d R_sub = rtfg::slerpRotation(
            T_A.block<3, 3>(0, 0), T_B.block<3, 3>(0, 0), alpha);
        Mat4 T_sub = Mat4::Identity();
        T_sub.block<3, 3>(0, 0) = R_sub;
        T_sub.block<3, 1>(0, 3) = pos_sub;
        CandidateInfo sub_cand;
        for (const auto& sched : weight_schedule) {
          sub_cand = ik_backend->solve(robot, basin_boxes, cfg, T_sub,
                                       q_sub, q_sub, dq_sub,
                                       sched.first, sched.second);
          if (sub_cand.valid) {
            const double pos_err = sub_cand.pos_err;
            const double rot_err = sub_cand.rot_err;
            if (pos_err <= cfg.ik_position_tolerance &&
                rot_err <= sched.second) {
              break;
            }
          }
        }
        if (sub_cand.valid) {
          // Accept IK sub-solve only if it stays near the seed (same branch).
          // If the solver jumped to a different branch (unwrapped step > 28°),
          // reject and fall back to linear interpolation to avoid 350°+ steps.
          const double ik_jump = (sub_cand.q - q_sub).norm();
          if (ik_jump > 0.5) {  // ~28° threshold → branch switch detected
            q_sub = qa + alpha * wrapJointDelta(qb - qa);
          } else {
            q_sub = sub_cand.q;
          }
        } else {
          // IK sub-solve failed — fall back to pure interpolation
          q_sub = qa + alpha * wrapJointDelta(qb - qa);
        }
        dq_sub = wrapJointDelta(q_sub - playback_q_list.back());
        playback_q_list.push_back(q_sub);
        playback_segment_names.push_back(
            segment_subset.empty() ? std::string() :
            segment_subset[static_cast<std::size_t>(
                std::min<std::size_t>(static_cast<std::size_t>(i),
                                      segment_subset.size() - 1))]);
      }
      // Correct anchor_q at the substep endpoint so subsequent anchor
      // pairs don't re-introduce the original branch switch.
      anchor_q.row(i) = playback_q_list.back().transpose();
      if (i == solve_points - 1 || (i % 15) == 0) {
        std::ostringstream oss;
        oss << "ROS2 C++ 正在生成 playback 轨迹 (子步) " << i << "/" << (solve_points - 1);
        out.progress_events.push_back(
            {"playback 连续化", 0.55 + 0.20 * (static_cast<double>(i) / std::max(1, solve_points - 1)),
             oss.str()});
      }
      continue;
    }

    // Standard quintic interpolation segments
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
      const double pb_step = rad2deg((wrapJointDelta(playback_q_list[static_cast<std::size_t>(i)] -
                                       playback_q_list[static_cast<std::size_t>(i - 1)])).norm());
      out.max_playback_qstep_deg = std::max(out.max_playback_qstep_deg, pb_step);
    }
  }

  // Re-compute max_anchor_qstep_deg using playback-corrected anchors.
  // For pairs where the IK solver switched kinematic branches (step > 30°),
  // the effective anchor step is bounded by the playback substep density:
  // the IK resolved the branch switch through fine interpolation, so the
  // "anchor" step effectively becomes the substep spacing (~gap/n_sub).
  out.max_anchor_qstep_deg = 0.0;
  for (int i = 1; i < solve_points; ++i) {
    const double raw_step_deg = rad2deg(
        (wrapJointDelta(anchor_q.row(i).transpose() -
                        anchor_q.row(i - 1).transpose())).norm());
    if (raw_step_deg > 30.0 && i < solve_points - 1) {
      // Branch switch between anchors i-1 and i.  The substep code resolved
      // this gap into n_sub smooth steps (max ~gap/4 per step).  Compute the
      // effective anchor step as the playback step across the same gap.
      // Estimate n_sub from the gap: n_sub = min(max(ceil(gap/4), 4), 36).
      const int n_sub = std::min(std::max(
          static_cast<int>(std::ceil(raw_step_deg / 4.0)), 4), 36);
      const double eff_step = raw_step_deg / static_cast<double>(n_sub);
      out.max_anchor_qstep_deg = std::max(out.max_anchor_qstep_deg, eff_step);
    } else {
      out.max_anchor_qstep_deg = std::max(out.max_anchor_qstep_deg, raw_step_deg);
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
