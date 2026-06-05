#pragma once
// cuda_ik_solver.h — CUDA batch IK solver class implementing IKSolverBase

#include "ik_backend.h"       // IKSolverBase (from cpp package)
#include "cuda_memory.h"
#include "cuda_kernels.h"
#include "cuda_collision.h"

#include <cuda_runtime.h>
#include <memory>
#include <vector>

namespace rtfg {

// ============================================================================
// CudaBatchIK — GPU-accelerated batch IK solver
//
// Solves IK for multiple target poses in one GPU kernel launch.
// Mapping: 1 block per target pose, 128 threads/block.
//
// Usage:
//   CudaBatchIK solver;
//   solver.initialize(robot_model, solver_config);  // once
//   for each batch: solver.solveBatch(targets, seeds, q_prev, dq_prev);
// ============================================================================
class CudaBatchIK final : public IKSolverBase {
public:
  CudaBatchIK();
  ~CudaBatchIK() override;

  std::string name() const override { return "cuda"; }

  // Single-pose solve (IKSolverBase interface). Collects one call,
  // defers to batch on flush or explicit solve.
  CandidateInfo solve(const RobotModel& robot,
                      const std::vector<BasinBox>& basin_boxes,
                      const SolverConfig& cfg,
                      const Mat4& target, const Eigen::VectorXd& seed,
                      const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev,
                      const std::array<double, 6>& weights,
                      double orient_limit) const override;

  // ---- Batch API ----

  // Initialize constant memory with robot parameters (call once)
  void initialize(const RobotModel& robot, const SolverConfig& cfg);

  // Add a target pose to the pending batch
  void enqueue(const Mat4& target, const Eigen::VectorXd& seed,
               const std::array<double, 6>& weights, double orient_limit);

  // Execute batch solve on GPU
  void flush(std::vector<CandidateInfo>& results);

  // Convenience: enqueue + flush for single batch
  std::vector<CandidateInfo> solveBatch(
      const std::vector<Mat4>& targets,
      const std::vector<Eigen::VectorXd>& seeds,
      const Eigen::VectorXd& q_prev,
      const Eigen::VectorXd& dq_prev);

  // ---- Multi-Seed Batch API (Optimization A: single kernel launch) ----
  //
  // Solves IK for ONE target pose against ALL seeds × ALL weight levels
  // in a SINGLE kernel launch with Grid=(K, W, 1).
  //
  // This replaces the CPU-side double loop:
  //   for w in 0..W-1: for k in 0..K-1: solve(target, seeds[k], weights[w])
  // which caused up to K×W separate kernel launches (192 for 48 seeds × 4 weights).
  //
  // Returns all K×W CandidateInfo results in a flat vector (row-major:
  // seed-major, weight-minor: result[seed_idx * W + weight_idx]).
  //
  // @param target      Single target pose (4×4)
  // @param all_seeds   Vector of K seed joint angles, each size 6
  // @param q_prev      Previous joint angles (for continuity cost)
  // @param dq_prev     Previous joint velocity (for continuity cost)
  // @param orient_limits Per-weight-level orientation tolerance (size W, radians)
  // @return            K×W CandidateInfo results
  std::vector<CandidateInfo> solveMultiSeed(
      const Mat4& target,
      const std::vector<Eigen::VectorXd>& all_seeds,
      const Eigen::VectorXd& q_prev,
      const Eigen::VectorXd& dq_prev,
      const std::vector<double>& orient_limits);

  // ---- Multi-Seed + GPU Top-K API (Optimization B: GPU sorting) ----
  //
  // Extends solveMultiSeed with GPU-side continuity cost computation and
  // top-K selection via bitonic sort in shared memory.
  //
  // Instead of returning all K×W results, the GPU:
  //   1. Computes continuity cost for all candidates (kernel 4)
  //   2. Bitonic-sorts costs in shared memory (kernel 5)
  //   3. Returns only top-K (lowest cost) candidates
  //
  // This reduces D2H transfer from K×W×6 doubles to topK×6 doubles
  // (e.g., 192 → 20 candidates = 90% reduction) and eliminates CPU-side
  // cost computation and sorting (~3 ms for 52,416 candidates).
  //
  // @param target         Single target pose (4×4)
  // @param all_seeds      Vector of K seed joint angles
  // @param q_prev         Previous joint angles (for continuity cost)
  // @param dq_prev        Previous joint velocity
  // @param orient_limits  Per-weight-level orientation tolerance
  // @param topK           Number of top candidates to return (default 20)
  // @param joint_weights  Joint weights for cost (default all 1.0)
  // @return               Top-K CandidateInfo results (sorted by cost, ascending)
  std::vector<CandidateInfo> solveMultiSeedWithTopK(
      const Mat4& target,
      const std::vector<Eigen::VectorXd>& all_seeds,
      const Eigen::VectorXd& q_prev,
      const Eigen::VectorXd& dq_prev,
      const std::vector<double>& orient_limits,
      int topK = 20,
      const std::array<double, 6>& joint_weights = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

  // Clear pending queue
  void clear();

  // Access the underlying stream for async operations
  cudaStream_t stream() const { return stream_; }

  // ---- GPU Collision Detection API ----

  // Upload environment objects to GPU (call once per environment or on change).
  // env_objects layout: [N_objects × 8] doubles
  //   type 0 = box:     (0, cx, cy, cz, hx, hy, hz, 0)
  //   type 1 = sphere:  (1, cx, cy, cz, r,  0,  0,  0)
  //   type 2 = cylinder:(2, cx, cy, cz, r,  h,  0,  0)
  void uploadEnvironment(const std::vector<double>& env_objects);

  // Upload per-frame robot bounding boxes to GPU (call each frame before check).
  // robot_boxes layout: [N_frames × N_boxes × 6] doubles: (cx, cy, cz, hx, hy, hz)
  void updateRobotBoxesGPU(const std::vector<double>& robot_boxes,
                           int N_frames, int N_boxes);

  // Run GPU collision check for all frames. Must call uploadEnvironment first.
  // Returns (clearances, colliding) vectors of length N_frames.
  // clearances[i] = min clearance for frame i (INF if no objects registered)
  // colliding[i]  = 1.0 if any pair penetrates, 0.0 otherwise
  std::pair<std::vector<double>, std::vector<double>> checkCollisionsGPU(
      int N_frames, int N_boxes);

private:
  // Device memory buffers
  std::unique_ptr<cuda::DeviceBuffer<double>> d_targets_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_seeds_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_results_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_errors_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_shovel_errors_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_iterations_;

  // Multi-seed batch buffers (Optimization A: single launch for K×W grid)
  // d_target_single_:  [16] single target transform
  // d_all_seeds_:      [K * 6] all seeds for one target
  // d_multi_results_:  [K * W * 6] all output joint angles
  // d_multi_errors_:   [K * W * 2] wrist errors for all (seed, weight)
  // d_multi_shovel_:   [K * W * 2] shovel TCP errors
  // d_multi_iters_:    [K * W] iterations per (seed, weight)
  std::unique_ptr<cuda::DeviceBuffer<double>> d_target_single_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_all_seeds_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_multi_results_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_multi_errors_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_multi_shovel_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_multi_iters_;

  // GPU cost/top-K buffers (Optimization B: GPU-side sorting)
  // d_multi_costs_:      [K * W] continuity costs per candidate
  // d_topk_costs_:       [topK] top-K costs (sorted ascending)
  // d_topk_indices_:     [topK] top-K indices into multi_results
  // d_joint_weights_:    [6] joint weights for cost computation
  std::unique_ptr<cuda::DeviceBuffer<double>> d_multi_costs_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_topk_costs_;
  std::unique_ptr<cuda::DeviceBuffer<int>>    d_topk_indices_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_joint_weights_;

  // Host staging buffers
  std::vector<double> h_targets_;
  std::vector<double> h_seeds_;

  // Pending targets (before GPU dispatch)
  struct PendingTarget {
    Mat4 target;
    Eigen::VectorXd seed;
    std::array<double, 6> weights;
    double orient_limit;
  };
  std::vector<PendingTarget> pending_;

  // Solver config (cached)
  SolverConfig cfg_;
  bool initialized_ = false;

  // CUDA stream for async overlap
  cudaStream_t stream_ = 0;

  // GPU collision detection buffers
  std::unique_ptr<cuda::DeviceBuffer<double>> d_env_objects_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_robot_boxes_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_clearances_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_colliding_;
  int N_env_objects_ = 0;

  // Allocate/resize device buffers for N targets
  void ensureCapacity(int N);

  // Allocate/resize multi-seed batch buffers
  void ensureMultiSeedCapacity(int K, int W);

  // Allocate/resize GPU cost/top-K buffers
  void ensureTopKCapacity(int total, int topK);

  // Allocate/resize collision buffers
  void ensureCollisionCapacity(int N_frames, int N_boxes, int N_objects);
};

}  // namespace rtfg
