#pragma once
// cuda_ik_solver.h — CUDA batch IK solver class implementing IKSolverBase

#include "ik_backend.h"       // IKSolverBase (from cpp package)
#include "cuda_memory.h"
#include "cuda_kernels.h"

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

  // Clear pending queue
  void clear();

  // Access the underlying stream for async operations
  cudaStream_t stream() const { return stream_; }

private:
  // Device memory buffers
  std::unique_ptr<cuda::DeviceBuffer<double>> d_targets_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_seeds_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_results_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_errors_;
  std::unique_ptr<cuda::DeviceBuffer<double>> d_iterations_;

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

  // Allocate/resize device buffers for N targets
  void ensureCapacity(int N);
};

}  // namespace rtfg
