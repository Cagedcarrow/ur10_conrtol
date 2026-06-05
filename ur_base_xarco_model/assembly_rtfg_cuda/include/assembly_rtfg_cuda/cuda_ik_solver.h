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

  // Allocate/resize collision buffers
  void ensureCollisionCapacity(int N_frames, int N_boxes, int N_objects);
};

}  // namespace rtfg
