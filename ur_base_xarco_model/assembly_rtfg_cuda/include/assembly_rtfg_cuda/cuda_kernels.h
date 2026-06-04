#pragma once
// cuda_kernels.h — CUDA kernel function declarations for UR10 IK batch solver

#include <cuda_runtime.h>

namespace rtfg {
namespace cuda {

// ============================================================================
// Kernel 1: Batch IK solver
//   One block per target pose. 128 threads/block (4 warps).
//   Grid: (N, 1, 1), Block: (128, 1, 1)
// ============================================================================
cudaError_t launch_ik_batch_solve(
    const double* d_targets,     // [N, 16] target transforms
    const double* d_seeds,        // [N, 6]  initial seed joint angles
    double* d_results,            // [N, 6]  output joint angles
    double* d_errors,             // [N, 2]  output (pos_err, rot_err)
    double* d_iterations,         // [N]     iterations used per point
    int max_iter,                 // max DLS iterations
    double pos_tol,               // position tolerance (m)
    double orient_tol,            // orientation tolerance (rad)
    int N,                        // number of target poses
    cudaStream_t stream = 0);

// ============================================================================
// Kernel 2: Continuity cost computation
//   One thread per IK result. Computes cost = norm(dq) + 0.65*norm(dq-dq_prev).
// ============================================================================
cudaError_t launch_compute_continuity_cost(
    const double* d_results,     // [N, 6] IK results
    const double* d_q_prev,       // [6]    previous joint angles
    const double* d_dq_prev,      // [6]    previous velocity
    double* d_costs,              // [N]    output costs
    int N,
    cudaStream_t stream = 0);

}  // namespace cuda
}  // namespace rtfg
