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
    double* d_errors,             // [N, 2]  output (pos_err, rot_err) at wrist
    double* d_shovel_errors,      // [N, 2]  output (pos_err, rot_err) at shovel TCP
    double* d_iterations,         // [N]     iterations used per point
    int max_iter,                 // max DLS iterations
    double pos_tol,               // position tolerance (m)
    double orient_tol,            // orientation tolerance (rad)
    int weight_level,             // weight schedule level (0=strictest, 3=position-only)
    int N,                        // number of target poses
    cudaStream_t stream = 0);

// ============================================================================
// Kernel 2: Multi-seed × multi-weight batch IK solver (OPTIMIZATION A)
//   One block per (seed, weight) pair. 128 threads/block (4 warps).
//   Grid: (K, W, 1), Block: (128, 1, 1)
//   All K×W blocks solve for the same target pose.
//   Eliminates CPU-side seed×weight loop → single kernel launch instead of 192.
// ============================================================================
cudaError_t launch_ik_batch_solve_multi(
    const double* d_target,          // [16] single target transform
    const double* d_all_seeds,       // [K * 6] all seed joint angles
    double* d_results,               // [K * W * 6] output joint angles
    double* d_errors,                // [K * W * 2] wrist errors
    double* d_shovel_errors,         // [K * W * 2] shovel TCP errors
    double* d_iterations,            // [K * W] iterations used
    int max_iter,                    // max DLS iterations
    double pos_tol,                  // position tolerance (m)
    double orient_tol,               // orientation tolerance (rad)
    int K,                           // number of seeds
    int W,                           // number of weight levels
    cudaStream_t stream = 0);

// ============================================================================
// Kernel 3: Continuity cost computation (original, per-target)
//   One thread per IK result. Computes cost = norm(dq) + 0.65*norm(dq-dq_prev).
// ============================================================================
cudaError_t launch_compute_continuity_cost(
    const double* d_results,     // [N, 6] IK results
    const double* d_q_prev,       // [6]    previous joint angles
    const double* d_dq_prev,      // [6]    previous velocity
    double* d_costs,              // [N]    output costs
    int N,
    cudaStream_t stream = 0);

// ============================================================================
// Kernel 4: Continuity cost computation — all candidates of one target (OPT B)
//   One block, 128 threads stride through K×W candidates.
//   Computes weighted squared joint displacement from q_prev.
// ============================================================================
cudaError_t launch_compute_continuity_cost_all(
    const double* d_results,         // [total * 6] IK results
    const double* d_q_prev,          // [6] reference joint angles
    double* d_costs,                 // [total] output costs
    int total,                       // K * W
    const double* d_joint_weights,   // [6] joint weights
    cudaStream_t stream = 0);

// ============================================================================
// Kernel 5: Top-K filter via bitonic sort in shared memory (OPT B)
//   Sorts K×W candidates by cost, returns top-K (lowest cost) indices.
//   Unconverged solutions (iterations < min_iterations) are filtered out.
// ============================================================================
cudaError_t launch_filter_topk_per_target(
    const double* d_costs,           // [total] input costs
    const double* d_iterations,      // [total] iterations used
    double* d_topk_costs,            // [topK] output costs (lowest first)
    int* d_topk_indices,             // [topK] output indices
    int total,                       // K * W
    int topK,                        // number to select
    int min_iterations,              // minimum iterations for validity
    cudaStream_t stream = 0);

}  // namespace cuda
}  // namespace rtfg
