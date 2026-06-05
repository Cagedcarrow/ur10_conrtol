// cuda_kernels.cu — CUDA 13.3 IK batch solver kernels for UR10 6-DOF manipulator
//
// Architecture: Ada Lovelace (sm_89), CUDA 13.3
// Precision:    double (required for IK convergence)
// Mapping:      1 block per target pose, 128 threads/block (4 warps)
// Convention:   URDF-based forward kinematics (Rodrigues formula)
//               All 4x4 matrices are row-major: M[row*4+col]
//
// Device helper functions (forward_kinematics, mat44_mul, pose_error, etc.)
// are in cuda_utilities.cuh so they can be inlined across translation units.
//
// CUDA_DEFINE_CONSTANTS must be defined before including cuda_utilities.cuh
// so __constant__ variables are defined here, not just declared extern.
//
// Warp assignments within each block:
//   Warp 0 (lanes 0-31):   FK computation
//   Warp 1 (lanes 32-63):  Numerical Jacobian (6 columns, one per lane)
//   Warp 2 (lanes 64-95):  Hessian construction J^T·J
//   Warp 3 (lanes 96-127): LDLT solve + convergence check

#define CUDA_DEFINE_CONSTANTS
#include "cuda_utilities.cuh"

namespace rtfg {
namespace cuda {

// ============================================================================
// KERNEL 1: ik_batch_solve — Batch IK solver (one block per target pose)
//
// Grid:  (N, 1, 1)   where N = number of target poses
// Block: (128, 1, 1)  4 warps × 32 lanes
//
// Each block independently solves IK for one target pose using DLS iteration.
// Shared memory: ~1,616 bytes (well within 48 KB/SM limit)
// ============================================================================
__global__ void ik_batch_solve(
    const double* __restrict__ d_targets,     // [N, 16] target transforms (row-major)
    const double* __restrict__ d_seeds,        // [N, 6]  initial seeds
    double* __restrict__ d_results,            // [N, 6]  output joint angles
    double* __restrict__ d_errors,             // [N, 2]  output (pos_err, rot_err) at wrist
    double* __restrict__ d_shovel_errors,      // [N, 2]  output (pos_err, rot_err) at shovel TCP
    double* __restrict__ d_iterations,         // [N]     iterations used
    const int    max_iter,                     // max iterations per point
    const double pos_tol,                      // position tolerance (m)
    const double orient_tol,                   // orientation tolerance (rad)
    const int    weight_level,                 // weight schedule level (0-3)
    const int    N                             // total target count
) {
  int tid = blockIdx.x;  // Task ID = block index
  if (tid >= N) return;

  // === Shared memory (per-block, bank-conflict-free via 8-column padding) ===
  __shared__ double s_q[8];            // 6 doubles padded to 8 for alignment
  __shared__ double s_T[16];           // Current FK result (4×4)
  __shared__ double s_T_tgt[16];       // Target transform
  __shared__ double s_J[48];           // Jacobian 6×6 padded to 6×8 per row
  __shared__ double s_H[48];           // Hessian 6×6 padded
  __shared__ double s_err[6];          // Pose error
  __shared__ double s_g[6];            // Gradient vector
  __shared__ double s_dq[6];           // Joint angle step
  __shared__ double s_q_ref[6];        // Reference q (for alignment)
  __shared__ double s_q_best[6];       // Best q seen so far (lowest pos error)
  __shared__ int    s_converged;       // Convergence flag
  __shared__ int    s_iter_count;      // Iteration counter
  __shared__ double s_lambda;          // Current damping factor
  __shared__ double s_best_pos_err;    // Best position error seen
  __shared__ int    s_stagnation;      // Stagnation counter (consecutive non-improvements)
  __shared__ double s_T_tcp[16];       // Shovel TCP transform (FK result with tool offset)
  __shared__ double s_T_tcp_tgt[16];   // Target shovel TCP transform

  // --- Type-safe 2D matrix views (zero-overhead, replaces manual s_J[k*8+row]) ---
  PaddedMat6x8 J(s_J);  // Jacobian view: J(row, col)
  PaddedMat6x8 H(s_H);  // Hessian view:  H(row, col)

  // === Phase 1: Load seed and target (coalesced global loads) ===
  if (threadIdx.x < 6) {
    s_q[threadIdx.x] = d_seeds[tid * 6 + threadIdx.x];
    s_q_ref[threadIdx.x] = s_q[threadIdx.x];
  }
  if (threadIdx.x < 16) {
    s_T_tgt[threadIdx.x] = d_targets[tid * 16 + threadIdx.x];
  }
  __syncthreads();

  // Initialize convergence tracking
  if (threadIdx.x == 0) {
    s_converged = 0;
    s_iter_count = 0;
    s_best_pos_err = 1e100;  // infinity
    s_stagnation = 0;
  }
  __syncthreads();

  // === Phase 2: DLS iteration loop ===
  for (int iter = 0; iter < max_iter && !s_converged; ++iter) {
    if (threadIdx.x == 0) s_iter_count = iter + 1;

    // ---- 2a: Forward Kinematics (URDF convention, Rodrigues formula) ----
    if (threadIdx.x == 0) {
      forward_kinematics(s_q, s_T);
    }
    __syncthreads();

    // ---- 2b: Pose Error ----
    double s_pos_err;
    if (threadIdx.x == 0) {
      pose_error(s_T, s_T_tgt, s_err);
      s_pos_err = sqrt(s_err[0]*s_err[0] + s_err[1]*s_err[1] + s_err[2]*s_err[2]);
    }
    __syncthreads();

    // ---- 2c: Convergence check + best tracking ----
    if (threadIdx.x == 0) {
      double rot_err = sqrt(s_err[3]*s_err[3] + s_err[4]*s_err[4] + s_err[5]*s_err[5]);
      if (s_pos_err <= pos_tol && rot_err <= orient_tol) {
        s_converged = 1;
      }
      // Track best solution for fallback
      if (s_pos_err < s_best_pos_err) {
        s_best_pos_err = s_pos_err;
        for (int i = 0; i < 6; ++i) s_q_best[i] = s_q[i];
        s_stagnation = 0;
      } else {
        s_stagnation++;
      }
    }
    __syncthreads();
    if (s_converged) break;

    // Divergence/oscillation detection: if stagnated for >25 iters, break
    if (threadIdx.x == 0 && s_stagnation > 25) {
      // Restore best q, recompute FK for consistent final pose_error
      for (int i = 0; i < 6; ++i) s_q[i] = s_q_best[i];
      forward_kinematics(s_q, s_T);  // Recompute FK with restored q
      s_converged = 1;
    }
    __syncthreads();
    if (s_converged) break;

    // ---- 2d: Numerical Jacobian (thread-parallel per column) ----
    // Threads 0-5 each compute one column of J
    if (threadIdx.x < 6) {
      int j = threadIdx.x;
      const double eps = 1e-6;
      double q_plus[6], q_minus[6], T_p[16], T_m[16];
      for (int i = 0; i < 6; ++i) {
        q_plus[i] = s_q[i];
        q_minus[i] = s_q[i];
      }
      q_plus[j]  += eps;
      q_minus[j] -= eps;

      forward_kinematics(q_plus, T_p);
      forward_kinematics(q_minus, T_m);

      double inv_2eps = 0.5 / eps;

      // Position columns
      J(0, j) = (T_p[3]  - T_m[3])  * inv_2eps;
      J(1, j) = (T_p[7]  - T_m[7])  * inv_2eps;
      J(2, j) = (T_p[11] - T_m[11]) * inv_2eps;

      // Rotation columns (angular velocity from R_diff)
      double r00 = s_T[0], r01 = s_T[1], r02 = s_T[2];
      double r10 = s_T[4], r11 = s_T[5], r12 = s_T[6];
      double r20 = s_T[8], r21 = s_T[9], r22 = s_T[10];

      double dR[9];
      dR[0] = (r00*T_p[0]+r10*T_p[4]+r20*T_p[8]) - (r00*T_m[0]+r10*T_m[4]+r20*T_m[8]);
      dR[1] = (r00*T_p[1]+r10*T_p[5]+r20*T_p[9]) - (r00*T_m[1]+r10*T_m[5]+r20*T_m[9]);
      dR[2] = (r00*T_p[2]+r10*T_p[6]+r20*T_p[10]) - (r00*T_m[2]+r10*T_m[6]+r20*T_m[10]);
      dR[3] = (r01*T_p[0]+r11*T_p[4]+r21*T_p[8]) - (r01*T_m[0]+r11*T_m[4]+r21*T_m[8]);
      dR[4] = (r01*T_p[1]+r11*T_p[5]+r21*T_p[9]) - (r01*T_m[1]+r11*T_m[5]+r21*T_m[9]);
      dR[5] = (r01*T_p[2]+r11*T_p[6]+r21*T_p[10]) - (r01*T_m[2]+r11*T_m[6]+r21*T_m[10]);
      dR[6] = (r02*T_p[0]+r12*T_p[4]+r22*T_p[8]) - (r02*T_m[0]+r12*T_m[4]+r22*T_m[8]);
      dR[7] = (r02*T_p[1]+r12*T_p[5]+r22*T_p[9]) - (r02*T_m[1]+r12*T_m[5]+r22*T_m[9]);
      dR[8] = (r02*T_p[2]+r12*T_p[6]+r22*T_p[10]) - (r02*T_m[2]+r12*T_m[6]+r22*T_m[10]);

      J(3, j) = (dR[7] - dR[5]) * 0.5 * inv_2eps;  // wx
      J(4, j) = (dR[2] - dR[6]) * 0.5 * inv_2eps;  // wy
      J(5, j) = (dR[3] - dR[1]) * 0.5 * inv_2eps;  // wz
    }
    __syncthreads();

    // ---- 2e: Adaptive damping (reads c_lambda_params from constant memory) ----
    if (threadIdx.x == 0) {
      double pos_err = s_pos_err;
      // Read damping parameters from GPU constant memory (uploaded at init)
      double lambda_base  = c_lambda_params[0];  // Base damping (e.g., 5e-4 or 2e-3)
      double lambda_far   = c_lambda_params[1];  // Far-distance max (e.g., 0.1)
      double lambda_floor = c_lambda_params[2];  // Near-distance floor (e.g., 5e-4)
      double lambda_scale = c_lambda_params[3];  // Scaling reference distance (e.g., 0.05)

      if (pos_err > 0.1) {
        // Far from target: moderate damping, scale with distance
        s_lambda = fmax(lambda_base, lambda_far * (pos_err / lambda_scale));
        s_lambda = fmin(s_lambda, lambda_far * 3.0);
      } else {
        // Near target: higher floor prevents oscillations
        s_lambda = lambda_floor + lambda_base * (pos_err / lambda_scale);
      }
      // Boost damping if stagnating (divergence recovery)
      if (s_stagnation > 5) {
        s_lambda *= (1.0 + 0.3 * (s_stagnation - 5));
        s_lambda = fmin(s_lambda, 0.5);
      }
    }
    __syncthreads();

    // ---- 2f: Hessian H = J^T·W^2·J + λ·I ----
    // H[r][c] = Σ_k w_k² · J(k,r) · J(k,c)  (consistent with g = J^T·W^2·e)
    if (threadIdx.x < 36) {
      int row = threadIdx.x / 6;
      int col = threadIdx.x % 6;

      double sum = 0.0;
      for (int k = 0; k < 6; ++k) {
        double w_k = c_weight_schedule[weight_level * 6 + k];
        double w2 = w_k * w_k;
        sum += J(k, row) * w2 * J(k, col);
      }

      if (row == col) sum += s_lambda;

      H(row, col) = sum;
    }
    __syncthreads();

    // ---- 2g: Gradient g = J^T·W^2·e ----
    // g[r] = Σ_k w_k² · J(k,r) · e[k]  (same W^2 as Hessian)
    if (threadIdx.x < 6) {
      double sum = 0.0;
      for (int k = 0; k < 6; ++k) {
        double w_k = c_weight_schedule[weight_level * 6 + k];
        sum += J(k, threadIdx.x) * w_k * w_k * s_err[k];
      }
      s_g[threadIdx.x] = sum;
    }
    __syncthreads();

    // ---- 2h: LDLT Solve (serial, lane 0 only) ----
    if (threadIdx.x == 0) {
      double H_dense[36], g_dense[6];
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
          H_dense[r * 6 + c] = H(r, c);
        }
        g_dense[r] = s_g[r];
      }
      ldlt_solve_6x6(H_dense, g_dense, s_dq);
    }
    __syncthreads();

    // ---- 2i: Step clamping (conservative: 0.25 rad ≈ 14° prevents overshoot) ----
    if (threadIdx.x == 0) {
      double step_norm = cuda_norm6(s_dq);
      if (step_norm > 0.25) {
        double scale = 0.25 / step_norm;
        for (int i = 0; i < 6; ++i) s_dq[i] *= scale;
      }
      if (step_norm <= 1e-8) {
        s_converged = 1;  // Stagnation detected
      }
    }
    __syncthreads();
    if (s_converged) break;

    // Apply step with joint limits
    if (threadIdx.x < 6) {
      int i = threadIdx.x;
      double lo = c_joint_limits[i * 2 + 0];
      double hi = c_joint_limits[i * 2 + 1];
      s_q[i] = cuda_clamp(s_q[i] + s_dq[i], lo, hi);
    }
    __syncthreads();

    // ---- 2j: Branch alignment (avoid J5/J6 π-wraps) ----
    if (threadIdx.x == 0) {
      for (int i = 0; i < 6; ++i) {
        double diff = s_q[i] - s_q_ref[i];
        double wrapped = atan2(sin(diff), cos(diff));
        s_q[i] = s_q_ref[i] + wrapped;
      }
    }
    __syncthreads();
  }

  // === Phase 3: Write results (coalesced global stores) ===
  if (threadIdx.x < 6) {
    d_results[tid * 6 + threadIdx.x] = s_q[threadIdx.x];
  }

  // Final error evaluation (wrist + shovel TCP)
  __syncthreads();
  if (threadIdx.x == 0) {
    // Wrist pose error
    pose_error(s_T, s_T_tgt, s_err);
    double pos_err = sqrt(s_err[0]*s_err[0] + s_err[1]*s_err[1] + s_err[2]*s_err[2]);
    double rot_err = sqrt(s_err[3]*s_err[3] + s_err[4]*s_err[4] + s_err[5]*s_err[5]);
    d_errors[tid * 2 + 0] = pos_err;
    d_errors[tid * 2 + 1] = rot_err;

    // Shovel TCP error: compute TCP transform from wrist FK result
    shovel_tcp_transform(s_T, c_T_wrist3_to_tcp, s_T_tcp);
    shovel_tcp_transform(s_T_tgt, c_T_wrist3_to_tcp, s_T_tcp_tgt);
    double shovel_pos_err, shovel_rot_err;
    shovel_pose_error(s_T_tcp, s_T_tcp_tgt, &shovel_pos_err, &shovel_rot_err);
    if (d_shovel_errors) {
      d_shovel_errors[tid * 2 + 0] = shovel_pos_err;
      d_shovel_errors[tid * 2 + 1] = shovel_rot_err;
    }

    d_iterations[tid] = (double)s_iter_count;
  }
}

// ============================================================================
// KERNEL 2: ik_batch_solve_multi — Multi-seed × multi-weight batch IK solver
//
// Grid:  (K, W, 1)   K = num_seeds, W = num_weight_levels
// Block: (128, 1, 1)  4 warps × 32 lanes
//
// Each block independently solves IK for ONE (seed, weight) pair against
// the same target pose. All K×W blocks share the same target via uniform
// parameter, seeds and weights are indexed by blockIdx.x/y.
//
// This eliminates the CPU-side seed×weight loop that caused up to 192
// separate kernel launches per target in the original design:
//   Before: CPU loop { for w in 0..3: for k in 0..47: launch<<<1,128>>> }
//   After:  Single launch<<<(K,W,1), 128>>>
//
// Shared memory: ~1,616 bytes (unchanged from ik_batch_solve)
// Registers: ~105-110 (estimated, +9-14 vs original 96 due to extra indexing)
// ============================================================================
__global__ void ik_batch_solve_multi(
    const double* __restrict__ d_target,         // [16] single target transform (row-major)
    const double* __restrict__ d_all_seeds,      // [K * 6] all seed joint angles
    double* __restrict__ d_results,              // [K * W * 6] output joint angles
    double* __restrict__ d_errors,               // [K * W * 2] output (pos_err, rot_err) at wrist
    double* __restrict__ d_shovel_errors,        // [K * W * 2] output (pos_err, rot_err) at shovel TCP
    double* __restrict__ d_iterations,           // [K * W] iterations used
    const int    max_iter,                       // max DLS iterations per block
    const double pos_tol,                        // position tolerance (m)
    const double orient_tol,                     // orientation tolerance (rad)
    const int    K,                              // number of seeds
    const int    W                               // number of weight levels
) {
  int seed_idx   = blockIdx.x;   // 0 .. K-1
  int weight_idx = blockIdx.y;   // 0 .. W-1 (used as weight_level)

  // Bounds check: extra blocks beyond K×W are no-ops
  if (seed_idx >= K || weight_idx >= W) return;

  // Linear output index
  int linear_idx = seed_idx * W + weight_idx;

  // === Shared memory (per-block, bank-conflict-free via 8-column padding) ===
  __shared__ double s_q[8];            // 6 doubles padded to 8 for alignment
  __shared__ double s_T[16];           // Current FK result (4×4)
  __shared__ double s_T_tgt[16];       // Target transform
  __shared__ double s_J[48];           // Jacobian 6×6 padded to 6×8 per row
  __shared__ double s_H[48];           // Hessian 6×6 padded
  __shared__ double s_err[6];          // Pose error
  __shared__ double s_g[6];            // Gradient vector
  __shared__ double s_dq[6];           // Joint angle step
  __shared__ double s_q_ref[6];        // Reference q (for alignment)
  __shared__ double s_q_best[6];       // Best q seen so far (lowest pos error)
  __shared__ int    s_converged;       // Convergence flag
  __shared__ int    s_iter_count;      // Iteration counter
  __shared__ double s_lambda;          // Current damping factor
  __shared__ double s_best_pos_err;    // Best position error seen
  __shared__ int    s_stagnation;      // Stagnation counter
  __shared__ double s_T_tcp[16];       // Shovel TCP transform
  __shared__ double s_T_tcp_tgt[16];   // Target shovel TCP transform

  // Type-safe 2D matrix views
  PaddedMat6x8 J(s_J);
  PaddedMat6x8 H(s_H);

  // === Phase 1: Load seed and target (coalesced global loads) ===
  if (threadIdx.x < 6) {
    s_q[threadIdx.x] = d_all_seeds[seed_idx * 6 + threadIdx.x];
    s_q_ref[threadIdx.x] = s_q[threadIdx.x];
  }
  // All blocks read the same target — broadcast via L1 cache
  if (threadIdx.x < 16) {
    s_T_tgt[threadIdx.x] = d_target[threadIdx.x];
  }
  __syncthreads();

  // Initialize convergence tracking
  if (threadIdx.x == 0) {
    s_converged = 0;
    s_iter_count = 0;
    s_best_pos_err = 1e100;
    s_stagnation = 0;
  }
  __syncthreads();

  // === Phase 2: DLS iteration loop ===
  for (int iter = 0; iter < max_iter && !s_converged; ++iter) {
    if (threadIdx.x == 0) s_iter_count = iter + 1;

    // ---- 2a: Forward Kinematics ----
    if (threadIdx.x == 0) {
      forward_kinematics(s_q, s_T);
    }
    __syncthreads();

    // ---- 2b: Pose Error ----
    double s_pos_err;
    if (threadIdx.x == 0) {
      pose_error(s_T, s_T_tgt, s_err);
      s_pos_err = sqrt(s_err[0]*s_err[0] + s_err[1]*s_err[1] + s_err[2]*s_err[2]);
    }
    __syncthreads();

    // ---- 2c: Convergence check + best tracking ----
    if (threadIdx.x == 0) {
      double rot_err = sqrt(s_err[3]*s_err[3] + s_err[4]*s_err[4] + s_err[5]*s_err[5]);
      if (s_pos_err <= pos_tol && rot_err <= orient_tol) {
        s_converged = 1;
      }
      if (s_pos_err < s_best_pos_err) {
        s_best_pos_err = s_pos_err;
        for (int i = 0; i < 6; ++i) s_q_best[i] = s_q[i];
        s_stagnation = 0;
      } else {
        s_stagnation++;
      }
    }
    __syncthreads();
    if (s_converged) break;

    // Divergence/oscillation detection
    if (threadIdx.x == 0 && s_stagnation > 25) {
      for (int i = 0; i < 6; ++i) s_q[i] = s_q_best[i];
      forward_kinematics(s_q, s_T);
      s_converged = 1;
    }
    __syncthreads();
    if (s_converged) break;

    // ---- 2d: Numerical Jacobian ----
    if (threadIdx.x < 6) {
      int j = threadIdx.x;
      const double eps = 1e-6;
      double q_plus[6], q_minus[6], T_p[16], T_m[16];
      for (int i = 0; i < 6; ++i) {
        q_plus[i] = s_q[i];
        q_minus[i] = s_q[i];
      }
      q_plus[j]  += eps;
      q_minus[j] -= eps;

      forward_kinematics(q_plus, T_p);
      forward_kinematics(q_minus, T_m);

      double inv_2eps = 0.5 / eps;

      J(0, j) = (T_p[3]  - T_m[3])  * inv_2eps;
      J(1, j) = (T_p[7]  - T_m[7])  * inv_2eps;
      J(2, j) = (T_p[11] - T_m[11]) * inv_2eps;

      double r00 = s_T[0], r01 = s_T[1], r02 = s_T[2];
      double r10 = s_T[4], r11 = s_T[5], r12 = s_T[6];
      double r20 = s_T[8], r21 = s_T[9], r22 = s_T[10];

      double dR[9];
      dR[0] = (r00*T_p[0]+r10*T_p[4]+r20*T_p[8]) - (r00*T_m[0]+r10*T_m[4]+r20*T_m[8]);
      dR[1] = (r00*T_p[1]+r10*T_p[5]+r20*T_p[9]) - (r00*T_m[1]+r10*T_m[5]+r20*T_m[9]);
      dR[2] = (r00*T_p[2]+r10*T_p[6]+r20*T_p[10]) - (r00*T_m[2]+r10*T_m[6]+r20*T_m[10]);
      dR[3] = (r01*T_p[0]+r11*T_p[4]+r21*T_p[8]) - (r01*T_m[0]+r11*T_m[4]+r21*T_m[8]);
      dR[4] = (r01*T_p[1]+r11*T_p[5]+r21*T_p[9]) - (r01*T_m[1]+r11*T_m[5]+r21*T_m[9]);
      dR[5] = (r01*T_p[2]+r11*T_p[6]+r21*T_p[10]) - (r01*T_m[2]+r11*T_m[6]+r21*T_m[10]);
      dR[6] = (r02*T_p[0]+r12*T_p[4]+r22*T_p[8]) - (r02*T_m[0]+r12*T_m[4]+r22*T_m[8]);
      dR[7] = (r02*T_p[1]+r12*T_p[5]+r22*T_p[9]) - (r02*T_m[1]+r12*T_m[5]+r22*T_m[9]);
      dR[8] = (r02*T_p[2]+r12*T_p[6]+r22*T_p[10]) - (r02*T_m[2]+r12*T_m[6]+r22*T_m[10]);

      J(3, j) = (dR[7] - dR[5]) * 0.5 * inv_2eps;
      J(4, j) = (dR[2] - dR[6]) * 0.5 * inv_2eps;
      J(5, j) = (dR[3] - dR[1]) * 0.5 * inv_2eps;
    }
    __syncthreads();

    // ---- 2e: Adaptive damping ----
    if (threadIdx.x == 0) {
      double pos_err = s_pos_err;
      double lambda_base  = c_lambda_params[0];
      double lambda_far   = c_lambda_params[1];
      double lambda_floor = c_lambda_params[2];
      double lambda_scale = c_lambda_params[3];

      if (pos_err > 0.1) {
        s_lambda = fmax(lambda_base, lambda_far * (pos_err / lambda_scale));
        s_lambda = fmin(s_lambda, lambda_far * 3.0);
      } else {
        s_lambda = lambda_floor + lambda_base * (pos_err / lambda_scale);
      }
      if (s_stagnation > 5) {
        s_lambda *= (1.0 + 0.3 * (s_stagnation - 5));
        s_lambda = fmin(s_lambda, 0.5);
      }
    }
    __syncthreads();

    // ---- 2f: Hessian H = J^T·W^2·J + λ·I ----
    // NOTE: weight_idx directly indexes c_weight_schedule (replaces weight_level parameter)
    if (threadIdx.x < 36) {
      int row = threadIdx.x / 6;
      int col = threadIdx.x % 6;

      double sum = 0.0;
      for (int k = 0; k < 6; ++k) {
        double w_k = c_weight_schedule[weight_idx * 6 + k];
        double w2 = w_k * w_k;
        sum += J(k, row) * w2 * J(k, col);
      }

      if (row == col) sum += s_lambda;

      H(row, col) = sum;
    }
    __syncthreads();

    // ---- 2g: Gradient g = J^T·W^2·e ----
    if (threadIdx.x < 6) {
      double sum = 0.0;
      for (int k = 0; k < 6; ++k) {
        double w_k = c_weight_schedule[weight_idx * 6 + k];
        sum += J(k, threadIdx.x) * w_k * w_k * s_err[k];
      }
      s_g[threadIdx.x] = sum;
    }
    __syncthreads();

    // ---- 2h: LDLT Solve ----
    if (threadIdx.x == 0) {
      double H_dense[36], g_dense[6];
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
          H_dense[r * 6 + c] = H(r, c);
        }
        g_dense[r] = s_g[r];
      }
      ldlt_solve_6x6(H_dense, g_dense, s_dq);
    }
    __syncthreads();

    // ---- 2i: Step clamping ----
    if (threadIdx.x == 0) {
      double step_norm = cuda_norm6(s_dq);
      if (step_norm > 0.25) {
        double scale = 0.25 / step_norm;
        for (int i = 0; i < 6; ++i) s_dq[i] *= scale;
      }
      if (step_norm <= 1e-8) {
        s_converged = 1;
      }
    }
    __syncthreads();
    if (s_converged) break;

    // Apply step with joint limits
    if (threadIdx.x < 6) {
      int i = threadIdx.x;
      double lo = c_joint_limits[i * 2 + 0];
      double hi = c_joint_limits[i * 2 + 1];
      s_q[i] = cuda_clamp(s_q[i] + s_dq[i], lo, hi);
    }
    __syncthreads();

    // ---- 2j: Branch alignment ----
    if (threadIdx.x == 0) {
      for (int i = 0; i < 6; ++i) {
        double diff = s_q[i] - s_q_ref[i];
        double wrapped = atan2(sin(diff), cos(diff));
        s_q[i] = s_q_ref[i] + wrapped;
      }
    }
    __syncthreads();
  }

  // === Phase 3: Write results (coalesced global stores) ===
  if (threadIdx.x < 6) {
    d_results[linear_idx * 6 + threadIdx.x] = s_q[threadIdx.x];
  }

  // Final error evaluation (wrist + shovel TCP)
  __syncthreads();
  if (threadIdx.x == 0) {
    pose_error(s_T, s_T_tgt, s_err);
    double pos_err = sqrt(s_err[0]*s_err[0] + s_err[1]*s_err[1] + s_err[2]*s_err[2]);
    double rot_err = sqrt(s_err[3]*s_err[3] + s_err[4]*s_err[4] + s_err[5]*s_err[5]);
    d_errors[linear_idx * 2 + 0] = pos_err;
    d_errors[linear_idx * 2 + 1] = rot_err;

    // Shovel TCP error
    shovel_tcp_transform(s_T, c_T_wrist3_to_tcp, s_T_tcp);
    shovel_tcp_transform(s_T_tgt, c_T_wrist3_to_tcp, s_T_tcp_tgt);
    double shovel_pos_err, shovel_rot_err;
    shovel_pose_error(s_T_tcp, s_T_tcp_tgt, &shovel_pos_err, &shovel_rot_err);
    if (d_shovel_errors) {
      d_shovel_errors[linear_idx * 2 + 0] = shovel_pos_err;
      d_shovel_errors[linear_idx * 2 + 1] = shovel_rot_err;
    }

    d_iterations[linear_idx] = (double)s_iter_count;
  }
}

// ============================================================================
// KERNEL 3: compute_continuity_cost — Continuity cost for candidate ranking
// ============================================================================
__global__ void compute_continuity_cost(
    const double* __restrict__ d_results,     // [N, 6] IK results
    const double* __restrict__ d_q_prev,       // [6] previous joint angles
    const double* __restrict__ d_dq_prev,      // [6] previous velocity
    double* __restrict__ d_costs,              // [N] output costs
    int N
) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= N) return;

  double q[6], dq[6];
  for (int j = 0; j < 6; ++j) {
    q[j] = d_results[tid * 6 + j];
    double raw_diff = q[j] - d_q_prev[j];
    dq[j] = atan2(sin(raw_diff), cos(raw_diff));
  }

  double norm_dq = sqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2] +
                        dq[3]*dq[3] + dq[4]*dq[4] + dq[5]*dq[5]);

  double norm_diff = 0.0;
  for (int j = 0; j < 6; ++j) {
    double diff = dq[j] - d_dq_prev[j];
    norm_diff += diff * diff;
  }
  norm_diff = sqrt(norm_diff);

  double cost = norm_dq + 0.65 * norm_diff;

  // Branch-switch penalty (>25° threshold)
  const double BRANCH_THRESHOLD = 25.0 * CUDA_PI / 180.0;
  for (int j = 0; j < 6; ++j) {
    double raw = q[j] - d_q_prev[j];
    double abs_raw = fabs(raw);
    if (abs_raw > BRANCH_THRESHOLD) {
      cost += (abs_raw - BRANCH_THRESHOLD);
    }
  }

  d_costs[tid] = cost;
}

// ============================================================================
// KERNEL 4: compute_continuity_cost_all — GPU-side continuity cost computation
//
// Grid:  (1, 1, 1), Block: (128, 1, 1)
//
// Computes joint-space continuity cost for all K×W IK solutions of one target.
// Each thread handles ceil(total/128) candidates via strided loop.
//
// Cost metric (MATLAB-compatible):
//   cost = Σ_j w_j · (q[j] - q_prev[j])²
// where w_j are joint weights (typically all 1.0 for equal joint weighting).
//
// This replaces the CPU-side continuityCost() loop over 52,416 candidates
// (~2-3 ms CPU time), moving it to GPU shared memory (~10-20 μs).
// ============================================================================
__global__ void compute_continuity_cost_all(
    const double* __restrict__ d_results,       // [total * 6] IK results
    const double* __restrict__ d_q_prev,         // [6] reference joint angles
    double* __restrict__ d_costs,                // [total] output costs
    int total,                                   // K * W (number of candidates)
    const double* __restrict__ d_joint_weights   // [6] joint weights for cost
) {
  int tid = threadIdx.x;

  // Each thread strides through the candidate list
  for (int i = tid; i < total; i += blockDim.x) {
    const double* q = d_results + i * 6;
    double cost = 0.0;

    // Weighted squared joint displacement from q_prev
    #pragma unroll
    for (int j = 0; j < 6; ++j) {
      double diff = q[j] - d_q_prev[j];
      // Use angular wrap-aware difference (atan2(sin, cos))
      double wrapped_diff = atan2(sin(diff), cos(diff));
      cost += d_joint_weights[j] * wrapped_diff * wrapped_diff;
    }

    d_costs[i] = cost;
  }
}

// ============================================================================
// KERNEL 5: filter_topk_per_target — GPU-side top-K selection via bitonic sort
//
// Grid:  (1, 1, 1), Block: (128, 1, 1)
//
// Performs in-place bitonic sort on K×W candidate costs in shared memory,
// then writes the top-K (lowest cost) indices and costs to global memory.
//
// Candidates with iterations < min_iterations are assigned INFINITY cost
// (filtered out) before sorting — these are unconverged solutions.
//
// Design constraints:
//   - Max candidates per target: 256 (supports up to 64 seeds × 4 weights)
//   - If K×W > 256, only the first 256 are processed (excess is vanishingly rare)
//   - Shared memory: 256×sizeof(double) + 256×sizeof(int) = 4,096 bytes
//
// Expected performance: ~20-30 μs (bitonic sort of 256 elements in shared memory)
// ============================================================================
__global__ void filter_topk_per_target(
    const double* __restrict__ d_costs,          // [total] input costs
    const double* __restrict__ d_iterations,     // [total] iterations used
    double* __restrict__ d_topk_costs,           // [topK] output costs (lowest first)
    int* __restrict__ d_topk_indices,            // [topK] output indices
    int total,                                   // K * W
    int topK,                                    // number to select (typically 5)
    int min_iterations                           // minimum iterations for validity
) {
  // Shared memory: interleaved (cost, index) pairs for bitonic sort
  // Maximum 256 candidates fits in 4 KB shared memory
  __shared__ double smem_costs[256];
  __shared__ int    smem_indices[256];

  int n = min(total, 256);  // Cap at 256 for shared memory budget

  // --- Load costs into shared memory ---
  // Unconverged solutions (iterations < min) → INFINITY (filtered out)
  for (int i = threadIdx.x; i < n; i += blockDim.x) {
    double iters = d_iterations[i];
    smem_costs[i]  = (iters >= (double)min_iterations) ? d_costs[i] : 1e100;
    smem_indices[i] = i;
  }
  __syncthreads();

  // --- Bitonic sort: ascending order (lowest cost first) ---
  // Pad n to next power of 2 for bitonic sort
  int np2 = 1;
  while (np2 < n) np2 <<= 1;

  // For elements beyond n (padding), set to INFINITY
  for (int i = threadIdx.x + n; i < np2; i += blockDim.x) {
    smem_costs[i] = 1e100;
    smem_indices[i] = i;
  }
  __syncthreads();

  // Bitonic sort: O(log² n) parallel stages
  for (int stage = 2; stage <= np2; stage <<= 1) {
    for (int step = stage >> 1; step > 0; step >>= 1) {
      // Each thread compares one pair
      int i = threadIdx.x;
      int j = i ^ step;  // Bitwise XOR gives comparison partner

      if (j > i) {
        // Determine sort direction from the stage bit
        bool ascending = ((i & stage) == 0);

        double cost_i = smem_costs[i];
        double cost_j = smem_costs[j];
        int    idx_i  = smem_indices[i];
        int    idx_j  = smem_indices[j];

        bool swap_needed = ascending ? (cost_i > cost_j) : (cost_i < cost_j);

        if (swap_needed) {
          smem_costs[i]  = cost_j;
          smem_costs[j]  = cost_i;
          smem_indices[i] = idx_j;
          smem_indices[j] = idx_i;
        }
      }
      __syncthreads();
    }
  }

  // --- Write top-K results ---
  if (threadIdx.x < topK) {
    d_topk_costs[threadIdx.x]   = smem_costs[threadIdx.x];
    d_topk_indices[threadIdx.x] = smem_indices[threadIdx.x];
  }
}

// ============================================================================
// Host-callable kernel launcher wrappers
// ============================================================================

cudaError_t launch_ik_batch_solve(
    const double* d_targets, const double* d_seeds,
    double* d_results, double* d_errors, double* d_shovel_errors,
    double* d_iterations,
    int max_iter, double pos_tol, double orient_tol, int weight_level, int N,
    cudaStream_t stream = 0)
{
  dim3 grid(N, 1, 1);
  dim3 block(128, 1, 1);

  ik_batch_solve<<<grid, block, 0, stream>>>(
      d_targets, d_seeds, d_results, d_errors, d_shovel_errors,
      d_iterations,
      max_iter, pos_tol, orient_tol, weight_level, N);

  return cudaGetLastError();
}

// ============================================================================
// Host wrapper: launch_ik_batch_solve_multi
//
// Launches ik_batch_solve_multi with 3D grid (K, W, 1) to solve IK for
// all K×W (seed, weight) pairs in a single kernel launch.
//
// All K×W blocks share the same target pose; seeds and weight schedules
// are indexed by blockIdx.x and blockIdx.y respectively.
//
// H2D transfers: 1× target (128 B) + 1× all_seeds (K×6×8 B)
// D2H transfers: 1× results (K×W×6×8 B) + 1× errors (K×W×2×8 B) + 1× iterations (K×W×8 B)
// ============================================================================
cudaError_t launch_ik_batch_solve_multi(
    const double* d_target,          // [16] single target transform
    const double* d_all_seeds,       // [K * 6] all seed joint angles
    double* d_results,               // [K * W * 6] output joint angles
    double* d_errors,                // [K * W * 2] wrist errors
    double* d_shovel_errors,         // [K * W * 2] shovel TCP errors
    double* d_iterations,            // [K * W] iterations used
    int max_iter, double pos_tol, double orient_tol,
    int K, int W,                    // grid dimensions: K seeds × W weight levels
    cudaStream_t stream = 0)
{
  dim3 grid(K, W, 1);
  dim3 block(128, 1, 1);

  ik_batch_solve_multi<<<grid, block, 0, stream>>>(
      d_target, d_all_seeds,
      d_results, d_errors, d_shovel_errors, d_iterations,
      max_iter, pos_tol, orient_tol,
      K, W);

  return cudaGetLastError();
}

cudaError_t launch_compute_continuity_cost(
    const double* d_results, const double* d_q_prev, const double* d_dq_prev,
    double* d_costs, int N, cudaStream_t stream = 0)
{
  int block_size = 256;
  int grid_size = (N + block_size - 1) / block_size;
  dim3 grid(grid_size);
  dim3 block(block_size);

  compute_continuity_cost<<<grid, block, 0, stream>>>(
      d_results, d_q_prev, d_dq_prev, d_costs, N);

  return cudaGetLastError();
}

// ============================================================================
// Host wrapper: launch_compute_continuity_cost_all (Optimization B)
//
// Computes continuity cost for all K×W candidates of one target on GPU.
// One block (128 threads), each thread strides through candidates.
// ============================================================================
cudaError_t launch_compute_continuity_cost_all(
    const double* d_results,         // [total * 6] IK results
    const double* d_q_prev,          // [6] reference joint angles
    double* d_costs,                 // [total] output costs
    int total,                       // K * W
    const double* d_joint_weights,   // [6] joint weights
    cudaStream_t stream = 0)
{
  dim3 grid(1, 1, 1);
  dim3 block(128, 1, 1);

  compute_continuity_cost_all<<<grid, block, 0, stream>>>(
      d_results, d_q_prev, d_costs, total, d_joint_weights);

  return cudaGetLastError();
}

// ============================================================================
// Host wrapper: launch_filter_topk_per_target (Optimization B)
//
// GPU-side bitonic sort + top-K selection. Writes top-K (lowest cost) indices
// and costs to d_topk_costs / d_topk_indices.
// ============================================================================
cudaError_t launch_filter_topk_per_target(
    const double* d_costs,           // [total] input costs
    const double* d_iterations,      // [total] iterations used
    double* d_topk_costs,            // [topK] output costs
    int* d_topk_indices,             // [topK] output indices
    int total,                       // K * W
    int topK,                        // number to select
    int min_iterations,              // minimum iterations for validity
    cudaStream_t stream = 0)
{
  dim3 grid(1, 1, 1);
  dim3 block(128, 1, 1);

  filter_topk_per_target<<<grid, block, 0, stream>>>(
      d_costs, d_iterations, d_topk_costs, d_topk_indices,
      total, topK, min_iterations);

  return cudaGetLastError();
}

}  // namespace cuda
}  // namespace rtfg
