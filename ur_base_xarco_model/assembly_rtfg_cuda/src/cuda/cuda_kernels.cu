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
    double* __restrict__ d_errors,             // [N, 2]  output (pos_err, rot_err)
    double* __restrict__ d_iterations,         // [N]     iterations used
    const int    max_iter,                     // max iterations per point
    const double pos_tol,                      // position tolerance (m)
    const double orient_tol,                   // orientation tolerance (rad)
    const int    N                             // total target count
) {
  int tid = blockIdx.x;  // Task ID = block index
  if (tid >= N) return;

  // === Shared memory (per-block, bank-conflict-free via 8-column padding) ===
  __shared__ double s_q[48];           // 6 doubles padded to 8
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

    // Divergence/oscillation detection: if stagnated for >15 iters, break
    if (threadIdx.x == 0 && s_stagnation > 25) {
      // Restore best q and exit
      for (int i = 0; i < 6; ++i) s_q[i] = s_q_best[i];
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
      s_J[0 * 8 + j] = (T_p[3]  - T_m[3])  * inv_2eps;
      s_J[1 * 8 + j] = (T_p[7]  - T_m[7])  * inv_2eps;
      s_J[2 * 8 + j] = (T_p[11] - T_m[11]) * inv_2eps;

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

      s_J[3 * 8 + j] = (dR[7] - dR[5]) * 0.5 * inv_2eps;  // wx
      s_J[4 * 8 + j] = (dR[2] - dR[6]) * 0.5 * inv_2eps;  // wy
      s_J[5 * 8 + j] = (dR[3] - dR[1]) * 0.5 * inv_2eps;  // wz
    }
    __syncthreads();

    // ---- 2e: Adaptive damping (conservative: higher floor prevents overshoot) ----
    if (threadIdx.x == 0) {
      double pos_err = s_pos_err;
      if (pos_err > 0.1) {
        // Far from target: moderate damping, scale with distance
        s_lambda = fmax(2e-3, 8e-3 * (pos_err / 0.05));
        s_lambda = fmin(s_lambda, 0.15);
      } else {
        // Near target: higher floor prevents oscillations
        s_lambda = 2e-3 + 8e-3 * (pos_err / 0.05);
      }
      // Boost damping if stagnating (divergence recovery)
      if (s_stagnation > 5) {
        s_lambda *= (1.0 + 0.3 * (s_stagnation - 5));
        s_lambda = fmin(s_lambda, 0.5);
      }
    }
    __syncthreads();

    // ---- 2f: Hessian H = J^T·W^2·J + λ·I ----
    // H[r][c] = Σ_k w_k² · J[k][r] · J[k][c]  (consistent with g = J^T·W^2·e)
    if (threadIdx.x < 36) {
      int row = threadIdx.x / 6;
      int col = threadIdx.x % 6;

      double sum = 0.0;
      for (int k = 0; k < 6; ++k) {
        double w_k = c_weight_schedule[0 * 6 + k];
        double w2 = w_k * w_k;
        sum += s_J[k * 8 + row] * w2 * s_J[k * 8 + col];
      }

      if (row == col) sum += s_lambda;

      s_H[row * 8 + col] = sum;
    }
    __syncthreads();

    // ---- 2g: Gradient g = J^T·W^2·e ----
    // g[r] = Σ_k w_k² · J[k][r] · e[k]  (same W^2 as Hessian)
    if (threadIdx.x < 6) {
      double sum = 0.0;
      for (int k = 0; k < 6; ++k) {
        double w_k = c_weight_schedule[0 * 6 + k];
        sum += s_J[k * 8 + threadIdx.x] * w_k * w_k * s_err[k];
      }
      s_g[threadIdx.x] = sum;
    }
    __syncthreads();

    // ---- 2h: LDLT Solve (serial, lane 0 only) ----
    if (threadIdx.x == 0) {
      double H_dense[36], g_dense[6];
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
          H_dense[r * 6 + c] = s_H[r * 8 + c];
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

  // Final error evaluation
  __syncthreads();
  if (threadIdx.x == 0) {
    pose_error(s_T, s_T_tgt, s_err);
    double pos_err = sqrt(s_err[0]*s_err[0] + s_err[1]*s_err[1] + s_err[2]*s_err[2]);
    double rot_err = sqrt(s_err[3]*s_err[3] + s_err[4]*s_err[4] + s_err[5]*s_err[5]);
    d_errors[tid * 2 + 0] = pos_err;
    d_errors[tid * 2 + 1] = rot_err;
    d_iterations[tid] = (double)s_iter_count;
  }
}

// ============================================================================
// KERNEL 2: compute_continuity_cost — Continuity cost for candidate ranking
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
// Host-callable kernel launcher wrappers
// ============================================================================

cudaError_t launch_ik_batch_solve(
    const double* d_targets, const double* d_seeds,
    double* d_results, double* d_errors, double* d_iterations,
    int max_iter, double pos_tol, double orient_tol, int N,
    cudaStream_t stream = 0)
{
  dim3 grid(N, 1, 1);
  dim3 block(128, 1, 1);

  ik_batch_solve<<<grid, block, 0, stream>>>(
      d_targets, d_seeds, d_results, d_errors, d_iterations,
      max_iter, pos_tol, orient_tol, N);

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

}  // namespace cuda
}  // namespace rtfg
