#pragma once
// cuda_utilities.cuh — CUDA 13.3 utility macros and math helpers for UR10 IK batch solver
//
// Key design decisions:
//   - double precision throughout (IK requires it)
//   - URDF convention: T = T * origin[i] * Rodrigues(axis[i], q[i])
//   - All 4×4 matrices are row-major: M[row*4+col]
//   - Rodrigues' formula for arbitrary-axis rotation (necessary for UR10's
//     mixed axes: Z / Y / Y / Y / -Z / Y)

#include <cuda_runtime.h>
#include <cmath>

// ---- Error checking macro ----
#define CUDA_CHECK(call)                                                      \
  do {                                                                        \
    cudaError_t err = call;                                                  \
    if (err != cudaSuccess) {                                                 \
      fprintf(stderr, "CUDA error at %s:%d — %s\n", __FILE__, __LINE__,     \
              cudaGetErrorString(err));                                       \
      return err;                                                             \
    }                                                                         \
  } while (0)

#define CUDA_CHECK_KERNEL()                                                   \
  do {                                                                        \
    cudaError_t err = cudaGetLastError();                                    \
    if (err != cudaSuccess) {                                                 \
      fprintf(stderr, "Kernel launch error at %s:%d — %s\n", __FILE__,      \
              __LINE__, cudaGetErrorString(err));                             \
      return err;                                                             \
    }                                                                         \
  } while (0)

// ---- Math constants ----
#define CUDA_PI 3.14159265358979323846
#define CUDA_TWO_PI 6.28318530717958647692

namespace rtfg {
namespace cuda {

// ---- Device math helpers ----
__device__ __forceinline__ double cuda_wrap(double x) {
  return atan2(sin(x), cos(x));
}

__device__ __forceinline__ double cuda_clamp(double x, double lo, double hi) {
  return fmin(fmax(x, lo), hi);
}

__device__ __forceinline__ double cuda_rad2deg(double rad) {
  return rad * 180.0 / CUDA_PI;
}

__device__ __forceinline__ double cuda_deg2rad(double deg) {
  return deg * CUDA_PI / 180.0;
}

// 6-vector norm (double)
__device__ __forceinline__ double cuda_norm6(const double* v) {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] +
              v[3] * v[3] + v[4] * v[4] + v[5] * v[5]);
}

// ============================================================================
// PaddedMat6x8 — Lightweight 6×8 padded shared memory wrapper
//
// Provides type-safe operator()(row, col) access to 6×6 matrices stored in
// 48-element double arrays with 8-column padding for bank-conflict avoidance.
// All methods are __device__ __forceinline__ for zero overhead.
//
// This achieves the intent of cuda::std::mdspan (type-safe 2D access) without
// the C++20 and heavy template dependency. For 6×6 matrices, this lightweight
// wrapper is a better engineering choice than the full mdspan implementation.
// ============================================================================
struct PaddedMat6x8 {
  double* data;  // Points to __shared__ double[48]

  __device__ __forceinline__ PaddedMat6x8(double* d) : data(d) {}

  // Access element at (row, col) with 8-column padding: idx = row * 8 + col
  __device__ __forceinline__ double& operator()(int row, int col) {
    return data[row * 8 + col];
  }
  __device__ __forceinline__ const double& operator()(int row, int col) const {
    return data[row * 8 + col];
  }

  // Access underlying raw pointer (for backward compatibility)
  __device__ __forceinline__ double* raw() { return data; }
};

}  // namespace cuda
}  // namespace rtfg

// ============================================================================
// Constant memory declarations
// Pattern: header declares extern; cuda_kernels.cu defines with CUDA_DEFINE_CONSTANTS.
// CUDA __constant__ extern is problematic — use this C-style include guard.
// ============================================================================
#ifdef CUDA_DEFINE_CONSTANTS
  #define EXTERN_CONSTANT __constant__
#else
  #define EXTERN_CONSTANT extern __constant__
#endif

EXTERN_CONSTANT double c_segment_origins[96];
EXTERN_CONSTANT double c_segment_axes[18];
EXTERN_CONSTANT int    c_q_index[6];
EXTERN_CONSTANT double c_T_wrist3_to_tcp[16];
EXTERN_CONSTANT double c_joint_limits[12];
EXTERN_CONSTANT double c_weight_schedule[24];
EXTERN_CONSTANT double c_lambda_params[4];

#undef EXTERN_CONSTANT

namespace rtfg {
namespace cuda {

// ============================================================================
// Rodrigues' rotation formula: R ∈ SO(3) ⊆ SE(3) for axis-angle representation
//
//   R = I + sin(θ)·[a]× + (1-cos(θ))·[a]×²
//
// where [a]× is the skew-symmetric cross-product matrix of unit axis a.
// This is the standard URDF convention for revolute joint rotation axis.
//
// Parameters:
//   ax, ay, az — unit rotation axis (already normalized)
//   angle      — rotation angle in radians
//   R          — output 4×4 homogeneous matrix (row-major, R[row*4+col])
// ============================================================================
__device__ __forceinline__ void build_rotation_matrix(
    double ax, double ay, double az, double angle, double* R) {
  double c = cos(angle);
  double s = sin(angle);
  double t = 1.0 - c;

  // Row 0: [t*ax*ax+c,  t*ax*ay-s*az, t*ax*az+s*ay, 0]
  R[0]  = t * ax * ax + c;
  R[1]  = t * ax * ay + s * az;
  R[2]  = t * ax * az - s * ay;
  R[3]  = 0.0;

  // Row 1: [t*ax*ay+s*az, t*ay*ay+c,  t*ay*az-s*ax, 0]
  R[4]  = t * ax * ay - s * az;
  R[5]  = t * ay * ay + c;
  R[6]  = t * ay * az + s * ax;
  R[7]  = 0.0;

  // Row 2: [t*ax*az-s*ay, t*ay*az+s*ax, t*az*az+c,  0]
  R[8]  = t * ax * az + s * ay;
  R[9]  = t * ay * az - s * ax;
  R[10] = t * az * az + c;
  R[11] = 0.0;

  // Row 3: [0, 0, 0, 1]
  R[12] = 0.0;
  R[13] = 0.0;
  R[14] = 0.0;
  R[15] = 1.0;
}

// ============================================================================
// 4×4 matrix multiply: C = A * B (all row-major)
// ============================================================================
__device__ __forceinline__ void mat44_mul(const double* A, const double* B, double* C) {
  for (int r = 0; r < 4; ++r) {
    double a0 = A[r * 4 + 0];
    double a1 = A[r * 4 + 1];
    double a2 = A[r * 4 + 2];
    double a3 = A[r * 4 + 3];
    C[r * 4 + 0] = a0 * B[0] + a1 * B[4] + a2 * B[8]  + a3 * B[12];
    C[r * 4 + 1] = a0 * B[1] + a1 * B[5] + a2 * B[9]  + a3 * B[13];
    C[r * 4 + 2] = a0 * B[2] + a1 * B[6] + a2 * B[10] + a3 * B[14];
    C[r * 4 + 3] = a0 * B[3] + a1 * B[7] + a2 * B[11] + a3 * B[15];
  }
}

// ============================================================================
// Forward kinematics — URDF convention (matches CPU robot_model.cpp exactly)
//
// Algorithm:
//   T = I₄
//   for each revolute segment i in {0..5}:
//       T = T * c_segment_origins[i]
//       T = T * Rodrigues(c_segment_axes[i], q[c_q_index[i]])
//   T = T * c_T_wrist3_to_tcp
//
// Returns T_base_to_tcp as 4×4 row-major homogeneous matrix.
// ============================================================================
__device__ __forceinline__ void forward_kinematics(const double* q, double* T_tip) {
  // Initialize to I₄
  for (int i = 0; i < 16; ++i) T_tip[i] = (i % 5 == 0) ? 1.0 : 0.0;

  double T_tmp[16], R[16];

  for (int seg = 0; seg < 6; ++seg) {
    // Step 1: T = T * origin
    mat44_mul(T_tip, &c_segment_origins[seg * 16], T_tmp);
    for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];

    // Step 2: T = T * Rodrigues(axis, q[index])
    double theta = q[c_q_index[seg]];
    build_rotation_matrix(c_segment_axes[seg * 3 + 0],
                          c_segment_axes[seg * 3 + 1],
                          c_segment_axes[seg * 3 + 2],
                          theta, R);
    mat44_mul(T_tip, R, T_tmp);
    for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];
  }

  // Step 3: T = T * T_wrist3_to_tcp (fixed tool offset)
  mat44_mul(T_tip, c_T_wrist3_to_tcp, T_tmp);
  for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];
}

// ============================================================================
// Pose error (6-DOF) between current and target transforms
//   err[0:2] = position error (3)
//   err[3:5] = rotation error in world frame (3)
// ============================================================================
__device__ __forceinline__ void pose_error(const double* T_cur, const double* T_tgt,
                                            double* err) {
  // Position error
  err[0] = T_tgt[3]  - T_cur[3];
  err[1] = T_tgt[7]  - T_cur[7];
  err[2] = T_tgt[11] - T_cur[11];

  // Rotation error: R_err = R_cur^T * R_tgt → axis-angle
  double r00 = T_cur[0], r01 = T_cur[1], r02 = T_cur[2];
  double r10 = T_cur[4], r11 = T_cur[5], r12 = T_cur[6];
  double r20 = T_cur[8], r21 = T_cur[9], r22 = T_cur[10];

  double t00 = T_tgt[0], t01 = T_tgt[1], t02 = T_tgt[2];
  double t10 = T_tgt[4], t11 = T_tgt[5], t12 = T_tgt[6];
  double t20 = T_tgt[8], t21 = T_tgt[9], t22 = T_tgt[10];

  double e00 = r00 * t00 + r10 * t10 + r20 * t20;
  double e01 = r00 * t01 + r10 * t11 + r20 * t21;
  double e02 = r00 * t02 + r10 * t12 + r20 * t22;
  double e10 = r01 * t00 + r11 * t10 + r21 * t20;
  double e11 = r01 * t01 + r11 * t11 + r21 * t21;
  double e12 = r01 * t02 + r11 * t12 + r21 * t22;
  double e20 = r02 * t00 + r12 * t10 + r22 * t20;
  double e21 = r02 * t01 + r12 * t11 + r22 * t21;
  double e22 = r02 * t02 + r12 * t12 + r22 * t22;

  double trace = e00 + e11 + e22;
  double angle = acos(cuda_clamp((trace - 1.0) * 0.5, -1.0, 1.0));

  if (angle < 1e-12 || angle > CUDA_PI - 1e-12) {
    err[3] = 0.0; err[4] = 0.0; err[5] = 0.0;
  } else {
    double s = 0.5 * angle / sin(angle);
    double wx = s * (e21 - e12);
    double wy = s * (e02 - e20);
    double wz = s * (e10 - e01);
    err[3] = T_cur[0] * wx + T_cur[1] * wy + T_cur[2] * wz;
    err[4] = T_cur[4] * wx + T_cur[5] * wy + T_cur[6] * wz;
    err[5] = T_cur[8] * wx + T_cur[9] * wy + T_cur[10] * wz;
  }
}

// ============================================================================
// Shovel kinematics — complete tool chain analysis
//
// Tool chain: UR10 wrist3 → ur10-sensor_shovel → sensor_shovel_tcp_fixed
//   T_wrist3_to_sensor_shovel: rpy=(-1.5707963, 0, 0), xyz=(0, 0.09, 0)
//   T_sensor_shovel_to_tcp: rpy=(-1.5708, 1.5708, -0.61087), xyz=(-0.47377, 0.077109, 0.0733)
//   T_wrist3_to_tcp = T_wrist3_to_sensor_shovel * T_sensor_shovel_to_tcp
//
// The shovel TCP (sensor_shovel_tcp_fixed) is the actual trajectory fitting target.
// These functions enable analysis of shovel-tip accuracy vs wrist accuracy.
// ============================================================================

// Compute TCP (shovel tip) transform from wrist3 transform using tool offset.
// T_tcp = T_wrist3 * T_wrist3_to_tcp  (row-major 4×4 homogeneous matrices)
__device__ __forceinline__ void shovel_tcp_transform(
    const double* T_wrist3, const double* T_wrist3_to_tcp, double* T_tcp) {
  mat44_mul(T_wrist3, T_wrist3_to_tcp, T_tcp);
}

// Shovel TCP pose error: position and orientation errors at the shovel tip.
// Computes:
//   pos_err = Euclidean distance between current and target TCP positions
//   rot_err = geodesic angular distance on SO(3) between orientations
// The errors are in world frame, directly measuring shovel-tip tracking accuracy.
__device__ __forceinline__ void shovel_pose_error(
    const double* T_tcp_cur, const double* T_tcp_tgt,
    double* pos_err, double* rot_err) {
  // Position error (world-frame Euclidean distance at shovel tip)
  double dx = T_tcp_tgt[3]  - T_tcp_cur[3];
  double dy = T_tcp_tgt[7]  - T_tcp_cur[7];
  double dz = T_tcp_tgt[11] - T_tcp_cur[11];
  *pos_err = sqrt(dx * dx + dy * dy + dz * dz);

  // Orientation error via geodesic distance on SO(3)
  // R_err = R_cur^T * R_tgt, trace gives cos(θ) = (trace(R_err) - 1) / 2
  double r00 = T_tcp_cur[0], r01 = T_tcp_cur[1], r02 = T_tcp_cur[2];
  double r10 = T_tcp_cur[4], r11 = T_tcp_cur[5], r12 = T_tcp_cur[6];
  double r20 = T_tcp_cur[8], r21 = T_tcp_cur[9], r22 = T_tcp_cur[10];

  double t00 = T_tcp_tgt[0], t01 = T_tcp_tgt[1], t02 = T_tcp_tgt[2];
  double t10 = T_tcp_tgt[4], t11 = T_tcp_tgt[5], t12 = T_tcp_tgt[6];
  double t20 = T_tcp_tgt[8], t21 = T_tcp_tgt[9], t22 = T_tcp_tgt[10];

  // Only need diagonal of R_err = R_cur^T * R_tgt for trace
  double e00 = r00 * t00 + r10 * t10 + r20 * t20;
  double e11 = r01 * t01 + r11 * t11 + r21 * t21;
  double e22 = r02 * t02 + r12 * t12 + r22 * t22;

  double trace = e00 + e11 + e22;
  *rot_err = acos(cuda_clamp((trace - 1.0) * 0.5, -1.0, 1.0));
}

// Extract shovel TCP position vector from 4×4 homogeneous transform.
// Returns (x, y, z) of the translation component.
__device__ __forceinline__ void shovel_tcp_position(
    const double* T_tcp, double* px, double* py, double* pz) {
  *px = T_tcp[3];
  *py = T_tcp[7];
  *pz = T_tcp[11];
}

// ============================================================================
// 6×6 LDL^T Cholesky decomposition and solve (lane 0 serial)
//   Solves: H * x = g  where H = L * D * L^T (6×6, SPD)
//   Returns x in dq. ~63 scalar ops, ~0.1 μs on GPU.
// ============================================================================
__device__ __forceinline__ void ldlt_solve_6x6(const double* H, const double* g, double* dq) {
  double L[6][6] = {{0}};
  double D[6] = {0};
  double y[6] = {0};
  double x[6] = {0};

  double A[6][6];
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      A[i][j] = H[i * 6 + j];

  // LDL^T decomposition
  for (int j = 0; j < 6; ++j) {
    double d = A[j][j];
    for (int k = 0; k < j; ++k)
      d -= L[j][k] * L[j][k] * D[k];
    D[j] = d;

    for (int i = j + 1; i < 6; ++i) {
      double sum = A[i][j];
      for (int k = 0; k < j; ++k)
        sum -= L[i][k] * L[j][k] * D[k];
      L[i][j] = sum / D[j];
    }
    L[j][j] = 1.0;
  }

  // Forward substitution: L * y = g
  for (int i = 0; i < 6; ++i) {
    double sum = g[i];
    for (int k = 0; k < i; ++k)
      sum -= L[i][k] * y[k];
    y[i] = sum;
  }

  // Diagonal scaling: z = y / D
  for (int i = 0; i < 6; ++i)
    y[i] = y[i] / D[i];

  // Backward substitution: L^T * x = z
  for (int i = 5; i >= 0; --i) {
    double sum = y[i];
    for (int k = i + 1; k < 6; ++k)
      sum -= L[k][i] * x[k];
    x[i] = sum;
  }

  for (int i = 0; i < 6; ++i) dq[i] = x[i];
}

}  // namespace cuda
}  // namespace rtfg
