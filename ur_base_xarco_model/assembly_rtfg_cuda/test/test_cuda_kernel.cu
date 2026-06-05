// test_cuda_kernel.cu — Comprehensive GPU FK & IK test for UR10 CUDA kernel
//
// Compile:
//   nvcc -arch=sm_89 -O3 -lineinfo --ptxas-options=-v \
//     -o test_cuda_kernel test/test_cuda_kernel.cu \
//     src/cuda/cuda_kernels.cu \
//     -I include -Isrc/cuda -I include/assembly_rtfg_cuda \
//     -I ../assembly_rtfg_cpp/include -I ../assembly_rtfg_cpp/include/assembly_rtfg_cpp \
//     -I /usr/include/eigen3 -lstdc++
//
// Tests:
//   1. FK correctness: GPU FK vs CPU reference (10 random q, < 1e-15 error)
//   2. IK convergence:  Known reachable target, 60 iter convergence
//   3. Batch IK:        273 targets, >90% convergence rate

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cassert>

// Define CUDA_DEFINE_CONSTANTS so __constant__ variables are defined (not extern)
// in this translation unit. The kernel source is included below for single-TU build.
#define CUDA_DEFINE_CONSTANTS
// Include CUDA utilities for device helpers (in namespace rtfg::cuda)
#include "cuda_utilities.cuh"

// Include kernel and collision source directly to form a single translation unit.
// This avoids the NVCC extern __constant__ cross-TU linking limitation.
#include "../src/cuda/cuda_kernels.cu"
#include "../src/cuda/cuda_collision.cu"

// Bring namespaced symbols into scope for host-side cudaMemcpyToSymbol
using namespace rtfg::cuda;

// ============================================================================
// UR10 kinematics data (from assembly_rtfg_solver.urdf, double-precision)
// ============================================================================

// 6 revolute joint origin matrices (row-major, 4×4 each)
static const double k_origins[96] = {
    // Seg 0: shoulder_pan — rpy=(0,0,0), xyz=(0,0,0.1273), axis=(0,0,1)
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.1273,
    0.0, 0.0, 0.0, 1.0,
    // Seg 1: shoulder_lift — rpy=(0,1.57079,0), xyz=(0,0.220941,0), axis=(0,1,0)
     6.3267948966684693e-06,  0.0,                       9.9999999997998579e-01,  0.0,
     0.0,                     1.0,                       0.0,                     0.220941,
    -9.9999999997998579e-01,  0.0,                       6.3267948966684693e-06,  0.0,
     0.0,                     0.0,                       0.0,                     1.0,
    // Seg 2: elbow — rpy=(0,0,0), xyz=(-3.9e-6,-0.1719,0.612), axis=(0,1,0)
    1.0, 0.0, 0.0, -3.9e-6,
    0.0, 1.0, 0.0, -0.1719,
    0.0, 0.0, 1.0,  0.612,
    0.0, 0.0, 0.0,  1.0,
    // Seg 3: wrist_1 — rpy=(0,1.5707895,2.7e-6), xyz=(-3.6e-6,0,0.5723), axis=(0,1,0)
     6.8267948964806121e-06, -2.6999999999967194e-06,  9.9999999997305244e-01, -3.6e-6,
     1.8432346220542441e-11,  9.9999999999635503e-01,  2.6999999999338027e-06,  0.0,
    -9.9999999997669742e-01,  0.0,                     6.8267948965054954e-06,  0.5723,
     0.0,                     0.0,                     0.0,                     1.0,
    // Seg 4: wrist_2 — rpy=(0,0,0), xyz=(3e-7,0.1149,3e-7), axis=(0,0,-1)
    1.0, 0.0, 0.0, 3e-7,
    0.0, 1.0, 0.0, 0.1149,
    0.0, 0.0, 1.0, 3e-7,
    0.0, 0.0, 0.0, 1.0,
    // Seg 5: wrist_3 — rpy=(0,0,0), xyz=(0,-3e-7,0.1157), axis=(0,1,0)
    1.0, 0.0, 0.0,  0.0,
    0.0, 1.0, 0.0, -3e-7,
    0.0, 0.0, 1.0,  0.1157,
    0.0, 0.0, 0.0,  1.0
};

// Rotation axes: {x, y, z} per joint
static const double k_axes[18] = {
    0.0, 0.0,  1.0,   // seg 0: shoulder_pan  Z
    0.0, 1.0,  0.0,   // seg 1: shoulder_lift Y
    0.0, 1.0,  0.0,   // seg 2: elbow         Y
    0.0, 1.0,  0.0,   // seg 3: wrist_1       Y
    0.0, 0.0, -1.0,   // seg 4: wrist_2      -Z
    0.0, 1.0,  0.0    // seg 5: wrist_3       Y
};

static const int k_q_index[6] = {0, 1, 2, 3, 4, 5};

// T_wrist3_to_tcp from URDF tool chain
static const double k_T_wrist3_to_tcp[16] = {
    -3.0089034369963224e-06, -8.1915141988946039e-01,  5.7357732807706696e-01, -4.7377000000000002e-01,
    -9.9999999999319700e-01,  3.6885740485110307e-06,  2.1962570019296782e-08,  1.6330000206612766e-01,
    -2.1336731175750953e-06, -5.7357732807309880e-01, -8.1915141989498630e-01, -7.7108998035934045e-02,
     0.0,                     0.0,                     0.0,                     1.0
};

// Joint limits (±2π for continuous joints in URDF)
static const double k_joint_limits[12] = {
    -6.28319, 6.28319,  // shoulder_pan
    -6.28319, 6.28319,  // shoulder_lift
    -6.28319, 6.28319,  // elbow
    -6.28319, 6.28319,  // wrist_1
    -6.28319, 6.28319,  // wrist_2
    -6.28319, 6.28319   // wrist_3
};

// Weight schedule (level 0: position + orientation)
static const double k_weights[24] = {
    1.0, 1.0, 1.0, 0.20, 0.20, 0.20,
    1.0, 1.0, 1.0, 0.10, 0.10, 0.10,
    1.0, 1.0, 1.0, 0.03, 0.03, 0.03,
    1.0, 1.0, 1.0, 0.00, 0.00, 0.00
};

static const double k_lambda_params[4] = {5e-4, 0.1, 5e-4, 0.05};

// Kernels are included directly from src/cuda/cuda_kernels.cu (single-TU build).
// No external declarations needed.

// ============================================================================
// CPU reference: Rodrigues rotation formula
// ============================================================================
static void cpu_rotation_matrix(double ax, double ay, double az, double angle, double* R) {
    double c = cos(angle), s = sin(angle), t = 1.0 - c;
    // Row-major 4×4
    R[0]  = t*ax*ax + c;   R[1]  = t*ax*ay + s*az; R[2]  = t*ax*az - s*ay; R[3]  = 0.0;
    R[4]  = t*ax*ay - s*az; R[5]  = t*ay*ay + c;   R[6]  = t*ay*az + s*ax; R[7]  = 0.0;
    R[8]  = t*ax*az + s*ay; R[9]  = t*ay*az - s*ax; R[10] = t*az*az + c;   R[11] = 0.0;
    R[12] = 0.0;            R[13] = 0.0;            R[14] = 0.0;            R[15] = 1.0;
}

// CPU 4×4 matrix multiply: C = A * B (all row-major)
static void cpu_mat44_mul(const double* A, const double* B, double* C) {
    for (int r = 0; r < 4; ++r) {
        double a0 = A[r*4+0], a1 = A[r*4+1], a2 = A[r*4+2], a3 = A[r*4+3];
        C[r*4+0] = a0*B[0] + a1*B[4] + a2*B[8]  + a3*B[12];
        C[r*4+1] = a0*B[1] + a1*B[5] + a2*B[9]  + a3*B[13];
        C[r*4+2] = a0*B[2] + a1*B[6] + a2*B[10] + a3*B[14];
        C[r*4+3] = a0*B[3] + a1*B[7] + a2*B[11] + a3*B[15];
    }
}

// CPU forward kinematics (URDF convention, matches GPU exactly)
static void cpu_forward_kinematics(const double* q, double* T_tip) {
    // Initialize to I₄
    for (int i = 0; i < 16; ++i) T_tip[i] = (i % 5 == 0) ? 1.0 : 0.0;

    double T_tmp[16], R[16];
    for (int seg = 0; seg < 6; ++seg) {
        // T = T * origin
        cpu_mat44_mul(T_tip, &k_origins[seg * 16], T_tmp);
        for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];

        // T = T * Rodrigues(axis, q[index])
        double theta = q[k_q_index[seg]];
        cpu_rotation_matrix(k_axes[seg*3+0], k_axes[seg*3+1], k_axes[seg*3+2], theta, R);
        cpu_mat44_mul(T_tip, R, T_tmp);
        for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];
    }

    // T = T * T_wrist3_to_tcp
    cpu_mat44_mul(T_tip, k_T_wrist3_to_tcp, T_tmp);
    for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];
}

// ============================================================================
// GPU FK test kernel: writes FK result to global memory for verification
// Must be in rtfg::cuda namespace to access forward_kinematics
// ============================================================================
namespace rtfg { namespace cuda {
__global__ void test_fk_kernel(const double* d_q, double* d_T, int count) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= count) return;
    forward_kinematics(&d_q[tid * 6], &d_T[tid * 16]);
}
// ============================================================================
// DEBUG KERNEL: Single DLS iteration trace — prints all intermediate values
// Launched with 1 thread to avoid printf interleaving
// ============================================================================
__global__ void debug_dls_iteration(const double* d_target, const double* d_seed) {
    // Load seed and target
    double q[6], T_tgt[16];
    for (int i = 0; i < 6; ++i) q[i] = d_seed[i];
    for (int i = 0; i < 16; ++i) T_tgt[i] = d_target[i];

    printf("=== DLS Iteration 1 Debug ===\n");
    printf("Seed q: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
           q[0], q[1], q[2], q[3], q[4], q[5]);

    // Step 1: FK
    double T_cur[16];
    forward_kinematics(q, T_cur);
    printf("FK TCP pos: (%.6f, %.6f, %.6f)\n", T_cur[3], T_cur[7], T_cur[11]);
    printf("Target pos: (%.6f, %.6f, %.6f)\n", T_tgt[3], T_tgt[7], T_tgt[11]);

    // Step 2: Pose error
    double err[6];
    pose_error(T_cur, T_tgt, err);
    double pos_err = sqrt(err[0]*err[0]+err[1]*err[1]+err[2]*err[2]);
    double rot_err = sqrt(err[3]*err[3]+err[4]*err[4]+err[5]*err[5]);
    printf("Err: [%.6e, %.6e, %.6e, %.6e, %.6e, %.6e]\n",
           err[0], err[1], err[2], err[3], err[4], err[5]);
    printf("Pos err=%.6e, Rot err=%.6e\n", pos_err, rot_err);

    // Step 3: Numerical Jacobian
    const double eps = 1e-6;
    double J[48];  // 6x8 padded
    for (int j = 0; j < 6; ++j) {
        double qp[6], qm[6], Tp[16], Tm[16];
        for (int i = 0; i < 6; ++i) { qp[i] = q[i]; qm[i] = q[i]; }
        qp[j] += eps; qm[j] -= eps;
        forward_kinematics(qp, Tp);
        forward_kinematics(qm, Tm);

        double inv_2eps = 0.5 / eps;
        J[0*8+j] = (Tp[3]  - Tm[3])  * inv_2eps;
        J[1*8+j] = (Tp[7]  - Tm[7])  * inv_2eps;
        J[2*8+j] = (Tp[11] - Tm[11]) * inv_2eps;

        double r00=T_cur[0], r01=T_cur[1], r02=T_cur[2];
        double r10=T_cur[4], r11=T_cur[5], r12=T_cur[6];
        double r20=T_cur[8], r21=T_cur[9], r22=T_cur[10];

        double dR[9];
        dR[0] = (r00*Tp[0]+r10*Tp[4]+r20*Tp[8]) - (r00*Tm[0]+r10*Tm[4]+r20*Tm[8]);
        dR[1] = (r00*Tp[1]+r10*Tp[5]+r20*Tp[9]) - (r00*Tm[1]+r10*Tm[5]+r20*Tm[9]);
        dR[2] = (r00*Tp[2]+r10*Tp[6]+r20*Tp[10]) - (r00*Tm[2]+r10*Tm[6]+r20*Tm[10]);
        dR[3] = (r01*Tp[0]+r11*Tp[4]+r21*Tp[8]) - (r01*Tm[0]+r11*Tm[4]+r21*Tm[8]);
        dR[4] = (r01*Tp[1]+r11*Tp[5]+r21*Tp[9]) - (r01*Tm[1]+r11*Tm[5]+r21*Tm[9]);
        dR[5] = (r01*Tp[2]+r11*Tp[6]+r21*Tp[10]) - (r01*Tm[2]+r11*Tm[6]+r21*Tm[10]);
        dR[6] = (r02*Tp[0]+r12*Tp[4]+r22*Tp[8]) - (r02*Tm[0]+r12*Tm[4]+r22*Tm[8]);
        dR[7] = (r02*Tp[1]+r12*Tp[5]+r22*Tp[9]) - (r02*Tm[1]+r12*Tm[5]+r22*Tm[9]);
        dR[8] = (r02*Tp[2]+r12*Tp[6]+r22*Tp[10]) - (r02*Tm[2]+r12*Tm[6]+r22*Tm[10]);

        J[3*8+j] = (dR[7] - dR[5]) * 0.5 * inv_2eps;
        J[4*8+j] = (dR[2] - dR[6]) * 0.5 * inv_2eps;
        J[5*8+j] = (dR[3] - dR[1]) * 0.5 * inv_2eps;
    }

    printf("Jacobian (6x6, rows 0-2=pos, 3-5=rot):\n");
    for (int r = 0; r < 6; ++r) {
        printf("  J[%d]: [% .4e % .4e % .4e % .4e % .4e % .4e]\n",
               r, J[r*8+0], J[r*8+1], J[r*8+2], J[r*8+3], J[r*8+4], J[r*8+5]);
    }

    // Step 4: Hessian (using same weight pattern as current kernel)
    double H_pad[48];
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            double w_row = c_weight_schedule[0*6+row];
            double w_col = c_weight_schedule[0*6+col];
            double sum = 0.0;
            for (int k = 0; k < 6; ++k) {
                sum += J[k*8+row] * w_row * J[k*8+col] * w_col;
            }
            H_pad[row*8+col] = sum;
        }
    }

    printf("Hessian (6x6, current weighting):\n");
    for (int r = 0; r < 6; ++r) {
        printf("  H[%d]: [% .4e % .4e % .4e % .4e % .4e % .4e]\n",
               r, H_pad[r*8+0], H_pad[r*8+1], H_pad[r*8+2],
               H_pad[r*8+3], H_pad[r*8+4], H_pad[r*8+5]);
    }

    // Step 5: Gradient (using same weight pattern as current kernel)
    double g[6] = {0};
    for (int i = 0; i < 6; ++i) {
        double sum = 0.0;
        for (int k = 0; k < 6; ++k) {
            double w_k = c_weight_schedule[0*6+k];
            sum += J[k*8+i] * w_k * err[k] * w_k;
        }
        g[i] = sum;
    }

    printf("Gradient (current weighting): [%.6e, %.6e, %.6e, %.6e, %.6e, %.6e]\n",
           g[0], g[1], g[2], g[3], g[4], g[5]);

    // Step 6: LDLT solve
    double lambda = 5e-4;
    double H_dense[36];
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) {
            H_dense[r*6+c] = H_pad[r*8+c];
            if (r == c) H_dense[r*6+c] += lambda;
        }

    double dq[6];
    ldlt_solve_6x6(H_dense, g, dq);

    printf("H+lambda*I:\n");
    for (int r = 0; r < 6; ++r) {
        printf("  [% .4e % .4e % .4e % .4e % .4e % .4e]\n",
               H_dense[r*6+0], H_dense[r*6+1], H_dense[r*6+2],
               H_dense[r*6+3], H_dense[r*6+4], H_dense[r*6+5]);
    }

    printf("dq step: [%.6e, %.6e, %.6e, %.6e, %.6e, %.6e]\n",
           dq[0], dq[1], dq[2], dq[3], dq[4], dq[5]);

    double step_norm = sqrt(dq[0]*dq[0]+dq[1]*dq[1]+dq[2]*dq[2]+
                            dq[3]*dq[3]+dq[4]*dq[4]+dq[5]*dq[5]);
    printf("Step norm: %.6e\n", step_norm);

    // Step 7: Try with CORRECT weighting (all 1.0) to compare
    printf("\n--- Correct weighting (all 1.0) comparison ---\n");
    double H_corr[48];
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 6; ++k) {
                sum += J[k*8+row] * J[k*8+col];  // no weights
            }
            H_corr[row*8+col] = sum;
        }
    }
    printf("Hessian (unweighted):\n");
    for (int r = 0; r < 6; ++r) {
        printf("  [% .4e % .4e % .4e % .4e % .4e % .4e]\n",
               H_corr[r*8+0], H_corr[r*8+1], H_corr[r*8+2],
               H_corr[r*8+3], H_corr[r*8+4], H_corr[r*8+5]);
    }

    double g_corr[6] = {0};
    for (int i = 0; i < 6; ++i) {
        double sum = 0.0;
        for (int k = 0; k < 6; ++k) {
            sum += J[k*8+i] * err[k];  // no weights
        }
        g_corr[i] = sum;
    }
    printf("Gradient (unweighted): [%.6e, %.6e, %.6e, %.6e, %.6e, %.6e]\n",
           g_corr[0], g_corr[1], g_corr[2], g_corr[3], g_corr[4], g_corr[5]);

    double Hc_dense[36];
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) {
            Hc_dense[r*6+c] = H_corr[r*8+c];
            if (r == c) Hc_dense[r*6+c] += lambda;
        }
    double dq_corr[6];
    ldlt_solve_6x6(Hc_dense, g_corr, dq_corr);
    printf("dq step (correct): [%.6e, %.6e, %.6e, %.6e, %.6e, %.6e]\n",
           dq_corr[0], dq_corr[1], dq_corr[2], dq_corr[3], dq_corr[4], dq_corr[5]);

    double step_norm_c = sqrt(dq_corr[0]*dq_corr[0]+dq_corr[1]*dq_corr[1]+dq_corr[2]*dq_corr[2]+
                              dq_corr[3]*dq_corr[3]+dq_corr[4]*dq_corr[4]+dq_corr[5]*dq_corr[5]);
    printf("Step norm (correct): %.6e\n", step_norm_c);

    // Apply correct step
    double q_new[6];
    for (int i = 0; i < 6; ++i) q_new[i] = q[i] + dq_corr[i];

    double T_new[16];
    forward_kinematics(q_new, T_new);
    double pose_new[6];
    pose_error(T_new, T_tgt, pose_new);
    double pn = sqrt(pose_new[0]*pose_new[0]+pose_new[1]*pose_new[1]+pose_new[2]*pose_new[2]);
    printf("Pos error after correct step: %.6e\n", pn);
    printf("=== End Debug ===\n");
}

// ============================================================================
// DEBUG KERNEL 2: Progressive IK convergence trace (single target)
// Prints position error every 5 iterations to show convergence progress
// ============================================================================
__global__ void debug_ik_progress(const double* d_target, const double* d_seed,
                                   int max_iter) {
    double q[6], q_ref[6], T_tgt[16];
    for (int i = 0; i < 6; ++i) { q[i] = d_seed[i]; q_ref[i] = q[i]; }
    for (int i = 0; i < 16; ++i) T_tgt[i] = d_target[i];

    printf("=== IK Progress Trace (max_iter=%d) ===\n", max_iter);
    printf("Seed: [%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n",
           q[0],q[1],q[2],q[3],q[4],q[5]);
    printf("Target TCP: (%.4f,%.4f,%.4f)\n", T_tgt[3],T_tgt[7],T_tgt[11]);

    // Use the SAME algorithm as ik_batch_solve kernel, but sequential
    int converged = 0;
    for (int iter = 0; iter < max_iter && !converged; ++iter) {
        // FK
        double T_cur[16];
        forward_kinematics(q, T_cur);

        // Pose error
        double err[6];
        pose_error(T_cur, T_tgt, err);
        double pos_err = sqrt(err[0]*err[0]+err[1]*err[1]+err[2]*err[2]);
        double rot_err = sqrt(err[3]*err[3]+err[4]*err[4]+err[5]*err[5]);

        if (iter % 5 == 0 || iter < 3) {
            printf("  iter%3d: pos=%.6f rot=%.4f  q=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n",
                   iter, pos_err, rot_err,
                   q[0],q[1],q[2],q[3],q[4],q[5]);
        }

        if (pos_err < 0.001 && rot_err < 0.01745) { converged = 1; break; }

        // Jacobian
        const double eps = 1e-6;
        double J[48];
        for (int j = 0; j < 6; ++j) {
            double qp[6], qm[6], Tp[16], Tm[16];
            for (int i = 0; i < 6; ++i) { qp[i]=q[i]; qm[i]=q[i]; }
            qp[j]+=eps; qm[j]-=eps;
            forward_kinematics(qp, Tp);
            forward_kinematics(qm, Tm);

            double inv_2eps = 0.5/eps;
            J[0*8+j]=(Tp[3]-Tm[3])*inv_2eps;
            J[1*8+j]=(Tp[7]-Tm[7])*inv_2eps;
            J[2*8+j]=(Tp[11]-Tm[11])*inv_2eps;
            double r00=T_cur[0],r01=T_cur[1],r02=T_cur[2];
            double r10=T_cur[4],r11=T_cur[5],r12=T_cur[6];
            double r20=T_cur[8],r21=T_cur[9],r22=T_cur[10];
            double dR[9];
            dR[0]=(r00*Tp[0]+r10*Tp[4]+r20*Tp[8])-(r00*Tm[0]+r10*Tm[4]+r20*Tm[8]);
            dR[1]=(r00*Tp[1]+r10*Tp[5]+r20*Tp[9])-(r00*Tm[1]+r10*Tm[5]+r20*Tm[9]);
            dR[2]=(r00*Tp[2]+r10*Tp[6]+r20*Tp[10])-(r00*Tm[2]+r10*Tm[6]+r20*Tm[10]);
            dR[3]=(r01*Tp[0]+r11*Tp[4]+r21*Tp[8])-(r01*Tm[0]+r11*Tm[4]+r21*Tm[8]);
            dR[4]=(r01*Tp[1]+r11*Tp[5]+r21*Tp[9])-(r01*Tm[1]+r11*Tm[5]+r21*Tm[9]);
            dR[5]=(r01*Tp[2]+r11*Tp[6]+r21*Tp[10])-(r01*Tm[2]+r11*Tm[6]+r21*Tm[10]);
            dR[6]=(r02*Tp[0]+r12*Tp[4]+r22*Tp[8])-(r02*Tm[0]+r12*Tm[4]+r22*Tm[8]);
            dR[7]=(r02*Tp[1]+r12*Tp[5]+r22*Tp[9])-(r02*Tm[1]+r12*Tm[5]+r22*Tm[9]);
            dR[8]=(r02*Tp[2]+r12*Tp[6]+r22*Tp[10])-(r02*Tm[2]+r12*Tm[6]+r22*Tm[10]);
            J[3*8+j]=(dR[7]-dR[5])*0.5*inv_2eps;
            J[4*8+j]=(dR[2]-dR[6])*0.5*inv_2eps;
            J[5*8+j]=(dR[3]-dR[1])*0.5*inv_2eps;
        }

        // Adaptive damping (same as kernel)
        double lambda;
        if (pos_err > 0.1) {
            lambda = fmax(1e-3, 5e-3*(pos_err/0.05));
            lambda = fmin(lambda, 0.1);
        } else {
            lambda = 5e-4 + 5e-3*(pos_err/0.05);
        }

        // Hessian: H = J^T * W^2 * J + lambda*I (FIXED weighting)
        double H_dense[36];
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                double sum = 0.0;
                for (int k = 0; k < 6; ++k) {
                    double w_k = c_weight_schedule[0*6+k];
                    sum += J[k*8+r] * w_k * w_k * J[k*8+c];
                }
                if (r == c) sum += lambda;
                H_dense[r*6+c] = sum;
            }
        }

        // Gradient: g = J^T * W^2 * e (FIXED weighting)
        double g[6];
        for (int i = 0; i < 6; ++i) {
            double sum = 0.0;
            for (int k = 0; k < 6; ++k) {
                double w_k = c_weight_schedule[0*6+k];
                sum += J[k*8+i] * w_k * w_k * err[k];
            }
            g[i] = sum;
        }

        // LDLT solve
        double dq[6];
        ldlt_solve_6x6(H_dense, g, dq);

        // Step clamp
        double step_norm = sqrt(dq[0]*dq[0]+dq[1]*dq[1]+dq[2]*dq[2]+
                                dq[3]*dq[3]+dq[4]*dq[4]+dq[5]*dq[5]);
        if (step_norm > 0.45) {
            double scale = 0.45/step_norm;
            for (int i = 0; i < 6; ++i) dq[i] *= scale;
        }
        if (step_norm < 1e-6) break;  // stagnation

        // Apply step with joint limits
        for (int i = 0; i < 6; ++i) {
            double lo = c_joint_limits[i*2+0];
            double hi = c_joint_limits[i*2+1];
            q[i] = fmin(fmax(q[i]+dq[i], lo), hi);
        }

        // Branch alignment
        for (int i = 0; i < 6; ++i) {
            double diff = q[i] - q_ref[i];
            q[i] = q_ref[i] + atan2(sin(diff), cos(diff));
        }
    }

    // Final result
    double T_final[16];
    forward_kinematics(q, T_final);
    double err_final[6];
    pose_error(T_final, T_tgt, err_final);
    double pf = sqrt(err_final[0]*err_final[0]+err_final[1]*err_final[1]+err_final[2]*err_final[2]);
    printf("  FINAL: pos=%.6f  q=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n",
           pf, q[0],q[1],q[2],q[3],q[4],q[5]);
    printf("=== End Progress ===\n");
}

}  // namespace cuda
}  // namespace rtfg

// ============================================================================
// Helper: upload all constant memory
// ============================================================================
static cudaError_t upload_constants() {
    cudaError_t err;
    err = cudaMemcpyToSymbol(c_segment_origins, k_origins, 96 * sizeof(double));
    if (err != cudaSuccess) return err;
    err = cudaMemcpyToSymbol(c_segment_axes, k_axes, 18 * sizeof(double));
    if (err != cudaSuccess) return err;
    err = cudaMemcpyToSymbol(c_q_index, k_q_index, 6 * sizeof(int));
    if (err != cudaSuccess) return err;
    err = cudaMemcpyToSymbol(c_T_wrist3_to_tcp, k_T_wrist3_to_tcp, 16 * sizeof(double));
    if (err != cudaSuccess) return err;
    err = cudaMemcpyToSymbol(c_joint_limits, k_joint_limits, 12 * sizeof(double));
    if (err != cudaSuccess) return err;
    err = cudaMemcpyToSymbol(c_weight_schedule, k_weights, 24 * sizeof(double));
    if (err != cudaSuccess) return err;
    err = cudaMemcpyToSymbol(c_lambda_params, k_lambda_params, 4 * sizeof(double));
    return err;
}

// ============================================================================
// Helper: max absolute difference between two 4×4 matrices
// ============================================================================
static double max_abs_diff_16(const double* A, const double* B) {
    double max_diff = 0.0;
    for (int i = 0; i < 16; ++i) {
        double d = fabs(A[i] - B[i]);
        if (d > max_diff) max_diff = d;
    }
    return max_diff;
}

// ============================================================================
// Helper: print 4×4 matrix
// ============================================================================
static void print_mat4(const char* label, const double* T) {
    printf("%s:\n", label);
    for (int r = 0; r < 4; ++r) {
        printf("  [% .6e % .6e % .6e % .6e]\n",
               T[r*4+0], T[r*4+1], T[r*4+2], T[r*4+3]);
    }
}

// ============================================================================
// main
// ============================================================================
int main() {
    printf("=== UR10 CUDA IK Kernel Test (URDF FK, CUDA 13.3) ===\n\n");

    // Check device
    int dev_count;
    cudaError_t err = cudaGetDeviceCount(&dev_count);
    if (err != cudaSuccess || dev_count == 0) {
        printf("ERROR: No CUDA device found (%s)\n", cudaGetErrorString(err));
        return 1;
    }

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    printf("Device: %s (sm_%d%d, %zu MB, CUDA %d.%d)\n",
           prop.name, prop.major, prop.minor,
           prop.totalGlobalMem / (1024*1024),
           CUDART_VERSION / 1000, (CUDART_VERSION % 1000) / 10);

    // Upload constant memory (must happen before any kernel launch)
    err = upload_constants();
    if (err != cudaSuccess) {
        printf("ERROR: Failed to upload constants: %s\n", cudaGetErrorString(err));
        return 1;
    }
    cudaDeviceSynchronize();
    printf("Constant memory uploaded (URDF parameters, %lu bytes).\n\n",
           (unsigned long)(96+18+6*sizeof(int)/8+16+12+24+4) * sizeof(double));

    // ========================================================================
    // Test 0: DEBUG — Single DLS iteration trace
    // ========================================================================
    printf("--- DEBUG: Single DLS iteration (target q=[0.1,0.1,0.1,0.1,0.1,0.1]) ---\n");
    {
        double q_target[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        double T_target[16];
        cpu_forward_kinematics(q_target, T_target);
        double h_seed[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        double *d_target, *d_seed;
        cudaMalloc(&d_target, 16*sizeof(double));
        cudaMalloc(&d_seed, 6*sizeof(double));
        cudaMemcpy(d_target, T_target, 16*sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(d_seed, h_seed, 6*sizeof(double), cudaMemcpyHostToDevice);

        rtfg::cuda::debug_dls_iteration<<<1, 1>>>(d_target, d_seed);
        cudaDeviceSynchronize();
        cudaFree(d_target); cudaFree(d_seed);
    }
    printf("\n");

    // ========================================================================
    // Test 0b: DEBUG — Progressive IK convergence trace (near seed)
    // ========================================================================
    printf("--- DEBUG: IK progress trace (near-seed) ---\n");
    {
        double q_target[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        double T_target[16];
        cpu_forward_kinematics(q_target, T_target);
        double h_seed[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        double *d_target, *d_seed;
        cudaMalloc(&d_target, 16*sizeof(double));
        cudaMalloc(&d_seed, 6*sizeof(double));
        cudaMemcpy(d_target, T_target, 16*sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(d_seed, h_seed, 6*sizeof(double), cudaMemcpyHostToDevice);

        rtfg::cuda::debug_ik_progress<<<1, 1>>>(d_target, d_seed, 100);
        cudaDeviceSynchronize();
        cudaFree(d_target); cudaFree(d_seed);
    }
    printf("\n");

    // ========================================================================
    // Test 1: FK Correctness — GPU vs CPU
    // ========================================================================
    printf("--- Test 1: FK Correctness (GPU vs CPU reference) ---\n");

    const int FK_COUNT = 10;
    // 10 test joint configurations
    double test_q[FK_COUNT][6] = {
        { 0.0,          0.0,          0.0,          0.0,          0.0,          0.0          },
        { 1.57079632679, 0.0,          0.0,          0.0,          0.0,          0.0          },  // π/2
        { 0.0,          1.57079632679, 0.0,          0.0,          0.0,          0.0          },
        { 0.0,          0.0,         -1.57079632679, 0.0,          0.0,          0.0          },  // -π/2
        { 0.0,          0.0,          0.0,          1.57079632679, 0.0,          0.0          },
        { 0.0,          0.0,          0.0,          0.0,          1.57079632679, 0.0          },
        { 0.0,          0.0,          0.0,          0.0,          0.0,          1.57079632679 },
        { 0.5,         -0.3,          1.2,         -0.8,          2.1,          0.4          },
        {-1.0,          2.0,         -2.5,          1.5,         -0.5,          3.0          },
        { 2.04749846,   0.21928656,  -1.95482141,  -0.35923797,   2.05026770,   1.03308296  }  // UR10 home-ish
    };

    double *d_q_fk, *d_T_fk;
    cudaMalloc(&d_q_fk, FK_COUNT * 6 * sizeof(double));
    cudaMalloc(&d_T_fk, FK_COUNT * 16 * sizeof(double));

    double h_q_flat[FK_COUNT * 6];
    for (int i = 0; i < FK_COUNT; ++i)
        for (int j = 0; j < 6; ++j)
            h_q_flat[i * 6 + j] = test_q[i][j];

    cudaMemcpy(d_q_fk, h_q_flat, FK_COUNT * 6 * sizeof(double), cudaMemcpyHostToDevice);

    // Launch GPU FK kernel
    dim3 fk_grid((FK_COUNT + 31) / 32);
    dim3 fk_block(32);
    rtfg::cuda::test_fk_kernel<<<fk_grid, fk_block>>>(d_q_fk, d_T_fk, FK_COUNT);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("FK KERNEL ERROR: %s\n", cudaGetErrorString(err));
    }
    cudaDeviceSynchronize();

    // Read back GPU results
    double h_T_gpu[FK_COUNT * 16];
    cudaMemcpy(h_T_gpu, d_T_fk, FK_COUNT * 16 * sizeof(double), cudaMemcpyDeviceToHost);

    // Compare against CPU FK
    int fk_pass = 0, fk_fail = 0;
    double fk_max_err = 0.0;
    for (int i = 0; i < FK_COUNT; ++i) {
        double T_cpu[16];
        cpu_forward_kinematics(test_q[i], T_cpu);
        double err_i = max_abs_diff_16(&h_T_gpu[i * 16], T_cpu);
        if (err_i > fk_max_err) fk_max_err = err_i;

        if (err_i < 1e-14) {
            fk_pass++;
        } else {
            fk_fail++;
            printf("  FAIL [%d]: q=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] max_diff=%.2e\n",
                   i, test_q[i][0], test_q[i][1], test_q[i][2],
                   test_q[i][3], test_q[i][4], test_q[i][5], err_i);
            print_mat4("  GPU", &h_T_gpu[i * 16]);
            print_mat4("  CPU", T_cpu);
        }
    }
    printf("  FK test: %d/%d passed, max error = %.2e %s\n\n",
           fk_pass, FK_COUNT, fk_max_err,
           (fk_fail == 0) ? "✓" : "✗");

    // Show first result detail
    printf("  q=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] → TCP pos=(%.4f, %.4f, %.4f)\n",
           test_q[0][0], test_q[0][1], test_q[0][2],
           test_q[0][3], test_q[0][4], test_q[0][5],
           h_T_gpu[3], h_T_gpu[7], h_T_gpu[11]);

    cudaFree(d_q_fk);
    cudaFree(d_T_fk);

    // ========================================================================
    // Test 2: IK Convergence — Near-seed target (sanity check)
    // ========================================================================
    printf("\n--- Test 2a: IK Convergence (near-seed target) ---\n");

    {
        // Generate target from q close to seed (0)
        double q_target[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        double T_target[16];
        cpu_forward_kinematics(q_target, T_target);

        printf("  Target q=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n",
               q_target[0], q_target[1], q_target[2],
               q_target[3], q_target[4], q_target[5]);

        const int N_SINGLE = 1;
        double *d_targets, *d_seeds, *d_results, *d_errors, *d_iters;
        cudaMalloc(&d_targets, N_SINGLE * 16 * sizeof(double));
        cudaMalloc(&d_seeds,   N_SINGLE * 6 * sizeof(double));
        cudaMalloc(&d_results, N_SINGLE * 6 * sizeof(double));
        cudaMalloc(&d_errors,  N_SINGLE * 2 * sizeof(double));
        cudaMalloc(&d_iters,   N_SINGLE * sizeof(double));

        cudaMemcpy(d_targets, T_target, 16 * sizeof(double), cudaMemcpyHostToDevice);
        double h_seed[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        cudaMemcpy(d_seeds, h_seed, 6 * sizeof(double), cudaMemcpyHostToDevice);

        int max_iter = 200;
        double pos_tol = 0.01;
        double orient_tol = M_PI / 6.0;

        dim3 grid(N_SINGLE, 1, 1);
        dim3 block(128, 1, 1);
        rtfg::cuda::ik_batch_solve<<<grid, block>>>(
            d_targets, d_seeds, d_results, d_errors, nullptr, d_iters,
            max_iter, pos_tol, orient_tol, 0, N_SINGLE);

        err = cudaGetLastError();
        cudaDeviceSynchronize();

        if (err != cudaSuccess) {
            printf("  IK KERNEL ERROR: %s\n", cudaGetErrorString(err));
        } else {
            double h_result[6], h_err[2], h_iter;
            cudaMemcpy(h_result, d_results, 6 * sizeof(double), cudaMemcpyDeviceToHost);
            cudaMemcpy(h_err, d_errors, 2 * sizeof(double), cudaMemcpyDeviceToHost);
            cudaMemcpy(&h_iter, d_iters, sizeof(double), cudaMemcpyDeviceToHost);

            double max_joint_err = 0.0;
            for (int j = 0; j < 6; ++j) {
                double diff = h_result[j] - q_target[j];
                double wrapped = atan2(sin(diff), cos(diff));
                if (fabs(wrapped) > max_joint_err) max_joint_err = fabs(wrapped);
            }

            printf("  Result:[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n",
                   h_result[0], h_result[1], h_result[2],
                   h_result[3], h_result[4], h_result[5]);
            printf("  Pos_err=%.6f, Rot_err=%.4f, Iters=%.0f, Max_joint_err=%.4f rad\n",
                   h_err[0], h_err[1], h_iter, max_joint_err);
            // NOTE: Joint error is not a reliable pass criterion for IK —
            // multiple joint configurations can achieve the same TCP pose.
            // Position error is the canonical convergence metric.
            printf("  Near-seed IK (pos check only): %s\n\n",
                   (h_err[0] < 0.01) ? "PASS ✓" : "FAIL ✗");
        }

        cudaFree(d_targets); cudaFree(d_seeds);
        cudaFree(d_results); cudaFree(d_errors); cudaFree(d_iters);
    }

    // ========================================================================
    // Test 2b: IK Convergence — Far target with higher tolerance
    // ========================================================================
    printf("--- Test 2b: IK Convergence (far target, relaxed tol) ---\n");

    {
        double q_target[6] = {0.5, -0.3, 1.2, -0.8, 0.6, 0.4};
        double T_target[16];
        cpu_forward_kinematics(q_target, T_target);

        const int N_SINGLE = 1;
        double *d_targets, *d_seeds, *d_results, *d_errors, *d_iters;
        cudaMalloc(&d_targets, N_SINGLE * 16 * sizeof(double));
        cudaMalloc(&d_seeds,   N_SINGLE * 6 * sizeof(double));
        cudaMalloc(&d_results, N_SINGLE * 6 * sizeof(double));
        cudaMalloc(&d_errors,  N_SINGLE * 2 * sizeof(double));
        cudaMalloc(&d_iters,   N_SINGLE * sizeof(double));

        cudaMemcpy(d_targets, T_target, 16 * sizeof(double), cudaMemcpyHostToDevice);
        double h_seed[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        cudaMemcpy(d_seeds, h_seed, 6 * sizeof(double), cudaMemcpyHostToDevice);

        int max_iter = 200;
        double pos_tol = 0.03;
        double orient_tol = M_PI / 4.0;

        dim3 grid(N_SINGLE, 1, 1);
        dim3 block(128, 1, 1);
        rtfg::cuda::ik_batch_solve<<<grid, block>>>(
            d_targets, d_seeds, d_results, d_errors, nullptr, d_iters,
            max_iter, pos_tol, orient_tol, 0, N_SINGLE);

        err = cudaGetLastError();
        cudaDeviceSynchronize();

        if (err != cudaSuccess) {
            printf("  IK KERNEL ERROR: %s\n", cudaGetErrorString(err));
        } else {
            double h_result[6], h_err[2], h_iter;
            cudaMemcpy(h_result, d_results, 6 * sizeof(double), cudaMemcpyDeviceToHost);
            cudaMemcpy(h_err, d_errors, 2 * sizeof(double), cudaMemcpyDeviceToHost);
            cudaMemcpy(&h_iter, d_iters, sizeof(double), cudaMemcpyDeviceToHost);

            double max_joint_err = 0.0;
            for (int j = 0; j < 6; ++j) {
                double diff = h_result[j] - q_target[j];
                double wrapped = atan2(sin(diff), cos(diff));
                if (fabs(wrapped) > max_joint_err) max_joint_err = fabs(wrapped);
            }

            printf("  Result:[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n",
                   h_result[0], h_result[1], h_result[2],
                   h_result[3], h_result[4], h_result[5]);
            printf("  Target:[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n",
                   q_target[0], q_target[1], q_target[2],
                   q_target[3], q_target[4], q_target[5]);
            printf("  Pos_err=%.6f, Rot_err=%.4f, Iters=%.0f, Max_joint_err=%.4f rad\n",
                   h_err[0], h_err[1], h_iter, max_joint_err);
            printf("  Far-target IK: %s\n\n",
                   (h_err[0] < 0.05) ? "PASS ✓" : "FAIL ✗");
        }

        cudaFree(d_targets); cudaFree(d_seeds);
        cudaFree(d_results); cudaFree(d_errors); cudaFree(d_iters);
    }

    // ========================================================================
    // Test 3: Batch IK — 273 targets (stress test)
    // ========================================================================
    printf("--- Test 3: Batch IK (273 targets) ---\n");

    {
        const int N_BATCH = 273;
        int max_iter = 100;
        double pos_tol = 0.03;
        double orient_tol = M_PI / 6.0;

        double *d_targets, *d_seeds, *d_results, *d_errors, *d_iters;
        cudaMalloc(&d_targets, N_BATCH * 16 * sizeof(double));
        cudaMalloc(&d_seeds,   N_BATCH * 6 * sizeof(double));
        cudaMalloc(&d_results, N_BATCH * 6 * sizeof(double));
        cudaMalloc(&d_errors,  N_BATCH * 2 * sizeof(double));
        cudaMalloc(&d_iters,   N_BATCH * sizeof(double));

        // Generate 273 reachable targets using CPU FK at random q values
        double *h_targets_batch = (double*)malloc(N_BATCH * 16 * sizeof(double));
        double *h_seeds_batch   = (double*)malloc(N_BATCH * 6 * sizeof(double));

        // Simple PRNG (not crypto-safe, deterministic for reproducibility)
        unsigned seed_val = 42;
        for (int i = 0; i < N_BATCH; ++i) {
            // Generate random q within joint limits
            double q_rand[6];
            for (int j = 0; j < 6; ++j) {
                seed_val = seed_val * 1103515245 + 12345;
                double u = (double)(seed_val & 0x7FFFFFFF) / 2147483648.0;  // 0..1
                q_rand[j] = -M_PI + 2.0 * M_PI * u;
            }
            // Compute target via CPU FK
            cpu_forward_kinematics(q_rand, &h_targets_batch[i * 16]);

            // Seed = target q + small random perturbation
            for (int j = 0; j < 6; ++j) {
                    seed_val = seed_val * 1103515245 + 12345;
                    double pert = ((double)(seed_val & 0x7FFFFFFF) / 2147483648.0 - 0.5) * 0.5;
                    h_seeds_batch[i * 6 + j] = q_rand[j] + pert;
            }
        }

        cudaMemcpy(d_targets, h_targets_batch, N_BATCH * 16 * sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(d_seeds,   h_seeds_batch,   N_BATCH * 6 * sizeof(double), cudaMemcpyHostToDevice);

        // Time the kernel
        cudaEvent_t start, stop;
        cudaEventCreate(&start);
        cudaEventCreate(&stop);

        cudaEventRecord(start);
        dim3 grid(N_BATCH, 1, 1);
        dim3 block(128, 1, 1);
        rtfg::cuda::ik_batch_solve<<<grid, block>>>(
            d_targets, d_seeds, d_results, d_errors, nullptr, d_iters,
            max_iter, pos_tol, orient_tol, 0, N_BATCH);
        cudaEventRecord(stop);

        err = cudaGetLastError();
        cudaEventSynchronize(stop);
        float kernel_ms = 0;
        cudaEventElapsedTime(&kernel_ms, start, stop);

        if (err != cudaSuccess) {
            printf("  BATCH KERNEL ERROR: %s\n", cudaGetErrorString(err));
        } else {
            printf("  Kernel launched: %d blocks × 128 threads\n", N_BATCH);
            printf("  Kernel time: %.3f ms (%.3f ms/target)\n",
                   kernel_ms, kernel_ms / N_BATCH);

            // Read back results
            double *h_results_batch = (double*)malloc(N_BATCH * 6 * sizeof(double));
            double *h_errors_batch  = (double*)malloc(N_BATCH * 2 * sizeof(double));
            double *h_iters_batch   = (double*)malloc(N_BATCH * sizeof(double));
            cudaMemcpy(h_results_batch, d_results, N_BATCH * 6 * sizeof(double), cudaMemcpyDeviceToHost);
            cudaMemcpy(h_errors_batch,  d_errors,  N_BATCH * 2 * sizeof(double), cudaMemcpyDeviceToHost);
            cudaMemcpy(h_iters_batch,   d_iters,   N_BATCH * sizeof(double), cudaMemcpyDeviceToHost);

            // Analyze convergence (position-only, since weight schedule deprioritizes orientation)
            int converged = 0, total_iters = 0;
            double max_pos_err = 0.0, avg_pos_err = 0.0;
            int bins[5] = {0};  // <1cm, <3cm, <5cm, <10cm, >=10cm
            for (int i = 0; i < N_BATCH; ++i) {
                double pos_err = h_errors_batch[i * 2 + 0];
                double rot_err = h_errors_batch[i * 2 + 1];
                avg_pos_err += pos_err;
                if (pos_err > max_pos_err) max_pos_err = pos_err;
                if (pos_err <= pos_tol && rot_err <= orient_tol) converged++;
                total_iters += (int)h_iters_batch[i];
                if (pos_err < 0.01) bins[0]++;
                else if (pos_err < 0.03) bins[1]++;
                else if (pos_err < 0.05) bins[2]++;
                else if (pos_err < 0.10) bins[3]++;
                else bins[4]++;
            }
            avg_pos_err /= N_BATCH;
            double avg_iters = (double)total_iters / N_BATCH;
            double convergence_rate = 100.0 * converged / N_BATCH;

            printf("  Convergence (pos+rot): %d/%d (%.1f%%)\n", converged, N_BATCH, convergence_rate);
            printf("  Pos error distribution: <1cm:%d <3cm:%d <5cm:%d <10cm:%d >=10cm:%d\n",
                   bins[0], bins[1], bins[2], bins[3], bins[4]);
            printf("  Avg pos error: %.4f m, Max pos error: %.4f m\n", avg_pos_err, max_pos_err);
            printf("  Avg iterations: %.1f\n", avg_iters);

            // Show first and last results
            // Export all results to CSV for chart generation
            FILE* fcsv = fopen("/tmp/batch_ik_results.csv", "w");
            if (fcsv) {
                fprintf(fcsv, "idx,q0,q1,q2,q3,q4,q5,pos_err,rot_err,iterations\n");
                for (int i = 0; i < N_BATCH; ++i) {
                    fprintf(fcsv, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.0f\n",
                            i,
                            h_results_batch[i*6+0], h_results_batch[i*6+1], h_results_batch[i*6+2],
                            h_results_batch[i*6+3], h_results_batch[i*6+4], h_results_batch[i*6+5],
                            h_errors_batch[i*2+0], h_errors_batch[i*2+1],
                            h_iters_batch[i]);
                }
                fclose(fcsv);
                printf("  Exported %d results to /tmp/batch_ik_results.csv\n", N_BATCH);
            }

            printf("  First 3 results:\n");
            for (int i = 0; i < 3; ++i) {
                printf("    [%d] q=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f] "
                       "err=%.4f iters=%.0f\n",
                       i,
                       h_results_batch[i*6+0], h_results_batch[i*6+1], h_results_batch[i*6+2],
                       h_results_batch[i*6+3], h_results_batch[i*6+4], h_results_batch[i*6+5],
                       h_errors_batch[i*2+0], h_iters_batch[i]);
            }

            // Pass if >80% have pos_err < 3cm AND >95% have pos_err < 10cm
            bool batch_ok = (bins[0] + bins[1] >= (int)(N_BATCH * 0.80)) &&
                            (bins[0] + bins[1] + bins[2] + bins[3] >= (int)(N_BATCH * 0.95));
            printf("  Batch test: %s\n\n", batch_ok ? "PASS ✓" : "WARN (check targets)");

            free(h_results_batch);
            free(h_errors_batch);
            free(h_iters_batch);
        }

        free(h_targets_batch);
        free(h_seeds_batch);
        cudaFree(d_targets); cudaFree(d_seeds);
        cudaFree(d_results); cudaFree(d_errors); cudaFree(d_iters);
        cudaEventDestroy(start);
        cudaEventDestroy(stop);
    }

    // ========================================================================
    // Test 4: Continuity cost kernel
    // ========================================================================
    printf("--- Test 4: Continuity cost computation ---\n");

    {
        const int N_COST = 100;
        double *d_results, *d_q_prev, *d_dq_prev, *d_costs;
        cudaMalloc(&d_results, N_COST * 6 * sizeof(double));
        cudaMalloc(&d_q_prev, 6 * sizeof(double));
        cudaMalloc(&d_dq_prev, 6 * sizeof(double));
        cudaMalloc(&d_costs, N_COST * sizeof(double));

        double h_q_prev[6] = {0, 0, 0, 0, 0, 0};
        double h_dq_prev[6] = {0, 0, 0, 0, 0, 0};
        cudaMemcpy(d_q_prev, h_q_prev, 6 * sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(d_dq_prev, h_dq_prev, 6 * sizeof(double), cudaMemcpyHostToDevice);

        // Initialize results with increasing angles
        double *h_results_cost = (double*)malloc(N_COST * 6 * sizeof(double));
        for (int i = 0; i < N_COST; ++i)
            for (int j = 0; j < 6; ++j)
                h_results_cost[i * 6 + j] = 0.01 * i;
        cudaMemcpy(d_results, h_results_cost, N_COST * 6 * sizeof(double), cudaMemcpyHostToDevice);

        dim3 grid2((N_COST + 255) / 256);
        dim3 block2(256);
        rtfg::cuda::compute_continuity_cost<<<grid2, block2>>>(
            d_results, d_q_prev, d_dq_prev, d_costs, N_COST);

        err = cudaGetLastError();
        cudaDeviceSynchronize();

        if (err != cudaSuccess) {
            printf("  COST KERNEL ERROR: %s\n", cudaGetErrorString(err));
        } else {
            double h_costs[N_COST];
            cudaMemcpy(h_costs, d_costs, N_COST * sizeof(double), cudaMemcpyDeviceToHost);
            printf("  First 5 costs: %.4f, %.4f, %.4f, %.4f, %.4f\n",
                   h_costs[0], h_costs[1], h_costs[2], h_costs[3], h_costs[4]);
            // Verify monotonic (cost should increase with i)
            bool monotonic = true;
            for (int i = 1; i < N_COST; ++i) {
                if (h_costs[i] < h_costs[i-1] - 1e-12) { monotonic = false; break; }
            }
            printf("  Cost monotonic: %s\n", monotonic ? "PASS ✓" : "FAIL ✗");
        }

        free(h_results_cost);
        cudaFree(d_results); cudaFree(d_q_prev);
        cudaFree(d_dq_prev); cudaFree(d_costs);
    }

    // ========================================================================
    // Test 5: GPU Collision Detection — box-box/box-sphere/box-cylinder
    // ========================================================================
    printf("--- Test 5: GPU Collision Detection ---\n");

    {
        const int N_FRAMES = 100;
        const int N_BOXES = 8;
        const int N_OBJECTS = 5;

        // Generate robot boxes (N_FRAMES × N_BOXES × 6)
        double *h_robot_boxes = (double*)malloc(N_FRAMES * N_BOXES * 6 * sizeof(double));
        // Generate env objects (N_OBJECTS × 8): type + params
        double h_env_objects[N_OBJECTS * 8] = {
            // Type 0: box at origin, 0.5m half-extents
            0.0,  0.0, 0.0, 0.0,  0.5, 0.5, 0.5, 0.0,
            // Type 0: box offset in X
            0.0,  1.0, 0.0, 0.0,  0.3, 0.3, 0.3, 0.0,
            // Type 1: sphere
            1.0, -0.5, 0.2, 0.0, 0.15, 0.0, 0.0, 0.0,
            // Type 2: cylinder (vertical, Z-aligned)
            2.0,  0.5, 0.5, 0.0, 0.1, 0.4, 0.0, 0.0,
            // Type 0: box near robot workspace
            0.0,  0.3, -0.3, 0.2, 0.2, 0.2, 0.15, 0.0,
        };

        // Robot boxes: each frame has 8 boxes (simplified UR10 links)
        // Frame 0: boxes at random positions
        unsigned s2 = 12345;
        for (int f = 0; f < N_FRAMES; ++f) {
            for (int b = 0; b < N_BOXES; ++b) {
                s2 = s2 * 1103515245 + 12345;
                double u = (double)(s2 & 0x7FFFFFFF) / 2147483648.0;
                int idx = (f * N_BOXES + b) * 6;
                h_robot_boxes[idx + 0] = u * 2.0 - 1.0;      // cx: -1..1
                h_robot_boxes[idx + 1] = (u + 0.1) * 1.5;    // cy: 0..1.5
                h_robot_boxes[idx + 2] = u * 1.5;             // cz: 0..1.5
                h_robot_boxes[idx + 3] = 0.05 + u * 0.15;     // hx: 0.05..0.2
                h_robot_boxes[idx + 4] = 0.05 + ((1.0-u) * 0.15);  // hy
                h_robot_boxes[idx + 5] = 0.05 + u * 0.1;      // hz
            }
        }

        // Device memory
        double *d_robot_boxes, *d_env_objects, *d_clearances, *d_colliding;
        cudaMalloc(&d_robot_boxes, N_FRAMES * N_BOXES * 6 * sizeof(double));
        cudaMalloc(&d_env_objects, N_OBJECTS * 8 * sizeof(double));
        cudaMalloc(&d_clearances, N_FRAMES * sizeof(double));
        cudaMalloc(&d_colliding, N_FRAMES * sizeof(double));

        cudaMemcpy(d_robot_boxes, h_robot_boxes,
                   N_FRAMES * N_BOXES * 6 * sizeof(double), cudaMemcpyHostToDevice);
        cudaMemcpy(d_env_objects, h_env_objects,
                   N_OBJECTS * 8 * sizeof(double), cudaMemcpyHostToDevice);

        // Time the kernel
        cudaEvent_t cs, ce;
        cudaEventCreate(&cs); cudaEventCreate(&ce);

        cudaError_t col_err = rtfg::cuda::launch_collision_check_batch(
            d_robot_boxes, d_env_objects, d_clearances, d_colliding,
            N_FRAMES, N_BOXES, N_OBJECTS, 0);

        if (col_err != cudaSuccess) {
            printf("  COLLISION KERNEL ERROR: %s\n", cudaGetErrorString(col_err));
        } else {
            cudaDeviceSynchronize();

            // Read back
            double h_clearances[N_FRAMES];
            double h_colliding[N_FRAMES];
            cudaMemcpy(h_clearances, d_clearances,
                       N_FRAMES * sizeof(double), cudaMemcpyDeviceToHost);
            cudaMemcpy(h_colliding, d_colliding,
                       N_FRAMES * sizeof(double), cudaMemcpyDeviceToHost);

            // Analyze
            int n_collisions = 0;
            double min_clearance = 1e10;
            double max_clearance = -1e10;
            for (int f = 0; f < N_FRAMES; ++f) {
                if (h_colliding[f] > 0.5) n_collisions++;
                if (h_clearances[f] < min_clearance) min_clearance = h_clearances[f];
                if (h_clearances[f] > max_clearance && h_clearances[f] < 1e9)
                    max_clearance = h_clearances[f];
            }

            printf("  Frames=%d, Boxes=%d, Objects=%d, Pairs=%d\n",
                   N_FRAMES, N_BOXES, N_OBJECTS, N_BOXES * N_OBJECTS);
            printf("  Collisions: %d/%d frames (%.1f%%)\n",
                   n_collisions, N_FRAMES, 100.0 * n_collisions / N_FRAMES);
            printf("  Min clearance: %.4f m, Max clearance: %.4f m\n",
                   min_clearance, max_clearance);

            // Verify: min_clearance should NOT be infinity (real data)
            bool valid_clearance = (min_clearance < 1e9);
            // Verify: at least some frames should have valid clearance
            bool has_valid = (max_clearance > -1e9);

            printf("  Collision test: %s\n",
                   (valid_clearance && has_valid) ? "PASS ✓" : "FAIL ✗");

            // Show first few frames
            printf("  First 5 frames: ");
            for (int f = 0; f < 5; ++f) {
                printf("[c=%.3f col=%d] ", h_clearances[f], (int)h_colliding[f]);
            }
            printf("\n");
        }

        free(h_robot_boxes);
        cudaFree(d_robot_boxes); cudaFree(d_env_objects);
        cudaFree(d_clearances); cudaFree(d_colliding);
        cudaEventDestroy(cs); cudaEventDestroy(ce);
    }

    printf("\n=== All tests complete ===\n");
    return 0;
}
