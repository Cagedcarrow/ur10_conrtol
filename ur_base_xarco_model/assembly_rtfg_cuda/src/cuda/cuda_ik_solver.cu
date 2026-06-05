// cuda_ik_solver.cu — CudaBatchIK class implementation (CUDA 13.3)
//
// Bridges the ROS2 IKSolverBase interface with CUDA batch kernel execution.
// Manages device memory lifecycle, constant memory initialization, and
// kernel launch orchestration.
//
// Key fix (Task 2): Segment index bug — original code iterated first 6 segments
// including 2 fixed joints, missing wrist_2 and wrist_3. Now skips fixed
// segments and correctly packs all 6 revolute joints.

#include "cuda_ik_solver.h"
#include "cuda_memory.h"
#include "cuda_kernels.h"
#include "cuda_utilities.cuh"
#include "ik_solver.h"        // clampToLimits, alignToReference, continuityCost
#include "utils.h"             // poseError, rad2deg, wrapJointDelta

#include <Eigen/Dense>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <cassert>

// NOTE: __constant__ variables are declared extern in cuda_utilities.cuh (global scope)
// and defined in cuda_kernels.cu via CUDA_DEFINE_CONSTANTS.
// CUDA __constant__ does not support C++ namespaces — these are global symbols.
// Do NOT redeclare them inside namespace rtfg.

namespace rtfg {

// ============================================================================
// UR10 URDF segment origin matrices (computed from assembly_rtfg_solver.urdf)
//
// Convention: rpyToRot(roll, pitch, yaw) = Rz(yaw)·Ry(pitch)·Rx(roll)
//             T = [R | xyz; 0 0 0 1]   (row-major storage)
//
// All values computed at double precision; cos/sin match Eigen exactly.
// ============================================================================

namespace {

// Seg 0: ur10_shoulder_pan
//   origin: rpy=(0, 0, 0), xyz=(0, 0, 0.1273)
//   axis:   (0, 0, 1)
const double kSeg0_origin[16] = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.1273,
    0.0, 0.0, 0.0, 1.0
};

// Seg 1: ur10_shoulder_lift
//   origin: rpy=(0, 1.57079, 0), xyz=(0, 0.220941, 0)
//   axis:   (0, 1, 0)
const double kSeg1_origin[16] = {
     6.3267948966684693e-06,  0.0,                       9.9999999997998579e-01,  0.0,
     0.0,                     1.0,                       0.0,                     0.220941,
    -9.9999999997998579e-01,  0.0,                       6.3267948966684693e-06,  0.0,
     0.0,                     0.0,                       0.0,                     1.0
};

// Seg 2: ur10_elbow
//   origin: rpy=(0, 0, 0), xyz=(-3.9e-6, -0.1719, 0.612)
//   axis:   (0, 1, 0)
const double kSeg2_origin[16] = {
    1.0, 0.0, 0.0, -3.9e-6,
    0.0, 1.0, 0.0, -0.1719,
    0.0, 0.0, 1.0,  0.612,
    0.0, 0.0, 0.0,  1.0
};

// Seg 3: ur10_wrist_1
//   origin: rpy=(0, 1.5707895, 2.7e-6), xyz=(-3.6e-6, 0, 0.5723)
//   axis:   (0, 1, 0)
const double kSeg3_origin[16] = {
     6.8267948964806121e-06, -2.6999999999967194e-06,  9.9999999997305244e-01, -3.6e-6,
     1.8432346220542441e-11,  9.9999999999635503e-01,  2.6999999999338027e-06,  0.0,
    -9.9999999997669742e-01,  0.0,                     6.8267948965054954e-06,  0.5723,
     0.0,                     0.0,                     0.0,                     1.0
};

// Seg 4: ur10_wrist_2
//   origin: rpy=(0, 0, 0), xyz=(3e-7, 0.1149, 3e-7)
//   axis:   (0, 0, -1)
const double kSeg4_origin[16] = {
    1.0, 0.0, 0.0, 3e-7,
    0.0, 1.0, 0.0, 0.1149,
    0.0, 0.0, 1.0, 3e-7,
    0.0, 0.0, 0.0, 1.0
};

// Seg 5: ur10_wrist_3
//   origin: rpy=(0, 0, 0), xyz=(0, -3e-7, 0.1157)
//   axis:   (0, 1, 0)
const double kSeg5_origin[16] = {
    1.0, 0.0, 0.0,  0.0,
    0.0, 1.0, 0.0, -3e-7,
    0.0, 0.0, 1.0,  0.1157,
    0.0, 0.0, 0.0,  1.0
};

// Rotation axes (unit vectors, from URDF <axis> elements):
//   shoulder_pan: (0, 0, 1)
//   shoulder_lift: (0, 1, 0)
//   elbow:         (0, 1, 0)
//   wrist_1:       (0, 1, 0)
//   wrist_2:       (0, 0, -1)  ← NOTE: negative Z!
//   wrist_3:       (0, 1, 0)
const double kAxes[18] = {
    0.0, 0.0,  1.0,   // seg 0: shoulder_pan
    0.0, 1.0,  0.0,   // seg 1: shoulder_lift
    0.0, 1.0,  0.0,   // seg 2: elbow
    0.0, 1.0,  0.0,   // seg 3: wrist_1
    0.0, 0.0, -1.0,   // seg 4: wrist_2  (negative Z!)
    0.0, 1.0,  0.0    // seg 5: wrist_3
};

// q_index mapping: which joint angle drives each segment
const int kQIndex[6] = {0, 1, 2, 3, 4, 5};

// T_wrist3_to_tcp: fixed tool offset computed from URDF
// Tool chain: ur10_wrist_3 → ur10-sensor_shovel → sensor_shovel_tcp_fixed
//   ur10-sensor_shovel: rpy=(-1.5707963, 0, 0), xyz=(0, 0.09, 0)
//   sensor_shovel_tcp_fixed: rpy=(-1.5708, 1.5708, -0.61087), xyz=(-0.47377, 0.077109, 0.0733)
//   T_wrist3_to_tcp = T_ur10-sensor_shovel * T_sensor_shovel_tcp_fixed
const double kT_wrist3_to_tcp[16] = {
    -3.0089034369963224e-06, -8.1915141988946039e-01,  5.7357732807706696e-01, -4.7377000000000002e-01,
    -9.9999999999319700e-01,  3.6885740485110307e-06,  2.1962570019296782e-08,  1.6330000206612766e-01,
    -2.1336731175750953e-06, -5.7357732807309880e-01, -8.1915141989498630e-01, -7.7108998035934045e-02,
     0.0,                     0.0,                     0.0,                     1.0
};

}  // anonymous namespace

// ============================================================================
// Constructor / Destructor
// ============================================================================

CudaBatchIK::CudaBatchIK() {
  cudaError_t err = cudaStreamCreate(&stream_);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaStreamCreate failed: " +
                             std::string(cudaGetErrorString(err)));
  }
}

CudaBatchIK::~CudaBatchIK() {
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

// ============================================================================
// initialize() — Copy robot model parameters to GPU constant memory
//
// FIXED (Task 2): Now skips fixed segments and correctly packs all 6 revolute
// joints using URDF convention (segment origin + Rodrigues rotation axis).
// ============================================================================

void CudaBatchIK::initialize(const RobotModel& robot, const SolverConfig& cfg) {
  cfg_ = cfg;

  // --- Pack URDF segment origins (6×16 doubles, row-major) ---
  // Traverse all segments, skip fixed ones, pack 6 revolute joint origins.
  double h_origins[96] = {0};
  {
    int seg_idx = 0;
    for (size_t i = 0; i < robot.segments.size() && seg_idx < 6; ++i) {
      if (!robot.segments[i].movable) continue;  // Skip fixed joints
      const auto& seg = robot.segments[i];
      // Copy 4×4 origin matrix (Eigen stores column-major internally,
      // but Eigen::Matrix4d(r,c) gives the element. We convert to row-major.)
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          h_origins[seg_idx * 16 + r * 4 + c] = seg.origin(r, c);
      seg_idx++;
    }
    // Verify we packed exactly 6 revolute joints
    if (seg_idx != 6) {
      throw std::runtime_error(
          "Expected 6 revolute joints in URDF chain, found " +
          std::to_string(seg_idx));
    }
  }

  // --- Pack segment axes (6×3 doubles) ---
  // Same traversal: skip fixed, pack only revolute joint axes.
  double h_axes[18] = {0};
  {
    int seg_idx = 0;
    for (size_t i = 0; i < robot.segments.size() && seg_idx < 6; ++i) {
      if (!robot.segments[i].movable) continue;
      const auto& seg = robot.segments[i];
      h_axes[seg_idx * 3 + 0] = seg.axis.x();
      h_axes[seg_idx * 3 + 1] = seg.axis.y();
      h_axes[seg_idx * 3 + 2] = seg.axis.z();
      seg_idx++;
    }
  }

  // --- Pack q_index mapping ({0, 1, 2, 3, 4, 5}) ---
  // The 6 revolute joints in chain order map to q indices 0..5.
  int h_q_index[6] = {0};
  {
    int seg_idx = 0;
    for (size_t i = 0; i < robot.segments.size() && seg_idx < 6; ++i) {
      if (!robot.segments[i].movable) continue;
      h_q_index[seg_idx] = robot.segments[i].q_index;
      seg_idx++;
    }
  }

  // --- T_wrist3_to_tcp: use precomputed value from robot model ---
  double h_T_wrist3_to_tcp[16] = {0};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      h_T_wrist3_to_tcp[r * 4 + c] = robot.T_wrist3_to_tcp(r, c);

  // --- Pack joint limits ---
  // Only pack limits for revolute joints, indexed by q_index.
  double h_limits[12] = {0};
  {
    bool seen[6] = {false};
    for (const auto& seg : robot.segments) {
      if (seg.movable && seg.q_index >= 0 && seg.q_index < 6) {
        int qi = seg.q_index;
        h_limits[qi * 2 + 0] = seg.lower;
        h_limits[qi * 2 + 1] = seg.upper;
        seen[qi] = true;
      }
    }
    // Verify all 6 joint limits were set
    for (int i = 0; i < 6; ++i) {
      if (!seen[i]) {
        throw std::runtime_error("Missing joint limits for q_index " +
                                 std::to_string(i));
      }
    }
  }

  // --- Pack weight schedules (4 levels) ---
  // Level 0: pos=[1,1,1] rot=[0.20,0.20,0.20]  (most strict)
  // Level 1: pos=[1,1,1] rot=[0.10,0.10,0.10]
  // Level 2: pos=[1,1,1] rot=[0.03,0.03,0.03]
  // Level 3: pos=[1,1,1] rot=[0.00,0.00,0.00]  (position only)
  double h_weights[24] = {
    1.0, 1.0, 1.0, 0.20, 0.20, 0.20,
    1.0, 1.0, 1.0, 0.10, 0.10, 0.10,
    1.0, 1.0, 1.0, 0.03, 0.03, 0.03,
    1.0, 1.0, 1.0, 0.00, 0.00, 0.00
  };

  // --- Pack damping params ---
  double h_lambda[4] = {
    cfg.lambda,           // [0] base lambda
    0.1,                  // [1] far-distance lambda max
    5e-4,                 // [2] near-distance lambda floor
    0.05                  // [3] scaling reference distance
  };

  // --- Copy to GPU constant memory ---
  {
    auto check_sym = [](cudaError_t e, const char* name) {
      if (e != cudaSuccess)
        throw std::runtime_error(std::string("cudaMemcpyToSymbol(") + name +
                                 ") failed: " + cudaGetErrorString(e));
    };
    check_sym(cudaMemcpyToSymbol(c_segment_origins, h_origins, 96 * sizeof(double)),
              "c_segment_origins");
    check_sym(cudaMemcpyToSymbol(c_segment_axes, h_axes, 18 * sizeof(double)),
              "c_segment_axes");
    check_sym(cudaMemcpyToSymbol(c_q_index, h_q_index, 6 * sizeof(int)),
              "c_q_index");
    check_sym(cudaMemcpyToSymbol(c_T_wrist3_to_tcp, h_T_wrist3_to_tcp, 16 * sizeof(double)),
              "c_T_wrist3_to_tcp");
    check_sym(cudaMemcpyToSymbol(c_joint_limits, h_limits, 12 * sizeof(double)),
              "c_joint_limits");
    check_sym(cudaMemcpyToSymbol(c_weight_schedule, h_weights, 24 * sizeof(double)),
              "c_weight_schedule");
    check_sym(cudaMemcpyToSymbol(c_lambda_params, h_lambda, 4 * sizeof(double)),
              "c_lambda_params");
  }

  cudaDeviceSynchronize();
  initialized_ = true;
}

// ============================================================================
// ensureCapacity() — Resize device buffers for N targets
// ============================================================================

void CudaBatchIK::ensureCapacity(int N) {
  if (!d_targets_ || d_targets_->size() < static_cast<size_t>(N * 16)) {
    d_targets_ = std::make_unique<cuda::DeviceBuffer<double>>(N * 16);
  }
  if (!d_seeds_ || d_seeds_->size() < static_cast<size_t>(N * 6)) {
    d_seeds_ = std::make_unique<cuda::DeviceBuffer<double>>(N * 6);
  }
  if (!d_results_ || d_results_->size() < static_cast<size_t>(N * 6)) {
    d_results_ = std::make_unique<cuda::DeviceBuffer<double>>(N * 6);
  }
  if (!d_errors_ || d_errors_->size() < static_cast<size_t>(N * 2)) {
    d_errors_ = std::make_unique<cuda::DeviceBuffer<double>>(N * 2);
  }
  if (!d_shovel_errors_ || d_shovel_errors_->size() < static_cast<size_t>(N * 2)) {
    d_shovel_errors_ = std::make_unique<cuda::DeviceBuffer<double>>(N * 2);
  }
  if (!d_iterations_ || d_iterations_->size() < static_cast<size_t>(N)) {
    d_iterations_ = std::make_unique<cuda::DeviceBuffer<double>>(N);
  }
}

// ============================================================================
// enqueue() — Add a target pose to the pending batch
// ============================================================================

void CudaBatchIK::enqueue(const Mat4& target, const Eigen::VectorXd& seed,
                           const std::array<double, 6>& weights,
                           double orient_limit) {
  PendingTarget pt;
  pt.target = target;
  pt.seed = seed;
  pt.weights = weights;
  pt.orient_limit = orient_limit;
  pending_.push_back(pt);
}

// ============================================================================
// flush() — Execute batch IK solve on GPU
// ============================================================================

void CudaBatchIK::flush(std::vector<CandidateInfo>& results) {
  if (pending_.empty()) return;
  if (!initialized_) {
    throw std::runtime_error("CudaBatchIK::flush() called before initialize()");
  }

  int N = static_cast<int>(pending_.size());
  ensureCapacity(N);

  // --- Pack host buffers (AoS → SoA for coalesced access) ---
  h_targets_.resize(N * 16);
  h_seeds_.resize(N * 6);

  for (int i = 0; i < N; ++i) {
    const Mat4& T = pending_[i].target;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        h_targets_[i * 16 + r * 4 + c] = T(r, c);
      }
    }
    for (int j = 0; j < 6; ++j) {
      h_seeds_[i * 6 + j] = pending_[i].seed(j);
    }
  }

  // --- Transfer to device ---
  d_targets_->toDevice(h_targets_.data());
  d_seeds_->toDevice(h_seeds_.data());

  // --- Launch kernel ---
  cuda::launch_ik_batch_solve(
      d_targets_->get(), d_seeds_->get(),
      d_results_->get(), d_errors_->get(), d_shovel_errors_->get(),
      d_iterations_->get(),
      cfg_.max_iterations,
      cfg_.ik_position_tolerance,
      pending_[0].orient_limit,
      0,    // weight_level: 0=full pos+rot, 1-2=reduced rot, 3=position-only
      N, stream_);

  cudaDeviceSynchronize();

  // --- Read back results ---
  std::vector<double> h_results(N * 6);
  std::vector<double> h_errors(N * 2);
  std::vector<double> h_shovel_errors(N * 2);
  std::vector<double> h_iters(N);
  d_results_->toHost(h_results.data());
  d_errors_->toHost(h_errors.data());
  d_shovel_errors_->toHost(h_shovel_errors.data());
  d_iterations_->toHost(h_iters.data());

  // --- Build CandidateInfo results ---
  results.clear();
  results.reserve(N);
  for (int i = 0; i < N; ++i) {
    CandidateInfo cand;
    cand.q.resize(6);
    for (int j = 0; j < 6; ++j) {
      cand.q(j) = h_results[i * 6 + j];
    }
    cand.pos_err = h_errors[i * 2 + 0];
    cand.rot_err = h_errors[i * 2 + 1];
    cand.shovel_pos_err = h_shovel_errors[i * 2 + 0];
    cand.shovel_rot_err = h_shovel_errors[i * 2 + 1];
    cand.iterations_used = static_cast<int>(h_iters[i]);
    cand.valid = (cand.pos_err < 1.0);
    cand.clearance = std::numeric_limits<double>::infinity();
    cand.cost = 0.0;
    cand.joint_cost = 0.0;
    results.push_back(cand);
  }

  pending_.clear();
}

// ============================================================================
// solveBatch() — Convenience: enqueue + flush
// ============================================================================

std::vector<CandidateInfo> CudaBatchIK::solveBatch(
    const std::vector<Mat4>& targets,
    const std::vector<Eigen::VectorXd>& seeds,
    const Eigen::VectorXd& q_prev,
    const Eigen::VectorXd& dq_prev) {
  clear();
  for (size_t i = 0; i < targets.size(); ++i) {
    std::array<double, 6> w = {1.0, 1.0, 1.0, 0.20, 0.20, 0.20};
    enqueue(targets[i], seeds[i], w, M_PI / 6.0);
  }
  std::vector<CandidateInfo> results;
  flush(results);
  return results;
}

// ============================================================================
// solve() — Single-pose solve (IKSolverBase interface)
// ============================================================================

CandidateInfo CudaBatchIK::solve(const RobotModel& robot,
                                  const std::vector<BasinBox>& basin_boxes,
                                  const SolverConfig& cfg,
                                  const Mat4& target, const Eigen::VectorXd& seed,
                                  const Eigen::VectorXd& q_prev,
                                  const Eigen::VectorXd& dq_prev,
                                  const std::array<double, 6>& weights,
                                  double orient_limit) const {
  CudaBatchIK* self = const_cast<CudaBatchIK*>(this);
  if (!initialized_) {
    self->initialize(robot, cfg);
  }

  self->clear();
  self->enqueue(target, seed, weights, orient_limit);

  std::vector<CandidateInfo> results;
  self->flush(results);

  if (results.empty()) {
    CandidateInfo cand;
    cand.valid = false;
    cand.failure_reason = "CUDA batch solve returned no results";
    return cand;
  }

  CandidateInfo cand = results[0];
  cand.cost = continuityCost(cand.q, q_prev, dq_prev);
  cand.joint_cost = cand.cost;
  return cand;
}

// ============================================================================
// clear() — Reset pending queue
// ============================================================================

void CudaBatchIK::clear() {
  pending_.clear();
  h_targets_.clear();
  h_seeds_.clear();
}

// ============================================================================
// GPU Collision Detection Implementation
// ============================================================================

void CudaBatchIK::ensureCollisionCapacity(int N_frames, int N_boxes, int N_objects) {
  size_t robot_boxes_needed = static_cast<size_t>(N_frames) * N_boxes * 6;
  if (!d_robot_boxes_ || d_robot_boxes_->size() < robot_boxes_needed) {
    d_robot_boxes_ = std::make_unique<cuda::DeviceBuffer<double>>(robot_boxes_needed);
  }
  if (!d_clearances_ || d_clearances_->size() < static_cast<size_t>(N_frames)) {
    d_clearances_ = std::make_unique<cuda::DeviceBuffer<double>>(N_frames);
  }
  if (!d_colliding_ || d_colliding_->size() < static_cast<size_t>(N_frames)) {
    d_colliding_ = std::make_unique<cuda::DeviceBuffer<double>>(N_frames);
  }
}

void CudaBatchIK::uploadEnvironment(const std::vector<double>& env_objects) {
  N_env_objects_ = static_cast<int>(env_objects.size()) / 8;
  if (N_env_objects_ == 0) {
    // No objects to check — clear the buffer to avoid stale data
    d_env_objects_.reset();
    return;
  }

  size_t needed = static_cast<size_t>(N_env_objects_) * 8;
  if (!d_env_objects_ || d_env_objects_->size() < needed) {
    d_env_objects_ = std::make_unique<cuda::DeviceBuffer<double>>(needed);
  }
  // Copy non-const data via const_cast (toDevice expects non-const for memcpy src)
  std::vector<double> env_copy = env_objects;
  d_env_objects_->toDevice(env_copy.data());
}

void CudaBatchIK::updateRobotBoxesGPU(const std::vector<double>& robot_boxes,
                                       int N_frames, int N_boxes) {
  ensureCollisionCapacity(N_frames, N_boxes, N_env_objects_ > 0 ? N_env_objects_ : 1);

  size_t expected = static_cast<size_t>(N_frames) * N_boxes * 6;
  if (robot_boxes.size() < expected) {
    throw std::runtime_error(
        "updateRobotBoxesGPU: robot_boxes size " + std::to_string(robot_boxes.size()) +
        " < expected " + std::to_string(expected));
  }
  std::vector<double> boxes_copy = robot_boxes;
  d_robot_boxes_->toDevice(boxes_copy.data());
}

std::pair<std::vector<double>, std::vector<double>>
CudaBatchIK::checkCollisionsGPU(int N_frames, int N_boxes) {
  std::vector<double> clearances(N_frames, std::numeric_limits<double>::infinity());
  std::vector<double> colliding(N_frames, 0.0);

  if (N_env_objects_ <= 0 || !d_env_objects_) {
    return {clearances, colliding};  // No environment → no collisions
  }

  if (!d_robot_boxes_) {
    return {clearances, colliding};  // No robot boxes uploaded
  }

  // Launch GPU collision kernel
  cudaError_t err = cuda::launch_collision_check_batch(
      d_robot_boxes_->get(), d_env_objects_->get(),
      d_clearances_->get(), d_colliding_->get(),
      N_frames, N_boxes, N_env_objects_,
      stream_);

  if (err != cudaSuccess) {
    throw std::runtime_error(
        "checkCollisionsGPU: kernel launch failed: " +
        std::string(cudaGetErrorString(err)));
  }

  cudaStreamSynchronize(stream_);

  // Read back results
  d_clearances_->toHost(clearances.data());
  d_colliding_->toHost(colliding.data());

  return {clearances, colliding};
}

}  // namespace rtfg
