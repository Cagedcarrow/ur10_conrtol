#pragma once

#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <kdl/chain.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rtfg {

using Mat4 = Eigen::Matrix4d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using CollisionObjectd = fcl::CollisionObjectd;
using CollisionGeometryd = fcl::CollisionGeometryd;

struct BasinBox {
  std::string name;
  Vec3 size;
  Mat4 pose;
};

struct ProgressEvent {
  std::string phase;
  double progress = 0.0;
  std::string message;
};

struct SegmentSpec {
  std::string joint_name;
  std::string parent_link;
  std::string child_link;
  Mat4 origin = Mat4::Identity();
  Vec3 axis = Vec3::UnitZ();
  bool movable = false;
  int q_index = -1;
  double lower = -M_PI;
  double upper = M_PI;
};

enum class GeometryType { Box, Cylinder, Mesh, Unknown };

struct LinkCollision {
  std::string link_name;
  int chain_index = -1;
  bool is_tool = false;
  Mat4 origin = Mat4::Identity();
  GeometryType type = GeometryType::Unknown;
  Vec3 size = Vec3::Zero();
  double radius = 0.0;
  double length = 0.0;
  std::shared_ptr<CollisionGeometryd> geometry;
};

struct RobotModel {
  std::vector<SegmentSpec> segments;
  std::vector<LinkCollision> collisions;
  std::vector<std::string> link_order;
  std::unordered_map<std::string, int> link_index;
  std::vector<std::string> tool_links;
  std::string base_link;
  std::string tip_link;
  std::string mesh_root;
  std::shared_ptr<KDL::Chain> kdl_chain;     // KDL chain base -> wrist_3 (6 DOF)
  Mat4 T_wrist3_to_tcp = Mat4::Identity();    // fixed tool offset wrist_3 -> TCP
};

struct CandidateInfo {
  Eigen::VectorXd q;
  double pos_err = std::numeric_limits<double>::infinity();
  double rot_err = std::numeric_limits<double>::infinity();
  double clearance = -std::numeric_limits<double>::infinity();
  double cost = std::numeric_limits<double>::infinity();
  double joint_cost = std::numeric_limits<double>::infinity();
  int iterations_used = 0;
  std::string failure_reason;
  bool valid = false;
};

struct CollisionSummary {
  double min_self = std::numeric_limits<double>::infinity();
  double min_tool_body = std::numeric_limits<double>::infinity();
  double min_tool_basin = std::numeric_limits<double>::infinity();
  std::string min_self_object;
  std::string min_tool_body_object;
  std::string min_tool_basin_object;
  std::string violation_type;
  std::string violation_object;
  double violation_clearance = std::numeric_limits<double>::infinity();
};

struct SolverConfig {
  std::string solver_mode = "full";
  std::string solver_backend = "numeric";
  double clearance_threshold = 2e-3;
  double ik_position_tolerance = 3e-2;
  int max_iterations = 60;
  int max_iterations_full = 60;
  int max_iterations_realtime = 30;
  double lambda = 5e-3;
  double stagnation_epsilon = 1e-5;
  int stagnation_patience = 3;
  double dq_stop_threshold = 1e-6;
  int max_collision_candidates_full = 12;
  int max_collision_candidates_realtime = 2;
  int collision_check_stride_full = 1;
  int collision_check_stride_realtime = 5;
  bool publish_sparse_posearray_realtime = true;
  int posearray_stride_realtime = 10;
  bool enable_window_solve = false;
  int window_size = 100;
  int window_overlap = 10;
};

struct SolverTiming {
  std::string solver_mode;
  int points_requested = 0;
  int points_solved = 0;
  double total_wall_s = 0.0;
  double ik_total_s = 0.0;
  double collision_total_s = 0.0;
  double playback_generation_s = 0.0;
  double posearray_pack_s = 0.0;
  double joint_trajectory_pack_s = 0.0;
  int n_poses_solved = 0;
  int n_kdl_success = 0;
  int n_custom_success = 0;
  int n_failed = 0;
  int original_candidates = 0;
  int candidates_filtered_by_error = 0;
  int candidate_quick_rejects = 0;
  int candidates_entered_fcl = 0;
  int candidates_truncated = 0;
  int fcl_calls = 0;
  int fcl_skipped_points = 0;
  int fcl_forced_points = 0;
  int continuous_prediction_hits = 0;
  int continuous_prediction_fallbacks = 0;
  int posearray_points = 0;
  int joint_trajectory_points = 0;
  int ik_iterations_total = 0;
  int ik_iterations_max = 0;
  double avg_ik_iterations = 0.0;
  double min_per_pose_s = std::numeric_limits<double>::infinity();
  double max_per_pose_s = 0.0;
  double avg_per_pose_s = 0.0;
};

struct TrajectoryResult {
  Eigen::MatrixXd anchor_q;
  Eigen::MatrixXd playback_q;
  Eigen::MatrixXd tcp_path;
  std::vector<std::string> playback_segment_names;
  std::vector<ProgressEvent> progress_events;
  CollisionSummary global_minimums;
  std::vector<int> collision_traj_indices;
  std::vector<int> collision_sample_indices;
  std::vector<std::string> collision_types;
  std::vector<std::string> collision_objects;
  std::vector<std::string> collision_segments;
  std::vector<uint8_t> collision_checked_flags;
  Eigen::MatrixXd collision_points;
  bool has_collision = false;
  int first_collision_traj_idx = -1;
  int first_collision_sample_idx = -1;
  std::string first_collision_type;
  std::string first_collision_object;
  std::string first_collision_segment;
  Vec3 first_collision_point = Vec3::Zero();
  double first_collision_clearance = std::numeric_limits<double>::infinity();
  double max_target_rot_deg = 0.0;
  double max_actual_rot_deg = 0.0;
  double max_anchor_qstep_deg = 0.0;
  double max_playback_qstep_deg = 0.0;
  SolverTiming timing;
};

}  // namespace rtfg
