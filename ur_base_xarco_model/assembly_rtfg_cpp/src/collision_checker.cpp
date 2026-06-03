#include "collision_checker.h"
#include "robot_model.h"

#include <fcl/fcl.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rtfg {
namespace {

bool isMountingBase(const std::string& link_name) {
  return link_name == "base_jizuo" ||
         link_name == "base_jizuo_base_ur10_with_dizuo" ||
         link_name == "ur10";
}

}  // namespace

CollisionSummary evaluateConfiguration(const RobotModel& robot,
                                       const std::vector<BasinBox>& basin_boxes,
                                       const SolverConfig& cfg, const Eigen::VectorXd& q) {
  CollisionSummary summary;
  auto poses = forwardKinematics(robot, q);

  // Reusable FCL collision objects — shared_ptr geometry is reused,
  // only the transform is updated each call (avoids expensive BVH rebuild).
  thread_local std::vector<std::unique_ptr<CollisionObjectd>> robot_objects;
  thread_local const RobotModel* cached_robot = nullptr;

  if (!cached_robot || cached_robot != &robot) {
    robot_objects.clear();
    robot_objects.reserve(robot.collisions.size());
    for (const auto& col : robot.collisions) {
      Mat4 T = poses.at(col.link_name) * col.origin;
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.matrix() = T;
      auto obj = std::make_unique<CollisionObjectd>(col.geometry, tf);
      robot_objects.push_back(std::move(obj));
    }
    cached_robot = &robot;
  } else {
    // Update transforms only — geometry is shared, no BVH rebuild needed
    for (size_t i = 0; i < robot.collisions.size(); ++i) {
      Mat4 T = poses.at(robot.collisions[i].link_name) * robot.collisions[i].origin;
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.matrix() = T;
      robot_objects[i]->setTransform(tf);
    }
  }

  // Compute AABBs (needed after transform update for broadphase culling)
  for (auto& obj : robot_objects) {
    obj->computeAABB();
  }

  auto choose_violation = [&](const std::string& type, const std::string& name,
                               double clearance) {
    static const std::map<std::string, int> pri = {
        {"tool_basin", 3}, {"tool_body", 2}, {"self", 1}};
    if (summary.violation_type.empty() ||
        pri.at(type) > pri.at(summary.violation_type)) {
      summary.violation_type = type;
      summary.violation_object = name;
      summary.violation_clearance = clearance;
    }
  };

  // Self-collision and tool-body
  for (size_t i = 0; i < robot.collisions.size(); ++i) {
    for (size_t j = i + 1; j < robot.collisions.size(); ++j) {
      const auto& a = robot.collisions[i];
      const auto& b = robot.collisions[j];
      if (std::abs(a.chain_index - b.chain_index) <= 1) continue;
      if (isMountingBase(a.link_name) && isMountingBase(b.link_name)) continue;

      fcl::DistanceRequestd request(true);
      fcl::DistanceResultd result;
      double dist =
          fcl::distance(robot_objects[i].get(), robot_objects[j].get(), request, result);
      bool a_tool = a.is_tool;
      bool b_tool = b.is_tool;
      std::string pair_name = a.link_name + " <-> " + b.link_name;

      if (a_tool ^ b_tool) {
        if (dist < summary.min_tool_body) {
          summary.min_tool_body = dist;
          summary.min_tool_body_object = pair_name;
        }
        if (dist < cfg.clearance_threshold) {
          choose_violation("tool_body", pair_name, dist);
        }
      } else if (!a_tool && !b_tool) {
        if (dist < summary.min_self) {
          summary.min_self = dist;
          summary.min_self_object = pair_name;
        }
        if (dist < cfg.clearance_threshold) {
          choose_violation("self", pair_name, dist);
        }
      }
    }
  }

  // Tool-basin collisions
  std::vector<int> tool_indices;
  for (size_t i = 0; i < robot.collisions.size(); ++i) {
    if (robot.collisions[i].is_tool) tool_indices.push_back(static_cast<int>(i));
  }

  thread_local std::vector<std::unique_ptr<CollisionObjectd>> basin_objects;
  thread_local const std::vector<BasinBox>* cached_basin_boxes = nullptr;
  if (cached_basin_boxes != &basin_boxes) {
    basin_objects.clear();
    basin_objects.reserve(basin_boxes.size());
    for (const auto& box : basin_boxes) {
      auto geom = std::make_shared<fcl::Boxd>(box.size.x(), box.size.y(), box.size.z());
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.matrix() = box.pose;
      auto obj = std::make_unique<CollisionObjectd>(geom, tf);
      obj->computeAABB();
      basin_objects.push_back(std::move(obj));
    }
    cached_basin_boxes = &basin_boxes;
  }

  for (int tool_idx : tool_indices) {
    for (size_t basin_idx = 0; basin_idx < basin_boxes.size(); ++basin_idx) {
      fcl::DistanceRequestd request(true);
      fcl::DistanceResultd result;
      double dist =
          fcl::distance(robot_objects[tool_idx].get(), basin_objects[basin_idx].get(), request, result);
      std::string pair_name = basin_boxes[basin_idx].name;
      if (dist < summary.min_tool_basin) {
        summary.min_tool_basin = dist;
        summary.min_tool_basin_object = pair_name;
      }
      if (dist < cfg.clearance_threshold) {
        choose_violation("tool_basin", pair_name, dist);
      }
    }
  }

  return summary;
}

}  // namespace rtfg
