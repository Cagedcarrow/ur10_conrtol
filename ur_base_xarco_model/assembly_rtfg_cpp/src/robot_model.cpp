#include "robot_model.h"
#include "utils.h"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fcl/fcl.h>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

std::shared_ptr<rtfg::CollisionGeometryd> loadStlAsFclModel(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("Failed to open STL: " + path);
  }
  std::vector<rtfg::Vec3> vertices;
  std::vector<fcl::Triangle> triangles;
  char header[80] = {};
  in.read(header, 80);
  std::uint32_t tri_count = 0;
  in.read(reinterpret_cast<char*>(&tri_count), sizeof(tri_count));
  bool binary_ok = in.good();
  if (binary_ok) {
    in.seekg(0, std::ios::end);
    std::streamoff file_size = in.tellg();
    std::streamoff expected = 84 + static_cast<std::streamoff>(tri_count) * 50;
    binary_ok = (file_size == expected);
  }
  if (binary_ok) {
    in.seekg(84, std::ios::beg);
    vertices.reserve(static_cast<size_t>(tri_count) * 3);
    triangles.reserve(static_cast<size_t>(tri_count));
    for (std::uint32_t i = 0; i < tri_count; ++i) {
      float buf[12];
      std::uint16_t attr = 0;
      in.read(reinterpret_cast<char*>(buf), sizeof(buf));
      in.read(reinterpret_cast<char*>(&attr), sizeof(attr));
      if (!in.good()) throw std::runtime_error("Corrupt binary STL: " + path);
      rtfg::Vec3 v0(buf[3], buf[4], buf[5]);
      rtfg::Vec3 v1(buf[6], buf[7], buf[8]);
      rtfg::Vec3 v2(buf[9], buf[10], buf[11]);
      int base = static_cast<int>(vertices.size());
      vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v2);
      triangles.emplace_back(base, base + 1, base + 2);
    }
  } else {
    in.close();
    std::ifstream txt(path);
    if (!txt) throw std::runtime_error("Failed to reopen STL as text: " + path);
    std::string line;
    std::vector<rtfg::Vec3> facet_vertices;
    while (std::getline(txt, line)) {
      std::istringstream iss(line);
      std::string tok;
      iss >> tok;
      if (tok == "vertex") {
        double x = 0, y = 0, z = 0;
        iss >> x >> y >> z;
        facet_vertices.emplace_back(x, y, z);
        if (facet_vertices.size() == 3) {
          int base = static_cast<int>(vertices.size());
          vertices.push_back(facet_vertices[0]);
          vertices.push_back(facet_vertices[1]);
          vertices.push_back(facet_vertices[2]);
          triangles.emplace_back(base, base + 1, base + 2);
          facet_vertices.clear();
        }
      }
    }
  }
  using BVH = fcl::BVHModel<fcl::OBBRSSd>;
  auto bvh = std::make_shared<BVH>();
  bvh->beginModel();
  bvh->addSubModel(vertices, triangles);
  bvh->endModel();
  return bvh;
}

std::shared_ptr<rtfg::CollisionGeometryd> buildGeomFromUrdf(
    const urdf::CollisionSharedPtr& collision, const std::string& mesh_root) {
  if (!collision || !collision->geometry) return nullptr;
  switch (collision->geometry->type) {
    case urdf::Geometry::BOX: {
      auto* box = dynamic_cast<urdf::Box*>(collision->geometry.get());
      return std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
    }
    case urdf::Geometry::CYLINDER: {
      auto* cyl = dynamic_cast<urdf::Cylinder*>(collision->geometry.get());
      return std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
    }
    case urdf::Geometry::MESH: {
      auto* mesh = dynamic_cast<urdf::Mesh*>(collision->geometry.get());
      std::string filename = mesh->filename;
      if (filename.rfind("package://", 0) == 0) {
        throw std::runtime_error("package:// mesh paths not supported");
      }
      std::string full_path = filename;
      if (!filename.empty() && filename[0] != '/') {
        full_path = mesh_root + "/" + filename;
      }
      return loadStlAsFclModel(full_path);
    }
    default:
      return nullptr;
  }
}

}  // namespace

namespace rtfg {

RobotModel loadRobotModel(const std::string& urdf_path, const std::string& base_link,
                          const std::string& tip_link) {
  auto model = urdf::parseURDFFile(urdf_path);
  if (!model) {
    throw std::runtime_error("Failed to parse URDF: " + urdf_path);
  }

  RobotModel robot;
  robot.base_link = base_link;
  robot.tip_link = tip_link;
  std::string mesh_root = urdf_path.substr(0, urdf_path.find_last_of('/'));
  robot.mesh_root = mesh_root;

  std::unordered_map<std::string, urdf::JointSharedPtr> child_joint_map;
  for (const auto& kv : model->joints_) {
    child_joint_map[kv.second->child_link_name] = kv.second;
  }

  std::vector<urdf::JointSharedPtr> reverse_joints;
  std::string current = tip_link;
  while (current != base_link) {
    auto it = child_joint_map.find(current);
    if (it == child_joint_map.end()) {
      throw std::runtime_error("Unable to trace chain from tip to base for link: " + current);
    }
    reverse_joints.push_back(it->second);
    current = it->second->parent_link_name;
  }
  std::reverse(reverse_joints.begin(), reverse_joints.end());

  robot.link_order.push_back(base_link);
  robot.link_index[base_link] = 0;
  int movable_idx = 0;
  for (size_t i = 0; i < reverse_joints.size(); ++i) {
    const auto& joint = reverse_joints[i];
    SegmentSpec seg;
    seg.joint_name = joint->name;
    seg.parent_link = joint->parent_link_name;
    seg.child_link = joint->child_link_name;
    seg.origin = urdfPoseToTform(joint->parent_to_joint_origin_transform);
    seg.axis = Vec3(joint->axis.x, joint->axis.y, joint->axis.z);
    if (seg.axis.norm() < 1e-12) {
      seg.axis = Vec3::UnitZ();
    } else {
      seg.axis.normalize();
    }
    seg.movable =
        (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS);
    if (seg.movable) {
      seg.q_index = movable_idx++;
      if (joint->type == urdf::Joint::CONTINUOUS || !joint->limits) {
        seg.lower = -2.0 * M_PI;
        seg.upper = 2.0 * M_PI;
      } else {
        seg.lower = joint->limits->lower;
        seg.upper = joint->limits->upper;
      }
    }
    robot.segments.push_back(seg);
    robot.link_index[seg.child_link] = static_cast<int>(robot.link_order.size());
    robot.link_order.push_back(seg.child_link);
  }

  if (movable_idx != 6) {
    throw std::runtime_error("Expected UR10 six revolute joints, got " +
                             std::to_string(movable_idx));
  }

  robot.tool_links = {"sensor_shovel", "sensor_shovel_tcp"};

  // Load collision geometries
  std::set<std::string> tool_names = {"sensor_shovel", "sensor_shovel_tcp"};
  for (size_t i = 0; i < robot.link_order.size(); ++i) {
    auto link = model->getLink(robot.link_order[i]);
    if (!link || !link->collision) continue;
    auto geometry = buildGeomFromUrdf(link->collision, mesh_root);
    if (!geometry) continue;
    LinkCollision lc;
    lc.link_name = link->name;
    lc.chain_index = static_cast<int>(i);
    lc.is_tool = tool_names.count(link->name) > 0;
    lc.origin = urdfPoseToTform(link->collision->origin);
    lc.geometry = geometry;
    if (auto* box = dynamic_cast<urdf::Box*>(link->collision->geometry.get())) {
      lc.type = GeometryType::Box;
      lc.size = Vec3(box->dim.x, box->dim.y, box->dim.z);
    } else if (auto* cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get())) {
      lc.type = GeometryType::Cylinder;
      lc.radius = cyl->radius;
      lc.length = cyl->length;
    } else if (dynamic_cast<urdf::Mesh*>(link->collision->geometry.get())) {
      lc.type = GeometryType::Mesh;
    }
    robot.collisions.push_back(lc);
  }

  // Build KDL chain from base_link to ur10_wrist_3 (6-DOF arm only)
  // The tool (sensor_shovel -> sensor_shovel_tcp) is a fixed offset.
  {
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*model, kdl_tree)) {
      throw std::runtime_error("KDL failed to parse URDF tree");
    }
    auto kdl_chain = std::make_shared<KDL::Chain>();
    if (!kdl_tree.getChain(base_link, "ur10_wrist_3", *kdl_chain)) {
      throw std::runtime_error("KDL cannot extract chain from " + base_link +
                               " to ur10_wrist_3");
    }
    robot.kdl_chain = kdl_chain;

    // Compute wrist_3 → TCP offset using forward kinematics at home config
    KDL::ChainFkSolverPos_recursive fk_solver(*kdl_chain);
    KDL::JntArray q_kdl(kdl_chain->getNrOfJoints());
    KDL::Frame wrist3_frame;
    fk_solver.JntToCart(q_kdl, wrist3_frame);

    Mat4 T_base_wrist3 = Mat4::Identity();
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) T_base_wrist3(r, c) = wrist3_frame.M(r, c);
    T_base_wrist3(0, 3) = wrist3_frame.p(0);
    T_base_wrist3(1, 3) = wrist3_frame.p(1);
    T_base_wrist3(2, 3) = wrist3_frame.p(2);

    // Get TCP at zero config from our own FK
    Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(6);
    Mat4 T_base_tcp = tipTransform(robot, q_zero);

    robot.T_wrist3_to_tcp = T_base_wrist3.inverse() * T_base_tcp;
  }

  return robot;
}

std::unordered_map<std::string, Mat4> forwardKinematics(const RobotModel& robot,
                                                        const Eigen::VectorXd& q) {
  std::unordered_map<std::string, Mat4> poses;
  poses[robot.base_link] = Mat4::Identity();
  Mat4 T = Mat4::Identity();
  for (const auto& seg : robot.segments) {
    T = T * seg.origin;
    if (seg.movable) {
      Eigen::AngleAxisd aa(q(seg.q_index), seg.axis);
      T = T * rtToTform(aa.toRotationMatrix(), Vec3::Zero());
    }
    poses[seg.child_link] = T;
  }
  return poses;
}

Mat4 tipTransform(const RobotModel& robot, const Eigen::VectorXd& q) {
  auto poses = forwardKinematics(robot, q);
  return poses.at(robot.tip_link);
}

Mat6 numericJacobian(const RobotModel& robot, const Eigen::VectorXd& q) {
  Mat6 J = Mat6::Zero();
  Mat4 T = Mat4::Identity();
  std::array<Vec3, 6> joint_origins{};
  std::array<Vec3, 6> joint_axes{};
  for (const auto& seg : robot.segments) {
    T = T * seg.origin;
    if (seg.movable) {
      joint_origins[seg.q_index] = T.block<3, 1>(0, 3);
      joint_axes[seg.q_index] = T.block<3, 3>(0, 0) * seg.axis;
      Eigen::AngleAxisd aa(q(seg.q_index), seg.axis);
      T = T * rtToTform(aa.toRotationMatrix(), Vec3::Zero());
    }
  }
  const Vec3 tcp = T.block<3, 1>(0, 3);
  for (int i = 0; i < 6; ++i) {
    const Vec3 axis = joint_axes[i].normalized();
    J.block<3, 1>(0, i) = axis.cross(tcp - joint_origins[i]);
    J.block<3, 1>(3, i) = axis;
  }
  return J;
}

Eigen::VectorXd clampToLimits(const RobotModel& robot, const Eigen::VectorXd& q) {
  Eigen::VectorXd qc = q;
  for (const auto& seg : robot.segments) {
    if (!seg.movable) continue;
    qc(seg.q_index) = std::min(std::max(qc(seg.q_index), seg.lower), seg.upper);
  }
  return qc;
}

Eigen::VectorXd clampToLimitsWithRebound(const RobotModel& robot, const Eigen::VectorXd& q) {
  Eigen::VectorXd qc = q;
  for (const auto& seg : robot.segments) {
    if (!seg.movable) continue;
    const double val = qc(seg.q_index);
    const double range = seg.upper - seg.lower;
    if (val < seg.lower) {
      qc(seg.q_index) = seg.lower + 0.10 * std::min(range, seg.lower - val);
    } else if (val > seg.upper) {
      qc(seg.q_index) = seg.upper - 0.10 * std::min(range, val - seg.upper);
    }
  }
  return qc;
}

Eigen::VectorXd alignToReference(const RobotModel& robot, const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& q_ref) {
  Eigen::VectorXd q_aligned = q;
  for (const auto& seg : robot.segments) {
    if (!seg.movable) continue;
    int idx = seg.q_index;
    double best = q_aligned(idx);
    double best_dist = std::abs(best - q_ref(idx));
    for (int k = -2; k <= 2; ++k) {
      double candidate = q_aligned(idx) + 2.0 * M_PI * k;
      if (candidate < seg.lower - 1e-9 || candidate > seg.upper + 1e-9) continue;
      double dist = std::abs(candidate - q_ref(idx));
      if (dist < best_dist) {
        best_dist = dist;
        best = candidate;
      }
    }
    q_aligned(idx) = std::min(std::max(best, seg.lower), seg.upper);
  }
  return q_aligned;
}

Mat4 urdfPoseToTform(const urdf::Pose& pose) {
  Vec3 xyz(pose.position.x, pose.position.y, pose.position.z);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  pose.rotation.getRPY(roll, pitch, yaw);
  return xyzrpyToTform(xyz, Vec3(roll, pitch, yaw));
}

}  // namespace rtfg
