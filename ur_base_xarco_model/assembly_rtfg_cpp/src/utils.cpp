#include "utils.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rtfg {

// --- math utilities ---

void fail(const std::string& id, const std::string& msg) {
  throw std::runtime_error(id + ": " + msg);
}

double clamp01(double x) {
  return std::max(0.0, std::min(1.0, x));
}

double rad2deg(double x) {
  return x * 180.0 / M_PI;
}

Mat4 rtToTform(const Eigen::Matrix3d& R, const Vec3& p) {
  Mat4 T = Mat4::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  return T;
}

Eigen::Matrix3d rpyToRot(double r, double p, double y) {
  Eigen::AngleAxisd rx(r, Vec3::UnitX());
  Eigen::AngleAxisd ry(p, Vec3::UnitY());
  Eigen::AngleAxisd rz(y, Vec3::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

Mat4 xyzrpyToTform(const Vec3& xyz, const Vec3& rpy) {
  return rtToTform(rpyToRot(rpy.x(), rpy.y(), rpy.z()), xyz);
}

Vec3 rotToLogVec(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  if (std::isnan(aa.angle()) || aa.axis().hasNaN()) {
    return Vec3::Zero();
  }
  double angle = aa.angle();
  if (angle < 1e-12) {
    return Vec3::Zero();
  }
  return aa.axis() * angle;
}

double rotationDistance(const Eigen::Matrix3d& R0, const Eigen::Matrix3d& R1) {
  return rotToLogVec(R0.transpose() * R1).norm();
}

double quinticBlend(double t) {
  t = clamp01(t);
  return 10.0 * t * t * t - 15.0 * t * t * t * t + 6.0 * t * t * t * t * t;
}

Eigen::Matrix3d slerpRotation(const Eigen::Matrix3d& R0, const Eigen::Matrix3d& R1, double a) {
  Eigen::Quaterniond q0(R0);
  Eigen::Quaterniond q1(R1);
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }
  return q0.slerp(clamp01(a), q1).toRotationMatrix();
}

Eigen::VectorXd wrapJointDelta(const Eigen::VectorXd& dq) {
  Eigen::VectorXd out = dq;
  for (int i = 0; i < out.size(); ++i) {
    out[i] = std::atan2(std::sin(out[i]), std::cos(out[i]));
  }
  return out;
}

double continuityCost(const Eigen::VectorXd& q_candidate, const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev) {
  Eigen::VectorXd dq_pos = wrapJointDelta(q_candidate - q_prev);
  Eigen::VectorXd dq_vel = dq_pos - dq_prev;
  return dq_pos.norm() + 0.65 * dq_vel.norm();
}

Vec6 poseError(const Mat4& current, const Mat4& target) {
  Vec6 err = Vec6::Zero();
  err.head<3>() = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
  Eigen::Matrix3d Rerr = current.block<3, 3>(0, 0).transpose() * target.block<3, 3>(0, 0);
  Vec3 w_local = rotToLogVec(Rerr);
  err.tail<3>() = current.block<3, 3>(0, 0) * w_local;
  return err;
}

}  // namespace rtfg
