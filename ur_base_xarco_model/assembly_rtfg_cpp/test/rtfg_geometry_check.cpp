#include "trajectory_generator.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool close(double a, double b, double tol)
{
  return std::abs(a - b) <= tol;
}

}  // namespace

int main()
{
  const auto share = ament_index_cpp::get_package_share_directory("assembly_rtfg_cpp");
  const auto cfg = rtfg::loadRuntimeConfig(
    share + "/config/environment_runtime_config.yaml",
    share + "/urdf/assembly_rtfg_solver.urdf");
  const auto plan = rtfg::buildTargetPlan(cfg.trajectory, cfg.pose);

  bool ok = true;
  ok = ok && close(plan.approach_start.x(), -1.824688, 0.002);
  ok = ok && close(plan.approach_start.y(), 0.255095, 0.002);
  ok = ok && close(plan.approach_start.z(), 0.475450, 0.002);
  ok = ok && close(plan.entry.x(), -1.824688, 0.002);
  ok = ok && close(plan.entry.y(), 0.051666, 0.002);
  ok = ok && close(plan.entry.z(), 0.358000, 0.002);
  ok = ok && close(plan.arc_end.x(), -1.824688, 0.002);
  ok = ok && close(plan.arc_end.y(), -0.142848, 0.002);
  ok = ok && close(plan.arc_end.z(), 0.305880, 0.002);
  ok = ok && close(plan.arc_radius, 0.389029, 0.002);
  ok = ok && close(plan.vertical_penetration, 0.052120, 0.002);

  std::cout << "target_poses=" << plan.tforms.size()
            << " approach_start=" << plan.approach_start.transpose()
            << " entry=" << plan.entry.transpose()
            << " arc_end=" << plan.arc_end.transpose()
            << " radius=" << plan.arc_radius
            << " vertical_penetration=" << plan.vertical_penetration
            << std::endl;

  return ok ? 0 : 1;
}
