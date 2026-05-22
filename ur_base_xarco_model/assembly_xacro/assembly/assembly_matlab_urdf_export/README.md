# MATLAB URDF Export

This folder is a self-contained URDF export generated from:

`/root/ur10_ws/src/ur_base_xarco_model/assembly_xacro/assembly/assembly.urdf.xacro`

Contents:

- `assembly.urdf`: expanded URDF for MATLAB import.
- `meshes/`: all referenced STL/DAE mesh files.
- `textures/`: copied texture folder from the source directory. It is currently empty.

Export notes:

- Mesh paths are relative, for example `meshes/base.dae`.
- ROS package paths, xacro macros, Gazebo plugins, ros2_control blocks, and non-standard `quat_xyzw` attributes were excluded from the exported URDF.
- The original `assembly.urdf.xacro` file was not modified.
