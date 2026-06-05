#pragma once
// cuda_collision.h — GPU collision detection interface for UR10 assembly task
//
// Provides analytical clearance computation for simple geometry pairs (box-box,
// box-sphere, box-cylinder) on GPU, complementing CPU FCL for complex meshes.
//
// Data layout:
//   Robot boxes:  [N_frames × N_boxes × 6] where each box = (cx, cy, cz, hx, hy, hz)
//   Env objects:   [N_objects × 8]          where each object = (type, p1..p6, _)
//     type 0 = box     → p1..p6 = (cx, cy, cz, hx, hy, hz)
//     type 1 = sphere   → p1..p4 = (cx, cy, cz, r), p5..p6 = unused
//     type 2 = cylinder → p1..p5 = (cx, cy, cz, r, h), p6 = unused
//   Clearances:    [N_frames]               min clearance per frame (∞ if none)
//   Colliding:     [N_frames]               1.0 if any pair penetrates, 0.0 otherwise

#include <cuda_runtime.h>

namespace rtfg {
namespace cuda {

// Launch the collision check batch kernel.
// N_boxes = number of robot link bounding boxes (typically 6 for UR10 + 2 for shovel = 8)
// N_objects = number of environment objects to check against
// stream = CUDA stream for async overlap with IK computation
cudaError_t launch_collision_check_batch(
    const double* d_robot_boxes,
    const double* d_env_objects,
    double* d_clearances,
    double* d_colliding,
    int N_frames, int N_boxes, int N_objects,
    cudaStream_t stream = 0);

}  // namespace cuda
}  // namespace rtfg
