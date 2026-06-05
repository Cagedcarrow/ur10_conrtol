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

// Launch the collision check batch kernel (original AABB-based).
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

// ============================================================================
// Two-phase OBB+GJK collision detection (Optimization C)
//
// Phase 1 (OBB Broad Phase): SAT-based OBB-OBB intersection test.
//   Each robot link is represented as an OBB built from its FK 4×4 transform.
//   Environment objects (box/sphere/cylinder) are also converted to OBB.
//   SAT with 15 separating axes provides tight rejection (~95% of pairs).
//
// Phase 2 (GJK Narrow Phase): GJK intersection test for surviving pairs.
//   Uses Minkowski difference support functions and Johnson's distance
//   subalgorithm. Converges in 2-6 iterations for typical robot links.
//
// Expected: ~0.35 ms for 1,365 pairs vs ~680 ms CPU FCL (~1,900× speedup).
//
// Parameters:
//   d_link_transforms: [N_frames × N_links × 16] FK 4×4 transforms (row-major)
//   d_link_params:     [N_links × 2] (length, radius) per link in meters
//   d_env_objects:     [N_objects × 8] environment objects (same format as batch)
//   d_clearances:      [N_frames] output min clearance (INF if no collision)
//   d_colliding:       [N_frames] output 1.0 if any pair collides
// ============================================================================
cudaError_t launch_collision_check_obb_gjk(
    const double* d_link_transforms,
    const double* d_link_params,
    const double* d_env_objects,
    double* d_clearances,
    double* d_colliding,
    int N_frames, int N_links, int N_objects,
    cudaStream_t stream = 0);

}  // namespace cuda
}  // namespace rtfg
