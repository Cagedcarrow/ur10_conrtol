// cuda_collision.cu — GPU-accelerated collision detection for UR10 assembly task
//
// Implements box-box, box-sphere, and box-cylinder clearance computation on GPU.
// Designed to complement (not replace) CPU FCL for complex mesh-mesh collisions.
//
// Architecture:
//   Grid:  (N_frames, 1, 1) — one block per trajectory playback frame
//   Block: (N_pairs, 1, 1)  — one thread per collision pair
//   Each thread computes minimum clearance for one (robot_box, env_object) pair.
//
// The GPU collision path handles simple geometric pairs where analytical
// distance formulas are available. Complex mesh collisions remain on CPU via FCL.
// This hybrid approach gives the best of both: GPU parallelism for many simple
// pairs + FCL fidelity for complex geometry.

#include "cuda_collision.h"
#include "cuda_utilities.cuh"
#include <cstdio>

namespace rtfg {
namespace cuda {

// ============================================================================
// Helper: atomic minimum for double (CUDA doesn't have atomicMin for double)
// Uses atomicCAS loop — safe for infrequent updates (one per frame).
// ============================================================================
__device__ __forceinline__ void atomicMin_double(double* addr, double val) {
  unsigned long long* addr_as_ull = reinterpret_cast<unsigned long long*>(addr);
  unsigned long long old = *addr_as_ull;
  unsigned long long assumed;
  do {
    assumed = old;
    double old_val = __longlong_as_double(assumed);
    if (val >= old_val) break;
    old = atomicCAS(addr_as_ull, assumed, __double_as_longlong(val));
  } while (assumed != old);
}

// ============================================================================
// Collision primitives: analytical distance formulas for simple geometry pairs
// ============================================================================

// Box-box clearance (axis-aligned bounding boxes).
// Each box defined by (cx, cy, cz, hx, hy, hz): center + half-extents.
// Returns clearance > 0 if separated, < 0 if penetrating.
__device__ __forceinline__ double box_box_clearance(
    double bx, double by, double bz, double bhx, double bhy, double bhz,
    double ox, double oy, double oz, double ohx, double ohy, double ohz) {
  double dx = fabs(bx - ox) - (bhx + ohx);
  double dy = fabs(by - oy) - (bhy + ohy);
  double dz = fabs(bz - oz) - (bhz + ohz);
  return fmax(dx, fmax(dy, dz));
}

// Box-sphere clearance.
// Sphere defined by (sx, sy, sz, sr): center + radius.
__device__ __forceinline__ double box_sphere_clearance(
    double bx, double by, double bz, double bhx, double bhy, double bhz,
    double sx, double sy, double sz, double sr) {
  // Closest point on box to sphere center
  double cx = cuda_clamp(sx, bx - bhx, bx + bhx);
  double cy = cuda_clamp(sy, by - bhy, by + bhy);
  double cz = cuda_clamp(sz, bz - bhz, bz + bhz);
  // Distance from closest point to sphere center
  double dx = sx - cx, dy = sy - cy, dz = sz - cz;
  return sqrt(dx*dx + dy*dy + dz*dz) - sr;
}

// Box-cylinder clearance (cylinder axis assumed vertical/Z-aligned).
// Cylinder defined by (cx, cy, cz, cr, ch): base center, radius, half-height.
__device__ __forceinline__ double box_cylinder_clearance(
    double bx, double by, double bz, double bhx, double bhy, double bhz,
    double cx, double cy, double cz, double cr, double ch) {
  // 1. Horizontal (XY) clearance: box vs infinite cylinder
  // Closest point on box (in XY plane) to cylinder axis
  double closest_x = cuda_clamp(cx, bx - bhx, bx + bhx);
  double closest_y = cuda_clamp(cy, by - bhy, by + bhy);
  double h_dist = sqrt((closest_x - cx)*(closest_x - cx) +
                       (closest_y - cy)*(closest_y - cy)) - cr;

  // 2. Vertical (Z) clearance
  double z_dist = fabs(bz - cz) - (bhz + ch);

  // Combined: if horizontal overlap, vertical clearance matters; vice versa
  if (h_dist < 0 && z_dist < 0) {
    // Penetrating in both dimensions — return max penetration
    return fmax(h_dist, z_dist);
  }
  if (h_dist < 0) return z_dist;    // Horizontally overlapping, check vertical
  if (z_dist < 0) return h_dist;    // Vertically overlapping, check horizontal
  // Separated in both: return 3D distance
  return sqrt(h_dist*h_dist + z_dist*z_dist);
}

// ============================================================================
// KERNEL: collision_check_batch
//
// Computes minimum clearance for each trajectory frame against all environment
// objects. Results stored per-frame via atomic minimum.
//
// Grid:  (N_frames, 1, 1)
// Block: (N_pairs_per_frame, 1, 1)   where N_pairs = N_boxes × N_objects
// ============================================================================
__global__ void collision_check_batch(
    const double* __restrict__ d_robot_boxes,   // [N_frames, N_boxes, 6] (xyz + half-extents)
    const double* __restrict__ d_env_objects,    // [N_objects, 8] (type + params)
    double* __restrict__ d_clearances,           // [N_frames] output min clearance
    double* __restrict__ d_colliding,            // [N_frames] output: 1.0 if collision
    int N_frames, int N_boxes, int N_objects
) {
  int frame = blockIdx.x;
  int tid   = threadIdx.x;
  int total_pairs = N_boxes * N_objects;

  if (frame >= N_frames) return;

  // Initialize per-frame clearance to large value (lane 0)
  if (tid == 0) {
    d_clearances[frame] = 1e10;
    d_colliding[frame] = 0.0;
  }
  __syncthreads();

  // Each thread handles one (box, object) pair, striding through all pairs
  for (int pair = tid; pair < total_pairs; pair += blockDim.x) {
    int box_idx = pair / N_objects;
    int obj_idx = pair % N_objects;

    // Load robot box
    int box_offset = (frame * N_boxes + box_idx) * 6;
    double bx  = d_robot_boxes[box_offset + 0];
    double by  = d_robot_boxes[box_offset + 1];
    double bz  = d_robot_boxes[box_offset + 2];
    double bhx = d_robot_boxes[box_offset + 3];
    double bhy = d_robot_boxes[box_offset + 4];
    double bhz = d_robot_boxes[box_offset + 5];

    // Load environment object
    int obj_offset = obj_idx * 8;
    double obj_type = d_env_objects[obj_offset + 0];

    double clearance = 1e10;  // Default: no collision for invalid types

    if (obj_type == 0.0) {
      // Box-box
      double ox  = d_env_objects[obj_offset + 1];
      double oy  = d_env_objects[obj_offset + 2];
      double oz  = d_env_objects[obj_offset + 3];
      double ohx = d_env_objects[obj_offset + 4];
      double ohy = d_env_objects[obj_offset + 5];
      double ohz = d_env_objects[obj_offset + 6];
      clearance = box_box_clearance(bx, by, bz, bhx, bhy, bhz,
                                    ox, oy, oz, ohx, ohy, ohz);
    } else if (obj_type == 1.0) {
      // Box-sphere
      double sx = d_env_objects[obj_offset + 1];
      double sy = d_env_objects[obj_offset + 2];
      double sz = d_env_objects[obj_offset + 3];
      double sr = d_env_objects[obj_offset + 4];
      clearance = box_sphere_clearance(bx, by, bz, bhx, bhy, bhz,
                                       sx, sy, sz, sr);
    } else if (obj_type == 2.0) {
      // Box-cylinder (vertical axis)
      double cx = d_env_objects[obj_offset + 1];
      double cy = d_env_objects[obj_offset + 2];
      double cz = d_env_objects[obj_offset + 3];
      double cr = d_env_objects[obj_offset + 4];
      double ch = d_env_objects[obj_offset + 5];
      clearance = box_cylinder_clearance(bx, by, bz, bhx, bhy, bhz,
                                          cx, cy, cz, cr, ch);
    }

    // Atomic min for per-frame minimum clearance
    atomicMin_double(&d_clearances[frame], clearance);

    // Mark collision if any pair penetrates
    if (clearance < 0.0) {
      d_colliding[frame] = 1.0;
    }
  }
}

// ============================================================================
// Host wrapper: launch_collision_check_batch
// ============================================================================
cudaError_t launch_collision_check_batch(
    const double* d_robot_boxes, const double* d_env_objects,
    double* d_clearances, double* d_colliding,
    int N_frames, int N_boxes, int N_objects,
    cudaStream_t stream)
{
  // Max 256 threads per block; each thread handles multiple pairs via striding
  int total_pairs = N_boxes * N_objects;
  int block_size = (total_pairs < 256) ? ((total_pairs + 31) / 32 * 32) : 256;
  if (block_size < 32) block_size = 32;

  dim3 grid(N_frames, 1, 1);
  dim3 block(block_size, 1, 1);

  collision_check_batch<<<grid, block, 0, stream>>>(
      d_robot_boxes, d_env_objects, d_clearances, d_colliding,
      N_frames, N_boxes, N_objects);

  return cudaGetLastError();
}

}  // namespace cuda
}  // namespace rtfg
