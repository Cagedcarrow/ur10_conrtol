// cuda_memory.cu — CUDA memory management implementation
// Thin implementation file — most logic is in header (templates).
// This file provides the explicit template instantiations needed for linking.

#include "cuda_memory.h"

// Explicit template instantiations for common types
namespace rtfg {
namespace cuda {

template class DeviceBuffer<double>;
template class DeviceBuffer<int>;
template class DeviceBuffer<float>;

// Constant memory is instantiated as needed by kernel files.
// The __constant__ variables themselves are defined in cuda_kernels.cu.

}  // namespace cuda
}  // namespace rtfg
