#pragma once
// cuda_memory.h — RAII wrappers for CUDA device memory management (CUDA 13.3)

#include <cuda_runtime.h>
#include <stdexcept>
#include <string>

namespace rtfg {
namespace cuda {

// ============================================================================
// DeviceBuffer<T> — RAII wrapper for cudaMalloc / cudaFree
// ============================================================================
template <typename T>
class DeviceBuffer {
public:
  DeviceBuffer() : ptr_(nullptr), count_(0) {}

  explicit DeviceBuffer(size_t count) : count_(count) {
    cudaError_t err = cudaMalloc(&ptr_, count * sizeof(T));
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaMalloc failed: " +
                               std::string(cudaGetErrorString(err)));
    }
  }

  ~DeviceBuffer() {
    if (ptr_) {
      cudaFree(ptr_);
    }
  }

  // Non-copyable
  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

  // Movable
  DeviceBuffer(DeviceBuffer&& other) noexcept
      : ptr_(other.ptr_), count_(other.count_) {
    other.ptr_ = nullptr;
    other.count_ = 0;
  }

  DeviceBuffer& operator=(DeviceBuffer&& other) noexcept {
    if (this != &other) {
      if (ptr_) cudaFree(ptr_);
      ptr_ = other.ptr_;
      count_ = other.count_;
      other.ptr_ = nullptr;
      other.count_ = 0;
    }
    return *this;
  }

  // Resize (destroys old data)
  void resize(size_t count) {
    if (ptr_) cudaFree(ptr_);
    count_ = count;
    cudaError_t err = cudaMalloc(&ptr_, count * sizeof(T));
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaMalloc (resize) failed: " +
                               std::string(cudaGetErrorString(err)));
    }
  }

  // Copy host → device
  void toDevice(const T* host_data, size_t count, cudaStream_t stream = 0) {
    if (count > count_) {
      throw std::runtime_error("DeviceBuffer::toDevice: count exceeds capacity");
    }
    cudaError_t err = cudaMemcpyAsync(ptr_, host_data, count * sizeof(T),
                                       cudaMemcpyHostToDevice, stream);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaMemcpy H2D failed: " +
                               std::string(cudaGetErrorString(err)));
    }
  }

  void toDevice(const T* host_data) { toDevice(host_data, count_); }

  // Copy device → host
  void toHost(T* host_data, size_t count, cudaStream_t stream = 0) const {
    if (count > count_) {
      throw std::runtime_error("DeviceBuffer::toHost: count exceeds capacity");
    }
    cudaError_t err = cudaMemcpyAsync(host_data, ptr_, count * sizeof(T),
                                       cudaMemcpyDeviceToHost, stream);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaMemcpy D2H failed: " +
                               std::string(cudaGetErrorString(err)));
    }
  }

  void toHost(T* host_data) const { toHost(host_data, count_); }

  // Accessors
  T* get() { return ptr_; }
  const T* get() const { return ptr_; }
  size_t size() const { return count_; }
  size_t bytes() const { return count_ * sizeof(T); }
  bool empty() const { return count_ == 0 || ptr_ == nullptr; }

private:
  T* ptr_;
  size_t count_;
};

// ============================================================================
// ConstantMemory<T> — RAII wrapper for __constant__ memory initialization
// Usage:
//   __constant__ double c_data[MAX];
//   ConstantMemory<double, MAX> cm(c_data);
//   cm.fromHost(host_data);
// ============================================================================
template <typename T>
class ConstantMemory {
public:
  ConstantMemory() : symbol_ptr_(nullptr) {}

  // Initialize with pointer to __constant__ symbol
  void bind(const void* symbol_ptr) { symbol_ptr_ = symbol_ptr; }

  // Copy from host to constant memory
  void fromHost(const T* host_data, size_t count, cudaStream_t stream = 0) {
    cudaError_t err = cudaMemcpyToSymbolAsync(symbol_ptr_, host_data,
                                               count * sizeof(T), 0,
                                               cudaMemcpyHostToDevice, stream);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaMemcpyToSymbol failed: " +
                               std::string(cudaGetErrorString(err)));
    }
  }

private:
  const void* symbol_ptr_;
};

// ============================================================================
// CUDA context / device management helpers
// ============================================================================

inline void initCudaDevice(int device_id = 0) {
  cudaError_t err = cudaSetDevice(device_id);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaSetDevice(" + std::to_string(device_id) +
                             ") failed: " + std::string(cudaGetErrorString(err)));
  }
}

inline void syncDevice() {
  cudaError_t err = cudaDeviceSynchronize();
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaDeviceSynchronize failed: " +
                             std::string(cudaGetErrorString(err)));
  }
}

inline int getDeviceCount() {
  int count = 0;
  cudaGetDeviceCount(&count);
  return count;
}

}  // namespace cuda
}  // namespace rtfg
