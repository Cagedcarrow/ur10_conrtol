# CMakeLists.txt 与 package.xml 详解

## CMakeLists.txt

### 总体结构

```cmake
cmake_minimum_required(VERSION 3.22)
project(assembly_rtfg_cuda)

# ── CUDA 语言支持 ──
enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES 89)     # Ada Lovelace RTX 4060
set(CMAKE_CUDA_FLAGS "-lineinfo -O3 --ptxas-options=-v")

# ── 依赖查找 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(CUDA REQUIRED)          # CUDA 13.3
# ... 其他 ROS2 依赖 ...

# ── 静态库: CUDA 核心 ──
add_library(rtfg_cuda_core STATIC
    src/cuda/cuda_kernels.cu          # 核函数
    src/cuda/cuda_ik_solver.cu        # CudaBatchIK
    src/cuda/cuda_memory.cu           # 模板实例化
    src/ik_solver.cpp                 # IK 工厂 (从 cpp 复制)
    src/trajectory_solver.cpp         # 轨迹求解 (从 cpp 复制)
)

# ── 可执行文件: 求解器节点 ──
add_executable(rtfg_cuda_solver_node
    src/rtfg_solver_node.cpp
)
target_link_libraries(rtfg_cuda_solver_node
    rtfg_cuda_core                    # 链接 CUDA 核心库
    ${cuda_libraries}
)

# ── CUDA 编译设置 ──
set_target_properties(rtfg_cuda_core PROPERTIES
    CUDA_ARCHITECTURES "89"
    CUDA_SEPARABLE_COMPILATION ON
)
```

### 关键编译选项

| 选项 | 值 | 说明 |
|------|-----|------|
| `CMAKE_CUDA_ARCHITECTURES` | 89 | 针对 RTX 4060 (sm_89) 优化 |
| `CMAKE_CUDA_FLAGS` | `-lineinfo -O3 --ptxas-options=-v` | 调试信息 + 最高优化 + 资源报告 |
| `CUDA_SEPARABLE_COMPILATION` | ON | 允许设备代码跨 .cu 文件链接 |
| `CXX_STANDARD` | 17 | C++17 标准 |

## package.xml

### 关键依赖

```xml
<package format="3">
  <name>assembly_rtfg_cuda</name>
  <version>0.1.0</version>
  <description>CUDA 13.3 accelerated UR10 trajectory fitting</description>
  <maintainer email="...">liuxiaopeng</maintainer>

  <!-- CUDA 依赖 -->
  <depend>cuda-cublas-dev</depend>     <!-- cuBLAS (当前未使用) -->
  <depend>cuda-cudart-dev</depend>     <!-- CUDA Runtime API -->

  <!-- ROS2 依赖 -->
  <depend>rclcpp</depend>
  <depend>moveit_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>control_msgs</depend>
  <depend>std_srvs</depend>
  <depend>sensor_msgs</depend>
  <depend>visualization_msgs</depend>

  <!-- 共享服务定义 -->
  <depend>assembly_rtfg_cpp</depend>
</package>
```

### 需要注意的点

1. **cuBLAS 依赖声明但未使用**: `cuda-cublas-dev` 在 package.xml 中声明但 CMakeLists.txt 中未链接 `CUDA::cublas`. 这是当前版本的已知冗余
2. **共享服务定义**: 服务从 `assembly_rtfg_cpp` 包引用，无需在本包中重新定义
3. **CUDA Runtime >= 13.3**: 需要 CUDA 13.3 或更高版本以支持 sm_89

## 构建命令

```bash
# 标准 colcon 构建
colcon build --packages-select assembly_rtfg_cuda \
    --cmake-args -DCMAKE_CUDA_ARCHITECTURES=89

# 独立编译测试
nvcc -arch=sm_89 -O3 -lineinfo --ptxas-options=-v \
    -o test_cuda_kernel \
    test/test_cuda_kernel.cu \
    src/cuda/cuda_kernels.cu \
    src/cuda/cuda_ik_solver.cu \
    -I include -Isrc/cuda -I include/assembly_rtfg_cuda \
    -I ../assembly_rtfg_cpp/include \
    -I /usr/include/eigen3 -lstdc++
```
