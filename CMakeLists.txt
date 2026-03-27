cmake_minimum_required(VERSION 3.24)
project(SimRobot VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# vcpkg 工具链（如果使用 vcpkg，构建时传入 CMAKE_TOOLCHAIN_FILE）
# cmake -B build -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

# ── 依赖 ──────────────────────────────────────────────
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# ── 核心库（header-only） ──────────────────────────────
add_library(simrobot_core INTERFACE)
target_include_directories(simrobot_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(simrobot_core INTERFACE Eigen3::Eigen)
target_compile_options(simrobot_core INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# ── 测试 ──────────────────────────────────────────────
option(SIMROBOT_BUILD_TESTS "Build unit tests" ON)
if(SIMROBOT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ── 示例 ──────────────────────────────────────────────
option(SIMROBOT_BUILD_EXAMPLES "Build examples" OFF)
if(SIMROBOT_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
