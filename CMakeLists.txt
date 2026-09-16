# SPDX-License-Identifier: AGPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

cmake_minimum_required(VERSION 3.24)

project(TRACE
    VERSION 0.1.0
    DESCRIPTION "TRACE - Tracking, Re-identification, Association, Convergence & Events"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(TRACE_WITH_XSIMD "Vectorise hot loops with xsimd"        ON)
option(TRACE_WITH_CUDA  "Build CUDA particle/GMM kernels"       OFF)
option(TRACE_WITH_QT    "Build the Qt operator console"         OFF)
option(TRACE_NATIVE_ARCH "Tune for the building machine's CPU"  ON)
option(TRACE_BUILD_TESTS "Build the test suite"                 ON)

# ---------------------------------------------------------------------------
# xsimd
# ---------------------------------------------------------------------------
if(TRACE_WITH_XSIMD)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xsimd/include/xsimd/xsimd.hpp")
        add_library(trace_xsimd INTERFACE)
        target_include_directories(trace_xsimd SYSTEM INTERFACE
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xsimd/include")
        message(STATUS "TRACE: using vendored xsimd")
    else()
        find_package(xsimd QUIET)
        if(xsimd_FOUND)
            add_library(trace_xsimd INTERFACE)
            target_link_libraries(trace_xsimd INTERFACE xsimd)
            message(STATUS "TRACE: using system xsimd")
        else()
            message(WARNING "TRACE: xsimd not found - falling back to scalar kernels")
            set(TRACE_WITH_XSIMD OFF)
        endif()
    endif()
endif()

# ---------------------------------------------------------------------------
# CUDA (optional; the engine has a complete CPU path)
# ---------------------------------------------------------------------------
if(TRACE_WITH_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        set(CMAKE_CUDA_STANDARD 20)
        set(CMAKE_CUDA_STANDARD_REQUIRED ON)
        if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
            set(CMAKE_CUDA_ARCHITECTURES 70 75 80 86 89 90)
        endif()
        message(STATUS "TRACE: CUDA enabled (${CMAKE_CUDA_COMPILER})")
    else()
        message(WARNING "TRACE: no CUDA compiler found - disabling CUDA backend")
        set(TRACE_WITH_CUDA OFF)
    endif()
endif()

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
file(GLOB TRACE_CORE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/detectors/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sim/*.cpp")

add_library(trace_core ${TRACE_CORE_SOURCES})
add_library(TRACE::core ALIAS trace_core)

target_include_directories(trace_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

target_compile_features(trace_core PUBLIC cxx_std_23)

if(TRACE_WITH_XSIMD)
    target_link_libraries(trace_core PUBLIC trace_xsimd)
    # No TRACE_XSIMD_ARCH_NAME. It was set to "generic" here and never derived
    # from the build, so simd::backend_name() misreported every vectorised
    # build; it now asks xsimd what architecture the batch type resolved to.
    target_compile_definitions(trace_core PUBLIC TRACE_WITH_XSIMD)
endif()

if(MSVC)
    target_compile_options(trace_core PRIVATE /W4 /permissive-)
    if(TRACE_NATIVE_ARCH)
        target_compile_options(trace_core PUBLIC /arch:AVX2)
    endif()
else()
    target_compile_options(trace_core PRIVATE
        -Wall -Wextra -Wpedantic -Wno-unused-parameter)
    if(TRACE_NATIVE_ARCH)
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-march=native" TRACE_HAS_MARCH_NATIVE)
        if(TRACE_HAS_MARCH_NATIVE)
            target_compile_options(trace_core PUBLIC -march=native)
        endif()
    endif()
endif()

if(TRACE_WITH_CUDA)
    file(GLOB TRACE_CUDA_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda/*.cu")
    if(TRACE_CUDA_SOURCES)
        add_library(trace_cuda ${TRACE_CUDA_SOURCES})
        target_include_directories(trace_cuda PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}/include")
        set_target_properties(trace_cuda PROPERTIES
            CUDA_SEPARABLE_COMPILATION ON
            POSITION_INDEPENDENT_CODE ON)
        target_compile_definitions(trace_cuda PUBLIC TRACE_WITH_CUDA)
        target_link_libraries(trace_core PUBLIC trace_cuda)
        target_compile_definitions(trace_core PUBLIC TRACE_WITH_CUDA)
    endif()
endif()

# ---------------------------------------------------------------------------
# Applications
# ---------------------------------------------------------------------------
add_subdirectory(src/apps)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(TRACE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

message(STATUS "TRACE ${PROJECT_VERSION}")
message(STATUS "  build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  xsimd      : ${TRACE_WITH_XSIMD}")
message(STATUS "  cuda       : ${TRACE_WITH_CUDA}")
message(STATUS "  qt         : ${TRACE_WITH_QT}")
