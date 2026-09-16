# SPDX-License-Identifier: AGPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

cmake_minimum_required(VERSION 3.24)

project(TRACE
    VERSION 0.2.0
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
        # Split, because an absolute source path in an exported target is a
        # path on the machine that built it and nowhere else. The vendored
        # headers are installed alongside TRACE's own below.
        target_include_directories(trace_xsimd SYSTEM INTERFACE
            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/xsimd/include>"
            "$<INSTALL_INTERFACE:include/trace_vendor>")
        set(TRACE_XSIMD_VENDORED ON)
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
        # Split the same way trace_core's is. A bare source path here is not an
        # oversight that shows up later: `trace_core` links this target and
        # both are exported, so with the path unwrapped CMake refuses to
        # generate at all and -DTRACE_WITH_CUDA=ON stopped configuring the
        # moment the install rules were added.
        target_include_directories(trace_cuda PUBLIC
            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
            "$<INSTALL_INTERFACE:include>")
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

# ---------------------------------------------------------------------------
# Install and export
# ---------------------------------------------------------------------------
#
# There were none of these. `trace_core` declared `$<INSTALL_INTERFACE:include>`,
# which is only meaningful to an install that exports it, so `cmake --install`
# produced nothing and no consumer could `find_package(TRACE)`. A library that
# cannot be installed is a library that can only be vendored.
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# EXPORT_NAME so the installed target is `TRACE::core`, which is the alias the
# in-tree build offers and the name the documentation gives. Without it the
# export takes the raw target name and a consumer gets `TRACE::trace_core`:
# find_package succeeds, target_link_libraries fails, and the difference is
# invisible until somebody actually consumes the package.
set(TRACE_EXPORT_TARGETS trace_core)
set_target_properties(trace_core PROPERTIES EXPORT_NAME core)
if(TRACE_WITH_XSIMD AND TARGET trace_xsimd)
    list(APPEND TRACE_EXPORT_TARGETS trace_xsimd)
    set_target_properties(trace_xsimd PROPERTIES EXPORT_NAME xsimd)
endif()
if(TRACE_WITH_CUDA AND TARGET trace_cuda)
    list(APPEND TRACE_EXPORT_TARGETS trace_cuda)
    set_target_properties(trace_cuda PROPERTIES EXPORT_NAME cuda)
endif()

install(TARGETS ${TRACE_EXPORT_TARGETS}
    EXPORT TRACETargets
    ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/trace"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING PATTERN "*.hpp")

if(TRACE_XSIMD_VENDORED)
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xsimd/include/xsimd"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/trace_vendor"
        FILES_MATCHING PATTERN "*.hpp")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/xsimd/LICENSE"
        DESTINATION "${CMAKE_INSTALL_DOCDIR}"
        RENAME "LICENSE.xsimd")
endif()

# AGPL section 4 requires the licence to travel with the work.
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE"
    "${CMAKE_CURRENT_SOURCE_DIR}/THIRD-PARTY.md"
    DESTINATION "${CMAKE_INSTALL_DOCDIR}")

install(EXPORT TRACETargets
    FILE TRACETargets.cmake
    NAMESPACE TRACE::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TRACE")

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/TRACEConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/TRACEConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/TRACEConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TRACE")

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/TRACEConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/TRACEConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TRACE")

# `-march=native` is PUBLIC, so it reaches anything that links this target - and
# it means the CONSUMER's machine, not the one that built the archive. With
# xsimd on, the SIMD lane count decides sizeof(simd::Batch), so an archive built
# on one microarchitecture and consumed on another disagrees with its own
# headers about the layout of a type crossing the boundary. That is a silent
# ODR violation, and it is not hypothetical: it produced several convincing
# false measurements during this project's own audit before it was diagnosed.
#
# `trace::abi::compatible()` turns that into something a consumer can detect.
# This warning is so they know to ask.
if(TRACE_NATIVE_ARCH AND TRACE_WITH_XSIMD)
    message(STATUS
        "TRACE: -march=native + xsimd - this build is ABI-specific to this "
        "machine's SIMD width. For an installed or redistributed package "
        "configure with -DTRACE_NATIVE_ARCH=OFF, or check "
        "trace::abi::compatible() at startup.")
endif()

message(STATUS "TRACE ${PROJECT_VERSION}")
message(STATUS "  build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  xsimd      : ${TRACE_WITH_XSIMD}")
message(STATUS "  cuda       : ${TRACE_WITH_CUDA}")
message(STATUS "  qt         : ${TRACE_WITH_QT}")
