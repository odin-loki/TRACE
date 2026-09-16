# SPDX-License-Identifier: AGPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

# Self-contained test binaries - no external framework, so the suite runs
# anywhere the engine builds.

set(TRACE_TESTS
    test_rng
    test_particle_filter
    test_pmbm
    test_pattern_of_life
    test_engine
    test_sim
    test_motion_constraint
    test_assignment
    test_appearance
    test_credibility
    test_scaling
    test_network)

foreach(t IN LISTS TRACE_TESTS)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${t}.cpp")
        add_executable(${t} ${t}.cpp)
        target_link_libraries(${t} PRIVATE TRACE::core)
        add_test(NAME ${t} COMMAND ${t})
    endif()
endforeach()
