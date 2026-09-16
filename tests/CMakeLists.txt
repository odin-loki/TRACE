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
    test_network
    test_detectors)

# A named test whose source is missing is a configuration ERROR, not a test to
# skip. The `if(EXISTS ...)` here used to swallow it silently: delete or
# mistype a file and the suite simply got smaller, ctest reported every
# remaining test passing, and nothing anywhere said one had gone. That is the
# same failure shape as a CI job that cannot start reporting success.
foreach(t IN LISTS TRACE_TESTS)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${t}.cpp")
        message(FATAL_ERROR
            "TRACE_TESTS names ${t} but ${CMAKE_CURRENT_SOURCE_DIR}/${t}.cpp "
            "does not exist. Add the file, or remove the name from the list.")
    endif()
    add_executable(${t} ${t}.cpp)
    target_link_libraries(${t} PRIVATE TRACE::core)
    add_test(NAME ${t} COMMAND ${t})
endforeach()

# And a test source nobody listed is a test nobody runs. Catching that needs
# the reverse check, because the list is the authority and the directory is
# not.
file(GLOB TRACE_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
foreach(src IN LISTS TRACE_TEST_SOURCES)
    get_filename_component(stem "${src}" NAME_WE)
    if(NOT stem IN_LIST TRACE_TESTS)
        message(FATAL_ERROR
            "${stem}.cpp exists but is not in TRACE_TESTS, so it would never "
            "be built or run. Add it to the list, or delete the file.")
    endif()
endforeach()

message(STATUS "TRACE: ${CMAKE_CURRENT_SOURCE_DIR} - "
               "building and registering every test in TRACE_TESTS")
