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
    test_assignment)

foreach(t IN LISTS TRACE_TESTS)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${t}.cpp")
        add_executable(${t} ${t}.cpp)
        target_link_libraries(${t} PRIVATE TRACE::core)
        add_test(NAME ${t} COMMAND ${t})
    endif()
endforeach()
