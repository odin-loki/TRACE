# SPDX-License-Identifier: AGPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

# Console applications. Each is a thin driver over TRACE::core.

add_executable(trace_maze maze_main.cpp)
target_link_libraries(trace_maze PRIVATE TRACE::core)

add_executable(trace_sim sim_main.cpp)
target_link_libraries(trace_sim PRIVATE TRACE::core)

# Replays real MOTChallenge sequences; see scripts/fetch_mot.sh for the data.
add_executable(trace_mot mot_main.cpp)
target_link_libraries(trace_mot PRIVATE TRACE::core)

# Scalability sweep: how does cost grow with the number of tracked entities?
add_executable(trace_bench bench_main.cpp)
target_link_libraries(trace_bench PRIVATE TRACE::core)

# The library already installs. These are the tools the README tells people to
# run, and a prefix that does not contain them is a prefix you still have to
# scrape the build tree for.
install(TARGETS trace_maze trace_sim trace_mot trace_bench
    RUNTIME DESTINATION bin)

if(TRACE_WITH_QT)
    add_subdirectory(gui)
endif()
