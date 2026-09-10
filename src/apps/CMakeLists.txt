# Console applications. Each is a thin driver over TRACE::core.

add_executable(trace_maze maze_main.cpp)
target_link_libraries(trace_maze PRIVATE TRACE::core)

add_executable(trace_sim sim_main.cpp)
target_link_libraries(trace_sim PRIVATE TRACE::core)

if(TRACE_WITH_QT)
    add_subdirectory(gui)
endif()
