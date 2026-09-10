# Qt operator console. Built only when TRACE_WITH_QT is on.

find_package(Qt6 6.4 COMPONENTS Widgets QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 5.15 COMPONENTS Widgets QUIET)
endif()

if(NOT Qt6_FOUND AND NOT Qt5_FOUND)
    message(WARNING "TRACE: Qt not found - skipping the operator console")
    return()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(trace_console
    main.cpp
    console_window.cpp
    console_window.hpp
    track_view.cpp
    track_view.hpp)

target_link_libraries(trace_console PRIVATE TRACE::core)

if(Qt6_FOUND)
    target_link_libraries(trace_console PRIVATE Qt6::Widgets)
    target_compile_definitions(trace_console PRIVATE TRACE_QT_VERSION=6)
else()
    target_link_libraries(trace_console PRIVATE Qt5::Widgets)
    target_compile_definitions(trace_console PRIVATE TRACE_QT_VERSION=5)
endif()
