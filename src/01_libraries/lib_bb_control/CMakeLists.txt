# --------------------------------------------------------
# lib_bb_control
#
# The pure core of BlueBoat feedback control: the one PID
# primitive (bb::Pid, pid.h -- ScalarPID-parity semantics plus
# the lifecycle controls ScalarPID cannot express) and the
# cascaded controller engine (BBPIDEngine) that pBBPID wraps.
#
# NO MOOS, NO HARDWARE, NO THREADS, NO CLOCK -- time enters
# only as caller-supplied timestamps/dt. Same rule as
# lib_bb_command: if something here needs MOOSTime() or a file
# descriptor, it belongs in the app, not the library.
#
# The parity TEST links marine_pid to use the real ScalarPID as
# an oracle; that dependency exists only under BUILD_UNIT_TESTS.
# --------------------------------------------------------

set(SRC pid.cpp
        BBPIDEngine.cpp)

set(HEADERS pid.h
            BBPIDEngine.h)

add_library(bb_control ${SRC})

target_include_directories(bb_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_UNIT_TESTS)
    add_subdirectory(test)
endif()
