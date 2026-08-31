# --------------------------------------------------------
# Unit tests for lib_bb_control
#
# Host-side only: no hardware, no MOOS runtime. Built only when
# BUILD_UNIT_TESTS is ON (./build.sh --unit_tests), registered
# with CTest so `ctest` runs the lot.
#
# test_pid_scalar_parity is the one place the moos-ivp ScalarPID
# dependency survives: as the ORACLE that pins the water-proven
# arithmetic bb::Pid must reproduce on clean ticks. It therefore
# links marine_pid (and its MOOS underpinnings); the other tests
# stay pure.
# --------------------------------------------------------

add_executable(test_pid test_pid.cpp)
target_link_libraries(test_pid PRIVATE bb_control)
add_test(NAME test_pid COMMAND test_pid)

add_executable(test_bbpid_engine test_bbpid_engine.cpp)
target_link_libraries(test_bbpid_engine PRIVATE bb_control)
add_test(NAME test_bbpid_engine COMMAND test_bbpid_engine)

# Parity replay driver (NOT a registered test: it takes a log file).
# LegacyBBPIDEngine.{h,cpp} are `git show HEAD:` extracts of the
# engine that ran on the water 27 Aug, symbol-renamed -- regenerate
# them if the baseline ever moves. Links marine_pid (ScalarPID) and
# geometry (angle180) exactly as the legacy engine did.
add_executable(bb_pid_replay
    replay/bb_pid_replay.cpp
    replay/LegacyBBPIDEngine.cpp)
target_include_directories(bb_pid_replay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/replay)
target_link_libraries(bb_pid_replay PRIVATE
    bb_control
    marine_pid
    geometry
    ${MOOS_LIBRARIES}
    mbutil
    m
    pthread)

add_executable(test_pid_scalar_parity test_pid_scalar_parity.cpp)
target_link_libraries(test_pid_scalar_parity PRIVATE
    bb_control
    marine_pid
    ${MOOS_LIBRARIES}
    mbutil
    m
    pthread)
add_test(NAME test_pid_scalar_parity COMMAND test_pid_scalar_parity)
