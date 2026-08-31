# --------------------------------------------------------
# Unit tests for the pBB_DGPS_EKF model (BB_DGPS_EKF_Model.hpp
# is a standalone armadillo header -- no MOOS runtime needed).
#
# Host-side only, built when BUILD_UNIT_TESTS is ON
# (./build.sh --unit_tests), registered with CTest.
# --------------------------------------------------------

add_executable(test_dgps_ekf_model test_dgps_ekf_model.cpp)
target_include_directories(test_dgps_ekf_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${ARMADILLO_INCLUDE_DIR})
target_link_libraries(test_dgps_ekf_model PRIVATE
    ${ARMA_LIBS}
    m
    pthread)
add_test(NAME test_dgps_ekf_model COMMAND test_dgps_ekf_model)

# Offline A/B replay driver (NOT a registered test: it takes a log
# file). Replays a recorded GNSS_STATE / GYRO_Z_LVL_IMU stream
# through the pre-change filter (base) and the lever-arm-corrected,
# TS-gated filter (new) side by side. Needs MOOSGeodesy for the
# lat/lon -> local grid conversion the app does.
add_executable(dgps_ekf_replay replay/dgps_ekf_replay.cpp)
target_include_directories(dgps_ekf_replay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${ARMADILLO_INCLUDE_DIR})
target_link_libraries(dgps_ekf_replay PRIVATE
    ${ARMA_LIBS}
    ${MOOS_LIBRARIES}
    MOOSGeodesy
    m
    pthread)
