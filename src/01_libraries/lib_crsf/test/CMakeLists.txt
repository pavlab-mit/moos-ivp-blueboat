# --------------------------------------------------------
# Unit tests for lib_crsf
#
# Host-side only - no hardware, no MOOS. Built only when
# BUILD_UNIT_TESTS is ON (./build.sh --unit_tests).
# --------------------------------------------------------

set(TESTS
    test_crsf_parser
    test_crsf_frames
    test_rc_link_state
)

foreach(test_name ${TESTS})
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE crsf)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
