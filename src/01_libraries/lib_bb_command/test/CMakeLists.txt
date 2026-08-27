# --------------------------------------------------------
# Unit tests for lib_bb_command
#
# Host-side only: no hardware, no MOOS. Built only when
# BUILD_UNIT_TESTS is ON (./build.sh --unit_tests), and
# registered with CTest so `ctest` runs the lot.
#
# The mixer test reads a golden CSV generated from a
# transcription of the real ArduPilot function -- see
# tools/gen_golden_vectors.cpp. The path is compiled in so the
# test can be run from any working directory.
# --------------------------------------------------------

set(TESTS
    test_wire_format
    test_command_envelope
    test_command_mailbox
    test_authority
    test_mixer_stage
    test_actuator_stage
    test_arm_sequencer
    test_slew_limiter
    test_skid_mixer
    test_esc_mapper
)

foreach(test_name ${TESTS})
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE bb_command)
    target_compile_definitions(${test_name} PRIVATE
        BB_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden")
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
