#--------------------------------------------------------
# bb_replay -- offline .alog replay of the command pipeline
#
# Host tool: no MOOS, no navigator-cpp, no hardware. It must
# build and run on a laptop, because that is the point of the
# replay tier.
#--------------------------------------------------------

add_executable(bb_replay
    main.cpp
    alog_reader.cpp
    scenarios.cpp
)

target_link_libraries(bb_replay
    bb_command
)

# The fail-closed scenarios need no log and no hardware, so they
# belong in the ordinary test run rather than being a thing
# someone remembers to invoke. They ARE the safety argument for
# going replay -> dock -> water with no on-water shadow phase.
if(BUILD_UNIT_TESTS)
    add_test(NAME test_fail_closed_scenarios
             COMMAND bb_replay --mode=fail)
endif()
