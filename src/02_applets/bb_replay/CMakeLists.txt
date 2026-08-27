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
)

target_link_libraries(bb_replay
    bb_command
)
