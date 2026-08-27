# --------------------------------------------------------
# lib_bb_command
#
# The pure core of the BlueBoat command pipeline: wire
# contract, source lease, slew limit, ArduRover skid mixer,
# ESC mapping.
#
# NO MOOS, NO HARDWARE, NO THREADS. Everything here runs on a
# laptop in milliseconds, which is the point -- it is where the
# interesting logic lives so it can be tested without a boat.
# Keep it that way: if something here needs MOOSTime() or a
# file descriptor, it belongs in the app, not the library.
# --------------------------------------------------------

set(SRC command_envelope.cpp
        command_mailbox.cpp
        authority.cpp
        mixer_stage.cpp
        actuator_stage.cpp
        slew_limiter.cpp
        skid_mixer.cpp
        esc_mapper.cpp)

set(HEADERS command_envelope.h
            command_mailbox.h
            authority.h
            mixer_stage.h
            actuator_stage.h
            slew_limiter.h
            skid_mixer.h
            esc_mapper.h)

add_library(bb_command ${SRC})

target_include_directories(bb_command PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_UNIT_TESTS)
    add_subdirectory(test)
endif()
