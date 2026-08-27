##############################################################################
# FILE:        moos-ivp-blueboat/src/02_applets/CMakeLists.txt
# DATE:        2026/07/24
# DESCRIPTION: CMakeLists.txt for moos-ivp-blueboat applets/test utilities
##############################################################################

# SBUS / RC-link diagnostic (builds everywhere; runs on the vehicle)
ADD_SUBDIRECTORY(sbus_probe)

# RC-link diagnostic for BOTH protocols (SBUS + CRSF), with
# protocol auto-detection. Supersedes sbus_probe; the older tool
# stays until the fleet migration to CRSF is complete.
ADD_SUBDIRECTORY(rc_probe)

# CRSF telemetry downlink test: sends battery + flight-mode frames
# and watches the receiver's downlink LQ to confirm the TX path.
ADD_SUBDIRECTORY(rc_telem_test)

# Doodle Radio utility (requires JSON library)
IF(${DOODLE_RADIO_AVAIL})
    ADD_SUBDIRECTORY(doodle_radio_utility)
ENDIF(${DOODLE_RADIO_AVAIL})

# Navigator hardware applets (navigator-cpp). Build wherever the
# library is found - on dev machines navigator-cpp provides platform
# stubs, so these compile and link everywhere; hardware calls return
# errors at runtime off-vehicle. Board revision (V1/V2) and Pi model
# are detected at runtime, so there is no per-version gating.
IF(${NAVIGATOR_CPP_FOUND})
    ADD_SUBDIRECTORY(init_bb_pwm)   # ESC arming (boot script)
    ADD_SUBDIRECTORY(bb_adc)        # battery-voltage gate for bb_init.sh (VOLTAGE=)
    ADD_SUBDIRECTORY(test_bb_adc)   # interactive ADC/current diagnostic
    ADD_SUBDIRECTORY(bb_neopixel)   # optional LED-strip status (future hardware)
    ADD_SUBDIRECTORY(bb_attitude)   # pitch gate for bb_init.sh
    ADD_SUBDIRECTORY(sample_navigator) # cal data collector (gyro/mag cal scripts)
ENDIF()

# test_gnss_heading uses the UM982 (Unicore) parser library
ADD_SUBDIRECTORY(test_gnss_heading)

# bb_replay: offline .alog replay of the command pipeline. Pure
# host tool (lib_bb_command only) -- builds everywhere.
ADD_SUBDIRECTORY(bb_replay)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
