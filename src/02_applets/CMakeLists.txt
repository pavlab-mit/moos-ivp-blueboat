##############################################################################
# FILE:        moos-ivp-blueboat/src/02_applets/CMakeLists.txt
# DATE:        2025/03/30
# DESCRIPTION: CMakeLists.txt for moos-ivp-blueboat applets/test utilities
##############################################################################

# Doodle Radio utility (requires JSON library)
IF(${DOODLE_RADIO_AVAIL})
    ADD_SUBDIRECTORY(doodle_radio_utility)
ENDIF(${DOODLE_RADIO_AVAIL})

# Vehicle-only applets (require Navigator hardware)
IF(${ON_VEHICLE})
    ADD_SUBDIRECTORY(init_bb_pwm)

    # Boot-script helpers (used by scripts/bb_init.sh)
    ADD_SUBDIRECTORY(bb_adc)        # battery-voltage read (low-battery gate)
    ADD_SUBDIRECTORY(bb_neopixel)   # optional LED-strip status (future hardware)

    # bb_attitude requires Armadillo
    IF(${ARMA_AVAIL})
        ADD_SUBDIRECTORY(bb_attitude)
        ADD_SUBDIRECTORY(sample_navigator)
    ENDIF(${ARMA_AVAIL})

    # test_gnss_heading uses the UM982 (Unicore) parser library
    ADD_SUBDIRECTORY(test_gnss_heading)
ENDIF(${ON_VEHICLE})

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
