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

    # bb_attitude requires Armadillo
    IF(${ARMA_AVAIL})
        ADD_SUBDIRECTORY(bb_attitude)
    ENDIF(${ARMA_AVAIL})

    # test_gnss_heading uses the UM982 (Unicore) parser library
    ADD_SUBDIRECTORY(test_gnss_heading)
ENDIF(${ON_VEHICLE})

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
