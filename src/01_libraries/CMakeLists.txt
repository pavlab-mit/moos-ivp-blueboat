##############################################################################
# FILE:        moos-ivp-blueboat/src/01_libraries/CMakeLists.txt
# DATE:        2025/03/30
# DESCRIPTION: CMakeLists.txt for moos-ivp-blueboat libraries
##############################################################################

# Madgwick AHRS filter
ADD_SUBDIRECTORY(lib_MadgwickAHRS)

# SBUS protocol parser for RC receivers
ADD_SUBDIRECTORY(lib_sbus)

# Unicore GNSS parser
ADD_SUBDIRECTORY(lib_unicore)

# Doodle Radio interface (requires nlohmann/json)
IF(${DOODLE_RADIO_AVAIL})
    ADD_SUBDIRECTORY(lib_doodle_radio)
ENDIF(${DOODLE_RADIO_AVAIL})

# lib_broker_v2 is header-only, no build needed
# lib_nb_utils  is header-only, no build needed
# lib_filters   is header-only, no build needed (LPF / LPFSmoother)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
