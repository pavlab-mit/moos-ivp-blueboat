##############################################################################
# FILE:        moos-ivp-blueboat/src/CMakeLists.txt
# DATE:        2025/03/30
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-blueboat source directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS
    ./lib_*
    ./01_libraries/lib_*
)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#============================================================================
# List the subdirectories to build...
#============================================================================

# Libraries
ADD_SUBDIRECTORY(01_libraries)

# Applets / test utilities
ADD_SUBDIRECTORY(02_applets)

# Processes - always build (developer simulation support)
ADD_SUBDIRECTORY(pThrustMix)
ADD_SUBDIRECTORY(pBB_Health)
ADD_SUBDIRECTORY(uRTCMBroker)
ADD_SUBDIRECTORY(iFrontSeatBroker)
ADD_SUBDIRECTORY(iBackSeatBroker)
ADD_SUBDIRECTORY(uBBBundler)
ADD_SUBDIRECTORY(iUnicoreGPS)

# Armadillo-dependent processes
IF(${ARMA_AVAIL})
    ADD_SUBDIRECTORY(pBB_EKF)
    ADD_SUBDIRECTORY(pBB_DGPS_EKF)
ENDIF(${ARMA_AVAIL})

# JSON-dependent processes (DoodleLabs radio)
IF(${DOODLE_RADIO_AVAIL})
    ADD_SUBDIRECTORY(iDoodleLabs)
ENDIF(${DOODLE_RADIO_AVAIL})

# Vehicle-only components (require Navigator hardware and Armadillo)
IF(${ON_VEHICLE} AND ${ARMA_AVAIL})
    ADD_SUBDIRECTORY(iBBNavigatorInterface)
ENDIF()

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
