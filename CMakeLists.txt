#=======================================================================
# FILE:  moos-ivp-blueboat/CMakeLists.txt
# DATE:  2025/03/30
# INFO:  Top-level CMakeLists.txt file for the moos-ivp-blueboat project
# NAME:  Raymond Turrisi - refactored from moos-ivp-seascout
#=======================================================================

CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT( IVP_BLUEBOAT )

set (CMAKE_CXX_STANDARD 11)

#=======================================================================
# Set the output directories for the binary and library files
#=======================================================================

GET_FILENAME_COMPONENT(IVP_EXTEND_BIN_DIR "${CMAKE_SOURCE_DIR}/bin"  ABSOLUTE )
GET_FILENAME_COMPONENT(IVP_EXTEND_LIB_DIR "${CMAKE_SOURCE_DIR}/lib"  ABSOLUTE )

SET( LIBRARY_OUTPUT_PATH      "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( ARCHIVE_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( LIBRARY_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )

SET( EXECUTABLE_OUTPUT_PATH    "${IVP_EXTEND_BIN_DIR}" CACHE PATH "" )
SET( RUNTIME_OUTPUT_DIRECTORY "${IVP_EXTEND_BIN_DIR}"  CACHE PATH "" )

#=======================================================================
# Find MOOS
#=======================================================================
find_package(MOOS 10.0)
INCLUDE_DIRECTORIES(${MOOS_INCLUDE_DIRS})

#=============================================================================
# FINDING MOOSGeodesy' HEADERS AND LIBRARIES...
#=============================================================================
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS})
link_directories(${MOOSGeodesy_LIBRARY_PATH})

message("+++++++++++++++++++++++++++++++++++++++++")
message("MOOSGeodesy_INCLUDE_DIRS:" ${MOOSGeodesy_INCLUDE_DIRS})
message("MOOSGeodesy_LIBRARY_PATH:" ${MOOSGeodesy_LIBRARY_PATH})
message("+++++++++++++++++++++++++++++++++++++++++")

#=======================================================================
# Find the "moos-ivp" base directory
#=======================================================================

# Search for the moos-ivp folder
find_path( MOOSIVP_SOURCE_TREE_BASE
           NAMES build-ivp.sh build-moos.sh configure-ivp.sh
           PATHS "../moos-ivp" "../../moos-ivp" "../../moos-ivp/trunk/" "../moos-ivp/trunk/"
           DOC "Base directory of the MOOS-IvP source tree"
           NO_DEFAULT_PATH
)

if (NOT MOOSIVP_SOURCE_TREE_BASE)
    message("Please set MOOSIVP_SOURCE_TREE_BASE to  ")
    message("the location of the \"moos-ivp\" folder ")
    return()
endif()

#======================================================================
# Specify where to find IvP's headers and libraries...
#======================================================================

FILE(GLOB IVP_INCLUDE_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/ivp/src/lib_* )
INCLUDE_DIRECTORIES(${IVP_INCLUDE_DIRS})

FILE(GLOB IVP_LIBRARY_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/lib )
LINK_DIRECTORIES(${IVP_LIBRARY_DIRS})

#======================================================================
# Specify Compiler Flags
#======================================================================

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
     wsock32 )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m
      pthread )
endif (${WIN32})


IF( ${WIN32} )
   #---------------------------------------------
   # Windows Compiler Flags
   #---------------------------------------------
   IF(MSVC)
      # Flags for Microsoft Visual Studio
      SET( WALL_ON OFF CACHE BOOL
         "tell me about all compiler warnings (-Wall) ")
      IF(WALL_ON)
         SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
      ENDIF(WALL_ON)
   ELSE(MSVC)
      # Other Windows compilers go here
   ENDIF(MSVC)

ELSE( ${WIN32} )
   #---------------------------------------------
   # Linux and Apple Compiler Flags
   #---------------------------------------------
   # Force -fPIC because gcc complains when we don't use it with x86_64 code.
   # Note sure why: -fPIC should only be needed for shared objects, and
   # AFAIK, CMake gets that right when building shared objects. -CJC
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -Wdeprecated-declarations")
   IF(CMAKE_COMPILER_IS_GNUCXX)
      # Flags for the GNU C++ Compiler
      SET( WALL_ON OFF CACHE BOOL
         "tell me about all compiler warnings (-Wall) ")
      IF(WALL_ON)
         SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" -C++11)
      ENDIF( WALL_ON)

      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
   ELSE(CMAKE_COMPILER_IS_GNUCXX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
   ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ENDIF( ${WIN32} )

#=======================================================================
# Special Build Instructions
#=======================================================================

# Check if Armadillo and dependencies are available - set global environment variables to streamline build files

find_package(Armadillo)
find_package(BLAS)
find_package(LAPACK)

if(BLAS_FOUND AND LAPACK_FOUND)
    set(ARMA_AVAIL TRUE)
    if(Armadillo_FOUND)
      SET(ARMADILLO_INCLUDE_DIR ${Armadillo_INCLUDE_DIRS})
    else()
      SET(ARMADILLO_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/src/01_libraries/lib_armadillo/armadillo-12.6.6/include")
      message(WARNING "Armadillo library installation not found - using a header only include version, assuming reference to: \n\t-<${CMAKE_SOURCE_DIR}/src/01_libraries/lib_armadillo/armadillo-12.6.6/include>")
    endif()
    add_definitions(-DARMA_DONT_USE_WRAPPER)
    SET(ARMA_LIBS ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
else()
   set(ARMA_AVAIL FALSE)
   message(WARNING "BLAS and LAPACK library not found - No applications which depend on Armadillo will be built. \n Try: \n\t - Mac: <brew install openblas> \n\t - Ubuntu/Debian: <sudo apt install libblas-dev liblapack-dev>")
endif()

# Check if nlohmann/json package is available
find_package(nlohmann_json QUIET)

if(nlohmann_json_FOUND)
    set(NLOHMANN_JSON_AVAIL TRUE)
    message(STATUS "nlohmann/json found - JSON-dependent applications will be built")
else()
    set(NLOHMANN_JSON_AVAIL FALSE)
    message(WARNING "nlohmann/json not found - Applications depending on JSON (iDoodleLabs, lib_doodle_radio) will not be built. \n Try: \n\t - Mac: <brew install nlohmann-json> \n\t - Ubuntu/Debian: <sudo apt install nlohmann-json3-dev>")
endif()

# lib_doodle_radio depends on nlohmann/json
set(DOODLE_RADIO_AVAIL ${NLOHMANN_JSON_AVAIL})

# Check NAV_PLATFORM_TYPE environment variable (defaults to DEVELOPER if not set)
if("$ENV{NAV_PLATFORM_TYPE}" STREQUAL "BLUEBOAT")
      set(ON_VEHICLE TRUE)
      message(STATUS "NAV_PLATFORM_TYPE=$ENV{NAV_PLATFORM_TYPE} - Building vehicle components")
else()
      set(ON_VEHICLE FALSE)
      if(DEFINED ENV{NAV_PLATFORM_TYPE})
            message(STATUS "NAV_PLATFORM_TYPE=$ENV{NAV_PLATFORM_TYPE} - Skipping vehicle components")
      else()
            message(STATUS "NAV_PLATFORM_TYPE=DEVELOPER (default) - Skipping vehicle components")
      endif()
endif()

IF(${ON_VEHICLE})
   include(ExternalProject)
   # Navigator version and download URL
   set(NAVIGATOR_VERSION "0.0.6")
   set(ZIP_URL "https://github.com/bluerobotics/navigator-lib/releases/download/${NAVIGATOR_VERSION}/cpp.zip")

   set(ZIP_UNPACK_DIR "${CMAKE_BINARY_DIR}/navigator_lib")
   message(STATUS "Identified system processor: ${CMAKE_SYSTEM_PROCESSOR}")

   # Download and unpack Navigator library
   ExternalProject_Add( navigator_lib
   URL ${ZIP_URL}
   DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/navigator_lib_download"
   SOURCE_DIR "${CMAKE_BINARY_DIR}/navigator_lib"
   CONFIGURE_COMMAND ""
   BUILD_COMMAND ""
   INSTALL_COMMAND ""
   TEST_COMMAND ""
   BUILD_BYPRODUCTS "${NAVIGATOR_LIB}"  # Specify the expected library file as a byproduct
)

      if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l")
         set(LIB_PATH_SUBDIR "armv7-unknown-linux-gnueabihf")
      elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv8" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
         set(LIB_PATH_SUBDIR "armv7-unknown-linux-gnueabihf")
      else()
         message(FATAL_ERROR "Unsupported architecture")
   endif()

   set(NAVIGATOR_LIB "${ZIP_UNPACK_DIR}/${LIB_PATH_SUBDIR}/libbluerobotics_navigator.so")
   set(NAVIGATOR_INCLUDE "${ZIP_UNPACK_DIR}/")
   include_directories(${NAVIGATOR_INCLUDE})

ENDIF(${ON_VEHICLE})

#=======================================================================
# Find navigator-common (sibling repository with shared utilities)
#=======================================================================
find_path(NAVIGATOR_COMMON_DIR
    NAMES src/01_libraries/lib_rpi_utils
    PATHS "../navigator-common" "../../navigator-common"
    DOC "Base directory of the navigator-common repository"
    NO_DEFAULT_PATH
)

if(NAVIGATOR_COMMON_DIR)
    message(STATUS "Found navigator-common at: ${NAVIGATOR_COMMON_DIR}")
    FILE(GLOB NAVIGATOR_COMMON_LIBS ${NAVIGATOR_COMMON_DIR}/src/01_libraries/lib_*)
    INCLUDE_DIRECTORIES(${NAVIGATOR_COMMON_LIBS})
    LINK_DIRECTORIES(${NAVIGATOR_COMMON_DIR}/lib)
else()
    message(WARNING "navigator-common not found - some components may not build")
endif()

#=======================================================================
# Add Subdirectories
#=======================================================================

ADD_SUBDIRECTORY( src )
