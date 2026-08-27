#=======================================================================
# FILE:  moos-ivp-blueboat/CMakeLists.txt
# DATE:  2025/03/30
# INFO:  Top-level CMakeLists.txt file for the moos-ivp-blueboat project
# NAME:  Raymond Turrisi - refactored from moos-ivp-seascout
#=======================================================================

CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT( IVP_BLUEBOAT )

set (CMAKE_CXX_STANDARD 14)

# Host-side unit tests are opt-in so the normal build stays lean:
#   ./build.sh --unit_tests
option(BUILD_UNIT_TESTS "Build unit test binaries" OFF)

# CTest. Tests register themselves via add_test() in each
# lib's test/CMakeLists.txt, so:
#   ./build.sh --unit_tests && (cd build && ctest --output-on-failure)
# runs every host-side test in the repo with one command.
if(BUILD_UNIT_TESTS)
    enable_testing()
endif()

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

#=======================================================================
# Find navigator-cpp (pavlab-mit) - pure C++ Navigator hardware library
#=======================================================================
# Replaces the Blue Robotics navigator-lib prebuilt download. The
# library auto-detects Navigator V1/V2 and Pi 4/5 at runtime, so the
# old NAVOS_VERSION / RASPBERRY_PI_VERSION env vars are gone. On dev
# machines the library builds with platform stubs, so Navigator apps
# build (and link) everywhere the library is found.
#
# Search order:
#   1. Installed copy (/usr/local via navigator-cpp's install.sh)
#   2. Sibling checkout built in-tree (../navigator-cpp)

find_library(NAVIGATOR_CPP_LIB navigator_cpp
    PATHS "${CMAKE_SOURCE_DIR}/../navigator-cpp/lib"
)
find_path(NAVIGATOR_CPP_INCLUDE nav_bindings.h
    PATHS "${CMAKE_SOURCE_DIR}/../navigator-cpp/include"
)

if(NAVIGATOR_CPP_LIB AND NAVIGATOR_CPP_INCLUDE)
    set(NAVIGATOR_CPP_FOUND TRUE)
    message(STATUS "Found navigator-cpp: ${NAVIGATOR_CPP_LIB}")
    message(STATUS "navigator-cpp headers: ${NAVIGATOR_CPP_INCLUDE}")
    # A STATIC navigator-cpp (older installs shipped libnavigator_cpp.a)
    # does not carry its libgpiod dependency the way the shared .so
    # does, so consumers must link gpiod explicitly or every GPIO
    # symbol fails at link time.
    if(NAVIGATOR_CPP_LIB MATCHES "\\.a$")
        find_library(GPIOD_LIB gpiod)
        if(GPIOD_LIB)
            message(STATUS "Static navigator-cpp: also linking ${GPIOD_LIB}")
            set(NAVIGATOR_CPP_LIB "${NAVIGATOR_CPP_LIB};${GPIOD_LIB}")
        else()
            message(WARNING "Static navigator-cpp found but libgpiod is missing - Navigator apps will fail to link. Try: sudo apt install libgpiod-dev (or update navigator-cpp to the shared-library version)")
        endif()
    endif()
else()
    set(NAVIGATOR_CPP_FOUND FALSE)
    message(WARNING "navigator-cpp not found - Navigator hardware apps (iBBNavigatorInterface, vehicle applets) will not be built. \n Try: \n\t - clone https://github.com/pavlab-mit/navigator-cpp beside this repo, ./build.sh -r, and (on the vehicle) sudo ./install.sh")
endif()

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
