# --------------------------------------------------------
# The CMakeLists.txt for:                   lib_MadgwickAHRS
# Author(s): Ray Turrisi
# --------------------------------------------------------

set(MADGWICK_SRC
    MadgwickAHRS.cpp
)
set(MADGWICK_HEADERS
    MadgwickAHRS.h
)
add_library(madgwicks ${MADGWICK_SRC})
target_include_directories(madgwicks PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
