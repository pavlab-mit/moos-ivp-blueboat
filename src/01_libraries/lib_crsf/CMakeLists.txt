# --------------------------------------------------------
# The CMakeLists.txt for: lib_crsf
# CRSF (Crossfire) protocol parser for ELRS receivers
# --------------------------------------------------------

set(SRC crsf_parser.cpp
        crsf_frames.cpp
        rc_link_state.cpp)

set(HEADERS crsf_parser.h
            crsf_frames.h
            rc_link_state.h)

add_library(crsf ${SRC})

target_include_directories(crsf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Unit tests are opt-in: ./build.sh --unit_tests
if(BUILD_UNIT_TESTS)
    add_subdirectory(test)
endif()
