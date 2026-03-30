# --------------------------------------------------------
# The CMakeLists.txt for:                   lib_doodle_radio
# Author(s): J. Wenger
# --------------------------------------------------------

IF(${DOODLE_RADIO_AVAIL})
    set(HEADERS DoodleRadioDriver.h)
    set(SOURCES DoodleRadioDriver.cpp)
    
    # Create library with both header and implementation
    add_library(doodle_radio ${SOURCES} ${HEADERS})
    target_include_directories(doodle_radio PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(doodle_radio PUBLIC 
        nlohmann_json::nlohmann_json
        ${CURL_LIBRARIES}
    )
    
    message(STATUS "Building lib_doodle_radio with nlohmann/json and CURL support")
ELSE()
    message(STATUS "Skipping lib_doodle_radio - nlohmann/json not available")
ENDIF()