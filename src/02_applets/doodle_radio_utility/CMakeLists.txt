# --------------------------------------------------------
# The CMakeLists.txt for:              doodle_radio_utility
# Author(s):                           J. Wenger
# --------------------------------------------------------

SET(SRC main.cpp)

ADD_EXECUTABLE(doodle_radio_utility ${SRC})

target_include_directories(doodle_radio_utility PRIVATE
    ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_doodle_radio
    ${NLOHMANN_JSON_INCLUDE_DIR}
)

TARGET_LINK_LIBRARIES(doodle_radio_utility
    ${SYSTEM_LIBS}
    doodle_radio
    nlohmann_json::nlohmann_json
    curl
)