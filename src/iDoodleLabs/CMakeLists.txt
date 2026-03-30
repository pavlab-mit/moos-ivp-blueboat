#--------------------------------------------------------
# The CMakeLists.txt for:                       iDoodleLabs
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  DoodleLabs.cpp
  DoodleLabs_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iDoodleLabs ${SRC})

target_include_directories(iDoodleLabs PRIVATE
    ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_doodle_radio
    ${NLOHMANN_JSON_INCLUDE_DIR}
)

TARGET_LINK_LIBRARIES(iDoodleLabs
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   doodle_radio
   curl
   apputil
   geometry 
   mbutil
   m
   pthread)

