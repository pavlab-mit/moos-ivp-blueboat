#--------------------------------------------------------
# The CMakeLists.txt for:                       iTeleop
# Author(s):
#--------------------------------------------------------

SET(SRC
  Teleop.cpp
  Teleop_Info.cpp
  main.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/src/01_libraries/lib_broker_v2)

ADD_EXECUTABLE(iTeleop ${SRC})

TARGET_LINK_LIBRARIES(iTeleop
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
