#--------------------------------------------------------
# The CMakeLists.txt for:                       iFrontSeatBroker
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  FrontSeatBroker.cpp
  FrontSeatBroker_Info.cpp
  main.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/src/01_libraries/lib_nb_utils)
include_directories(${CMAKE_SOURCE_DIR}/src/01_libraries/lib_broker_v2)

ADD_EXECUTABLE(iFrontSeatBroker ${SRC})

TARGET_LINK_LIBRARIES(iFrontSeatBroker
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

