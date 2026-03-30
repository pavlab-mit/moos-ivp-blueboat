#--------------------------------------------------------
# The CMakeLists.txt for:                       iBackSeatBroker
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  BackSeatBroker.cpp
  BackSeatBroker_Info.cpp
  main.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/src/01_libraries/lib_nb_utils)
include_directories(${CMAKE_SOURCE_DIR}/src/01_libraries/lib_broker_v2)

ADD_EXECUTABLE(iBackSeatBroker ${SRC})

TARGET_LINK_LIBRARIES(iBackSeatBroker
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

