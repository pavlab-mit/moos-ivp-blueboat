#--------------------------------------------------------
# The CMakeLists.txt for:                       uRTCMBroker
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  RTCMBroker.cpp
  RTCMBroker_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uRTCMBroker ${SRC})

TARGET_LINK_LIBRARIES(uRTCMBroker
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

