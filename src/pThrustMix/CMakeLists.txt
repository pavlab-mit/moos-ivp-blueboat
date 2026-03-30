#--------------------------------------------------------
# The CMakeLists.txt for:                       pThrustMix
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  ThrustMix.cpp
  ThrustMix_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pThrustMix ${SRC})

TARGET_LINK_LIBRARIES(pThrustMix
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

