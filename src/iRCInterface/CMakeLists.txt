#--------------------------------------------------------
# The CMakeLists.txt for:                    iRCInterface
# Author(s):                                Jeremy Wenger
#--------------------------------------------------------

SET(SRC
  RCInterface.cpp
  RCInterface_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iRCInterface ${SRC})

TARGET_LINK_LIBRARIES(iRCInterface
   bb_command
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   crsf
   m
   pthread)
