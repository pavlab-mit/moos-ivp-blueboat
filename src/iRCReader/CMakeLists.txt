#--------------------------------------------------------
# The CMakeLists.txt for:                       iRCReader
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

SET(SRC
  RCReader.cpp
  RCReader_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iRCReader ${SRC})

TARGET_LINK_LIBRARIES(iRCReader
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   sbus
   m
   pthread)
