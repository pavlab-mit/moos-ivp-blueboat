#--------------------------------------------------------
# The CMakeLists.txt for:                       iUnicoreGPS
# Author(s):                              Jeremy Wenger
#--------------------------------------------------------

SET(SRC
  UnicoreGPS.cpp
  UnicoreGPS_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iUnicoreGPS ${SRC})

TARGET_LINK_LIBRARIES(iUnicoreGPS
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   mbutil
   unicore
   m
   pthread)
