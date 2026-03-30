#--------------------------------------------------------
# The CMakeLists.txt for:                       uBBBundler
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  BBBundler.cpp
  BBBundler_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uBBBundler ${SRC})

TARGET_LINK_LIBRARIES(uBBBundler
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

