#--------------------------------------------------------
# The CMakeLists.txt for:                    pDiffThrustPID_v2
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

SET(SRC
  DiffThrustPID_v2.cpp
  DiffThrustPID_v2_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pDiffThrustPID_v2 ${SRC})

TARGET_LINK_LIBRARIES(pDiffThrustPID_v2
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
