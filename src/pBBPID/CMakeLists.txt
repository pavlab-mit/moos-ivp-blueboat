#--------------------------------------------------------
# The CMakeLists.txt for:                          pBBPID
# Author(s):                                  Karan Mahesh
#--------------------------------------------------------

SET(SRC
  BBPID.cpp
  BBPIDEngine.cpp
  BBPID_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBBPID ${SRC})

TARGET_LINK_LIBRARIES(pBBPID
   bb_command
   ${MOOS_LIBRARIES}
   marine_pid
   apputil
   geometry
   mbutil
   m
   pthread)
