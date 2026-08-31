#--------------------------------------------------------
# The CMakeLists.txt for:                          pBBPID
# Author(s):                                  Karan Mahesh
#--------------------------------------------------------

SET(SRC
  BBPID.cpp
  BBPID_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBBPID ${SRC})

# The engine (and the bb::Pid primitive underneath it) live in
# lib_bb_control so they build and test without MOOS or a boat.
TARGET_LINK_LIBRARIES(pBBPID
   bb_command
   bb_control
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread)
