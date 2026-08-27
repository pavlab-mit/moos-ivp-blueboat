#--------------------------------------------------------
# The CMakeLists.txt for:              pBBCommandArbiter
#--------------------------------------------------------

SET(SRC
  BBCommandArbiter.cpp
  BBCommandArbiter_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBBCommandArbiter ${SRC})

TARGET_LINK_LIBRARIES(pBBCommandArbiter
   bb_command
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
