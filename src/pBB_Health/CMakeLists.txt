#--------------------------------------------------------
# The CMakeLists.txt for:                       pBB_Health
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

SET(SRC
  BB_Health.cpp
  BB_Health_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBB_Health ${SRC})

TARGET_LINK_LIBRARIES(pBB_Health
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread)

