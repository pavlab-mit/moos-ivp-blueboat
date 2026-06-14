#--------------------------------------------------------
# The CMakeLists.txt for:                       pBB_Status
# Author(s):
#--------------------------------------------------------

SET(SRC
  BB_Status.cpp
  BB_Status_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBB_Status ${SRC})

TARGET_LINK_LIBRARIES(pBB_Status
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
