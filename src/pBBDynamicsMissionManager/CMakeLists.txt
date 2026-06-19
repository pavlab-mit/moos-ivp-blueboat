#--------------------------------------------------------
# The CMakeLists.txt for:                       pBBDynamicsMissionManager
# Author(s):               Karan Mahesh                
#--------------------------------------------------------

SET(SRC
  BBDynamicsMissionManager.cpp
  BBDynamicsMissionManager_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBBDynamicsMissionManager ${SRC})

TARGET_LINK_LIBRARIES(pBBDynamicsMissionManager
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

