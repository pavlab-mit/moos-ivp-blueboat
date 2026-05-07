#--------------------------------------------------------
# The CMakeLists.txt for:                          iUnicore
# Author(s):                              Jeremy Wenger
#--------------------------------------------------------

SET(SRC
  Unicore.cpp
  Unicore_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iUnicore ${SRC})

TARGET_LINK_LIBRARIES(iUnicore
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   unicore
   m
   pthread)
