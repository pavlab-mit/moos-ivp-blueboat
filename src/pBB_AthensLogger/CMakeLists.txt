#--------------------------------------------------------
# The CMakeLists.txt for:                       pBB_AthensLogger
# Author(s):                              Ray Turrisi
#--------------------------------------------------------

SET(SRC
  BB_AthensLogger.cpp
  BB_AthensLogger_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBB_AthensLogger ${SRC})

target_compile_options(pBB_AthensLogger PRIVATE -Os)

# For compute-heavy apps (simulators, estimators), consider:
#   target_compile_options(pBB_AthensLogger PRIVATE -O3)
#   target_compile_options(pBB_AthensLogger PRIVATE -ffast-math)
#   set_property(TARGET pBB_AthensLogger PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

TARGET_LINK_LIBRARIES(pBB_AthensLogger
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
