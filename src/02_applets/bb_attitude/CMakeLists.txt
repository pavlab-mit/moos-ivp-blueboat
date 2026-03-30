#--------------------------------------------------------
# The CMakeLists.txt for:              bb_attitude
# BlueBoat Attitude Measurement Applet
# Author(s):                              
#--------------------------------------------------------

if(ARMA_AVAIL)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

  SET(SRC
   main.cpp
  )

  ADD_EXECUTABLE(bb_attitude ${SRC})

  add_dependencies(bb_attitude
  navigator_lib
  )

  target_include_directories(bb_attitude PRIVATE
   ${NAVIGATOR_INCLUDE}
   ${ARMADILLO_INCLUDE_DIR}
   ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_MadgwickAHRS
   )

  TARGET_LINK_LIBRARIES(bb_attitude
   ${ARMA_LIBS}
   ${MOOS_LIBRARIES}
   madgwicks
   apputil
   mbutil
   m
   pthread
   ${NAVIGATOR_LIB})

else()
    message(WARNING "Armadillo dependencies not found - Not Building <bb_attitude>")
endif()
