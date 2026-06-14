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

  # NAVOS v1 / v2 selection. When building against navigator-lib 0.1.2 the
  # applet must call set_navigator_version()/set_raspberry_pi_version() before
  # init(); these defines gate that prelude in main.cpp. Mirrors init_bb_pwm.
  if(NAVIGATOR_LIB_VERSION STREQUAL "2"
     OR NAVOS_VERSION STREQUAL "2")
    target_compile_definitions(bb_attitude PRIVATE IBBNAV_NAVOS_V2=1)
    if(RASPBERRY_PI_VERSION STREQUAL "5")
      target_compile_definitions(bb_attitude PRIVATE IBBNAV_RASPBERRY_PI5=1)
    endif()
  endif()

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
