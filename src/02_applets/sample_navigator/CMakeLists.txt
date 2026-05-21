#--------------------------------------------------------
# The CMakeLists.txt for:              sample_navigator
# Navigator Sensor Sampling Applet
#--------------------------------------------------------

if(ARMA_AVAIL)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

  SET(SRC
   main.cpp
  )

  ADD_EXECUTABLE(sample_navigator ${SRC})

  add_dependencies(sample_navigator
  navigator_lib
  )

  target_include_directories(sample_navigator PRIVATE
   ${NAVIGATOR_INCLUDE}
   ${ARMADILLO_INCLUDE_DIR}
   ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_MadgwickAHRS
   ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_filters
   ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_nb_utils
   )

  TARGET_LINK_LIBRARIES(sample_navigator
   ${ARMA_LIBS}
   ${MOOS_LIBRARIES}
   madgwicks
   apputil
   mbutil
   m
   pthread
   ${NAVIGATOR_LIB})

else()
    message(WARNING "Armadillo dependencies not found - Not Building <sample_navigator>")
endif()
