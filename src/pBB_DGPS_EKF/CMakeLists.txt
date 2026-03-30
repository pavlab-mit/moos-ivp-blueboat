#--------------------------------------------------------
# The CMakeLists.txt for:                       pBB_DGPS_EKF
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

if(ARMA_AVAIL)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

  SET(SRC
    BB_DGPS_EKF.cpp
    BB_DGPS_EKF_Info.cpp
    main.cpp
  )

  ADD_EXECUTABLE(pBB_DGPS_EKF ${SRC})

  add_definitions(-DARMA_DONT_USE_WRAPPER)

  target_include_directories(pBB_DGPS_EKF PRIVATE
    ${ARMADILLO_INCLUDE_DIR}
  )

  TARGET_LINK_LIBRARIES(pBB_DGPS_EKF
    ${ARMA_LIBS}
    ${MOOS_LIBRARIES}
    MOOSGeodesy
    apputil
    geometry
    mbutil
    m
    pthread)

else()
  message(WARNING "Armadillo dependencies not found - Not Building <pBB_DGPS_EKF>")
endif()
