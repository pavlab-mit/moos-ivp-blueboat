#--------------------------------------------------------
# The CMakeLists.txt for:                       pBB_EKF
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

if(ARMA_AVAIL)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  
  SET(SRC
    BB_EKF.cpp
    BB_EKF_Info.cpp
    main.cpp
  )
  
  ADD_EXECUTABLE(pBB_EKF ${SRC})
  
  add_definitions(-DARMA_DONT_USE_WRAPPER)
  
  target_include_directories(pBB_EKF PRIVATE 
    ${ARMADILLO_INCLUDE_DIR}
  )
  
  TARGET_LINK_LIBRARIES(pBB_EKF
    ${ARMA_LIBS}
    ${MOOS_LIBRARIES}
    ${MOOSGeodesy_LIBRARIES}
    apputil
    mbutil
    m
    pthread)
    
else()
  message(WARNING "Armadillo dependencies not found - Not Building <pBB_EKF>")
endif()

