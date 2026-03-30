#--------------------------------------------------------
# The CMakeLists.txt for:                  iBBNavigatorInterface
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

if(ARMA_AVAIL)
  SET(SRC
    BBNavigatorInterface.cpp
    BBNavigatorInterface_Info.cpp
    main.cpp
  )

  ADD_EXECUTABLE(iBBNavigatorInterface ${SRC})

  add_dependencies(iBBNavigatorInterface navigator_lib)

  add_definitions(-DARMA_DONT_USE_WRAPPER)

  TARGET_INCLUDE_DIRECTORIES(iBBNavigatorInterface PRIVATE
    "${NAVIGATOR_INCLUDE}"
    ${ARMADILLO_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_MadgwickAHRS
  )

  TARGET_LINK_LIBRARIES(iBBNavigatorInterface
    ${MOOS_LIBRARIES}
    ${SYSTEM_LIBS}
    ${NAVIGATOR_LIB}
    ${ARMA_LIBS}
    MadgwickAHRS
    rpi_utils
    apputil
    mbutil
    m
    pthread)
else()
  message(WARNING "Armadillo dependencies not found - Not Building <iBBNavigatorInterface>")
endif()

