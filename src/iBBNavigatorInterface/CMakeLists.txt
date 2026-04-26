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
    madgwicks
    apputil
    mbutil
    m
    pthread
    # libatomic: armv7l lacks native 64-bit atomic ops, so
    # std::atomic<double> on m_latest_set_thrust_* lowers to
    # __atomic_load_8 / __atomic_store_8 helper calls in libatomic.
    # x86_64 / aarch64 builds also accept this link with no harm.
    atomic)
else()
  message(WARNING "Armadillo dependencies not found - Not Building <iBBNavigatorInterface>")
endif()

