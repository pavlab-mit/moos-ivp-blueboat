#--------------------------------------------------------
# The CMakeLists.txt for:            iBBNavigatorInterface
# Author(s):    Raymond Turrisi (orig.), Jeremy Wenger
#--------------------------------------------------------
# Built against navigator-cpp (libnavigator_cpp.so +
# nav_bindings.h). Hardware revision (Navigator V1/V2) and
# Pi model (4/5) are detected at runtime, so there is one
# build for the whole fleet. On dev machines navigator-cpp
# provides platform stubs, so this target links everywhere
# the library is found - no vehicle-only gating.
#--------------------------------------------------------

SET(SRC
  BBNavigatorInterface.cpp
  NavActuator.cpp
  NavSensors.cpp
  BBNavigatorInterface_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iBBNavigatorInterface ${SRC})

TARGET_INCLUDE_DIRECTORIES(iBBNavigatorInterface PRIVATE
  "${NAVIGATOR_CPP_INCLUDE}"
)

TARGET_LINK_LIBRARIES(iBBNavigatorInterface
   bb_command
  ${MOOS_LIBRARIES}
  ${SYSTEM_LIBS}
  ${NAVIGATOR_CPP_LIB}
  apputil
  mbutil
  m
  pthread)

# libatomic: armv7l lacks native 64-bit atomic ops, so
# std::atomic<double> on m_latest_set_thrust_* lowers to
# __atomic_load_8 / __atomic_store_8 helper calls in libatomic.
# macOS has no separate libatomic (builtins are in the compiler rt).
IF(NOT APPLE)
  TARGET_LINK_LIBRARIES(iBBNavigatorInterface atomic)
ENDIF()
