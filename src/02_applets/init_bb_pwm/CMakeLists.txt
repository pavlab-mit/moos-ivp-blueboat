#--------------------------------------------------------
# The CMakeLists.txt for:                       init_bb_pwm
# BlueBoat ESC arming utility (NAVOS v1 / v2 dual-target)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(init_bb_pwm ${SRC})

add_dependencies(init_bb_pwm navigator_lib)

# Adapt the variable name on the right-hand side to whatever your
# top-level CMakeLists.txt uses to select navigator-lib version.
# Common names in this tree: NAVIGATOR_LIB_VERSION, NAVOS_VERSION.
if(NAVIGATOR_LIB_VERSION STREQUAL "2"
   OR NAVOS_VERSION STREQUAL "2")
  target_compile_definitions(init_bb_pwm PRIVATE IBBNAV_NAVOS_V2=1)
  if(RASPBERRY_PI_VERSION STREQUAL "5")
    target_compile_definitions(init_bb_pwm PRIVATE IBBNAV_RASPBERRY_PI5=1)
  endif()
endif()

TARGET_INCLUDE_DIRECTORIES(init_bb_pwm PRIVATE "${NAVIGATOR_INCLUDE}")

TARGET_LINK_LIBRARIES(init_bb_pwm
   ${SYSTEM_LIBS}
   ${NAVIGATOR_LIB})
