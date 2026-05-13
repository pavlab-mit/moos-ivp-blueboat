#--------------------------------------------------------
# The CMakeLists.txt for:                       init_bb_pwm
# BlueBoat ESC arming utility (navigator-lib 0.1.2)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(init_bb_pwm ${SRC})

add_dependencies(init_bb_pwm navigator_lib)

if(RASPBERRY_PI_VERSION STREQUAL "5")
  target_compile_definitions(init_bb_pwm PRIVATE IBBNAV_RASPBERRY_PI5=1)
endif()

TARGET_INCLUDE_DIRECTORIES(init_bb_pwm PRIVATE "${NAVIGATOR_INCLUDE}")

TARGET_LINK_LIBRARIES(init_bb_pwm
   ${SYSTEM_LIBS}
   ${NAVIGATOR_LIB})
