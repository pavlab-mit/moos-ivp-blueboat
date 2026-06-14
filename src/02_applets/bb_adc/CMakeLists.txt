#--------------------------------------------------------
# The CMakeLists.txt for:                          bb_adc
# BlueBoat battery-voltage reader (NAVOS v1 / v2 dual-target)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(bb_adc ${SRC})

add_dependencies(bb_adc navigator_lib)

# NAVOS v1 / v2 selection. The ADC bulk-read API and the required
# version prelude differ between 0.0.6 and 0.1.2; main.cpp gates on
# these defines. Mirrors init_bb_pwm.
if(NAVIGATOR_LIB_VERSION STREQUAL "2"
   OR NAVOS_VERSION STREQUAL "2")
  target_compile_definitions(bb_adc PRIVATE IBBNAV_NAVOS_V2=1)
  if(RASPBERRY_PI_VERSION STREQUAL "5")
    target_compile_definitions(bb_adc PRIVATE IBBNAV_RASPBERRY_PI5=1)
  endif()
endif()

TARGET_INCLUDE_DIRECTORIES(bb_adc PRIVATE "${NAVIGATOR_INCLUDE}")

TARGET_LINK_LIBRARIES(bb_adc
   ${SYSTEM_LIBS}
   ${NAVIGATOR_LIB})
