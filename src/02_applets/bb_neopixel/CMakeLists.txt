#--------------------------------------------------------
# The CMakeLists.txt for:                       bb_neopixel
# BlueBoat NeoPixel status utility (NAVOS v1 / v2 dual-target)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(bb_neopixel ${SRC})

add_dependencies(bb_neopixel navigator_lib)

# NAVOS v1 / v2 selection. set_neopixel_rgbw()/set_rgb_led_strip_size()
# exist in both releases, but 0.1.2 needs the version prelude before
# init(); main.cpp gates on these defines.
if(NAVIGATOR_LIB_VERSION STREQUAL "2"
   OR NAVOS_VERSION STREQUAL "2")
  target_compile_definitions(bb_neopixel PRIVATE IBBNAV_NAVOS_V2=1)
  if(RASPBERRY_PI_VERSION STREQUAL "5")
    target_compile_definitions(bb_neopixel PRIVATE IBBNAV_RASPBERRY_PI5=1)
  endif()
endif()

TARGET_INCLUDE_DIRECTORIES(bb_neopixel PRIVATE "${NAVIGATOR_INCLUDE}")

TARGET_LINK_LIBRARIES(bb_neopixel
   ${SYSTEM_LIBS}
   ${NAVIGATOR_LIB})
