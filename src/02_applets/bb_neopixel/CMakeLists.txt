#--------------------------------------------------------
# The CMakeLists.txt for:                       bb_neopixel
# BlueBoat NeoPixel status utility (navigator-cpp)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(bb_neopixel ${SRC})

TARGET_INCLUDE_DIRECTORIES(bb_neopixel PRIVATE "${NAVIGATOR_CPP_INCLUDE}")

TARGET_LINK_LIBRARIES(bb_neopixel
   ${SYSTEM_LIBS}
   ${NAVIGATOR_CPP_LIB})
