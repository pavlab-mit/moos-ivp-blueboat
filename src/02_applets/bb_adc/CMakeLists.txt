#--------------------------------------------------------
# The CMakeLists.txt for:                          bb_adc
# BlueBoat battery-voltage reader (navigator-cpp)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(bb_adc ${SRC})

TARGET_INCLUDE_DIRECTORIES(bb_adc PRIVATE "${NAVIGATOR_CPP_INCLUDE}")

TARGET_LINK_LIBRARIES(bb_adc
   ${SYSTEM_LIBS}
   ${NAVIGATOR_CPP_LIB})
