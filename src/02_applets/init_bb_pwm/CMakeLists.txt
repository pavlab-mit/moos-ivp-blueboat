#--------------------------------------------------------
# The CMakeLists.txt for:                       init_bb_pwm
# BlueBoat ESC arming utility (navigator-cpp)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(init_bb_pwm ${SRC})

TARGET_INCLUDE_DIRECTORIES(init_bb_pwm PRIVATE "${NAVIGATOR_CPP_INCLUDE}")

TARGET_LINK_LIBRARIES(init_bb_pwm
   ${SYSTEM_LIBS}
   ${NAVIGATOR_CPP_LIB})
