#--------------------------------------------------------
# The CMakeLists.txt for:              bb_attitude
# BlueBoat Attitude Measurement Applet (navigator-cpp)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
 main.cpp
)

ADD_EXECUTABLE(bb_attitude ${SRC})

target_include_directories(bb_attitude PRIVATE
 ${NAVIGATOR_CPP_INCLUDE}
 )

TARGET_LINK_LIBRARIES(bb_attitude
 ${SYSTEM_LIBS}
 m
 pthread
 ${NAVIGATOR_CPP_LIB})
