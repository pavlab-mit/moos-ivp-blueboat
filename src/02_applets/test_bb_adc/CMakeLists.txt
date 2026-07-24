#--------------------------------------------------------
# The CMakeLists.txt for:                     test_bb_adc
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(test_bb_adc ${SRC})

TARGET_INCLUDE_DIRECTORIES(test_bb_adc PRIVATE "${NAVIGATOR_CPP_INCLUDE}")

TARGET_LINK_LIBRARIES(test_bb_adc
   ${SYSTEM_LIBS}
   ${NAVIGATOR_CPP_LIB})
