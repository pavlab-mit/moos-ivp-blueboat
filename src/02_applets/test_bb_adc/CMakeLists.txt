#--------------------------------------------------------
# The CMakeLists.txt for:                     test_bb_adc
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------


SET(SRC
   main.cpp
)

ADD_EXECUTABLE(test_bb_adc ${SRC})

add_dependencies(test_bb_adc navigator_lib)

TARGET_INCLUDE_DIRECTORIES(test_bb_adc PRIVATE "${NAVIGATOR_INCLUDE}")

TARGET_LINK_LIBRARIES(test_bb_adc
   ${SYSTEM_LIBS}
   ${NAVIGATOR_LIB})
