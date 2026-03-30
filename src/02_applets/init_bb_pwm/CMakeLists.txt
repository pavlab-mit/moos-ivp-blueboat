#--------------------------------------------------------
# The CMakeLists.txt for:                        cal_fins
# Author(s):                              Raymond Turrisi 
#--------------------------------------------------------


SET(SRC
   main.cpp  
)  

ADD_EXECUTABLE(init_bb_pwm ${SRC})

add_dependencies(init_bb_pwm navigator_lib)

TARGET_INCLUDE_DIRECTORIES(init_bb_pwm PRIVATE "${NAVIGATOR_INCLUDE}")

TARGET_LINK_LIBRARIES(init_bb_pwm
   ${SYSTEM_LIBS}
   ${NAVIGATOR_LIB})

