#--------------------------------------------------------
# The CMakeLists.txt for:                 sample_navigator
# Navigator sensor sampler / cal data collector (navigator-cpp)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
   main.cpp
)

ADD_EXECUTABLE(sample_navigator ${SRC})

TARGET_INCLUDE_DIRECTORIES(sample_navigator PRIVATE "${NAVIGATOR_CPP_INCLUDE}")

TARGET_LINK_LIBRARIES(sample_navigator
   ${SYSTEM_LIBS}
   ${NAVIGATOR_CPP_LIB})
