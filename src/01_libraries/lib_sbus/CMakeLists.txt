# --------------------------------------------------------
# The CMakeLists.txt for: lib_sbus
# Author(s): Raymond Turrisi
# --------------------------------------------------------

set(SRC sbus_handler.cpp)

set(HEADERS sbus_handler.h)

# Build Library
add_library(sbus ${SRC})
