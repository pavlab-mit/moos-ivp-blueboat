#--------------------------------------------------------
# The CMakeLists.txt for:                      sbus_probe
# SBUS / RC-link diagnostic utility (no MOOS required)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(sbus_probe
    main.cpp
)

target_link_libraries(sbus_probe
    sbus
    pthread
)

target_include_directories(sbus_probe PRIVATE
    ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_sbus
)
