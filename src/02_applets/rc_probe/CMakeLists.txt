#--------------------------------------------------------
# The CMakeLists.txt for:                        rc_probe
# RC link diagnostic utility, SBUS + CRSF (no MOOS required)
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rc_probe
    main.cpp
)

target_link_libraries(rc_probe
    crsf
    sbus
    pthread
)

target_include_directories(rc_probe PRIVATE
    ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_crsf
    ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_sbus
)
