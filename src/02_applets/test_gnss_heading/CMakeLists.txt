#--------------------------------------------------------
# The CMakeLists.txt for:              test_gnss_heading
# BlueBoat GNSS Heading Test Utility (UM982 only)
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_gnss_heading
    main.cpp
)

target_link_libraries(test_gnss_heading
    unicore
    pthread
)

target_include_directories(test_gnss_heading PRIVATE
    ${CMAKE_SOURCE_DIR}/src/01_libraries/lib_unicore
)
