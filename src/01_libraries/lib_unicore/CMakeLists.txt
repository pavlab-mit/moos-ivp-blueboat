cmake_minimum_required(VERSION 3.10)
project(lib_unicore)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(unicore STATIC
    UnicoreParser.cpp
)

target_include_directories(unicore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
