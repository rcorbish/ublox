cmake_minimum_required (VERSION 3.4)

project (ublox LANGUAGES CXX)

find_package (Threads)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}  )

add_compile_options( -std=c++14 )
add_compile_options( -g )

subdirs(src)