cmake_minimum_required(VERSION 3.10)
project(MILSTD110C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(milstd110c
    src/FECEncoder.cpp
    src/Interleaver.cpp
)

target_include_directories(milstd110c PUBLIC include)

enable_testing()
add_subdirectory(tests)
