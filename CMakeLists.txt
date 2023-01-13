cmake_minimum_required(VERSION 3.9)
project(FuzzyControls VERSION 1.0.0)


add_library(
    pid
    include/simplePID.hpp
    src/simplePID.cpp
)

target_include_directories(pid PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_subdirectory(src)