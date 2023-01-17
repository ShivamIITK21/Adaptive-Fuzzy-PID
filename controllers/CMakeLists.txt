add_library(
    controllers
    include/controllers.hpp
    include/simplePID.hpp
    include/fuzzyPID.hpp
    src/simplePID.cpp
    src/fuzzyPID.cpp
)

target_include_directories(controllers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")