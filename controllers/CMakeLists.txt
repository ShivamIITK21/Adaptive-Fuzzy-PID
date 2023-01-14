add_library(
    controllers
    include/controllers.hpp
    include/simplePID.hpp
    src/simplePID.cpp
)

target_include_directories(controllers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")