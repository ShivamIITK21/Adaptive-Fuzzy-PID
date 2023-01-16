add_library(
    auv
    include/auv.hpp
    src/auv.cpp
)
target_include_directories(auv PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(auv PUBLIC controllers)