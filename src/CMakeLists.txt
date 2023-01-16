set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
add_executable(main main.cpp)
target_link_libraries(main PUBLIC controllers)
target_link_libraries(main PUBLIC auv)