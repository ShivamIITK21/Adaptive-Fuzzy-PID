set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(FUZZYLITE_DIR "/home/shivam/fuzzylite/fuzzylite")

include_directories(${FUZZYLITE_DIR})

link_directories("${FUZZYLITE_DIR}/release/bin")



add_executable(main main.cpp)
target_link_libraries(main fuzzylite-static)
target_link_libraries(main controllers)
target_link_libraries(main auv)

target_compile_options(main PRIVATE --std=c++11)