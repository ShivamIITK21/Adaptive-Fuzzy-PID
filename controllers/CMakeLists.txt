add_library(
    controllers
    include/controllers.hpp
    include/simplePID.hpp
    include/fuzzyPID.hpp
    src/simplePID.cpp
    src/fuzzyPID.cpp
)

# set(FUZZYLITE_DIR "/home/shivam/fuzzylite/fuzzylite")

target_include_directories(controllers PUBLIC ${FUZZYLITE_DIR})
link_directories("${FUZZYLITE_DIR}/release/bin")

target_link_libraries(controllers PUBLIC fuzzylite-static)

target_include_directories(controllers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")