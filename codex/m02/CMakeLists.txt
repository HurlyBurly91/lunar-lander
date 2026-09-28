cmake_minimum_required(VERSION 3.20)
project(lunar_lander LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(lander
    src/sim.cpp
)

target_include_directories(lander PUBLIC include)

target_compile_options(lander PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(lander_cli
    src/main.cpp
)

target_link_libraries(lander_cli PRIVATE lander)

enable_testing()

add_executable(lander_tests
    tests/test_sim.cpp
)

target_link_libraries(lander_tests PRIVATE lander)

add_test(NAME lander_tests COMMAND lander_tests)
