cmake_minimum_required(VERSION 3.20)
project(lunar_lander LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# SDL3 is vendored under third_party/SDL (pinned to release-3.4.16) and built
# in-tree as a static library, so the game has no runtime SDL dependency.
# Tests and examples of SDL itself are not needed.
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TESTS OFF CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SDL_HIDAPI OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/SDL)

# The physics core. Independent of rendering, driven by fixed timesteps,
# and covered headlessly by tests/test_sim.cpp.
add_library(lander
    src/sim.cpp
    src/terrain.cpp
)
target_include_directories(lander PUBLIC include)
target_compile_options(lander PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# Stage 1 playable game: SDL window, flat ground, pads, HUD, restart.
add_executable(lander_gui
    src/gui.cpp
)
target_link_libraries(lander_gui PRIVATE lander SDL3::SDL3)
target_compile_options(lander_gui PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

enable_testing()

add_executable(lander_tests
    tests/test_sim.cpp
)
target_link_libraries(lander_tests PRIVATE lander)

add_test(NAME lander_tests COMMAND lander_tests)

# Presentation-only camera logic (M02), tested headlessly and apart from any
# rendering or SDL dependency.
add_executable(lander_camera_tests
    tests/test_camera.cpp
)
target_link_libraries(lander_camera_tests PRIVATE lander)

add_test(NAME lander_camera_tests COMMAND lander_camera_tests)

# Presentation-only starfield logic (M03): deterministic and independent of
# camera translation/zoom, tested headlessly with no SDL dependency.
add_executable(lander_starfield_tests
    tests/test_starfield.cpp
)
target_include_directories(lander_starfield_tests PRIVATE include)
target_compile_options(lander_starfield_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_test(NAME lander_starfield_tests COMMAND lander_starfield_tests)
