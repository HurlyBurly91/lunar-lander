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
    src/ballistic.cpp
    src/flight_computer.cpp
    src/autopilot.cpp
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

# M05: the deterministic two-body binary-system ephemeris, isolated from the
# full simulation and SDL, verified analytically.
add_executable(lander_binary_tests
    tests/test_binary.cpp
)
target_link_libraries(lander_binary_tests PRIVATE lander)

add_test(NAME lander_binary_tests COMMAND lander_binary_tests)

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

# M05-R3: reusable triple-tap guarded-action logic, tested headlessly with no
# SDL dependency.
add_executable(lander_guarded_actions_tests
    tests/test_guarded_actions.cpp
)
target_include_directories(lander_guarded_actions_tests PRIVATE include)
target_compile_options(lander_guarded_actions_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_test(NAME lander_guarded_actions_tests
         COMMAND lander_guarded_actions_tests)

# M05-R3: pure presentation geometry for the body/pad annulus renderer,
# tested headlessly with no SDL dependency.
add_executable(lander_render_geom_tests
    tests/test_render_geom.cpp
)
target_link_libraries(lander_render_geom_tests PRIVATE lander)
target_compile_options(lander_render_geom_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_test(NAME lander_render_geom_tests
          COMMAND lander_render_geom_tests)

# M06: the deterministic flight computer (shared ballistic prediction, the
# single maneuver node, planners, attitude controller, node executor), tested
# headlessly with no SDL dependency.
add_executable(lander_flight_computer_tests
    tests/test_flight_computer.cpp
)
target_link_libraries(lander_flight_computer_tests PRIVATE lander)
target_compile_options(lander_flight_computer_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_test(NAME lander_flight_computer_tests
          COMMAND lander_flight_computer_tests)
