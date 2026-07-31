cmake_minimum_required(VERSION 3.16)
project(vision_standalone CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4 REQUIRED)

add_executable(vision_standalone
    src/main.cpp
    src/pre_game.cpp
    src/vision_common.cpp
    src/line_projection.cpp
    src/world_coordinates.cpp
)

target_include_directories(vision_standalone PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_standalone PRIVATE ${OpenCV_LIBS})

enable_testing()

add_executable(world_coordinates_tests
    tests/world_coordinates_tests.cpp
    src/world_coordinates.cpp
)

target_include_directories(world_coordinates_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(world_coordinates_tests PRIVATE ${OpenCV_LIBS})

add_test(NAME world_coordinates_tests COMMAND world_coordinates_tests)

add_executable(line_projection_tests
    tests/line_projection_tests.cpp
    src/line_projection.cpp
    src/vision_common.cpp
)

target_include_directories(line_projection_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(line_projection_tests PRIVATE ${OpenCV_LIBS})

add_test(NAME line_projection_tests COMMAND line_projection_tests)