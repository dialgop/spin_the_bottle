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