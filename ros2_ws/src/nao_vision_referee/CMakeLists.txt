cmake_minimum_required(VERSION 3.16)
project(nao_vision_referee CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)

# vision_standalone isn't a ROS package - its sources are compiled straight
# into this node rather than duplicated, so there's one copy of the pipeline.
set(VISION_STANDALONE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../vision_standalone")
set(DEFAULT_BOTTLE_VIDEO_PATH "${VISION_STANDALONE_DIR}/data/bottle_examples/1000275431_nao.mp4")
set(DEFAULT_FACE_VIDEO_PATH "${VISION_STANDALONE_DIR}/data/faces_examples/Man_surprised_nao.mp4")
set(FACE_MODEL_PATH "${VISION_STANDALONE_DIR}/data/face_detection_yunet_2023mar.onnx")

add_executable(referee_node
    src/referee_node.cpp
    ${VISION_STANDALONE_DIR}/src/pre_game.cpp
    ${VISION_STANDALONE_DIR}/src/vision_common.cpp
    ${VISION_STANDALONE_DIR}/src/line_projection.cpp
    ${VISION_STANDALONE_DIR}/src/world_coordinates.cpp
    ${VISION_STANDALONE_DIR}/src/face_detection.cpp
)

target_include_directories(referee_node PRIVATE ${VISION_STANDALONE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(referee_node PRIVATE
    DEFAULT_BOTTLE_VIDEO_PATH="${DEFAULT_BOTTLE_VIDEO_PATH}"
    DEFAULT_FACE_VIDEO_PATH="${DEFAULT_FACE_VIDEO_PATH}"
    FACE_MODEL_PATH="${FACE_MODEL_PATH}"
)
# Plain (non-keyword) form: ament_target_dependencies uses the plain form
# internally, and CMake disallows mixing keyword/plain target_link_libraries
# calls on the same target.
target_link_libraries(referee_node ${OpenCV_LIBS})
ament_target_dependencies(referee_node rclcpp trajectory_msgs)

install(TARGETS referee_node DESTINATION lib/${PROJECT_NAME})

ament_package()