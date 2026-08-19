cmake_minimum_required(VERSION 3.16)
project(nao_vision_referee CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)

# vision_standalone isn't a ROS package - its sources are compiled straight
# into this node rather than duplicated, so there's one copy of the pipeline.
set(VISION_STANDALONE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../vision_standalone")
set(BOTTLE_EXAMPLES_DIR "${VISION_STANDALONE_DIR}/data/bottle_examples")
set(FACE_EXAMPLES_DIR "${VISION_STANDALONE_DIR}/data/faces_examples")
set(FACE_MODEL_PATH "${VISION_STANDALONE_DIR}/data/face_detection_yunet_2023mar.onnx")

# robot_gesture (arm-pose math) and settle_watcher (real-footage decisions)
# are shared between referee_node and referee_logic_tests.
set(REFEREE_LOGIC_SOURCES
    src/robot_gesture.cpp
    src/settle_watcher.cpp
    ${VISION_STANDALONE_DIR}/src/pre_game.cpp
    ${VISION_STANDALONE_DIR}/src/vision_common.cpp
    ${VISION_STANDALONE_DIR}/src/line_projection.cpp
    ${VISION_STANDALONE_DIR}/src/world_coordinates.cpp
    ${VISION_STANDALONE_DIR}/src/face_detection.cpp
)
set(REFEREE_LOGIC_INCLUDE_DIRS include ${VISION_STANDALONE_DIR}/include ${OpenCV_INCLUDE_DIRS})
set(REFEREE_LOGIC_COMPILE_DEFINITIONS
    BOTTLE_EXAMPLES_DIR="${BOTTLE_EXAMPLES_DIR}"
    FACE_EXAMPLES_DIR="${FACE_EXAMPLES_DIR}"
    FACE_MODEL_PATH="${FACE_MODEL_PATH}"
)

add_executable(referee_node
    src/referee_node.cpp
    ${REFEREE_LOGIC_SOURCES}
)

target_include_directories(referee_node PRIVATE ${REFEREE_LOGIC_INCLUDE_DIRS})
target_compile_definitions(referee_node PRIVATE ${REFEREE_LOGIC_COMPILE_DEFINITIONS})
# Plain (non-keyword) form: ament_target_dependencies uses the plain form
# internally, and CMake disallows mixing keyword/plain target_link_libraries
# calls on the same target.
target_link_libraries(referee_node ${OpenCV_LIBS})
ament_target_dependencies(referee_node rclcpp trajectory_msgs sensor_msgs std_msgs cv_bridge)

install(TARGETS referee_node DESTINATION lib/${PROJECT_NAME})

enable_testing()

add_executable(referee_logic_tests
    tests/referee_logic_tests.cpp
    ${REFEREE_LOGIC_SOURCES}
)

target_include_directories(referee_logic_tests PRIVATE ${REFEREE_LOGIC_INCLUDE_DIRS})
target_compile_definitions(referee_logic_tests PRIVATE ${REFEREE_LOGIC_COMPILE_DEFINITIONS})
target_link_libraries(referee_logic_tests ${OpenCV_LIBS})
ament_target_dependencies(referee_logic_tests rclcpp)

add_test(NAME referee_logic_tests COMMAND referee_logic_tests)

ament_package()
