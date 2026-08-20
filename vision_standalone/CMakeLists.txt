cmake_minimum_required(VERSION 3.16)
project(vision_standalone CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4 REQUIRED)

set(FACE_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data/face_detection_yunet_2023mar.onnx")
set(FACE_EXAMPLES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data/faces_examples")
set(BOTTLE_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data/bottle_detector_model.onnx")
set(BOTTLE_EXAMPLES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data/bottle_examples")

add_executable(vision_standalone
    src/main.cpp
    src/pre_game.cpp
    src/vision_common.cpp
    src/line_projection.cpp
    src/world_coordinates.cpp
    src/face_detection.cpp
    src/bottle_detector.cpp
)

target_include_directories(vision_standalone PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_standalone PRIVATE ${OpenCV_LIBS})
target_compile_definitions(vision_standalone PRIVATE
    FACE_MODEL_PATH="${FACE_MODEL_PATH}"
    BOTTLE_MODEL_PATH="${BOTTLE_MODEL_PATH}"
)

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

add_executable(pre_game_tests
    tests/pre_game_tests.cpp
    src/pre_game.cpp
    src/vision_common.cpp
)

target_include_directories(pre_game_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pre_game_tests PRIVATE ${OpenCV_LIBS})

add_test(NAME pre_game_tests COMMAND pre_game_tests)

add_executable(face_detection_tests
    tests/face_detection_tests.cpp
    src/face_detection.cpp
)

target_include_directories(face_detection_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(face_detection_tests PRIVATE ${OpenCV_LIBS})
target_compile_definitions(face_detection_tests PRIVATE
    FACE_MODEL_PATH="${FACE_MODEL_PATH}"
    FACE_EXAMPLES_DIR="${FACE_EXAMPLES_DIR}"
)

add_test(NAME face_detection_tests COMMAND face_detection_tests)

add_executable(bottle_detector_tests
    tests/bottle_detector_tests.cpp
    src/bottle_detector.cpp
)

target_include_directories(bottle_detector_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bottle_detector_tests PRIVATE ${OpenCV_LIBS})
target_compile_definitions(bottle_detector_tests PRIVATE
    BOTTLE_MODEL_PATH="${BOTTLE_MODEL_PATH}"
    BOTTLE_EXAMPLES_DIR="${BOTTLE_EXAMPLES_DIR}"
)

add_test(NAME bottle_detector_tests COMMAND bottle_detector_tests)