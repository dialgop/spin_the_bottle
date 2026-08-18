cmake_minimum_required(VERSION 3.16)
project(nao_video_display CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/image_display_plugin.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} rclcpp webots_ros2_driver pluginlib sensor_msgs cv_bridge)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

pluginlib_export_plugin_description_file(webots_ros2_driver nao_video_display_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
