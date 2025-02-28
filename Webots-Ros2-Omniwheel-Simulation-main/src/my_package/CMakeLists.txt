cmake_minimum_required(VERSION 3.5)
project(my_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)

# MyRobotDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/MyRobotDriver_O1.cpp
  src/MyRobotDriver_O2.cpp
  src/MyRobotDriver_O3.cpp
  src/MyRobotDriver_O4.cpp
  src/MyRobotDriver_O5.cpp

  src/MyRobotDriver_B1.cpp
  src/MyRobotDriver_B2.cpp
  src/MyRobotDriver_B3.cpp
  src/MyRobotDriver_B4.cpp
  src/MyRobotDriver_B5.cpp
  src/ball_driver.cpp

)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
