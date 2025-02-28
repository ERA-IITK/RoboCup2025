cmake_minimum_required(VERSION 3.5)
project(my_nodes)

if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# my_subscriber_node
add_executable(my_subscriber_node src/my_subscriber_node.cpp)
ament_target_dependencies(my_subscriber_node rclcpp sensor_msgs)

target_include_directories(my_subscriber_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

target_link_libraries(my_subscriber_node
  ${OpenCV_LIBS}
)

# odom_subscriber_node
add_executable(odom_subscriber_node src/odom_subscriber_node.cpp)
ament_target_dependencies(odom_subscriber_node rclcpp nav_msgs)

target_include_directories(odom_subscriber_node PRIVATE
  ${nav_msgs_INCLUDE_DIRS}
)

target_link_libraries(odom_subscriber_node
  ${nav_msgs_LIBRARIES}
)

# point_cloud_transformer_node
add_executable(point_cloud_transformer_node src/point_cloud_transformer_node.cpp)
ament_target_dependencies(point_cloud_transformer_node rclcpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs)

target_include_directories(point_cloud_transformer_node PRIVATE
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_sensor_msgs_INCLUDE_DIRS}
)

target_link_libraries(point_cloud_transformer_node
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_sensor_msgs_LIBRARIES}
)

install(TARGETS
  my_subscriber_node
  odom_subscriber_node
  point_cloud_transformer_node  # Add this line for the transformer node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
