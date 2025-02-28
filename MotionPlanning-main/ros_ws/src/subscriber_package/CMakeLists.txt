cmake_minimum_required(VERSION 3.5)
project(subscriber_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find additional dependencies
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

# Add the talker executable
# add_executable(main src/main.cpp src/Visualize.cpp src/Motion.cpp src/Robot.cpp src/Coordinate.hpp)
add_executable(main src/main.cpp src/Visualize.cpp)

# Include directories for additional dependencies
include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

ament_target_dependencies(main rclcpp geometry_msgs)

# Link against additional libraries
target_link_libraries(main
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Install the talker executable
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
