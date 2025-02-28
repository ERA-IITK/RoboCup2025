cmake_minimum_required(VERSION 3.5)
project(controller_package)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(ACADO REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${ACADO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(controller src/controller.cpp)
# add_executable(controller src/tester.cpp)

target_link_libraries(controller
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ACADO_SHARED_LIBRARIES}
  ${Boost_LIBRARIES}
  Eigen3::Eigen
)

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  controller
  
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "webots_ros2_driver"
  "pluginlib"
)

install(TARGETS
  controller
  
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

