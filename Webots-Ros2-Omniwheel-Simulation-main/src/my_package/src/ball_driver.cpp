#include "my_package/ball_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>

#define SAMPLING_PERIOD 25 // GPS sampling time in ms

namespace my_ball_driver {

  void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

    gps = wb_robot_get_device("GPS");
    wb_gps_enable(gps, SAMPLING_PERIOD);

    // Subscribe to cmd_vel to receive velocity commands
    cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/ball/cmd_vel", rclcpp::SensorDataQoS().reliable(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          this->cmd_vel_msg.linear = msg->linear;
          this->cmd_vel_msg.angular = msg->angular;
        }
    );

    ball_publisher_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("ball_data", 10);

    timer_ = node->create_wall_timer(std::chrono::milliseconds(SAMPLING_PERIOD), [this]() { // Timer for periodic publishing
      wb_robot_step(SAMPLING_PERIOD);                       // Step simulation in Webots
      const double *gps_position = wb_gps_get_values(gps);  // Retrieve GPS data

      auto msg = std_msgs::msg::Float32MultiArray();        // Fill ROS2 message
      msg.data = {static_cast<float>(gps_position[0]), static_cast<float>(gps_position[1])};
      ball_publisher_->publish(msg);                         // Publish GPS data
    });
  }

  void MyRobotDriver::step() {
  }
} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_ball_driver::MyRobotDriver, webots_ros2_driver::PluginInterface)

