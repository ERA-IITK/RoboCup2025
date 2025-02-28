#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_ball_driver
{
    class MyRobotDriver : public webots_ros2_driver::PluginInterface
    {
    public:
        void step() override;
        void init(webots_ros2_driver::WebotsNode *node,
                  std::unordered_map<std::string, std::string> &parameters) override;

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
        geometry_msgs::msg::Twist cmd_vel_msg;


        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ball_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        WbDeviceTag gps;
        
    };
} // namespace my_robot_driver
#endif
