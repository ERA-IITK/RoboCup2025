#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomSubscriberNode : public rclcpp::Node {
public:
  OdomSubscriberNode() : Node("odom_subscriber_node") {
    // Subscribe to the /odom topic
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OdomSubscriberNode::odomCallback, this, std::placeholders::_1));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Callback function for /odom topic
    RCLCPP_INFO(get_logger(), "Received Odometry Message\nPosition: [%f, %f, %f]\nOrientation: [%f, %f, %f, %f]",
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create an instance of the OdomSubscriberNode
  auto node = std::make_shared<OdomSubscriberNode>();

  // Set the desired frequency (e.g., 10 Hz)
  rclcpp::WallRate loop_rate(10);

  // Use rclcpp::ok to check if the node should continue spinning
  while (rclcpp::ok()) {
    // Process any pending messages
    rclcpp::spin_some(node);

    // Your node-specific logic goes here...

    // Sleep to control the loop frequency
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
