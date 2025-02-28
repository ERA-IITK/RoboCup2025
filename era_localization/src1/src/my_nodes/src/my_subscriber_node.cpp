#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv2/opencv.hpp>

class MySubscriberNode : public rclcpp::Node
{
public:
  MySubscriberNode() : Node("my_subscriber_node")
  {
    // Subscribe to the image_raw topic
    image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",
        10,  // Set the queue size
        std::bind(&MySubscriberNode::imageCallback, this, std::placeholders::_1));

    // Subscribe to the camera_info topic
    camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info",
        10,  // Set the queue size
        std::bind(&MySubscriberNode::cameraInfoCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Process the image data here
    RCLCPP_INFO(this->get_logger(), "Received image message");

    // Save the image as a file (e.g., in PNG format)
    cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<unsigned char *>(msg->data.data()));
    cv::imwrite("/home/noble_pegasus/ros2_ws/images/image.png", image);

    // Print the data type of the image
    RCLCPP_INFO(this->get_logger(), "Image Data Type: %s", msg->encoding.c_str());
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Process the camera info data here
    RCLCPP_INFO(this->get_logger(), "Received camera info message");
    RCLCPP_INFO(this->get_logger(), "Width: %d", msg->width);
    RCLCPP_INFO(this->get_logger(), "Height: %d", msg->height);
    RCLCPP_INFO(this->get_logger(), "Distortion Model: %s", msg->distortion_model.c_str());

    //Access the distortion matrix (D)
  std::array<double, 9> intrinsic_matrix = msg->k;
  RCLCPP_INFO(this->get_logger(), "Intrinsic Camera Matrix (K):");
  for (size_t i = 0; i < intrinsic_matrix.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "K[%zu]: %f", i, intrinsic_matrix[i]);
  }

  // Extract individual elements of the intrinsic matrix
  double fx = intrinsic_matrix[0]; // Focal length in the x-direction
  double fy = intrinsic_matrix[4]; // Focal length in the y-direction
  double cx = intrinsic_matrix[2]; // Principal point x-coordinate
  double cy = intrinsic_matrix[5]; // Principal point y-coordinate

  RCLCPP_INFO(this->get_logger(), "Focal Length (fx): %f", fx);
  RCLCPP_INFO(this->get_logger(), "Focal Length (fy): %f", fy);
  RCLCPP_INFO(this->get_logger(), "Principal Point (cx): %f", cx);
  RCLCPP_INFO(this->get_logger(), "Principal Point (cy): %f", cy);

    // Print other camera info parameters as needed
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create an instance of the MySubscriberNode
  auto node = std::make_shared<MySubscriberNode>();

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
