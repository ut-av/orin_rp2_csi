#pragma once

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

class MonoProcessor : public rclcpp::Node
{
public:
  MonoProcessor();
  void init();

private:
  void capture_and_process();
  bool load_calibration(const std::string& filename);
  void process_mono();
  void print_help();

  cv::VideoCapture cap_;
  cv::Mat image_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  image_transport::Publisher pub_image_raw_;
  image_transport::Publisher pub_image_rect_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string display_mode_;
  std::string image_format_;
};