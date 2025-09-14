#pragma once

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

class StereoProcessor : public rclcpp::Node
{
public:
  StereoProcessor();

private:
  void capture_and_process();
  void process_stereo();
  bool load_calibration(const std::string& filename);

  cv::VideoCapture left_cap_;
  cv::VideoCapture right_cap_;
  image_transport::Publisher pub_disparity_;
  cv::Ptr<cv::StereoSGBM> stereo_matcher_;
  cv::Mat left_image_;
  cv::Mat right_image_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string display_mode_;
};