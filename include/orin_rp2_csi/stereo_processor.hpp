#pragma once

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.hpp>
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
  void populate_camera_info();

  cv::VideoCapture left_cap_;
  cv::VideoCapture right_cap_;
  image_transport::Publisher pub_disparity_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_camera_info_;
  cv::Ptr<cv::StereoSGBM> stereo_matcher_;
  cv::Mat left_image_;
  cv::Mat right_image_;
  cv::Mat left_camera_matrix_;
  cv::Mat left_dist_coeffs_;
  cv::Mat right_camera_matrix_;
  cv::Mat right_dist_coeffs_;
  sensor_msgs::msg::CameraInfo left_camera_info_msg_;
  sensor_msgs::msg::CameraInfo right_camera_info_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string display_mode_;
};