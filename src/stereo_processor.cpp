#include "orin_rp2_csi/stereo_processor.hpp"

StereoProcessor::StereoProcessor() : Node("stereo_processor")
{
    image_transport::ImageTransport it(shared_from_this());

    std::string left_pipeline = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
    std::string right_pipeline = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";

    left_cap_.open(left_pipeline, cv::CAP_GSTREAMER);
    right_cap_.open(right_pipeline, cv::CAP_GSTREAMER);

    if (!left_cap_.isOpened() || !right_cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open cameras");
        return;
    }

    pub_disparity_ = it.advertise("disparity", 1);
    pub_left_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 1);
    pub_right_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 1);
    RCLCPP_INFO(this->get_logger(), "Publishing camera info to topics: left/camera_info, right/camera_info");

    stereo_matcher_ = cv::StereoSGBM::create(0, 16, 3);

    try {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("orin_rp2_csi");
        std::string calib_file = "stereo_calib.yml";  // Adjust filename as needed
        if (!load_calibration(pkg_path + "/calibration/" + calib_file)) {
            RCLCPP_WARN(this->get_logger(), "Calibration file not found, using default parameters");
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error loading calibration: %s", e.what());
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&StereoProcessor::capture_and_process, this));

    this->declare_parameter<std::string>("display_mode", "none");
    display_mode_ = this->get_parameter("display_mode").as_string();
}

void StereoProcessor::capture_and_process()
{
    left_cap_ >> left_image_;
    right_cap_ >> right_image_;

    if (!left_image_.empty() && !right_image_.empty()) {
        process_stereo();
    }
}

void StereoProcessor::process_stereo()
{
    cv::Mat disparity;
    stereo_matcher_->compute(left_image_, right_image_, disparity);

    // Publish camera info for both cameras
    if (pub_left_camera_info_->get_subscription_count() > 0) {
        left_camera_info_msg_.header.stamp = this->now();
        left_camera_info_msg_.header.frame_id = "left_camera_frame";
        pub_left_camera_info_->publish(left_camera_info_msg_);
    }

    if (pub_right_camera_info_->get_subscription_count() > 0) {
        right_camera_info_msg_.header.stamp = this->now();
        right_camera_info_msg_.header.frame_id = "right_camera_frame";
        pub_right_camera_info_->publish(right_camera_info_msg_);
    }

    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", disparity).toImageMsg();
    pub_disparity_.publish(msg);

    if (display_mode_ == "depth") {
        cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
        cv::imshow("Depth", disparity);
    } else if (display_mode_ == "side_by_side" || display_mode_ == "combined") {
        cv::namedWindow("Stereo", cv::WINDOW_AUTOSIZE);
        cv::Mat combined;
        cv::hconcat(left_image_, right_image_, combined);
        cv::imshow("Stereo", combined);
    }

    if (display_mode_ != "none") {
        cv::waitKey(1);
    }
}

bool StereoProcessor::load_calibration(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }

    // Load camera matrices and distortion coefficients
    fs["left_camera_matrix"] >> left_camera_matrix_;
    fs["left_distortion_coefficients"] >> left_dist_coeffs_;
    fs["right_camera_matrix"] >> right_camera_matrix_;
    fs["right_distortion_coefficients"] >> right_dist_coeffs_;

    fs.release();

    // Check if calibration was loaded successfully
    if (left_camera_matrix_.empty() || right_camera_matrix_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to load camera matrices from calibration file");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded stereo calibration");
    populate_camera_info();
    
    return true;
}

void StereoProcessor::populate_camera_info()
{
    int width = 1920;
    int height = 1080;

    // Populate left camera info
    left_camera_info_msg_.width = width;
    left_camera_info_msg_.height = height;
    left_camera_info_msg_.distortion_model = "plumb_bob";

    // Left camera matrix K
    left_camera_info_msg_.k[0] = left_camera_matrix_.at<double>(0, 0);  // fx
    left_camera_info_msg_.k[1] = left_camera_matrix_.at<double>(0, 1);  // 0
    left_camera_info_msg_.k[2] = left_camera_matrix_.at<double>(0, 2);  // cx
    left_camera_info_msg_.k[3] = left_camera_matrix_.at<double>(1, 0);  // 0
    left_camera_info_msg_.k[4] = left_camera_matrix_.at<double>(1, 1);  // fy
    left_camera_info_msg_.k[5] = left_camera_matrix_.at<double>(1, 2);  // cy
    left_camera_info_msg_.k[6] = left_camera_matrix_.at<double>(2, 0);  // 0
    left_camera_info_msg_.k[7] = left_camera_matrix_.at<double>(2, 1);  // 0
    left_camera_info_msg_.k[8] = left_camera_matrix_.at<double>(2, 2);  // 1

    // Left distortion coefficients
    left_camera_info_msg_.d.clear();
    for (int i = 0; i < left_dist_coeffs_.cols; i++) {
        left_camera_info_msg_.d.push_back(left_dist_coeffs_.at<double>(0, i));
    }

    // Left rectification matrix (identity)
    left_camera_info_msg_.r[0] = 1.0; left_camera_info_msg_.r[1] = 0.0; left_camera_info_msg_.r[2] = 0.0;
    left_camera_info_msg_.r[3] = 0.0; left_camera_info_msg_.r[4] = 1.0; left_camera_info_msg_.r[5] = 0.0;
    left_camera_info_msg_.r[6] = 0.0; left_camera_info_msg_.r[7] = 0.0; left_camera_info_msg_.r[8] = 1.0;

    // Left projection matrix [K|0]
    left_camera_info_msg_.p[0] = left_camera_matrix_.at<double>(0, 0);
    left_camera_info_msg_.p[1] = left_camera_matrix_.at<double>(0, 1);
    left_camera_info_msg_.p[2] = left_camera_matrix_.at<double>(0, 2);
    left_camera_info_msg_.p[3] = 0.0;
    left_camera_info_msg_.p[4] = left_camera_matrix_.at<double>(1, 0);
    left_camera_info_msg_.p[5] = left_camera_matrix_.at<double>(1, 1);
    left_camera_info_msg_.p[6] = left_camera_matrix_.at<double>(1, 2);
    left_camera_info_msg_.p[7] = 0.0;
    left_camera_info_msg_.p[8] = left_camera_matrix_.at<double>(2, 0);
    left_camera_info_msg_.p[9] = left_camera_matrix_.at<double>(2, 1);
    left_camera_info_msg_.p[10] = left_camera_matrix_.at<double>(2, 2);
    left_camera_info_msg_.p[11] = 0.0;

    // Populate right camera info
    right_camera_info_msg_.width = width;
    right_camera_info_msg_.height = height;
    right_camera_info_msg_.distortion_model = "plumb_bob";

    // Right camera matrix K
    right_camera_info_msg_.k[0] = right_camera_matrix_.at<double>(0, 0);  // fx
    right_camera_info_msg_.k[1] = right_camera_matrix_.at<double>(0, 1);  // 0
    right_camera_info_msg_.k[2] = right_camera_matrix_.at<double>(0, 2);  // cx
    right_camera_info_msg_.k[3] = right_camera_matrix_.at<double>(1, 0);  // 0
    right_camera_info_msg_.k[4] = right_camera_matrix_.at<double>(1, 1);  // fy
    right_camera_info_msg_.k[5] = right_camera_matrix_.at<double>(1, 2);  // cy
    right_camera_info_msg_.k[6] = right_camera_matrix_.at<double>(2, 0);  // 0
    right_camera_info_msg_.k[7] = right_camera_matrix_.at<double>(2, 1);  // 0
    right_camera_info_msg_.k[8] = right_camera_matrix_.at<double>(2, 2);  // 1

    // Right distortion coefficients
    right_camera_info_msg_.d.clear();
    for (int i = 0; i < right_dist_coeffs_.cols; i++) {
        right_camera_info_msg_.d.push_back(right_dist_coeffs_.at<double>(0, i));
    }

    // Right rectification matrix (identity)
    right_camera_info_msg_.r[0] = 1.0; right_camera_info_msg_.r[1] = 0.0; right_camera_info_msg_.r[2] = 0.0;
    right_camera_info_msg_.r[3] = 0.0; right_camera_info_msg_.r[4] = 1.0; right_camera_info_msg_.r[5] = 0.0;
    right_camera_info_msg_.r[6] = 0.0; right_camera_info_msg_.r[7] = 0.0; right_camera_info_msg_.r[8] = 1.0;

    // Right projection matrix [K|0]
    right_camera_info_msg_.p[0] = right_camera_matrix_.at<double>(0, 0);
    right_camera_info_msg_.p[1] = right_camera_matrix_.at<double>(0, 1);
    right_camera_info_msg_.p[2] = right_camera_matrix_.at<double>(0, 2);
    right_camera_info_msg_.p[3] = 0.0;
    right_camera_info_msg_.p[4] = right_camera_matrix_.at<double>(1, 0);
    right_camera_info_msg_.p[5] = right_camera_matrix_.at<double>(1, 1);
    right_camera_info_msg_.p[6] = right_camera_matrix_.at<double>(1, 2);
    right_camera_info_msg_.p[7] = 0.0;
    right_camera_info_msg_.p[8] = right_camera_matrix_.at<double>(2, 0);
    right_camera_info_msg_.p[9] = right_camera_matrix_.at<double>(2, 1);
    right_camera_info_msg_.p[10] = right_camera_matrix_.at<double>(2, 2);
    right_camera_info_msg_.p[11] = 0.0;

    RCLCPP_INFO(this->get_logger(), "Stereo camera info populated with calibration parameters");
}
