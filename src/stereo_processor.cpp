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

    stereo_matcher_ = cv::StereoSGBM::create(0, 16, 3);

    try {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("orin_rp2_csi");
        std::string calib_file = "stereo_calib.yml";  // Adjust filename as needed
        if (!load_calibration(pkg_path + "/data/" + calib_file)) {
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

    // Load calibration parameters, e.g., stereo_matcher_->setMinDisparity(fs["minDisparity"]);
    // Adjust based on your calibration file structure

    return true;
}
