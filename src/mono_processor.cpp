#include "orin_rp2_csi/mono_processor.hpp"

MonoProcessor::MonoProcessor() : Node("mono_processor")
{
    image_transport::ImageTransport it(shared_from_this());

    this->declare_parameter<std::string>("sensor_id", "0");
    std::string sensor_id = this->get_parameter("sensor_id").as_string();

    std::string pipeline = "nvarguscamerasrc sensor-id=" + sensor_id + " ! video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";

    cap_.open(pipeline, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
    }

    pub_image_ = it.advertise("image", 1);

    try {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("orin_rp2_csi");
        std::string calib_file = "mono_calib.yml";  // Adjust filename as needed
        if (!load_calibration(pkg_path + "/data/" + calib_file)) {
            RCLCPP_WARN(this->get_logger(), "Calibration file not found, proceeding without rectification");
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error loading calibration: %s", e.what());
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&MonoProcessor::capture_and_process, this));
}

void MonoProcessor::capture_and_process()
{
    cap_ >> image_;

    if (!image_.empty()) {
        process_mono();
    }
}

bool MonoProcessor::load_calibration(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();
    return true;
}

void MonoProcessor::process_mono()
{
    cv::Mat rectified;
    if (!camera_matrix_.empty() && !dist_coeffs_.empty()) {
        cv::undistort(image_, rectified, camera_matrix_, dist_coeffs_);
    } else {
        rectified = image_.clone();
    }
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified).toImageMsg();
    pub_image_.publish(msg);
}