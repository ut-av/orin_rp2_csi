#include "orin_rp2_csi/mono_processor.hpp"

MonoProcessor::MonoProcessor() : Node("mono_processor")
{
    RCLCPP_INFO(this->get_logger(), "MonoProcessor constructor started");

    // Ensure the parameter is declared as an integer
    this->declare_parameter<int>("sensor_id", 0);
    int sensor_id_int = this->get_parameter("sensor_id").as_int();
    std::string sensor_id = std::to_string(sensor_id_int);

    std::string pipeline = "nvarguscamerasrc sensor-id=" + sensor_id + " ! video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1, format=NV12 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";

    std::cout << "Using pipeline: " << pipeline << std::endl;

    cap_.open(pipeline, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
    }

    try {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("orin_rp2_csi");
        std::string calib_file = "mono_calib.yml";  // Adjust filename as needed
        if (!load_calibration(pkg_path + "/calibration/" + calib_file)) {
            RCLCPP_WARN(this->get_logger(), "Calibration file not found, proceeding without rectification");
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error loading calibration: %s", e.what());
    }

    // Add a parameter for display mode
    this->declare_parameter<std::string>("display_mode", "show");
    display_mode_ = this->get_parameter("display_mode").as_string();

    std::cout << "MonoProcessor constructor completed" << std::endl;
    std::cout << "Display mode: " << display_mode_ << std::endl;
}

void MonoProcessor::init()
{
    RCLCPP_INFO(this->get_logger(), "About to initialize ImageTransport");
    try {
        auto self = shared_from_this();
        image_transport::ImageTransport it(self);
        pub_image_ = it.advertise("image", 1);
    } catch (const std::bad_weak_ptr& e) {
        RCLCPP_ERROR(this->get_logger(), "bad_weak_ptr in ImageTransport: %s", e.what());
        throw;
    }

    RCLCPP_INFO(this->get_logger(), "About to call shared_from_this() for timer");
    try {
        auto self = shared_from_this();
        RCLCPP_INFO(this->get_logger(), "shared_from_this() succeeded");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), [self]() { std::static_pointer_cast<MonoProcessor>(self)->capture_and_process(); });
    } catch (const std::bad_weak_ptr& e) {
        RCLCPP_ERROR(this->get_logger(), "bad_weak_ptr caught: %s", e.what());
        throw;
    }
}

void MonoProcessor::capture_and_process()
{
    cap_ >> image_;

    if (!image_.empty() && image_.cols > 0 && image_.rows > 0) {
        process_mono();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Captured image is invalid or empty");
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

    if (display_mode_ != "none") {
        cv::Mat resized_image;
        cv::resize(image_, resized_image, cv::Size(640, 480)); // Adjust size as needed
        cv::namedWindow("Mono Feed", cv::WINDOW_AUTOSIZE);
        cv::imshow("Mono Feed", resized_image);
        cv::waitKey(1);
    }
}