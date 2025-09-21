#include "orin_rp2_csi/mono_processor.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

MonoProcessor::MonoProcessor() : Node("mono_processor")
{
    RCLCPP_INFO(this->get_logger(), "MonoProcessor constructor started");

    // Add help parameter
    this->declare_parameter<bool>("help", false);
    bool show_help = this->get_parameter("help").as_bool();
    
    if (show_help) {
        print_help();
        rclcpp::shutdown();
        return;
    }

    // Ensure the parameter is declared as an integer
    this->declare_parameter<int>("sensor_id", 0);
    int sensor_id_int = this->get_parameter("sensor_id").as_int();
    std::string sensor_id = std::to_string(sensor_id_int);

    // Add parameters for resolution
    this->declare_parameter<int>("width", 1920);
    this->declare_parameter<int>("height", 1080);
    this->declare_parameter<int>("framerate", 30);
    
    int width = this->get_parameter("width").as_int();
    int height = this->get_parameter("height").as_int();
    int framerate = this->get_parameter("framerate").as_int();

    std::string pipeline = "nvarguscamerasrc sensor-id=" + sensor_id + 
                          " ! video/x-raw(memory:NVMM), width=" + std::to_string(width) + 
                          ", height=" + std::to_string(height) + 
                          ", framerate=" + std::to_string(framerate) + "/1, format=NV12" +
                          " ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";

    std::cout << "Using pipeline: " << pipeline << std::endl;

    cap_.open(pipeline, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
    }

    try {
        std::string home_dir = std::getenv("HOME");
        std::string calib_file = home_dir + "/roboracer_ws/params/cameras/" + sensor_id + "/calibration.txt";
        if (!load_calibration(calib_file)) {
            RCLCPP_WARN(this->get_logger(), "Calibration file not found at %s, proceeding without rectification", calib_file.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error loading calibration: %s", e.what());
    }

    // Add a parameter for display mode
    this->declare_parameter<std::string>("display_mode", "raw");
    display_mode_ = this->get_parameter("display_mode").as_string();

    // Add a parameter for image format (raw or compressed)
    this->declare_parameter<std::string>("image_format", "raw");
    image_format_ = this->get_parameter("image_format").as_string();

    std::cout << "MonoProcessor constructor completed" << std::endl;
    std::cout << "Display mode: " << display_mode_ << std::endl;
    std::cout << "Image format: " << image_format_ << std::endl;
}

void MonoProcessor::init()
{
    RCLCPP_INFO(this->get_logger(), "About to initialize ImageTransport");
    try {
        auto self = shared_from_this();
        
        // Get sensor_id to create topic name
        int sensor_id_int = this->get_parameter("sensor_id").as_int();
        std::string base_topic = "camera_" + std::to_string(sensor_id_int);
        
        image_transport::ImageTransport it(self);
        
        // Publisher for raw (unrectified) images
        std::string raw_topic = base_topic + "/image_raw";
        pub_image_raw_ = it.advertise(raw_topic, 1);
        RCLCPP_INFO(this->get_logger(), "Publishing raw images to topic: %s", raw_topic.c_str());
        
        // Publisher for rectified images
        std::string rect_topic = base_topic + "/image_rect";
        pub_image_rect_ = it.advertise(rect_topic, 1);
        RCLCPP_INFO(this->get_logger(), "Publishing rectified images to topic: %s", rect_topic.c_str());
        
        RCLCPP_INFO(this->get_logger(), "Image format: %s", image_format_.c_str());
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
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        // Remove spaces and find the assignment
        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;
        
        std::string key = line.substr(0, eq_pos);
        key.erase(std::remove_if(key.begin(), key.end(), ::isspace), key.end());
        
        std::string value_str = line.substr(eq_pos + 1);
        
        // Extract values between brackets
        size_t start = value_str.find('[');
        size_t end = value_str.find(']');
        if (start == std::string::npos || end == std::string::npos) continue;
        
        value_str = value_str.substr(start + 1, end - start - 1);
        
        // Parse comma-separated values
        std::vector<double> values;
        std::stringstream ss(value_str);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            // Remove whitespace
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            if (!token.empty()) {
                values.push_back(std::stod(token));
            }
        }
        
        if (key == "D" && values.size() >= 5) {
            dist_coeffs_ = cv::Mat(1, 5, CV_64F);
            for (int i = 0; i < 5; i++) {
                dist_coeffs_.at<double>(0, i) = values[i];
            }
        } else if (key == "K" && values.size() >= 9) {
            camera_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 9; i++) {
                camera_matrix_.at<double>(i / 3, i % 3) = values[i];
            }
        }
        // Note: R and P matrices are parsed but not used in this simple implementation
    }
    
    file.close();
    
    // Check if we successfully loaded the required matrices
    bool success = !camera_matrix_.empty() && !dist_coeffs_.empty();
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Successfully loaded calibration from: %s", filename.c_str());
    }
    
    return success;
}

void MonoProcessor::print_help()
{
    std::cout << "\n=== ORIN RP2 CSI Camera Node Help ===\n\n";
    std::cout << "DESCRIPTION:\n";
    std::cout << "  Captures images from CSI cameras and publishes them as ROS2 topics.\n";
    std::cout << "  Supports both raw and rectified image publishing with calibration.\n\n";
    
    std::cout << "PARAMETERS:\n";
    std::cout << "  sensor_id     Camera sensor ID (integer)\n";
    std::cout << "                Range: 0-7 (depending on connected cameras)\n";
    std::cout << "                Default: 0\n\n";
    
    std::cout << "  width         Image width in pixels (integer)\n";
    std::cout << "                Common values: 640, 1280, 1920, 3840\n";
    std::cout << "                Default: 1920\n\n";
    
    std::cout << "  height        Image height in pixels (integer)\n";
    std::cout << "                Common values: 480, 720, 1080, 2160\n";
    std::cout << "                Default: 1080\n\n";
    
    std::cout << "  framerate     Frames per second (integer)\n";
    std::cout << "                Range: 1-120 (depending on resolution)\n";
    std::cout << "                Recommended: 15-60\n";
    std::cout << "                Default: 30\n\n";
    
    std::cout << "  display_mode  Display visualization (string)\n";
    std::cout << "                Options: \"raw\", \"rect\", \"none\"\n";
    std::cout << "                  raw:  Show raw (unrectified) camera feed\n";
    std::cout << "                  rect: Show rectified camera feed (requires calibration)\n";
    std::cout << "                  none: No display window\n";
    std::cout << "                Default: \"raw\"\n\n";
    
    std::cout << "  image_format  Image publishing format (string)\n";
    std::cout << "                Options: \"raw\", \"compressed\"\n";
    std::cout << "                Default: \"raw\"\n\n";
    
    std::cout << "PUBLISHED TOPICS:\n";
    std::cout << "  /camera_{sensor_id}/image_raw      - Raw (unrectified) images\n";
    std::cout << "  /camera_{sensor_id}/image_rect     - Rectified images (if calibration available)\n";
    std::cout << "  Both topics also publish compressed variants automatically\n\n";
    
    std::cout << "CALIBRATION:\n";
    std::cout << "  Calibration files should be placed at:\n";
    std::cout << "  $HOME/roboracer_ws/params/cameras/{sensor_id}/calibration.txt\n\n";
    
    std::cout << "USAGE EXAMPLES:\n";
    std::cout << "  # Basic usage with camera 0:\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor\n\n";
    
    std::cout << "  # Camera 1 with no display:\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor --ros-args -p sensor_id:=1 -p display_mode:=none\n\n";
    
    std::cout << "  # Show rectified feed (requires calibration):\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor --ros-args -p display_mode:=rect\n\n";
    
    std::cout << "  # Custom resolution (720p at 60fps):\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor --ros-args -p width:=1280 -p height:=720 -p framerate:=60\n\n";
    
    std::cout << "  # High performance (480p at 120fps):\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor --ros-args -p width:=640 -p height:=480 -p framerate:=120\n\n";
    
    std::cout << "  # 4K recording (low framerate):\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor --ros-args -p width:=3840 -p height:=2160 -p framerate:=15\n\n";
    
    std::cout << "  # Multiple cameras simultaneously:\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor --ros-args -p sensor_id:=0 -p display_mode:=none &\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor --ros-args -p sensor_id:=1 -p display_mode:=none\n\n";
    
    std::cout << "  # Show this help:\n";
    std::cout << "  ros2 run orin_rp2_csi mono_processor --ros-args -p help:=true\n\n";
    
    std::cout << "COMMON RESOLUTIONS:\n";
    std::cout << "  VGA:     640x480\n";
    std::cout << "  HD:      1280x720\n";
    std::cout << "  FHD:     1920x1080\n";
    std::cout << "  4K UHD:  3840x2160\n\n";
    
    std::cout << "NOTE: Higher resolutions may require lower framerates.\n";
    std::cout << "      Calibration improves image quality when available.\n\n";
}

void MonoProcessor::process_mono()
{
    cv::Mat rectified;
    bool need_rectification = false;
    
    // Publish raw (unrectified) image if there are subscribers
    if (pub_image_raw_.getNumSubscribers() > 0) {
        sensor_msgs::msg::Image::SharedPtr raw_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
        raw_msg->header.stamp = this->now();
        raw_msg->header.frame_id = "camera_frame";
        pub_image_raw_.publish(raw_msg);
    }
    
    // Check if we need rectification for publishing or display
    need_rectification = (pub_image_rect_.getNumSubscribers() > 0) || (display_mode_ == "rect");
    
    if (need_rectification) {
        if (!camera_matrix_.empty() && !dist_coeffs_.empty()) {
            cv::undistort(image_, rectified, camera_matrix_, dist_coeffs_);
        } else {
            rectified = image_.clone();
        }
    }
    
    // Publish rectified image if there are subscribers
    if (pub_image_rect_.getNumSubscribers() > 0) {
        sensor_msgs::msg::Image::SharedPtr rect_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified).toImageMsg();
        rect_msg->header.stamp = this->now();
        rect_msg->header.frame_id = "camera_frame";
        pub_image_rect_.publish(rect_msg);
    }

    // Display based on mode
    if (display_mode_ == "raw") {
        cv::Mat resized_image;
        cv::resize(image_, resized_image, cv::Size(640, 480));
        cv::namedWindow("Raw Camera Feed", cv::WINDOW_AUTOSIZE);
        cv::imshow("Raw Camera Feed", resized_image);
        cv::waitKey(1);
    } else if (display_mode_ == "rect") {
        if (need_rectification) {
            cv::Mat resized_image;
            cv::resize(rectified, resized_image, cv::Size(640, 480));
            cv::namedWindow("Rectified Camera Feed", cv::WINDOW_AUTOSIZE);
            cv::imshow("Rectified Camera Feed", resized_image);
            cv::waitKey(1);
        } else {
            // Fall back to raw if no calibration available
            cv::Mat resized_image;
            cv::resize(image_, resized_image, cv::Size(640, 480));
            cv::namedWindow("Camera Feed (No Calibration)", cv::WINDOW_AUTOSIZE);
            cv::imshow("Camera Feed (No Calibration)", resized_image);
            cv::waitKey(1);
        }
    }
    // If display_mode_ == "none", no display window is shown
}