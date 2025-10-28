cmake_minimum_required(VERSION 3.22)
project(orin_rp2_csi)

# find dependencies
include_directories(include)

# Set compute capability for Orin's Ampere GPU (8.7)
set(OpenCV_CUDA_VERSION "12.6")
set(CUDA_ARCH_BIN "8.7")
set(CUDA_DIR /usr/local/cuda)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED QUIET)

# Add mono_processor executable
add_executable(mono_processor src/mono_node.cpp src/mono_processor.cpp)
ament_target_dependencies(mono_processor rclcpp cv_bridge image_transport sensor_msgs std_msgs ament_index_cpp OpenCV)

# Add stereo_processor executable
add_executable(stereo_processor src/stereo_node.cpp src/stereo_processor.cpp)
ament_target_dependencies(stereo_processor rclcpp cv_bridge image_transport sensor_msgs std_msgs ament_index_cpp OpenCV)

# Install executable and launch/params files
install(TARGETS
  mono_processor
  stereo_processor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch params calibration
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
