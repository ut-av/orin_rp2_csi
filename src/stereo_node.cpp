#include "rclcpp/rclcpp.hpp"
#include "orin_rp2_csi/stereo_processor.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoProcessor>());
    rclcpp::shutdown();
    return 0;
}
