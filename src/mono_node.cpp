#include "rclcpp/rclcpp.hpp"
#include "orin_rp2_csi/mono_processor.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonoProcessor>());
    rclcpp::shutdown();
    return 0;
}
