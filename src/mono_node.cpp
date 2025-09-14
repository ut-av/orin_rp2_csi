#include "rclcpp/rclcpp.hpp"
#include "orin_rp2_csi/mono_processor.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonoProcessor>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
