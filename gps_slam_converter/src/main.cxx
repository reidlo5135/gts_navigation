#include "gps_slam_converter/gps_slam_converter.hxx"

int main(int argc, const char *const *argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr rcl_node_ptr = std::make_shared<gps_slam_converter::converter::Converter>();
    rclcpp::spin(rcl_node_ptr);
    rclcpp::shutdown();

    return 0;
}