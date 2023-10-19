/**
 * @file main.cxx
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-07-31
 * @brief implementation file for main program
 */

/**
 * @brief include/gps_slam_navigation/gps_slam_navigation.hxx include area
 * @include gps_slam_navigation/gps_slam_navigation.hxx
 */

#include "gps_slam_navigation/gps_slam_navigation.hxx"

/**
 * @brief function for initialize rclcpp & execute main program
 * @param argc int
 * @param argv const char * const *
 * @return RCL_STOP_FLAG int
 */
int main(int argc, const char *const *argv)
{
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr rcl_node_ptr = std::make_shared<gps_slam_navigation::navigation::Navigator>();

	signal(SIGINT, &gps_slam_navigation::navigation::Navigator::signal_handler);
	signal(SIGTSTP, &gps_slam_navigation::navigation::Navigator::signal_handler);

	rclcpp::spin(rcl_node_ptr);
	rclcpp::shutdown();

	return RCL_STOP_FLAG;
}