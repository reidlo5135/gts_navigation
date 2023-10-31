/**
 * @file main.cxx
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-07-31
 * @brief implementation file for main program
 */

/**
 * @brief include/gts_navigator/gts_navigator.hxx include area
 * @include gts_navigator/gts_navigator.hxx
 */

#include "gts_navigator/gts_navigator.hxx"

/**
 * @brief function for initialize rclcpp & execute main program
 * @param argc int
 * @param argv const char * const *
 * @return RCL_STOP_FLAG int
 */
int main(int argc, const char *const *argv)
{
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr rcl_node = std::make_shared<gts_navigator::Navigator>();
	rclcpp::executors::MultiThreadedExecutor rcl_multithread_executor;
	rcl_multithread_executor.add_node(rcl_node);

	signal(SIGINT, &gts_navigator::Navigator::signal_handler);
	signal(SIGTSTP, &gts_navigator::Navigator::signal_handler);

	rcl_multithread_executor.spin();
	rclcpp::shutdown();

	return RCL_STOP_FLAG;
}