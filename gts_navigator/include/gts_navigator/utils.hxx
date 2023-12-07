#ifndef UTILS__HXX
#define UTILS__HXX

/**
 * @brief include default cpp header files
 */
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <signal.h>
#include <memory>

/**
 * @brief include header files for rclcpp
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging_macros.h>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <gps_slam_conversion_msgs/srv/conversion.hpp>
#include <gts_navigation_msgs/msg/goal_waypoints.hpp>
#include <gts_navigation_msgs/msg/navigation_status_stamped.hpp>
#include <gts_navigation_msgs/msg/navigation_result_stamped.hpp>
#include <gts_navigation_msgs/msg/navigation_control.hpp>

/**
 * ------------------------------------------------------
 * ------------------ RCL AREA START --------------------
 * ------------------------------------------------------
 */

/**
 * @brief static const instance for define name of rclcpp::Node
 */
static constexpr const char *RCL_NODE_NAME = "gts_navigator";

static constexpr const char *RCL_HEADER_FRAME_ID = "gts";

/**
 * @brief static const instance for define default value of rclcpp::QoS
 */
static constexpr const int &RCL_DEFAULT_QOS = 10;

/**
 * @brief static const instance for define int for flag stopping RCL
 */
static constexpr const int &RCL_STOP_FLAG = 0;

static constexpr const int &RCL_DEFAULT_INT = 0;

/**
 * @brief static const instance for define default Double value
 */
static constexpr const double &RCL_DEFAULT_DOUBLE = 0.0;

/**
 * @brief static const instance for define gps_waypoints_vector default index
 */
static constexpr const int &GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX = 0;

/**
 * @brief static const instance for define gps_waypoints_vector default size
 */
static constexpr const int &GOAL_WAYPOINTS_VECTOR_DEFAULT_SIZE = 0;

static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_STOPPED = 5;

static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_RESUMED = 1;

static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_STARTED = 2;

static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED = 4;

static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_ABORTED = 6;

static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_CANCELED = 5;

/**
 * @brief static const instance for define flag of rclcpp::Subscription
 */
static constexpr const char *RCL_SUBSCRIPTION_FLAG = "subscription";

/**
 * @brief static const instance for define flag of rclcpp::Publisher
 */
static constexpr const char *RCL_PUBLISHER_FLAG = "publisher";

static constexpr const char *RCL_SERVICE_CLIENT_FLAG = "service_client";

static constexpr const char *RCL_SERVICE_SERVER_FLAG = "service_server";

/**
 * @brief static const instance for define flag of rclcpp_action::Client
 */
static constexpr const char *RCL_ACTION_CLIENT_FLAG = "action_client";

static constexpr const char *RCL_GPS_SLAM_CONVERSION_SERVICE_SERVER_NAME = "/gps_slam_converter/conversion";

/**
 * @brief static const instance for define topic rclcpp::Subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>
 */
static constexpr const char *RCL_GOAL_WAYPOINTS_STAMPED_TOPIC = "/gts_navigation/waypoints";

static constexpr const char *RCL_NAVIGATION_STATUS_STAMPED_TOPIC = "/gts_navigation/status";

static constexpr const char *RCL_NAVIGATION_RESULT_STAMPED_TOPIC = "/gts_navigation/result";

static constexpr const char *RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC = "/navigate_to_pose/_action/status";

/**
 * @brief static const instance for define name of rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 */
static constexpr const char *RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME = "navigate_to_pose";

static constexpr const char *RCL_GTS_NAVIGATION_CONTROL_SUBSCRIPTION_NAME = "/gts_navigation/control";

/**
 * --------------------------------------------------.---
 * ------------------- RCL AREA END ---------------------
 * ------------------------------------------------------
 */


/**
 * @brief define macros area
 */
#define RCLCPP_LINE_INFO() \
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_ERROR() \
    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_WARN() \
    RCUTILS_LOG_WARN_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

/**
 * @brief using namespace area
 */
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

#endif