#ifndef UTILS__HXX
#define UTILS__HXX

#include <rcutils/logging_macros.h>

static constexpr const char *RCL_NODE_NAME = "gps_slam_converter";
static constexpr const double &RCL_DEFAULT_DOUBLE = 0.0;
static constexpr const char *RCL_ROBOT_POSE_TOPIC = "/robot_pose";
static constexpr const char *RCL_UBLOX_FIX_TOPIC = "/ublox/fix";
static constexpr const char *RCL_GPS_TO_SLAM_TOPIC = "/gps_to_slam";
static constexpr const char *RCL_SLAM_TO_GPS_TOPIC = "/slam_to_gps";
static constexpr const char *RCL_CONVERTER_SERVICE_SERVER_NAME = "/gps_slam_converter/conversion";
static constexpr const char *RCL_CONVERTER_SERVICE_CONVERSION_TARGET_GPS = "GPS";
static constexpr const char *RCL_CONVERTER_SERVICE_CONVERSION_TARGET_SLAM = "SLAM";

static constexpr const double &EARTH_RADIUS_KTM = 6371.0;
static constexpr const double &EARTH_RADIUS_M = EARTH_RADIUS_KTM * 1000;
static constexpr const double &GTS_DEFAULT_DOUBLE = 0.000000;

#define RCLCPP_LINE_INFO() \
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_ERROR() \
    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_WARN() \
    RCUTILS_LOG_WARN_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#endif