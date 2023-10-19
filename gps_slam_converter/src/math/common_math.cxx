#include "math/common_math.hxx"

gps_slam_converter::math::CommonMath::CommonMath()
{
}

gps_slam_converter::math::CommonMath::~CommonMath()
{
}

double gps_slam_converter::math::CommonMath::distance_formula(double x1, double y1, double x2, double y2)
{
    const double &sqr_x = (x2 - x1) * (x2 - x1);
    const double &sqr_y = (y2 - y1) * (y2 - y1);

    double distance = sqrt(sqr_x + sqr_y);

    return distance;
}

double gps_slam_converter::math::CommonMath::calculate_radian(double x1, double y1, double x2, double y2)
{
    const double &atan_x = x2 - x1;
    const double &atan_y = y2 - y1;

    double radian = atan2(atan_y, atan_x);

    return radian;
}

double gps_slam_converter::math::CommonMath::degrees_to_radian(double degrees)
{
    double radian = degrees * M_PI / 180.0;

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "calculated - degrees to radian : [%f]", radian);
    RCLCPP_LINE_INFO();

    return radian;
}

double gps_slam_converter::math::CommonMath::radian_to_degrees(double radians)
{
    double radian_degrees = (radians * (M_PI / 180.0));

    return radian_degrees;
}

double gps_slam_converter::math::CommonMath::angle_between_two_points(double x1, double x2, double y1, double y2)
{
    const double &atan_x = x2 - x1;
    const double &atan_y = y2 - y1;

    double radian = atan2(atan_y, atan_x);

    return radian;
}