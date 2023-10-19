#include "math/gps_math.hxx"

gps_slam_converter::math::GPSMath::GPSMath()
{
    common_math_ptr_ = std::make_shared<gps_slam_converter::math::CommonMath>();
}

gps_slam_converter::math::GPSMath::~GPSMath()
{
}

double gps_slam_converter::math::GPSMath::distance_formula_kilometers(double latitude_1, double longitude_1, double latitude_2, double longitude_2)
{
    const double &phi_1 = common_math_ptr_->degrees_to_radian(latitude_1);
    const double &phi_2 = common_math_ptr_->degrees_to_radian(latitude_2);
    const double &delta_phi = common_math_ptr_->degrees_to_radian(latitude_2 - latitude_1);
    const double &delta_lambda = common_math_ptr_->degrees_to_radian(longitude_2 - longitude_1);

    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "calculated - degrees to kilometers\n\tphi_1 : [%f]\n\tphi_2 : [%f]\n\tdelta_phi : [%f]\n\tdelta_lambda : [%f]",
        phi_1,
        phi_2,
        delta_phi,
        delta_lambda);
    RCLCPP_LINE_INFO();

    double a = (sin(delta_phi / 2.0) * sin(delta_phi / 2.0)) + (cos(phi_1) * cos(phi_2) * sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0));
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double distance_km = EARTH_RADIUS_KTM * c;

    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "calculated - degrees to kilometers\n\ta : [%f]\n\tc : [%f]distance_km : [%f]",
        a,
        c,
        distance_km);
    RCLCPP_LINE_INFO();

    return distance_km;
}

double gps_slam_converter::math::GPSMath::distance_formula_meters(double latitude_1, double longitude_1, double latitude_2, double longitude_2)
{
    const double distance = (this->distance_formula_kilometers(latitude_1, longitude_1, latitude_2, longitude_2) * 1000);

    return distance;
}

double gps_slam_converter::math::GPSMath::angle_between_two_points(double latitude_1, double longitude_1, double latitude_2, double longitude_2)
{
    double phi_1 = latitude_1 * M_PI / 180;
    double phi_2 = latitude_2 * M_PI / 180;
    double lambda_1 = longitude_1 * M_PI / 180;
    double lambda_2 = longitude_2 * M_PI / 180;

    double y = sin(lambda_2 - lambda_1) * cos(phi_2);
    double x = cos(phi_1) * sin(phi_2) - sin(phi_1) * cos(phi_2) * cos(lambda_2 - lambda_1);

    const double &angle = atan2(y, x);

    return angle;
}