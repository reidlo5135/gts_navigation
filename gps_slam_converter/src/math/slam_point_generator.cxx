#include "math/slam_point_generator.hxx"

gps_slam_converter::math::SLAMPointGenerator::SLAMPointGenerator()
{
    common_math_ = std::make_shared<gps_slam_converter::math::CommonMath>();
    gps_math_ = std::make_shared<gps_slam_converter::math::GPSMath>();
    gts_math_ = std::make_shared<gps_slam_converter::math::GTSMath>();
}

gps_slam_converter::math::SLAMPointGenerator::~SLAMPointGenerator()
{
}

std::shared_ptr<gps_slam_converter::points::SLAMPoint> gps_slam_converter::math::SLAMPointGenerator::geo_point_to_slam_point(
    std::shared_ptr<gps_slam_converter::points::GTSPoint> gts_point,
    std::shared_ptr<gps_slam_converter::points::KTMPoint> ktm_point)
{
    std::shared_ptr<gps_slam_converter::points::SLAMPoint> slam_point = std::make_shared<gps_slam_converter::points::SLAMPoint>();

    std::shared_ptr<gps_slam_converter::points::KTMPoint> ktm_bcm_point = gts_point->get__ktm_point_1_ptr();

    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "slam point generator GtS\n\tktm btm x : [%f]\n\tktm btm y : [%f]\n\tktm des x : [%f]\n\tktm des y : [%f]",
        ktm_bcm_point->get__x(),
        ktm_bcm_point->get__y(),
        ktm_point->get__x(),
        ktm_point->get__y());
    RCLCPP_LINE_INFO();

    const double &distance = common_math_->distance_formula(
        ktm_bcm_point->get__x(), ktm_bcm_point->get__y(),
        ktm_point->get__x(), ktm_point->get__y());

    const double &rotated_angle = gts_math_->calculate_rotation(gts_point);

    const double &start_ktm_to_des_radian = common_math_->calculate_radian(
        ktm_bcm_point->get__x(), ktm_bcm_point->get__y(),
        ktm_point->get__x(), ktm_point->get__y());

    const double &radian = start_ktm_to_des_radian + rotated_angle + M_PI;

    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "slam point generate Gts\n\tdistance : [%f]\n\trotate angle : [%f]\n\tstart_ktm_to_des_radian : [%f]\n\tradian : [%f]",
        distance,
        rotated_angle,
        start_ktm_to_des_radian,
        radian);
    RCLCPP_LINE_INFO();

    const double &x = gts_point->get__slam_point_1_ptr()->get__x() + distance * cos(radian);
    const double &y = gts_point->get__slam_point_1_ptr()->get__y() + distance * sin(radian);

    slam_point->set__x(x);
    slam_point->set__y(y);

    return slam_point;
}

std::shared_ptr<gps_slam_converter::points::GeoPoint> gps_slam_converter::math::SLAMPointGenerator::slam_point_to_geo_point(std::shared_ptr<gps_slam_converter::points::GTSPoint> gts_point, std::shared_ptr<gps_slam_converter::points::SLAMPoint> slam_des_point)
{
    std::shared_ptr<gps_slam_converter::points::SLAMPoint> slam_bcm_point = gts_point->get__slam_point_1_ptr();
    const double &distance = slam_math_->distance_formula(slam_bcm_point, slam_des_point);

    const double &rotate_angle = gts_math_->calculate_rotation(gts_point);
    const double &start_slam_to_des_slam_radian = common_math_->calculate_radian(
        slam_bcm_point->get__x(), slam_bcm_point->get__y(),
        slam_des_point->get__x(), slam_des_point->get__y());

    const double &radian = start_slam_to_des_slam_radian - rotate_angle - M_PI;

    const double &geo_point_x = gts_point->get__ktm_point_1_ptr()->get__x() + distance * cos(radian);
    const double &geo_point_y = gts_point->get__ktm_point_1_ptr()->get__y() + distance * sin(radian);

    std::shared_ptr<gps_slam_converter::points::GeoPoint> geo_point = this->ktm_math_->ktm_point_to_geo_point(geo_point_x, geo_point_y);

    return geo_point;
}