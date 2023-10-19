#include "math/slam_math.hxx"

gps_slam_navigation::math::SLAMMath::SLAMMath()
{
    common_math_ptr_ = std::make_shared<gps_slam_navigation::math::CommonMath>();
}

gps_slam_navigation::math::SLAMMath::~SLAMMath()
{
}

double gps_slam_navigation::math::SLAMMath::distance_formula(
    std::shared_ptr<gps_slam_navigation::points::SLAMPoint> start_point_ptr,
    std::shared_ptr<gps_slam_navigation::points::SLAMPoint> end_point_ptr)
{
    const double &start_x = start_point_ptr->get__x();
    const double &start_y = start_point_ptr->get__y();

    const double &end_x = end_point_ptr->get__x();
    const double &end_y = end_point_ptr->get__y();

    const double &distance = common_math_ptr_->distance_formula(start_x, start_y, end_x, end_y);

    return distance;
}

double gps_slam_navigation::math::SLAMMath::angle_between_two_points(
    std::shared_ptr<gps_slam_navigation::points::SLAMPoint> start_point_ptr,
    std::shared_ptr<gps_slam_navigation::points::SLAMPoint> end_point_ptr)
{
    const double &start_x = start_point_ptr->get__x();
    const double &start_y = start_point_ptr->get__y();

    const double &end_x = end_point_ptr->get__x();
    const double &end_y = end_point_ptr->get__y();

    const double &angle = common_math_ptr_->angle_between_two_points(start_x, start_y, end_x, end_y);

    return angle;
}

std::shared_ptr<gps_slam_navigation::points::SLAMPoint> gps_slam_navigation::math::SLAMMath::rotate_point(double x, double y, double angle)
{
    std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_ptr = std::make_shared<gps_slam_navigation::points::SLAMPoint>();

    double new_x = (x * cos(angle)) - (y * sin(angle));
    double new_y = (x * sin(angle)) - (y * cos(angle));

    slam_point_ptr->set__x(new_x);
    slam_point_ptr->set__y(new_y);

    return slam_point_ptr;
}