#include "math/gts_math.hxx"

gps_slam_converter::math::GTSMath::GTSMath()
{
    common_math_ptr_ = std::make_shared<gps_slam_converter::math::CommonMath>();
}

gps_slam_converter::math::GTSMath::~GTSMath()
{
}

double gps_slam_converter::math::GTSMath::calculate_rotation(std::shared_ptr<gps_slam_converter::points::GTSPoint> gts_point_ptr)
{
    const double &x1 = gts_point_ptr->get__slam_point_1_ptr()->get__x();
    const double &y1 = gts_point_ptr->get__slam_point_1_ptr()->get__y();

    const double &x2 = gts_point_ptr->get__slam_point_2_ptr()->get__x();
    const double &y2 = gts_point_ptr->get__slam_point_2_ptr()->get__y();

    const double &x3 = gts_point_ptr->get__ktm_point_1_ptr()->get__x();
    const double &y3 = gts_point_ptr->get__ktm_point_1_ptr()->get__y();

    const double &x4 = gts_point_ptr->get__ktm_point_2_ptr()->get__x();
    const double &y4 = gts_point_ptr->get__ktm_point_2_ptr()->get__y();

    double v_ab_x = x2 - x1;
    double v_ab_y = y2 - y1;
    double v_cd_x = x4 - x3;
    double v_cd_y = y4 - y3;

    double dot_product = (v_ab_x * v_cd_x) + (v_ab_y * v_cd_y);

    const double &magnitude_ab = sqrt(v_ab_x * v_ab_x + v_ab_y * v_ab_y);
    const double &magnitude_cd = sqrt(v_cd_x * v_cd_x + v_cd_y * v_cd_y);

    const double &angle_radian = acos(dot_product / (magnitude_ab * magnitude_cd));

    return angle_radian;
}