#include "math/ktm_math.hxx"

gps_slam_navigation::math::KTMMath::KTMMath()
{
}

gps_slam_navigation::math::KTMMath::~KTMMath()
{
}

std::shared_ptr<gps_slam_navigation::points::KTMPoint> gps_slam_navigation::math::KTMMath::geo_point_to_ktm_point(double latitude, double longitude)
{
    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "ktm math GtK raw\n\tlat : [%f]\n\tlon : [%f]",
        latitude,
        longitude);
    RCLCPP_LINE_INFO();

    PJ *P = proj_create_crs_to_crs(0, WGS_84, KTM, 0);

    PJ_COORD pj_coord;

    pj_coord.xy.x = latitude;
    pj_coord.xy.y = longitude;

    PJ_COORD re = proj_trans(P, PJ_FWD, pj_coord);

    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "ktm math GtK calculated\n\tlat : [%f]n\tlon : [%f]",
        re.xy.y,
        re.xy.x);
    RCLCPP_LINE_INFO();

    pj_free(P);

    std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point  = std::make_shared<gps_slam_navigation::points::KTMPoint>();
    ktm_point->set__x(re.xy.y);
    ktm_point->set__y(re.xy.x);

    return ktm_point;
}

std::shared_ptr<gps_slam_navigation::points::GeoPoint> gps_slam_navigation::math::KTMMath::ktm_point_to_geo_point(double x, double y)
{
    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "ktm math KtG raw\n\tx : [%f]\n\ty : [%f]",
        x,
        y);
    RCLCPP_LINE_INFO();

    PJ *P = proj_create_crs_to_crs(0, KTM, WGS_84, 0);

    PJ_COORD pj_coord;

    pj_coord.xy.x = x;
    pj_coord.xy.y = y;

    PJ_COORD re = proj_trans(P, PJ_FWD, pj_coord);

    RCUTILS_LOG_INFO_NAMED(
        RCL_NODE_NAME,
        "ktm math KtG calculated\n\tlat : [%f]\n\tlon : [%f]",
        re.xy.y,
        re.xy.x);
    RCLCPP_LINE_INFO();

    pj_free(P);

    std::shared_ptr<gps_slam_navigation::points::GeoPoint> geo_point = std::make_shared<gps_slam_navigation::points::GeoPoint>();
    geo_point->set__latitude(re.xy.y);
    geo_point->set__longitude(re.xy.x);

    return geo_point;
}