#include "points/gts_point.hxx"

gps_slam_converter::points::GTSPoint::GTSPoint()
{
    slam_point_1_ptr_ = std::make_shared<gps_slam_converter::points::SLAMPoint>();
    slam_point_2_ptr_ = std::make_shared<gps_slam_converter::points::SLAMPoint>();

    ktm_point_1_ptr_ = std::make_shared<gps_slam_converter::points::KTMPoint>();
    ktm_point_2_ptr_ = std::make_shared<gps_slam_converter::points::KTMPoint>();
}

gps_slam_converter::points::GTSPoint::~GTSPoint()
{
}

std::shared_ptr<gps_slam_converter::points::SLAMPoint> gps_slam_converter::points::GTSPoint::get__slam_point_1_ptr()
{
    return this->slam_point_1_ptr_;
}

void gps_slam_converter::points::GTSPoint::set__slam_point_1_ptr(std::shared_ptr<gps_slam_converter::points::SLAMPoint> slam_point_1_ptr)
{
    this->slam_point_1_ptr_ = slam_point_1_ptr;
}

std::shared_ptr<gps_slam_converter::points::SLAMPoint> gps_slam_converter::points::GTSPoint::get__slam_point_2_ptr()
{
    return this->slam_point_2_ptr_;
}

void gps_slam_converter::points::GTSPoint::set__slam_point_2_ptr(std::shared_ptr<gps_slam_converter::points::SLAMPoint> slam_point_2_ptr)
{
    this->slam_point_2_ptr_ = slam_point_2_ptr;
}

std::shared_ptr<gps_slam_converter::points::KTMPoint> gps_slam_converter::points::GTSPoint::get__ktm_point_1_ptr()
{
    return this->ktm_point_1_ptr_;
}

void gps_slam_converter::points::GTSPoint::set__ktm_point_1_ptr(std::shared_ptr<gps_slam_converter::points::KTMPoint> ktm_point_ptr)
{
    this->ktm_point_1_ptr_ = ktm_point_ptr;
}

std::shared_ptr<gps_slam_converter::points::KTMPoint> gps_slam_converter::points::GTSPoint::get__ktm_point_2_ptr()
{
    return this->ktm_point_2_ptr_;
}

void gps_slam_converter::points::GTSPoint::set__ktm_point_2_ptr(std::shared_ptr<gps_slam_converter::points::KTMPoint> ktm_point_ptr)
{
    this->ktm_point_2_ptr_ = ktm_point_ptr;
}