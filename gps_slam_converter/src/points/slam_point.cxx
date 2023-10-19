#include "points/slam_point.hxx"

gps_slam_converter::points::SLAMPoint::SLAMPoint()
{
}

gps_slam_converter::points::SLAMPoint::SLAMPoint(const double &x, const double &y)
{
    this->x_ = x;
    this->y_ = y;
}

gps_slam_converter::points::SLAMPoint::~SLAMPoint()
{
}

double gps_slam_converter::points::SLAMPoint::get__x()
{
    return this->x_;
}

void gps_slam_converter::points::SLAMPoint::set__x(const double &x)
{
    this->x_ = x;
}

double gps_slam_converter::points::SLAMPoint::get__y()
{
    return this->y_;
}

void gps_slam_converter::points::SLAMPoint::set__y(const double &y)
{
    this->y_ = y;
}