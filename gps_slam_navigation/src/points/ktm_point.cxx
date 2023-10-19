#include "points/ktm_point.hxx"

gps_slam_navigation::points::KTMPoint::KTMPoint()
{
}

gps_slam_navigation::points::KTMPoint::~KTMPoint()
{
}

double gps_slam_navigation::points::KTMPoint::get__x()
{
    return this->x_;
}

void gps_slam_navigation::points::KTMPoint::set__x(const double &x)
{
    this->x_ = x;
}

double gps_slam_navigation::points::KTMPoint::get__y()
{
    return this->y_;
}

void gps_slam_navigation::points::KTMPoint::set__y(const double &y)
{
    this->y_ = y;
}