#include "points/geo_point.hxx"

gps_slam_navigation::points::GeoPoint::GeoPoint()
{
}

gps_slam_navigation::points::GeoPoint::~GeoPoint()
{
}

double gps_slam_navigation::points::GeoPoint::get__latitude()
{
    return this->latitude_;
}

void gps_slam_navigation::points::GeoPoint::set__latitude(double latitude)
{
    this->latitude_ = latitude;
}

double gps_slam_navigation::points::GeoPoint::get__longitude()
{
    return this->longitude_;
}

void gps_slam_navigation::points::GeoPoint::set__longitude(double longitude)
{
    this->longitude_ = longitude_;
}