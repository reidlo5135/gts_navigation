#ifndef SLAM_POINT_GENERATOR__HXX
#define SLAM_POINT_GENERATOR__HXX

#include "math/math.hxx"
#include "math/common_math.hxx"
#include "math/gps_math.hxx"
#include "math/gts_math.hxx"
#include "math/slam_math.hxx"
#include "math/ktm_math.hxx"
#include "points/geo_point.hxx"
#include "points/slam_point.hxx"
#include "points/gts_point.hxx"
#include "points/ktm_point.hxx"

namespace gps_slam_navigation
{
    namespace math
    {
        class SLAMPointGenerator final
        {
        private:
            std::shared_ptr<gps_slam_navigation::math::CommonMath> common_math_;
            std::shared_ptr<gps_slam_navigation::math::GPSMath> gps_math_;
            std::shared_ptr<gps_slam_navigation::math::GTSMath> gts_math_;
            std::shared_ptr<gps_slam_navigation::math::SLAMMath> slam_math_;
            std::shared_ptr<gps_slam_navigation::math::KTMMath> ktm_math_;

        public:
            explicit SLAMPointGenerator();
            virtual ~SLAMPointGenerator();
            std::shared_ptr<gps_slam_navigation::points::SLAMPoint> geo_point_to_slam_point(std::shared_ptr<gps_slam_navigation::points::GTSPoint> gts_point, std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_des_point);
            std::shared_ptr<gps_slam_navigation::points::GeoPoint> slam_point_to_geo_point(std::shared_ptr<gps_slam_navigation::points::GTSPoint> gts_point, std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_des_point);
        };
    }
}

#endif