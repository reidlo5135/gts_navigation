#ifndef GTS__HXX
#define GTS__HXX

#include "points/points.hxx"
#include "points/slam_point.hxx"
#include "points/ktm_point.hxx"

namespace gps_slam_navigation
{
    namespace points
    {
        class GTSPoint final
        {
        private:
            std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_1_ptr_;
            std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_2_ptr_;

            std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point_1_ptr_;
            std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point_2_ptr_;

        public:
            explicit GTSPoint();
            virtual ~GTSPoint();
            std::shared_ptr<gps_slam_navigation::points::SLAMPoint> get__slam_point_1_ptr();
            void set__slam_point_1_ptr(std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_1_ptr);
            std::shared_ptr<gps_slam_navigation::points::SLAMPoint> get__slam_point_2_ptr();
            void set__slam_point_2_ptr(std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_2_ptr);
            std::shared_ptr<gps_slam_navigation::points::KTMPoint> get__ktm_point_1_ptr();
            void set__ktm_point_1_ptr(std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point_ptr);
            std::shared_ptr<gps_slam_navigation::points::KTMPoint> get__ktm_point_2_ptr();
            void set__ktm_point_2_ptr(std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point_ptr);
        };
    }
}

#endif