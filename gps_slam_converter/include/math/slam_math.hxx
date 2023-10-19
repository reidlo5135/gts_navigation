#ifndef SLAM_MATH__HXX
#define SLAM_MATH__HXX

#include "math/math.hxx"
#include "math/common_math.hxx"
#include "points/slam_point.hxx"

namespace gps_slam_converter
{
    namespace math
    {
        class SLAMMath final
        {
        private:
            std::shared_ptr<gps_slam_converter::math::CommonMath> common_math_ptr_;

        public:
            explicit SLAMMath();
            virtual ~SLAMMath();
            double distance_formula(
                std::shared_ptr<gps_slam_converter::points::SLAMPoint> start_point_ptr,
                std::shared_ptr<gps_slam_converter::points::SLAMPoint> end_point_ptr);
            double angle_between_two_points(
                std::shared_ptr<gps_slam_converter::points::SLAMPoint> start_point_ptr,
                std::shared_ptr<gps_slam_converter::points::SLAMPoint> end_point_ptr);
            std::shared_ptr<gps_slam_converter::points::SLAMPoint> rotate_point(double x, double y, double angle);
        };
    }
}

#endif