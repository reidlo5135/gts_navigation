#ifndef COMMON_MATH__HXX
#define COMMON_MATH__HXX

#include "math/math.hxx"
#include "points/points.hxx"

namespace gps_slam_navigation
{
    namespace math
    {
        class CommonMath final
        {
        public:
            explicit CommonMath();
            virtual ~CommonMath();
            double distance_formula(double x1, double y1, double x2, double y2);
            double calculate_radian(double x1, double y1, double x2, double y2);
            double degrees_to_radian(double degrees);
            double radian_to_degrees(double radians);
            double angle_between_two_points(double x1, double x2, double y1, double y2);
        };
    }
}

#endif