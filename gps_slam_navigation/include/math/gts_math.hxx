#ifndef GTS_MATH__HXX
#define GTS_MATH__HXX

#include "math/math.hxx"
#include "points/gts_point.hxx"
#include "math/common_math.hxx"

namespace gps_slam_navigation
{
    namespace math
    {
        class GTSMath final
        {
        private:
            std::shared_ptr<gps_slam_navigation::math::CommonMath> common_math_ptr_;

        public:
            explicit GTSMath();
            virtual ~GTSMath();
            double calculate_rotation(std::shared_ptr<gps_slam_navigation::points::GTSPoint> gts_point_ptr);
        };
    }
}

#endif