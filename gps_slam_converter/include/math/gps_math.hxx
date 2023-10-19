#ifndef GPS_MATH__HXX
#define GPS_MATH__HXX

#include "math/math.hxx"
#include "math/common_math.hxx"

namespace gps_slam_converter
{
    namespace math
    {
        class GPSMath final
        {
        private:
            std::shared_ptr<gps_slam_converter::math::CommonMath> common_math_ptr_;

        public:
            explicit GPSMath();
            virtual ~GPSMath();
            double distance_formula_kilometers(double latitude_1, double longitude_1, double latitude_2, double longitude_2);
            double distance_formula_meters(double latitude_1, double longitude_1, double latitude_2, double longitude_2);
            double angle_between_two_points(double latitude_1, double longitude_1, double latitude_2, double longitude_2);
        };
    }
}

#endif