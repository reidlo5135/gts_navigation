#ifndef GEO_POINT__HXX
#define GEO_POINT__HXX

#include "math/math.hxx"

namespace gps_slam_navigation
{
    namespace points
    {
        class GeoPoint final
        {
        private:
            double latitude_;
            double longitude_;

        public:
            explicit GeoPoint();
            virtual ~GeoPoint();

            double get__latitude();
            void set__latitude(double latitude);

            double get__longitude();
            void set__longitude(double longitude);
        };
    }
}

#endif