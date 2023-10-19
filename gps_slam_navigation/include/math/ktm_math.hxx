#ifndef KTM_MATH__HXX
#define KTM_MATH__HXX

#include "math/math.hxx"
#include "points/ktm_point.hxx"
#include "points/geo_point.hxx"

static constexpr const char *WGS_84 = "EPSG:4326";
static constexpr const char *KTM = "EPSG:5186";

namespace gps_slam_navigation
{
    namespace math
    {
        class KTMMath final
        {
        public:
            explicit KTMMath();
            virtual ~KTMMath();
            std::shared_ptr<gps_slam_navigation::points::KTMPoint> geo_point_to_ktm_point(double latitude, double longitude);
            std::shared_ptr<gps_slam_navigation::points::GeoPoint> ktm_point_to_geo_point(double x, double y);
        };
    }
}

#endif