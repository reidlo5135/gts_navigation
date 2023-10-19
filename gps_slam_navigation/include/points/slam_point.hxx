#ifndef SLAM__HXX
#define SLAM__HXX

#include "points/points.hxx"

namespace gps_slam_navigation
{
    namespace points
    {
        class SLAMPoint final
        {
        private:
            double x_;
            double y_;
        public:
            explicit SLAMPoint();
            explicit SLAMPoint(const double &x, const double &y);
            virtual ~SLAMPoint();
            double get__x();
            void set__x(const double &x);
            double get__y();
            void set__y(const double &y);
        };
    }
}

#endif