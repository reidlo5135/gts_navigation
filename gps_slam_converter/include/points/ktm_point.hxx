#ifndef KTM__HXX
#define KTM__HXX

#include "points/points.hxx"

namespace gps_slam_converter
{
    namespace points
    {
        class KTMPoint final
        {
        private:
            double x_;
            double y_;

        public:
            explicit KTMPoint();
            virtual ~KTMPoint();
            double get__x();
            void set__x(const double &x);
            double get__y();
            void set__y(const double &y);
        };
    }
}

#endif