#ifndef QUATERNION__HXX
#define QUATERNION__HXX

#include "points/points.hxx"

namespace gps_slam_converter
{
    namespace points
    {
        class QuaternionPoint final
        {
        private:
            double x_, y_, z_, w_;
            double roll_, pitch_, yaw_;
            double qr_distance_;
            double height_;
            double base_line_;
            double double_radian_;
            double euler_x_;
            double euler_y_;
            double euler_z_;
            double euler_w_;

        public:
            explicit QuaternionPoint();
            virtual ~QuaternionPoint();
            void quaternion_to_euler_angle();
            double get__roll();
            double get__pitch();
            double get__yaw();
            double get__x();
            void set__x(const double &x);
            double get__y();
            void set__y(const double &y);
            double get__z();
            void set__z(const double &z);
            double get__w();
            void set__w(const double &w);
            double degree_to_radian(double degree);
            double get__height();
            double get__base_line();
            void euler_angle_to_quaternion(double yaw, double pitch, double roll);
            double get__euler_w();
            double get__euler_z();
            double get__euler_x();
            double get_euler_y();
        };
    }
}

#endif