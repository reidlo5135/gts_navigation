#include "points/quaternion_point.hxx"

gps_slam_navigation::points::QuaternionPoint::QuaternionPoint()
    : x_(RCL_DEFAULT_DOUBLE),
      y_(RCL_DEFAULT_DOUBLE),
      z_(RCL_DEFAULT_DOUBLE),
      w_(RCL_DEFAULT_DOUBLE),
      roll_(RCL_DEFAULT_DOUBLE),
      pitch_(RCL_DEFAULT_DOUBLE),
      yaw_(RCL_DEFAULT_DOUBLE),
      qr_distance_(RCL_DEFAULT_DOUBLE),
      height_(RCL_DEFAULT_DOUBLE),
      base_line_(RCL_DEFAULT_DOUBLE),
      double_radian_(RCL_DEFAULT_DOUBLE),
      euler_x_(RCL_DEFAULT_DOUBLE),
      euler_y_(RCL_DEFAULT_DOUBLE),
      euler_z_(RCL_DEFAULT_DOUBLE),
      euler_w_(RCL_DEFAULT_DOUBLE)
{
}

gps_slam_navigation::points::QuaternionPoint::~QuaternionPoint()
{
}

void gps_slam_navigation::points::QuaternionPoint::quaternion_to_euler_angle()
{
    double sinr_cosp = 2 * (w_ * x_ + y_ * z_);
    double cosr_cosp = 1 - 2 * (x_ * x_ + y_ * y_);

    roll_ = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = std::sqrt(1 + 2 * (w_ * y_ - x_ * z_));
    double cosp = std::sqrt(1 - 2 * (w_ * y_ - x_ * z_));

    pitch_ = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    if (pitch_ == 1.5708)
    {
        pitch_ = 1.5918;
    }
    else if (pitch_ == -1.5708)
    {
        pitch_ = -1.5918;
    }

    double siny_cosp = 2 * (w_ * z_ + x_ * y_);
    double cosy_cosp = 1 - 2 * (y_ * y_ + z_ * z_);

    yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

double gps_slam_navigation::points::QuaternionPoint::get__roll()
{
    return this->roll_;
}

double gps_slam_navigation::points::QuaternionPoint::get__pitch()
{
    return this->pitch_;
}

double gps_slam_navigation::points::QuaternionPoint::get__yaw()
{
    return this->yaw_;
}

double gps_slam_navigation::points::QuaternionPoint::get__x()
{
    return (float)this->x_;
}

void gps_slam_navigation::points::QuaternionPoint::set__x(const double &x)
{
    this->x_ = x;
}

double gps_slam_navigation::points::QuaternionPoint::get__y()
{
    return (float)this->y_;
}

void gps_slam_navigation::points::QuaternionPoint::set__y(const double &y)
{
    this->y_ = y;
}

double gps_slam_navigation::points::QuaternionPoint::get__z()
{
    return (float)this->z_;
}

void gps_slam_navigation::points::QuaternionPoint::set__z(const double &z)
{
    this->z_ = z;
}

double gps_slam_navigation::points::QuaternionPoint::get__w()
{
    return (float)this->w_;
}

void gps_slam_navigation::points::QuaternionPoint::set__w(const double &w)
{
    this->w_ = w;
}

double gps_slam_navigation::points::QuaternionPoint::degree_to_radian(double degree)
{
    double_radian_ = degree * M_PI / 180;
    return double_radian_;
}

double gps_slam_navigation::points::QuaternionPoint::get__height()
{
    height_ = sin(double_radian_) * qr_distance_;
    return height_;
}

double gps_slam_navigation::points::QuaternionPoint::get__base_line()
{
    double temp_height = height_;
    double temp_qr_distance = qr_distance_;

    temp_height = pow(temp_height, 2);
    temp_qr_distance = pow(temp_qr_distance, 2);

    base_line_ = temp_height + temp_qr_distance;
    base_line_ = sqrt(base_line_);

    return base_line_;
}

void gps_slam_navigation::points::QuaternionPoint::euler_angle_to_quaternion(double yaw, double pitch, double roll)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    euler_x_ = sr * cp * cy - cr * sp * sy;

    euler_y_ = cr * sp * cy + sr * cp * sy;

    euler_z_ = cr * cp * sy - sr * sp * cy;

    euler_w_ = cr * cp * cy + sr * sp * sy;
}

double gps_slam_navigation::points::QuaternionPoint::get__euler_w()
{
    return euler_w_;
}

double gps_slam_navigation::points::QuaternionPoint::get__euler_z()
{
    return euler_z_;
}

double gps_slam_navigation::points::QuaternionPoint::get__euler_x()
{
    return euler_x_;
}

double gps_slam_navigation::points::QuaternionPoint::get_euler_y()
{
    return euler_y_;
}