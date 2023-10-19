#ifndef GPS_TO_SLAM__HXX
#define GPS_TO_SLAM__HXX

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "math/math.hxx"
#include "math/ktm_math.hxx"
#include "math/gps_math.hxx"
#include "math/common_math.hxx"
#include "math/slam_point_generator.hxx"

#include "points/points.hxx"
#include "points/gts_point.hxx"
#include "points/ktm_point.hxx"
#include "points/slam_point.hxx"
#include "points/quaternion_point.hxx"

#include "gps_slam_converter/utils.hxx"

namespace gps_slam_converter
{
    namespace conversions
    {
        class GPSToSLAM final
        {
        private:

        public:
            explicit GPSToSLAM();
            virtual ~GPSToSLAM();
            geometry_msgs::msg::Pose::SharedPtr convert(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data, geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data);
        };
    }
}

#endif