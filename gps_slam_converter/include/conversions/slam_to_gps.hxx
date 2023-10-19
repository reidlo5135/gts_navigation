#ifndef SLAM_TO_GPS__HXX
#define SLAM_TO_GPS__HXX

#include <iostream>
#include <chrono>
#include <ctime>

#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
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
        class SLAMToGPS final
        {
        private:

        public:
            explicit SLAMToGPS();
            virtual ~SLAMToGPS();
            sensor_msgs::msg::NavSatFix::SharedPtr convert(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data, geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data);
        };
    }
}

#endif