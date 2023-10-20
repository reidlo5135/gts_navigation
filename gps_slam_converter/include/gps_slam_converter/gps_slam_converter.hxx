#ifndef GPS_SLAM_CONVERTER__HXX
#define GPS_SLAM_CONVERTER__HXX

#include <stdio.h>
#include <math.h>
#include <cmath>
#include <signal.h>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gps_navigation_msgs/srv/coordinate_conversion.hpp>

#include "math/math.hxx"
#include "points/points.hxx"

#include "gps_slam_converter/utils.hxx"

#include "conversions/gps_to_slam.hxx"
#include "conversions/slam_to_gps.hxx"

using std::placeholders::_1;
using std::placeholders::_2;

namespace gps_slam_converter
{
    namespace converter
    {
        class Converter final : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr rcl_node_;
            std::shared_ptr<gps_slam_converter::conversions::GPSToSLAM> gps_to_slam_converter;
            std::shared_ptr<gps_slam_converter::conversions::SLAMToGPS> slam_to_gps_converter;

            geometry_msgs::msg::Pose::SharedPtr slam_pose_;
            sensor_msgs::msg::NavSatFix::SharedPtr gps_;

            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slam_pose_subscription_;
            rclcpp::CallbackGroup::SharedPtr slam_pose_subscription_cb_group_;

            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
            rclcpp::CallbackGroup::SharedPtr gps_subscription_cb_group_;

            rclcpp::CallbackGroup::SharedPtr converted_slam_publisher_cb_group_;
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr converted_slam_publisher_;

            rclcpp::CallbackGroup::SharedPtr converted_gps_publisher_cb_group_;
            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr converted_gps_publisher_;

            rclcpp::Service<gps_navigation_msgs::srv::CoordinateConversion>::SharedPtr converter_service_;

            void slam_pose_subscription_cb(geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data);
            void gps_subscription_cb(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data);
            void converter_service_cb(const gps_navigation_msgs::srv::CoordinateConversion::Request::SharedPtr request, gps_navigation_msgs::srv::CoordinateConversion::Response::SharedPtr response);

        public:
            explicit Converter();
            virtual ~Converter();
            static void signal_handler(int signal_input);
        };
    }
}

#endif