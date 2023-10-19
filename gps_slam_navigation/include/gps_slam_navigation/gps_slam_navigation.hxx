/**
 * @file gps_slam_navigation.hxx
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-07-31
 * @brief header file for gps_slam_navigation.cpp
 */

#ifndef GPS_SLAM_NAVIGATION__HXX
#define GPS_SLAM_NAVIGATION__HXX

#include "math/math.hxx"
#include "math/gts_math.hxx"
#include "math/gps_math.hxx"
#include "math/ktm_math.hxx"
#include "math/slam_math.hxx"
#include "math/common_math.hxx"
#include "math/slam_point_generator.hxx"

#include "points/points.hxx"
#include "points/gts_point.hxx"
#include "points/slam_point.hxx"
#include "points/quaternion_point.hxx"

#include "gps_slam_navigation/utils.hxx"

/**
 * @namespace gps_slam_navigation
 * @brief namespace for define CommonMath, Navigator classes
 */

namespace gps_slam_navigation
{
    namespace navigation
    {
        /**
         * @class gps_slam_navigation::Navigator
         * @brief final class for implements gps_slam_navigation by extends rclcpp::Node
         */
        class Navigator final : public rclcpp::Node
        {
        private:
            /**
             * @brief int field instance for gps_goal_waypoints_list_'s index
             */
            int slam_goal_waypoints_list_index_;

            size_t slam_goal_waypoints_list_size_;

            int slam_goal_waypoints_scenario_list_index_;

            /**
             * @brief double field instance for gps_goal_waypoints
             */
            std::vector<geometry_msgs::msg::Pose> gps_goal_waypoints_list_;

            std::vector<gps_slam_navigation::points::SLAMPoint> slam_goal_waypoints_list_;

            /**
             * @brief int field instance for gps_goal_waypoints_list_'s size
             */
            size_t gps_goal_waypoints_list_size_;

            std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_;

            std::shared_ptr<gps_slam_navigation::points::GTSPoint> gts_point_;

            std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point_;

            std::shared_ptr<gps_slam_navigation::points::QuaternionPoint> quaternion_point_;

            /**
             * @brief shared pointer for gps_slam_navigation::math::CommonMath
             */
            std::shared_ptr<gps_slam_navigation::math::CommonMath> common_math_;

            std::shared_ptr<gps_slam_navigation::math::GTSMath> gts_math_;

            std::shared_ptr<gps_slam_navigation::math::GPSMath> gps_math_;

            std::shared_ptr<gps_slam_navigation::math::SLAMMath> slam_math_;

            std::shared_ptr<gps_slam_navigation::math::KTMMath> ktm_math_;

            std::shared_ptr<gps_slam_navigation::math::SLAMPointGenerator> slam_point_generator_;

            /**
             * @brief shared pointer for rclcpp::Node
             */
            rclcpp::Node::SharedPtr rcl_node_ptr_;

            /**
             * @brief shared pointer for rclcpp_action::Client<nav2_msgs::action::NavigateToPose>
             */
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;

            /**
             * @brief shared pointer for rclcpp::CallbackGroup which indicates gps_subscription_;
             */
            rclcpp::CallbackGroup::SharedPtr gps_subscription_cb_group_;

            /**
             * @brief shared pointer for rclcpp::Subscription<sensor_msgs::msg::NavSatFix>
             */
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;

            /**
             * @brief shared pointer for rclcpp::CallbackGroup which indicates rcl_gps_goal_waypoints_stamped_ptr_;
             */
            rclcpp::CallbackGroup::SharedPtr gps_waypoints_subscription_cb_group_;

            /**
             * @brief shared pointer for rclcpp::Subscription<gps_navigation_msgs::msg::GoalWayPointsStamped>
             */
            rclcpp::Subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>::SharedPtr gps_waypoints_subscription_;

            rclcpp::CallbackGroup::SharedPtr slam_pose_subscription_subscription_cb_group;

            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slam_pose_subscription_;

            rclcpp::CallbackGroup::SharedPtr gps_navigation_status_publisher_cb_group_;

            rclcpp::Publisher<gps_navigation_msgs::msg::NavigationStatusStamped>::SharedPtr gps_navigation_status_publisher_;

            rclcpp::CallbackGroup::SharedPtr gps_navigation_result_publisher_cb_group_;

            rclcpp::Publisher<gps_navigation_msgs::msg::NavigationResultStamped>::SharedPtr gps_navigation_result_publisher_;

            rclcpp::CallbackGroup::SharedPtr navigate_to_pose_goal_status_subscription_cb_group_;

            rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_to_pose_goal_status_subscription_;

            /**
             * @brief function for flag established RCL connections
             * @return void
             */
            void flag_rcl_connections(const char *connection_type, const char *connection_name);

            /**
             * @brief function for handle subscription callback from rclcpp::Subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>
             * @param gps_waypoints_cb_data const gps_navigation_msgs::msg::GoalWaypointsStamped::SharedPtr
             * @return void
             */
            void gps_waypoints_subscription_cb(const gps_navigation_msgs::msg::GoalWaypointsStamped::SharedPtr gps_waypoints_cb_data);

            /**
             * @brief function for handle subscription callback from rclcpp::Subscription<sensor_msgs::msg::NavSatFix>
             * @param gps_cb_data cosnt sensor_msgs::msg::NavSatFix::SharedPtr
             * @return void
             */
            void gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data);

            geometry_msgs::msg::Pose::SharedPtr slam_pose_;

            void slam_pose_subscription_cb(const geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data);

            void gps_navigation_status_publish(const gps_navigation_msgs::msg::NavigationStatusStamped::UniquePtr &gps_navigation_status, const int &goal_status_code);

            void gps_navigation_result_publish(const gps_navigation_msgs::msg::NavigationResultStamped::UniquePtr &gps_navigation_result, const int &goal_status_code);

            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_goal_handle_;

            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal::SharedPtr navigate_to_pose_goal_;

            void navigate_to_pose_goal_status_subscription_cb(const action_msgs::msg::GoalStatusArray::SharedPtr goal_status_array_cb_data);

            /**
             * @brief function for send goal to rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
             * @return void
             */
            void navigate_to_pose_send_goal();

            /**
             * @brief function for handle response callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
             * @param future std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>
             * @return void
             */
            void navigate_to_pose_goal_response_cb(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future);

            /**
             * @brief function for handle feedback callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
             * @param goal_handle const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
             * @param feedback const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
             * @return void
             */
            void navigate_to_pose_feedback_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

            /**
             * @brief function for handle wrapped_result callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
             * @param wrapped_result const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &
             * @return void
             */
            void navigate_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &wrapped_result);

        public:
            /**
             * create a new this class' instance
             * @brief default constructor
             */
            explicit Navigator();

            /**
             * destroy this class' instance
             * @brief default destructor
             */
            virtual ~Navigator();

            /**
             * @brief function for handle signal_input when program exit
             * @param signal_input The signal_input of input
             * @return void
             * @see signal_input.h
             */
            static void signal_handler(int signal_input);
        };
    }
}

#endif