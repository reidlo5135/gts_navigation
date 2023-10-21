#ifndef GTS_NAVIGATOR__HXX
#define GTS_NAVIGATOR__HXX

#include "gts_navigator/utils.hxx"

static constexpr const double &SLAM_MINIMUM_X = 0.0;
static constexpr const double &SLAM_MINIMUM_Y = 0.0;
static constexpr const double &SLAM_MAXIMUM_X = 37.46598;
static constexpr const double &SLAM_MAXIMUM_Y = 127.123412;

namespace gts_navigator
{
    class Navigator final : public rclcpp::Node
    {
    private:
        std::vector<geometry_msgs::msg::Point> goal_waypoints_list_;
        size_t goal_waypoints_list_size_;
        int slam_waypoints_list_index_;
        std::vector<geometry_msgs::msg::Point> slam_waypoints_list_;
        size_t slam_waypoints_list_size_;

        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::CallbackGroup::SharedPtr goal_waypoints_subscription_cb_group_;
        rclcpp::Subscription<gts_navigation_msgs::msg::GoalWaypoints>::SharedPtr goal_waypoints_subscription_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_goal_handle_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal::SharedPtr navigate_to_pose_goal_;
        rclcpp::CallbackGroup::SharedPtr navigate_to_pose_goal_status_subscription_cb_group_;
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_to_pose_goal_status_subscription_;

        rclcpp::CallbackGroup::SharedPtr gts_navigation_status_publisher_cb_group_;
        rclcpp::Publisher<gts_navigation_msgs::msg::NavigationStatusStamped>::SharedPtr gts_navigation_status_publisher_;
        rclcpp::CallbackGroup::SharedPtr gts_navigation_result_publisher_cb_group_;
        rclcpp::Publisher<gts_navigation_msgs::msg::NavigationResultStamped>::SharedPtr gts_navigation_result_publisher_;

        void flag_rcl_connections(const char *connection_type, const char *connection_name);
        void goal_waypoints_subscription_cb(const gts_navigation_msgs::msg::GoalWaypoints::SharedPtr goal_waypoints_cb_data);
        void navigate_to_pose_goal_status_subscription_cb(const action_msgs::msg::GoalStatusArray::SharedPtr goal_status_array_cb_data);
        void gts_navigation_status_publish(const gts_navigation_msgs::msg::NavigationStatusStamped::UniquePtr &gps_navigation_status, const int &goal_status_code);
        void gts_navigation_result_publish(const gts_navigation_msgs::msg::NavigationResultStamped::UniquePtr &gps_navigation_result, const int &goal_status_code);
        void navigate_to_pose_send_goal();
        void navigate_to_pose_goal_response_cb(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future);
        void navigate_to_pose_feedback_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
        void navigate_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &wrapped_result);

    public:
        explicit Navigator();
        virtual ~Navigator();
        static void signal_handler(int signal_input);
    };
}

#endif