#include "gts_navigator/gts_navigator.hxx"

gts_navigator::Navigator::Navigator()
    : Node(RCL_NODE_NAME),
      slam_waypoints_list_index_(GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX)
{
    this->node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    RCLCPP_INFO(this->node_ptr_->get_logger(), "[%s] has been started...", RCL_NODE_NAME);
    RCLCPP_LINE_INFO();

    this->navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this->node_ptr_,
        RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);

    this->flag_rcl_connections(RCL_ACTION_CLIENT_FLAG, RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);

    this->goal_waypoints_subscription_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_waypoints_subscription_opts;
    gps_waypoints_subscription_opts.callback_group = goal_waypoints_subscription_cb_group_;

    this->goal_waypoints_subscription_ = this->node_ptr_->create_subscription<gts_navigation_msgs::msg::GoalWaypoints>(
        RCL_GOAL_WAYPOINTS_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gts_navigator::Navigator::goal_waypoints_subscription_cb, this, _1),
        gps_waypoints_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_GOAL_WAYPOINTS_STAMPED_TOPIC);

    this->navigate_to_pose_goal_status_subscription_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions navigate_to_pose_goal_status_subscription_opts;
    navigate_to_pose_goal_status_subscription_opts.callback_group = navigate_to_pose_goal_status_subscription_cb_group_;

    this->navigate_to_pose_goal_status_subscription_ = this->node_ptr_->create_subscription<action_msgs::msg::GoalStatusArray>(
        RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gts_navigator::Navigator::navigate_to_pose_goal_status_subscription_cb, this, _1),
        navigate_to_pose_goal_status_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC);

    gts_navigation_status_publisher_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions gps_navigation_status_publisher_opts;
    gps_navigation_status_publisher_opts.callback_group = gts_navigation_status_publisher_cb_group_;

    gts_navigation_status_publisher_ = this->node_ptr_->create_publisher<gts_navigation_msgs::msg::NavigationStatusStamped>(
        RCL_NAVIGATION_STATUS_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)));

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_NAVIGATION_STATUS_STAMPED_TOPIC);

    gts_navigation_result_publisher_cb_group_ = this->node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions gps_navigation_result_publisher_opts;
    gps_navigation_result_publisher_opts.callback_group = gts_navigation_result_publisher_cb_group_;

    gts_navigation_result_publisher_ = this->node_ptr_->create_publisher<gts_navigation_msgs::msg::NavigationResultStamped>(
        RCL_NAVIGATION_RESULT_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)));

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_NAVIGATION_RESULT_STAMPED_TOPIC);

    this->navigate_to_pose_goal_ = std::make_shared<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal>();
}

gts_navigator::Navigator::~Navigator()
{
}

void gts_navigator::Navigator::signal_handler(int signal_input)
{
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "===== %s has been terminated with SIG [%d] =====", RCL_NODE_NAME, signal_input);
    signal(signal_input, SIG_IGN);
    exit(RCL_STOP_FLAG);
}

void gts_navigator::Navigator::flag_rcl_connections(const char *connection_type, const char *connection_name)
{
    RCLCPP_INFO(this->node_ptr_->get_logger(), "RCL [%s - %s] created...", connection_type, connection_name);
    RCLCPP_LINE_INFO();
}

void gts_navigator::Navigator::goal_waypoints_subscription_cb(const gts_navigation_msgs::msg::GoalWaypoints::SharedPtr goal_waypoints_cb_data)
{
    bool is_goal_waypoints_list_already_exists = !(this->goal_waypoints_list_.empty());

    if (is_goal_waypoints_list_already_exists)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "goal_waypoints_list has already scheduled");
        RCLCPP_LINE_ERROR();
        return;
    }

    this->goal_waypoints_list_ = goal_waypoints_cb_data->goal_waypoints_list;
    this->goal_waypoints_list_size_ = goal_waypoints_list_.size();

    RCLCPP_INFO(this->node_ptr_->get_logger(), "goal_waypoints_list schedule size : [%d]", goal_waypoints_list_size_);
    RCLCPP_LINE_INFO();

    for (const geometry_msgs::msg::Point &goal_waypoints : this->goal_waypoints_list_)
    {
        const double &goal_x = goal_waypoints.x;
        const double &goal_y = goal_waypoints.y;
        const double &heading = goal_waypoints.z;

        RCLCPP_INFO(
            this->node_ptr_->get_logger(), "goal_waypoints_list goal_waypoints_list\n\tx : [%f]\n\ty : [%f]\n\theading : [%f]",
            goal_x,
            goal_y,
            heading);
        RCLCPP_LINE_INFO();

        const double &slam_x = SLAM_MAXIMUM_X - goal_x;
        const double &slam_y = SLAM_MAXIMUM_Y - goal_y;

        RCLCPP_INFO(
            this->node_ptr_->get_logger(),
            "goal_waypoints_list slam_waypoints_list\n\tslam x : [%f]\n\tslam y : [%f]",
            slam_x,
            slam_y);
        RCLCPP_LINE_INFO();

        geometry_msgs::msg::Point::UniquePtr geometry_msgs_point = std::make_unique<geometry_msgs::msg::Point>();
        geometry_msgs_point->set__x(slam_x);
        geometry_msgs_point->set__y(slam_y);
        geometry_msgs_point->set__z(heading);

        const geometry_msgs::msg::Point &&geometry_msgs_point_moved = std::move(*geometry_msgs_point);
        this->slam_waypoints_list_.push_back(geometry_msgs_point_moved);
    }

    this->navigate_to_pose_send_goal();
}

void gts_navigator::Navigator::navigate_to_pose_goal_status_subscription_cb(const action_msgs::msg::GoalStatusArray::SharedPtr goal_status_array_cb_data)
{
    bool is_goal_status_array_empty = goal_status_array_cb_data->status_list.empty();

    if (is_goal_status_array_empty)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "navigate_to_pose goal status callback status list is empty...");
        RCLCPP_LINE_ERROR();
    }
    else
    {
        std::chrono::_V2::system_clock::time_point current_time = std::chrono::system_clock::now();
        std::chrono::_V2::system_clock::duration current_time_duration = current_time.time_since_epoch();

        const int32_t &current_time_sec = std::chrono::duration_cast<std::chrono::seconds>(current_time_duration).count();
        const int32_t &current_time_nanosec = current_time_sec % 1000000000;

        builtin_interfaces::msg::Time::UniquePtr builtin_interfaces_time = std::make_unique<builtin_interfaces::msg::Time>();
        builtin_interfaces_time->set__sec(current_time_sec);
        builtin_interfaces_time->set__nanosec(current_time_nanosec);

        std_msgs::msg::Header::UniquePtr std_msgs_header = std::make_unique<std_msgs::msg::Header>();
        std_msgs_header->set__frame_id(RCL_HEADER_FRAME_ID);

        const builtin_interfaces::msg::Time &&builtin_interfaces_time_moved = std::move(*builtin_interfaces_time);
        std_msgs_header->set__stamp(builtin_interfaces_time_moved);

        const std_msgs::msg::Header &&std_msgs_header_moved = std::move(*std_msgs_header);

        gts_navigation_msgs::msg::NavigationStatusStamped::UniquePtr gts_navigation_status = std::make_unique<gts_navigation_msgs::msg::NavigationStatusStamped>();
        gts_navigation_status->set__header(std_msgs_header_moved);

        gts_navigation_msgs::msg::NavigationResultStamped::UniquePtr gts_navigation_result = std::make_unique<gts_navigation_msgs::msg::NavigationResultStamped>();
        gts_navigation_result->set__header(std_msgs_header_moved);

        const action_msgs::msg::GoalStatus &goal_status = goal_status_array_cb_data->status_list.back();
        const uint8_t &goal_status_code = goal_status.status;

        RCLCPP_INFO(
            this->node_ptr_->get_logger(),
            "navigate_to_pose status callback\n\twaypoints index : [%d]\n\twaypoints size : [%d]\n\tgoal_status_code : [%d]",
            slam_waypoints_list_index_ + 1,
            slam_waypoints_list_size_,
            goal_status_code);
        RCLCPP_LINE_INFO();

        bool is_slam_waypoints_ended = slam_waypoints_list_index_ == (slam_waypoints_list_size_ - 1);

        if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_STARTED)
        {
            RCLCPP_INFO(this->node_ptr_->get_logger(), "===== navigate_to_pose status callback goal started =====");
            RCLCPP_LINE_INFO();

            this->gts_navigation_status_publish(gts_navigation_status, goal_status_code);
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED)
        {
            RCLCPP_INFO(this->node_ptr_->get_logger(), "===== navigate_to_pose status callback goal succeeded =====");
            RCLCPP_LINE_INFO();

            if (is_slam_waypoints_ended)
            {
                RCLCPP_INFO(this->node_ptr_->get_logger(), "===== navigate_to_pose status callback goal waypoints ended =====");
                RCLCPP_LINE_INFO();

                this->gts_navigation_result_publish(gts_navigation_result, goal_status_code);

                slam_waypoints_list_index_ = GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX;
                slam_waypoints_list_size_ = GOAL_WAYPOINTS_VECTOR_DEFAULT_SIZE;

                goal_waypoints_list_.clear();
                slam_waypoints_list_.clear();

                navigate_to_pose_client_->async_cancel_all_goals();
            }
            else
            {
                slam_waypoints_list_index_++;

                RCLCPP_INFO(this->node_ptr_->get_logger(), "===== navigate_to_pose status callback will proceed next [%d] goal =====", slam_waypoints_list_index_);
                RCLCPP_LINE_INFO();

                this->gts_navigation_status_publish(gts_navigation_status, goal_status_code);
                this->navigate_to_pose_send_goal();
            }
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_ABORTED)
        {
            RCLCPP_WARN(this->node_ptr_->get_logger(), "!!!!! navigate_to_pose status callback goal aborted !!!!!");
            RCLCPP_LINE_WARN();

            RCLCPP_WARN(this->node_ptr_->get_logger(), "navigate_to_pose status callback will proceed previous [%d]st goal", slam_waypoints_list_index_);
            RCLCPP_LINE_WARN();

            this->gts_navigation_status_publish(gts_navigation_status, goal_status_code);
            this->navigate_to_pose_send_goal();
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_CANCELED)
        {
            RCLCPP_ERROR(this->node_ptr_->get_logger(), "!!!!! navigate_to_pose status callback goal canceled !!!!!");
            RCLCPP_LINE_ERROR();

            this->gts_navigation_status_publish(gts_navigation_status, goal_status_code);
        }
        else
        {
            RCLCPP_ERROR(this->node_ptr_->get_logger(), "navigate_to_pose status callback unknown goal result code : [%d]", goal_status_code);
            RCLCPP_LINE_ERROR();
            return;
        }
    }
}

void gts_navigator::Navigator::gts_navigation_status_publish(const gts_navigation_msgs::msg::NavigationStatusStamped::UniquePtr &gps_navigation_status, const int &goal_status_code)
{
    bool is_gps_navigation_status_nullptr = gps_navigation_status == nullptr;

    if (is_gps_navigation_status_nullptr)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "navigation_status_stamped publishing failed with nullptr");
        return;
    }

    RCLCPP_INFO(this->node_ptr_->get_logger(), "navigation_status_stamped published");
    RCLCPP_LINE_INFO();

    gps_navigation_status->set__status_code(goal_status_code);
    gps_navigation_status->set__current_goal_index(slam_waypoints_list_index_);

    const gts_navigation_msgs::msg::NavigationStatusStamped &&rcl_navigation_status_stamped_ptr_moved = std::move(*gps_navigation_status);
    this->gts_navigation_status_publisher_->publish(rcl_navigation_status_stamped_ptr_moved);
}

void gts_navigator::Navigator::gts_navigation_result_publish(const gts_navigation_msgs::msg::NavigationResultStamped::UniquePtr &gps_navigation_result, const int &goal_status_code)
{
    bool is_gps_navigation_result_nullptr = gps_navigation_result == nullptr;

    if (is_gps_navigation_result_nullptr)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "navigation_result_stamped publishing failed with nullptr");
        return;
    }

    RCLCPP_INFO(this->node_ptr_->get_logger(), "navigation_resykt_stamped published");
    RCLCPP_LINE_INFO();

    gps_navigation_result->set__result_code(goal_status_code);
    gps_navigation_result->set__result_index(slam_waypoints_list_index_);

    const gts_navigation_msgs::msg::NavigationResultStamped &&rcl_navigation_result_stamped_ptr_moved = std::move(*gps_navigation_result);
    this->gts_navigation_result_publisher_->publish(rcl_navigation_result_stamped_ptr_moved);
}

void gts_navigator::Navigator::navigate_to_pose_send_goal()
{
    bool is_goal_waypoints_list_empty = this->goal_waypoints_list_.empty();

    if (is_goal_waypoints_list_empty)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "navigate_to_pose send goal_waypoints_list is empty...");
        RCLCPP_LINE_ERROR();
        return;
    }

    const double &slam_x = this->slam_waypoints_list_[slam_waypoints_list_index_].x;
    const double &slam_y = this->slam_waypoints_list_[slam_waypoints_list_index_].y;

    bool is_navigate_to_pose_server_ready = this->navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!is_navigate_to_pose_server_ready)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "navigate_to_pose action server is not available after waiting");
        RCLCPP_LINE_ERROR();
        return;
    }
    else
    {
        RCLCPP_INFO(
            this->node_ptr_->get_logger(),
            "navigate_to_pose action server is ready\n\tslam x : [%f]\n\tslam y : [%f]",
            slam_x,
            slam_y);
        RCLCPP_LINE_INFO();
    }

    geometry_msgs::msg::PoseStamped::UniquePtr geometry_msgs_pose_stamped = std::make_unique<geometry_msgs::msg::PoseStamped>();

    geometry_msgs::msg::Point::UniquePtr geometry_msgs_point = std::make_unique<geometry_msgs::msg::Point>();
    geometry_msgs_point->set__x(slam_x);
    geometry_msgs_point->set__y(slam_y);
    geometry_msgs_point->set__z(RCL_DEFAULT_DOUBLE);

    geometry_msgs::msg::Quaternion::UniquePtr geometry_msgs_quaternion = std::make_unique<geometry_msgs::msg::Quaternion>();
    geometry_msgs_quaternion->set__x(RCL_DEFAULT_DOUBLE);
    geometry_msgs_quaternion->set__y(RCL_DEFAULT_DOUBLE);
    geometry_msgs_quaternion->set__z(RCL_DEFAULT_DOUBLE);
    geometry_msgs_quaternion->set__w(RCL_DEFAULT_DOUBLE);

    geometry_msgs::msg::Pose::UniquePtr geometry_msgs_pose = std::make_unique<geometry_msgs::msg::Pose>();

    const geometry_msgs::msg::Point &&geometry_msgs_point_moved = std::move(*geometry_msgs_point);
    geometry_msgs_pose->set__position(geometry_msgs_point_moved);

    const geometry_msgs::msg::Quaternion &&geometry_msgs_quaternion_moved = std::move(*geometry_msgs_quaternion);
    geometry_msgs_pose->set__orientation(geometry_msgs_quaternion_moved);

    const geometry_msgs::msg::Pose &&geometry_msgs_pose_moved = std::move(*geometry_msgs_pose);
    geometry_msgs_pose_stamped->set__pose(geometry_msgs_pose_moved);

    const geometry_msgs::msg::PoseStamped &&geometry_msgs_pose_stamped_moved = std::move(*geometry_msgs_pose_stamped);
    this->navigate_to_pose_goal_->set__pose(geometry_msgs_pose_stamped_moved);

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions navigate_to_pose_send_goal_opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    navigate_to_pose_send_goal_opts.feedback_callback = std::bind(&gts_navigator::Navigator::navigate_to_pose_feedback_cb, this, _1, _2);
    navigate_to_pose_send_goal_opts.goal_response_callback = std::bind(&gts_navigator::Navigator::navigate_to_pose_goal_response_cb, this, _1);
    navigate_to_pose_send_goal_opts.result_callback = std::bind(&gts_navigator::Navigator::navigate_to_pose_result_cb, this, _1);

    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> navigate_to_pose_goal_future = navigate_to_pose_client_->async_send_goal(*navigate_to_pose_goal_, navigate_to_pose_send_goal_opts);
    this->navigate_to_pose_goal_handle_ = navigate_to_pose_goal_future.get();

    RCLCPP_INFO(
        this->node_ptr_->get_logger(),
        "navigate_to_pose goal sent\n\tpose_x : [%f]\n\tpose_y : [%f]\n\torien_z: [%f]\n\torien_w : [%f]",
        navigate_to_pose_goal_->pose.pose.position.x,
        navigate_to_pose_goal_->pose.pose.position.y,
        navigate_to_pose_goal_->pose.pose.orientation.z,
        navigate_to_pose_goal_->pose.pose.orientation.w);
    RCLCPP_LINE_INFO();
}

void gts_navigator::Navigator::navigate_to_pose_goal_response_cb(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
{
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle = future.get();

    if (!goal_handle)
    {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "navigate_to_pose goal response callback was rejected by [%s] server", RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);
        RCLCPP_LINE_ERROR();
    }
    else
    {
        RCLCPP_INFO(this->node_ptr_->get_logger(), "navigate_to_pose goal response callback has been accepted by [%s] server, waiting for result...", RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);
        RCLCPP_LINE_INFO();
    }
}

/**
 * @brief function for handle feedback callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @param goal_handle_ptr const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
 * @param feedback_ptr const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
 * @return void
 */
void gts_navigator::Navigator::navigate_to_pose_feedback_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_ptr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_ptr)
{
}

/**
 * @brief function for handle result callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @param result const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &
 * @return void
 */
void gts_navigator::Navigator::navigate_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &wrapped_result)
{
}