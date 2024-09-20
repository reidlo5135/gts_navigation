#include "gts_navigator/gts_navigator.hxx"

gts_navigator::Navigator::Navigator()
    : Node(RCL_NODE_NAME),
      slam_waypoints_list_index_(GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX),
      is_navigation_resumed_flag_(false)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] node has been created", RCL_NODE_NAME);
        RCLCPP_LINE_INFO();
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "failed to create %s node", RCL_NODE_NAME);
        RCLCPP_LINE_ERROR();
        exit(RCL_STOP_FLAG);
    }

    this->navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this->node_,
        RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);

    this->flag_rcl_connections(RCL_ACTION_CLIENT_FLAG, RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);

    this->gps_goal_waypoints_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_waypoints_subscription_opts;
    gps_waypoints_subscription_opts.callback_group = gps_goal_waypoints_subscription_cb_group_;

    this->gps_goal_waypoints_subscription_ = this->node_->create_subscription<gts_navigation_msgs::msg::GoalWaypoints>(
        RCL_GOAL_WAYPOINTS_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gts_navigator::Navigator::gps_goal_waypoints_subscription_cb, this, _1),
        gps_waypoints_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_GOAL_WAYPOINTS_STAMPED_TOPIC);

    this->gps_slam_conversion_client_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->gps_slam_conversion_client_ = this->node_->create_client<gps_slam_conversion_msgs::srv::Conversion>(
        RCL_GPS_SLAM_CONVERSION_SERVICE_SERVER_NAME,
        rmw_qos_profile_services_default,
        this->gps_slam_conversion_client_cb_group_);

    this->flag_rcl_connections(RCL_SERVICE_CLIENT_FLAG, RCL_GPS_SLAM_CONVERSION_SERVICE_SERVER_NAME);

    this->navigate_to_pose_goal_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions navigate_to_pose_goal_status_subscription_opts;
    navigate_to_pose_goal_status_subscription_opts.callback_group = navigate_to_pose_goal_status_subscription_cb_group_;

    this->navigate_to_pose_goal_status_subscription_ = this->node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gts_navigator::Navigator::navigate_to_pose_goal_status_subscription_cb, this, _1),
        navigate_to_pose_goal_status_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC);

    this->gts_navigation_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions gts_navigation_status_publisher_opts;
    gts_navigation_status_publisher_opts.callback_group = gts_navigation_status_publisher_cb_group_;

    this->gts_navigation_status_publisher_ = this->node_->create_publisher<gts_navigation_msgs::msg::NavigationStatusStamped>(
        RCL_NAVIGATION_STATUS_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)));

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_NAVIGATION_STATUS_STAMPED_TOPIC);

    this->gts_navigation_result_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions gts_navigation_result_publisher_opts;
    gts_navigation_result_publisher_opts.callback_group = gts_navigation_result_publisher_cb_group_;

    this->gts_navigation_result_publisher_ = this->node_->create_publisher<gts_navigation_msgs::msg::NavigationResultStamped>(
        RCL_NAVIGATION_RESULT_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)));

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_NAVIGATION_RESULT_STAMPED_TOPIC);

    this->gts_navigation_control_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gts_navigation_control_subscription_opts;
    gts_navigation_control_subscription_opts.callback_group = this->gts_navigation_control_subscription_cb_group_;

    this->gts_navigation_control_subscription_ = this->node_->create_subscription<gts_navigation_msgs::msg::NavigationControl>(
        RCL_GTS_NAVIGATION_CONTROL_SUBSCRIPTION_NAME,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gts_navigator::Navigator::gts_navigation_control_subscription_cb, this, _1),
        gts_navigation_control_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_GTS_NAVIGATION_CONTROL_SUBSCRIPTION_NAME);

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
    RCLCPP_INFO(this->node_->get_logger(), "RCL [%s - %s] created...", connection_type, connection_name);
    RCLCPP_LINE_INFO();
}

std_msgs::msg::Header gts_navigator::Navigator::build_header()
{
    std::chrono::_V2::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::_V2::system_clock::duration current_time_duration = current_time.time_since_epoch();

    const int32_t &current_time_sec = std::chrono::duration_cast<std::chrono::seconds>(current_time_duration).count();
    const int32_t &current_time_nanosec = current_time_sec % 1000000000;

    builtin_interfaces::msg::Time::UniquePtr time = std::make_unique<builtin_interfaces::msg::Time>();
    time->set__sec(current_time_sec);
    time->set__nanosec(current_time_nanosec);

    std_msgs::msg::Header::UniquePtr header = std::make_unique<std_msgs::msg::Header>();
    header->set__frame_id(RCL_HEADER_FRAME_ID);

    const builtin_interfaces::msg::Time &&time_moved = std::move(*time);
    header->set__stamp(time_moved);

    const std_msgs::msg::Header &&header_moved = std::move(*header);

    return header_moved;
}

void gts_navigator::Navigator::gps_goal_waypoints_subscription_cb(const gts_navigation_msgs::msg::GoalWaypoints::SharedPtr gps_goal_waypoints_cb_data)
{
    bool is_gps_goal_waypoints_list_empty = gps_goal_waypoints_cb_data->goal_waypoints_list.empty();

    if (is_gps_goal_waypoints_list_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "gps_goal_waypoints_list is empty");
        RCLCPP_LINE_ERROR();
        return;
    }

    bool is_slam_waypoints_list_already_exists = !(this->slam_waypoints_list_.empty());

    if (is_slam_waypoints_list_already_exists)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "slam_waypoints_list has already scheduled");
        RCLCPP_LINE_ERROR();
        return;
    }

    RCLCPP_INFO(this->node_->get_logger(), "===== gps_goal_waypoints_list cb received =====");
    RCLCPP_LINE_INFO();

    this->gps_slam_conversion_service_req(gps_goal_waypoints_cb_data->goal_waypoints_list);
    this->slam_waypoints_list_size_ = slam_waypoints_list_.size();

    RCLCPP_INFO(this->node_->get_logger(), "slam_waypoints_list schedule size : [%d]", slam_waypoints_list_size_);
    RCLCPP_LINE_INFO();

    this->navigate_to_pose_send_goal();
}

void gts_navigator::Navigator::gps_slam_conversion_service_req(std::vector<sensor_msgs::msg::NavSatFix> gps_goal_waypoints_list)
{
    bool is_goal_waypoints_list_empty = gps_goal_waypoints_list.empty();

    if (is_goal_waypoints_list_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "[%s] request goal_waypoints_list is empty", RCL_GPS_SLAM_CONVERSION_SERVICE_SERVER_NAME);
        RCLCPP_LINE_ERROR();
        return;
    }

    RCLCPP_INFO(this->node_->get_logger(), "===== gps_goal_waypoints_list request received =====");
    RCLCPP_LINE_INFO();

    gps_slam_conversion_msgs::srv::Conversion::Request::SharedPtr request = std::make_shared<gps_slam_conversion_msgs::srv::Conversion::Request>();

    std_msgs::msg::String::UniquePtr conversion_target = std::make_unique<std_msgs::msg::String>();
    conversion_target->set__data("SLAM");
    const std_msgs::msg::String &conversion_target_moved = std::move(*conversion_target);

    request->set__conversion_target(conversion_target_moved);
    request->set__gps_request_list(gps_goal_waypoints_list);

    std::shared_future<std::shared_ptr<gps_slam_conversion_msgs::srv::Conversion_Response>> result_future = this->gps_slam_conversion_client_->async_send_request(request);
    std::future_status result_future_status = result_future.wait_for(std::chrono::seconds(5));

    if (result_future_status == std::future_status::ready)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] Received response", RCL_GPS_SLAM_CONVERSION_SERVICE_SERVER_NAME);
        RCLCPP_LINE_INFO();

        gps_slam_conversion_msgs::srv::Conversion::Response::SharedPtr response = result_future.get();
        std::vector<geometry_msgs::msg::Pose> slam_pose_response_list = response->slam_pose_response_list;
        for (const geometry_msgs::msg::Pose &pose : slam_pose_response_list)
        {
            geometry_msgs::msg::Point point = pose.position;
            this->slam_waypoints_list_.push_back(point);
        }

        const size_t &slam_waypoints_list_size = this->slam_waypoints_list_.size();
        const size_t &slam_pose_response_list_size = slam_pose_response_list.size();

        if (slam_waypoints_list_size == slam_pose_response_list_size)
        {
            RCLCPP_INFO(this->node_->get_logger(), "[%s] Handling response is successful", RCL_GPS_SLAM_CONVERSION_SERVICE_SERVER_NAME);
            RCLCPP_LINE_INFO();
            return;
        }
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "[%s] Failed to request", RCL_GPS_SLAM_CONVERSION_SERVICE_SERVER_NAME);
        RCLCPP_LINE_ERROR();
        return;
    }
}

void gts_navigator::Navigator::navigate_to_pose_goal_status_subscription_cb(const action_msgs::msg::GoalStatusArray::SharedPtr goal_status_array_cb_data)
{
    bool is_goal_status_array_empty = goal_status_array_cb_data->status_list.empty();

    if (is_goal_status_array_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "navigate_to_pose goal status callback status list is empty...");
        RCLCPP_LINE_ERROR();
    }
    else
    {
        const action_msgs::msg::GoalStatus &goal_status = goal_status_array_cb_data->status_list.back();
        const uint8_t &goal_status_code = goal_status.status;

        RCLCPP_INFO(
            this->node_->get_logger(),
            "navigate_to_pose status callback\n\twaypoints index : [%d]\n\twaypoints size : [%d]\n\tgoal_status_code : [%d]",
            slam_waypoints_list_index_ + 1,
            slam_waypoints_list_size_,
            goal_status_code);
        RCLCPP_LINE_INFO();

        bool is_slam_waypoints_ended = slam_waypoints_list_index_ == (slam_waypoints_list_size_ - 1);

        if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_STARTED)
        {
            if (this->is_navigation_resumed_flag_ == false && slam_waypoints_list_index_ == RCL_DEFAULT_INT)
            {
                RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback navigation started =====");
                RCLCPP_LINE_INFO();

                this->gts_navigation_status_publish(RCL_DEFAULT_INT);
            }
            else
            {
                RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback goal started =====");
                RCLCPP_LINE_INFO();

                this->gts_navigation_status_publish(goal_status_code);
            }
        }

        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED)
        {
            RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback goal succeeded =====");
            RCLCPP_LINE_INFO();

            if (is_slam_waypoints_ended)
            {
                RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback goal waypoints ended =====");
                RCLCPP_LINE_INFO();

                this->gts_navigation_result_publish(goal_status_code);

                slam_waypoints_list_index_ = GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX;
                slam_waypoints_list_size_ = GOAL_WAYPOINTS_VECTOR_DEFAULT_SIZE;
                slam_waypoints_list_.clear();

                this->is_navigation_resumed_flag_ == false;
                this->navigate_to_pose_client_->async_cancel_all_goals();
            }
            else
            {
                slam_waypoints_list_index_++;

                RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback will proceed next [%d] goal =====", slam_waypoints_list_index_);
                RCLCPP_LINE_INFO();

                this->is_navigation_resumed_flag_ == false;
                this->gts_navigation_status_publish(goal_status_code);
                this->navigate_to_pose_send_goal();
            }
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_ABORTED)
        {
            RCLCPP_WARN(this->node_->get_logger(), "!!!!! navigate_to_pose status callback goal aborted !!!!!");
            RCLCPP_LINE_WARN();

            RCLCPP_WARN(this->node_->get_logger(), "navigate_to_pose status callback will proceed previous [%d]st goal", slam_waypoints_list_index_);
            RCLCPP_LINE_WARN();

            this->gts_navigation_status_publish(goal_status_code);

            RCLCPP_WARN(this->node_->get_logger(), "!!!!! Goal Aborted Goal Canceling !!!!!");
            RCLCPP_LINE_WARN();
            this->navigate_to_pose_client_->async_cancel_all_goals();

            RCLCPP_WARN(this->node_->get_logger(), "!!!!! Goal Aborted Goal Retrying !!!!!");
            RCLCPP_LINE_WARN();
            this->navigate_to_pose_send_goal();
        }
        else if (goal_status_code == 3)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "!!!!! Goal Canceling !!!!!");
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_CANCELED)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "!!!!! Goal Canceled !!!!!");
            RCLCPP_LINE_ERROR();
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "navigate_to_pose status callback unknown goal result code : [%d]", goal_status_code);
            RCLCPP_LINE_ERROR();
            return;
        }
    }
}

void gts_navigator::Navigator::gts_navigation_status_publish(const int &goal_status_code)
{
    gts_navigation_msgs::msg::NavigationStatusStamped::UniquePtr navigation_status_stamped = std::make_unique<gts_navigation_msgs::msg::NavigationStatusStamped>();
    std_msgs::msg::Header built_header = this->build_header();

    navigation_status_stamped->set__header(built_header);
    navigation_status_stamped->set__status_code(goal_status_code);
    navigation_status_stamped->set__current_goal_index(slam_waypoints_list_index_);

    const gts_navigation_msgs::msg::NavigationStatusStamped &&navigation_status_moved = std::move(*navigation_status_stamped);
    this->gts_navigation_status_publisher_->publish(navigation_status_moved);

    RCLCPP_INFO(this->node_->get_logger(), "navigation_status_stamped published");
    RCLCPP_LINE_INFO();
}

void gts_navigator::Navigator::gts_navigation_result_publish(const int &goal_status_code)
{
    gts_navigation_msgs::msg::NavigationResultStamped::UniquePtr navigation_result_stamped = std::make_unique<gts_navigation_msgs::msg::NavigationResultStamped>();
    std_msgs::msg::Header built_header = this->build_header();

    navigation_result_stamped->set__header(built_header);
    navigation_result_stamped->set__result_code(goal_status_code);
    navigation_result_stamped->set__result_index(slam_waypoints_list_index_);

    const gts_navigation_msgs::msg::NavigationResultStamped &&navigation_result_stamped_moved = std::move(*navigation_result_stamped);
    this->gts_navigation_result_publisher_->publish(navigation_result_stamped_moved);

    RCLCPP_INFO(this->node_->get_logger(), "navigation_result_stamped published");
    RCLCPP_LINE_INFO();
}

void gts_navigator::Navigator::gts_navigation_control_subscription_cb(const gts_navigation_msgs::msg::NavigationControl::SharedPtr gts_navigation_control_cb_data)
{
    const bool &is_control_cancel_navigation = (gts_navigation_control_cb_data->cancel_navigation == true);
    const bool &is_control_resume_navigation = (gts_navigation_control_cb_data->resume_navigation == true);

    if (is_control_cancel_navigation && is_control_resume_navigation)
    {
        return;
    }
    else if (is_control_cancel_navigation && !is_control_resume_navigation)
    {
        bool is_slam_waypoints_list_empty = (this->slam_waypoints_list_.empty()) || (this->slam_waypoints_list_size_ == RCL_DEFAULT_INT);

        RCLCPP_INFO(this->node_->get_logger(), "gts_navigation_control_subscription is_slam_waypoints_list_empty : [%d]", is_slam_waypoints_list_empty);
        RCLCPP_LINE_INFO();

        if (is_slam_waypoints_list_empty)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "gts_navigation_control_subscription waypoints_list is empty...");
            RCLCPP_LINE_ERROR();
            return;
        }
        else
        {
            RCLCPP_INFO(this->node_->get_logger(), "gts_navigation_control_subscription trying to cancel all goals...");
            RCLCPP_LINE_INFO();

            this->navigate_to_pose_client_->async_cancel_all_goals();

            this->gts_navigation_status_publish(RCL_NAVIGATE_TO_POSE_GOAL_STOPPED);
        }
    }
    else if (is_control_resume_navigation && !is_control_cancel_navigation)
    {
        RCLCPP_INFO(
            this->node_->get_logger(),
            "gts_navigation_control_subscription trying to resume navigation...\n\tcurrent waypoints_index : [%d]\n\twaypoints_list size : [%d]",
            this->slam_waypoints_list_index_, this->slam_waypoints_list_size_);
        RCLCPP_LINE_INFO();

        this->is_navigation_resumed_flag_ = true;
        this->navigate_to_pose_send_goal();
        this->gts_navigation_status_publish(RCL_NAVIGATE_TO_POSE_GOAL_RESUMED);
    }
    else
    {
        return;
    }
}

void gts_navigator::Navigator::navigate_to_pose_send_goal()
{
    bool is_slam_waypoints_list_empty = this->slam_waypoints_list_.empty();

    if (is_slam_waypoints_list_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "navigate_to_pose send goal slam_waypoints_list is empty...");
        RCLCPP_LINE_ERROR();
        return;
    }

    const double &slam_x = this->slam_waypoints_list_[slam_waypoints_list_index_].x;
    const double &slam_y = this->slam_waypoints_list_[slam_waypoints_list_index_].y;

    bool is_navigate_to_pose_server_ready = this->navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!is_navigate_to_pose_server_ready)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "navigate_to_pose action server is not available after waiting... vacate SLAM waypoints_list");
        RCLCPP_LINE_ERROR();
        this->slam_waypoints_list_.clear();
        this->slam_waypoints_list_index_ = RCL_DEFAULT_INT;
        this->slam_waypoints_list_size_ = RCL_DEFAULT_INT;
        return;
    }

    RCLCPP_INFO(
        this->node_->get_logger(),
        "navigate_to_pose action server is ready\n\tslam x : [%f]\n\tslam y : [%f]",
        slam_x,
        slam_y);
    RCLCPP_LINE_INFO();

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
        this->node_->get_logger(),
        "navigate_to_pose goal sent\n\tpose_x : [%f]\n\tpose_y : [%f]\n\torien_z: [%f]\n\torien_w : [%f]",
        navigate_to_pose_goal_->pose.pose.position.x,
        navigate_to_pose_goal_->pose.pose.position.y,
        navigate_to_pose_goal_->pose.pose.orientation.z,
        navigate_to_pose_goal_->pose.pose.orientation.w);
    RCLCPP_LINE_INFO();
}

void gts_navigator::Navigator::navigate_to_pose_goal_response_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "navigate_to_pose goal response callback was rejected by [%s] server", RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);
        RCLCPP_LINE_ERROR();
    }
    else
    {
        RCLCPP_INFO(this->node_->get_logger(), "navigate_to_pose goal response callback has been accepted by [%s] server, waiting for result...", RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);
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