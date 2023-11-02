#include "gts_navigator/gts_navigator.hxx"

gts_navigator::Navigator::Navigator()
    : Node(RCL_NODE_NAME),
      is_gts_navigation_started_(false),
      start_time_(""),
      end_time_(""),
      slam_waypoints_list_index_(GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX)
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

    this->navigate_to_pose_goal_ = std::make_shared<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal>();

    this->gts_navigation_goal_cancel_service_server_ = this->node_->create_service<gts_navigation_msgs::srv::GoalCancel>(
        RCL_GTS_NAVIGATION_GOAL_CANCEL_SERVICE_SERVER_NAME,
        std::bind(&gts_navigator::Navigator::gts_navigation_goal_cancel_request_cb, this, _1, _2, _3));

    this->flag_rcl_connections(RCL_SERVICE_SERVER_FLAG, RCL_GTS_NAVIGATION_GOAL_CANCEL_SERVICE_SERVER_NAME);

    this->navigation_task_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions navigation_task_publisher_opts;
    navigation_task_publisher_opts.callback_group = this->navigation_task_publisher_cb_group_;
    this->navigation_task_publisher_ = this->node_->create_publisher<robot_status_msgs::msg::NavigationStatus>(
        RCL_GTS_NAVIGATION_TASK_STATUS_PUBLISHER_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        navigation_task_publisher_opts);

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_GTS_NAVIGATION_TASK_STATUS_PUBLISHER_TOPIC);
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
        std_msgs::msg::Header built_header = this->build_header();

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

        const char *current_time = this->get_current_time();

        if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_STARTED)
        {
            RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback goal started =====");
            RCLCPP_LINE_INFO();

            this->set__is_gts_navigation_started(true);
            this->set__start_time(current_time);
            this->gts_navigation_task_status_publish(RCL_GTS_NAVIGATION_TASK_STATUS_MOVE_FLAG, RCL_GTS_NAVIGATION_SUCCEEDED_FLAG);
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED)
        {
            RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback goal succeeded =====");
            RCLCPP_LINE_INFO();

            this->set__is_gts_navigation_started(false);
            this->set__end_time(current_time);
            this->gts_navigation_task_status_publish(RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG, RCL_GTS_NAVIGATION_SUCCEEDED_FLAG);

            if (is_slam_waypoints_ended)
            {
                RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback goal waypoints ended =====");
                RCLCPP_LINE_INFO();

                slam_waypoints_list_index_ = GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX;
                slam_waypoints_list_size_ = GOAL_WAYPOINTS_VECTOR_DEFAULT_SIZE;
                slam_waypoints_list_.clear();

                navigate_to_pose_client_->async_cancel_all_goals();
            }
            else
            {
                slam_waypoints_list_index_++;

                RCLCPP_INFO(this->node_->get_logger(), "===== navigate_to_pose status callback will proceed next [%d] goal =====", slam_waypoints_list_index_);
                RCLCPP_LINE_INFO();

                this->navigate_to_pose_send_goal();
            }
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_ABORTED)
        {
            RCLCPP_WARN(this->node_->get_logger(), "!!!!! navigate_to_pose status callback goal aborted !!!!!");
            RCLCPP_LINE_WARN();

            RCLCPP_WARN(this->node_->get_logger(), "navigate_to_pose status callback will proceed previous [%d]st goal", slam_waypoints_list_index_);
            RCLCPP_LINE_WARN();
            
            this->set__is_gts_navigation_started(false);
            this->set__end_time(current_time);
            this->gts_navigation_task_status_publish(RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG, RCL_GTS_NAVIGATION_FAILED_FLAG);

            this->navigate_to_pose_send_goal();
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_CANCELED)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "!!!!! navigate_to_pose status callback goal canceled !!!!!");
            RCLCPP_LINE_ERROR();

            this->set__is_gts_navigation_started(false);
            this->set__end_time(current_time);
            this->gts_navigation_task_status_publish(RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG, RCL_GTS_NAVIGATION_FAILED_FLAG);
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "navigate_to_pose status callback unknown goal result code : [%d]", goal_status_code);
            RCLCPP_LINE_ERROR();

            this->set__is_gts_navigation_started(false);
            this->set__end_time(current_time);
            this->gts_navigation_task_status_publish(RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG, RCL_GTS_NAVIGATION_FAILED_FLAG);
            return;
        }
    }
}

void gts_navigator::Navigator::navigate_to_pose_send_goal()
{
    bool is_slam_waypoints_list_empty = this->slam_waypoints_list_.empty();

    if (is_slam_waypoints_list_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "navigate_to_pose send goal_waypoints_list is empty...");
        RCLCPP_LINE_ERROR();
        return;
    }

    const double &slam_x = this->slam_waypoints_list_[slam_waypoints_list_index_].x;
    const double &slam_y = this->slam_waypoints_list_[slam_waypoints_list_index_].y;

    bool is_navigate_to_pose_server_ready = this->navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!is_navigate_to_pose_server_ready)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "navigate_to_pose action server is not available after waiting & clear goal_waypoints_list");
        RCLCPP_LINE_ERROR();
        this->slam_waypoints_list_.clear();
        return;
    }
    else
    {
        RCLCPP_INFO(
            this->node_->get_logger(),
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
        this->node_->get_logger(),
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

void gts_navigator::Navigator::gts_navigation_goal_cancel_request_cb(const std::shared_ptr<rmw_request_id_t> request_header, const gts_navigation_msgs::srv::GoalCancel::Request::SharedPtr request, const gts_navigation_msgs::srv::GoalCancel::Response::SharedPtr response)
{
    const bool &is_request_cancel_goals = (request->cancel_goals == true);

    RCLCPP_INFO(this->node_->get_logger(), "gts_navigation_goal_cancel server request : [%d]", is_request_cancel_goals);
    RCLCPP_LINE_INFO();

    if (is_request_cancel_goals)
    {
        RCLCPP_INFO(this->node_->get_logger(), "gts_navigation_goal_cancel server trying to cancel all goals");
        RCLCPP_LINE_INFO();

        const std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response_<std::allocator<void>>>> &cancel_goals_future = this->navigate_to_pose_client_->async_cancel_all_goals();
        const std::shared_ptr<action_msgs::srv::CancelGoal_Response> &cancel_goals_result = cancel_goals_future.get();

        const int8_t &cancel_goals_result_return_code = cancel_goals_result->return_code;
        RCLCPP_INFO(this->node_->get_logger(), "gts_navigation_goal_cancel server return code : [%d]", cancel_goals_result_return_code);
        RCLCPP_LINE_INFO();

        const std::vector<action_msgs::msg::GoalInfo, std::allocator<action_msgs::msg::GoalInfo>> &canceling_goals_vec = cancel_goals_result->goals_canceling;

        for (const action_msgs::msg::GoalInfo &canceling_goal_info : canceling_goals_vec)
        {
            const std::array<uint8_t, 16UL> &canceling_goal_info_uuid_arr = canceling_goal_info.goal_id.uuid;
            for (const uint8_t &canceling_goal_uuid : canceling_goal_info_uuid_arr)
            {
                RCLCPP_INFO(this->node_->get_logger(), "gts_navigation_goal_cancel server canceling_goal_info uuid : [%d]", canceling_goal_uuid);
                RCLCPP_LINE_INFO();
            }
        }
    }
    else
    {
        return;
    }
}

void gts_navigator::Navigator::gts_navigation_task_status_publish(const char *job, const char *status)
{
    bool is_started = this->get__is_gts_navigation_started();

    const char *current_time = this->get_current_time();
    robot_status_msgs::msg::NavigationStatus::UniquePtr navigation_status = std::make_unique<robot_status_msgs::msg::NavigationStatus>();
    navigation_status->set__job_group(job);
    navigation_status->set__job_kind(job);
    navigation_status->set__status(status);
    navigation_status->set__start_time(this->get__start_time());
    navigation_status->set__end_time(this->get__end_time());
    navigation_status->set__start_battery_level(RCL_DEFAULT_FLOAT);
    navigation_status->set__end_battery_level(RCL_DEFAULT_FLOAT);

    if (is_started == true)
    {
        navigation_status->set__is_started(true);
        navigation_status->set__is_ended(false);
    }
    else
    {
        navigation_status->set__is_started(false);
        navigation_status->set__is_ended(true);
    }

    const robot_status_msgs::msg::NavigationStatus &&navigation_status_moved = std::move(*navigation_status);

    this->navigation_task_publisher_->publish(navigation_status_moved);
}

const char *gts_navigator::Navigator::get_current_time()
{
    time_t raw_time;
    struct tm *time_info;

    time(&raw_time);
    time_info = localtime(&raw_time);

    char buffer[14];
    strftime(buffer, sizeof(buffer), "%y%m%d%H%M%S", time_info);

    return buffer;
}