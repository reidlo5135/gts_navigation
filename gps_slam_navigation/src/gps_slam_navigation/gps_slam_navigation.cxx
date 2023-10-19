/**
 * @file gps_slam_navigation.cxx
 * @author reidlo(naru5135@wavem.net)
 * @date 2023-07-31
 * @brief implementation file for rclcpp::Node
 */

/**
 * @brief include/gps_slam_navigation/gps_slam_navigation.hxx include area
 * @include gps_slam_navigation/gps_slam_navigation.hxx
 */

#include "gps_slam_navigation/gps_slam_navigation.hxx"

/**
 * create a new this class' instance and extends rclcpp::Node
 * @brief default constructor
 * @see rclcpp::Node
 */
gps_slam_navigation::navigation::Navigator::Navigator()
    : Node(RCL_NODE_NAME),
      slam_goal_waypoints_list_index_(RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX),
      slam_goal_waypoints_scenario_list_index_(RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX)
{
    slam_point_ = std::make_shared<gps_slam_navigation::points::SLAMPoint>();
    gts_point_ = std::make_shared<gps_slam_navigation::points::GTSPoint>();
    ktm_point_ = std::make_shared<gps_slam_navigation::points::KTMPoint>();
    quaternion_point_ = std::make_shared<gps_slam_navigation::points::QuaternionPoint>();

    common_math_ = std::make_shared<gps_slam_navigation::math::CommonMath>();
    gts_math_ = std::make_shared<gps_slam_navigation::math::GTSMath>();
    gps_math_ = std::make_shared<gps_slam_navigation::math::GPSMath>();
    slam_math_ = std::make_shared<gps_slam_navigation::math::SLAMMath>();
    ktm_math_ = std::make_shared<gps_slam_navigation::math::KTMMath>();

    slam_point_generator_ = std::make_shared<gps_slam_navigation::math::SLAMPointGenerator>();

    rcl_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "[%s] has been started...", RCL_NODE_NAME);
    RCLCPP_LINE_INFO();

    navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        rcl_node_ptr_,
        RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);

    this->flag_rcl_connections(RCL_ACTION_CLIENT_FLAG, RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);

    gps_subscription_cb_group_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_subscription_opts;
    gps_subscription_opts.callback_group = gps_subscription_cb_group_;

    gps_subscription_ = rcl_node_ptr_->create_subscription<sensor_msgs::msg::NavSatFix>(
        RCL_UBLOX_FIX_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gps_slam_navigation::navigation::Navigator::gps_subscription_cb, this, _1),
        gps_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_UBLOX_FIX_TOPIC);

    gps_waypoints_subscription_cb_group_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_waypoints_subscription_opts;
    gps_waypoints_subscription_opts.callback_group = gps_waypoints_subscription_cb_group_;

    gps_waypoints_subscription_ = rcl_node_ptr_->create_subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>(
        RCL_GPS_GOAL_WAYPOINTS_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gps_slam_navigation::navigation::Navigator::gps_waypoints_subscription_cb, this, _1),
        gps_waypoints_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_GPS_GOAL_WAYPOINTS_STAMPED_TOPIC);

    slam_pose_subscription_subscription_cb_group = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions slam_pose_subscription_opts;
    slam_pose_subscription_opts.callback_group = slam_pose_subscription_subscription_cb_group;

    slam_pose_subscription_ = rcl_node_ptr_->create_subscription<geometry_msgs::msg::Pose>(
        RCL_ROBOT_POSE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gps_slam_navigation::navigation::Navigator::slam_pose_subscription_cb, this, _1),
        slam_pose_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_ROBOT_POSE_TOPIC);

    gps_navigation_status_publisher_cb_group_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions gps_navigation_status_publisher_opts;
    gps_navigation_status_publisher_opts.callback_group = navigate_to_pose_goal_status_subscription_cb_group_;

    gps_navigation_status_publisher_ = rcl_node_ptr_->create_publisher<gps_navigation_msgs::msg::NavigationStatusStamped>(
        RCL_NAVIGATION_STATUS_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)));

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_NAVIGATION_STATUS_STAMPED_TOPIC);

    gps_navigation_result_publisher_cb_group_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions gps_navigation_result_publisher_opts;
    gps_navigation_result_publisher_opts.callback_group = gps_navigation_result_publisher_cb_group_;

    gps_navigation_result_publisher_ = rcl_node_ptr_->create_publisher<gps_navigation_msgs::msg::NavigationResultStamped>(
        RCL_NAVIGATION_RESULT_STAMPED_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)));

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_NAVIGATION_RESULT_STAMPED_TOPIC);

    navigate_to_pose_goal_ = std::make_shared<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal>();

    navigate_to_pose_goal_status_subscription_cb_group_ = rcl_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions navigate_to_pose_goal_status_subscription_opts;
    navigate_to_pose_goal_status_subscription_opts.callback_group = navigate_to_pose_goal_status_subscription_cb_group_;

    navigate_to_pose_goal_status_subscription_ = rcl_node_ptr_->create_subscription<action_msgs::msg::GoalStatusArray>(
        RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&gps_slam_navigation::navigation::Navigator::navigate_to_pose_goal_status_subscription_cb, this, _1),
        navigate_to_pose_goal_status_subscription_opts);

    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_NAVIGATE_TO_POSE_GOAL_STATUS_TOPIC);
}

/**
 * destroy this class' instance
 * @brief default destructor
 */
gps_slam_navigation::navigation::Navigator::~Navigator()
{
}

/**
 * @brief function for handle signal_input when program exit
 * @param signal_input The signal_input of input
 * @return void
 * @see signal_input.h
 */
void gps_slam_navigation::navigation::Navigator::signal_handler(int signal_input)
{
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "===== gps_slam_navigation has been terminated with SIG [%d] =====", signal_input);
    signal(signal_input, SIG_IGN);
    exit(RCL_STOP_FLAG);
}

/**
 * @brief function for flag established RCL connections
 * @return void
 */
void gps_slam_navigation::navigation::Navigator::flag_rcl_connections(const char *connection_type, const char *connection_name)
{
    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "RCL [%s - %s] created...", connection_type, connection_name);
    RCLCPP_LINE_INFO();
}

/**
 * @brief function for handle subscription callback from rclcpp::Subscription<gps_navigation_msgs::msg::GoalWaypointsStamped>
 * @param gps_waypoints_cb_data const gps_navigation_msgs::msg::GoalWaypointsStamped::SharedPtr
 * @return void
 */
void gps_slam_navigation::navigation::Navigator::gps_waypoints_subscription_cb(const gps_navigation_msgs::msg::GoalWaypointsStamped::SharedPtr gps_waypoints_cb_data)
{
    bool is_gps_goal_waypoints_empty = gps_goal_waypoints_list_.empty();

    if (!is_gps_goal_waypoints_empty)
    {
        slam_goal_waypoints_list_index_ = RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX;
        slam_goal_waypoints_list_size_ = RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_SIZE;

        gps_goal_waypoints_list_.clear();
        slam_goal_waypoints_list_.clear();

        RCLCPP_WARN(
            rcl_node_ptr_->get_logger(),
            "===== gps_goal_waypoints has been renewed =====\n\twaypoints_vec_idx : [%d]\n\twaypoints_vec_size : [%d]",
            slam_goal_waypoints_list_index_,
            slam_goal_waypoints_list_size_);
        RCLCPP_LINE_WARN();
    }

    gps_goal_waypoints_list_ = gps_waypoints_cb_data->goal_waypoints_list;
    gps_goal_waypoints_list_size_ = gps_goal_waypoints_list_.size();

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "gps_waypoints_list schedule size : [%i]", gps_goal_waypoints_list_size_);
    RCLCPP_LINE_INFO();

    for (const geometry_msgs::msg::Pose &gps_goal_waypoints : gps_goal_waypoints_list_)
    {
        const double &gps_goal_x = gps_goal_waypoints.position.x;
        const double &gps_goal_y = gps_goal_waypoints.position.y;

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(), "gps_waypoints_list vector\n\tx : [%f]\n\ty : [%f]",
            gps_goal_x,
            gps_goal_y);
        RCLCPP_LINE_INFO();

        std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point_ptr = ktm_math_->geo_point_to_ktm_point(
            gps_goal_x,
            gps_goal_y);

        std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_ptr = slam_point_generator_->geo_point_to_slam_point(gts_point_, ktm_point_ptr);
        slam_goal_waypoints_list_.push_back(*slam_point_ptr);
    }

    for (gps_slam_navigation::points::SLAMPoint &slam_goal : slam_goal_waypoints_list_)
    {
        const double &slam_goal_x = slam_goal.get__x();
        const double &slam_goal_y = slam_goal.get__y();

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "gps_waypoints_list slam points\n\tx : [%f]\n\ty : [%f]",
            slam_goal_x,
            slam_goal_y);
        RCLCPP_LINE_INFO();
    }

    gps_goal_waypoints_list_ = gps_waypoints_cb_data->goal_waypoints_list;
    gps_goal_waypoints_list_size_ = gps_goal_waypoints_list_.size();
    slam_goal_waypoints_list_size_ = gps_goal_waypoints_list_size_;

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "gps_waypoints_list schedule size : [%i]", gps_goal_waypoints_list_size_);
    RCLCPP_LINE_INFO();

    for (const geometry_msgs::msg::Pose &gps_goal_waypoints : gps_goal_waypoints_list_)
    {
        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(), "gps_waypoints_list vector\n\tx : [%f]\n\ty : [%f]",
            gps_goal_waypoints.position.x,
            gps_goal_waypoints.position.y);
        RCLCPP_LINE_INFO();
    }

    this->navigate_to_pose_send_goal();
}

void gps_slam_navigation::navigation::Navigator::gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data)
{
    const double &current_gps_lat = gps_cb_data->latitude;
    const double &current_gps_lon = gps_cb_data->longitude;

    bool is_geo_point_1_ptr_default = (gts_point_->get__ktm_point_1_ptr()->get__x() == GTS_DEFAULT_DOUBLE) && (gts_point_->get__ktm_point_1_ptr()->get__y() == GTS_DEFAULT_DOUBLE);
    bool is_geo_point_2_ptr_default = (gts_point_->get__ktm_point_2_ptr()->get__x() == GTS_DEFAULT_DOUBLE) && (gts_point_->get__ktm_point_2_ptr()->get__y() == GTS_DEFAULT_DOUBLE);

    bool is_robot_pose_ptr_null = slam_pose_ == nullptr;

    if (is_geo_point_1_ptr_default && !is_robot_pose_ptr_null)
    {
        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "ublox fix callback set first coordinate");
        RCLCPP_LINE_INFO();

        std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point_1_ptr = ktm_math_->geo_point_to_ktm_point(
            37.465932,
            127.124136);

        gts_point_->set__ktm_point_1_ptr(ktm_point_1_ptr);

        std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_1_ptr = gts_point_->get__slam_point_1_ptr();

        const double &robot_pose_x = slam_pose_->position.x;
        const double &robot_pose_y = slam_pose_->position.y;

        slam_point_1_ptr->set__x(2.724780);
        slam_point_1_ptr->set__y(-2.330876);

        gts_point_->set__slam_point_1_ptr(slam_point_1_ptr);

        std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_current_point_ptr = ktm_math_->geo_point_to_ktm_point(
            current_gps_lat, current_gps_lon);

        const double &differ_1 = common_math_->distance_formula(
            gts_point_->get__ktm_point_1_ptr()->get__x(), gts_point_->get__ktm_point_1_ptr()->get__y(),
            ktm_current_point_ptr->get__x(), ktm_current_point_ptr->get__y());

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "ublox fix callback KtS target result 1\n\tx : [%f]\n\ty : [%f]",
            ktm_point_1_ptr->get__x(),
            ktm_point_1_ptr->get__y());
        RCLCPP_LINE_INFO();

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "ublox fix callback KtS result 1\n\tx : [%f]\n\ty : [%f]\n\tdiffer : [%f]",
            ktm_current_point_ptr->get__x(),
            ktm_current_point_ptr->get__y(),
            differ_1);
        RCLCPP_LINE_INFO();
    }

    if (!is_geo_point_1_ptr_default && is_geo_point_2_ptr_default)
    {
        const double &ktm_point_1_lat = gts_point_->get__ktm_point_1_ptr()->get__x();
        const double &ktm_point_1_lon = gts_point_->get__ktm_point_1_ptr()->get__y();

        const double &gps_distance = gps_math_->distance_formula_meters(
            ktm_point_1_lat,
            ktm_point_1_lon,
            current_gps_lat,
            current_gps_lon);

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "ublox fix callback ktm point 1 gps distance : [%f]",
            gps_distance);
        RCLCPP_LINE_INFO();

        if (gps_distance >= 15)
        {
            std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_point_2_ptr = ktm_math_->geo_point_to_ktm_point(
                37.465798,
                127.124107);

            gts_point_->set__ktm_point_2_ptr(ktm_point_2_ptr);

            std::shared_ptr<gps_slam_navigation::points::SLAMPoint> slam_point_2_ptr = gts_point_->get__slam_point_2_ptr();

            const double &robot_pose_x = slam_pose_->position.x;
            const double &robot_pose_y = slam_pose_->position.y;

            slam_point_2_ptr->set__x(-12.386119);
            slam_point_2_ptr->set__y(0.497768);

            gts_point_->set__slam_point_2_ptr(slam_point_2_ptr);

            RCLCPP_INFO(
                rcl_node_ptr_->get_logger(),
                "======= ublox fix callback distance result =======\n\tKTM 1 lat : [%f]\n\tKTM 1 long : [%f]\n\tKTM 2 lat : [%f]\n\tKTM 2 long : [%f]\n\tSLAM 1 x : [%f]\n\tSLAM 1 y : [%f]\n\tSLAM 2 x : [%f]\n\tSLAM 2 y : [%f]",
                gts_point_->get__ktm_point_1_ptr()->get__x(),
                gts_point_->get__ktm_point_1_ptr()->get__y(),
                gts_point_->get__ktm_point_2_ptr()->get__x(),
                gts_point_->get__ktm_point_2_ptr()->get__y(),
                gts_point_->get__slam_point_1_ptr()->get__x(),
                gts_point_->get__slam_point_1_ptr()->get__y(),
                gts_point_->get__slam_point_2_ptr()->get__x(),
                gts_point_->get__slam_point_2_ptr()->get__y());
            RCLCPP_LINE_INFO();

            std::shared_ptr<gps_slam_navigation::points::KTMPoint> ktm_current_point_ptr = ktm_math_->geo_point_to_ktm_point(
                current_gps_lat, current_gps_lon);

            const double &differ_2 = common_math_->distance_formula(
                gts_point_->get__ktm_point_2_ptr()->get__x(), gts_point_->get__ktm_point_2_ptr()->get__y(),
                ktm_current_point_ptr->get__x(), ktm_current_point_ptr->get__y());

            RCLCPP_INFO(
                rcl_node_ptr_->get_logger(),
                "ublox fix callback KtS target result 2\n\tx : [%f]\n\ty : [%f]",
                ktm_point_2_ptr->get__x(),
                ktm_point_2_ptr->get__y());
            RCLCPP_LINE_INFO();

            RCLCPP_INFO(
                rcl_node_ptr_->get_logger(),
                "ublox fix callback KtS result 2\n\tx : [%f]\n\ty : [%f]\n\tdiffer : [%f]",
                ktm_current_point_ptr->get__x(),
                ktm_current_point_ptr->get__y(),
                differ_2);
            RCLCPP_LINE_INFO();
        }
    }
}

void gps_slam_navigation::navigation::Navigator::slam_pose_subscription_cb(const geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data)
{
    slam_pose_ = slam_pose_cb_data;

    bool is_slam_pose_nullptr = slam_pose_ == nullptr;

    if (is_slam_pose_nullptr)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "robot pose callback rcl_robot_pose_ptr is nullptr");
        RCLCPP_LINE_ERROR();
    }
}

void gps_slam_navigation::navigation::Navigator::gps_navigation_status_publish(const gps_navigation_msgs::msg::NavigationStatusStamped::UniquePtr &gps_navigation_status, const int &goal_status_code)
{
    bool is_gps_navigation_status_nullptr = gps_navigation_status == nullptr;

    if (is_gps_navigation_status_nullptr)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigation_status_stamped publishing failed with nullptr");
        return;
    }

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "navigation_status_stamped published");
    RCLCPP_LINE_INFO();

    gps_navigation_status->set__status_code(goal_status_code);
    gps_navigation_status->set__current_goal_index(slam_goal_waypoints_list_index_);

    const gps_navigation_msgs::msg::NavigationStatusStamped &&rcl_navigation_status_stamped_ptr_moved = std::move(*gps_navigation_status);
    gps_navigation_status_publisher_->publish(rcl_navigation_status_stamped_ptr_moved);
}

void gps_slam_navigation::navigation::Navigator::gps_navigation_result_publish(const gps_navigation_msgs::msg::NavigationResultStamped::UniquePtr &gps_navigation_result, const int &goal_status_code)
{
    bool is_gps_navigation_result_nullptr = gps_navigation_result == nullptr;

    if (is_gps_navigation_result_nullptr)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigation_result_stamped publishing failed with nullptr");
        return;
    }

    RCLCPP_INFO(rcl_node_ptr_->get_logger(), "navigation_resykt_stamped published");
    RCLCPP_LINE_INFO();

    gps_navigation_result->set__result_code(goal_status_code);
    gps_navigation_result->set__result_index(slam_goal_waypoints_scenario_list_index_);

    const gps_navigation_msgs::msg::NavigationResultStamped &&rcl_navigation_result_stamped_ptr_moved = std::move(*gps_navigation_result);
    gps_navigation_result_publisher_->publish(rcl_navigation_result_stamped_ptr_moved);
}

void gps_slam_navigation::navigation::Navigator::navigate_to_pose_goal_status_subscription_cb(const action_msgs::msg::GoalStatusArray::SharedPtr goal_status_array_cb_data)
{
    bool is_goal_status_array_empty = goal_status_array_cb_data->status_list.empty();

    if (is_goal_status_array_empty)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose goal status callback status list is empty...");
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

        gps_navigation_msgs::msg::NavigationStatusStamped::UniquePtr gps_navigation_status = std::make_unique<gps_navigation_msgs::msg::NavigationStatusStamped>();
        gps_navigation_status->set__header(std_msgs_header_moved);

        gps_navigation_msgs::msg::NavigationResultStamped::UniquePtr gps_navigation_result = std::make_unique<gps_navigation_msgs::msg::NavigationResultStamped>();
        gps_navigation_result->set__header(std_msgs_header_moved);

        const action_msgs::msg::GoalStatus &goal_status = goal_status_array_cb_data->status_list.back();
        const uint8_t &goal_status_code = goal_status.status;

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "navigate_to_pose status callback\n\twaypoints index : [%d]\n\twaypoints size : [%d]\n\tgoal_status_code : [%d]",
            slam_goal_waypoints_list_index_ + 1,
            slam_goal_waypoints_list_size_,
            goal_status_code);
        RCLCPP_LINE_INFO();

        bool is_slam_goal_waypoints_ended = slam_goal_waypoints_list_index_ == (slam_goal_waypoints_list_size_ - 1);

        if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_STARTED)
        {
            RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== navigate_to_pose status callback goal started =====");
            RCLCPP_LINE_INFO();

            this->gps_navigation_status_publish(gps_navigation_status, goal_status_code);
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED)
        {
            RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== navigate_to_pose status callback goal succeeded =====");
            RCLCPP_LINE_INFO();

            if (is_slam_goal_waypoints_ended)
            {
                RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== navigate_to_pose status callback goal waypoints ended =====");
                RCLCPP_LINE_INFO();

                slam_goal_waypoints_scenario_list_index_++;

                this->gps_navigation_result_publish(gps_navigation_result, goal_status_code);

                slam_goal_waypoints_list_index_ = RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_IDX;
                slam_goal_waypoints_list_size_ = RCL_GPS_GOAL_WAYPOINTS_VECTOR_DEFAULT_SIZE;

                gps_goal_waypoints_list_.clear();
                slam_goal_waypoints_list_.clear();

                navigate_to_pose_client_->async_cancel_all_goals();
            }
            else
            {
                slam_goal_waypoints_list_index_++;

                RCLCPP_INFO(rcl_node_ptr_->get_logger(), "===== navigate_to_pose status callback will proceed next [%d] goal =====", slam_goal_waypoints_list_index_);
                RCLCPP_LINE_INFO();

                this->gps_navigation_status_publish(gps_navigation_status, goal_status_code);
                this->navigate_to_pose_send_goal();
            }
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_ABORTED)
        {
            RCLCPP_WARN(rcl_node_ptr_->get_logger(), "!!!!! navigate_to_pose status callback goal aborted !!!!!");
            RCLCPP_LINE_WARN();

            RCLCPP_WARN(rcl_node_ptr_->get_logger(), "navigate_to_pose status callback will proceed previous [%d]st goal", slam_goal_waypoints_list_index_);
            RCLCPP_LINE_WARN();

            this->gps_navigation_status_publish(gps_navigation_status, goal_status_code);
            this->navigate_to_pose_send_goal();
        }
        else if (goal_status_code == RCL_NAVIGATE_TO_POSE_GOAL_CANCELED)
        {
            RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "!!!!! navigate_to_pose status callback goal canceled !!!!!");
            RCLCPP_LINE_ERROR();

            this->gps_navigation_status_publish(gps_navigation_status, goal_status_code);
        }
        else
        {
            RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose status callback unknown goal result code : [%d]", goal_status_code);
            RCLCPP_LINE_ERROR();
            return;
        }
    }
}

/**
 * @brief function for send goal to rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @return void
 */
void gps_slam_navigation::navigation::Navigator::navigate_to_pose_send_goal()
{
    bool is_slam_goal_waypoints_vec_empty = slam_goal_waypoints_list_.empty();

    if (is_slam_goal_waypoints_vec_empty)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose send goal waypoints is empty...");
        RCLCPP_LINE_ERROR();
        return;
    }

    const double &slam_x = slam_goal_waypoints_list_[slam_goal_waypoints_list_index_].get__x();
    const double &slam_y = slam_goal_waypoints_list_[slam_goal_waypoints_list_index_].get__y();

    bool is_robot_pose_ptr_nullptr = slam_pose_ == nullptr;

    if (!is_robot_pose_ptr_nullptr)
    {
        const double &current_x = slam_pose_->position.x;
        const double &current_y = slam_pose_->position.y;

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "navigate_to_pose send goal\n\tcurrent x: : [%f]\n\tcurrent y : [%f]\n\tslam x : [%f]\n\tslam y : [%f]",
            current_x,
            current_y,
            slam_x,
            slam_y);
        RCLCPP_LINE_INFO();

        const double &angle = common_math_->angle_between_two_points(current_x, current_y, slam_x, slam_y);

        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "navigate_to_pose send goal angle : [%f]", angle);
        RCLCPP_LINE_INFO();

        quaternion_point_->euler_angle_to_quaternion(angle, RCL_DEFAULT_DOUBLE, RCL_DEFAULT_DOUBLE);

        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
            "navigate_to_pose send goal quaternion\n\tz : [%f]\n\tw : [%f]",
            quaternion_point_->get__euler_z(),
            quaternion_point_->get__euler_w());
        RCLCPP_LINE_INFO();
    }

    bool is_navigate_to_pose_server_ready = navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!is_navigate_to_pose_server_ready)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose action server is not available after waiting");
        RCLCPP_LINE_ERROR();
        return;
    }
    else
    {
        RCLCPP_INFO(
            rcl_node_ptr_->get_logger(),
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
    geometry_msgs_quaternion->set__z(quaternion_point_->get__euler_z());
    geometry_msgs_quaternion->set__w(quaternion_point_->get__euler_w());

    geometry_msgs::msg::Pose::UniquePtr geometry_msgs_pose = std::make_unique<geometry_msgs::msg::Pose>();

    const geometry_msgs::msg::Point &&geometry_msgs_point_moved = std::move(*geometry_msgs_point);
    geometry_msgs_pose->set__position(geometry_msgs_point_moved);

    const geometry_msgs::msg::Quaternion &&geometry_msgs_quaternion_moved = std::move(*geometry_msgs_quaternion);
    geometry_msgs_pose->set__orientation(geometry_msgs_quaternion_moved);

    const geometry_msgs::msg::Pose &&geometry_msgs_pose_moved = std::move(*geometry_msgs_pose);
    geometry_msgs_pose_stamped->set__pose(geometry_msgs_pose_moved);

    const geometry_msgs::msg::PoseStamped &&geometry_msgs_pose_stamped_moved = std::move(*geometry_msgs_pose_stamped);
    navigate_to_pose_goal_->set__pose(geometry_msgs_pose_stamped_moved);

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions navigate_to_pose_send_goal_opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    navigate_to_pose_send_goal_opts.feedback_callback = std::bind(&gps_slam_navigation::navigation::Navigator::navigate_to_pose_feedback_cb, this, _1, _2);
    navigate_to_pose_send_goal_opts.goal_response_callback = std::bind(&gps_slam_navigation::navigation::Navigator::navigate_to_pose_goal_response_cb, this, _1);
    navigate_to_pose_send_goal_opts.result_callback = std::bind(&gps_slam_navigation::navigation::Navigator::navigate_to_pose_result_cb, this, _1);

    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> navigate_to_pose_goal_future = navigate_to_pose_client_->async_send_goal(*navigate_to_pose_goal_, navigate_to_pose_send_goal_opts);
    navigate_to_pose_goal_handle_ = navigate_to_pose_goal_future.get();

    RCLCPP_INFO(
        rcl_node_ptr_->get_logger(),
        "navigate_to_pose goal sent\n\tpose_x : [%f]\n\tpose_y : [%f]\n\torien_z: [%f]\n\torien_w : [%f]",
        navigate_to_pose_goal_->pose.pose.position.x,
        navigate_to_pose_goal_->pose.pose.position.y,
        navigate_to_pose_goal_->pose.pose.orientation.z,
        navigate_to_pose_goal_->pose.pose.orientation.w);
    RCLCPP_LINE_INFO();
}

/**
 * @brief function for handle response callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @param future std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>
 * @return void
 */
void gps_slam_navigation::navigation::Navigator::navigate_to_pose_goal_response_cb(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
{
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle = future.get();

    if (!goal_handle)
    {
        RCLCPP_ERROR(rcl_node_ptr_->get_logger(), "navigate_to_pose goal response callback was rejected by [%s] server", RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);
        RCLCPP_LINE_ERROR();
    }
    else
    {
        RCLCPP_INFO(rcl_node_ptr_->get_logger(), "navigate_to_pose goal response callback has been accepted by [%s] server, waiting for result...", RCL_NAVIGATE_TO_POSE_ACTION_SERVER_NAME);
        RCLCPP_LINE_INFO();
    }
}

/**
 * @brief function for handle feedback callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @param goal_handle_ptr const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
 * @param feedback_ptr const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
 * @return void
 */
void gps_slam_navigation::navigation::Navigator::navigate_to_pose_feedback_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_ptr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_ptr)
{
}

/**
 * @brief function for handle result callback from rclcpp_action::Server<nav2_msgs::action::NavigateToPose>
 * @param result const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &
 * @return void
 */
void gps_slam_navigation::navigation::Navigator::navigate_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &wrapped_result)
{
}