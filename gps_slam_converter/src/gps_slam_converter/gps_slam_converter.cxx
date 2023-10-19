#include "gps_slam_converter/gps_slam_converter.hxx"

gps_slam_converter::converter::Converter::Converter()
    : Node(RCL_NODE_NAME)
{
    this->rcl_node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    if (this->rcl_node_ != nullptr)
    {
        RCLCPP_INFO(this->rcl_node_->get_logger(), "%s node created", RCL_NODE_NAME);
        RCLCPP_LINE_INFO();
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "failed to create %s node", RCL_NODE_NAME);
        RCLCPP_LINE_ERROR();
        exit(0);
    }

    this->gps_to_slam_converter = std::make_shared<gps_slam_converter::conversions::GPSToSLAM>();
    this->slam_to_gps_converter = std::make_shared<gps_slam_converter::conversions::SLAMToGPS>();

    this->slam_pose_subscription_cb_group_ = this->rcl_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions slam_pose_subscription_opts;
    slam_pose_subscription_opts.callback_group = this->slam_pose_subscription_cb_group_;
    this->slam_pose_subscription_ = this->rcl_node_->create_subscription<geometry_msgs::msg::Pose>(
        RCL_ROBOT_POSE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&gps_slam_converter::converter::Converter::slam_pose_subscription_cb, this, _1),
        slam_pose_subscription_opts);

    this->gps_subscription_cb_group_ = this->rcl_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_subscription_opts;
    gps_subscription_opts.callback_group = this->gps_subscription_cb_group_;
    this->gps_subscription_ = this->rcl_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        RCL_UBLOX_FIX_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&gps_slam_converter::converter::Converter::gps_subscription_cb, this, _1),
        gps_subscription_opts);

    this->converted_slam_publisher_cb_group_ = this->rcl_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions converted_slam_publisher_opts;
    converted_slam_publisher_opts.callback_group = this->converted_slam_publisher_cb_group_;
    this->converted_slam_publisher_ = this->rcl_node_->create_publisher<geometry_msgs::msg::Pose>(
        RCL_GPS_TO_SLAM_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        converted_slam_publisher_opts);

    this->converted_gps_publisher_cb_group_ = this->rcl_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions converted_gps_publihser_opts;
    converted_gps_publihser_opts.callback_group = this->converted_gps_publisher_cb_group_;
    this->converted_gps_publisher_ = this->rcl_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
        RCL_SLAM_TO_GPS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        converted_gps_publihser_opts);

    this->converter_service_ = this->rcl_node_->create_service<gps_slam_conversion_msgs::srv::Conversion>(
        RCL_CONVERTER_SERVICE_SERVER_NAME,
        std::bind(&gps_slam_converter::converter::Converter::converter_service_cb, this, _1, _2));
}

gps_slam_converter::converter::Converter::~Converter()
{
}

void gps_slam_converter::converter::Converter::signal_handler(int signal_input)
{
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "===== {%s} has been terminated with SIG [%d] =====", RCL_NODE_NAME, signal_input);
    signal(signal_input, SIG_IGN);
    exit(0);
}

void gps_slam_converter::converter::Converter::slam_pose_subscription_cb(geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data)
{
    slam_pose_ = slam_pose_cb_data;

    bool is_slam_pose_null = (slam_pose_cb_data == nullptr);

    if (is_slam_pose_null)
    {
        RCLCPP_ERROR(this->rcl_node_->get_logger(), "/robot_pose callback is null");
        RCLCPP_LINE_ERROR();
    }
    else
    {
        RCLCPP_INFO(this->rcl_node_->get_logger(), "===== starts converting SLAM into GPS =====");
        sensor_msgs::msg::NavSatFix::SharedPtr converted_gps_data_from_slam = this->slam_to_gps_converter->convert(gps_, slam_pose_cb_data);

        if (converted_gps_data_from_slam == nullptr)
        {
            RCLCPP_ERROR(this->rcl_node_->get_logger(), "===== failed to converting SLAM into GPS =====");
            RCLCPP_LINE_ERROR();
            return;
        }

        RCLCPP_INFO(
            this->rcl_node_->get_logger(), "converted GPS data from SLAM\n\tlat : [%f]\n\tlon : [%f]",
            converted_gps_data_from_slam->latitude,
            converted_gps_data_from_slam->longitude);
        RCLCPP_LINE_INFO();

        this->converted_gps_publisher_->publish(*converted_gps_data_from_slam);
    }
}

void gps_slam_converter::converter::Converter::gps_subscription_cb(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data)
{
    gps_ = gps_cb_data;

    bool is_gps_null = (gps_ == nullptr);

    if (is_gps_null)
    {
        RCLCPP_ERROR(this->rcl_node_->get_logger(), "/ublox/fix callback is null");
        RCLCPP_LINE_ERROR();
    }
    else
    {
        RCLCPP_INFO(this->rcl_node_->get_logger(), "===== starts converting GPS into SLAM =====");
        geometry_msgs::msg::Pose::SharedPtr converted_slam_data_from_gps = this->gps_to_slam_converter->convert(gps_cb_data, slam_pose_);

        if (converted_slam_data_from_gps == nullptr)
        {
            RCLCPP_ERROR(this->rcl_node_->get_logger(), "===== failed to converting GPS into SLAM =====");
            RCLCPP_LINE_ERROR();
            return;
        }

        RCLCPP_INFO(
            this->rcl_node_->get_logger(), "converted SLAM data from GPS\n\tx : [%f]\n\ty : [%f]\n\tz : [%f]\n\tw : [%f]",
            converted_slam_data_from_gps->orientation.x,
            converted_slam_data_from_gps->orientation.y,
            converted_slam_data_from_gps->orientation.z,
            converted_slam_data_from_gps->orientation.w);

        this->converted_slam_publisher_->publish(*converted_slam_data_from_gps);
    }
}

void gps_slam_converter::converter::Converter::converter_service_cb(const gps_slam_conversion_msgs::srv::Conversion::Request::SharedPtr request, gps_slam_conversion_msgs::srv::Conversion::Response::SharedPtr response)
{
    const std::string &request_conversion_target_data = request->conversion_target.data;
    std::vector<sensor_msgs::msg::NavSatFix> gps_request_list = request->gps_request_list;
    std::vector<geometry_msgs::msg::Pose> slam_pose_request_list = request->slam_pose_request_list;

    if (request_conversion_target_data == "")
    {
        RCLCPP_ERROR(this->rcl_node_->get_logger(), "[%s] conversion_target is empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
        RCLCPP_LINE_ERROR();
        return;
    }
    else if (request_conversion_target_data == RCL_CONVERTER_SERVICE_CONVERSION_TARGET_SLAM)
    {
        bool is_gps_request_list_empty = gps_request_list.empty();
        bool is_slam_pose_request_list_empty = slam_pose_request_list.empty();

        if (!is_slam_pose_request_list_empty)
        {
            RCLCPP_ERROR(this->rcl_node_->get_logger(), "[%s] This request is not allowed when conversion_target is SLAM - Caused by pose_request_list isn't empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
            RCLCPP_LINE_ERROR();
            return;
        }

        int gps_request_list_size = 0;

        if (is_gps_request_list_empty)
        {
            RCLCPP_ERROR(this->rcl_node_->get_logger(), "[%s] Caused by gps_request_list is empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
            RCLCPP_LINE_ERROR();
            return;
        }
        else
        {
            RCLCPP_INFO(this->rcl_node_->get_logger(), "===== starts converting SLAM into GPS =====");
            gps_request_list_size = gps_request_list.size();

            std::vector<geometry_msgs::msg::Pose> slam_pose_response_list;
            int slam_pose_response_list_size = slam_pose_response_list.size();

            for (const sensor_msgs::msg::NavSatFix &gps_request : gps_request_list)
            {
                sensor_msgs::msg::NavSatFix::SharedPtr gps_request_shared = std::make_shared<sensor_msgs::msg::NavSatFix>(gps_request);
                geometry_msgs::msg::Pose::SharedPtr converted_slam_data_from_gps = this->gps_to_slam_converter->convert(gps_request_shared, slam_pose_);

                if (converted_slam_data_from_gps == nullptr)
                {
                    RCLCPP_ERROR(this->rcl_node_->get_logger(), "===== failed to converting SLAM into GPS =====");
                    RCLCPP_LINE_ERROR();
                    break;
                }
                else
                {
                    RCLCPP_INFO(
                        this->rcl_node_->get_logger(), "converted SLAM data from GPS\n\tx : [%f]\n\ty : [%f]\n\tz : [%f]\n\tw : [%f]",
                        converted_slam_data_from_gps->orientation.x,
                        converted_slam_data_from_gps->orientation.y,
                        converted_slam_data_from_gps->orientation.z,
                        converted_slam_data_from_gps->orientation.w);
                    RCLCPP_LINE_INFO();

                    slam_pose_response_list.push_back(*converted_slam_data_from_gps);
                }
            }

            if (gps_request_list_size == slam_pose_response_list_size)
            {
                std_msgs::msg::String::SharedPtr response_conversion_target = std::make_shared<std_msgs::msg::String>();
                response_conversion_target->set__data(RCL_CONVERTER_SERVICE_CONVERSION_TARGET_SLAM);

                response->set__conversion_target(*response_conversion_target);
                response->set__slam_pose_response_list(slam_pose_response_list);
            }
        }
    }
    else if (request_conversion_target_data == RCL_CONVERTER_SERVICE_CONVERSION_TARGET_GPS)
    {
        bool is_gps_request_list_empty = gps_request_list.empty();
        bool is_slam_pose_request_list_empty = slam_pose_request_list.empty();

        if (!is_gps_request_list_empty)
        {
            RCLCPP_ERROR(this->rcl_node_->get_logger(), "[%s] This request is not allowed when conversion_target is GPS - Caused by gps_request_list isn't empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
            RCLCPP_LINE_ERROR();
            return;
        }

        int slam_pose_request_list_size = 0;

        if (is_slam_pose_request_list_empty)
        {
            RCLCPP_ERROR(this->rcl_node_->get_logger(), "[%s] Caused by slam_pose_request_list is empty", RCL_CONVERTER_SERVICE_SERVER_NAME);
            RCLCPP_LINE_ERROR();
            return;
        }
        else
        {
            RCLCPP_INFO(this->rcl_node_->get_logger(), "===== starts converting GPS into SLAM =====");
            slam_pose_request_list_size = slam_pose_request_list.size();

            std::vector<sensor_msgs::msg::NavSatFix> gps_response_list;
            int gps_response_list_size = gps_response_list.size();

            for (const geometry_msgs::msg::Pose &slam_pose_request : slam_pose_request_list)
            {
                geometry_msgs::msg::Pose::SharedPtr slam_pose_request_shared = std::make_shared<geometry_msgs::msg::Pose>(slam_pose_request);
                sensor_msgs::msg::NavSatFix::SharedPtr converted_gps_data_from_slam = this->slam_to_gps_converter->convert(gps_, slam_pose_request_shared);

                if (converted_gps_data_from_slam == nullptr)
                {
                    RCLCPP_ERROR(this->rcl_node_->get_logger(), "===== failed to converting SLAM into GPS =====");
                    RCLCPP_LINE_ERROR();
                    break;
                }
                else
                {
                    RCLCPP_INFO(
                        this->rcl_node_->get_logger(), "converted GPS data from SLAM\n\tlat : [%f]\n\tlon : [%f]",
                        converted_gps_data_from_slam->latitude,
                        converted_gps_data_from_slam->longitude);
                    RCLCPP_LINE_INFO();

                    gps_response_list.push_back(*converted_gps_data_from_slam);
                }
            }

            if (slam_pose_request_list_size == gps_response_list_size)
            {
                std_msgs::msg::String::SharedPtr response_conversion_target = std::make_shared<std_msgs::msg::String>();
                response_conversion_target->set__data(RCL_CONVERTER_SERVICE_CONVERSION_TARGET_GPS);

                response->set__conversion_target(*response_conversion_target);
                response->set__gps_response_list(gps_response_list);
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->rcl_node_->get_logger(), "%s request is invalid", RCL_CONVERTER_SERVICE_SERVER_NAME);
        RCLCPP_LINE_ERROR();
    }
}