#include "conversions/slam_to_gps.hxx"

gps_slam_converter::conversions::SLAMToGPS::SLAMToGPS()
{
}

gps_slam_converter::conversions::SLAMToGPS::~SLAMToGPS()
{
}

sensor_msgs::msg::NavSatFix::SharedPtr gps_slam_converter::conversions::SLAMToGPS::convert(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data, geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data)
{
    std::shared_ptr<gps_slam_converter::math::CommonMath> common_math = std::make_shared<gps_slam_converter::math::CommonMath>();
    std::shared_ptr<gps_slam_converter::math::KTMMath> ktm_math = std::make_shared<gps_slam_converter::math::KTMMath>();
    std::shared_ptr<gps_slam_converter::math::GPSMath> gps_math = std::make_shared<gps_slam_converter::math::GPSMath>();

    bool is_gps_null = (gps_cb_data == nullptr);

    if (is_gps_null)
    {
        RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_to_gps_converter gps callback is null");
        RCLCPP_LINE_ERROR();
        return nullptr;
    }
    else
    {
        const double &current_gps_lat = gps_cb_data->latitude;
        const double &current_gps_lon = gps_cb_data->longitude;

        RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME,
                               "slam_to_gps_converter convert\n\tlat : [%f]\n\tlon : [%f]",
                               current_gps_lat,
                               current_gps_lon);
        RCLCPP_LINE_INFO();

        std::shared_ptr<gps_slam_converter::points::GTSPoint> gts_point = std::make_shared<gps_slam_converter::points::GTSPoint>();

        bool is_gts_ktm_point_1_default = (gts_point->get__ktm_point_1_ptr()->get__x() == GTS_DEFAULT_DOUBLE) && (gts_point->get__ktm_point_1_ptr()->get__y() == GTS_DEFAULT_DOUBLE);
        bool is_gts_ktm_point_2_default = (gts_point->get__ktm_point_2_ptr()->get__x() == GTS_DEFAULT_DOUBLE) && (gts_point->get__ktm_point_2_ptr()->get__y() == GTS_DEFAULT_DOUBLE);
        bool is_gps_cb_data_null = (gps_cb_data == nullptr);

        std::chrono::_V2::system_clock::time_point now = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::duration time_since_epoch = now.time_since_epoch();

        std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
        std::chrono::nanoseconds nanoseconds = time_since_epoch - seconds;

        int32_t seconds_int_32 = static_cast<int32_t>(seconds.count());
        int32_t nanoseconds_int_32 = static_cast<int32_t>(nanoseconds.count());

        builtin_interfaces::msg::Time::UniquePtr nav_sat_fix_header_stamp = std::make_unique<builtin_interfaces::msg::Time>();
        nav_sat_fix_header_stamp->set__sec(seconds_int_32);
        nav_sat_fix_header_stamp->set__nanosec(nanoseconds_int_32);

        builtin_interfaces::msg::Time &&nav_sat_fix_header_stamp_moved = std::move(*nav_sat_fix_header_stamp);

        std_msgs::msg::Header::UniquePtr nav_sat_fix_header = std::make_unique<std_msgs::msg::Header>();
        nav_sat_fix_header->set__stamp(nav_sat_fix_header_stamp_moved);
        nav_sat_fix_header->set__frame_id("slam_to_gps");

        std_msgs::msg::Header &&nav_sat_fix_header_moved = std::move(*nav_sat_fix_header);

        sensor_msgs::msg::NavSatStatus::UniquePtr nav_sat_status = std::make_unique<sensor_msgs::msg::NavSatStatus>();
        nav_sat_status->set__service(0);
        nav_sat_status->set__status(0);

        sensor_msgs::msg::NavSatStatus &&nav_sat_status_moved = std::move(*nav_sat_status);

        std::array<double, 9UL> position_covariance = {};

        sensor_msgs::msg::NavSatFix::SharedPtr converted_nav_sat_fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
        converted_nav_sat_fix->set__header(nav_sat_fix_header_moved);
        converted_nav_sat_fix->set__status(nav_sat_status_moved);
        converted_nav_sat_fix->set__latitude(RCL_DEFAULT_DOUBLE);
        converted_nav_sat_fix->set__longitude(RCL_DEFAULT_DOUBLE);
        converted_nav_sat_fix->set__altitude(RCL_DEFAULT_DOUBLE);
        converted_nav_sat_fix->set__position_covariance(position_covariance);
        converted_nav_sat_fix->set__position_covariance_type(0);

        if (is_gts_ktm_point_1_default && !is_gps_cb_data_null)
        {
            RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_to_gps_converter convert set first coordinate...");
            RCLCPP_LINE_INFO();

            std::shared_ptr<gps_slam_converter::points::KTMPoint> fixed_ktm_point_1 = ktm_math->geo_point_to_ktm_point(
                37.465932,
                127.124136);

            gts_point->set__ktm_point_1_ptr(fixed_ktm_point_1);

            std::shared_ptr<gps_slam_converter::points::SLAMPoint> fixed_slam_point_1 = gts_point->get__slam_point_1_ptr();

            const double &current_slam_pose_x = slam_pose_cb_data->position.x;
            const double &current_slam_pose_y = slam_pose_cb_data->position.y;

            fixed_slam_point_1->set__x(2.724780);
            fixed_slam_point_1->set__y(-2.330876);

            gts_point->set__slam_point_1_ptr(fixed_slam_point_1);

            std::shared_ptr<gps_slam_converter::points::KTMPoint> current_ktm_point = ktm_math->geo_point_to_ktm_point(current_gps_lat, current_gps_lon);

            const double &differ_1 = common_math->distance_formula(
                gts_point->get__ktm_point_1_ptr()->get__x(), gts_point->get__ktm_point_1_ptr()->get__y(),
                current_ktm_point->get__x(), current_ktm_point->get__y());

            RCUTILS_LOG_INFO_NAMED(
                RCL_NODE_NAME,
                "slam_to_gps_converter convert fixed ktm 1\n\tx : [%f]\n\tx : [%f]",
                fixed_ktm_point_1->get__x(),
                fixed_ktm_point_1->get__y());
            RCLCPP_LINE_INFO();
        }

        if (!is_gts_ktm_point_1_default && is_gts_ktm_point_2_default)
        {
            const double &ktm_point_1_lat = gts_point->get__ktm_point_1_ptr()->get__x();
            const double &ktm_point_1_lon = gts_point->get__ktm_point_1_ptr()->get__y();

            const double &gps_distance = gps_math->distance_formula_meters(
                ktm_point_1_lat,
                ktm_point_1_lon,
                current_gps_lat,
                current_gps_lon);

            RCUTILS_LOG_INFO_NAMED(
                RCL_NODE_NAME,
                "slam_to_gps_converter convert\n\tgps distance : [%f]",
                gps_distance);
            RCLCPP_LINE_INFO();

            if (gps_distance >= 15)
            {
                std::shared_ptr<gps_slam_converter::points::KTMPoint> fixed_ktm_point_2 = ktm_math->geo_point_to_ktm_point(
                    37.465798,
                    127.124107);

                gts_point->set__ktm_point_2_ptr(fixed_ktm_point_2);

                std::shared_ptr<gps_slam_converter::points::SLAMPoint> slam_point_2_ptr = gts_point->get__slam_point_2_ptr();

                const double &current_slam_pose_x = slam_pose_cb_data->position.x;
                const double &current_slam_pose_y = slam_pose_cb_data->position.y;

                slam_point_2_ptr->set__x(-12.386119);
                slam_point_2_ptr->set__y(0.497768);

                gts_point->set__slam_point_2_ptr(slam_point_2_ptr);

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "======= slam_to_gps_converter convert distance result =======\n\tKTM 1 lat : [%f]\n\tKTM 1 long : [%f]\n\tKTM 2 lat : [%f]\n\tKTM 2 long : [%f]\n\tSLAM 1 x : [%f]\n\tSLAM 1 y : [%f]\n\tSLAM 2 x : [%f]\n\tSLAM 2 y : [%f]",
                    gts_point->get__ktm_point_1_ptr()->get__x(),
                    gts_point->get__ktm_point_1_ptr()->get__y(),
                    gts_point->get__ktm_point_2_ptr()->get__x(),
                    gts_point->get__ktm_point_2_ptr()->get__y(),
                    gts_point->get__slam_point_1_ptr()->get__x(),
                    gts_point->get__slam_point_1_ptr()->get__y(),
                    gts_point->get__slam_point_2_ptr()->get__x(),
                    gts_point->get__slam_point_2_ptr()->get__y());
                RCLCPP_LINE_INFO();

                std::shared_ptr<gps_slam_converter::points::KTMPoint> ktm_current_point_ptr = ktm_math->geo_point_to_ktm_point(current_gps_lat, current_gps_lon);

                const double &differ_2 = common_math->distance_formula(
                    gts_point->get__ktm_point_2_ptr()->get__x(), gts_point->get__ktm_point_2_ptr()->get__y(),
                    ktm_current_point_ptr->get__x(), ktm_current_point_ptr->get__y());

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "slam_to_gps_converter convert KtS target result 2\n\tx : [%f]\n\ty : [%f]",
                    fixed_ktm_point_2->get__x(),
                    fixed_ktm_point_2->get__y());
                RCLCPP_LINE_INFO();

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "slam_to_gps_converter convert KtS result 2\n\tx : [%f]\n\ty : [%f]\n\tdiffer : [%f]",
                    ktm_current_point_ptr->get__x(),
                    ktm_current_point_ptr->get__y(),
                    differ_2);
                RCLCPP_LINE_INFO();

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "slam_to_gps_converter convert result\n\tcurrent GPS x: : [%f]\n\tcurrent GPS y : [%f]\n\tcurrent SLAM x : [%f]\n\tSLAM y : [%f]",
                    current_gps_lat,
                    current_gps_lon,
                    current_slam_pose_x,
                    current_slam_pose_y);
                RCLCPP_LINE_INFO();

                std::shared_ptr<gps_slam_converter::points::KTMPoint> ktm_point = ktm_math->geo_point_to_ktm_point(current_slam_pose_x, current_slam_pose_y);

                std::shared_ptr<gps_slam_converter::math::SLAMPointGenerator> slam_point_generator = std::make_shared<gps_slam_converter::math::SLAMPointGenerator>();
                std::shared_ptr<gps_slam_converter::points::SLAMPoint> current_slam_point = std::make_shared<gps_slam_converter::points::SLAMPoint>();
                current_slam_point->set__x(current_slam_pose_x);
                current_slam_point->set__y(current_slam_pose_y);

                std::shared_ptr<gps_slam_converter::points::GeoPoint> geo_point = slam_point_generator->slam_point_to_geo_point(gts_point, current_slam_point);

                const double &geo_lat = geo_point->get__latitude();
                const double &geo_lon = geo_point->get__longitude();

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "slam_to_gps_converter convert result\n\tcurrent GPS x: : [%f]\n\tcurrent GPS y : [%f]\n\tconverted GPS x : [%f]\n\ttconverted GPS y : [%f]",
                    current_gps_lat,
                    current_gps_lon,
                    geo_lat,
                    geo_lon);
                RCLCPP_LINE_INFO();

                const double &angle = common_math->angle_between_two_points(current_gps_lat, current_gps_lon, geo_lat, geo_lon);

                RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "slam_to_gps_converter convert angle : [%f]", angle);
                RCLCPP_LINE_INFO();

                converted_nav_sat_fix->set__latitude(geo_lat);
                converted_nav_sat_fix->set__longitude(geo_lon);
            }
        }

        return converted_nav_sat_fix;
    }
}