#include "conversions/gps_to_slam.hxx"

gps_slam_converter::conversions::GPSToSLAM::GPSToSLAM()
{
}

gps_slam_converter::conversions::GPSToSLAM::~GPSToSLAM()
{
}

geometry_msgs::msg::Pose::SharedPtr gps_slam_converter::conversions::GPSToSLAM::convert(sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_data, geometry_msgs::msg::Pose::SharedPtr slam_pose_cb_data)
{
    std::shared_ptr<gps_slam_converter::math::CommonMath> common_math = std::make_shared<gps_slam_converter::math::CommonMath>();
    std::shared_ptr<gps_slam_converter::math::KTMMath> ktm_math = std::make_shared<gps_slam_converter::math::KTMMath>();
    std::shared_ptr<gps_slam_converter::math::GPSMath> gps_math = std::make_shared<gps_slam_converter::math::GPSMath>();

    bool is_slam_pose_null = (slam_pose_cb_data == nullptr);

    if (is_slam_pose_null)
    {
        RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_to_slam_converter slam_pose callback is null");
        RCLCPP_LINE_ERROR();
        return nullptr;
    }
    else
    {
        const double &current_gps_lat = gps_cb_data->latitude;
        const double &current_gps_lon = gps_cb_data->longitude;

        RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME,
                               "gps_to_slam_converter convert\n\tlat : [%f]\n\tlon : [%f]",
                               current_gps_lat,
                               current_gps_lon);
        RCLCPP_LINE_INFO();

        std::shared_ptr<gps_slam_converter::points::GTSPoint> gts_point = std::make_shared<gps_slam_converter::points::GTSPoint>();
        RCLCPP_LINE_INFO();

        bool is_gts_ktm_point_1_default = (gts_point->get__ktm_point_1_ptr()->get__x() == GTS_DEFAULT_DOUBLE) && (gts_point->get__ktm_point_1_ptr()->get__y() == GTS_DEFAULT_DOUBLE);
        bool is_gts_ktm_point_2_default = (gts_point->get__ktm_point_2_ptr()->get__x() == GTS_DEFAULT_DOUBLE) && (gts_point->get__ktm_point_2_ptr()->get__y() == GTS_DEFAULT_DOUBLE);
        bool is_slam_pose_cb_data_null = (slam_pose_cb_data == nullptr);

        geometry_msgs::msg::Point::UniquePtr slam_pose_point = std::make_unique<geometry_msgs::msg::Point>();
        slam_pose_point->set__x(RCL_DEFAULT_DOUBLE);
        slam_pose_point->set__y(RCL_DEFAULT_DOUBLE);
        slam_pose_point->set__z(RCL_DEFAULT_DOUBLE);
        RCLCPP_LINE_INFO();

        geometry_msgs::msg::Point &&slam_pose_point_moved = std::move(*slam_pose_point);
        RCLCPP_LINE_INFO();

        geometry_msgs::msg::Quaternion::UniquePtr slam_pose_quaternion = std::make_unique<geometry_msgs::msg::Quaternion>();
        slam_pose_quaternion->set__x(RCL_DEFAULT_DOUBLE);
        slam_pose_quaternion->set__y(RCL_DEFAULT_DOUBLE);
        slam_pose_quaternion->set__z(RCL_DEFAULT_DOUBLE);
        slam_pose_quaternion->set__w(RCL_DEFAULT_DOUBLE);
        RCLCPP_LINE_INFO();

        geometry_msgs::msg::Quaternion &&slam_pose_quaternion_moved = std::move(*slam_pose_quaternion);
        RCLCPP_LINE_INFO();

        geometry_msgs::msg::Pose::SharedPtr converted_slam_pose = std::make_shared<geometry_msgs::msg::Pose>();
        converted_slam_pose->set__position(slam_pose_point_moved);
        converted_slam_pose->set__orientation(slam_pose_quaternion_moved);
        RCLCPP_LINE_INFO();

        if (is_gts_ktm_point_1_default && !is_slam_pose_cb_data_null)
        {
            RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_to_slam_converter convert set first coordinate...");
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
                "gps_to_slam_converter convert fixed ktm 1\n\tx : [%f]\n\tx : [%f]",
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
                "gps_to_slam_converter convert\n\tgps distance : [%f]",
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
                    "======= gps_to_slam_converter convert distance result =======\n\tKTM 1 lat : [%f]\n\tKTM 1 long : [%f]\n\tKTM 2 lat : [%f]\n\tKTM 2 long : [%f]\n\tSLAM 1 x : [%f]\n\tSLAM 1 y : [%f]\n\tSLAM 2 x : [%f]\n\tSLAM 2 y : [%f]",
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
                    "gps_to_slam_converter convert KtS target result 2\n\tx : [%f]\n\ty : [%f]",
                    fixed_ktm_point_2->get__x(),
                    fixed_ktm_point_2->get__y());
                RCLCPP_LINE_INFO();

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "gps_to_slam_converter convert KtS result 2\n\tx : [%f]\n\ty : [%f]\n\tdiffer : [%f]",
                    ktm_current_point_ptr->get__x(),
                    ktm_current_point_ptr->get__y(),
                    differ_2);
                RCLCPP_LINE_INFO();

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "gps_to_slam_converter convert result\n\tcurrent GPS x: : [%f]\n\tcurrent GPS y : [%f]\n\tcurrent SLAM x : [%f]\n\tSLAM y : [%f]",
                    current_gps_lat,
                    current_gps_lon,
                    current_slam_pose_x,
                    current_slam_pose_y);
                RCLCPP_LINE_INFO();

                std::shared_ptr<gps_slam_converter::points::KTMPoint> ktm_point = ktm_math->geo_point_to_ktm_point(current_gps_lat, current_gps_lon);

                std::shared_ptr<gps_slam_converter::math::SLAMPointGenerator> slam_point_generator = std::make_shared<gps_slam_converter::math::SLAMPointGenerator>();
                std::shared_ptr<gps_slam_converter::points::SLAMPoint> slam_point = slam_point_generator->geo_point_to_slam_point(gts_point, ktm_point);

                const double &slam_x = slam_point->get__x();
                const double &slam_y = slam_point->get__y();

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "gps_to_slam_converter convert result\n\tcurrent SLAM x: : [%f]\n\tcurrent SLAM y : [%f]\n\tconverted SLAM x : [%f]\n\ttconverted SLAM y : [%f]",
                    current_slam_pose_x,
                    current_slam_pose_y,
                    slam_x,
                    slam_y);
                RCLCPP_LINE_INFO();

                const double &angle = common_math->angle_between_two_points(current_slam_pose_x, current_slam_pose_y, slam_x, slam_y);

                RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "gps_to_slam_converter convert angle : [%f]", angle);
                RCLCPP_LINE_INFO();

                std::shared_ptr<gps_slam_converter::points::QuaternionPoint> quaternion_point = std::make_shared<gps_slam_converter::points::QuaternionPoint>();
                quaternion_point->euler_angle_to_quaternion(angle, RCL_DEFAULT_DOUBLE, RCL_DEFAULT_DOUBLE);

                const double &slam_z = quaternion_point->get__euler_z();
                const double &slam_w = quaternion_point->get__euler_w();

                RCUTILS_LOG_INFO_NAMED(
                    RCL_NODE_NAME,
                    "gps_to_slam_converter convert slam_pose_quaternion\n\tconverted SLAM z : [%f]\n\tconverted SLAM w : [%f]",
                    slam_z,
                    slam_w);
                RCLCPP_LINE_INFO();

                quaternion_point->set__x(slam_x);
                quaternion_point->set__y(slam_y);
                quaternion_point->set__z(slam_z);
                quaternion_point->set__w(slam_w);

                slam_pose_quaternion->set__x(quaternion_point->get__x());
                slam_pose_quaternion->set__y(quaternion_point->get__y());
                slam_pose_quaternion->set__z(quaternion_point->get__z());
                slam_pose_quaternion->set__w(quaternion_point->get__w());
            }
        }

        geometry_msgs::msg::Quaternion &&quaternion_moved = std::move(*slam_pose_quaternion);
        converted_slam_pose->set__orientation(quaternion_moved);

        return converted_slam_pose;
    }
}