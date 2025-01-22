/*********************************************************************
 *
 * SPDX-License-Identifier: LGPL-3.0
 *
 *  Authors: Mariano Jaimez Tarifa and Javier Monroy
 *           MAPIR group, University of Malaga, Spain
 *           http://mapir.uma.es
 *
 *  Date: January 2016
 *
 * This pkgs offers a fast and reliable estimation of 2D odometry based on planar laser scans.
 * SRF is a fast and precise method to estimate the planar motion of a lidar from consecutive range scans.
 * SRF presents a dense method for estimating planar motion with a laser scanner. Starting from a symmetric
 * representation of geometric consistency between scans, we derive a precise range flow constraint and
 * express the motion of the scan observations as a function of the rigid motion of the scanner.
 * In contrast to existing techniques, which align the incoming scan with either the previous one or the last
 * selected keyscan, we propose a combined and efficient formulation to jointly align all these three scans at
 * every iteration. This new formulation preserves the advantages of keyscan-based strategies but is more robust
 * against suboptimal selection of keyscans and the presence of moving objects.
 *
 *  More Info: http://mapir.isa.uma.es/work/SRF-Odometry
 *********************************************************************/

#include "srf_laser_odometry/nodes/srf_node.h"

using namespace std;
using namespace Eigen;

//-----------------------------------------------------------------------------------
//                                   UTILS
//-----------------------------------------------------------------------------------
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 6, 6> covariance2dTo3d(const Eigen::MatrixBase<Derived> &cov_2d)
{
    static_assert(Eigen::MatrixBase<Derived>::RowsAtCompileTime == 3, "Input arg must be of size 3*3");
    static_assert(Eigen::MatrixBase<Derived>::ColsAtCompileTime == 3, "Input arg must be of size 3*3");

    using T = typename Derived::Scalar;

    Eigen::Matrix<T, 6, 6> cov_3d = Eigen::Matrix<T, 6, 6>::Zero();

    cov_3d.block(0, 0, 2, 2) = cov_2d.block(0, 0, 2, 2);
    cov_3d.block(0, 5, 2, 1) = cov_2d.block(0, 2, 2, 1);
    cov_3d.block(5, 0, 1, 2) = cov_2d.block(2, 0, 1, 2);

    cov_3d(5, 5) = cov_2d(2, 2);

    return cov_3d;
}

// --------------------------
// CLaserOdometry2D Wrapper
//---------------------------
CLaserOdometry2D::CLaserOdometry2D() : Node("SRF_laser_odom")
{
    // Read Parameters
    //----------------
    this->declare_parameter<std::string>("laser_scan_topic", "/laser_scan");
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<std::string>("base_frame_id", "/base_link");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("odom_frame_id", "/odom");
    this->declare_parameter<std::string>("init_pose_from_topic", "");
    this->declare_parameter<int>("laser_decimation", 1);
    this->declare_parameter<double>("laser_min_range", -1.0);
    this->declare_parameter<double>("laser_max_range", -1.0);
    this->declare_parameter<double>("increment_covariance_threshold", std::numeric_limits<double>::max());
    this->declare_parameter<std::string>("operation_mode",
                                         "HYBRID"); // CS=consecutiveScans, KS=keyScans, HYBRID=threeScansWithKeyScan

    auto laser_scan_topic = this->get_parameter("laser_scan_topic").get_value<std::string>();
    publish_tf_ = this->get_parameter("publish_tf").get_value<bool>();
    base_frame_id_ = this->get_parameter("base_frame_id").get_value<std::string>();
    auto odom_topic = this->get_parameter("odom_topic").get_value<std::string>();
    odom_frame_id_ = this->get_parameter("odom_frame_id").get_value<std::string>();
    auto init_pose_from_topic = this->get_parameter("init_pose_from_topic").get_value<std::string>();
    laser_decimation_ = this->get_parameter("laser_decimation").get_value<int>();
    laser_min_range_ = this->get_parameter("laser_min_range").get_value<double>();
    laser_max_range_ = this->get_parameter("laser_max_range").get_value<double>();
    increment_covariance_threshold_ = this->get_parameter("increment_covariance_threshold").get_value<double>();
    operation_mode_ = this->get_parameter("operation_mode").get_value<std::string>();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Publishers and Subscribers
    //--------------------------
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_scan_topic, 1, std::bind(&CLaserOdometry2D::laser_callback, this, std::placeholders::_1));
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
    laser_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("srf_laser_truncated", 5);

    // init pose
    //----------
    if (init_pose_from_topic != "")
    {
        initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            init_pose_from_topic, 1, std::bind(&CLaserOdometry2D::init_pose_callback, this, std::placeholders::_1));
        GT_pose_initialized = false;
    }
    else
    {
        GT_pose_initialized = true;
        initial_robot_pose.pose.pose.position.x = 0;
        initial_robot_pose.pose.pose.position.y = 0;
        initial_robot_pose.pose.pose.position.z = 0;
        initial_robot_pose.pose.pose.orientation.x = 0;
        initial_robot_pose.pose.pose.orientation.y = 0;
        initial_robot_pose.pose.pose.orientation.z = 0;
        initial_robot_pose.pose.pose.orientation.w = 1;
    }

    // Init variables
    //----------------
    module_initialized = false;
    first_laser_scan = true;
    laser_counter = 0;
}

bool CLaserOdometry2D::is_initialized()
{
    return module_initialized;
}

bool CLaserOdometry2D::scan_available()
{
    return new_scan_available;
}

void CLaserOdometry2D::init()
{
    const unsigned int scan_size = last_scan.ranges.size();            // Num of samples (size) of the scan laser
    const float fov = fabs(last_scan.angle_max - last_scan.angle_min); // Horizontal Laser's FOV

    // Init core class with Laser specs and corresponding operation_mode
    if (!strcmp(operation_mode_.c_str(), "CS"))
        srf_obj_.initialize(scan_size, fov, 0);
    else if (!strcmp(operation_mode_.c_str(), "KS"))
        srf_obj_.initialize(scan_size, fov, 1);
    else if (!strcmp(operation_mode_.c_str(), "HYBRID"))
        srf_obj_.initialize(scan_size, fov, 2);
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Operation mode not implemented. Using default HYBRID");
        srf_obj_.initialize(scan_size, fov, 2);
    }

    // Set laser pose on the robot (through tF)
    //  This allow estimation of the odometry with respect to the robot base reference system.
    geometry_msgs::msg::Transform LaserPoseOnTheRobot;
    try
    {
        LaserPoseOnTheRobot =
            tf_buffer_->lookupTransform(base_frame_id_, last_scan.header.frame_id, tf2::TimePointZero).transform;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        // rclcpp::Duration(1.0).sleep();
    }

    // Keep this transform as Eigen Matrix3d
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(LaserPoseOnTheRobot.translation.x, LaserPoseOnTheRobot.translation.y,
                                     LaserPoseOnTheRobot.translation.z));
    transform.setRotation(tf2::Quaternion(LaserPoseOnTheRobot.rotation.x, LaserPoseOnTheRobot.rotation.y,
                                          LaserPoseOnTheRobot.rotation.z, LaserPoseOnTheRobot.rotation.w));
    const tf2::Matrix3x3 &basis = transform.getBasis();
    Eigen::Matrix3d R;

    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R(r, c) = basis[r][c];

    laser_tf_ = Pose3d(R);

    const tf2::Vector3 &t = transform.getOrigin();
    laser_tf_.translation()(0) = t[0];
    laser_tf_.translation()(1) = t[1];
    laser_tf_.translation()(2) = t[2];

    if (!module_initialized)
    {
        // Robot initial pose
        auto p = initial_robot_pose.pose.pose;
        tf2::Transform initial_robot_transform;
        initial_robot_transform.setOrigin(tf2::Vector3(p.position.x, p.position.y, p.position.z));
        initial_robot_transform.setRotation(
            tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
        set_laser_pose_from_robot_transform(initial_robot_transform);
    }
    else
    {
        // Last odometry estimation
        auto robot_transform_msg = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, tf2::TimePointZero);
        tf2::Transform robot_transform;
        tf2::fromMsg(robot_transform_msg.transform, robot_transform);
        set_laser_pose_from_robot_transform(robot_transform);
    }

    module_initialized = true;
    last_odom_time = this->get_clock()->now();
}

void CLaserOdometry2D::set_laser_pose_from_robot_transform(const tf2::Transform &initial_robot_transform)
{
    const tf2::Matrix3x3 &basis2 = initial_robot_transform.getBasis();
    Eigen::Matrix3d R2;

    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R2(r, c) = basis2[r][c];

    Pose3d init_pose(R2);

    const tf2::Vector3 &t2 = initial_robot_transform.getOrigin();
    init_pose.translation()(0) = t2[0];
    init_pose.translation()(1) = t2[1];
    init_pose.translation()(2) = t2[2];

    auto init_laser_tf = init_pose * laser_tf_;

    srf_obj_.laser_pose = Pose2d::Identity();
    srf_obj_.laser_pose.translation().x() = init_laser_tf.translation().x();
    srf_obj_.laser_pose.translation().y() = init_laser_tf.translation().y();

    Eigen::Matrix3d rotation_matrix_3d = init_laser_tf.rotation();
    double yaw_angle = std::atan2(rotation_matrix_3d(1, 0), rotation_matrix_3d(0, 0));
    Eigen::Rotation2Df rotation_2d(static_cast<float>(yaw_angle));

    srf_obj_.laser_pose.linear() = rotation_2d.toRotationMatrix();
    RCLCPP_DEBUG(this->get_logger(), "Laser pose set to: %.3f, %.3f, %.3f", srf_obj_.laser_pose.translation().x(),
                 srf_obj_.laser_pose.translation().y(), yaw_angle);
}

void CLaserOdometry2D::reset()
{
    first_laser_scan = true;
    laser_counter = 0;
}

void CLaserOdometry2D::publish_pose_from_SRF()
{
    // GET ROBOT POSE from LASER POSE
    //--------------------------------
    auto laser_pose_3d = Pose3d::Identity();
    laser_pose_3d.translation()(0) = srf_obj_.laser_pose.translation().x();
    laser_pose_3d.translation()(1) = srf_obj_.laser_pose.translation().y();
    double yaw = atan2(srf_obj_.laser_pose.rotation().matrix()(1, 0), srf_obj_.laser_pose.rotation().matrix()(0, 0));
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    laser_pose_3d.linear() = quat.toRotationMatrix();

    // Get the robot pose
    robot_pose = laser_pose_3d * laser_tf_.inverse();
    // robot_pose = srf_obj_.laser_pose * laser_tf_.inverse();

    // Estimate linear/angular speeds (mandatory for base_local_planner)
    // last_scan -> the last scan received (practically now)
    // last_odom_time -> The time of the previous scan lasser used to estimate the pose
    //-------------------------------------------------------------------------------------
    double time_inc_sec = (rclcpp::Time(last_scan.header.stamp) - last_odom_time).seconds();
    last_odom_time = last_scan.header.stamp;
    if (time_inc_sec <= 0)
        RCLCPP_WARN(this->get_logger(), "Time increment between Odom estimation is: %.6f sec", time_inc_sec);
    else
    {
        double cov_odo_2d_diag = srf_obj_.get_increment_covariance().diagonal().sum();
        if (cov_odo_2d_diag > increment_covariance_threshold_ || cov_odo_2d_diag != cov_odo_2d_diag)
        {
            RCLCPP_WARN(this->get_logger(), "Increment covariance is too high: %.6f - resetting", cov_odo_2d_diag);
            reset();
            return;
        }

        double lin_speed_x = srf_obj_.kai_loc(0) / time_inc_sec;
        double lin_speed_y = srf_obj_.kai_loc(1) / time_inc_sec;
        double ang_speed = srf_obj_.kai_loc(2) / time_inc_sec;
        robot_oldpose = robot_pose;

        // first, we'll publish the odometry over tf
        //---------------------------------------
        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = this->get_clock()->now();
            odom_trans.header.frame_id = odom_frame_id_;
            odom_trans.child_frame_id = base_frame_id_;
            odom_trans.transform.translation.x = robot_pose.translation().x();
            odom_trans.transform.translation.y = robot_pose.translation().y();

            float yaw = atan2(robot_pose.rotation().matrix()(1, 0), robot_pose.rotation().matrix()(0, 0));
            Eigen::Quaternionf quat;
            quat = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

            odom_trans.transform.rotation.z = quat.z();
            odom_trans.transform.rotation.w = quat.w();
            // send the transform
            tf_broadcaster_->sendTransform(odom_trans);
        }

        // next, we'll publish the odometry message over ROS topic
        //-------------------------------------------------------
        nav_msgs::msg::Odometry odom;
        // odom.header.stamp = ros::Time::now();
        odom.header.stamp = last_scan.header.stamp;
        odom.header.frame_id = odom_frame_id_;
        // set the position
        odom.pose.pose.position.x = robot_pose.translation().x();
        odom.pose.pose.position.y = robot_pose.translation().y();

        auto cov_odo_3d = covariance2dTo3d(srf_obj_.get_increment_covariance());

        std::copy(cov_odo_3d.data(), cov_odo_3d.data() + cov_odo_3d.size(), odom.pose.covariance.begin());

        float yaw = atan2(robot_pose.rotation().matrix()(1, 0), robot_pose.rotation().matrix()(0, 0));
        Eigen::Quaternionf quat;
        quat = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        // set the velocity
        odom.child_frame_id = base_frame_id_;
        odom.twist.twist.linear.x = lin_speed_x; // linear speed
        odom.twist.twist.linear.y = lin_speed_y;
        odom.twist.twist.angular.z = ang_speed; // angular speed
        // publish the message
        odom_pub->publish(odom);
    }

    // Clear current laser
    new_scan_available = false;
}

//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2D::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr new_scan)
{
    if (GT_pose_initialized)
    {
        // Keep in memory the last received laser_scan
        last_scan = *new_scan;
        laser_counter++;
        // RCLCPP_INFO([SRF] "Laser Counter %u",laser_counter);

        // Initialize module on first scan
        if (first_laser_scan)
        {
            init();
            first_laser_scan = false;
            // Load first scan and build pyramid
            for (unsigned int i = 0; i < last_scan.ranges.size(); i++)
                srf_obj_.range_wf(i) = last_scan.ranges[i];
            srf_obj_.create_scan_pyramid();

            // Set laser min-max distances (if not set as parameters)
            if (laser_min_range_ == -1)
                laser_min_range_ = 0.0;
            if (laser_max_range_ == -1)
                laser_max_range_ = 0.99f * last_scan.range_max;
        }
        else
        {
            if (laser_counter % laser_decimation_ == 0)
            {
                // copy laser scan to internal srf variable
                for (unsigned int i = 0; i < last_scan.ranges.size(); i++)
                {
                    // Check min-max distances, e.g. to avoid including points of the own robot
                    if ((last_scan.ranges[i] > laser_max_range_) || (last_scan.ranges[i] < laser_min_range_))
                    {
                        srf_obj_.range_wf(i) = 0.f; // invalid measurement
                        last_scan.ranges[i] = 0.0;
                    }
                    else
                        srf_obj_.range_wf(i) = last_scan.ranges[i];
                }

                // Publish truncated laser for visualization
                laser_pub->publish(last_scan);

                // Check the average laser measurement
                float sum = 0.f;
                for (unsigned int i = 0; i < last_scan.ranges.size(); i++)
                {
                    sum += srf_obj_.range_wf(i);
                }

                RCLCPP_DEBUG(this->get_logger(), "The average laser measurement is %f", sum);

                // Process odometry estimation
                srf_obj_.odometry_calculation();
                publish_pose_from_SRF();
            }
        }
    }
}

void CLaserOdometry2D::init_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr new_initPose)
{
    // Initialize robot pose on first GT pose. Else do Nothing!
    // Usefull for comparison with other odometry methods.
    if (!GT_pose_initialized)
    {
        initial_robot_pose = *new_initPose;
        GT_pose_initialized = true;
    }
}

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CLaserOdometry2D>());
    rclcpp::shutdown();
    return 0;
}
