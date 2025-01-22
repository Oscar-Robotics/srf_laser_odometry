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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// SRF
#include "srf_laser_odometry/utilities/laser_odometry_refscans.h"

using namespace srf;

class CLaserOdometry2D : public rclcpp::Node
{
  public:
    // Variables
    std::string base_frame_id_;
    std::string odom_topic, odom_frame_id_;
    std::string init_pose_from_topic;
    std::string operation_mode_;
    double laser_min_range_, laser_max_range_, increment_covariance_threshold_;
    bool publish_tf_;
    Pose3d robot_pose, robot_oldpose;
    int laser_counter, laser_decimation_;

    Pose3d laser_tf_;

    // Core class of SRF
    SRF_RefS srf_obj_;

    // methods
    CLaserOdometry2D();
    bool is_initialized();
    bool scan_available();
    void init();
    void set_laser_pose_from_robot_transform(const tf2::Transform &init_pose);
    void reset();
    void publish_pose_from_SRF(); // Publishes the last odometric pose with ROS format

  protected:
    sensor_msgs::msg::LaserScan last_scan;
    bool module_initialized, first_laser_scan, new_scan_available, GT_pose_initialized;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; // Do not put inside the callback
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
    rclcpp::Time last_odom_time;
    nav_msgs::msg::Odometry initial_robot_pose;

    // Subscriptions & Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initPose_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;

    // CallBacks
    void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr new_scan);
    void init_pose_callback(nav_msgs::msg::Odometry::ConstSharedPtr new_initPose);
};
