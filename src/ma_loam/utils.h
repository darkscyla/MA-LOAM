#pragma once

// --- Internal Includes ---
#include <ma_loam/cluster_icp.h>

// --- ROS Includes ---
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// --- Eigen Includes ---
#include <Eigen/Core>

// --- Standard Includes ---
#include <string>
#include <thread>
#include <vector>

namespace ma_loam {

std::vector<double>
parse_pose(const ros::NodeHandle &_nh, const std::string &_param_name) {
  try {
    std::vector<double> pose;
    const bool valid =
        _nh.getParam(_param_name, pose) &&
        (pose.size() == 6 or pose.size() == 7); // xyz-rpy or xyz-quat

    if (!valid) {
      return {};
    }

    if (pose.size() == 6) {
      tf2::Quaternion quat;
      quat.setRPY(pose[3], pose[4], pose[5]);
      quat.normalize();

      pose.resize(7);
      pose[3] = quat.x();
      pose[4] = quat.y();
      pose[5] = quat.z();
      pose[6] = quat.w();
    }

    return pose;
  } catch (const std::exception &_) {
    return {};
  }
}

void
set_threads(const ros::NodeHandle &nh, unsigned int &_threads) {
  bool parallel;
  nh.param<bool>("parallel_solver", parallel, false);

  if (parallel) {
    const auto num_threads = std::thread::hardware_concurrency();
    if (num_threads) {
      _threads = num_threads;
    }
  }
}

void
set_initial_pose(const ros::NodeHandle &nh, pose_wrapper &_pose) {
  const auto raw_pose = parse_pose(nh, "sensor_pose_init");
  if (raw_pose.size() == 7) {
    // Transform the sensor pose given the robot pose
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);

    geometry_msgs::Pose pose;
    pose.position.x = raw_pose[0];
    pose.position.y = raw_pose[1];
    pose.position.z = raw_pose[2];
    pose.orientation.x = raw_pose[3];
    pose.orientation.y = raw_pose[4];
    pose.orientation.z = raw_pose[5];
    pose.orientation.w = raw_pose[6];

    try {
      const auto transform = tf_buffer.lookupTransform(
          "base_footprint", "velodyne", ros::Time(0), ros::Duration(10));
      tf2::doTransform(pose, pose, transform);
    } catch (const tf2::TransformException &_ex) {
      ROS_ERROR_STREAM("Unable to get the transform between the sensor frame "
                       "and base_footprint: "
                       << _ex.what());
    }

    // Copy over translation
    _pose.trans.x() = pose.position.x;
    _pose.trans.y() = pose.position.y;
    _pose.trans.z() = pose.position.z;

    // Copy over rotation
    _pose.quat.x() = pose.orientation.x;
    _pose.quat.y() = pose.orientation.y;
    _pose.quat.z() = pose.orientation.z;
    _pose.quat.w() = pose.orientation.w;
  } else {
    ROS_INFO("Unable to fetch sensor initial pose");
  }
}

} // namespace ma_loam
