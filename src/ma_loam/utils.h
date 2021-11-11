#pragma once

// --- Internal Includes ---
#include <ma_loam/cluster_icp.h>

// --- ROS Includes ---
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

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
  const auto pose = parse_pose(nh, "sensor_pose_init");
  if (pose.size() == 7) {
    // Copy over translation
    _pose.trans.x() = pose[0];
    _pose.trans.y() = pose[1];
    _pose.trans.z() = pose[2];

    // Copy over rotation
    _pose.quat.x() = pose[3];
    _pose.quat.y() = pose[4];
    _pose.quat.z() = pose[5];
    _pose.quat.w() = pose[6];
  } else {
    ROS_INFO("Unable to fetch sensor initial pose");
  }
}

} // namespace ma_loam
