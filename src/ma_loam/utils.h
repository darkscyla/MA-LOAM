// --- ROS Includes ---
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

// --- Standard Includes ---
#include <string>
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

} // namespace ma_loam
