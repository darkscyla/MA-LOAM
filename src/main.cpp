// --- Internal Includes ---
#include <ma_loam/cluster_icp_ros.h>

// --- PCL Includes ---
#include <pcl/io/pcd_io.h>

// --- ROS Includes ---
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>

/**
 * @brief Wrapper class that takes care of subscribing to the sensor data,
 * applying CICP, creating AABB tree and finally solving the minimization
 * problem
 */
class handler {
public:
  using cloud_t = ma_loam::cluster_icp::cloud_t;

  handler()
      : nh_{}, pcl_sub_{nh_.subscribe<cloud_t>("velodyne_points", 1,
                                               &handler::callback, this)},
        translation_{0, 0, 0}, quaternion_{0, 0, 0, 1} {
    // Check if should print optimization solution
    nh_.param<bool>("print_optimization", print_optimization_, false);

    // Print the initial guess
    if (print_optimization_) {
      ROS_INFO("Initial Guess");
      log_info();
    }
  }

  void
  callback(const cloud_t::ConstPtr &_msg) {
    cicp_ros_.set_point_cloud(_msg);

    // Solve the minimization probelm
    ceres::Problem problem;
    cicp_ros_.setup_problem(problem, nullptr, quaternion_, translation_);

    // Add parametrization for quaternion as all 4 components are not
    // independent
    problem.SetParameterization(quaternion_,
                                new ceres::EigenQuaternionParameterization());

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (print_optimization_) {
      ROS_INFO_STREAM(summary.BriefReport());

      ROS_INFO("After minimization");
      log_info();
    }
  }

  void
  log_info() const {
    ROS_INFO_STREAM("Pose [tx, ty, tz, qx, qy, qz, qw]: "
                    << translation_[0] << " " << translation_[1] << " "
                    << translation_[2] << " " << quaternion_[0] << " "
                    << quaternion_[1] << " " << quaternion_[2] << " "
                    << quaternion_[3]);
  };

private:
  // ROS stuff
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;

  // Translation and orientation for ceres
  double translation_[3];
  double quaternion_[4];

  // CICP
  ma_loam::cicp_ros cicp_ros_;
  bool print_optimization_;
};

int
main(int argc, char **argv) {
  // Setup the initial configuration
  ros::init(argc, argv, "cluster_icp_node");
  handler handle; ///> Runs everything in the background

  ros::spin();

  return 0;
}
