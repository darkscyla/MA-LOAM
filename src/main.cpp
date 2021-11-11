// --- Internal Includes ---
#include <ma_loam/cluster_icp_ros.h>
#include <ma_loam/utils.h>

// --- PCL Includes ---
#include <pcl/io/pcd_io.h>

// --- ROS Includes ---
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// --- Standard Includes ---
#include <thread>
#include <vector>

/**
 * @brief Wrapper class that takes care of subscribing to the sensor data,
 * applying CICP, creating AABB tree and finally solving the minimization
 * problem
 */
class handler {
public:
  using cloud_t = ma_loam::cluster_icp::cloud_t;

  handler()
      : init_(false), nh_{}, pcl_sub_{nh_.subscribe<cloud_t>(
                                 "velodyne_points", 1, &handler::callback,
                                 this)},
        pose_pub_{
            nh_.advertise<geometry_msgs::PoseStamped>("optimized_pose", 10)},
        translation_{0, 0, 0}, quaternion_{0, 0, 0, 1}, threads_(1) {
    // Setup
    ma_loam::set_threads(nh_, threads_);
    ma_loam::set_initial_pose(nh_, global_pose_);
    set_log_lvl();
    init_ = set_static_tranform();
  }

private:
  bool
  set_static_tranform() {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);

    try {
      transform_ = tf_buffer.lookupTransform("velodyne", "base_footprint",
                                             ros::Time(0), ros::Duration(10));
    } catch (const tf2::TransformException &_ex) {
      ROS_ERROR_STREAM("Unable to get the transform between the sensor frame "
                       "and base_footprint: "
                       << _ex.what());
      return false;
    }
    return true;
  }

  void
  set_log_lvl() {
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
    if (!init_) {
      return;
    }
    const auto stamp = ros::Time::now(); ///> Use received time as stamp
    cicp_ros_.set_input_cloud(_msg);

    // Set up the global pose
    ma_loam::pose_wrapper global_pose;

    // Solve the minimization probelm
    ceres::Problem problem;
    cicp_ros_.setup_problem(problem, new ceres::HuberLoss(1.0), global_pose_, quaternion_,
                            translation_);

    // Add parametrization for quaternion as all 4 components are not
    // independent
    problem.SetParameterization(quaternion_,
                                new ceres::EigenQuaternionParameterization());

    ceres::Solver::Options options;
    options.num_threads = threads_;
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Update the global pose and publish the result
    Eigen::Map<Eigen::Quaterniond> l_quat(quaternion_);
    Eigen::Map<Eigen::Vector3d> l_trans(translation_);
    global_pose_.quat = global_pose_.quat * l_quat;
    global_pose_.trans = global_pose_.trans + global_pose_.quat * l_trans;

    publish_optimized_pose(stamp);

    if (print_optimization_) {
      ROS_INFO_STREAM(summary.BriefReport());

      ROS_INFO("After minimization");
      log_info();
    }
  }

  void
  publish_optimized_pose(const ros::Time &_stamp) const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = _stamp;

    pose.pose.position.x = global_pose_.trans.x();
    pose.pose.position.y = global_pose_.trans.y();
    pose.pose.position.z = global_pose_.trans.z();

    pose.pose.orientation.x = global_pose_.quat.x();
    pose.pose.orientation.y = global_pose_.quat.y();
    pose.pose.orientation.z = global_pose_.quat.z();
    pose.pose.orientation.w = global_pose_.quat.w();
    tf2::doTransform(pose.pose, pose.pose, transform_);

    pose_pub_.publish(pose);
  }

  void
  log_info() const {
    ROS_INFO_STREAM("Pose [tx, ty, tz, qx, qy, qz, qw]: "
                    << global_pose_.trans.x() << " " << global_pose_.trans.y()
                    << " " << global_pose_.trans.z() << " "
                    << global_pose_.quat.x() << " " << global_pose_.quat.y()
                    << " " << global_pose_.quat.z() << " "
                    << global_pose_.quat.w());
  };

  // Initiliazed
  bool init_;

  // ROS stuff
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pose_pub_;
  geometry_msgs::TransformStamped transform_;

  // Local and global translation and orientation for ceres. We do not wrap the
  // local pose for ceres
  double translation_[3];
  double quaternion_[4];
  ma_loam::pose_wrapper global_pose_;

  // CICP
  ma_loam::cicp_ros cicp_ros_;
  unsigned int threads_;
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
