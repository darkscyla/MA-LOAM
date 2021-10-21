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
    set_threads();
    set_initial_pose();
    set_log_lvl();
    init_ = set_static_tranform();
  }

private:
  void
  set_threads() {
    bool parallel;
    nh_.param<bool>("parallel_solver", parallel, false);

    if (parallel) {
      const auto num_threads = std::thread::hardware_concurrency();
      if (num_threads) {
        threads_ = num_threads;
      }
    }
  }

  void
  set_initial_pose() {
    const auto pose = ma_loam::parse_pose(nh_, "sensor_pose_init");
    if (pose.size() == 7) {
      std::copy(pose.begin(), pose.begin() + 3, std::begin(translation_));
      std::copy(pose.begin() + 3, pose.end(), std::begin(quaternion_));
    } else {
      ROS_INFO("Unable to fetch sensor initial pose");
    }
  }

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

    cicp_ros_.set_point_cloud(_msg);

    // Solve the minimization probelm
    ceres::Problem problem;
    cicp_ros_.setup_problem(problem, nullptr, quaternion_, translation_);

    // Add parametrization for quaternion as all 4 components are not
    // independent
    problem.SetParameterization(quaternion_,
                                new ceres::EigenQuaternionParameterization());

    ceres::Solver::Options options;
    options.num_threads = threads_;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
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

    pose.pose.position.x = translation_[0];
    pose.pose.position.y = translation_[1];
    pose.pose.position.z = translation_[2];

    pose.pose.orientation.x = quaternion_[0];
    pose.pose.orientation.y = quaternion_[1];
    pose.pose.orientation.z = quaternion_[2];
    pose.pose.orientation.w = quaternion_[3];
    tf2::doTransform(pose.pose, pose.pose, transform_);

    pose_pub_.publish(pose);
  }

  void
  log_info() const {
    ROS_INFO_STREAM("Pose [tx, ty, tz, qx, qy, qz, qw]: "
                    << translation_[0] << " " << translation_[1] << " "
                    << translation_[2] << " " << quaternion_[0] << " "
                    << quaternion_[1] << " " << quaternion_[2] << " "
                    << quaternion_[3]);
  };

  // Initiliazed
  bool init_;

  // ROS stuff
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pose_pub_;
  geometry_msgs::TransformStamped transform_;

  // Translation and orientation for ceres
  double translation_[3];
  double quaternion_[4];

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
