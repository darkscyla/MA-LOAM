#pragma once

// --- Internal Includes ---
#include <ma_loam/cluster_icp.h>

// --- ROS Includes ---
#include <ros/ros.h>

// --- Ceres Includes ---
#include <ceres/ceres.h>

// --- Standard Includes ---
#include <memory>
#include <string>
#include <vector>

namespace ma_loam {

class cicp_ros {
public:
  using cloud_t = cluster_icp::cloud_t;
  using cloud_ptr_t = cluster_icp::cloud_ptr_t;
  using cloud_ptr_const_t = cloud_t::ConstPtr;

  /**
   * @brief Creates the underlying cicp related objects including loading the
   * AABB tree. It can later be queried for pruned point cloud or add
   * contribution to ceres problem
   */
  cicp_ros();

  /**
   * @brief Given the point cloud, processes it using clustering and stores the
   * pruned point cloud internally
   *
   * @param _cloud Input point cloud on which clustering is applied. The
   * resulting point cloud is sotred internally
   */
  void
  set_input_cloud(cloud_ptr_const_t _cloud);

  /**
   * @brief Add the contribution of the underlying point cloud to Ceres problem
   *
   * @note Remember to correctly add the quaternion parametrization to the
   * problem
   *
   * @param[in] _problem The problem to which the contribution is added
   * @param[in] _loss_function The loss function to use for the minimization
   * @param[in] _global_pose Reference to the global pose. It is used to
   * transform the local points to global
   * @param[in] _quaternion Pointer to quaternion array to optimize
   * @param[in] _translation Pointer to translation array to optimize
   * @param[in] _weight Weight of the mesh feature
   */
  void
  setup_problem(ceres::Problem &_problem, ceres::LossFunction *_loss_function,
                const pose_wrapper &_global_pose, double *_quaternion,
                double *_translation, double _weight) const;

  /**
   * @brief Returns the number of points in the point cloud
   *
   * @return size_t Size of the clustered point cloud
   */
  size_t
  size() const {
    return cloud_->points.size();
  }

private:
  /**
   * @brief Helper method to get the absolute path of the environment stl file
   *
   * @return std::string Absolute path of environment stl file
   */
  std::string
  fetch_environment_path() const;

  /**
   * @brief Loads in the parameters from the ROS param server
   */
  void
  load_params();

  ros::NodeHandle nh_;

  // MA-LOAM underlying AABB tree
  aabb_tree_mesh mesh_;

  // Pruned point cloud after clustering and their original sizes
  cloud_ptr_t cloud_;
  std::vector<size_t> sizes_;

  // CICP Parameters
  int nn_normals_;   ///> neighbors to search for normals computation
  int max_features_; ///> Max features per voxel to retain
  int min_elements_; ///> Min elements for feature to be valid
  float voxel_size_; ///> in meters (i.e. point cloud units)
  float merge_eps_;  ///> Difference to tolerate for merging
                     ///> This distance d can be converted to angle
                     ///> using theta = 2 * asin( d / 2)

  // Optimization parameters
  int max_mesh_features_; ///> Maximum mesh features to consider

  // Visualization
  using pub_ptr = std::unique_ptr<ros::Publisher>;

  pub_ptr normals_pub_;    ///> Publish normals as marker array
  pub_ptr orig_pcl_pub_;   ///> Original point cloud publisher
  pub_ptr pruned_pcl_pub_; ///> Point cloud after CICP publisher

  float scale_;    ///> Visualization scale for normals in rviz
  bool visualize_; ///> To visualize or not to visualize
};

} // namespace ma_loam
