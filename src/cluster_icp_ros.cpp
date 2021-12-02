// --- Internal Includes ---
#include <ma_loam/cluster_icp_ros.h>

// --- ROS Includes ---
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// --- Standard Includes ---
#include <future>

namespace ma_loam {

namespace visualization {

visualization_msgs::MarkerArray
to_marker_array(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &_normals,
    const pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::Ptr
        &_voxel_grid,
    const float _scale = 1.0f) {

  // Define a default marker
  visualization_msgs::Marker def_marker;

  def_marker.header.frame_id = "map";
  def_marker.header.stamp = ros::Time();
  def_marker.ns = "cloud_normals";
  def_marker.type = visualization_msgs::Marker::ARROW;
  def_marker.action = visualization_msgs::Marker::ADD;

  def_marker.points.resize(2);
  def_marker.pose.orientation.w = 1;

  def_marker.scale.x = 0.1 * _scale;
  def_marker.scale.y = 0.2 * _scale;
  def_marker.color.a = 1;

  const auto size = _cloud->size();
  assert(size == _normals->size());

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.reserve(size);

  for (auto itt = _voxel_grid->leaf_depth_begin(),
            itt_end = _voxel_grid->leaf_depth_end();
       itt != itt_end; ++itt) {
    // For each voxel leaf, we assign a unique color (rand is a bad way of
    // generating random numbers but we dont care at the moment as it is just
    // debug info)
    def_marker.color.r = ((float)rand()) / (RAND_MAX);
    def_marker.color.g = ((float)rand()) / (RAND_MAX);
    def_marker.color.b = ((float)rand()) / (RAND_MAX);

    for (const auto idx : itt.getLeafContainer().getPointIndicesVector()) {
      // Must be inside bounds, should never happen
      assert(static_cast<size_t>(idx) < size);

      // Update the position
      def_marker.points[0].x = _cloud->points[idx].x;
      def_marker.points[0].y = _cloud->points[idx].y;
      def_marker.points[0].z = _cloud->points[idx].z;

      def_marker.points[1].x =
          _cloud->points[idx].x + _scale * _normals->points[idx].normal_x;
      def_marker.points[1].y =
          _cloud->points[idx].y + _scale * _normals->points[idx].normal_y;
      def_marker.points[1].z =
          _cloud->points[idx].z + _scale * _normals->points[idx].normal_z;

      // Give itt a unique id
      def_marker.id = idx;

      marker_array.markers.push_back(def_marker);
    }
  }

  return marker_array;
}

} // namespace visualization

cicp_ros::cicp_ros()
    : nh_{}, mesh_{fetch_environment_path()}, cloud_(new cloud_t) {
  // Load in all the parameters
  load_params();
}

void
cicp_ros::set_input_cloud(cloud_ptr_const_t _input_cloud) {
  ma_loam::cluster_icp cicp;
  cicp.set_input_cloud(_input_cloud);

  // Compute normals in another thead while voxelizing the point cloud, normals
  // might potentially take time to compute
  auto normal_future =
      std::async(std::launch::async, &ma_loam::cluster_icp::estimate_normals,
                 cicp, nn_normals_);
  auto voxel_future = std::async(
      std::launch::async, &ma_loam::cluster_icp::voxelize, cicp, voxel_size_);

  const auto cloud_normals = normal_future.get();
  const auto voxel_grid = voxel_future.get();

  // Go over the voxel grid and try to prune them using k means clustering
  cloud_->clear();
  cloud_->header.frame_id = _input_cloud->header.frame_id;

  for (auto itt = voxel_grid->leaf_depth_begin(),
            itt_end = voxel_grid->leaf_depth_end();
       itt != itt_end; ++itt) {
    const auto &indicies = itt.getLeafContainer().getPointIndicesVector();

    // 4 below represents 4 dimensions for clustering namely x, y, z and
    // curvature
    ma_loam::kmeans_optimizer optimizer(indicies.size(), 4);

    // Copy over the data
    std::vector<float> pt_data;
    for (const auto idx : indicies) {
      const auto &pt = cloud_normals->points[idx];
      pt_data = {pt.normal_x, pt.normal_y, pt.normal_z, pt.curvature};
      optimizer.addDataPoint(pt_data);
    }

    // Optimize the points
    const auto cluster_indices =
        optimizer.optimize(max_features_, min_elements_, merge_eps_);

    // Add the optimized points to the pruned point cloud by taking their
    // geometric mean
    for (const auto &opt_indices : cluster_indices) {
      Eigen::Vector3f center = Eigen::Vector3f::Zero();
      for (const auto local_idx : opt_indices) {
        const auto global_idx = indicies[local_idx];
        center += _input_cloud->points[global_idx].getVector3fMap();
      }
      center /= opt_indices.size();
      cloud_->push_back({center.x(), center.y(), center.z()});
    }
  }

  if (visualize_) {
    // Publish the info
    normals_pub_->publish(visualization::to_marker_array(
        _input_cloud, cloud_normals, voxel_grid, scale_));
    orig_pcl_pub_->publish(_input_cloud);
    pruned_pcl_pub_->publish(cloud_); // We moved the pruned cloud
  }
}

void
cicp_ros::setup_problem(ceres::Problem &_problem,
                        ceres::LossFunction *_loss_function,
                        const pose_wrapper &_global_pose, double *_quaternion,
                        double *_translation, const double _weight) const {
  // Add contribution only if mesh exists
  if (!mesh_.empty()) {
    for (const auto &point : cloud_->points) {
      _problem.AddResidualBlock(
          point_to_mesh_cost::create(mesh_, point, _global_pose, _weight),
          _loss_function, _quaternion, _translation);
    }
  }
}

std::string
cicp_ros::fetch_environment_path() const {
  // Get the base environment name
  std::string env_name;
  if (!nh_.getParam("environment_name", env_name)) {
    ROS_WARN(
        "CICP ROS: Unable to fetch environment name from the param server");
    return {};
  }

  // Get the post-fix tag. This allows us to specify different models for LIDAR
  // scan and point to mesh correspondences
  std::string postfix;
  nh_.param<std::string>("mesh_postfix", postfix, "");

  const std::string full_path = ros::package::getPath("ma_loam") +
                                "/resources/environments/" + env_name + "/" +
                                env_name + postfix + ".stl";
  ROS_INFO_STREAM("CICP ROS: Full path to mesh file is: " << full_path);
  return full_path;
}

void
cicp_ros::load_params() {
  // Update the threshold for AABB tree if it exists and is positive
  {
    float threshold;
    if (nh_.getParam("distance_threshold", threshold) && threshold >= 0) {
      mesh_.set_distance_threshold(threshold);
    }
  }

  // Get parameters for nearest neighbors and k means clustering
  nh_.param<int>("nn_normals", nn_normals_, 10);
  nh_.param<int>("max_features", max_features_, 3);
  nh_.param<int>("min_elements", min_elements_, 1);

  nh_.param<float>("voxel_size", voxel_size_, 0.1);
  nh_.param<float>("merge_eps", merge_eps_, 0.25);

  // We do not setup visualization stuff unless requested
  nh_.param<bool>("visualize", visualize_, false);
  if (visualize_) {
    normals_pub_ = std::make_unique<ros::Publisher>(
        nh_.advertise<visualization_msgs::MarkerArray>("surface_normals", 1));
    orig_pcl_pub_ = std::make_unique<ros::Publisher>(
        nh_.advertise<cloud_t>("original_pcl", 1));
    pruned_pcl_pub_ = std::make_unique<ros::Publisher>(
        nh_.advertise<cloud_t>("pruned_pcl", 1));

    nh_.param<float>("scale", scale_, 0.05);
  }
}

} // namespace ma_loam
