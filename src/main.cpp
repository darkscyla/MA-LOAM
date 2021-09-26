// --- Internal Includes ---
#include <ma_loam/cluster_icp.h>

// --- PCL Includes ---
#include <pcl/io/pcd_io.h>

// --- ROS Includes ---
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// --- Standard Includes ---
#include <future>

visualization_msgs::MarkerArray
to_marker_array(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
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

    // Print some info regarding the voxel
    constexpr bool print_voxel_bounds = false;
    if (print_voxel_bounds) {
      Eigen::Vector3f min_pt, max_pt;
      _voxel_grid->getVoxelBounds(itt, min_pt, max_pt);
      std::cout << "x -> ( " << min_pt.x() << ", " << max_pt.x()
                << ", d: " << max_pt.x() - min_pt.x() << " ) ";
      std::cout << "y -> ( " << min_pt.y() << ", " << max_pt.y()
                << ", d: " << max_pt.y() - min_pt.y() << " ) ";
      std::cout << "z -> ( " << min_pt.z() << ", " << max_pt.z()
                << ", d: " << max_pt.z() - min_pt.z() << " )\n";
    }

    // For each voxel leaf, we assign a unique color (rand is a bad way of
    // generating rnadom numbers but we dont care at the moment as it is just
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

int
main(int argc, char **argv) {
  // Parameters
  const size_t normals_neighbors = 10; ///> neighbors to search for normals
  const float voxel_size = 0.1;        ///> in meters (point cloud units)
  const size_t clusters = 3;           ///> Max features per voxel to retain
  const size_t elem_cluster = 2;       ///> Min elements for cluster to be valid
  const float eps_cluster = 0.25;      ///> Difference to tolerate for merging
                                  ///> This distance d can be converted to angle
                                  ///> using theta = 2 * asin( d / 2)

  const float scale = 0.05f; ///> Visualization scale for normals in rviz

  // Actual implementation
  using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
  cloud_t::Ptr cloud(new cloud_t);
  pcl::io::loadPCDFile(ros::package::getPath("ma_loam") +
                           "/resources/pcls/simple_corridor_sample.pcd",
                       *cloud);
  cloud->header.frame_id = "map";

  ma_loam::cluster_icp cicp;
  cicp.set_input_cloud(cloud);

  // Compute normals in another thead while voxelizing the point cloud, normals
  // might potentially take time to compute
  auto normal_future =
      std::async(std::launch::async, &ma_loam::cluster_icp::estimate_normals,
                 cicp, normals_neighbors);
  auto voxel_future = std::async(
      std::launch::async, &ma_loam::cluster_icp::voxelize, cicp, voxel_size);

  const auto cloud_normals = normal_future.get();
  const auto voxel_grid = voxel_future.get();

  // Go over the voxel grid and try to prune them using k means clustering
  cloud_t::Ptr pruned_cloud(new cloud_t);
  pruned_cloud->header.frame_id = cloud->header.frame_id;

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
        optimizer.optimize(clusters, elem_cluster, eps_cluster);

    // Add the optimized points to the pruned point cloud by taking their
    // geometric mean
    for (const auto &opt_indices : cluster_indices) {
      Eigen::Vector3f center = Eigen::Vector3f::Zero();
      for (const auto local_idx : opt_indices) {
        const auto global_idx = indicies[local_idx];
        center += cloud->points[global_idx].getVector3fMap();
      }
      center /= opt_indices.size();
      pruned_cloud->push_back({center.x(), center.y(), center.z()});
    }
  }

  // Load in the environemnt model
  ma_loam::aabb_tree_mesh mesh(
      ros::package::getPath("ma_loam") +
      "/resources/environments/simple_corridor/simple_corridor.stl");

  // Setup the initial configuration
  const double translation_init[3] = {0, 0, 0}; ///> xyz
  double translation[3];
  std::copy(std::begin(translation_init), std::end(translation_init),
            std::begin(translation));

  const double quaternion_init[4] = {0, 0, 0, 1}; ///> xyzw
  double quaternion[4];
  std::copy(std::begin(quaternion_init), std::end(quaternion_init),
            std::begin(quaternion));

  ceres::Problem problem;

  // Add the point to point on mesh cost
  for (const auto &point : pruned_cloud->points) {
    problem.AddResidualBlock(ma_loam::point_to_mesh_cost::create(mesh, point),
                             nullptr, quaternion, translation);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial translation: " << translation_init[0] << " "
            << translation_init[1] << " " << translation_init[2] << "\n";
  std::cout << "Final translation: " << translation[0] << " " << translation[1]
            << " " << translation[2] << "\n";
  std::cout << "Initial quaternion: " << quaternion_init[0] << " "
            << quaternion_init[1] << " " << quaternion_init[2] << " "
            << quaternion_init[3] << "\n";
  std::cout << "Final quaternion: " << quaternion[0] << " " << quaternion[1]
            << " " << quaternion[2] << " " << quaternion[3] << "\n";

  // Publish to ROS
  {
    const auto normals_markers =
        to_marker_array(cloud, cloud_normals, voxel_grid, scale);

    // Publish the normals to rviz
    ros::init(argc, argv, "normals_publisher_node");

    ros::NodeHandle nh;
    ros::Publisher normal_pub =
        nh.advertise<visualization_msgs::MarkerArray>("surface_normals", 1);
    ros::Publisher original_pub = nh.advertise<cloud_t>("original_pcl", 1);
    ros::Publisher pruned_pub = nh.advertise<cloud_t>("pruned_pcl", 1);

    ros::Rate rate(1);

    while (ros::ok()) {
      normal_pub.publish(normals_markers);
      original_pub.publish(cloud);
      pruned_pub.publish(pruned_cloud);
      rate.sleep();
    }
  }

  return 0;
}
