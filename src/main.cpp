// --- Internal Includes ---
#include <ma_loam/cluster_icp.h>

// --- PCL Includes ---
#include <pcl/io/pcd_io.h>

// --- ROS Includes ---
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(ros::package::getPath("ma_loam") +
                           "/resources/pcls/simple_corridor_sample.pcd",
                       *cloud);

  ma_loam::cluster_icp cicp;
  cicp.set_input_cloud(cloud);

  // Compute normals in another thead while voxelizing the point cloud, normals
  // might potentially take time to compute
  auto normal_future = std::async(
      std::launch::async, &ma_loam::cluster_icp::estimate_normals, cicp, 10);
  auto voxel_future = std::async(std::launch::async,
                                 &ma_loam::cluster_icp::voxelize, cicp, 0.1);

  const auto cloud_normals = normal_future.get();
  const auto voxel_grid = voxel_future.get();

  // Publish to ROS
  {
    const float scale = 0.1f;
    const auto normals_markers =
        to_marker_array(cloud, cloud_normals, voxel_grid, scale);

    // Publish the normals to rviz
    ros::init(argc, argv, "normals_publisher_node");

    ros::NodeHandle nh;
    ros::Publisher pub =
        nh.advertise<visualization_msgs::MarkerArray>("surface_normals", 1);
    ros::Rate rate(1);

    while (ros::ok()) {
      pub.publish(normals_markers);
      rate.sleep();
    }
  }

  return 0;
}
