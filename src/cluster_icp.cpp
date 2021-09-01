// --- Internal Includes ---
#include <ma_loam/cluster_icp.h>

// --- PCL Includes ---
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

namespace ma_loam {

using point_t = cluster_icp::point_t;
using normal_t = cluster_icp::normal_t;

void
cluster_icp::set_input_cloud(cloud_ptr_t _cloud_ptr) {
  cloud_ = std::move(_cloud_ptr);
}

cluster_icp::cloud_normal_ptr_t
cluster_icp::estimate_normals(const size_t _neighbours) const {
  // Create a kd tree for neighbours search
  using search_t = pcl::search::KdTree<point_t>;
  search_t::Ptr kd_tree(new search_t());

  // Setup the normal estimator
  pcl::NormalEstimation<point_t, normal_t> n_est;
  n_est.setInputCloud(cloud_);
  n_est.setSearchMethod(kd_tree);
  n_est.setKSearch(_neighbours);

  // Now compute the normals
  pcl::PointCloud<normal_t>::Ptr cloud_normals(new pcl::PointCloud<normal_t>);
  n_est.compute(*cloud_normals);

  return cloud_normals;
}

cluster_icp::voxel_grid_ptr_t
cluster_icp::voxelize(float _resolution) const {
  voxel_grid_ptr_t voxel_grid(new voxel_grid_t(_resolution));
  voxel_grid->setInputCloud(cloud_);
  voxel_grid->addPointsFromInputCloud();

  return voxel_grid;

  // for (auto it = tree.leaf_depth_begin(), it_end = tree.leaf_depth_end();
  //      it != it_end; ++it, ++idx) {
  //   // Get the indices of the points in the voxel
  //   auto &container = it.getLeafContainer();
  //   const auto &indices = container.getPointIndicesVector();

  //   std::cout << "Voxel[" << idx << "]: "
  //             << "( ";
  //   for (const auto index : indices) {
  //     std::cout << index << " ";
  //   }
  //   std::cout << ")\n";
  // }
}

} // namespace ma_loam
