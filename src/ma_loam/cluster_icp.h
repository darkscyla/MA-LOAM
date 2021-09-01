// --- PCL Includes ---
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ma_loam {

class cluster_icp {
public:
  // Type defs
  using point_t = pcl::PointXYZ;

  using cloud_t = pcl::PointCloud<point_t>;
  using normal_t = pcl::Normal;
  using cloud_normal_t = pcl::PointCloud<normal_t>;

  using cloud_ptr_t = cloud_t::Ptr;
  using cloud_normal_ptr_t = cloud_normal_t::Ptr;

  using voxel_grid_t = pcl::octree::OctreePointCloudPointVector<point_t>;
  using voxel_grid_ptr_t = voxel_grid_t::Ptr;

  cluster_icp() = default;

  /**
   * @brief Set the input cloud for the cluster icp
   *
   * @param _cloud_ptr Shared pointer to the point cloud
   */
  void
  set_input_cloud(cloud_ptr_t _cloud_ptr);

  /**
   * @brief Estimates the normals of each point in the cloud using the given
   * number of neighbours. We basically perform principal component analysis
   * followed by computing the most dominant eigen vector
   *
   * @param _neighbours Number of neighbours to consider while computing the
   * normal
   * @return cloud_normal_ptr_t Point cloud of surface normals at each input
   * cloud point
   */
  cloud_normal_ptr_t
  estimate_normals(size_t _neighbours) const;

  /**
   * @brief Puts all the points into a voxel grid
   *
   * @param _resolution Size of the voxel bin
   * @return voxel_grid_ptr_t Voxel grid containing the point cloud
   */
  voxel_grid_ptr_t
  voxelize(float _resolution) const;

private:
  cloud_ptr_t cloud_;
};

} // namespace ma_loam
