// --- PCL Includes ---
#include <pcl/ml/kmeans.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// --- CGAL Includes ---
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Simple_cartesian.h>

// --- Standard Includes ---
#include <vector>

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

class kmeans_optimizer : public pcl::Kmeans {
public:
  // Inherit all base class constructors
  using Kmeans::Kmeans;

  /**
   * @brief Optimizes the number of clusters that maximizes the cohesion and
   * separation using the silhouette method. If only 2 clusters are found (which
   * is the minimun), they are merged if they are close enough to eachother
   * based on the given epsilon value
   *
   * @param _max_clusters Maximum number of clusters to try for optimal
   * silhouette
   * @param _min_cluster_elements Minimum number of elements a cluster must have
   * in order to be considered as a valid cluster
   * @param _eps Distance below which clusters are merged i.e. they are close
   * together
   * @return ClustersToPoints The optimized clusters created using k-means and
   * silhouette method. Each cluster is associated with a list of point ids
   */
  ClustersToPoints
  optimize(size_t _max_clusters, size_t _min_cluster_elements, float _eps);

private:
  /**
   * @brief Computes the distance matrix for all the current points in the data
   */
  void
  compute_distance_matrix();

  /**
   * @brief Computes the silhouette coefficient from the internal k means
   * clustering
   *
   * @return float The value of silhouette coefficient
   */
  float
  compute_silhouette_coefficient() const;

  /**
   * @brief Clears the internal state so we can run the k-means again
   *
   */
  void
  clear() {
    clusters_to_points_.clear();
    points_to_clusters_.clear();
    centroids_.clear();
  }

  // Disallow cluster size to be set externally
  using Kmeans::setClusterSize;
  using d_matrix_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

  // Distance matrix
  d_matrix_t distance_matrix_;
};

class aabb_tree_mesh {
public:
  // Type defs for CGAL ( lots of them :) )
  using coord_t = CGAL::Simple_cartesian<double>;

  using scalar_t = coord_t::FT;
  using point_t = coord_t::Point_3;
  using triangle_t = coord_t::Triangle_3;
  using triangles_t = std::vector<triangle_t>;
  using iterator_t = std::vector<triangle_t>::iterator;
  using primitive_t = CGAL::AABB_triangle_primitive<coord_t, iterator_t>;
  using aabb_triangle_traits_t = CGAL::AABB_traits<coord_t, primitive_t>;
  using tree_t = CGAL::AABB_tree<aabb_triangle_traits_t>;

  /**
   * @brief Loads in a stl file and creates an axis aligned bounding box (AABB)
   * tree for fast closest point to a triangular mesh queries. To furthur
   * accelerate this, we construct a KD-tree under the hood. It is assumed that
   * the stl file contains just a plain triangular mesh
   *
   * @param _stl_file_path Path of the stl file
   */
  aabb_tree_mesh(const std::string &_stl_file_path);

  const tree_t &
  tree() const {
    return tree_;
  }

private:
  /**
   * @brief Loads in a stl file and creates a vector of triangles representing
   * the underlying mesh
   *
   * @param _stl_file_path Path of the stl fle
   * @return triangles_t A vector of triangles representing the
   * underlying geometry of the stl file
   */
  static triangles_t
  load_stl_file(const std::string &_stl_file_path);

  triangles_t triangles_;
  tree_t tree_;
};

} // namespace ma_loam
