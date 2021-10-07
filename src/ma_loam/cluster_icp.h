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

// --- Ceres Includes ---
#include <ceres/ceres.h>

// --- Eigen Includes ---
#include <Eigen/Core>

// --- Standard Includes ---
#include <vector>

namespace ma_loam {

#define as_T(x) static_cast<T>(x)

class cluster_icp {
public:
  // Type defs
  using point_t = pcl::PointXYZ;

  using cloud_t = pcl::PointCloud<point_t>;
  using normal_t = pcl::Normal;
  using cloud_normal_t = pcl::PointCloud<normal_t>;

  using cloud_ptr_t = cloud_t::Ptr;
  using cloud_ptr_const_t = cloud_t::ConstPtr;
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
  set_input_cloud(cloud_ptr_const_t _cloud_ptr);

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
  cloud_ptr_const_t cloud_;
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
  using coord_t = CGAL::Simple_cartesian<double>; ///> Double faster than float

  using scalar_t = coord_t::FT;
  using point_3_t = coord_t::Point_3;
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

  /**
   * @brief Set the distance threshold above which closest point query returns
   * false. The closest point is still updated in place
   *
   * @param _threshold New value of threshold to set
   */
  void
  set_distance_threshold(const scalar_t _threshold) {
    dist_threshold_sqr_ = _threshold * _threshold;
  }

  /**
   * @brief Checks if the underlying mesh is empty
   * 
   * @return true If the mesh is empty
   * @return false If the mesh contains data
   */
  bool
  empty() const {
    return triangles_.empty();
  }

  /**
   * @brief
   *
   * @param[in] _query_pt Point for which closest point on mesh is required
   * @param[out] _closest_pt The closest point on mesh from query point
   * @return true If the distance between query and closest point is less than
   * or equal to the threshold
   * @return false If distance is greater than threshold
   */
  bool
  closest_point(const point_3_t &_query_pt, point_3_t &_closest_pt) const {
    _closest_pt = tree_.closest_point(_query_pt);
    return (_closest_pt - _query_pt).squared_length() <= dist_threshold_sqr_
               ? true
               : false;
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

  triangles_t triangles_;       ///> Underlying triangles representing the mesh
  tree_t tree_;                 ///> AABB tree with embedded kd tree
  scalar_t dist_threshold_sqr_; ///> Rejection distance
};

namespace conversions {

template <typename T>
aabb_tree_mesh::point_3_t
to_point_3(const T &_x, const T &_y, const T &_z) {
  return {_x, _y, _z};
}

// In case of jets, we only care about the real part
template <typename T, int N>
aabb_tree_mesh::point_3_t
to_point_3(const ceres::Jet<T, N> &_x, const ceres::Jet<T, N> &_y,
           const ceres::Jet<T, N> &_z) {
  return {_x.a, _y.a, _z.a};
}

} // namespace conversions

class point_to_mesh_cost {
public:
  using point_t = cluster_icp::point_t;

  /**
   * @brief Construct the point to mesh distance cost function. The underlying
   * AABB tree and point must remain valid till the end of the optimization as
   * we just store the references to both
   *
   * @param _mesh Reference to the AABB tree for fast point to point on mesh
   * computation
   * @param _point Reference to the un-aligned point to match to the mesh
   */
  point_to_mesh_cost(const aabb_tree_mesh &_mesh, const point_t &_point)
      : mesh_(_mesh), point_(_point) {}

  template <typename T>
  bool
  operator()(const T *const _quaternion, const T *const _translation,
             T *_residual) const {
    // Cast the underlying point to type T (which is either double if just
    // function evaluation is required or it is ceres::Jet if derivative is also
    // required)
    Eigen::Matrix<T, 3, 1> curr_pt{as_T(point_.x), as_T(point_.y),
                                   as_T(point_.z)};

    // Map the quaternion and translation to Eigen types
    const Eigen::Quaternion<T> quaternion =
        Eigen::Map<const Eigen::Quaternion<T>>(_quaternion);
    const Eigen::Matrix<T, 3, 1> translation =
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(_translation);

    // Apply the rotation and translation
    curr_pt = quaternion * curr_pt + translation;

    // Now, we find the closet point on the mesh. If within threshold, we
    // consider it active otherwise we set 0 cost
    aabb_tree_mesh::point_3_t mesh_pt;
    if (mesh_.closest_point(
            conversions::to_point_3(curr_pt.x(), curr_pt.y(), curr_pt.z()),
            mesh_pt)) {
      _residual[0] = curr_pt.x() - mesh_pt.x();
      _residual[1] = curr_pt.y() - mesh_pt.y();
      _residual[2] = curr_pt.z() - mesh_pt.z();
    } else {
      _residual[0] = as_T(0);
      _residual[1] = as_T(0);
      _residual[2] = as_T(0);
    }

    return true;
  }

  /**
   * @brief Factory method to create the appropriate cost function
   *
   * @param _mesh Reference to the AABB tree for fast point to point on mesh
   * computation
   * @param _point Reference to the un-aligned point to match to the mesh
   * @return ceres::CostFunction* Pointer to ceres cost function. Ceres will
   * take ownership of the cost function so there is no need to free it manually
   */
  static ceres::CostFunction *
  create(const aabb_tree_mesh &_mesh, const point_t &_point) {
    return new ceres::AutoDiffCostFunction<point_to_mesh_cost, 3, 4, 3>(
        new point_to_mesh_cost(_mesh, _point));
  }

private:
  const aabb_tree_mesh &mesh_; ///> Store a referece to the aabb tree
  const point_t &point_;       ///> Store a reference to the point cloud point
};

} // namespace ma_loam
