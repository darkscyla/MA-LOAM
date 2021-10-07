// --- Internal Includes ---
#include <ma_loam/cluster_icp.h>

// --- PCL Includes ---
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

// --- CGAL Includes ---
#include <CGAL/IO/STL_reader.h>

// --- Standard Includes ---
#include <fstream>
#include <iostream>
#include <limits>

namespace ma_loam {

// Define type as eigen without copy
#define as_eigen_4f(x) Eigen::Map<Eigen::Vector4f>(x.data())

using point_t = cluster_icp::point_t;
using normal_t = cluster_icp::normal_t;

void
cluster_icp::set_input_cloud(cloud_ptr_const_t _cloud_ptr) {
  cloud_ = std::move(_cloud_ptr);
}

cluster_icp::cloud_normal_ptr_t
cluster_icp::estimate_normals(const size_t _neighbours) const {
  // Create a kd tree for neighbours search
  using search_t = pcl::search::KdTree<point_t>;
  search_t::Ptr kd_tree(new search_t());

  // Setup the normal estimator
  pcl::NormalEstimationOMP<point_t, normal_t> n_est;
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
}

void
kmeans_optimizer::compute_distance_matrix() {
  const auto size = data_.size();

  // Nothing to do in case of empty data
  if (size == 0) {
    return;
  }

  distance_matrix_ = d_matrix_t(size, size);

  for (size_t row = 0, size_m1 = size - 1; row < size_m1; ++row) {
    for (size_t col = row + 1; col < size; ++col) {
      distance_matrix_(row, col) =
          (as_eigen_4f(data_[row]) - as_eigen_4f(data_[col])).norm();
      distance_matrix_(col, row) = distance_matrix_(row, col); ///> Symmetry
    }
  }
}

float
kmeans_optimizer::compute_silhouette_coefficient() const {
  // There is no separation coefficient for less than 2 clusters
  if (clusters_to_points_.size() < 2) {
    return 0;
  }

  float silhouette = 0;

  // For each point, we calculate the cohesion (a) and the separation (b) to
  // compute the silhouette coefficient given by sil = (b - a) / max(a, b)
  for (size_t curr_idx = 0, size = data_.size(); curr_idx < size; ++curr_idx) {
    // Cohesion part is where we calculate the average distance of the current
    // point to other points that are in the same cluster whereas separation is
    // the average distance to the closest cluster. We shall represent cluster
    // related parametes with c_ tag
    const auto curr_c_id = points_to_clusters_[curr_idx];
    float coh = 0, sep = std::numeric_limits<float>::infinity();

    for (size_t c_id = 0, c_size = clusters_to_points_.size(); c_id < c_size;
         ++c_id) {
      // If current cluster is emtpy, skip it (pcl internally has them, why???)
      if (clusters_to_points_[c_id].empty()) {
        continue;
      }

      // If same cluster as current point, compute cohesion
      if (c_id == curr_c_id) {
        for (const auto idx : clusters_to_points_[c_id]) {
          coh += distance_matrix_(curr_idx, idx);
        }
        coh = clusters_to_points_[c_id].size() > 1 ///< Average the cohesion
                  ? coh / (clusters_to_points_[c_id].size() - 1)
                  : 0;
      }
      // Otherwise, compute separation of current point to nearest cluster
      else {
        float curr_sep = 0;
        for (const auto idx : clusters_to_points_[c_id]) {
          curr_sep += distance_matrix_(curr_idx, idx);
        }
        curr_sep /= clusters_to_points_[c_id].size();

        // Update sep if this cluster was closer
        sep = std::min(curr_sep, sep);
      }
    }
    // Now add the contribution to the silhouette if not nan
    const auto curr_silhouette = (sep - coh) / std::max(coh, sep);
    if (!std::isnan(curr_silhouette)) {
      silhouette += (sep - coh) / std::max(coh, sep);
    }
  }

  return silhouette / data_.size();
}

pcl::Kmeans::ClustersToPoints
kmeans_optimizer::optimize(size_t _max_clusters, size_t _min_cluster_elements,
                           const float _eps) {
  // Make sure that min elements is valid
  _min_cluster_elements = std::max(_min_cluster_elements, 1ul);

  // In case of insufficient data, we quit early
  if (data_.size() < _min_cluster_elements) {
    return {};
  }

  // Determine the maximum possible clusters that can actually be computed
  _max_clusters = std::min<size_t>(_max_clusters,
                                   (data_.size() / _min_cluster_elements) + 1);

  // Compute the distance matrix as it will be used for k means optimization
  compute_distance_matrix();

  auto best_sil = -std::numeric_limits<float>::infinity();
  ClustersToPoints best_cluster;
  Centroids best_centroids;

  // Clustering only makes sense if there is more than 1 class in the data
  for (size_t clusters = 2; clusters <= _max_clusters; ++clusters) {
    // Set the cluster size
    Kmeans::setClusterSize(clusters);

    // Clear the internal data and run the k-means algorithm
    clear();
    kMeans();

    // Check if current fit is better
    const auto curr_sil = compute_silhouette_coefficient();
    if (curr_sil > best_sil) {
      // Copy only those clusters that are associated with at least
      // _min_cluster_elements
      best_cluster.clear();
      best_centroids.clear();

      for (size_t curr_c_idx = 0; curr_c_idx < clusters; ++curr_c_idx) {
        const auto &curr_c = clusters_to_points_[curr_c_idx];

        if (curr_c.size() >= _min_cluster_elements || clusters == 2) {
          best_cluster.push_back(curr_c);
          best_centroids.push_back(centroids_[curr_c_idx]);
        }
      }
      best_sil = curr_sil;
    }
  }
  // If there are only 2 clusters, we check if they can be merged
  if (best_cluster.size() == 2) {
    if (best_cluster[0].size() + best_cluster[1].size() <
        _min_cluster_elements) {
      // Do not merge if both the clusters sum is still less than min elements
      best_cluster.clear();
    } else if ((as_eigen_4f(best_centroids[0]) - as_eigen_4f(best_centroids[1]))
                   .squaredNorm() <= _eps * _eps) {
      // Collapse the 2 clusters into a single one
      best_cluster[0].insert(best_cluster[1].begin(), best_cluster[1].end());
      best_cluster.pop_back();
    } else {
      // Finally check if individual clusters have necessary elements
      best_cluster.erase(
          std::remove_if(best_cluster.begin(), best_cluster.end(),
                         [&_min_cluster_elements](const auto &_pts) {
                           return _pts.size() < _min_cluster_elements;
                         }),
          best_cluster.end());
    }
  }

  return best_cluster;
}

aabb_tree_mesh::aabb_tree_mesh(const std::string &_stl_file_path)
    : triangles_(load_stl_file(_stl_file_path)),
      tree_(triangles_.begin(), triangles_.end()),
      dist_threshold_sqr_(std::numeric_limits<scalar_t>::infinity()) {
  // Create the internal KD tree to accelerate nearest point queries
  tree_.accelerate_distance_queries();
}

aabb_tree_mesh::triangles_t
aabb_tree_mesh::load_stl_file(const std::string &_stl_file_path) {
  std::vector<point_3_t> vertices;
  std::vector<std::array<size_t, 3>> face_indices;

  // Loads in the file. Use RAII paradigm
  {
    std::ifstream stl_file(_stl_file_path, std::ifstream::in);
    const auto success = CGAL::read_STL(stl_file, vertices, face_indices);

    if (!success) {
      std::cerr << "ERROR: Unable to open file at " << _stl_file_path << "\n";
      return {};
    }
  }

  // Create AABB tree compatible triangles
  triangles_t triangles;
  triangles.reserve(face_indices.size());

  for (const auto &indices : face_indices) {
    triangles.push_back(
        {vertices[indices[0]], vertices[indices[1]], vertices[indices[2]]});
  }

  return triangles;
}

} // namespace ma_loam
