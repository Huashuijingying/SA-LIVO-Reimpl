/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.

Modified for ROS 2; immediate source: integralrobotics/FAST-LIVO2 commit
d4ad051 (2025-04-09).
Further modified for this SA-LIVO reproduction by Huashuijingying, 2026-08.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include "saif.h"
#include "adaptive_voxel_map_v2.h"
#include <Eigen/Dense>
#include <fstream>
#include <functional>
#include <math.h>
#include <memory>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;

typedef struct VoxelMapConfig
{
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  std::vector<int64_t> layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double near_range_extra_m_ = 0.0;  // extra radial sigma at range→0 (diagnostic)
  double near_range_scale_m_ = 1.0;  // exponential decay scale for near-range boost
  double sigma_num_;
  bool is_pub_plane_map_;

  // config of local map sliding
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;

  // ---- SA-LIVO (Sect. V): scale-invariant planarity + saturation ----------
  double planarity_ratio_th_;  // κth: dimensionless planarity gate (0.05)
  double min_eigen_abs_;       // εmin: absolute eigenvalue guard (0.01)
  int freeze_min_points_;      // min points before planarity freeze applies
  double chi2_thres_;          // τL: LiDAR χ² innovation gate (9 ≈ 3σ)
  bool multi_scale_en_;        // enable Scale-1 shell expansion
  double scale1_radius_;       // rs: base radius of the shell search (2 m)
  int max_shell_;              // ℓmax: clamp(⌈rs/vs⌉, 1, 6)
  double normal_consistency_cos_;  // cos⁻¹(0.95) ≈ 18° normal filter
  double shell_aggregate_cos_ = 0.8660254;  // 30° same-surface aggregation filter
                                            // (diagnostic: ≤0 disables; paper has none)
  double sa_scale1_range_gate_ = 0.0;       // >0: require the query point within
                                            // gate*radius of the Scale-1 fit centroid
  bool coarse_map_fallback_en_ = false;     // dual-resolution: fall back to a
                                            // 2x coarse octree for non-planar voxels
  bool info_plane_decorr_en_ = false;       // scale ΛL by distinct-planes/
                                            // matched-points ratio (per-frame)
  bool scale1_region_grow_en_ = false;      // region-growing Scale-1 (stays on
                                            // one surface, avoids mixing)
  bool adaptive_support_en_ = true; // arXiv v2 flat-grid query-time support
  int adaptive_n_pca_ = 25;         // v2 fixed minimum support
  bool map_cov_association_en_ = false; // Eq.11 covariance in association

  // SAIF (Sect. VII)
  double saif_sigma_min_ = 1.0;      // σmin information-amplitude threshold
  bool saif_sigma_min_adaptive_ = false; // σmin ← max(σmin, sqrt(λ_p10))
  double saif_sigma_min_lambda_max_ = 0.0; // >0: σmin ← max(σmin, scale·√λmax)
  bool saif_relative_prior_gate_en_ = false; // gate = min(√(λ_meas/λ_prior),1)
  double epsilon_min_adaptive_ = 0.0;    // >0: εmin_voxel = εmin·clamp(n/50,0.5,10)
  double eps_pose_inflate_ = 0.01;   // εP post-convergence covariance inflation
  double conv_phi_deg_ = 0.01;       // ϵϕ convergence threshold (deg)
  double conv_p_cm_ = 0.015;         // ϵp convergence threshold (cm)
  bool frozen_rebase_en_ = true;     // rebase frozen bV by −ΛV·δ0 each iteration
  bool lidar_jacobian_recompute_en_ = false; // recompute LiDAR h at every iterate
  bool baseline_lio_association_en_ = false; // use original FAST-LIVO2 octree association
  bool baseline_octree_map_en_ = false;      // exact FAST-LIVO2 octree map layer:
                                            //   λmin-only is_plane_ test + no planar freeze
  bool reassoc_each_iter_en_ = false;        // rebuild LiDAR correspondences every
                                            //   InEKF iteration (original StateEstimation behavior)
  bool force_lio_only_en_ = false;           // zero the visual info in the joint
                                            //   update (keeps img-aligned cadence)
  bool sigma2_full_state_cov_en_ = false;    // Eq.15 uses full Eq.12 world covariance
  double sigma2_state_cov_scale_ = 0.0;      // 0: sensor-only; >0: add
                                             // scale*(state-cov term) to point noise
  double plane_var_scale_ = 1.0;             // multiply the plane-covariance term in
                                             // the residual variance (diagnostic)
  bool plane_cov_paper_en_ = false;          // Eq.12 paper model: Σplane=(λmin/Neff)·I6
                                             // (fit-thickness driven, instead of the
                                             // propagated per-point covariance)
  double map_insert_min_vel_ = 0.0;          // >0: skip voxel-map insertion when the
                                             // estimated speed is below this threshold
                                             // (diagnostic: prevents stationary duplicate
                                             // scans from smearing planes under a drifting pose)
  double prior_scale_when_statecov_ = 0.0;   // >0: scale the solve prior P^-1 by
                                             // (1 - sigma2_state_cov_scale) to avoid
                                             // double-counting the state covariance
  bool adaptive_voxel_en_ = false;           // Eq.7 range-adaptive voxel sizing
  double adaptive_voxel_alpha_ = 0.5;        // alpha_r in Eq.7
  bool state_est_original_cov_en_ = false;   // original FAST-LIVO2 world covariance in StateEstimation
  double epsilon0_ = 1e-3;                   // ε0 residual-variance floor (Eq. 13)
} VoxelMapConfig;

typedef struct PointToPlane
{
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
} PointToPlane;

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

struct DS_POINT
{
  float xyz[3];
  float intensity;
  int count = 0;
};

void calcBodyCov(const Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{

public:
  VoxelOctoTree() = default;
  std::vector<pointWithVar> temp_points_;
  VoxelPlane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  VoxelOctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_init_num_;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;

  // ---- SA-LIVO (Sect. V-C/D): sufficient statistics + saturation ----------
  Eigen::Vector3d s1_;            // Σ x_i
  Eigen::Matrix3d S2_;            // Σ x_i x_iᵀ
  Eigen::Matrix3d covariance_sum_;  // Σ Σw,i (for Eq. 13/14 plane covariance)
  std::vector<Eigen::Vector3d> stats_points_obs_;
  std::vector<Eigen::Matrix3d> stats_cov_obs_;
  int n_stats_;                   // N
  bool saturated_;                // cap reached → frozen (long-term anchor)
  bool planar_frozen_;            // planarity dual test passed → frozen
  double planarity_ratio_;        // κ = λmin / tr(Σpts) of the last fit
  double min_eigen_value_fit_;    // λmin of the last fit
  double planarity_ratio_th_;     // κth: dimensionless planarity gate
  double min_eigen_abs_;          // εmin: absolute eigenvalue guard
  int freeze_min_points_;         // min points before planarity freeze applies
  double voxel_size_ = 0.5;       // physical side length of this voxel (Eq.7)
  double epsilon_min_adaptive_ = 0.0; // >0: εmin_voxel = εmin·clamp(n/50,0.5,10)
  bool baseline_map_en_ = false;  // exact original octree-map behavior (λmin-only, no freeze)

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold,
                double planarity_ratio_th = 0.05, double min_eigen_abs = 0.01, int freeze_min_points = 15,
                double epsilon_min_adaptive = 0.0, bool baseline_map_en = false)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold), epsilon_min_adaptive_(epsilon_min_adaptive), baseline_map_en_(baseline_map_en)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    saturated_ = false;
    planar_frozen_ = false;
    planarity_ratio_ = 1.0 / 3.0;
    min_eigen_value_fit_ = 1.0;
    planarity_ratio_th_ = planarity_ratio_th;
    min_eigen_abs_ = min_eigen_abs;
    freeze_min_points_ = freeze_min_points;
    s1_.setZero();
    S2_.setZero();
    covariance_sum_.setZero();
    stats_points_obs_.clear();
    stats_cov_obs_.clear();
    n_stats_ = 0;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }

  // Incremental sufficient-statistics update (O(1), retained after freeze)
  inline void updateStats(const Eigen::Vector3d &pw)
  {
    if (saturated_ || planar_frozen_) return;
    s1_ += pw;
    S2_ += pw * pw.transpose();
    n_stats_++;
  }
  inline void updateStats(const Eigen::Vector3d &pw, const Eigen::Matrix3d &cov)
  {
    if (saturated_ || planar_frozen_) return;
    s1_ += pw;
    S2_ += pw * pw.transpose();
    covariance_sum_ += cov;
    stats_points_obs_.push_back(pw);
    stats_cov_obs_.push_back(cov);
    n_stats_++;
  }
  // Sample covariance recovered from the sufficient statistics (Eq. 10)
  inline bool statsCovariance(Eigen::Vector3d &mean, Eigen::Matrix3d &cov) const
  {
    if (n_stats_ < 3) return false;
    mean = s1_ / n_stats_;
    cov = S2_ / n_stats_ - mean * mean.transpose();
    return true;
  }

  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
  void init_octo_tree();
  void cut_octo_tree();
  void UpdateOctoTree(const pointWithVar &pv);

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
  VoxelOctoTree *Insert(const pointWithVar &pv);
};

void loadVoxelConfig(rclcpp::Node::SharedPtr &node, VoxelMapConfig &voxel_config);

class VoxelMapManager
{
public:
  VoxelMapManager() = default;
  VoxelMapConfig config_setting_;
  int current_frame_id_ = 0;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_map_pub_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_coarse_;
  std::unique_ptr<AdaptiveVoxelGrid> adaptive_map_;
  // Collect all currently-valid map planes (walk the voxel hash map).
  void collectMapPlanes(std::vector<VoxelPlane> &plane_list);

  // Eq.7 range-adaptive voxel side length for a point: v(r)=min(αr·2^⌊log2 r⌋, vs).
  // r is the body-frame range; falls back to the uniform max voxel size when
  // the adaptive scheme is disabled.
  inline double voxelSizeForPoint(const V3D &point_b) const
  {
    if (!config_setting_.adaptive_voxel_en_) return config_setting_.max_voxel_size_;
    const double r = point_b.norm();
    const double r_safe = std::max(r, 0.2);
    const double level = std::floor(std::log2(r_safe));
    const double v = config_setting_.adaptive_voxel_alpha_ * std::pow(2.0, level);
    return std::min(v, config_setting_.max_voxel_size_);
  }

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;

  M3D extR_;
  V3D extT_;
  float build_residual_time, ekf_time;
  float ave_build_residual_time = 0.0;
  float ave_ekf_time = 0.0;
  int scan_count = 0;
  StatesGroup state_;
  // True when the last joint update had zero usable measurements (0 LiDAR
  // survivors and no active visual), i.e. total information loss.
  bool degenerate_update_ = false;
  V3D position_last_;

  V3D last_slide_position = {0,0,0};

  geometry_msgs::msg::Quaternion geoQuat_;

  int feats_down_size_;
  int effct_feat_num_;
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;

  FusedInfo6 fused_last_;  // last SAIF output (debug/logging)

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    AdaptiveVoxelConfig adaptive_config;
    adaptive_config.voxel_size = config_setting_.max_voxel_size_;
    adaptive_config.support_radius_m = config_setting_.scale1_radius_;
    adaptive_config.kappa_threshold = config_setting_.planarity_ratio_th_;
    adaptive_config.association_chi2 = config_setting_.chi2_thres_;
    adaptive_config.epsilon_abs = config_setting_.min_eigen_abs_;
    adaptive_config.n_pca = static_cast<std::uint32_t>(config_setting_.adaptive_n_pca_);
    adaptive_config.n_cap = static_cast<std::uint32_t>(config_setting_.max_points_num_);
    adaptive_config.use_map_covariance = config_setting_.map_cov_association_en_;
    adaptive_map_ = std::make_unique<AdaptiveVoxelGrid>(adaptive_config);
  };

  void StateEstimation(StatesGroup &state_propagat);

  // ---- SA-LIVO: unified joint InEKF update (Algorithm 2) -------------------
  // vis_info: pre-computed visual information form (ΛV, bV) frozen before the
  //           loop; pass nullptr for a LIO-only update (qΛV = 0).
  // q:        scene-level VIO quality factor ∈ [0, 1].
  void JointStateEstimation(StatesGroup &state_propagat, const InfoForm6 *vis_info, double q);

  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  void BuildVoxelMap();
  void updateCoarseMap(const std::vector<pointWithVar> &input_points);
  V3F RGBFromVoxel(const V3D &input_point);

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);
  // First-scan map seeding: populate body covariances / world points at the
  // given state and insert them into the map WITHOUT a state update.
  void seedMap(const StatesGroup &state);

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  // ---- SA-LIVO (Sect. V-F/G): cached correspondence + multi-scale search ----
  // One matched plane cached per LiDAR point after the first InEKF iteration
  struct CachedPlane
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D normal;
    V3D center;
    double d = 0.0;
    Eigen::Matrix<double, 6, 6> plane_var;  // Σplane
    Eigen::Matrix<double, 6, 1> h = Eigen::Matrix<double, 6, 1>::Zero();
    double sigma2 = 0.0;
    bool valid = false;
  };
  std::vector<CachedPlane> plane_cache_;

  // Saturation-priority multi-candidate plane selection (Sect. V-G):
  // Scale-0 single voxel (one-voxel shift if empty), Scale-1 concentric
  // Chebyshev shells with O(1) multi-scale PCA from sufficient statistics.
  // Outputs the winning plane into ptpl (or leaves is_sucess = false).
  void build_single_residual_sa(pointWithVar &pv, bool &is_sucess, PointToPlane &single_ptpl);
  void build_single_residual_v2(pointWithVar &pv, bool &is_success, PointToPlane &single_ptpl);

  void pubVoxelMap();

  void mapSliding();
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );

private:
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::msg::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::msg::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_
