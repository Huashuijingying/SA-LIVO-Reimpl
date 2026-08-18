/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "voxel_map.h"
#include "saif.h"
#include <cstdint>
#include <string>
#include <unordered_set>
using namespace Eigen;
// Eq.12 paper model: Σplane = (λmin/Neff)·I6, Neff = max(1, count).
// The fit thickness λmin grows when a voxel accumulates returns from a
// drifting trajectory, so the residual variance inflates and SAIF's gate can
// defer those directions to the IMU prior (instead of confidently locking
// onto the smeared map).
static Eigen::Matrix<double, 6, 6> paperPlaneCov(const double lambda_min, const int count)
{
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  const double neff = std::max(1.0, static_cast<double>(count));
  cov.diagonal().setConstant(std::max(lambda_min, 0.0) / neff);
  return cov;
}
void calcBodyCov(const Eigen::Vector3d &pb, const float range_inc, const float degree_inc,
                 Eigen::Matrix3d &cov, const double near_extra_m, const double near_scale_m)
{
  const double range = pb.norm();
  if (range <= 1e-12)
  {
    cov = Eigen::Matrix3d::Identity() *
          std::pow(static_cast<double>(range_inc) + near_extra_m, 2);
    return;
  }
  const Eigen::Vector3d direction = pb / range;
  // Near-range noise model (diagnostic, default off): extra radial sigma that
  // decays exponentially with range, so close points do not dominate plane fits
  // with Hesai XT32 near-range distortion/beam-mixing noise.
  const double extra = near_extra_m * std::exp(-range / std::max(near_scale_m, 1e-3));
  const double radial_variance = std::pow(static_cast<double>(range_inc) + extra, 2);
  const double angular_std = std::sin(DEG2RAD(static_cast<double>(degree_inc)));
  const double transverse_variance = range * range * angular_std * angular_std;
  cov = radial_variance * direction * direction.transpose() +
        transverse_variance * (Eigen::Matrix3d::Identity() - direction * direction.transpose());
  cov = 0.5 * (cov + cov.transpose());
}

void loadVoxelConfig(rclcpp::Node::SharedPtr &node, VoxelMapConfig &voxel_config)
{
  auto try_declare = [node]<typename ParameterT>(const std::string & name,
    const ParameterT & default_value)
  {
    if (!node->has_parameter(name))
    {
      return node->declare_parameter<ParameterT>(name, default_value);
    }
    else
    {
      return node->get_parameter(name).get_value<ParameterT>();
    }
  };

  // declare parameter
  try_declare.template operator()<bool>("publish.pub_plane_en", false);
  try_declare.template operator()<int>("lio.max_layer", 1);
  try_declare.template operator()<double>("lio.voxel_size", 0.5);
  try_declare.template operator()<double>("lio.min_eigen_value", 0.01);
  try_declare.template operator()<double>("lio.sigma_num", 3);
  try_declare.template operator()<double>("lio.beam_err", 0.02);
  try_declare.template operator()<double>("lio.dept_err", 0.05);
  try_declare.template operator()<double>("lio.near_range_extra_m", 0.0);
  try_declare.template operator()<double>("lio.near_range_scale_m", 1.0);

  // SA-LIVO scale-invariant planarity + saturation (Sect. V)
  try_declare.template operator()<double>("lio.planarity_ratio_th", 0.05);
  try_declare.template operator()<double>("lio.min_eigen_abs", 0.01);
  try_declare.template operator()<int>("lio.freeze_min_points", 15);
  try_declare.template operator()<double>("lio.chi2_thres", 9.0);
  try_declare.template operator()<bool>("lio.multi_scale_en", true);
  try_declare.template operator()<double>("lio.scale1_radius", 2.0);
  try_declare.template operator()<int>("lio.max_shell", 6);
  try_declare.template operator()<bool>("lio.adaptive_support_en", false);
  try_declare.template operator()<int>("lio.n_pca", 25);
  try_declare.template operator()<double>("lio.shell_aggregate_cos", 0.8660254);
  try_declare.template operator()<double>("lio.sa_scale1_range_gate", 0.0);
  try_declare.template operator()<bool>("lio.coarse_map_fallback_en", false);
  try_declare.template operator()<bool>("lio.info_plane_decorr_en", false);
  try_declare.template operator()<bool>("lio.scale1_region_grow_en", false);
  try_declare.template operator()<bool>("lio.map_cov_association_en", false);

  // SAIF (Sect. VII)
  try_declare.template operator()<double>("saif.sigma_min", 1.0);
  try_declare.template operator()<bool>("saif.sigma_min_adaptive_en", false);
  try_declare.template operator()<double>("saif.sigma_min_lambda_max_scale", 0.0);
  try_declare.template operator()<bool>("saif.relative_prior_gate_en", false);
  try_declare.template operator()<double>("saif.eps_pose_inflate", 0.01);
  try_declare.template operator()<double>("saif.conv_phi_deg", 0.01);
  try_declare.template operator()<double>("saif.conv_p_cm", 0.015);
  try_declare.template operator()<bool>("saif.frozen_rebase_en", true);
  try_declare.template operator()<bool>("saif.lidar_jacobian_recompute_en", false);
  try_declare.template operator()<bool>("saif.baseline_lio_association_en", false);
  try_declare.template operator()<bool>("lio.sigma2_full_state_cov_en", false);
  try_declare.template operator()<bool>("lio.adaptive_voxel_en", false);
  try_declare.template operator()<double>("lio.adaptive_voxel_alpha", 0.5);
  try_declare.template operator()<bool>("lio.state_est_original_cov_en", false);
  try_declare.template operator()<double>("lio.epsilon0", 1e-3);
  try_declare.template operator()<double>("lio.epsilon_min_adaptive", 0.0);
  try_declare.template operator()<bool>("lio.baseline_octree_map_en", false);
  try_declare.template operator()<bool>("lio.reassoc_each_iter_en", false);
  try_declare.template operator()<bool>("saif.force_lio_only_en", false);
  try_declare.template operator()<double>("lio.sigma2_state_cov_scale", 0.0);
  try_declare.template operator()<double>("lio.plane_var_scale", 1.0);
  try_declare.template operator()<bool>("lio.plane_cov_paper_en", false);
  try_declare.template operator()<double>("lio.map_insert_min_vel", 0.0);
  try_declare.template operator()<double>("lio.prior_scale_when_statecov", 0.0);

  // Declaration of parameter of type std::vector<int> won't build, https://github.com/ros2/rclcpp/issues/1585  
  try_declare.template operator()<vector<int64_t>>("lio.layer_init_num", std::vector<int64_t>{5,5,5,5,5}); 
  try_declare.template operator()<int>("lio.max_points_num", 50);
  try_declare.template operator()<int>("lio.max_iterations", 5);
  try_declare.template operator()<bool>("local_map.map_sliding_en", false);
  try_declare.template operator()<int>("local_map.half_map_size", 100);
  try_declare.template operator()<double>("local_map.sliding_thresh", 8.0);

  // get parameter
  node->get_parameter("publish.pub_plane_en", voxel_config.is_pub_plane_map_);
  node->get_parameter("lio.max_layer", voxel_config.max_layer_);
  node->get_parameter("lio.voxel_size", voxel_config.max_voxel_size_);
  node->get_parameter("lio.min_eigen_value", voxel_config.planner_threshold_);
  node->get_parameter("lio.sigma_num", voxel_config.sigma_num_);
  node->get_parameter("lio.beam_err", voxel_config.beam_err_);
  node->get_parameter("lio.dept_err", voxel_config.dept_err_);
  node->get_parameter("lio.near_range_extra_m", voxel_config.near_range_extra_m_);
  node->get_parameter("lio.near_range_scale_m", voxel_config.near_range_scale_m_);
  node->get_parameter("lio.layer_init_num", voxel_config.layer_init_num_);
  node->get_parameter("lio.max_points_num", voxel_config.max_points_num_);
  node->get_parameter("lio.max_iterations", voxel_config.max_iterations_);
  node->get_parameter("local_map.map_sliding_en", voxel_config.map_sliding_en);
  node->get_parameter("local_map.half_map_size", voxel_config.half_map_size);
  node->get_parameter("local_map.sliding_thresh", voxel_config.sliding_thresh);

  // SA-LIVO params
  node->get_parameter("lio.planarity_ratio_th", voxel_config.planarity_ratio_th_);
  node->get_parameter("lio.min_eigen_abs", voxel_config.min_eigen_abs_);
  node->get_parameter("lio.freeze_min_points", voxel_config.freeze_min_points_);
  node->get_parameter("lio.chi2_thres", voxel_config.chi2_thres_);
  node->get_parameter("lio.multi_scale_en", voxel_config.multi_scale_en_);
  node->get_parameter("lio.scale1_radius", voxel_config.scale1_radius_);
  node->get_parameter("lio.max_shell", voxel_config.max_shell_);
  node->get_parameter("lio.adaptive_support_en", voxel_config.adaptive_support_en_);
  node->get_parameter("lio.n_pca", voxel_config.adaptive_n_pca_);
  node->get_parameter("lio.shell_aggregate_cos", voxel_config.shell_aggregate_cos_);
  node->get_parameter("lio.sa_scale1_range_gate", voxel_config.sa_scale1_range_gate_);
  node->get_parameter("lio.coarse_map_fallback_en", voxel_config.coarse_map_fallback_en_);
  node->get_parameter("lio.info_plane_decorr_en", voxel_config.info_plane_decorr_en_);
  node->get_parameter("lio.scale1_region_grow_en", voxel_config.scale1_region_grow_en_);
  node->get_parameter("lio.map_cov_association_en", voxel_config.map_cov_association_en_);
  voxel_config.normal_consistency_cos_ = 0.95;  // cos⁻¹(0.95) ≈ 18° (Sect. V-G)

  if (voxel_config.max_voxel_size_ <= 0.0 || voxel_config.max_points_num_ <= 0 ||
      voxel_config.adaptive_n_pca_ <= 0 || voxel_config.planarity_ratio_th_ <= 0.0 ||
      voxel_config.planarity_ratio_th_ >= (1.0 / 3.0))
  {
    throw std::invalid_argument("invalid adaptive voxel-map configuration");
  }

  // SAIF (Sect. VII)
  node->get_parameter("saif.sigma_min", voxel_config.saif_sigma_min_);
  node->get_parameter("saif.sigma_min_adaptive_en", voxel_config.saif_sigma_min_adaptive_);
  node->get_parameter("saif.sigma_min_lambda_max_scale", voxel_config.saif_sigma_min_lambda_max_);
  node->get_parameter("saif.relative_prior_gate_en", voxel_config.saif_relative_prior_gate_en_);
  node->get_parameter("saif.eps_pose_inflate", voxel_config.eps_pose_inflate_);
  node->get_parameter("saif.conv_phi_deg", voxel_config.conv_phi_deg_);
  node->get_parameter("saif.conv_p_cm", voxel_config.conv_p_cm_);
  node->get_parameter("saif.frozen_rebase_en", voxel_config.frozen_rebase_en_);
  node->get_parameter("saif.lidar_jacobian_recompute_en", voxel_config.lidar_jacobian_recompute_en_);
  node->get_parameter("saif.baseline_lio_association_en", voxel_config.baseline_lio_association_en_);
  node->get_parameter("lio.sigma2_full_state_cov_en", voxel_config.sigma2_full_state_cov_en_);
  node->get_parameter("lio.adaptive_voxel_en", voxel_config.adaptive_voxel_en_);
  node->get_parameter("lio.adaptive_voxel_alpha", voxel_config.adaptive_voxel_alpha_);
  node->get_parameter("lio.state_est_original_cov_en", voxel_config.state_est_original_cov_en_);
  node->get_parameter("lio.epsilon0", voxel_config.epsilon0_);
  node->get_parameter("lio.epsilon_min_adaptive", voxel_config.epsilon_min_adaptive_);
  node->get_parameter("lio.baseline_octree_map_en", voxel_config.baseline_octree_map_en_);
  node->get_parameter("lio.reassoc_each_iter_en", voxel_config.reassoc_each_iter_en_);
  node->get_parameter("saif.force_lio_only_en", voxel_config.force_lio_only_en_);
  node->get_parameter("lio.sigma2_state_cov_scale", voxel_config.sigma2_state_cov_scale_);
  node->get_parameter("lio.plane_var_scale", voxel_config.plane_var_scale_);
  node->get_parameter("lio.plane_cov_paper_en", voxel_config.plane_cov_paper_en_);
  node->get_parameter("lio.map_insert_min_vel", voxel_config.map_insert_min_vel_);
  node->get_parameter("lio.prior_scale_when_statecov", voxel_config.prior_scale_when_statecov_);
}

void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
  plane->covariance_ = Eigen::Matrix3d::Zero();
  plane->center_ = Eigen::Vector3d::Zero();
  plane->normal_ = Eigen::Vector3d::Zero();
  plane->points_size_ = points.size();
  plane->radius_ = 0;
  for (auto pv : points)
  {
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
    plane->center_ += pv.point_w;
  }
  plane->center_ = plane->center_ / plane->points_size_;
  plane->covariance_ = plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
  Eigen::Matrix3cd evecs = es.eigenvectors();
  Eigen::Vector3cd evals = es.eigenvalues();
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();
  Eigen::Matrix3f::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax;
  Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
  Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
  Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
  Eigen::Matrix3d J_Q;
  J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_;
  // SA-LIVO (Sect. V-C): dimensionless planarity ratio + absolute guard
  //   κ = λmin / tr(Σpts) < κth  AND  λmin < εmin
  const double trace_pts = evalsReal.sum();
  this->planarity_ratio_ = (trace_pts > 1e-12) ? evalsReal(evalsMin) / trace_pts : 1.0 / 3.0;
  this->min_eigen_value_fit_ = evalsReal(evalsMin);
  // Paper semantics (Sect. V-C): scale-invariant ratio AND absolute guard.
  // (A/B found exp11 improves with the original λmin-only test at
  // min_eigen_value=5e-4 → 0.29, but exp14 diverges — scene-dependent.)
  double epsilon_min_used = this->min_eigen_abs_;
  if (this->epsilon_min_adaptive_ < 0.0)
  {
    // Noise-scaled εmin: the plane is accepted only if its λmin is well below
    // the voxel's own noise floor (points' world covariance projected on the
    // fitted normal). Scene-adaptive: sparse/noisy voxels get a looser guard,
    // tight low-noise voxels a stricter one.
    Eigen::VectorXd noise_proj(points.size());
    for (size_t pi = 0; pi < points.size(); ++pi)
    {
      const Eigen::Vector3d n_est = evecs.real().col(evalsMin).normalized();
      noise_proj(pi) = n_est.dot(points[pi].var * n_est);
    }
    std::sort(noise_proj.data(), noise_proj.data() + noise_proj.size());
    const double med_noise = noise_proj(noise_proj.size() / 2);
    epsilon_min_used = std::max(this->min_eigen_abs_, 1.0 * med_noise);
  }
  else if (this->epsilon_min_adaptive_ > 0.0)
  {
    // Scale-adaptive εmin: the absolute guard scales with the voxel's in-plane
    // extent (λmax). Large/noisy voxels get a looser guard; tight voxels a
    // stricter one, matching the paper's "independent of voxel scale" intent
    // only when the guard is expressed relative to the observed scale.
    epsilon_min_used = std::max(this->min_eigen_abs_, this->epsilon_min_adaptive_ * evalsReal(evalsMax));
  }
  // Diagnostic: exact original FAST-LIVO2 plane acceptance is the absolute
  // eigenvalue guard only (λmin < planer_threshold). The paper's dual test
  // (κ AND λmin, Eq. 9) over-rejects in some degenerate scenes (exp11).
  const bool dual_test = this->baseline_map_en_
                             ? (evalsReal(evalsMin) < this->planer_threshold_)
                             : ((this->planarity_ratio_ < this->planarity_ratio_th_) && (evalsReal(evalsMin) < epsilon_min_used));
  if (dual_test)
  {
    for (int i = 0; i < points.size(); i++)
    {
      Eigen::Matrix<double, 6, 3> J;
      Eigen::Matrix3d F;
      for (int m = 0; m < 3; m++)
      {
        if (m != (int)evalsMin)
        {
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].point_w - plane->center_).transpose() / ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
              (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
          F.row(m) = F_m;
        }
        else
        {
          Eigen::Matrix<double, 1, 3> F_m;
          F_m << 0, 0, 0;
          F.row(m) = F_m;
        }
      }
      J.block<3, 3>(0, 0) = evecs.real() * F;
      J.block<3, 3>(3, 0) = J_Q;
      plane->plane_var_ += J * points[i].var * J.transpose();
    }

    plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
    plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
    plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
    plane->min_eigen_value_ = evalsReal(evalsMin);
    plane->mid_eigen_value_ = evalsReal(evalsMid);
    plane->max_eigen_value_ = evalsReal(evalsMax);
    plane->radius_ = sqrt(evalsReal(evalsMax));
    plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) + plane->normal_(2) * plane->center_(2));
    plane->is_plane_ = true;
    plane->is_update_ = true;
    if (!plane->is_init_)
    {
      plane->id_ = voxel_plane_id;
      voxel_plane_id++;
      plane->is_init_ = true;
    }
  }
  else
  {
    plane->is_update_ = true;
    plane->is_plane_ = false;
  }
}

void VoxelOctoTree::init_octo_tree()
{
  if (temp_points_.size() > points_size_threshold_)
  {
    init_plane(temp_points_, plane_ptr_);
    if (plane_ptr_->is_plane_ == true)
    {
      octo_state_ = 0;
      // SA-LIVO: planarity freeze — statistics stop updating once the dual
      // planarity test passes with enough support points (Sect. V-C, Fig. 3)
      if ((n_stats_ >= freeze_min_points_) && !baseline_map_en_) planar_frozen_ = true;
      // new added
      if (temp_points_.size() > max_points_num_)
      {
        update_enable_ = false;
        saturated_ = true;
        std::vector<pointWithVar>().swap(temp_points_);
        new_points_ = 0;
      }
    }
    else
    {
      octo_state_ = 1;
      cut_octo_tree();
    }
    init_octo_ = true;
    new_points_ = 0;
  }
}

void VoxelOctoTree::cut_octo_tree()
{
  if (layer_ >= max_layer_)
  {
    octo_state_ = 0;
    return;
  }
  for (size_t i = 0; i < temp_points_.size(); i++)
  {
    int xyz[3] = {0, 0, 0};
    if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
    if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
    if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] == nullptr)
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_,
                                           planarity_ratio_th_, min_eigen_abs_, freeze_min_points_, epsilon_min_adaptive_,
                                           baseline_map_en_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
    }
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    leaves_[leafnum]->updateStats(temp_points_[i].point_w, temp_points_[i].var);
    leaves_[leafnum]->new_points_++;
  }
  for (uint i = 0; i < 8; i++)
  {
    if (leaves_[i] != nullptr)
    {
      if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_)
      {
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
        if (leaves_[i]->plane_ptr_->is_plane_)
        {
          leaves_[i]->octo_state_ = 0;
          if ((leaves_[i]->n_stats_ >= leaves_[i]->freeze_min_points_) && !leaves_[i]->baseline_map_en_)
            leaves_[i]->planar_frozen_ = true;
          // new added
          if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_)
          {
            leaves_[i]->update_enable_ = false;
            leaves_[i]->saturated_ = true;
            std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
            new_points_ = 0;
          }
        }
        else
        {
          leaves_[i]->octo_state_ = 1;
          leaves_[i]->cut_octo_tree();
        }
        leaves_[i]->init_octo_ = true;
        leaves_[i]->new_points_ = 0;
      }
    }
  }
}

void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv)
{
  if (!baseline_map_en_ && (saturated_ || planar_frozen_)) return;  // SA-LIVO: frozen voxels stop updating

  if (!init_octo_)
  {
    new_points_++;
    temp_points_.push_back(pv);
    updateStats(pv.point_w, pv.var);
    if (temp_points_.size() > points_size_threshold_) { init_octo_tree(); }
  }
  else
  {
    if (plane_ptr_->is_plane_)
    {
      if (update_enable_)
      {
        new_points_++;
        temp_points_.push_back(pv);
        updateStats(pv.point_w, pv.var);
        if (new_points_ > update_size_threshold_)
        {
          init_plane(temp_points_, plane_ptr_);
          new_points_ = 0;
          if ((n_stats_ >= freeze_min_points_) && !baseline_map_en_) planar_frozen_ = true;
        }
        if (temp_points_.size() >= max_points_num_)
        {
          update_enable_ = false;
          saturated_ = true;
          std::vector<pointWithVar>().swap(temp_points_);
          new_points_ = 0;
        }
      }
    }
    else
    {
      if (layer_ < max_layer_)
      {
        int xyz[3] = {0, 0, 0};
        if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
        if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
        if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) { leaves_[leafnum]->UpdateOctoTree(pv); }
        else
        {
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_,
                                               planarity_ratio_th_, min_eigen_abs_, freeze_min_points_, epsilon_min_adaptive_,
                                               baseline_map_en_);
          leaves_[leafnum]->layer_init_num_ = layer_init_num_;
          leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
          leaves_[leafnum]->quater_length_ = quater_length_ / 2;
          leaves_[leafnum]->UpdateOctoTree(pv);
        }
      }
      else
      {
        if (update_enable_)
        {
          new_points_++;
          temp_points_.push_back(pv);
          updateStats(pv.point_w, pv.var);
          if (new_points_ > update_size_threshold_)
          {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
            if ((n_stats_ >= freeze_min_points_) && !baseline_map_en_) planar_frozen_ = true;
          }
          if (temp_points_.size() > max_points_num_)
          {
            update_enable_ = false;
            saturated_ = true;
            std::vector<pointWithVar>().swap(temp_points_);
            new_points_ = 0;
          }
        }
      }
    }
  }
}

VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw)
{
  if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

  int xyz[3] = {0, 0, 0};
  xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
  xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
  xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
  int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

  // printf("leafnum: %d. \n", leafnum);

  return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv)
{
  if (!baseline_map_en_ && (saturated_ || planar_frozen_)) return this;  // SA-LIVO: frozen voxels stop updating

  if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) || (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_)))
  {
    new_points_++;
    temp_points_.push_back(pv);
    updateStats(pv.point_w, pv.var);
    return this;
  }

  if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_))
  {
    int xyz[3] = {0, 0, 0};
    xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] != nullptr) { return leaves_[leafnum]->Insert(pv); }
    else
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_,
                                           planarity_ratio_th_, min_eigen_abs_, freeze_min_points_, epsilon_min_adaptive_,
                                           baseline_map_en_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      return leaves_[leafnum]->Insert(pv);
    }
  }
  return nullptr;
}

void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{
  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  body_cov_list_.clear();
  body_cov_list_.reserve(feats_down_size_);

  // build_residual_time = 0.0;
  // ekf_time = 0.0;
  // double t0 = omp_get_wtime();

  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    if (point_this[2] == 0) { point_this[2] = 0.001; }
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var,
                config_setting_.near_range_extra_m_, config_setting_.near_range_scale_m_);
    body_cov_list_.push_back(var);
    point_this = extR_ * point_this + extT_;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }

  vector<pointWithVar>().swap(pv_list_);
  pv_list_.resize(feats_down_size_);

  int rematch_num = 0;
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
  for (int iterCount = 0; iterCount < config_setting_.max_iterations_; iterCount++)
  {
    double total_residual = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
    M3D rot_var = state_.cov.block<3, 3>(0, 0);
    M3D t_var = state_.cov.block<3, 3>(3, 3);
    for (size_t i = 0; i < feats_down_body_->size(); i++)
    {
      pointWithVar &pv = pv_list_[i];
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;

      const M3D &point_crossmat = cross_mat_list_[i];  // p_I skew
      const M3D Rwl = state_.rot_end * extR_;
      // Full world covariance for association/map update. body_cov_list_ is
      // LiDAR-frame noise; the rotational perturbation is right/body-frame.
      M3D cov;
      if (config_setting_.state_est_original_cov_en_)
      {
        // Original FAST-LIVO2 world covariance (proven on the official
        // baseline): cov = R·Σb·Rᵀ + (−⌊pI⌋)×·P_R·(−⌊pI⌋×)ᵀ + P_t.
        cov = state_.rot_end * body_cov_list_[i] * state_.rot_end.transpose() +
              (-point_crossmat) * rot_var * (-point_crossmat).transpose() + t_var;
      }
      else
      {
        const M3D sensor_cov = Rwl * body_cov_list_[i] * Rwl.transpose();
        cov = sensor_cov + t_var;
        if (g_right_invariant_en)
        {
          const M3D wcross = state_.rot_end * point_crossmat * state_.rot_end.transpose();
          cov += wcross * rot_var * wcross.transpose();
        }
        else
        {
          cov += state_.rot_end * point_crossmat * rot_var *
                 point_crossmat.transpose() * state_.rot_end.transpose();
        }
      }
      pv.var = cov;
      if (config_setting_.state_est_original_cov_en_)
        pv.var_nostate = state_.rot_end * body_cov_list_[i] * state_.rot_end.transpose();
      else
        pv.var_nostate = (state_.rot_end * extR_) * body_cov_list_[i] * (state_.rot_end * extR_).transpose();
      pv.body_var = body_cov_list_[i];
    }
    ptpl_list_.clear();

    // double t1 = omp_get_wtime();

    BuildResidualListOMP(pv_list_, ptpl_list_);

    // build_residual_time += omp_get_wtime() - t1;

    for (int i = 0; i < ptpl_list_.size(); i++)
    {
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
    effct_feat_num_ = ptpl_list_.size();
    cout << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_ 
         << " effective feature num: " << effct_feat_num_ << " average residual: " << total_residual / effct_feat_num_ << endl;

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
    MatrixXd Hsub(effct_feat_num_, 6);
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_);
    VectorXd R_inv(effct_feat_num_);
    VectorXd meas_vec(effct_feat_num_);
    meas_vec.setZero();
    for (int i = 0; i < effct_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/

      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end;
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;

      M3D var;
      // V3D normal_b = state_.rot_end.inverse() * ptpl_list_[i].normal_;
      // V3D point_b = ptpl_list_[i].point_b_;
      // double cos_theta = fabs(normal_b.dot(point_b) / point_b.norm());
      // ptpl_list_[i].body_cov_ = ptpl_list_[i].body_cov_ * (1.0 / cos_theta) * (1.0 / cos_theta);

      // point_w cov
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) + (-point_crossmat) * state_propagat.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose();

      // point_w cov (another_version)
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) - point_crossmat * state_propagat.cov.block<3, 3>(0, 0) * point_crossmat;

      // point_body cov
      var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose();

      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();

      R_inv(i) = 1.0 / (0.001 + sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);
      // R_inv(i) = 1.0 / (sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      /*** calculate the Measuremnt Jacobian matrix H ***/
      V3D A;
      if (g_right_invariant_en)
        A = state_.rot_end * point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_;
      else
        A = point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_;
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
    }
    EKF_stop_flg = false;
    flg_EKF_converged = false;
    /*** Iterative Kalman Filter Update ***/
    MatrixXd K(DIM_STATE, effct_feat_num_);
    // auto &&Hsub_T = Hsub.transpose();
    auto &&HTz = Hsub_T_R_inv * meas_vec;
    // fout_dbg<<"HTz: "<<HTz<<endl;
    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H.block<DIM_STATE, DIM_STATE>(0, 0) + state_.cov.block<DIM_STATE, DIM_STATE>(0, 0).inverse()).inverse();
    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
    auto vec = state_propagat - state_;
    VD(DIM_STATE)
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
    int minRow, minCol;
    state_ += solution;
    auto rot_add = solution.block<3, 1>(0, 0);
    auto t_add = solution.block<3, 1>(3, 0);
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }
    V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0);

    /*** Rematch Judgement ***/

    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2)))) { rematch_num++; }

    /*** Convergence Judgements and Covariance Update ***/
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1)))
    {
      /*** Covariance Update ***/
      // _state.cov = (I_STATE - G) * _state.cov;
      state_.cov.block<DIM_STATE, DIM_STATE>(0, 0) =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) - G.block<DIM_STATE, DIM_STATE>(0, 0)) * state_.cov.block<DIM_STATE, DIM_STATE>(0, 0);
      // total_distance += (_state.pos_end - position_last).norm();
      position_last_ = state_.pos_end;
      geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
      EKF_stop_flg = true;
    }
    if (EKF_stop_flg) break;
  }

  // double t2 = omp_get_wtime();
  // scan_count++;
  // ekf_time = t2 - t0 - build_residual_time;

  // ave_build_residual_time = ave_build_residual_time * (scan_count - 1) / scan_count + build_residual_time / scan_count;
  // ave_ekf_time = ave_ekf_time * (scan_count - 1) / scan_count + ekf_time / scan_count;

  // cout << "[ Mapping ] ekf_time: " << ekf_time << "s, build_residual_time: " << build_residual_time << "s" << endl;
  // cout << "[ Mapping ] ave_ekf_time: " << ave_ekf_time << "s, ave_build_residual_time: " << ave_build_residual_time << "s" << endl;
}

void VoxelMapManager::JointStateEstimation(StatesGroup &state_propagat, const InfoForm6 *vis_info, double q)
{
  static int dbg_sig_frames = 0;
  static std::vector<double> dbg_pv_term, dbg_pn_term, dbg_sig2s, dbg_rs;
  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  body_cov_list_.clear();
  body_cov_list_.reserve(feats_down_size_);

  // Beam-aligned body-frame point covariances and IMU-frame cross matrices
  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var,
                config_setting_.near_range_extra_m_, config_setting_.near_range_scale_m_);
    body_cov_list_.push_back(var);
    point_this = extR_ * point_this + extT_;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }

  vector<pointWithVar>().swap(pv_list_);
  pv_list_.resize(feats_down_size_);
  plane_cache_.assign(feats_down_size_, CachedPlane());

  const int N = feats_down_size_;
  const int max_iter = config_setting_.max_iterations_;
  const double tau_L = config_setting_.chi2_thres_;

  // Visual information form: assembled once at the propagated state and
  // frozen across all iterations (Algorithm 2 step 1). nullptr → LIO-only.
  InfoForm6 LambdaV;
  double q_used = 0.0;
  if (vis_info != nullptr)
  {
    LambdaV = *vis_info;
    q_used = q;
    if (config_setting_.force_lio_only_en_) q_used = 0.0;
  }

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> G_bar = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> prior_info = state_propagat.cov.inverse();
  if (config_setting_.prior_scale_when_statecov_ > 0.0 && config_setting_.sigma2_state_cov_scale_ > 0.0)
  {
    // The measurement noise already carries sigma2_state_cov_scale * (state
    // covariance); reduce the solve prior by the same fraction so the state
    // uncertainty is not counted twice.
    prior_info *= (1.0 - config_setting_.sigma2_state_cov_scale_);
  }
  Eigen::Matrix<double, DIM_STATE, 1> delta0 = Eigen::Matrix<double, DIM_STATE, 1>::Zero();
  bool have_gain = false;
  int conv_count = 0;
  int used_iters = 0;
  int final_survivors = 0;

  for (int iter = 0; iter < max_iter; iter++)
  {
    used_iters = iter + 1;

    // Keep every cached LiDAR/visual Jacobian in the propagated-state tangent.
    // Reconstruct the nonlinear state from x0 and the accumulated x0-tangent
    // increment instead of composing increments in changing body tangents.
    state_ = state_propagat;
    state_ += delta0;

    // --- transform points with the current iterate; full per-point world
    //     covariance for association/map update ------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
    const M3D R = state_.rot_end;
    const M3D rot_var = state_.cov.block<3, 3>(0, 0);
    const M3D t_var = state_.cov.block<3, 3>(3, 3);
    for (int i = 0; i < N; i++)
    {
      pointWithVar &pv = pv_list_[i];
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
      const M3D &body_cov = body_cov_list_[i];
      const M3D &cross = cross_mat_list_[i];
      pv.body_var = body_cov;
      if (iter == 0)
      {
        // v2 L1614-1629: point covariances are prepared once at the
        // propagated state and reused throughout the iterative solve.
        const M3D Rwl = R * extR_;
        pv.var_nostate = Rwl * body_cov * Rwl.transpose();
        if (g_right_invariant_en)
        {
          // Eq.12 under the right-invariant error: delta_phi is world-frame,
          // so the state-induced term is [RpI]x · P_rot · [RpI]x^T.
          const M3D wcross = R * cross * R.transpose();
          pv.var = pv.var_nostate + wcross * rot_var * wcross.transpose() + t_var;
        }
        else
        {
          pv.var = pv.var_nostate +
                   R * cross * rot_var * cross.transpose() * R.transpose() + t_var;
        }
      }
    }

    // --- LiDAR information form (ΛL, bL), Sect. V-F --------------------------
    // First iteration (paper): full search fills plane_cache_; later iterates
    // reuse the cached correspondences.  Diagnostic reassoc_each_iter_en
    // restores the original FAST-LIVO2 behavior (re-match every iterate).
    if (iter == 0 || config_setting_.reassoc_each_iter_en_)
    {
      ptpl_list_.clear();
      BuildResidualListOMP(pv_list_, ptpl_list_);
    }
    InfoForm6 LambdaL;
    int survivors = 0;
    std::unordered_set<std::string> plane_keys;
    for (int i = 0; i < N; i++)
    {
      CachedPlane &cp = plane_cache_[i];
      if (!cp.valid) continue;
      const pointWithVar &pv = pv_list_[i];

      // Signed point-to-plane distance at the current iterate (Eq. 5)
      const double r = cp.normal.dot(pv.point_w) + cp.d;

      // Eq.13: plane uncertainty + sensor-only point noise + epsilon0.
      // State uncertainty is already carried by the InEKF prior.
      if (iter == 0 || config_setting_.reassoc_each_iter_en_)
      {
        Eigen::Matrix<double, 1, 6> Jnq;
        Jnq.block<1, 3>(0, 0) = pv.point_w - cp.center;
        Jnq.block<1, 3>(0, 3) = -cp.normal;
        // Eq.15 point-noise term.  Empirically the full Eq.12 world covariance
        // (pv.var, which folds in the current state pose covariance) double-
        // counts the InEKF prior P^-1 and worsens APE (0.081 -> 0.204 on oq01).
        // The information-consistent choice in this EKF layout is the
        // sensor-only world point covariance (pv.var_nostate); plane
        // uncertainty is still propagated through cp.plane_var.
        // Diagnostic: sigma2_full_state_cov_en restores the paper's full
        // Eq.12 covariance in the residual variance, which attenuates LIO
        // information when state uncertainty grows (degenerate scenes).
        const M3D point_cov_used =
            (config_setting_.sigma2_full_state_cov_en_ || config_setting_.sigma2_state_cov_scale_ > 0.0)
                ? (pv.var_nostate +
                   config_setting_.sigma2_state_cov_scale_ * (pv.var - pv.var_nostate))
                : pv.var_nostate;
        cp.sigma2 = config_setting_.epsilon0_ +
                    config_setting_.plane_var_scale_ * (Jnq * cp.plane_var * Jnq.transpose()).value() +
                    (cp.normal.transpose() * point_cov_used * cp.normal).value();
      }
      if (iter == 0 || config_setting_.lidar_jacobian_recompute_en_ ||
          config_setting_.reassoc_each_iter_en_)
      {
        if (g_right_invariant_en)
        {
          // Eq.6 right-invariant: h_rot = [R pI]x n = R [pI]x R^T n.
          cp.h.block<3, 1>(0, 0) =
              R * cross_mat_list_[i] * R.transpose() * cp.normal;
        }
        else
        {
          cp.h.block<3, 1>(0, 0) = cross_mat_list_[i] * R.transpose() * cp.normal;
        }
        cp.h.block<3, 1>(3, 0) = cp.normal;
      }
      const double sigma2 = cp.sigma2;
      if (!(sigma2 > 0.0) || !std::isfinite(sigma2) || !cp.h.allFinite()) continue;

      // χ² innovation gate: r²/σ² > τL rejects outliers
      if (r * r / sigma2 > tau_L) continue;
      ++survivors;
      if (config_setting_.info_plane_decorr_en_)
      {
        char key[160];
        snprintf(key, sizeof(key), "%.4f %.4f %.4f %.3f %.3f %.3f %.3f",
                 cp.normal[0], cp.normal[1], cp.normal[2],
                 cp.center[0], cp.center[1], cp.center[2], cp.d);
        plane_keys.insert(key);
      }

      if (iter == 0 && dbg_sig_frames < 3)
      {
        Eigen::Matrix<double, 1, 6> Jnq3;
        Jnq3.block<1, 3>(0, 0) = pv.point_w - cp.center;
        Jnq3.block<1, 3>(0, 3) = -cp.normal;
        const double pvt = (Jnq3 * cp.plane_var * Jnq3.transpose()).value();
        const double pnt = (cp.normal.transpose() * pv.var_nostate * cp.normal).value();
        dbg_pv_term.push_back(pvt);
        dbg_pn_term.push_back(pnt);
        dbg_sig2s.push_back(sigma2);
        dbg_rs.push_back(std::fabs(r));
      }

      // Jacobian (Eq. 6, FAST-LIVO2 convention): h = [⌊p_I⌋_× Rᵀ n; n]
      LambdaL.accumulate(cp.h, 1.0 / sigma2, r);
    }
    final_survivors = survivors;
    if (config_setting_.info_plane_decorr_en_ && survivors > 0 && !plane_keys.empty())
    {
      // Per-frame decorrelation: the matched points on the same map plane are
      // correlated; scale the information form by the distinct-planes /
      // matched-points ratio (each scene gets its own effective count).
      const double ratio = std::min(1.0, static_cast<double>(plane_keys.size()) / static_cast<double>(survivors));
      LambdaL.Lambda *= ratio;
      LambdaL.b *= ratio;
    }
    if (iter == 0 && dbg_sig_frames < 3 && dbg_sig2s.size() > 20)
    {
      std::sort(dbg_pv_term.begin(), dbg_pv_term.end());
      std::sort(dbg_pn_term.begin(), dbg_pn_term.end());
      std::sort(dbg_sig2s.begin(), dbg_sig2s.end());
      std::sort(dbg_rs.begin(), dbg_rs.end());
      const size_t m = dbg_sig2s.size() / 2;
      printf("[ SA-SIG ] frame=%d n=%zu median plane_var_term=%.4e point_noise_term=%.4e sigma2=%.4e |r|=%.4e\n",
             dbg_sig_frames, dbg_sig2s.size(), dbg_pv_term[m], dbg_pn_term[m], dbg_sig2s[m], dbg_rs[m]);
      dbg_sig_frames++;
      dbg_pv_term.clear(); dbg_pn_term.clear(); dbg_sig2s.clear(); dbg_rs.clear();
    }
    static int dbg_match = 0;
    if (dbg_match++ < 100000)
    {
      int valid_cached = 0;
      for (int i = 0; i < N; i++) if (plane_cache_[i].valid) valid_cached++;
      printf("[ SA-LIO-MATCH ] N=%d candidates=%d survivors=%d\n", N, valid_cached, survivors);
    }

    // --- SAIF: subspace-aware fusion of the joint information form (Alg. 1) --
    InfoForm6 LambdaV_iter = LambdaV;
    if (config_setting_.frozen_rebase_en_)
      LambdaV_iter = rebaseFrozenInfo(LambdaV, delta0.template block<6, 1>(0, 0));
    // Scene-adaptive observability threshold: when enabled, raise σmin to
    // sqrt(λ_p10) of the joint information spectrum so the soft gate activates
    // on the weakest 10% of directions even when the raw information amplitude
    // is large (degenerate scenes where the fixed σmin=1 never triggers).
    double sigma_min_eff = config_setting_.saif_sigma_min_;
    if (config_setting_.saif_sigma_min_adaptive_)
    {
      InfoForm6 joint = LambdaL;
      joint.addScaled(LambdaV_iter, q_used);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es_a(joint.Lambda);
      if (es_a.info() == Eigen::Success)
      {
        Eigen::Matrix<double, 6, 1> lam = es_a.eigenvalues();
        if (config_setting_.saif_sigma_min_lambda_max_ > 0.0)
        {
          // Diagnostic: raise σmin relative to the STRONGEST joint direction
          // (σmin = scale·√λmax).  The weak directions of a compressed
          // spectrum (λmin ~ 1e3-1e4 ≫ 1) then fall below the gate and are
          // attenuated toward the IMU prior, while strong directions stay at
          // full weight — the paper's intended spectral-gap behavior.
          sigma_min_eff = std::max(sigma_min_eff,
                                   config_setting_.saif_sigma_min_lambda_max_ * std::sqrt(std::max(lam(5), 0.0)));
        }
        else
        {
          double lam_p10 = std::max(lam(0), 0.0);
          sigma_min_eff = std::max(sigma_min_eff, 2.0 * std::sqrt(lam_p10));
        }
      }
    }
    static const Eigen::Matrix<double, 6, 6> kNoPrior = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> prior_pose_mat = kNoPrior;
    if (config_setting_.saif_relative_prior_gate_en_)
      prior_pose_mat = prior_info.template block<6, 6>(0, 0);
    const Eigen::Matrix<double, 6, 6> *prior_pose_info =
        config_setting_.saif_relative_prior_gate_en_ ? &prior_pose_mat : nullptr;
    fused_last_ = saifFuse(LambdaL, LambdaV_iter, q_used, sigma_min_eff, prior_pose_info);

    if (iter == 0 && vis_info != nullptr)
    {
      static int dbg_l = 0;
      if ((dbg_l++ % 100) == 0 || dbg_l < 8)
      {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> esL(LambdaL.Lambda);
        printf("[ SA-LIO ] lamL=[%.1f %.1f %.1f %.1f %.1f %.1f] | |bL|=%.3f\n",
               esL.eigenvalues()(0), esL.eigenvalues()(1), esL.eigenvalues()(2),
               esL.eigenvalues()(3), esL.eigenvalues()(4), esL.eigenvalues()(5),
               LambdaL.b.norm());
      }
    }

    // --- unified joint InEKF solve at one linearization point (Eq. 38–40) ---
    // Eq.40 anchor term (x̂0 ⊟ x̂κ).  With the retraction x̂κ = x̂0 ⊞ delta0,
    // this equals -delta0 exactly in BOTH error conventions (left-invariant
    // R=R̂Exp(δφ) and right-invariant R=Exp(δφ)R̂), because x̂0 ⊟ (x̂0 ⊞ δ) = -δ.
    const Eigen::Matrix<double, DIM_STATE, 1> vec = -delta0;
    auto js = solveJointUpdate<DIM_STATE>(fused_last_, prior_info, vec);
    if (!js.ok || !js.dx.allFinite()) break;
    delta0 += js.dx;
    state_ = state_propagat;
    state_ += delta0;

    G_bar.setZero();
    G_bar.template block<DIM_STATE, 6>(0, 0) = js.G6;
    have_gain = true;

    // --- convergence: two consecutive small corrections (Algorithm 2) -------
    const double rot_add = js.dx.template block<3, 1>(0, 0).norm();
    const double t_add = js.dx.template block<3, 1>(3, 0).norm();
    if ((rot_add * 57.3 < config_setting_.conv_phi_deg_) && (t_add * 100 < config_setting_.conv_p_cm_))
    {
      conv_count++;
      if (conv_count >= 2) break;
    }
    else
    {
      conv_count = 0;
    }
  }

  // --- covariance update + pose-block inflation (Sect. VII-E) ---------------
  if (have_gain)
  {
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> I_STATE = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity();
    state_.cov = (I_STATE - G_bar) * state_.cov;
    state_.cov.template block<6, 6>(0, 0) *= (1.0 + config_setting_.eps_pose_inflate_);
  }

  effct_feat_num_ = final_survivors;
  position_last_ = state_.pos_end;

  degenerate_update_ = (final_survivors == 0) &&
      (vis_info == nullptr || q_used * (LambdaV.Lambda.norm() + LambdaV.b.norm()) < 1e-9);

  printf("[ SA-LIVO ] joint update: %d iters, %d final gated plane matches, gate weights: [%.3f %.3f %.3f %.3f %.3f %.3f]"
         " | |v|=%.3f covtr=%.3e |d|=%.3f lam=[%.2e %.2e %.2e %.2e %.2e %.2e]\n",
         used_iters, effct_feat_num_,
         fused_last_.gate_weights.size() > 0 ? fused_last_.gate_weights[0] : 0.0,
         fused_last_.gate_weights.size() > 1 ? fused_last_.gate_weights[1] : 0.0,
         fused_last_.gate_weights.size() > 2 ? fused_last_.gate_weights[2] : 0.0,
         fused_last_.gate_weights.size() > 3 ? fused_last_.gate_weights[3] : 0.0,
         fused_last_.gate_weights.size() > 4 ? fused_last_.gate_weights[4] : 0.0,
         fused_last_.gate_weights.size() > 5 ? fused_last_.gate_weights[5] : 0.0,
         state_.vel_end.norm(), state_.cov.trace(), delta0.norm(),
         fused_last_.eigenvals.size() > 0 ? fused_last_.eigenvals[0] : 0.0,
         fused_last_.eigenvals.size() > 1 ? fused_last_.eigenvals[1] : 0.0,
         fused_last_.eigenvals.size() > 2 ? fused_last_.eigenvals[2] : 0.0,
         fused_last_.eigenvals.size() > 3 ? fused_last_.eigenvals[3] : 0.0,
         fused_last_.eigenvals.size() > 4 ? fused_last_.eigenvals[4] : 0.0,
         fused_last_.eigenvals.size() > 5 ? fused_last_.eigenvals[5] : 0.0);
}

void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void VoxelMapManager::BuildVoxelMap()
{
  if (config_setting_.adaptive_support_en_)
  {
    adaptive_map_->clear();
    for (const auto &point : feats_down_world_->points)
    {
      const V3D point_w(point.x, point.y, point.z);
      if (point_w.allFinite()) adaptive_map_->addPoint(point_w);
    }
    std::cout << "[ VoxelMap v2 ] built flat cells: " << adaptive_map_->size() << std::endl;
    return;
  }

  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = convertToIntVectorSafe(config_setting_.layer_init_num_);

  std::vector<pointWithVar> input_points;

  for (size_t i = 0; i < feats_down_world_->size(); i++)
  {
    pointWithVar pv;
    pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var,
                config_setting_.near_range_extra_m_, config_setting_.near_range_scale_m_);
    const V3D point_i = extR_ * point_this + extT_;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_i);
    const M3D Rwl = state_.rot_end * extR_;
    const M3D rot_var = state_.cov.block<3, 3>(0, 0);
    if (g_right_invariant_en)
    {
      const M3D wcross = state_.rot_end * point_crossmat * state_.rot_end.transpose();
      var = Rwl * var * Rwl.transpose() + wcross * rot_var * wcross.transpose() +
            state_.cov.block<3, 3>(3, 3);
    }
    else
    {
      var = Rwl * var * Rwl.transpose() +
            state_.rot_end * point_crossmat * rot_var *
                point_crossmat.transpose() * state_.rot_end.transpose() +
            state_.cov.block<3, 3>(3, 3);
    }
    pv.var = var;
    input_points.push_back(pv);
  }

  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar p_v = input_points[i];
    const double voxel_size =
        config_setting_.adaptive_voxel_en_ ? voxelSizeForPoint(p_v.point_b) : voxel_size;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->updateStats(p_v.point_w, p_v.var);
      voxel_map_[position]->new_points_++;
    }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold,
                                                   config_setting_.planarity_ratio_th_, config_setting_.min_eigen_abs_,
                                                   config_setting_.freeze_min_points_,
                                                   config_setting_.epsilon_min_adaptive_,
                                                   config_setting_.baseline_octree_map_en_);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->voxel_size_ = voxel_size;
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->updateStats(p_v.point_w, p_v.var);
      voxel_map_[position]->new_points_++;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
    }
  }
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter)
  {
    iter->second->init_octo_tree();
  }
}

V3F VoxelMapManager::RGBFromVoxel(const V3D &input_point)
{
  int64_t loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = floor(input_point[j] / config_setting_.max_voxel_size_);
  }

  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  int64_t ind = loc_xyz[0] + loc_xyz[1] + loc_xyz[2];
  uint k((ind + 100000) % 3);
  V3F RGB((k == 0) * 255.0, (k == 1) * 255.0, (k == 2) * 255.0);
  // cout<<"RGB: "<<RGB.transpose()<<endl;
  return RGB;
}

void VoxelMapManager::seedMap(const StatesGroup &state)
{
  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  body_cov_list_.clear();
  body_cov_list_.reserve(feats_down_size_);
  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y,
                   feats_down_body_->points[i].z);
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var,
                config_setting_.near_range_extra_m_, config_setting_.near_range_scale_m_);
    body_cov_list_.push_back(var);
    point_this = extR_ * point_this + extT_;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }
  vector<pointWithVar>().swap(pv_list_);
  pv_list_.resize(feats_down_size_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
  TransformLidar(state.rot_end, state.pos_end, feats_down_body_, world_lidar);
  const M3D Rwl = state.rot_end * extR_;
  for (size_t i = 0; i < feats_down_size_; i++)
  {
    pointWithVar &pv = pv_list_[i];
    pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y,
        feats_down_body_->points[i].z;
    pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    pv.body_var = body_cov_list_[i];
    pv.var_nostate = Rwl * body_cov_list_[i] * Rwl.transpose();
    pv.var = pv.var_nostate;
  }
  UpdateVoxelMap(pv_list_);
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
  if (config_setting_.coarse_map_fallback_en_)
    updateCoarseMap(input_points);
  if (config_setting_.adaptive_support_en_)
  {
    for (const auto &point : input_points)
    {
      if (point.point_w.allFinite() && point.var.allFinite())
        adaptive_map_->addPoint(point.point_w, point.var);
    }
    return;
  }

  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = convertToIntVectorSafe(config_setting_.layer_init_num_);
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) { voxel_map_[position]->UpdateOctoTree(p_v); }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold,
                                                   config_setting_.planarity_ratio_th_, config_setting_.min_eigen_abs_,
                                                   config_setting_.freeze_min_points_,
                                                   config_setting_.epsilon_min_adaptive_,
                                                   config_setting_.baseline_octree_map_en_);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->voxel_size_ = voxel_size;
      voxel_map_[position]->UpdateOctoTree(p_v);
    }
  }
}

void VoxelMapManager::updateCoarseMap(const std::vector<pointWithVar> &input_points)
{
  // Dual-resolution diagnostic: a second octree at 2x the base voxel size
  // provides coarse-scale planes (e.g. a smooth ramp floor) for query points
  // whose fine voxel is non-planar.
  const float voxel_size = 2.0f * config_setting_.max_voxel_size_;
  const float planer_threshold = config_setting_.planner_threshold_;
  const int max_layer = config_setting_.max_layer_;
  const int max_points_num = config_setting_.max_points_num_;
  const std::vector<int> layer_init_num = convertToIntVectorSafe(config_setting_.layer_init_num_);
  for (const auto &p_v : input_points)
  {
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0f;
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_coarse_.find(position);
    if (iter != voxel_map_coarse_.end())
    {
      voxel_map_coarse_[position]->UpdateOctoTree(p_v);
    }
    else
    {
      VoxelOctoTree *octo = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold,
                                              config_setting_.planarity_ratio_th_, config_setting_.min_eigen_abs_,
                                              config_setting_.freeze_min_points_, config_setting_.epsilon_min_adaptive_,
                                              config_setting_.baseline_octree_map_en_);
      voxel_map_coarse_[position] = octo;
      octo->layer_init_num_ = layer_init_num;
      octo->quater_length_ = voxel_size / 4.0f;
      octo->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      octo->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      octo->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      octo->voxel_size_ = voxel_size;
      octo->UpdateOctoTree(p_v);
    }
  }
}

void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
  if (config_setting_.adaptive_support_en_)
  {
    ptpl_list.clear();
    plane_cache_.assign(pv_list.size(), CachedPlane());
    std::vector<PointToPlane> all_ptpl_list(pv_list.size());
    std::vector<std::uint8_t> useful_ptpl(pv_list.size(), 0U);
#ifdef MP_EN
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(pv_list.size()); ++i)
    {
      bool success = false;
      PointToPlane point_to_plane;
      build_single_residual_v2(pv_list[static_cast<std::size_t>(i)], success, point_to_plane);
      if (!success) continue;
      useful_ptpl[static_cast<std::size_t>(i)] = 1U;
      all_ptpl_list[static_cast<std::size_t>(i)] = point_to_plane;
      auto &cached = plane_cache_[static_cast<std::size_t>(i)];
      cached.valid = true;
      cached.normal = point_to_plane.normal_;
      cached.center = point_to_plane.center_;
      cached.d = point_to_plane.d_;
      cached.plane_var = point_to_plane.plane_var_;
    }
    for (std::size_t i = 0; i < useful_ptpl.size(); ++i)
    {
      if (useful_ptpl[i] != 0U) ptpl_list.push_back(all_ptpl_list[i]);
    }
    return;
  }

  int max_layer = config_setting_.max_layer_;
  double voxel_size = config_setting_.max_voxel_size_;
  double sigma_num = config_setting_.sigma_num_;
  ptpl_list.clear();
  plane_cache_.assign(pv_list.size(), CachedPlane());
  std::vector<PointToPlane> all_ptpl_list(pv_list.size());
  // vector<bool> bit-packs adjacent flags; parallel writes to distinct
  // indices can still race on the same storage word.
  std::vector<uint8_t> useful_ptpl(pv_list.size(), 0U);
  std::vector<size_t> index(pv_list.size());
  for (size_t i = 0; i < index.size(); ++i)
  {
    index[i] = i;
    useful_ptpl[i] = false;
  }
  #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
  #endif
  for (int i = 0; i < (int)index.size(); i++)
  {
    pointWithVar &pv = pv_list[i];
    PointToPlane single_ptpl;
    bool is_sucess = false;
    if (config_setting_.baseline_lio_association_en_)
    {
      // Diagnostic: original FAST-LIVO2 octree association (voxel + near-voxel
      // fallback). Kept behind a switch so the paper-faithful v2 flat-grid and
      // the SA octree selection remain the defaults.
      const double voxel_size_q =
          config_setting_.adaptive_voxel_en_ ? voxelSizeForPoint(pv.point_b) : voxel_size;
      float loc_xyz[3];
      for (int j = 0; j < 3; j++)
      {
        loc_xyz[j] = pv.point_w[j] / voxel_size_q;
        if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
      }
      VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
      auto iter = voxel_map_.find(position);
      if (iter != voxel_map_.end())
      {
        VoxelOctoTree *current_octo = iter->second;
        double prob = 0.0;
        build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl);
        if (!is_sucess)
        {
          VOXEL_LOCATION near_position = position;
          if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) { near_position.x = near_position.x + 1; }
          else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) { near_position.x = near_position.x - 1; }
          if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) { near_position.y = near_position.y + 1; }
          else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) { near_position.y = near_position.y - 1; }
          if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) { near_position.z = near_position.z + 1; }
          else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) { near_position.z = near_position.z - 1; }
          auto iter_near = voxel_map_.find(near_position);
          if (iter_near != voxel_map_.end())
          {
            build_single_residual(pv, iter_near->second, 0, is_sucess, prob, single_ptpl);
          }
        }
      }
      if (!is_sucess && config_setting_.coarse_map_fallback_en_)
      {
        // Dual-resolution fallback: query the 2x-coarse octree for a plane
        // (e.g. a smooth ramp floor whose fine voxel is non-planar).
        const double coarse_size = 2.0 * voxel_size_q;
        float cloc[3];
        for (int j = 0; j < 3; j++)
        {
          cloc[j] = pv.point_w[j] / coarse_size;
          if (cloc[j] < 0) cloc[j] -= 1.0;
        }
        VOXEL_LOCATION cpos((int64_t)cloc[0], (int64_t)cloc[1], (int64_t)cloc[2]);
        auto iter_c = voxel_map_coarse_.find(cpos);
        if (iter_c != voxel_map_coarse_.end())
        {
          double prob = 0.0;
          build_single_residual(pv, iter_c->second, 0, is_sucess, prob, single_ptpl);
        }
      }
    }
    else
    {
      // SA-LIVO (Sect. V-G): saturation-priority multi-scale plane selection.
      // No χ² gate here — the gate is applied at information-form assembly,
      // because the residual (and hence r²/σ²) changes across InEKF iterations.
      build_single_residual_sa(pv, is_sucess, single_ptpl);
    }
    if (is_sucess)
    {
      useful_ptpl[i] = true;
      all_ptpl_list[i] = single_ptpl;
      // Cache the correspondence: plane geometry does not change across
      // iterations, so later iterations recompute only the distance (O(N)).
      plane_cache_[i].valid = true;
      plane_cache_[i].normal = single_ptpl.normal_;
      plane_cache_[i].center = single_ptpl.center_;
      plane_cache_[i].d = single_ptpl.d_;
      plane_cache_[i].plane_var = single_ptpl.plane_var_;
    }
    else
    {
      useful_ptpl[i] = false;
    }
  }
  for (size_t i = 0; i < useful_ptpl.size(); i++)
  {
    if (useful_ptpl[i]) { ptpl_list.push_back(all_ptpl_list[i]); }
  }
}

void VoxelMapManager::build_single_residual_v2(pointWithVar &pv, bool &is_success,
                                                PointToPlane &single_ptpl)
{
  is_success = false;
  AdaptivePlaneFit fit;
  if (!adaptive_map_->queryPlane(pv.point_w, pv.var, fit)) return;

  single_ptpl.point_b_ = pv.point_b;
  single_ptpl.point_w_ = pv.point_w;
  single_ptpl.normal_ = fit.normal;
  single_ptpl.center_ = fit.center;
  single_ptpl.plane_var_ =
      config_setting_.plane_cov_paper_en_
          ? paperPlaneCov(fit.lambda_min, static_cast<int>(fit.support_count))
          : fit.covariance;
  single_ptpl.body_cov_ = pv.body_var;
  single_ptpl.layer_ = fit.shell;
  single_ptpl.d_ = fit.d;
  single_ptpl.eigen_value_ = fit.lambda_min;
  single_ptpl.is_valid_ = true;
  single_ptpl.dis_to_plane_ = static_cast<float>(fit.residual);
  pv.normal = fit.normal;
  is_success = true;
}

void VoxelMapManager::build_single_residual_sa(pointWithVar &pv, bool &is_sucess, PointToPlane &single_ptpl)
{
  is_sucess = false;
  const double voxel_size = voxelSizeForPoint(pv.point_b);
  const double kth = config_setting_.planarity_ratio_th_;
  const double emin = config_setting_.min_eigen_abs_;
  const int MIN_PTS = 5;

  // ---- query voxel location (with one-voxel shift if empty) ---------------
  float loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = pv.point_w[j] / voxel_size;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  VOXEL_LOCATION pos((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  auto iter = voxel_map_.find(pos);
  VoxelOctoTree *octo0 = nullptr;
  if (iter != voxel_map_.end())
  {
    octo0 = iter->second;
  }
  else
  {
    // one-voxel shift: probe the neighbour toward the point's fractional part
    VOXEL_LOCATION shifted = pos;
    for (int j = 0; j < 3; j++)
    {
      const int64_t pj = (j == 0) ? pos.x : (j == 1) ? pos.y : pos.z;
      const double center = (0.5 + pj) * voxel_size;
      const double quater = voxel_size / 4.0;
      const double pwj = (j == 0) ? pv.point_w[0] : (j == 1) ? pv.point_w[1] : pv.point_w[2];
      int64_t &sj = (j == 0) ? shifted.x : (j == 1) ? shifted.y : shifted.z;
      if (pwj > center + quater) sj = pj + 1;
      else if (pwj < center - quater) sj = pj - 1;
    }
    auto it2 = voxel_map_.find(shifted);
    if (it2 != voxel_map_.end()) octo0 = it2->second;
  }

  // ---- candidate set -------------------------------------------------------
  struct Cand
  {
    V3D normal, center;
    double d = 0.0;
    Eigen::Matrix<double, 6, 6> plane_var;  // Σplane (0 for aggregated fits)
    bool saturated = false, planar = false, valid = false;
    double kappa = 1.0 / 3.0;
    double r = 0.0, sigma = 1.0;
    int scale = 0;
  };
  std::vector<Cand> cands;
  cands.reserve(4);

  auto push_cand = [&](const V3D &n, const V3D &c, const Eigen::Matrix<double, 6, 6> &pvar, bool sat, bool plan,
                       double kappa, int scale, double lambda_min, int count)
  {
    Cand cd;
    cd.normal = n;
    cd.center = c;
    cd.d = -n.dot(c);
    cd.plane_var = config_setting_.plane_cov_paper_en_ ? paperPlaneCov(lambda_min, count) : pvar;
    cd.saturated = sat;
    cd.planar = plan;
    cd.valid = true;
    cd.kappa = kappa;
    cd.scale = scale;
    cd.r = n.dot(pv.point_w) + cd.d;
    Eigen::Matrix<double, 1, 6> Jnq;
    Jnq.block<1, 3>(0, 0) = pv.point_w - c;
    Jnq.block<1, 3>(0, 3) = -n;
    const double sig2 = 1e-3 + (Jnq * pvar * Jnq.transpose()).value() + (n.transpose() * pv.var * n).value();
    cd.sigma = std::sqrt(std::max(sig2, 1e-12));
    cands.push_back(cd);
  };

  // ---- Scale-0: single voxel (Sect. V-G) -----------------------------------
  if (octo0 != nullptr)
  {
    VoxelPlane &p0 = *octo0->plane_ptr_;
    if (p0.is_plane_)
    {
      // Original FAST-LIVO2 acceptance (build_single_residual): the query
      // point must lie within 3× the plane radius and within 3σ of the plane.
      // The SA path previously skipped these spatial gates and relied only on
      // the χ² gate at information assembly, which accepts far/off-plane
      // points whose biased residuals accumulate in degenerate scenes.
      const Eigen::Vector3d p_w2c = pv.point_w - p0.center_;
      const double dis2p = std::fabs(p0.normal_.dot(pv.point_w) + p0.d_);
      const double dis2c = p_w2c.squaredNorm();
      const double range_dis = std::sqrt(std::max(dis2c - dis2p * dis2p, 0.0));
      const bool in_radius = range_dis <= 3.0 * p0.radius_;
      Eigen::Matrix<double, 1, 6> Jnq_sp;
      Jnq_sp.block<1, 3>(0, 0) = p_w2c;
      Jnq_sp.block<1, 3>(0, 3) = -p0.normal_;
      const double sigma_l = (Jnq_sp * p0.plane_var_ * Jnq_sp.transpose()).value() +
                             p0.normal_.dot(pv.var * p0.normal_);
      const bool in_sigma = dis2p < 3.0 * std::sqrt(std::max(sigma_l, 1e-12));
      if (in_radius && in_sigma)
        push_cand(p0.normal_, p0.center_, p0.plane_var_, octo0->saturated_, true, octo0->planarity_ratio_, 0,
                  p0.min_eigen_value_, p0.points_size_);
    }
    // Note: no raw sufficient-statistics fallback for non-planar voxels.
    // Paper Pass 3 "geometrically valid" candidates come from the Scale-1
    // aggregated fits, not from a single non-planar voxel's stats (those raw
    // fits carry κ>0.1 with biased residuals and poisoned the information
    // form).  Non-planar query voxels fall through to the Scale-1 shells.
  }

  // ---- Scale-1: concentric Chebyshev shells, O(1) multi-scale PCA ----------
  bool skip_scale1 = false;
  if (!cands.empty() && cands[0].saturated && cands[0].planar && std::fabs(cands[0].r) / cands[0].sigma < 0.5)
  {
    skip_scale1 = true;  // stable region: Scale-0 saturated plane suffices
  }
  if (!skip_scale1 && config_setting_.multi_scale_en_)
  {
    const int lmax = std::min(std::max(static_cast<int>(std::ceil(config_setting_.scale1_radius_ / voxel_size)), 1),
                              config_setting_.max_shell_);

    if (config_setting_.scale1_region_grow_en_)
    {
      // Region-growing Scale-1 (diagnostic): seed from the query voxel's
      // statistics, then add shell voxels only when their centroid is close
      // to the current fit plane and their normal matches.  This stays on a
      // single surface (e.g. a stair/ramp slope) and avoids mixing the
      // opposite wall in narrow corridors.
      V3D s1g(V3D::Zero());
      M3D S2g(M3D::Zero());
      int ng = 0;
      std::vector<V3D> agg_points;
      std::vector<M3D> agg_covs;
      auto add_stats = [&](const VoxelOctoTree *vt)
      {
        s1g += vt->s1_;
        S2g += vt->S2_;
        agg_points.insert(agg_points.end(), vt->stats_points_obs_.begin(), vt->stats_points_obs_.end());
        agg_covs.insert(agg_covs.end(), vt->stats_cov_obs_.begin(), vt->stats_cov_obs_.end());
        ng += vt->n_stats_;
      };
      if (octo0 != nullptr && octo0->n_stats_ >= MIN_PTS)
        add_stats(octo0);
      auto fit_g = [&](V3D &mean, V3D &normal, double &kappa, double &lam_min)
      {
        if (ng < MIN_PTS) return false;
        mean = s1g / ng;
        const M3D cov = S2g / ng - mean * mean.transpose();
        Eigen::SelfAdjointEigenSolver<M3D> es(cov);
        lam_min = std::max(es.eigenvalues()(0), 0.0);
        kappa = lam_min / std::max(cov.trace(), 1e-12);
        normal = es.eigenvectors().col(0);
        return true;
      };
      V3D mean_g, normal_g;
      double kappa_g = 1.0 / 3.0, lam_g = 1.0;
      fit_g(mean_g, normal_g, kappa_g, lam_g);
      const double cos_same_surface = std::cos(30.0 * M_PI / 180.0);
      const double grow_dist = std::max(2.0 * voxel_size, 0.3);
      for (int l = 1; l <= lmax; l++)
      {
        bool changed = false;
        for (int dx = -l; dx <= l; dx++)
        {
          for (int dy = -l; dy <= l; dy++)
          {
            for (int dz = -l; dz <= l; dz++)
            {
              if (std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))) != l) continue;
              VOXEL_LOCATION np(pos.x + dx, pos.y + dy, pos.z + dz);
              auto it = voxel_map_.find(np);
              if (it == voxel_map_.end() || it->second->n_stats_ < MIN_PTS) continue;
              VoxelOctoTree *vt = it->second;
              V3D vmean;
              M3D vcov;
              if (!vt->statsCovariance(vmean, vcov)) continue;
              Eigen::SelfAdjointEigenSolver<M3D> es(vcov);
              const V3D vn = es.eigenvectors().col(0);
              if (std::fabs(vn.dot(normal_g)) < cos_same_surface) continue;
              const double d2p = std::fabs(normal_g.dot(vmean) + (-normal_g.dot(mean_g)));
              if (d2p > grow_dist) continue;
              add_stats(vt);
              changed = true;
              fit_g(mean_g, normal_g, kappa_g, lam_g);
            }
          }
        }
        if (!changed) break;
      }
      if (ng >= MIN_PTS && (kappa_g < config_setting_.planarity_ratio_th_) &&
          (lam_g < config_setting_.min_eigen_abs_))
      {
        push_cand(normal_g, mean_g, Eigen::Matrix<double, 6, 6>::Zero(), false, true, kappa_g, 1, lam_g, ng);
      }
    }
    else
    {

      // Reference normal for same-surface aggregation: use the Scale-0 plane
      // normal when available, otherwise the strongest (min-eigenvector) fit
      // from the query voxel's sufficient statistics. Neighbour voxels whose
      // fitted normal deviates by more than 30° are treated as belonging to a
      // different surface (e.g. the opposite wall in a corridor) and excluded
      // from the aggregation, preventing mixed-surface plane fits.
    const double cos_same_surface = config_setting_.shell_aggregate_cos_;
    V3D ref_normal = V3D::Zero();
    bool have_ref = false;
    if (octo0 != nullptr && octo0->plane_ptr_->is_plane_)
    {
      ref_normal = octo0->plane_ptr_->normal_;
      have_ref = true;
    }
    else
    {
      // Robust reference: if the query voxel is non-planar (or absent), use
      // the most-planar voxel (smallest κ) in the shell as the same-surface
      // reference.  A raw stats fit of a non-planar query voxel is an
      // unreliable reference and caused the aggregation filter to exclude the
      // correct neighbours (or, with no filter, to mix surfaces).
      double best_kappa = 1.0 / 3.0;
      for (int lr = 0; lr <= lmax && !have_ref; lr++)
      {
        for (int dx = -lr; dx <= lr; dx++)
        {
          for (int dy = -lr; dy <= lr; dy++)
          {
            for (int dz = -lr; dz <= lr; dz++)
            {
              if (std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))) != lr) continue;
              VOXEL_LOCATION np(pos.x + dx, pos.y + dy, pos.z + dz);
              auto it = voxel_map_.find(np);
              if (it == voxel_map_.end() || it->second->n_stats_ < MIN_PTS) continue;
              V3D mean;
              M3D cov;
              if (it->second->plane_ptr_->is_plane_)
              {
                ref_normal = it->second->plane_ptr_->normal_;
                have_ref = true;
                break;
              }
              if (it->second->statsCovariance(mean, cov))
              {
                Eigen::SelfAdjointEigenSolver<M3D> es(cov);
                const double kappa = es.eigenvalues()(0) / std::max(cov.trace(), 1e-12);
                if (kappa < best_kappa)
                {
                  best_kappa = kappa;
                  ref_normal = es.eigenvectors().col(0);
                  have_ref = true;
                }
              }
            }
            if (have_ref) break;
          }
          if (have_ref) break;
        }
      }
    }

    // aggregate sufficient statistics (Eq. 10) over shells 0..ℓ
    V3D s1(V3D::Zero());
    M3D S2(M3D::Zero());
    std::vector<V3D> agg_points;
    std::vector<M3D> agg_covs;
    int n = 0;
    if (octo0 != nullptr)
    {
      s1 += octo0->s1_;
      S2 += octo0->S2_;
      agg_points.insert(agg_points.end(), octo0->stats_points_obs_.begin(),
                        octo0->stats_points_obs_.end());
      agg_covs.insert(agg_covs.end(), octo0->stats_cov_obs_.begin(),
                      octo0->stats_cov_obs_.end());
      n += octo0->n_stats_;
    }
    for (int l = 1; l <= lmax; l++)
    {
      for (int dx = -l; dx <= l; dx++)
      {
        for (int dy = -l; dy <= l; dy++)
        {
          for (int dz = -l; dz <= l; dz++)
          {
            if (std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))) != l) continue;
            VOXEL_LOCATION np(pos.x + dx, pos.y + dy, pos.z + dz);
            auto it = voxel_map_.find(np);
            if (it != voxel_map_.end() && it->second->n_stats_ >= 3)
            {
              if (have_ref)
              {
                V3D nbr_normal = V3D::Zero();
                if (it->second->plane_ptr_->is_plane_)
                  nbr_normal = it->second->plane_ptr_->normal_;
                else if (it->second->n_stats_ >= 3)
                {
                  V3D mean;
                  M3D cov;
                  if (it->second->statsCovariance(mean, cov))
                  {
                    Eigen::SelfAdjointEigenSolver<M3D> es(cov);
                    nbr_normal = es.eigenvectors().col(0);
                  }
                }
                if (nbr_normal.norm() > 1e-6 && std::fabs(nbr_normal.dot(ref_normal)) < cos_same_surface)
                  continue;  // different surface — exclude from aggregation
              }
              s1 += it->second->s1_;
              S2 += it->second->S2_;
              agg_points.insert(agg_points.end(), it->second->stats_points_obs_.begin(),
                               it->second->stats_points_obs_.end());
              agg_covs.insert(agg_covs.end(), it->second->stats_cov_obs_.begin(),
                              it->second->stats_cov_obs_.end());
              n += it->second->n_stats_;
            }
          }
        }
      }
      if (n < MIN_PTS) continue;

      // constant-time PCA from the aggregated statistics
      const V3D mean = s1 / n;
      const M3D cov = S2 / n - mean * mean.transpose();
      Eigen::SelfAdjointEigenSolver<M3D> es(cov);
      const double lam_min = std::max(es.eigenvalues()(0), 0.0);
      const double tr = std::max(cov.trace(), 1e-12);
      const double kappa = lam_min / tr;
      const bool dual = (kappa < kth) && (lam_min < emin);

      Eigen::Matrix<double, 6, 6> plane_var = Eigen::Matrix<double, 6, 6>::Zero();
      if (dual && agg_points.size() == agg_covs.size())
      {
        const int min_idx = 0;  // SelfAdjointEigenSolver returns ascending λ
        for (std::size_t i = 0; i < agg_points.size(); ++i)
        {
          Eigen::Matrix<double, 6, 3> Ji;
          Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
          for (int m = 0; m < 3; ++m)
          {
            if (m == min_idx) continue;
            const double denom = static_cast<double>(n) *
                                 (std::max(es.eigenvalues()(min_idx), 0.0) - es.eigenvalues()(m));
            if (std::fabs(denom) < 1e-12) continue;
            const V3D vm = es.eigenvectors().col(m);
            const V3D vmin = es.eigenvectors().col(min_idx);
            F.row(m) = ((agg_points[i] - mean).transpose() / denom) *
                       (vm * vmin.transpose() + vmin * vm.transpose());
          }
          Ji.block<3, 3>(0, 0) = es.eigenvectors() * F;
          Ji.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() / static_cast<double>(n);
          plane_var += Ji * agg_covs[i] * Ji.transpose();
        }
      }

      // Paper Sect. V-G: Scale-1 provides the coarser-scale PLANAR fit;
      // expansion stops as soon as the dual test is met.  Pushing the
      // intermediate non-planar fits (as before) flooded Pass 3/4 with
      // far/off-plane candidates (κ up to 0.09, |r|/σ up to 18.8) that biased
      // the information form.  Only a dual-passing fit becomes a candidate.
      if (dual)
      {
        // The aggregated fit must be anchored near the query point: require
        // the in-plane distance to the fit centroid to be within
        // gate*radius.  Without this, shell fits pulled by mixed surfaces
        // (e.g. the opposite wall in a narrow corridor) match far points
        // with biased residuals even though the fit itself is planar.
        const double rad = std::sqrt(std::max(es.eigenvalues()(2), 0.0));
        const double dis2p = std::fabs(es.eigenvectors().col(0).dot(pv.point_w) +
                                       (-es.eigenvectors().col(0).dot(mean)));
        const double range_dis = std::sqrt(std::max((pv.point_w - mean).squaredNorm() - dis2p * dis2p, 0.0));
        if (config_setting_.sa_scale1_range_gate_ <= 0.0 ||
            range_dis <= config_setting_.sa_scale1_range_gate_ * rad)
        {
          push_cand(es.eigenvectors().col(0), mean, plane_var, false, true, kappa, 1, lam_min,
                    static_cast<int>(agg_points.size()));
        }
        break;
      }
    }
  }
  }

  if (cands.empty())
  {
    return;  // is_sucess = false
  }

  // ---- normal-consistency filter: keep candidates within 18° of the most
  // planar candidate (cos⁻¹(0.95), Sect. V-G) --------------------------------
  if (cands.size() > 1)
  {
    size_t best = 0;
    for (size_t i = 1; i < cands.size(); i++)
    {
      if (cands[i].kappa < cands[best].kappa) best = i;
    }
    const V3D n_ref = cands[best].normal;
    std::vector<Cand> filtered;
    filtered.reserve(cands.size());
    for (size_t i = 0; i < cands.size(); i++)
    {
      if (i == best || std::fabs(cands[i].normal.dot(n_ref)) >= config_setting_.normal_consistency_cos_)
      {
        filtered.push_back(cands[i]);
      }
    }
    cands.swap(filtered);
  }

  // ---- saturation-priority selection: four ordered passes (Sect. V-G) ------
  auto pick = [&](const std::function<bool(const Cand &)> &pred) -> const Cand *
  {
    const Cand *chosen = nullptr;
    for (const auto &c : cands)
    {
      if (!pred(c)) continue;
      if (chosen == nullptr || c.kappa < chosen->kappa ||
          (std::fabs(c.kappa - chosen->kappa) < 1e-12 && std::fabs(c.r) / c.sigma < std::fabs(chosen->r) / chosen->sigma))
      {
        chosen = &c;
      }
    }
    return chosen;
  };

  const Cand *chosen = nullptr;
  chosen = pick([](const Cand &c) { return c.saturated; });                      // Pass 1: saturated anchors
  if (chosen == nullptr) chosen = pick([](const Cand &c) { return c.planar && !c.saturated; });  // Pass 2: planar unsaturated
  if (chosen == nullptr) chosen = pick([](const Cand &c) { return c.valid; });   // Pass 3: any valid plane
  if (chosen == nullptr)                                                          // Pass 4: smallest |r|/σ
  {
    for (const auto &c : cands)
    {
      if (chosen == nullptr || std::fabs(c.r) / c.sigma < std::fabs(chosen->r) / chosen->sigma) chosen = &c;
    }
  }
  if (chosen == nullptr) return;

  // ---- fill the point-to-plane match ----------------------------------------
  single_ptpl.body_cov_ = pv.body_var;
  single_ptpl.point_b_ = pv.point_b;
  single_ptpl.point_w_ = pv.point_w;
  single_ptpl.plane_var_ = chosen->plane_var;
  single_ptpl.normal_ = chosen->normal;
  single_ptpl.center_ = chosen->center;
  single_ptpl.d_ = chosen->d;
  single_ptpl.layer_ = 0;
  single_ptpl.dis_to_plane_ = chosen->r;
  pv.normal = chosen->normal;
  is_sucess = true;
  static long dbg_sa_calls = 0;
  static long dbg_sa_s1 = 0;
  if ((dbg_sa_calls++ % 20000) == 0)
  {
    printf("[ SA-CAND ] calls=%ld scale1_frac=%.3f last_scale=%d last_kappa=%.4f last_r_sig=%.3f\n",
           dbg_sa_calls, static_cast<double>(dbg_sa_s1) / std::max(dbg_sa_calls, 1L), chosen->scale,
           chosen->kappa, std::fabs(chosen->r) / chosen->sigma);
  }
  if (chosen->scale == 1) dbg_sa_s1++;
}

void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess,
                                            double &prob, PointToPlane &single_ptpl)
{
  int max_layer = config_setting_.max_layer_;
  double sigma_num = config_setting_.sigma_num_;

  double radius_k = 3;
  Eigen::Vector3d p_w = pv.point_w;
  if (current_octo->plane_ptr_->is_plane_)
  {
    VoxelPlane &plane = *current_octo->plane_ptr_;
    Eigen::Vector3d p_world_to_center = p_w - plane.center_;
    float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);
    float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                          (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

    if (range_dis <= radius_k * plane.radius_)
    {
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
      J_nq.block<1, 3>(0, 3) = -plane.normal_;
      double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
      sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
      if (dis_to_plane < sigma_num * sqrt(sigma_l))
      {
        is_sucess = true;
        double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
        if (this_prob > prob)
        {
          prob = this_prob;
          pv.normal = plane.normal_;
          single_ptpl.body_cov_ = pv.body_var;
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          single_ptpl.plane_var_ =
              config_setting_.plane_cov_paper_en_
                  ? paperPlaneCov(plane.min_eigen_value_, plane.points_size_)
                  : plane.plane_var_;
          single_ptpl.normal_ = plane.normal_;
          single_ptpl.center_ = plane.center_;
          single_ptpl.d_ = plane.d_;
          single_ptpl.layer_ = current_layer;
          single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        }
        return;
      }
      else
      {
        // is_sucess = false;
        return;
      }
    }
    else
    {
      // is_sucess = false;
      return;
    }
  }
  else
  {
    if (current_layer < max_layer)
    {
      for (size_t leafnum = 0; leafnum < 8; leafnum++)
      {
        if (current_octo->leaves_[leafnum] != nullptr)
        {

          VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
          build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
        }
      }
      return;
    }
    else { return; }
  }
}

void VoxelMapManager::pubVoxelMap()
{
  double max_trace = 0.25;
  double pow_num = 0.2;
  rclcpp::Rate loop(500);
  float use_alpha = 0.8;
  visualization_msgs::msg::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);
  std::vector<VoxelPlane> pub_plane_list;
  if (config_setting_.adaptive_support_en_)
  {
    for (const auto &entry : adaptive_map_->cells())
    {
      const auto &k = entry.first;
      const V3D query((static_cast<double>(k.x) + 0.5) * config_setting_.max_voxel_size_,
                      (static_cast<double>(k.y) + 0.5) * config_setting_.max_voxel_size_,
                      (static_cast<double>(k.z) + 0.5) * config_setting_.max_voxel_size_);
      AdaptivePlaneFit fit;
      if (!adaptive_map_->queryPlane(query, Eigen::Matrix3d::Zero(), fit)) continue;
      VoxelPlane p;
      p.center_ = fit.center;
      p.normal_ = fit.normal;
      p.x_normal_ = fit.normal.unitOrthogonal().normalized();
      p.y_normal_ = fit.normal.cross(p.x_normal_).normalized();
      p.plane_var_ = fit.covariance;
      p.radius_ = static_cast<float>(fit.radius);
      p.min_eigen_value_ = static_cast<float>(fit.lambda_min);
      p.d_ = static_cast<float>(fit.d);
      p.points_size_ = static_cast<int>(fit.support_count);
      p.is_plane_ = p.is_init_ = p.is_update_ = true;
      pub_plane_list.push_back(p);
    }
  }
  else
  {
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
    {
      GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
    }
  }
  for (size_t i = 0; i < pub_plane_list.size(); i++)
  {
    V3D plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) { trace = max_trace; }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    double alpha;
    if (pub_plane_list[i].is_plane_) { alpha = use_alpha; }
    else { alpha = 0; }
    pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
  }
  voxel_map_pub_->publish(voxel_plane);
  loop.sleep();
}

void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list)
{
  if (current_octo->layer_ > pub_max_voxel_layer) { return; }
  if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
  if (current_octo->layer_ < current_octo->max_layer_)
  {
    if (!current_octo->plane_ptr_->is_plane_)
    {
      for (size_t i = 0; i < 8; i++)
      {
        if (current_octo->leaves_[i] != nullptr) { GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list); }
      }
    }
  }
  return;
}

void VoxelMapManager::collectMapPlanes(std::vector<VoxelPlane> &plane_list)
{
  plane_list.clear();
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
  {
    if (iter->second == nullptr) continue;
    GetUpdatePlane(iter->second, config_setting_.max_layer_, plane_list);
  }
}

void VoxelMapManager::pubSinglePlane(visualization_msgs::msg::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane,
                                     const float alpha, const Eigen::Vector3d rgb)
{
  visualization_msgs::msg::Marker plane;
  plane.header.frame_id = "camera_init";
  plane.header.stamp = rclcpp::Time();
  plane.ns = plane_ns;
  plane.id = single_plane.id_;
  plane.type = visualization_msgs::msg::Marker::CYLINDER;
  plane.action = visualization_msgs::msg::Marker::ADD;
  plane.pose.position.x = single_plane.center_[0];
  plane.pose.position.y = single_plane.center_[1];
  plane.pose.position.z = single_plane.center_[2];
  geometry_msgs::msg::Quaternion q;
  CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
  plane.pose.orientation = q;
  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_);
  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = rclcpp::Duration::from_seconds(0.01);
  plane_pub.markers.push_back(plane);
}

void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                                      geometry_msgs::msg::Quaternion &q)
{
  Eigen::Matrix3d rot;
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
  Eigen::Matrix3d rotation = rot.transpose();
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) { v = vmin; }

  if (v > vmax) { v = vmax; }

  double dr, dg, db;

  if (v < 0.1242)
  {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  }
  else if (v < 0.3747)
  {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  }
  else if (v < 0.6253)
  {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  }
  else if (v < 0.8758)
  {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  }
  else
  {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void VoxelMapManager::mapSliding()
{
  if((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh)
  {
    std::cout<<RED<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
    return;
  }

  //get global id now
  last_slide_position = position_last_;
  double t_sliding_start = omp_get_wtime();
  float loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
  if (config_setting_.adaptive_support_en_)
  {
    const AdaptiveVoxelKey center = adaptive_map_->keyOf(position_last_);
    const std::int64_t half = static_cast<std::int64_t>(config_setting_.half_map_size);
    const std::size_t erased = adaptive_map_->eraseOutside(
        {center.x - half, center.y - half, center.z - half},
        {center.x + half, center.y + half, center.z + half});
    std::cout << RED << "[DEBUG]: Delete " << erased << " flat voxels" << RESET << "\n";
  }
  else
  {
    clearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size, (int64_t)loc_xyz[0] - config_setting_.half_map_size,
                     (int64_t)loc_xyz[1] + config_setting_.half_map_size, (int64_t)loc_xyz[1] - config_setting_.half_map_size,
                     (int64_t)loc_xyz[2] + config_setting_.half_map_size, (int64_t)loc_xyz[2] - config_setting_.half_map_size);
  }
  double t_sliding_end = omp_get_wtime();
  std::cout<<RED<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
  return;
}

void VoxelMapManager::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
  int delete_voxel_cout = 0;
  // double delete_time = 0;
  // double last_delete_time = 0;
  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); )
  {
    const VOXEL_LOCATION& loc = it->first;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      // last_delete_time = omp_get_wtime();
      delete it->second;
      it = voxel_map_.erase(it);
      // delete_time += omp_get_wtime() - last_delete_time;
      delete_voxel_cout++;
    } else {
      ++it;
    }
  }
  std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<RESET<<"\n";
  // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}
