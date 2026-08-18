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

#include "LIVMapper.h"
#include <vikit/camera_loader.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <thread>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <opencv2/imgcodecs.hpp>

using namespace Sophus;
LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name, const rclcpp::NodeOptions & options)
    : node(std::make_shared<rclcpp::Node>(node_name, options)),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);
  
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(this->node);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this->node, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_rgb_wait_pub.reset(new PointCloudXYZRGB());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents(this->node);          // initialize components errors
  path.header.stamp = this->node->now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &node)
{
  // declare parameters
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
  try_declare.template operator()<std::string>("common.lid_topic", "/livox/lidar");
  try_declare.template operator()<std::string>("common.imu_topic", "/livox/imu");
  try_declare.template operator()<bool>("common.ros_driver_bug_fix", false);
  try_declare.template operator()<bool>("common.lossless_replay_qos", false);
  try_declare.template operator()<int>("common.img_en", 1);
  try_declare.template operator()<int>("common.lidar_en", 1);
  try_declare.template operator()<std::string>("common.img_topic", "/left_camera/image");
  try_declare.template operator()<std::string>("common.img2_topic", "");
  try_declare.template operator()<bool>("common.img_compressed_en", false);
  try_declare.template operator()<double>("common.img_rate_hz", 0.0);
  try_declare.template operator()<int>("common.img_buffer_max", 60);

  try_declare.template operator()<bool>("vio.normal_en", true);
  try_declare.template operator()<bool>("vio.inverse_composition_en", false);
  try_declare.template operator()<int>("vio.max_iterations", 5);
  try_declare.template operator()<int>("vio.img_point_cov", 100);
  try_declare.template operator()<bool>("vio.raycast_en", false);
  try_declare.template operator()<bool>("vio.exposure_estimate_en", true);
  try_declare.template operator()<double>("vio.inv_expo_cov", 0.1);
  try_declare.template operator()<int>("vio.grid_size", 5);
  try_declare.template operator()<int>("vio.grid_n_height", 17);
  try_declare.template operator()<int>("vio.patch_pyrimid_level", 4);
  try_declare.template operator()<int>("vio.patch_size", 8);
  try_declare.template operator()<int>("vio.outlier_threshold", 100);
  try_declare.template operator()<double>("time_offset.exposure_time_init", 0.0);
  try_declare.template operator()<double>("time_offset.img_time_offset", 0.0);
  try_declare.template operator()<double>("common.startup_warmup_s", 0.0);
  try_declare.template operator()<double>("common.sync_lookahead_s", 0.0);
  try_declare.template operator()<std::string>("common.direct_bag_path", "");
  try_declare.template operator()<int>("lio.map_insert_every", 1);
  try_declare.template operator()<int>("lio.lag_smooth", 0);
  try_declare.template operator()<int>("lio.lag_iters", 3);
  try_declare.template operator()<double>("lio.lag_odom_weight", 1.0);
  try_declare.template operator()<bool>("common.velocity_init_en", false);
  try_declare.template operator()<bool>("common.dump_attitude_en", false);
  try_declare.template operator()<bool>("common.zupt_en", false);
  try_declare.template operator()<double>("common.zupt_vel_thresh", 0.05);
  try_declare.template operator()<double>("common.zupt_hold_s", 1.0);
  try_declare.template operator()<bool>("state.right_invariant_en", false);
  try_declare.template operator()<bool>("uav.imu_rate_odom", false);
  try_declare.template operator()<bool>("uav.gravity_align_en", false);

  // SA-LIVO information-form VIO (Sect. VI)
  try_declare.template operator()<int>("sa_vio.window_size", 5);
  try_declare.template operator()<int>("sa_vio.patch_spacing", 8);
  try_declare.template operator()<int>("sa_vio.max_obs_per_frame", 100);
  try_declare.template operator()<int>("sa_vio.min_obs", 30);
  try_declare.template operator()<double>("sa_vio.depth_min", 0.5);
  try_declare.template operator()<double>("sa_vio.theta_max_deg", 80.0);
  try_declare.template operator()<double>("sa_vio.parallax_min_px", 0.5);
  try_declare.template operator()<double>("sa_vio.abs_res_max", 15.0);
  try_declare.template operator()<double>("sa_vio.chi2_thres", 6.635);
  try_declare.template operator()<double>("sa_vio.delta_rms", 12.0);
  try_declare.template operator()<double>("sa_vio.sigma_rms", 8.0);
  try_declare.template operator()<double>("sa_vio.rho_min", 0.3);
  try_declare.template operator()<double>("sa_vio.rho_max", 0.7);
  try_declare.template operator()<double>("sa_vio.delta_alpha", 0.2);
  try_declare.template operator()<double>("sa_vio.sigma_px", 1.0);
  try_declare.template operator()<bool>("sa_vio.photometric_huber_en", false);
  try_declare.template operator()<bool>("sa_vio.patch_mean_noise_en", true);
  try_declare.template operator()<bool>("sa_vio.adaptive_noise_en", false);
  try_declare.template operator()<double>("sa_vio.adaptive_noise_min_sigma", 1.0);
  try_declare.template operator()<double>("sa_vio.info_scale", 1.0);
  try_declare.template operator()<double>("sa_vio.lamv_max", 0.0);
  try_declare.template operator()<double>("sa_vio.visual_start_s", 0.0);
  try_declare.template operator()<double>("sa_vio.point_image_cell_px", 16.0);
  try_declare.template operator()<bool>("sa_vio.point_select_image_grid", false);
  try_declare.template operator()<bool>("sa_vio.affine_equal_weight", false);
  try_declare.template operator()<int>("sa_vio.patch_search_px", 0);
  try_declare.template operator()<bool>("sa_vio.rrms_after_gates", false);
  try_declare.template operator()<bool>("sa_vio.debug_obs_gates", false);
  try_declare.template operator()<bool>("sa_vio.sigma2_sensor_only", false);
  try_declare.template operator()<bool>("sa_vio.visual_anchor_from_map", false);
  try_declare.template operator()<bool>("sa_vio.sigma2_live_cov", false);
  try_declare.template operator()<bool>("sa_vio.dual_event_en", false);
  try_declare.template operator()<bool>("sa_vio.use_baseline_vio_frontend", false);
  try_declare.template operator()<bool>("sa_vio.baseline_vio_sequential", false);
  try_declare.template operator()<bool>("sa_vio.baseline_lio_state_estimation", false);
  try_declare.template operator()<bool>("sa_vio.seq_output_post_vio", false);
  try_declare.template operator()<bool>("sa_vio.seq_color_pre_vio", false);
  try_declare.template operator()<bool>("sa_vio.seq_color_lag", false);
  try_declare.template operator()<bool>("saif.degenerate_attitude_hold", false);
  try_declare.template operator()<double>("sa_vio.alpha_min", 0.2);
  try_declare.template operator()<double>("sa_vio.alpha_max", 3.0);
  try_declare.template operator()<double>("sa_vio.beta_min", -50.0);
  try_declare.template operator()<double>("sa_vio.beta_max", 50.0);
  try_declare.template operator()<double>("sa_vio.dup_R_deg", 0.5);
  try_declare.template operator()<double>("sa_vio.dup_t_m", 0.05);

  try_declare.template operator()<std::string>("evo.seq_name", "01");
  try_declare.template operator()<bool>("evo.pose_output_en", false);
  try_declare.template operator()<double>("imu.gyr_cov", 1.0);
  try_declare.template operator()<double>("imu.acc_cov", 1.0);
  try_declare.template operator()<int>("imu.imu_int_frame", 30);
  try_declare.template operator()<bool>("imu.use_baseline_init", false);
  try_declare.template operator()<bool>("imu.imu_en", true);
  try_declare.template operator()<bool>("imu.gravity_est_en", true);
  try_declare.template operator()<bool>("imu.ba_bg_est_en", true);

  try_declare.template operator()<double>("preprocess.blind", 0.01);
  try_declare.template operator()<double>("preprocess.filter_size_surf", 0.5);
  try_declare.template operator()<int>("preprocess.lidar_type", AVIA);
  try_declare.template operator()<int>("preprocess.scan_line",6);
  try_declare.template operator()<int>("preprocess.point_filter_num", 3);
  try_declare.template operator()<int>("preprocess.scan_rate", 10);
  try_declare.template operator()<bool>("preprocess.feature_extract_enabled", false);

  try_declare.template operator()<int>("pcd_save.interval", -1);
  try_declare.template operator()<bool>("pcd_save.pcd_save_en", false);
  try_declare.template operator()<int>("pcd_save.color_delay_frames", 1);
  try_declare.template operator()<double>("pcd_save.color_prev_mix", 0.0);
  try_declare.template operator()<double>("pcd_save.color_blur_sigma", 0.0);
  try_declare.template operator()<double>("pcd_save.color_pose_shift_s", 0.0);
  try_declare.template operator()<bool>("pcd_save.color_baseline_pairing_en", false);
  try_declare.template operator()<bool>("pcd_save.color_future_pose_en", false);
  try_declare.template operator()<bool>("pcd_save.color_fullres_en", false);
  try_declare.template operator()<bool>("sa_ba.en", false);
  try_declare.template operator()<int>("sa_ba.window_size", 5);
  try_declare.template operator()<int>("sa_ba.iters", 5);
  try_declare.template operator()<double>("sa_ba.visual_weight", 1.0);
  try_declare.template operator()<double>("sa_ba.odom_weight", 1.0);
  try_declare.template operator()<double>("sa_ba.anchor_weight", 1e6);
  try_declare.template operator()<bool>("sa_ba.photometric_en", false);
  try_declare.template operator()<std::string>("sa_ba.output", "color");
  try_declare.template operator()<bool>("pcd_save.colmap_output_en", false);
  try_declare.template operator()<double>("pcd_save.filter_size_pcd", 0.5);
  try_declare.template operator()<int>("pcd_save.point_stride", 1);
  try_declare.template operator()<vector<double>>("extrin_calib.extrinsic_T", vector<double>{});
  try_declare.template operator()<vector<double>>("extrin_calib.extrinsic_R", vector<double>{});
  try_declare.template operator()<vector<double>>("extrin_calib.Pcl", vector<double>{});
  try_declare.template operator()<vector<double>>("extrin_calib.Rcl", vector<double>{});
  try_declare.template operator()<vector<double>>("extrin_calib.Pcl2", vector<double>{});
  try_declare.template operator()<vector<double>>("extrin_calib.Rcl2", vector<double>{});
  try_declare.template operator()<double>("debug.plot_time", -10);
  try_declare.template operator()<int>("debug.frame_cnt", 6);

  try_declare.template operator()<double>("publish.blind_rgb_points", 0.01);
  try_declare.template operator()<int>("publish.pub_scan_num", 1);
  try_declare.template operator()<bool>("publish.pub_effect_point_en", false);
  try_declare.template operator()<bool>("publish.dense_map_en", false);

  // get parameter
  this->node->get_parameter("common.lid_topic", lid_topic);
  this->node->get_parameter("common.imu_topic", imu_topic);
  this->node->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->node->get_parameter("common.lossless_replay_qos", lossless_replay_qos);
  this->node->get_parameter("common.img_en", img_en);
  this->node->get_parameter("common.img_compressed_en", img_compressed_en);
  this->node->get_parameter("common.img_rate_hz", img_rate_hz);
  this->node->get_parameter("common.img_buffer_max", img_buffer_max);
  this->node->get_parameter("common.lidar_en", lidar_en);
  this->node->get_parameter("common.img_topic", img_topic);
  this->node->get_parameter("common.img2_topic", img2_topic_);

  this->node->get_parameter("vio.normal_en", normal_en);
  this->node->get_parameter("vio.inverse_composition_en", inverse_composition_en);
  this->node->get_parameter("vio.max_iterations", max_iterations);
  this->node->get_parameter("vio.img_point_cov", IMG_POINT_COV);
  this->node->get_parameter("vio.raycast_en", raycast_en);
  this->node->get_parameter("vio.exposure_estimate_en", exposure_estimate_en);
  this->node->get_parameter("vio.inv_expo_cov", inv_expo_cov);
  this->node->get_parameter("vio.grid_size", grid_size);
  this->node->get_parameter("vio.grid_n_height", grid_n_height);
  this->node->get_parameter("vio.patch_pyrimid_level", patch_pyrimid_level);
  this->node->get_parameter("vio.patch_size", patch_size);
  this->node->get_parameter("vio.outlier_threshold", outlier_threshold);
  this->node->get_parameter("time_offset.exposure_time_init", exposure_time_init);
  this->node->get_parameter("time_offset.img_time_offset", img_time_offset);
  this->node->get_parameter("common.startup_warmup_s", startup_warmup_s_);
  this->node->get_parameter("common.sync_lookahead_s", sync_lookahead_s_);
  this->node->get_parameter("common.direct_bag_path", direct_bag_path_);
  {
    bool right_invariant_en = false;
    this->node->get_parameter("state.right_invariant_en", right_invariant_en);
    g_right_invariant_en = right_invariant_en;
  }
  this->node->get_parameter("lio.map_insert_every", map_insert_every_);
  this->node->get_parameter("lio.lag_smooth", lag_smooth_);
  this->node->get_parameter("lio.lag_iters", lag_iters_);
  this->node->get_parameter("lio.lag_odom_weight", lag_odom_weight_);
  this->node->get_parameter("common.velocity_init_en", velocity_init_en_);
  this->node->get_parameter("common.dump_attitude_en", dump_attitude_en_);
  this->node->get_parameter("common.zupt_en", zupt_en_);
  this->node->get_parameter("common.zupt_vel_thresh", zupt_vel_thresh_);
  this->node->get_parameter("common.zupt_hold_s", zupt_hold_s_);
  this->node->get_parameter("uav.imu_rate_odom", imu_prop_enable);
  this->node->get_parameter("uav.gravity_align_en", gravity_align_en);

  // SA-LIVO information-form VIO (Sect. VI)
  this->node->get_parameter("sa_vio.window_size", sa_vio_cfg.window_size);
  this->node->get_parameter("sa_vio.patch_spacing", sa_vio_cfg.patch_spacing);
  this->node->get_parameter("sa_vio.max_obs_per_frame", sa_vio_cfg.max_obs_per_frame);
  this->node->get_parameter("sa_vio.min_obs", sa_vio_cfg.min_obs);
  this->node->get_parameter("sa_vio.depth_min", sa_vio_cfg.depth_min);
  this->node->get_parameter("sa_vio.theta_max_deg", sa_vio_cfg.theta_max_deg);
  this->node->get_parameter("sa_vio.parallax_min_px", sa_vio_cfg.parallax_min_px);
  this->node->get_parameter("sa_vio.abs_res_max", sa_vio_cfg.abs_res_max);
  this->node->get_parameter("sa_vio.chi2_thres", sa_vio_cfg.chi2_thres);
  this->node->get_parameter("sa_vio.delta_rms", sa_vio_cfg.delta_rms);
  this->node->get_parameter("sa_vio.sigma_rms", sa_vio_cfg.sigma_rms);
  this->node->get_parameter("sa_vio.rho_min", sa_vio_cfg.rho_min);
  this->node->get_parameter("sa_vio.rho_max", sa_vio_cfg.rho_max);
  this->node->get_parameter("sa_vio.delta_alpha", sa_vio_cfg.delta_alpha);
  this->node->get_parameter("sa_vio.sigma_px", sa_vio_cfg.sigma_px);
  this->node->get_parameter("sa_vio.photometric_huber_en", sa_vio_cfg.photometric_huber_en);
  this->node->get_parameter("sa_vio.patch_mean_noise_en", sa_vio_cfg.patch_mean_noise_en);
  this->node->get_parameter("sa_vio.adaptive_noise_en", sa_vio_cfg.adaptive_noise_en);
  this->node->get_parameter("sa_vio.adaptive_noise_min_sigma", sa_vio_cfg.adaptive_noise_min_sigma);
  this->node->get_parameter("sa_vio.info_scale", sa_vio_cfg.info_scale);
  this->node->get_parameter("sa_vio.lamv_max", sa_vio_cfg.lamv_max);
  this->node->get_parameter("sa_vio.visual_start_s", sa_vio_cfg.visual_start_s);
  this->node->get_parameter("sa_vio.point_image_cell_px", sa_vio_cfg.point_image_cell_px);
  this->node->get_parameter("sa_vio.point_select_image_grid", sa_vio_cfg.point_select_image_grid);
  this->node->get_parameter("sa_vio.affine_equal_weight", sa_vio_cfg.affine_equal_weight);
  this->node->get_parameter("sa_vio.patch_search_px", sa_vio_cfg.patch_search_px);
  this->node->get_parameter("sa_vio.rrms_after_gates", sa_vio_cfg.rrms_after_gates);
  this->node->get_parameter("sa_vio.debug_obs_gates", sa_vio_cfg.debug_obs_gates);
  this->node->get_parameter("sa_vio.sigma2_sensor_only", sa_vio_cfg.sigma2_sensor_only);
  this->node->get_parameter("sa_vio.visual_anchor_from_map", sa_vio_cfg.visual_anchor_from_map);
  this->node->get_parameter("sa_vio.sigma2_live_cov", sa_vio_cfg.sigma2_live_cov);
  this->node->get_parameter("sa_vio.dual_event_en", use_dual_event_);
  this->node->get_parameter("sa_vio.use_baseline_vio_frontend", use_baseline_vio_frontend_);
  this->node->get_parameter("sa_vio.baseline_vio_sequential", baseline_vio_sequential_);
  this->node->get_parameter("sa_vio.baseline_lio_state_estimation", baseline_lio_state_estimation_);
  this->node->get_parameter("sa_vio.seq_output_post_vio", seq_output_post_vio_);
  this->node->get_parameter("sa_vio.seq_color_pre_vio", seq_color_pre_vio_);
  this->node->get_parameter("sa_vio.seq_color_lag", seq_color_lag_);
  this->node->get_parameter("saif.degenerate_attitude_hold", degenerate_attitude_hold_);
  this->node->get_parameter("sa_vio.dup_R_deg", sa_vio_cfg.dup_R_deg);
  this->node->get_parameter("sa_vio.dup_t_m", sa_vio_cfg.dup_t_m);
  this->node->get_parameter("sa_vio.alpha_min", sa_vio_cfg.alpha_min);
  this->node->get_parameter("sa_vio.alpha_max", sa_vio_cfg.alpha_max);
  this->node->get_parameter("sa_vio.beta_min", sa_vio_cfg.beta_min);
  this->node->get_parameter("sa_vio.beta_max", sa_vio_cfg.beta_max);

  this->node->get_parameter("evo.seq_name", seq_name);
  this->node->get_parameter("evo.pose_output_en", pose_output_en);
  this->node->get_parameter("imu.gyr_cov", gyr_cov);
  this->node->get_parameter("imu.acc_cov", acc_cov);
  this->node->get_parameter("imu.imu_int_frame", imu_int_frame);
  {
    bool use_baseline_init = false;
    this->node->get_parameter("imu.use_baseline_init", use_baseline_init);
    p_imu->set_use_baseline_init(use_baseline_init);
  }
  this->node->get_parameter("imu.imu_en", imu_en);
  this->node->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->node->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->node->get_parameter("preprocess.blind", p_pre->blind);
  this->node->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->node->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
  this->node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->node->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->node->get_parameter("pcd_save.interval", pcd_save_interval);
  this->node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->node->get_parameter("pcd_save.color_delay_frames", color_delay_frames_);
  this->node->get_parameter("pcd_save.color_prev_mix", color_prev_mix_);
  this->node->get_parameter("pcd_save.color_blur_sigma", color_blur_sigma_);
  this->node->get_parameter("pcd_save.color_pose_shift_s", color_pose_shift_s_);
  this->node->get_parameter("pcd_save.color_baseline_pairing_en", color_baseline_pairing_en_);
  this->node->get_parameter("pcd_save.color_future_pose_en", color_future_pose_en_);
  this->node->get_parameter("pcd_save.color_fullres_en", color_fullres_en_);
  this->node->get_parameter("sa_ba.en", sa_ba_en_);
  this->node->get_parameter("sa_ba.window_size", sa_ba_window_);
  this->node->get_parameter("sa_ba.iters", sa_ba_iters_);
  this->node->get_parameter("sa_ba.visual_weight", sa_ba_visual_weight_);
  this->node->get_parameter("sa_ba.odom_weight", sa_ba_odom_weight_);
  this->node->get_parameter("sa_ba.anchor_weight", sa_ba_anchor_weight_);
  this->node->get_parameter("sa_ba.photometric_en", sa_ba_photometric_en_);
  this->node->get_parameter("sa_ba.output", sa_ba_output_);
  this->node->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->node->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->node->get_parameter("pcd_save.point_stride", pcd_point_stride);
  this->node->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->node->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->node->get_parameter("extrin_calib.Pcl", cameraextrinT);
  this->node->get_parameter("extrin_calib.Rcl", cameraextrinR);
  this->node->get_parameter("extrin_calib.Pcl2", cameraextrinT2);
  this->node->get_parameter("extrin_calib.Rcl2", cameraextrinR2);
  this->node->get_parameter("debug.plot_time", plot_time);
  this->node->get_parameter("debug.frame_cnt", frame_cnt);

  this->node->get_parameter("publish.blind_rgb_points", blind_rgb_points);
  this->node->get_parameter("publish.pub_scan_num", pub_scan_num);
  this->node->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->node->get_parameter("publish.dense_map_en", dense_map_en);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents(rclcpp::Node::SharedPtr &node) 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  
  // extrinT.assign({0.04165, 0.02326, -0.0284});
  // extrinR.assign({1, 0, 0, 0, 1, 0, 0, 0, 1});
  // cameraextrinT.assign({0.0194384, 0.104689,-0.0251952});
  // cameraextrinR.assign({0.00610193,-0.999863,-0.0154172,-0.00615449,0.0153796,-0.999863,0.999962,0.00619598,-0.0060598});

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromRosNs(this->node, "camera", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  // SA-LIVO information-form VIO module (Sect. VI)
  // Extrinsics follow the VIOManager convention:
  //   p_L = Rli * p_I + Pli  (IMU -> LiDAR)
  //   p_C = Rcl * p_L + Pcl  (LiDAR -> camera)
  {
    M3D Rli = extR.transpose();
    V3D Pli = -extR.transpose() * extT;
    M3D Rcl;
    Rcl << MAT_FROM_ARRAY(cameraextrinR);
    V3D Pcl;
    Pcl << VEC_FROM_ARRAY(cameraextrinT);
    sa_vio.reset(new SAVioManager());
    sa_vio->setConfig(sa_vio_cfg);
    sa_vio->setCamera(vio_manager->cam);
    sa_vio->setExtrinsics(Rli, Pli, Rcl, Pcl);
    sa_vio->reset();

    if (!img2_topic_.empty() && !cameraextrinR2.empty() && !cameraextrinT2.empty())
    {
      sa_vio2.reset(new SAVioManager());
      sa_vio2->setConfig(sa_vio_cfg);
      vk::AbstractCamera *cam2 = nullptr;
      if (!vk::camera_loader::loadFromRosNs(this->node, "camera2", cam2))
      {
        throw std::runtime_error("Second camera model not correctly specified (camera2).");
      }
      sa_vio2->setCamera(cam2);
      M3D Rcl2;
      Rcl2 << MAT_FROM_ARRAY(cameraextrinR2);
      V3D Pcl2;
      Pcl2 << VEC_FROM_ARRAY(cameraextrinT2);
      sa_vio2->setExtrinsics(Rli, Pli, Rcl2, Pcl2);
      sa_vio2->reset();
    }
  }

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_interval > 0) fout_pcd_pos.open(std::string(ROOT_DIR) + "Log/PCD/scans_pos.json", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &node, image_transport::ImageTransport &it_)
{
  image_transport::ImageTransport it(this->node);
  if (p_pre->lidar_type == AVIA) {
    sub_pcl = this->node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 200000, std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  } else {
    sub_pcl = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 200000, std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1));
  }
  sub_imu = this->node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  if (lossless_replay_qos)
  {
    // Deterministic merged-bag replay: publisher and subscriber are both
    // RELIABLE/KEEP_ALL so a busy joint update cannot drop queued frames.
    auto image_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().durability_volatile();
    if (img_compressed_en)
    {
      sub_img_comp = this->node->create_subscription<sensor_msgs::msg::CompressedImage>(
          img_topic, image_qos, std::bind(&LIVMapper::img_compressed_cbk, this, std::placeholders::_1));
    }
    else
    {
      sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(
          img_topic, image_qos, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));
    }
  }
  else
  {
    // Live camera and standard image_transport publishers are BEST_EFFORT;
    // stale live frames should be dropped rather than building a multi-GB
    // backlog. Lossless offline replay uses the branch above.
    auto image_qos = rclcpp::SensorDataQoS();
    if (img_compressed_en)
    {
      sub_img_comp = this->node->create_subscription<sensor_msgs::msg::CompressedImage>(
          img_topic, image_qos, std::bind(&LIVMapper::img_compressed_cbk, this, std::placeholders::_1));
    }
    else
    {
      sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(
          img_topic, image_qos, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));
    }
  }

  if (!img2_topic_.empty())
  {
    auto image_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().durability_volatile();
    sub_img2 = this->node->create_subscription<sensor_msgs::msg::Image>(
        img2_topic_, image_qos, std::bind(&LIVMapper::img2_cbk, this, std::placeholders::_1));
  }
  
  pubLaserCloudFullRes = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 100);
  pubSubVisualMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = this->node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubPath = this->node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  plane_pub = this->node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = this->node->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = this->node->create_wall_timer(0.004s, std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_= this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::applyDegenerateAttitudeHold()
{
  if (voxelmap_manager->degenerate_update_)
  {
    if (degenerate_attitude_hold_ && have_last_good_att_)
    {
      _state.rot_end = last_good_att_state_.rot_end;
      _state.gravity = last_good_att_state_.gravity;
      _state.bias_g = last_good_att_state_.bias_g;
      voxelmap_manager->state_.rot_end = _state.rot_end;
      voxelmap_manager->state_.gravity = _state.gravity;
      voxelmap_manager->state_.bias_g = _state.bias_g;
    }
  }
  else
  {
    last_good_att_state_ = _state;
    have_last_good_att_ = true;
  }
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;

  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ SA-LIVO ] No point!!!" << std::endl;
    return;
  }
    
  double t0 = omp_get_wtime();

  // ---- 1. downsample + world transform (LiDAR scan aligned at the camera
  //         capture time, undistorted by IMU backward propagation) -----------
  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;

  if (!lidar_map_inited)
  {
    // Start from an empty map. The common post-convergence UpdateVoxelMap()
    // inserts this first block exactly once; pre-inserting here would make the
    // block self-associate and double its capped Eq.9 statistics.
    lidar_map_inited = true;
  }

  const cv::Mat &img_cur = LidarMeasures.measures.back().img;
  // ---- 2. pre-loop visual information form (ΛV, bV, q) frozen across the
  //         joint InEKF iterations (Algorithm 2 step 1, Sect. VI) -----------
  InfoForm6 LambdaV;
  double q = 0.0;
  if (use_baseline_vio_frontend_ && !baseline_vio_sequential_)
  {
    // Diagnostic front-end: original FAST-LIVO2 VIOManager tracking/Jacobians,
    // accumulated into an information form at the propagated state x0 and
    // fused through SAIF.
    StatesGroup *saved_vio_state = vio_manager->state;
    vio_manager->state = &state_propagat;
    vio_manager->beginInfoForm();
    cv::Mat img_copy = img_cur;
    vio_manager->prepareVisualInfoForm(img_copy, _pv_list, voxelmap_manager->voxel_map_,
                                       LidarMeasures.measures.back().vio_time);
    vio_manager->state = saved_vio_state;
    LambdaV.Lambda = vio_manager->vioLambda();
    LambdaV.b = vio_manager->viob();
    LambdaV.Lambda *= sa_vio_cfg.info_scale;
    LambdaV.b *= sa_vio_cfg.info_scale;
    q = 1.0;
    vio_manager->endInfoForm();
    printf("[ SA-VIO-BASELINE ] |ΛV|=%.4e |bV|=%.4e\n", LambdaV.Lambda.norm(), LambdaV.b.norm());
    // VIOManager keeps the colour image resized to the camera-model
    // resolution (img_rgb); the raw full-res img_cur would be sampled with
    // scaled projection coordinates and offset every pixel by 2x.
    sa_vio->setDebugImage(vio_manager->img_rgb);
  }
  else
  {
    img2_cur_ = cv::Mat();
    const double vio_rel =
        first_lidar_header_time_ > 0.0
            ? (LidarMeasures.measures.back().vio_time - first_lidar_header_time_)
            : 0.0;
    const bool vision_on =
        (sa_vio_cfg.visual_start_s <= 0.0) || (vio_rel >= sa_vio_cfg.visual_start_s);
    if (vision_on)
    {
      sa_vio->buildVisualInfoForm(img_cur, _state, LambdaV, q);
      // Dual-camera visual fusion (SA extension): sum the second camera's
      // frozen information form into ΛV and gate q by both cameras.
      if (sa_vio2 && popImg2AtTime(LidarMeasures.measures.back().vio_time, img2_cur_))
      {
        InfoForm6 LambdaV2;
        double q2 = 0.0;
        sa_vio2->buildVisualInfoForm(img2_cur_, _state, LambdaV2, q2);
        LambdaV.Lambda += LambdaV2.Lambda;
        LambdaV.b += LambdaV2.b;
        q *= q2;
      }
    }
  }

  if (color_fullres_en_ && !img_cur.empty())
    sa_vio->setDebugImage(img_cur.clone());

  // ---- 3. unified single-loop joint InEKF update (Algorithm 2, Sect. VII) --
  StatesGroup seq_pre_vio_state;
  StatesGroup joint_pre_state;
  bool have_seq_pre_vio_state = false;
  if (use_baseline_vio_frontend_ && baseline_vio_sequential_)
  {
    // Milestone-1 diagnostic: the preceding LIO event already applied the
    // LiDAR update at the camera time; here only the original FAST-LIVO2
    // VIOManager visual update runs (baseline LIVO ordering).
    state_propagat = _state;
    const StatesGroup state_before_vio = _state;
    seq_pre_vio_state = state_before_vio;
    have_seq_pre_vio_state = true;
    cv::Mat img_copy = img_cur;
    vio_manager->processFrame(img_copy, _pv_list, voxelmap_manager->voxel_map_,
                              LidarMeasures.measures.back().vio_time);
    if (pose_output_en)
    {
      // Default: baseline saves the LIO pose at the camera time (before the
      // VIO correction); reproduce that semantics so the comparison is
      // apples-to-apples when the baseline trajectory is used as GT.
      // Diagnostic: seq_output_post_vio=true saves the VIO-corrected state
      // at the camera time instead (SA-side A/B only).
      static bool pos_opened_seq = false;
      std::ofstream evoFile;
      if (!pos_opened_seq)
      {
        evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
        pos_opened_seq = true;
      }
      else
      {
        evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      }
      const StatesGroup &out_state = seq_output_post_vio_ ? _state : state_before_vio;
      const double out_time = seq_output_post_vio_
                                  ? LidarMeasures.measures.back().vio_time
                                  : LidarMeasures.last_lio_update_time;
      Eigen::Quaterniond q_(out_state.rot_end);
      evoFile << std::fixed;
      evoFile << out_time << " " << out_state.pos_end[0] << " " << out_state.pos_end[1] << " " << out_state.pos_end[2] << " "
              << q_.x() << " " << q_.y() << " " << q_.z() << " " << q_.w() << std::endl;
    }
    sa_vio->setDebugImage(vio_manager->img_rgb);
  }
  else
  {
    joint_pre_state = state_propagat;
    if (dump_attitude_en_ && !att_dump_opened_)
    {
      att_dump_.open(std::string(ROOT_DIR) + "Log/att_dump_" + seq_name + ".txt");
      att_dump_opened_ = true;
    }
    const V3D att_prop0 = dump_attitude_en_ ? RotMtoEuler(state_propagat.rot_end) : V3D::Zero();
    voxelmap_manager->JointStateEstimation(state_propagat, &LambdaV, q);
    _state = voxelmap_manager->state_;
    _pv_list = voxelmap_manager->pv_list_;
    applyDegenerateAttitudeHold();
    if (dump_attitude_en_)
    {
      const V3D att_corr0 = RotMtoEuler(_state.rot_end);
      att_dump_ << std::fixed << std::setprecision(6)
                << LidarMeasures.measures.back().vio_time - first_lidar_header_time_ << " "
                << (att_prop0 * 57.2957795).transpose() << " "
                << (att_corr0 * 57.2957795).transpose() << " "
                << ((att_corr0 - att_prop0) * 57.2957795).transpose() << std::endl;
    }
    if (sa_ba_en_ && sa_ba_output_ == "color")
    {
      baAppendFrame(LambdaV, q);
    }

    // One-time velocity bootstrap: the filter starts at zero velocity while the
    // platform is already walking; seed the velocity from the displacement of
    // the first two LiDAR-registered poses to remove the startup ramp bias.
    if (velocity_init_en_ && !velocity_init_done_)
    {
      const double t = LidarMeasures.measures.back().vio_time;
      if (vinit_t0_ < 0.0)
      {
        vinit_t0_ = t;
        vinit_p0_ = _state.pos_end;
      }
      else if (vinit_t1_ < 0.0 && t - vinit_t0_ > 0.05)
      {
        vinit_t1_ = t;
        vinit_p1_ = _state.pos_end;
        const double dt = vinit_t1_ - vinit_t0_;
        if (dt > 1e-3)
        {
          _state.vel_end = (vinit_p1_ - vinit_p0_) / dt;
          _state.cov.block<3, 3>(6, 6) =
              Eigen::Matrix3d::Identity() * (0.1 * 0.1);
          printf("[ INIT ] velocity seeded: %.3f %.3f %.3f m/s (dt=%.2f)\n",
                 _state.vel_end[0], _state.vel_end[1], _state.vel_end[2], dt);
        }
        velocity_init_done_ = true;
      }
    }

    // Zero-velocity update: when the estimated speed stays below threshold
    // for a hold window (platform stationary), damp the velocity toward zero
    // so the position does not explode during the stationary corridor end.
    if (zupt_en_)
    {
      const double v = _state.vel_end.norm();
      const double t = LidarMeasures.measures.back().vio_time;
      if (v < zupt_vel_thresh_)
      {
        if (zupt_since_ < 0.0) zupt_since_ = t;
        else if (t - zupt_since_ > zupt_hold_s_)
        {
          _state.vel_end *= 0.8;
          _state.cov.block<3, 3>(6, 6) *= 0.8;
        }
      }
      else
      {
        zupt_since_ = -1.0;
      }
    }
  }

  // Colour diagnostic: in sequential mode the VIO-corrected _state can be
  // noisier than the pre-VIO LIO pose (see seq_output_post_vio A/B). Keep the
  // colour projection on the same pose as the saved trajectory when enabled.
  StatesGroup lag_color_state;
  if (seq_color_lag_ && lag_smooth_ > 0 && !lag_poses_.empty())
  {
    lag_color_state.rot_end = lag_poses_.back().R;
    lag_color_state.pos_end = lag_poses_.back().p;
  }
  StatesGroup color_pose_shifted;
  const double t_now = LidarMeasures.last_lio_update_time;
  const bool use_shift = color_pose_shift_s_ != 0.0 && have_prev_color_pose_ &&
                         prev_color_t_ > 0.0 && t_now - prev_color_t_ > 1e-6;
  if (use_shift)
  {
    const double a = std::clamp(color_pose_shift_s_ / (t_now - prev_color_t_), 0.0, 1.0);
    const Sophus::SO3d R0(prev_color_R_), R1(_state.rot_end);
    const Sophus::SO3d dR(R0.inverse() * R1);
    color_pose_shifted.rot_end =
        (R0 * Sophus::SO3d::exp(a * dR.log())).matrix();
    color_pose_shifted.pos_end = prev_color_p_ + a * (_state.pos_end - prev_color_p_);
  }
  const StatesGroup &color_pose =
      use_shift ? color_pose_shifted :
      (seq_color_pre_vio_ && have_seq_pre_vio_state) ? seq_pre_vio_state :
      (seq_color_lag_ && lag_smooth_ > 0 && !lag_poses_.empty()) ? lag_color_state : _state;
  // Sliding-window BA colour refinement: use the window's latest refined pose
  // for both world geometry and camera projection (colour-only mode leaves
  // _state / output trajectory / map untouched).
  StatesGroup ba_color_pose;
  bool use_ba_color = sa_ba_en_ && ba_ok_ &&
                      static_cast<int>(ba_window_.size()) >= sa_ba_window_;
  if (use_ba_color)
  {
    ba_color_pose.rot_end = ba_window_.back().R;
    ba_color_pose.pos_end = ba_window_.back().p;
  }
  const StatesGroup &color_pose_final = use_ba_color ? ba_color_pose : color_pose;

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en && !(use_baseline_vio_frontend_ && baseline_vio_sequential_))
  {
    Eigen::Quaterniond q_(_state.rot_end);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << LidarMeasures.last_lio_update_time << " "
        << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
        << q_.x() << " " << q_.y() << " " << q_.z() << " " << q_.w();
    traj_lines_.push_back(oss.str());
    traj_times_.push_back(LidarMeasures.last_lio_update_time);
  }

  // ---- fixed-lag sliding-window smoothing (SA extension) -------------------
  if (lag_smooth_ > 0)
  {
    LagPose lp;
    lp.t = LidarMeasures.last_lio_update_time;
    lp.R = _state.rot_end;
    lp.p = _state.pos_end;
    lp.pI.reserve(feats_down_size);
    lp.planes.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
      V3D p_b(feats_down_body->points[i].x, feats_down_body->points[i].y,
              feats_down_body->points[i].z);
      lp.pI.push_back(extR * p_b + extT);
      lp.planes.push_back(voxelmap_manager->plane_cache_[i]);
    }
    lag_poses_.push_back(lp);
    if (lag_poses_.size() >= 2)
    {
      const int n = static_cast<int>(lag_poses_.size());
      Sophus::SE3d T_prev(lag_poses_[n - 2].R, lag_poses_[n - 2].p);
      Sophus::SE3d T_cur(lag_poses_[n - 1].R, lag_poses_[n - 1].p);
      lag_rel0_.push_back(T_prev.inverse() * T_cur);
    }
    while (static_cast<int>(lag_poses_.size()) > lag_smooth_)
    {
      lag_poses_.pop_front();
      lag_rel0_.pop_front();
    }
    lagSmooth();
    // Update the buffered trajectory with the smoothed recent poses.
    const int W = static_cast<int>(lag_poses_.size());
    if (traj_lines_.size() >= static_cast<size_t>(W))
    {
      for (int i = 0; i < W; i++)
      {
        const LagPose &sp = lag_poses_[i];
        Eigen::Quaterniond qs(sp.R);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << sp.t << " " << sp.p[0] << " " << sp.p[1] << " " << sp.p[2] << " "
            << qs.x() << " " << qs.y() << " " << qs.z() << " " << qs.w();
        traj_lines_[traj_lines_.size() - W + i] = oss.str();
      }
    }
    writeTrajectory();
  }
  else if (pose_output_en && !(use_baseline_vio_frontend_ && baseline_vio_sequential_))
  {
    writeTrajectory();
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  // ---- 4. voxel map update with the refined pose ---------------------------
  const M3D R_map = (lag_smooth_ > 0 && !lag_poses_.empty()) ? lag_poses_.back().R : _state.rot_end;
  const V3D p_map = (lag_smooth_ > 0 && !lag_poses_.empty()) ? lag_poses_.back().p : _state.pos_end;
  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(R_map, p_map, feats_down_body, world_lidar);
  // The per-point lists are populated by the LIO solve (JointStateEstimation
  // / StateEstimation).  In sequential baseline-VIO mode the first image frame
  // can arrive before any LIO event (exp11 bag timing), leaving them empty;
  // guard the map update so the first frame is simply deferred to the next
  // LIO event instead of indexing out of bounds.
  const bool have_lio_lists =
      (voxelmap_manager->pv_list_.size() >= world_lidar->points.size()) &&
      (voxelmap_manager->cross_mat_list_.size() >= world_lidar->points.size()) &&
      (voxelmap_manager->body_cov_list_.size() >= world_lidar->points.size());
  if (have_lio_lists)
  {
    for (size_t i = 0; i < world_lidar->points.size(); i++)
    {
      voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
      M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
      const M3D body_cov = voxelmap_manager->body_cov_list_[i];
      const M3D Rwl = R_map * extR;
      const M3D sensor_cov = Rwl * body_cov * Rwl.transpose();
      const M3D full_cov = sensor_cov +
            R_map * point_crossmat * _state.cov.block<3, 3>(0, 0) * point_crossmat.transpose() * R_map.transpose() +
            _state.cov.block<3, 3>(3, 3);
      voxelmap_manager->pv_list_[i].var_nostate = sensor_cov;
      voxelmap_manager->pv_list_[i].var = full_cov;
    }
    map_insert_counter_++;
    const double insert_min_vel = voxelmap_manager->config_setting_.map_insert_min_vel_;
    const double vel_norm = _state.vel_end.norm();
    const bool vel_ok =
        (insert_min_vel <= 0.0) || (vel_norm >= insert_min_vel);
    if (vel_ok && (map_insert_every_ <= 1 || (map_insert_counter_ % map_insert_every_) == 0))
    {
      voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
    }
  }
  _pv_list = voxelmap_manager->pv_list_;
  if (use_baseline_vio_frontend_ && !baseline_vio_sequential_)
  {
    vio_manager->updateFrameState(_state);
    cv::Mat img_copy = img_cur;
    vio_manager->finalizeVisualFrame(img_copy, _pv_list, voxelmap_manager->voxel_map_);
  }

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }

  // ---- 5. advance the visual sliding window with the current frame ---------
  // Diagnostic: anchor visual observations on accumulated map planes (averaged
  // over scans) instead of the current frame's LiDAR points, which inherit the
  // per-frame pose bias.  Map planes are world-grid sampled to bound the count.
  std::vector<pointWithVar> map_anchor_pv;
  const std::vector<pointWithVar> *visual_pv = &_pv_list;
  if (sa_vio_cfg.visual_anchor_from_map)
  {
    std::vector<VoxelPlane> map_planes;
    voxelmap_manager->collectMapPlanes(map_planes);
    std::unordered_map<int64_t, uint8_t> seen_cell;
    const double grid = 0.8;
    for (const auto &pl : map_planes)
    {
      if (!pl.is_plane_) continue;
      const int64_t cx = static_cast<int64_t>(std::floor(pl.center_[0] / grid));
      const int64_t cy = static_cast<int64_t>(std::floor(pl.center_[1] / grid));
      const int64_t cz = static_cast<int64_t>(std::floor(pl.center_[2] / grid));
      const int64_t key = (cx * 73856093) ^ (cy * 19349663) ^ (cz * 83492791);
      if (seen_cell.count(key)) continue;
      seen_cell[key] = 1;
      pointWithVar pv;
      pv.point_w = pl.center_;
      pv.normal = pl.normal_;
      pv.var = pl.plane_var_.block<3, 3>(0, 0);
      pv.var_nostate = pv.var;
      pv.body_var = pv.var;
      map_anchor_pv.push_back(pv);
    }
    printf("[ SA-VIO-MAP ] visual_anchor_from_map: map_planes=%zu anchor_pts=%zu\n",
           map_planes.size(), map_anchor_pv.size());
    visual_pv = &map_anchor_pv;
  }
  sa_vio->advanceWindow(img_cur, _state, *visual_pv, LidarMeasures.measures.back().vio_time);
  sa_vio->printStatus();
  if (sa_vio2 && !img2_cur_.empty())
  {
    sa_vio2->advanceWindow(img2_cur_, _state, *visual_pv, LidarMeasures.measures.back().vio_time);
    sa_vio2->printStatus();
  }
  // Sliding-window BA runs after the visual window advanced so the SA window
  // frames are index-aligned with ba_window_ (v2 photometric residuals).
  if (sa_ba_en_ && static_cast<int>(ba_window_.size()) >= 2)
    ba_ok_ = baRun();
  else
    ba_ok_ = false;
  ++processed_vio_frames_;
  RCLCPP_INFO(this->node->get_logger(), "[ PIPELINE ] processed VIO frame: %zu time: %.9f",
              processed_vio_frames_, LidarMeasures.measures.back().vio_time);

  // ---- 6. publishing -------------------------------------------------------
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    const auto &point_l = laserCloudFullRes->points[i];
    const V3D p_i = extR * V3D(point_l.x, point_l.y, point_l.z) + extT;
    const V3D p_w = color_pose.rot_end * p_i + color_pose.pos_end;
    auto &po = laserCloudWorld->points[i];
    po.x = p_w(0); po.y = p_w(1); po.z = p_w(2); po.intensity = point_l.intensity;
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  // Exactly one colour attempt and one append for this immutable frame block.
  ColorFrameSnapshot color_frame;
  color_frame.lidar_cloud = laserCloudFullRes;
  color_frame.world_point_pose =
      use_ba_color ? color_pose_final :
      (color_baseline_pairing_en_ ? joint_pre_state : color_pose_final);
  color_frame.camera_projection_pose = color_pose_final;
  color_frame.image_time = LidarMeasures.measures.back().vio_time;
  color_frame.cloud_time = LidarMeasures.last_lio_update_time;
  if (color_delay_frames_ < 0)
  {
    // Opposite temporal alignment: colour the current LiDAR frame with the
    // previous camera image (equivalent to a -1 frame colour delay).
    const cv::Mat current_image = sa_vio->debugImage();
    if (prev_color_image_.empty())
    {
      color_frame.image = current_image.clone();
    }
    else if (color_prev_mix_ > 0.0)
    {
      cv::addWeighted(prev_color_image_, 1.0 - color_prev_mix_,
                      current_image, color_prev_mix_, 0.0, color_frame.image);
    }
    else
    {
      color_frame.image = prev_color_image_;
    }
    prev_color_image_ = current_image.clone();
  }
  else if (color_delay_frames_ > 0)
  {
    // Reference FAST-LIVO2 uses img_time_offset=+0.1 s, i.e. each LiDAR
    // frame is coloured with the following camera image. Delay the previous
    // frame until the next image arrives, keeping the estimator at 0 offset.
    if (pending_color_frame_.lidar_cloud)
    {
      pending_color_frame_.image = sa_vio->debugImage().clone();
      if (color_future_pose_en_)
        pending_color_frame_.camera_projection_pose = color_pose_final;
      colorizeAndSave(pending_color_frame_);
    }
    pending_color_frame_ = color_frame;
  }
  else
  {
    color_frame.image = sa_vio->debugImage().clone();
  }

  // BA-delayed colouring: queue the frame and colour it once the sliding
  // window has gathered future context (oldest refined pose in ba_window_).
  if (sa_ba_en_ && ba_ok_ && color_delay_frames_ <= 0)
  {
    // feats_undistort is mutated in place by the next event; deep-copy the
    // cloud so queued frames keep their own point set.
    color_frame.lidar_cloud =
        PointCloudXYZI::Ptr(new PointCloudXYZI(*color_frame.lidar_cloud));
    ba_color_queue_.push_back(color_frame);
    while (static_cast<int>(ba_color_queue_.size()) >= sa_ba_window_ &&
           static_cast<int>(ba_window_.size()) >= sa_ba_window_)
    {
      ColorFrameSnapshot f = ba_color_queue_.front();
      ba_color_queue_.pop_front();
      const BaFrame &ref = ba_window_.front();
      f.world_point_pose.rot_end = ref.R;
      f.world_point_pose.pos_end = ref.p;
      f.camera_projection_pose.rot_end = ref.R;
      f.camera_projection_pose.pos_end = ref.p;
      colorizeAndSave(f);
    }
  }
  else if (color_delay_frames_ <= 0)
  {
    colorizeAndSave(color_frame);
  }

  // Save the current (post-joint) pose for the next frame's colour-pose shift.
  prev_color_R_ = _state.rot_end;
  prev_color_p_ = _state.pos_end;
  prev_color_t_ = t_now;
  have_prev_color_pose_ = true;

  publish_frame_world(pubLaserCloudFullRes, sa_vio);
  publish_img_rgb(pubImage, sa_vio);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (omp_get_wtime() - t0) / frame_num;

  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                      SA-LIVO Joint Update Time            |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Total (incl. VIO pre-loop)", omp_get_wtime() - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited)
  {
    // Start from an empty map. The common post-convergence UpdateVoxelMap()
    // inserts this first block exactly once; pre-inserting here would make the
    // block self-associate and double its capped Eq.9 statistics.
    lidar_map_inited = true;
  }

  double t1 = omp_get_wtime();

  if (baseline_vio_sequential_ && baseline_lio_state_estimation_)
    voxelmap_manager->StateEstimation(state_propagat);  // original FAST-LIVO2 LIO
  else
  {
    if (dump_attitude_en_ && !att_dump_opened_)
    {
      att_dump_.open(std::string(ROOT_DIR) + "Log/att_dump_" + seq_name + ".txt");
      att_dump_opened_ = true;
    }
    const V3D att_prop1 = dump_attitude_en_ ? RotMtoEuler(state_propagat.rot_end) : V3D::Zero();
    voxelmap_manager->JointStateEstimation(state_propagat, nullptr, 0.0);  // SA-LIVO: LIO-only joint update (SAIF-gated)
    if (dump_attitude_en_)
    {
      const V3D att_corr1 = RotMtoEuler(voxelmap_manager->state_.rot_end);
      att_dump_ << std::fixed << std::setprecision(6)
                << LidarMeasures.last_lio_update_time - first_lidar_header_time_ << " "
                << (att_prop1 * 57.2957795).transpose() << " "
                << (att_corr1 * 57.2957795).transpose() << " "
                << ((att_corr1 - att_prop1) * 57.2957795).transpose() << std::endl;
    }
  }
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;
  applyDegenerateAttitudeHold();

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en && slam_mode_ != LIVO)
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    const M3D body_cov = voxelmap_manager->body_cov_list_[i];
    const M3D Rwl = _state.rot_end * extR;
    const M3D sensor_cov = Rwl * body_cov * Rwl.transpose();
    const M3D full_cov = sensor_cov +
          _state.rot_end * point_crossmat * _state.cov.block<3, 3>(0, 0) * point_crossmat.transpose() * _state.rot_end.transpose() +
          _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var_nostate = sensor_cov;
    voxelmap_manager->pv_list_[i].var = full_cov;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  // Do not colour an LIO frame with sa_vio->debugImage(): in LIVO mode that
  // image belongs to the previous VIO event, so image/pose/point timestamps
  // differ.  FAST-LIVO2 likewise colours only from handleVIO().  LIO-only
  // operation keeps the intensity-map save path below.

  if (!img_en) publish_frame_world(pubLaserCloudFullRes, sa_vio);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  // Flush any BA-delayed colour frames still in the queue at shutdown.
  if (sa_ba_en_)
  {
    while (!ba_color_queue_.empty())
    {
      ColorFrameSnapshot f = ba_color_queue_.front();
      ba_color_queue_.pop_front();
      colorizeAndSave(f);
    }
  }
  if (color_delay_frames_ > 0 && pending_color_frame_.lidar_cloud)
  {
    // Flush the last delayed frame with the final image (no next frame exists).
    pending_color_frame_.image = sa_vio->debugImage().clone();
    colorizeAndSave(pending_color_frame_);
    pending_color_frame_.lidar_cloud.reset();
  }
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    // File names carry the seq_name so different runs (LIVO vs LIO-only,
    // different offsets/ablation configs) do not overwrite each other.
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/PCD/" + seq_name + "_all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/PCD/" + seq_name + "_all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      // Write the raw (strided) cloud first so the deliverable exists even if
      // the voxel filter is slow on very large maps.
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);

      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run(rclcpp::Node::SharedPtr &node) 
{
  rclcpp::Rate rate(5000);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->node);
  std::thread direct_feeder;
  int tail_sync_fail = 0;
  if (!direct_bag_path_.empty())
  {
    RCLCPP_INFO(this->node->get_logger(), "[ DIRECT BAG ] in-process replay: %s", direct_bag_path_.c_str());
    direct_feeder = std::thread([this]() { feedDirectBag(); });
  }
  while (rclcpp::ok()) 
  {
    executor.spin_some();
    if (!sync_packages(LidarMeasures)) 
    {
      // Deterministic batch replay: once the feeder has finished and all
      // buffered data has been drained, exit the run loop so savePCD() runs.
      if (direct_done_)
      {
        size_t n_lid = 0, n_img = 0;
        {
          std::lock_guard<std::mutex> lk(mtx_buffer);
          n_lid = lid_raw_data_buffer.size();
          n_img = img_buffer.size();
        }
        // Once the feeder is done, an event needs both a LiDAR scan and an
        // image. If either stream is exhausted, no further LIO/VIO event can
        // occur; trailing data in the other stream (or leftover IMU) is
        // unprocessable and must not block the final save.
        if (n_lid == 0 || n_img == 0) break;
        // Feeder done but both streams still have data that cannot be paired
        // (e.g. trailing camera frames ahead of the last LiDAR scan). Give the
        // sync a short window, then finalise so PCD saving is not blocked.
        if (++tail_sync_fail >= 10000) break;  // ~2 s at 5000 Hz
      }
      else
      {
        tail_sync_fail = 0;
      }
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
    {
      // The immediately preceding LIO event already propagated the state and
      // undistorted the camera-time cloud. Keep that immutable cloud/state and
      // perform the one joint LiDAR-visual update now.
      stateEstimationAndMapping();
    }
    else
    {
      processImu();
      if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == LIO && !use_dual_event_)
        continue;  // single-joint mode: LIO event is preprocessing only
      stateEstimationAndMapping();
    }
  }
  if (direct_feeder.joinable()) direct_feeder.join();
  executor.remove_node(this->node);
  savePCD();
}

void LIVMapper::feedDirectBag()
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions so;
  so.uri = direct_bag_path_;
  so.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions co;
  co.input_serialization_format = rmw_get_serialization_format();
  co.output_serialization_format = rmw_get_serialization_format();
  reader.open(so, co);
  size_t n_imu = 0, n_lid = 0, n_img = 0;
  while (rclcpp::ok() && reader.has_next())
  {
    auto m = reader.read_next();
    if (m->topic_name == imu_topic && imu_en)
    {
      rclcpp::SerializedMessage sm(*m->serialized_data);
      auto msg = std::make_shared<sensor_msgs::msg::Imu>();
      rclcpp::Serialization<sensor_msgs::msg::Imu>().deserialize_message(&sm, msg.get());
      imu_cbk(msg);
      n_imu++;
    }
    else if (m->topic_name == lid_topic && lidar_en)
    {
      if (p_pre->lidar_type == AVIA)
      {
        rclcpp::SerializedMessage sm(*m->serialized_data);
        auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
        rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg>().deserialize_message(&sm, msg.get());
        livox_pcl_cbk(msg);
      }
      else
      {
        rclcpp::SerializedMessage sm(*m->serialized_data);
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2>().deserialize_message(&sm, msg.get());
        standard_pcl_cbk(msg);
      }
      n_lid++;
    }
    else if (m->topic_name == img_topic && img_en && img_compressed_en)
    {
      rclcpp::SerializedMessage sm(*m->serialized_data);
      auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      rclcpp::Serialization<sensor_msgs::msg::CompressedImage>().deserialize_message(&sm, msg.get());
      img_compressed_cbk(msg);
      n_img++;
    }
    else if (m->topic_name == img_topic && img_en)
    {
      rclcpp::SerializedMessage sm(*m->serialized_data);
      auto msg = std::make_shared<sensor_msgs::msg::Image>();
      rclcpp::Serialization<sensor_msgs::msg::Image>().deserialize_message(&sm, msg.get());
      img_cbk(msg);
      n_img++;
    }
    else if (!img2_topic_.empty() && m->topic_name == img2_topic_ && img_en)
    {
      rclcpp::SerializedMessage sm(*m->serialized_data);
      auto msg = std::make_shared<sensor_msgs::msg::Image>();
      rclcpp::Serialization<sensor_msgs::msg::Image>().deserialize_message(&sm, msg.get());
      img2_cbk(msg);
      n_img++;
    }
  }
  RCLCPP_INFO(this->node->get_logger(),
              "[ DIRECT BAG ] finished: imu=%zu lidar=%zu img=%zu", n_imu, n_lid, n_img);
  direct_done_ = true;
}

void LIVMapper::writeTrajectory()
{
  if (!pose_output_en || traj_lines_.empty()) return;
  std::ofstream f(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
  if (!f.is_open()) return;
  for (const auto &line : traj_lines_) f << line << "\n";
}

void LIVMapper::lagSmooth()
{
  const int W = static_cast<int>(lag_poses_.size());
  if (W < 2 || lag_smooth_ <= 0 || lag_iters_ <= 0) return;
  const int dim = 6 * W;
  const double eps = 1e-6;

  for (int it = 0; it < lag_iters_; it++)
  {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(dim);

    // LiDAR point-to-plane residuals against the shared map planes.
    for (int i = 0; i < W; i++)
    {
      LagPose &lp = lag_poses_[i];
      for (size_t j = 0; j < lp.pI.size(); j++)
      {
        if (!lp.planes[j].valid) continue;
        const V3D &n = lp.planes[j].normal;
        const double d = lp.planes[j].d;
        const double sig = std::max(lp.planes[j].sigma2, 1e-6);
        const V3D A = lp.R * lp.pI[j];
        const double r0 = n.dot(A + lp.p) + d;
        Eigen::Matrix<double, 1, 6> J;
        for (int k = 0; k < 3; k++)
        {
          V3D e = V3D::Zero();
          e[k] = 1.0;
          V3D A2 = (lp.R * Exp(e, eps)) * lp.pI[j];
          J(0, k) = ((n.dot(A2 + lp.p) + d) - r0) / eps;
          V3D p2 = lp.p + eps * e;
          J(0, k + 3) = ((n.dot(A + p2) + d) - r0) / eps;
        }
        const double w = 1.0 / sig;
        H.block<6, 6>(6 * i, 6 * i) += w * J.transpose() * J;
        g.segment<6>(6 * i) += w * J.transpose() * r0;
      }
    }

    // Odometry priors: keep relative transforms at the filter's original values.
    for (int i = 0; i + 1 < W; i++)
    {
      const Sophus::SE3d rel0 = lag_rel0_[i];
      const double w = lag_odom_weight_;
      auto eof = [&](const M3D &Ra, const V3D &pa, const M3D &Rb, const V3D &pb)
      {
        Sophus::SE3d Ta(Ra, pa);
        Sophus::SE3d Tb(Rb, pb);
        return (Ta.inverse() * Tb * rel0.inverse()).log();
      };
      const M3D &Ra = lag_poses_[i].R;
      const V3D &pa = lag_poses_[i].p;
      const M3D &Rb = lag_poses_[i + 1].R;
      const V3D &pb = lag_poses_[i + 1].p;
      Eigen::Matrix<double, 6, 1> e0 = eof(Ra, pa, Rb, pb);
      Eigen::Matrix<double, 6, 12> J = Eigen::Matrix<double, 6, 12>::Zero();
      for (int k = 0; k < 6; k++)
      {
        V3D e = V3D::Zero();
        e[k] = 1.0;
        if (k < 3)
        {
          J.col(k) = (eof(Ra * Exp(e, eps), pa, Rb, pb) - e0) / eps;
          J.col(6 + k) = (eof(Ra, pa, Rb * Exp(e, eps), pb) - e0) / eps;
        }
        else
        {
          J.col(k) = (eof(Ra, pa + eps * e, Rb, pb) - e0) / eps;
          J.col(6 + k) = (eof(Ra, pa, Rb, pb + eps * e) - e0) / eps;
        }
      }
      H.block<12, 12>(6 * i, 6 * i) += w * J.transpose() * J;
      g.segment<12>(6 * i) += w * J.transpose() * e0;
    }

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    if (ldlt.info() != Eigen::Success) break;
    const Eigen::VectorXd dx = -ldlt.solve(g);
    if (!dx.allFinite()) break;
    for (int i = 0; i < W; i++)
    {
      const Eigen::Matrix<double, 6, 1> d = dx.segment<6>(6 * i);
      lag_poses_[i].R = lag_poses_[i].R * Exp(V3D(d.head<3>()));
      lag_poses_[i].p += d.tail<3>();
    }
  }
}

void LIVMapper::baAppendFrame(const InfoForm6 &info, double q)
{
  if (feats_down_size < 8) return;
  BaFrame f;
  f.t = LidarMeasures.last_lio_update_time;
  f.R0 = _state.rot_end;
  f.p0 = _state.pos_end;
  f.R = f.R0;
  f.p = f.p0;
  f.pI.reserve(feats_down_size);
  f.planes.reserve(feats_down_size);
  for (int i = 0; i < feats_down_size; i++)
  {
    V3D p_b(feats_down_body->points[i].x, feats_down_body->points[i].y,
            feats_down_body->points[i].z);
    f.pI.push_back(extR * p_b + extT);
    f.planes.push_back(voxelmap_manager->plane_cache_[i]);
  }
  f.info = info;
  f.q = q;
  f.has_vis = (q > 0.0);
  if (!ba_window_.empty())
  {
    const BaFrame &prev = ba_window_.back();
    Sophus::SE3d Ta(prev.R0, prev.p0);
    Sophus::SE3d Tb(f.R0, f.p0);
    ba_rel0_.push_back(Ta.inverse() * Tb);
  }
  ba_window_.push_back(std::move(f));
  if (static_cast<int>(ba_window_.size()) > sa_ba_window_)
  {
    ba_window_.pop_front();
    ba_rel0_.pop_front();
  }
}

bool LIVMapper::baRun()
{
  const int W = static_cast<int>(ba_window_.size());
  if (W < 2 || sa_ba_iters_ <= 0) return false;
  const int dim = 6 * W;
  const double eps = 1e-6;
  const double tau_L = voxelmap_manager->config_setting_.chi2_thres_;
  const double anchor_w = std::max(sa_ba_anchor_weight_, 0.0);

  for (int it = 0; it < sa_ba_iters_; it++)
  {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(dim);

    // (1) LiDAR point-to-plane residuals against the cached map planes.
    for (int i = 0; i < W; i++)
    {
      BaFrame &f = ba_window_[i];
      for (size_t j = 0; j < f.pI.size(); j++)
      {
        if (!f.planes[j].valid) continue;
        const V3D &n = f.planes[j].normal;
        const double d = f.planes[j].d;
        const double sig = std::max(f.planes[j].sigma2, 1e-6);
        const V3D A = f.R * f.pI[j];
        const double r0 = n.dot(A + f.p) + d;
        if (r0 * r0 / sig > tau_L) continue;  // χ² gate
        Eigen::Matrix<double, 1, 6> J;
        for (int k = 0; k < 3; k++)
        {
          V3D e = V3D::Zero();
          e[k] = 1.0;
          V3D A2 = (f.R * Exp(e, eps)) * f.pI[j];
          J(0, k) = ((n.dot(A2 + f.p) + d) - r0) / eps;
          V3D p2 = f.p + eps * e;
          J(0, k + 3) = ((n.dot(A + p2) + d) - r0) / eps;
        }
        const double w = 1.0 / sig;
        H.block<6, 6>(6 * i, 6 * i) += w * J.transpose() * J;
        g.segment<6>(6 * i) += w * J.transpose() * r0;
      }
    }

    // (2) Odometry priors: keep relative transforms at the filter values.
    for (int i = 0; i + 1 < W; i++)
    {
      const Sophus::SE3d rel0 = ba_rel0_[i];
      const double w = sa_ba_odom_weight_;
      auto eof = [&](const M3D &Ra, const V3D &pa, const M3D &Rb, const V3D &pb)
      {
        Sophus::SE3d Ta(Ra, pa);
        Sophus::SE3d Tb(Rb, pb);
        return (Ta.inverse() * Tb * rel0.inverse()).log();
      };
      const M3D &Ra = ba_window_[i].R;
      const V3D &pa = ba_window_[i].p;
      const M3D &Rb = ba_window_[i + 1].R;
      const V3D &pb = ba_window_[i + 1].p;
      Eigen::Matrix<double, 6, 1> e0 = eof(Ra, pa, Rb, pb);
      Eigen::Matrix<double, 6, 12> J = Eigen::Matrix<double, 6, 12>::Zero();
      for (int k = 0; k < 6; k++)
      {
        V3D e = V3D::Zero();
        e[k] = 1.0;
        if (k < 3)
        {
          J.col(k) = (eof(Ra * Exp(e, eps), pa, Rb, pb) - e0) / eps;
          J.col(6 + k) = (eof(Ra, pa, Rb * Exp(e, eps), pb) - e0) / eps;
        }
        else
        {
          J.col(k) = (eof(Ra, pa + eps * e, Rb, pb) - e0) / eps;
          J.col(6 + k) = (eof(Ra, pa, Rb, pb + eps * e) - e0) / eps;
        }
      }
      H.block<12, 12>(6 * i, 6 * i) += w * J.transpose() * J;
      g.segment<12>(6 * i) += w * J.transpose() * e0;
    }

    // (3) Visual residuals: v1 frozen info-form priors, or v2 photometric
    // residuals re-linearised at the current BA pose.
    if (sa_ba_photometric_en_ && sa_vio &&
        sa_vio->windowSize() >= W)
    {
      for (int i = 0; i < W; i++)
      {
        StatesGroup x;
        x.rot_end = ba_window_[i].R;
        x.pos_end = ba_window_[i].p;
        std::vector<SAVioManager::PhotoResid> res;
        sa_vio->collectFramePhotometric(sa_vio->window()[static_cast<size_t>(i)], x, res);
        for (const auto &pr : res)
        {
          H.block<6, 6>(6 * i, 6 * i) +=
              sa_ba_visual_weight_ * pr.w * pr.J.transpose() * pr.J;
          g.segment<6>(6 * i) +=
              sa_ba_visual_weight_ * pr.w * pr.J.transpose() * pr.r;
        }
      }
    }
    else
    {
      for (int i = 0; i < W; i++)
      {
        const BaFrame &f = ba_window_[i];
        if (!f.has_vis) continue;
        const double w = sa_ba_visual_weight_ * f.q;
        if (!(w > 0.0)) continue;
        H.block<6, 6>(6 * i, 6 * i) += w * f.info.Lambda;
        g.segment<6>(6 * i) -= w * f.info.b;
      }
    }

    // (4) First-frame anchor (anti-drift gauge fixation).
    if (anchor_w > 0.0)
      H.block<6, 6>(0, 0) += anchor_w * Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    if (ldlt.info() != Eigen::Success) return false;
    const Eigen::VectorXd dx = -ldlt.solve(g);
    if (!dx.allFinite()) return false;
    double dmax = 0.0;
    for (int i = 0; i < W; i++)
    {
      const Eigen::Matrix<double, 6, 1> d = dx.segment<6>(6 * i);
      ba_window_[i].R = ba_window_[i].R * Exp(V3D(d.head<3>()));
      ba_window_[i].p += d.tail<3>();
      dmax = std::max(dmax, std::max(d.head<3>().norm(), d.tail<3>().norm()));
    }
    if (dmax < 1e-4) break;
  }
  return true;
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制 propagate 频率和 IMU 频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = stamp2Sec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = stamp2Sec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  if (first_lidar_header_time_ < 0.0) first_lidar_header_time_ = stamp2Sec(msg->header.stamp);
  // cout<<"got feature"<<endl;
  if (stamp2Sec(msg->header.stamp) < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(),"lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(stamp2Sec(msg->header.stamp));
  last_timestamp_lidar = stamp2Sec(msg->header.stamp);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  if (first_lidar_header_time_ < 0.0) first_lidar_header_time_ = stamp2Sec(msg_in->header.stamp);
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stamp2Sec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stamp2Sec(msg->header.stamp);
    RCLCPP_INFO(this->node->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stamp2Sec(msg->header.stamp);
  RCLCPP_INFO(this->node->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  RCLCPP_INFO(this->node->get_logger(), "get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;
  // ros_driver_fix_en applies an integer-second correction derived from the
  // first LiDAR stamp. Do not mix uncorrected pre-LiDAR IMUs with the corrected
  // sequence. In the normal (no-hack) path, retain every IMU regardless of
  // cross-topic callback order.
  if (ros_driver_fix_en && last_timestamp_lidar < 0.0) return;

  RCLCPP_INFO(this->node->get_logger(), "get imu at time: %.6f", stamp2Sec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  const bool have_lidar_stamp = last_timestamp_lidar >= 0.0;
  if (have_lidar_stamp && fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(this->node->get_logger(), "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en && have_lidar_stamp) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = sec2Stamp(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->node->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // Lossless replay: do not drop IMUs behind a >0.2 s gap guard. When the
  // node briefly falls behind, the guard silently drops a batch of IMUs,
  // changing propagation and making identical replays diverge. The loop-back
  // check above still rejects out-of-order data.

  last_timestamp_imu = timestamp;

  p_imu->cache_init_imu(msg);
  imu_buffer.push_back(msg);
  cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_compressed_cbk(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_in)
{
  if (!img_en) return;
  cv::Mat dec = cv::imdecode(cv::Mat(msg_in->data), cv::IMREAD_COLOR);
  if (dec.empty()) return;
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->header = msg_in->header;
  msg->height = static_cast<uint32_t>(dec.rows);
  msg->width = static_cast<uint32_t>(dec.cols);
  msg->encoding = "bgr8";
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(dec.cols * 3);
  msg->data.resize(static_cast<size_t>(dec.total()) * 3);
  std::memcpy(msg->data.data(), dec.data, msg->data.size());
  img_cbk(msg);
}

// static int i = 0;
void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->node->get_logger(), "img jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  // if (hilti_en)
  // {
  //   i++;
  //   if (i % 4 != 0) return;
  // }
  // double msg_header_time =  stamp2Sec(msg->header.stamp);
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  RCLCPP_INFO(this->node->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image loop back. \n");
    return;
  }

  // Paper-compliant image-rate subsampling (Oxford Spires cameras run ~20 Hz;
  // SA-LIVO/FAST-LIVO2 evaluate at 10 Hz to match the LiDAR scan rate).
  if (img_rate_hz > 0.0)
  {
    const double period = 1.0 / img_rate_hz;
    // Keep the first image of each 10 Hz slot.  This path is used for the
    // ~20 Hz Oxford Spires stream; the HILTI 40 Hz stream is sub-sampled to
    // 10 Hz data-side during ROS1->ROS2 conversion, so an already-10 Hz bag
    // must not be run with img_rate_hz set.
    if (last_img_kept_time_ >= 0.0 && msg_header_time - last_img_kept_time_ < period - 1e-4)
    {
      last_timestamp_img = msg_header_time;
      return;
    }
    last_img_kept_time_ = msg_header_time;
  }

  // Bounded backlog: in-process bag replay can decode frames much faster than
  // the joint update consumes them (1440x1080 Spires images ~5 MB each).  Keep
  // the feeder within img_buffer_max of the pipeline so memory stays bounded;
  // message order/content is unchanged, so replay remains deterministic.
  while (img_buffer_max > 0 && rclcpp::ok())
  {
    size_t n = 0;
    {
      std::lock_guard<std::mutex> lk(mtx_buffer);
      n = img_buffer.size();
    }
    if (n < static_cast<size_t>(img_buffer_max)) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  // Accept every non-duplicate image. The legacy <20 ms jump guard drops
  // 40 Hz frames when DDS delivers two images in a burst, making replay
  // nondeterministic; only exact duplicates (<1 ms) are rejected above.

  cv::Mat img_cur = getImageFromMsg(msg);
  // cv_bridge::toCvShare aliases the message data buffer; the message is a
  // function-local SharedPtr here, so the aliased Mat would dangle as soon
  // as img_cbk returns. Deep-copy before queueing (ASAN-verified: SEGV in
  // cv::resize inside advanceWindow/buildVisualInfoForm on a dangling Mat).
  img_cur = img_cur.clone();
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  if (img_rate_hz > 0.0) last_img_kept_time_ = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::img2_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image(*msg_in));
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img2) < 0.001) return;
  if (msg_header_time < last_timestamp_img2)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image2 loop back. \n");
    return;
  }
  mtx_buffer.lock();
  cv::Mat img_cur = getImageFromMsg(msg);
  img_cur = img_cur.clone();
  img2_buffer.push_back(img_cur);
  img2_time_buffer.push_back(msg_header_time);
  last_timestamp_img2 = msg_header_time;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::popImg2AtTime(double t, cv::Mat &out)
{
  if (img2_buffer.empty()) return false;
  std::lock_guard<std::mutex> lk(mtx_buffer);
  while (!img2_time_buffer.empty() && img2_time_buffer.front() + 0.06 < t)
  {
    img2_buffer.pop_front();
    img2_time_buffer.pop_front();
  }
  if (img2_buffer.empty()) return false;
  if (std::fabs(img2_time_buffer.front() - t) > 0.04) return false;
  out = img2_buffer.front();
  img2_buffer.pop_front();
  img2_time_buffer.pop_front();
  return true;
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      // Deterministic startup: drop images before the configured warmup window
      // so the first LIO/VIO event always starts after the same number of
      // LiDAR scans have been buffered (fixes replay-order nondeterminism).
      if (startup_warmup_s_ > 0.0 && first_lidar_header_time_ >= 0.0 &&
          img_capture_time < first_lidar_header_time_ + startup_warmup_s_)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        return false;
      }
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        RCLCPP_ERROR(this->node->get_logger(), "[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      // Startup lookahead: during the first ~lookahead seconds the event waits
      // for a fixed data horizon so the first LIO/VIO event is independent of
      // DDS arrival timing. After that, stream normally (no end-of-bag loss).
      const bool startup_phase =
          meas.last_lio_update_time < first_lidar_header_time_ + sync_lookahead_s_ + 0.1;
      const double need = startup_phase ? sync_lookahead_s_ : 0.0;
      if (img_capture_time + need > lid_newest_time ||
          img_capture_time + need > imu_newest_time)
      {
        // RCLCPP_ERROR(this->node->get_logger(), "lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // stamp2Sec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // printf("[ Data Cut ] LIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = stamp2Sec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = stamp2Sec(imu_buffer.front()->header.stamp);
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   stamp2Sec(imu_buffer.front()->header.stamp));
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->node->get_logger(), "out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, SAVioManagerPtr sa_vio)
{
  cv::Mat img_rgb = sa_vio->debugImage();
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = this->node->get_clock()->now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::colorizeAndSave(const ColorFrameSnapshot &frame)
{
  if (!(pcd_save_en && img_en) || frame.image.empty() || !frame.lidar_cloud)
  {
    return;
  }
  cv::Mat color_image = frame.image;
  if (color_blur_sigma_ > 0.0)
  {
    cv::GaussianBlur(frame.image, color_image, cv::Size(0, 0),
                     color_blur_sigma_, color_blur_sigma_);
  }
  if (std::fabs(frame.image_time - frame.cloud_time) > 1e-4)
  {
    RCLCPP_WARN(this->node->get_logger(), "Skip colour frame: image/cloud mismatch %.6f ms",
                (frame.image_time - frame.cloud_time) * 1e3);
    return;
  }
  for (std::size_t i = 0; i < frame.lidar_cloud->size(); ++i)
  {
    if (pcd_point_stride > 1 && (i % static_cast<std::size_t>(pcd_point_stride)) != 0) continue;
    const auto &point_l = frame.lidar_cloud->points[i];
    const V3D p_l(point_l.x, point_l.y, point_l.z);
    const V3D p_i_at_world_pose = extR * p_l + extT;
    const V3D p_w = frame.world_point_pose.rot_end * p_i_at_world_pose +
                    frame.world_point_pose.pos_end;
    const V3D p_i_at_camera = frame.camera_projection_pose.rot_end.transpose() *
                              (p_w - frame.camera_projection_pose.pos_end);
    const V3D pf = sa_vio->Rci() * p_i_at_camera + sa_vio->Pci();
    if (pf.z() <= 0.0) continue;
    const V2D pc = sa_vio->cam()->world2cam(pf);
    if (!sa_vio->cam()->isInFrame(pc.cast<int>(), 3)) continue;
    const double color_scale =
        (color_fullres_en_ && sa_vio->cam()->width() > 0)
            ? static_cast<double>(color_image.cols) / sa_vio->cam()->width()
            : 1.0;
    V3F pixel = sa_vio->getInterpolatedPixelForViz(color_image, pc * color_scale);
    pcl::PointXYZRGB pr;
    pr.x = static_cast<float>(p_w.x());
    pr.y = static_cast<float>(p_w.y());
    pr.z = static_cast<float>(p_w.z());
    pr.r = pixel[0];
    pr.g = pixel[1];
    pr.b = pixel[2];
    pcl_wait_save->push_back(pr);
  }
}

void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, SAVioManagerPtr sa_vio)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  if (img_en)
  {
    static int pub_num = 1;
    const cv::Mat img_rgb = sa_vio->debugImage();
    PointCloudXYZRGB current_rgb;
    current_rgb.reserve(pcl_w_wait_pub->size());
    for (const auto &point : pcl_w_wait_pub->points)
    {
      const V3D p_w(point.x, point.y, point.z);
      const V3D p_i = _state.rot_end.transpose() * (p_w - _state.pos_end);
      const V3D pf = sa_vio->Rci() * p_i + sa_vio->Pci();
      if (pf.z() <= 0.0) continue;
      const V2D pc = sa_vio->cam()->world2cam(pf);
      if (sa_vio->cam()->isInFrame(pc.cast<int>(), 3) && pf.norm() > blind_rgb_points)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = point.x; pointRGB.y = point.y; pointRGB.z = point.z;
        const V3F pixel = sa_vio->getInterpolatedPixelForViz(img_rgb, pc);
        pointRGB.r = pixel[0]; pointRGB.g = pixel[1]; pointRGB.b = pixel[2];
        current_rgb.push_back(pointRGB);
      }
    }
    *pcl_rgb_wait_pub += current_rgb;
    if (pub_num >= std::max(pub_scan_num, 1))
    {
      *laserCloudWorldRGB = *pcl_rgb_wait_pub;
      PointCloudXYZRGB().swap(*pcl_rgb_wait_pub);
      pub_num = 1;
    }
    else
    {
      pub_num++;
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (img_en)
  {
    // cout << "RGB pointcloud size: " << laserCloudWorldRGB->size() << endl;
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  else 
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = this->node->get_clock()->now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    static int scan_wait_num = 0;

    if (!img_en)
    {
      *pcl_wait_save_intensity += *pcl_w_wait_pub;
    }
    scan_wait_num++;

    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "Log/PCD/") + to_string(pcd_index) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      if (pcd_save_en)
      {
        cout << "current scan saved to /PCD/" << all_points_dir << endl;
        if (img_en)
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save);
        }
        else
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
          PointCloudXYZI().swap(*pcl_wait_save_intensity);
        }        
        Eigen::Quaterniond q(_state.rot_end);
        fout_pcd_pos << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.w() << " " << q.x() << " " << q.y()
                     << " " << q.z() << " " << endl;
        scan_wait_num = 0;
      }
    }
  }
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = this->node->get_clock()->now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = this->node->get_clock()->now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = this->node->get_clock()->now(); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  br = std::make_shared<tf2_ros::TransformBroadcaster>(this->node);
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br->sendTransform(geometry_msgs::msg::TransformStamped(createTransformStamped(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped")));
  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = this->node->get_clock()->now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = this->node->get_clock()->now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
