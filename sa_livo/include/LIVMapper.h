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

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "sa_vio.h"
#include "preprocess.h"
#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <atomic>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vikit/camera_loader.h>
#include <sophus/se3.hpp>

struct ColorFrameSnapshot
{
  PointCloudXYZI::ConstPtr lidar_cloud;
  cv::Mat image;
  StatesGroup world_point_pose;
  StatesGroup camera_projection_pose;
  double image_time = 0.0;
  double cloud_time = 0.0;
};

class LIVMapper
{
public:
  LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LIVMapper();
  void initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &nh, image_transport::ImageTransport &it_);
  void initializeComponents(rclcpp::Node::SharedPtr &node);
  void initializeFiles();
  void run(rclcpp::Node::SharedPtr &node);
  void feedDirectBag();                    // deterministic in-process bag feeder
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback();
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
 
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in);
  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in);
  void img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in);
  void img_compressed_cbk(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_in);
  void img2_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in);
  bool popImg2AtTime(double t, cv::Mat &out);
  void publish_img_rgb(const image_transport::Publisher &pubImage, SAVioManagerPtr sa_vio);
  void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, SAVioManagerPtr sa_vio);
  void colorizeAndSave(const ColorFrameSnapshot &frame);
  void publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap);
  void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pmavros_pose_publisherubOdomAftMapped);
  void publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher);
  void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath);
  void readParameters(rclcpp::Node::SharedPtr &node);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double last_timestamp_img2 = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  int pcd_point_stride = 1;         // save every N-th colored point (deliverable decimation)
  double _first_lidar_time = 0.0;
  double first_lidar_header_time_ = -1.0;  // first raw LiDAR scan header time
  double startup_warmup_s_ = 0.0;          // deterministic warmup before first LIO/VIO event
  double sync_lookahead_s_ = 0.0;          // require this much LiDAR/IMU data beyond each image time
  std::string direct_bag_path_ = "";       // in-process bag replay (bypasses DDS)
  std::atomic<bool> direct_done_{false};   // direct feeder finished reading the bag
  bool velocity_init_en_ = false;          // seed velocity from first two poses
  bool velocity_init_done_ = false;
  double vinit_t0_ = -1.0, vinit_t1_ = -1.0;
  V3D vinit_p0_ = V3D::Zero(), vinit_p1_ = V3D::Zero();
  bool zupt_en_ = false;                   // zero-velocity update (stationary hold)
  double zupt_vel_thresh_ = 0.05;
  double zupt_hold_s_ = 1.0;
  double zupt_since_ = -1.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;
  int map_insert_every_ = 1;         // insert map every N frames (1=every frame)
  int map_insert_counter_ = 0;
  // ---- fixed-lag sliding-window smoothing (SA extension) ------------------
  struct LagPose
  {
    double t = 0.0;
    M3D R = M3D::Identity();
    V3D p = V3D::Zero();
    std::vector<V3D> pI;                              // IMU-frame points
    std::vector<VoxelMapManager::CachedPlane> planes; // per-point matched planes
  };

  // One fixed-lag BA window frame (colour-pose refinement, v1 info-form priors).
  struct BaFrame
  {
    double t = 0.0;
    M3D R0 = M3D::Identity();   // filter pose at append (anchor / rel0)
    V3D p0 = V3D::Zero();
    M3D R = M3D::Identity();    // current BA iterate (starts at R0)
    V3D p = V3D::Zero();
    std::vector<V3D> pI;                              // IMU-frame LiDAR points
    std::vector<VoxelMapManager::CachedPlane> planes; // cached map planes
    InfoForm6 info;                                   // visual info form (ΛV,bV)
    double q = 0.0;                                   // visual quality factor
    bool has_vis = false;
  };
  std::deque<LagPose> lag_poses_;
  std::deque<Sophus::SE3d> lag_rel0_;                 // filter odometry priors
  std::deque<BaFrame> ba_window_;
  std::deque<Sophus::SE3d> ba_rel0_;
  std::deque<ColorFrameSnapshot> ba_color_queue_;  // delayed colour frames (BA future context)
  bool ba_ok_ = false;
  std::vector<std::string> traj_lines_;
  std::vector<double> traj_times_;
  int lag_smooth_ = 0;    // window size (0 = off)
  int lag_iters_ = 3;
  double lag_odom_weight_ = 1.0;
  void lagSmooth();
  void baAppendFrame(const InfoForm6 &info, double q);
  bool baRun();
  void writeTrajectory();

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false;
  bool lossless_replay_qos = false;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer;
  sensor_msgs::msg::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::msg::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;
  bool dump_attitude_en_ = false;             // per-frame IMU-prop vs LIO-corrected roll/pitch dump
  std::ofstream att_dump_;
  bool att_dump_opened_ = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  int outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  bool img_compressed_en = false;   // Oxford Spires bags publish CompressedImage
  double img_rate_hz = 0.0;         // 0=off; >0 keeps one image per 1/rate s (paper: 10 Hz)
  double last_img_kept_time_ = -1.0;
  int img_buffer_max = 60;          // bounded backlog (feeder backpressure); 0=unbounded
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img_comp;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  deque<cv::Mat> img2_buffer;
  deque<double> img2_time_buffer;
  cv::Mat img2_cur_;                    // current cam2 frame (empty if unavailable)
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  vector<double> cameraextrinT2;
  vector<double> cameraextrinR2;
  int IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_rgb_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;
  std::size_t processed_vio_frames_ = 0;
  bool use_dual_event_ = true;  // LIO event also performs a LiDAR-only update
  bool use_baseline_vio_frontend_ = false;  // original VIOManager → SAIF info form
  bool baseline_vio_sequential_ = false;    // LIO then original VIOManager update
  bool baseline_lio_state_estimation_ = false; // use original StateEstimation for LIO
  bool seq_output_post_vio_ = false;        // diagnostic: output VIO-corrected pose
  bool seq_color_pre_vio_ = false;          // diagnostic: colour with pre-VIO pose
  bool seq_color_lag_ = false;              // diagnostic: colour with fixed-lag smoothed pose

  // Total-information-loss guard: hold the last good attitude/gravity/gyro
  // bias while position/velocity keep integrating, preventing the gravity
  // mis-projection runaway observed in corridor degeneracy (exp18).
  bool degenerate_attitude_hold_ = false;
  StatesGroup last_good_att_state_;
  bool have_last_good_att_ = false;
  void applyDegenerateAttitudeHold();

  int color_delay_frames_ = 1;  // color frame n with image n+1 (reference +0.1 s)
  double color_prev_mix_ = 0.0; // blend current image into previous when delay<0
  double color_blur_sigma_ = 0.0; // Gaussian sigma applied to the colour image
  double color_pose_shift_s_ = 0.0; // colour pose interpolated backward by this offset (0=off)
  bool color_baseline_pairing_en_ = false; // baseline pairing: world=pre-joint pose, camera=post-joint pose
  bool color_future_pose_en_ = false;      // delayed colour uses the newly-arrived future pose as camera pose
  bool color_fullres_en_ = false;          // sample the raw full-resolution colour image
  bool sa_ba_en_ = false;           // sliding-window BA colour refinement
  int sa_ba_window_ = 5;
  int sa_ba_iters_ = 5;
  double sa_ba_visual_weight_ = 1.0;
  double sa_ba_odom_weight_ = 1.0;
  double sa_ba_anchor_weight_ = 1e6;
  bool sa_ba_photometric_en_ = false;  // v2: re-linearized photometric residuals
  std::string sa_ba_output_ = "color";
  bool have_prev_color_pose_ = false;
  M3D prev_color_R_ = M3D::Identity();
  V3D prev_color_p_ = V3D::Zero();
  double prev_color_t_ = -1.0;
  ColorFrameSnapshot pending_color_frame_;
  cv::Mat prev_color_image_;

  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;
  // SA-LIVO: information-form direct photometric VIO module (Sect. VI)
  SAVioManagerPtr sa_vio;
  SAVioManagerPtr sa_vio2;              // optional second camera (dual-cam fusion)
  std::string img2_topic_ = "";         // second camera topic ("" = disabled)
  SAVioConfig sa_vio_cfg;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_pub;
  std::shared_ptr<rclcpp::SubscriptionBase> sub_pcl;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSubVisualMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;
  rclcpp::Node::SharedPtr node;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif
