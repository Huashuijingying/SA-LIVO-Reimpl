/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * sa_vio.h — LiDAR-anchored direct photometric VIO in information form
 *
 * SA-LIVO (arXiv:2606.25699) Sect. VI: the visual module assembles the
 * information form (ΛV, bV) ONCE per frame, before the joint InEKF loop:
 *   - sliding window of W past frames (each with its own observations of
 *     LiDAR-anchored map points, sparse 9-pixel cross patch, frozen at
 *     creation)
 *   - per-frame affine brightness model (αj, βj) estimated by gradient-
 *     weighted least squares against the current image (Eq. 20–21)
 *   - pre-loop frozen photometric Jacobians (Eq. 24), depth-conditioned
 *     noise (Eq. 25), per-observation decorrelation wdec = 1/nused (Eq. 26)
 *   - multi-gate filtering and scene-level quality factor q (Eq. 27–29)
 *
 * Observations are selected from the current downsampled LiDAR scan
 * (LiDAR-anchored map points, Sect. VI-A/B): one per 0.5 m cell by gradient
 * energy, with depth-continuity and view-angle filters.
 */

#ifndef SA_LIVO_SA_VIO_H
#define SA_LIVO_SA_VIO_H

#include <array>
#include <deque>

#include <opencv2/opencv.hpp>
#include <vikit/abstract_camera.h>

#include "common_lib.h"
#include "saif.h"

// Sparse cross-pattern size: 9 pixels (center + 4 axial + 4 diagonal)
static constexpr int SA_PATCH_NUM = 9;

// One observation of a LiDAR-anchored map point in one sliding-window frame
struct SAObservation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  V3D pos_w;                              // world position (frozen at creation)
  V3D normal_w;                           // world surface normal (frozen)
  V3D point_i;                            // IMU-frame position (for Σw)
  M3D sigma_b;                            // body-frame beam-noise covariance
  M3D sigma_w;                            // full world covariance frozen at creation
  V2D px_ref;                             // reference pixel (frozen)
  double depth_ref = 0.0;                 // camera-frame depth at creation
  std::array<float, SA_PATCH_NUM> ref_intensities;  // frozen 9-pixel cross
  double ref_grad_mag = 0.0;              // mean |∇Iref| at creation
  int nused = 1;                          // usage counter → wdec = 1/nused
  int frame_id = -1;                      // creating window frame id
};

// One sliding-window reference frame
struct SAWindowFrame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id = -1;
  double t = 0.0;
  M3D R_w;                                // world rotation at frame time
  V3D p_w;                                // world position at frame time
  cv::Mat img;                            // gray reference image (viz/debug)
  double alpha = 1.0, beta = 0.0;         // affine brightness model
  double wcam = 1.0;                      // affine confidence factor
  std::vector<SAObservation> obs;
};

// Configuration (all defaults follow the paper; see reproduction notes)
struct SAVioConfig
{
  int window_size = 5;              // W
  int patch_spacing = 8;            // sp → ρ = max(1, ⌊sp/2⌋ − 1)
  int max_obs_per_frame = 100;      // Nobs per-frame cap (stride downsampling)
  int min_obs = 30;                 // Nmin support threshold for qcnt
  double depth_min = 0.5;           // dmin (m)
  double theta_max_deg = 80.0;      // θmax view-angle gate
  double parallax_min_px = 0.5;     // δplx parallax gate (px)
  double abs_res_max = 15.0;        // δabs absolute residual gate (gray)
  double chi2_thres = 6.635;        // τV χ² innovation gate
  double delta_rms = 12.0;          // δrms for qrms (gray)
  double sigma_rms = 8.0;           // σrms for qrms (gray)
  double rho_min = 0.3;             // ρmin for qrej
  double rho_max = 0.7;             // ρmax for qrej
  double alpha_min = 0.2, alpha_max = 3.0;   // affine clamps
  double beta_min = -50.0, beta_max = 50.0;
  double delta_alpha = 0.2;         // δα → wcam = exp(−max(|log α| − δα, 0))
  double sigma_px = 1.0;            // per-pixel photometric noise floor
  bool photometric_huber_en = false; // paper Eq.30 uses wcam·wdec/σ², no Huber
  bool patch_mean_noise_en = true;   // residual is the mean of SA_PATCH_NUM
                                     // pixels: σpx² term / SA_PATCH_NUM
  bool adaptive_noise_en = false;   // empirical per-frame noise scale (σ²_eff=max(σ², s_emp²))
  double adaptive_noise_min_sigma = 1.0;  // floor for the empirical scale (px)
  double info_scale = 1.0;          // global visual information-form gain
  double lamv_max = 0.0;            // diagnostic cap on max ΛV eigenvalue (0=off)
  double visual_start_s = 0.0;      // LIO-only bootstrap before vision activates
  bool rrms_after_gates = false;    // q_rrms uses only observations passing gates
  bool sigma2_sensor_only = false;  // visual σ² uses sensor-only world covariance
  bool sigma2_live_cov = false;     // Eq.25 Σw rebuilt from current state cov (Eq.12)
  double dup_R_deg = 0.5;           // duplicate-frame suppression δR
  double dup_t_m = 0.05;            // duplicate-frame suppression δt
  double depth_cont_radius_px = 4.0;      // ρd depth-continuity radius
  double depth_cont_thresh_m = 0.5;       // δd depth-continuity threshold
  double point_grid_size = 0.5;           // 0.5 m selection grid
  double point_image_cell_px = 16.0;      // paper VI-B: one point per image cell
  bool point_select_image_grid = false;   // paper VI-B image-grid binning (HILTI)
  bool affine_equal_weight = false;       // equal-weight affine (low-texture robust)
  int patch_search_px = 0;                // local patch search radius (px); 0=off
  bool debug_obs_gates = false;           // print per-frame observation gate counts
  bool visual_anchor_from_map = false;    // visual points from accumulated map planes
                                          // (averaged over scans) instead of the
                                          // current frame's LiDAR points (diagnostic:
                                          // current-frame anchors inherit per-frame
                                          // pose bias)
};

class SAVioManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SAVioManager() = default;

  void setConfig(const SAVioConfig &cfg) { cfg_ = cfg; }
  const SAVioConfig &getConfig() const { return cfg_; }

  // Camera model + extrinsics (same convention as VIOManager):
  // p_L = Rli·p_I + Pli, p_C = Rcl·p_L + Pcl, therefore
  // p_C = Rci·p_I + Pci with Rci = Rcl·Rli, Pci = Rcl·Pli + Pcl.
  void setCamera(vk::AbstractCamera *cam) { cam_ = cam; }
  void setExtrinsics(const M3D &Rli, const V3D &Pli, const M3D &Rcl, const V3D &Pcl)
  {
    Rli_ = Rli;
    Pli_ = Pli;
    Rcl_ = Rcl;
    Pcl_ = Pcl;
    Rci_ = Rcl * Rli;
    Pci_ = Rcl * Pli + Pcl;
  }

  // Pre-loop visual information form (Sect. VI-D/G). Computed once at the
  // propagated state x0 and frozen across all joint InEKF iterations.
  // Returns false (ΛV = 0, q = 0) when the window is empty.
  bool buildVisualInfoForm(const cv::Mat &img, const StatesGroup &x0, InfoForm6 &out, double &q);

  // After the joint update: create the new window frame from the current
  // image and LiDAR-anchored points, evict the oldest frame.
  void advanceWindow(const cv::Mat &img, const StatesGroup &x,
                     const std::vector<pointWithVar> &pv_list, double frame_time);

  void reset() { window_.clear(); frame_counter_ = 0; }

  // Statistics / debug
  int numValidObs() const { return n_valid_; }
  int numRejectedObs() const { return n_rej_; }
  double lastQuality() const { return q_last_; }
  int windowSize() const { return static_cast<int>(window_.size()); }
  const std::deque<SAWindowFrame> &window() const { return window_; }

  // One photometric residual re-linearised at an arbitrary pose (BA v2).
  struct PhotoResid
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double r = 0.0;
    double w = 0.0;                               // 1/σ²
    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
  };
  void collectFramePhotometric(const SAWindowFrame &f, const StatesGroup &x,
                               std::vector<PhotoResid> &out) const;
  const cv::Mat &debugImage() const { return img_cp_; }
  void setDebugImage(const cv::Mat &img) { img_cp_ = img.clone(); }
  double lastRms() const { return rrms_last_; }
  void printStatus() const;

  // Accessors for visualization / RGB coloring
  vk::AbstractCamera *cam() const { return cam_; }
  const M3D &Rci() const { return Rci_; }
  const V3D &Pci() const { return Pci_; }
  const cv::Mat &currentGray() const { return img_gray_; }
  // Bilinear BGR sampling for colored map publishing (returns BGR as RGB order)
  V3F getInterpolatedPixelForViz(const cv::Mat &img, const V2D &pc) const;

private:
  void toGray(const cv::Mat &in, cv::Mat &out);
  double sampleBilinear(const cv::Mat &img, double x, double y) const;
  void sampleCross(const cv::Mat &img, const V2D &px, std::array<float, SA_PATCH_NUM> &vals) const;
  void computeGradientFields(const cv::Mat &img);
  void gradientAt(double x, double y, double &gx, double &gy) const;

  // Project a world point with state x → camera frame + pixel
  bool projectPoint(const StatesGroup &x, const V3D &pos_w, V3D &pc, V2D &px) const;
  void computeProjectionJacobian(const V3D &pc, MD(2, 3) &J) const;

  // Per-frame affine brightness estimation (Eq. 21), gradient-weighted LSQ
  bool estimateAffine(const cv::Mat &img_cur, const StatesGroup &x0, SAWindowFrame &f,
                      double &rrms_for_q) const;

  // Observation selection for a new window frame (Sect. VI-B)
  void selectObservations(const cv::Mat &img, const StatesGroup &x,
                          const std::vector<pointWithVar> &pv_list, std::vector<SAObservation> &new_obs);

  SAVioConfig cfg_;
  vk::AbstractCamera *cam_ = nullptr;
  M3D Rli_, Rcl_, Rci_;
  V3D Pli_, Pcl_, Pci_;

  std::deque<SAWindowFrame> window_;
  int frame_counter_ = 0;

  // Per-call temporaries
  cv::Mat img_cp_, img_gray_, gx_, gy_;
  int n_valid_ = 0, n_rej_ = 0;
  double res_mean_ = 0.0;   // running sum of accepted photometric residuals
  int res_cnt_ = 0;
  double q_last_ = 0.0, rrms_last_ = 0.0;

  // Depth image for continuity filter (per advanceWindow call)
  cv::Mat depth_img_;
};

typedef std::shared_ptr<SAVioManager> SAVioManagerPtr;

#endif  // SA_LIVO_SA_VIO_H
