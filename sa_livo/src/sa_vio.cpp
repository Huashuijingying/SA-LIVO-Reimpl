/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * sa_vio.cpp — LiDAR-anchored direct photometric VIO in information form.
 * See sa_vio.h for the design and conventions.
 */

#include "sa_vio.h"

#include <omp.h>
#include <cmath>
#include <map>
#include <unordered_map>

namespace
{
inline double clampVal(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

// ⌊v⌋_× skew-symmetric matrix
inline Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}
}  // namespace

// ---------------------------------------------------------------------------
// Image sampling helpers
// ---------------------------------------------------------------------------

void SAVioManager::toGray(const cv::Mat &in, cv::Mat &out)
{
  if (in.channels() == 3)
  {
    cv::cvtColor(in, out, cv::COLOR_BGR2GRAY);
  }
  else if (in.channels() == 1)
  {
    out = in;
  }
  else
  {
    out = in;
  }
}

double SAVioManager::sampleBilinear(const cv::Mat &img, double x, double y) const
{
  const int w = img.cols, h = img.rows;
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  if (x0 < 0 || y0 < 0 || x0 >= w - 1 || y0 >= h - 1) return 0.0;
  const double fx = x - x0, fy = y - y0;
  const uint8_t *row0 = img.ptr<uint8_t>(y0);
  const uint8_t *row1 = img.ptr<uint8_t>(y0 + 1);
  const double v00 = row0[x0], v01 = row0[x0 + 1];
  const double v10 = row1[x0], v11 = row1[x0 + 1];
  return (1 - fy) * ((1 - fx) * v00 + fx * v01) + fy * ((1 - fx) * v10 + fx * v11);
}

void SAVioManager::sampleCross(const cv::Mat &img, const V2D &px, std::array<float, SA_PATCH_NUM> &vals) const
{
  const double rho = static_cast<double>(std::max(1, static_cast<int>(std::floor(cfg_.patch_spacing / 2.0)) - 1));
  const double cx = px[0], cy = px[1];
  const double du[SA_PATCH_NUM] = {0, rho, -rho, 0, 0, rho, -rho, rho, -rho};
  const double dv[SA_PATCH_NUM] = {0, 0, 0, rho, -rho, rho, rho, -rho, -rho};
  for (int k = 0; k < SA_PATCH_NUM; k++)
  {
    vals[k] = static_cast<float>(sampleBilinear(img, cx + du[k], cy + dv[k]));
  }
}

void SAVioManager::computeGradientFields(const cv::Mat &img)
{
  gx_ = cv::Mat::zeros(img.size(), CV_32FC1);
  gy_ = cv::Mat::zeros(img.size(), CV_32FC1);
  const int w = img.cols, h = img.rows;
  for (int v = 1; v < h - 1; v++)
  {
    const uint8_t *row = img.ptr<uint8_t>(v);
    float *gxr = gx_.ptr<float>(v);
    float *gyr = gy_.ptr<float>(v);
    const uint8_t *row_up = img.ptr<uint8_t>(v - 1);
    const uint8_t *row_dn = img.ptr<uint8_t>(v + 1);
    for (int u = 1; u < w - 1; u++)
    {
      gxr[u] = 0.5f * (static_cast<float>(row[u + 1]) - static_cast<float>(row[u - 1]));
      gyr[u] = 0.5f * (static_cast<float>(row_dn[u]) - static_cast<float>(row_up[u]));
    }
  }
}

void SAVioManager::gradientAt(double x, double y, double &gx, double &gy) const
{
  // Exact derivative of sampleBilinear().  Interpolating a separately
  // central-differenced gradient image is not the derivative of bilinear
  // intensity sampling and produced large, direction-changing Jacobian errors
  // on the text edges in Bright_Screen_Wall.
  const int w = img_gray_.cols, h = img_gray_.rows;
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  if (x0 < 0 || y0 < 0 || x0 >= w - 1 || y0 >= h - 1)
  {
    gx = gy = 0.0;
    return;
  }
  const double fx = x - x0, fy = y - y0;
  const uint8_t *row0 = img_gray_.ptr<uint8_t>(y0);
  const uint8_t *row1 = img_gray_.ptr<uint8_t>(y0 + 1);
  const double i00 = row0[x0], i01 = row0[x0 + 1];
  const double i10 = row1[x0], i11 = row1[x0 + 1];
  gx = (1.0 - fy) * (i01 - i00) + fy * (i11 - i10);
  gy = (1.0 - fx) * (i10 - i00) + fx * (i11 - i01);
}

// ---------------------------------------------------------------------------
// Projection
// ---------------------------------------------------------------------------

bool SAVioManager::projectPoint(const StatesGroup &x, const V3D &pos_w, V3D &pc, V2D &px) const
{
  // point in IMU frame, then camera frame: pc = Rci·Rᵀ·(pw − p) + Pci
  const V3D p_i = x.rot_end.transpose() * (pos_w - x.pos_end);
  pc = Rci_ * p_i + Pci_;
  if (pc[2] < cfg_.depth_min) return false;
  px = cam_->world2cam(pc);
  const int rho = std::max(1, static_cast<int>(std::floor(cfg_.patch_spacing / 2.0)) - 1);
  return cam_->isInFrame(px.cast<int>(), rho + 1);
}

void SAVioManager::collectFramePhotometric(const SAWindowFrame &f, const StatesGroup &x,
                                           std::vector<PhotoResid> &out) const
{
  out.clear();
  const double eps = 1e-6;
  const double sigma2 = cfg_.sigma_px * cfg_.sigma_px;
  const auto resid = [&](const StatesGroup &xx, const SAObservation &obs) -> double {
    V3D pc2;
    V2D px2;
    if (!projectPoint(xx, obs.pos_w, pc2, px2)) return 0.0;
    std::array<float, SA_PATCH_NUM> cur2;
    sampleCross(f.img, px2, cur2);
    double s = 0.0;
    for (int k = 0; k < SA_PATCH_NUM; k++)
      s += cur2[k] - static_cast<float>(f.alpha * obs.ref_intensities[k] + f.beta);
    return s / SA_PATCH_NUM;
  };
  for (const auto &obs : f.obs)
  {
    V3D pc;
    V2D px;
    if (!projectPoint(x, obs.pos_w, pc, px)) continue;
    if ((px - obs.px_ref).norm() < cfg_.parallax_min_px) continue;
    std::array<float, SA_PATCH_NUM> cur;
    sampleCross(f.img, px, cur);
    double rsum = 0.0;
    for (int k = 0; k < SA_PATCH_NUM; k++)
      rsum += cur[k] - static_cast<float>(f.alpha * obs.ref_intensities[k] + f.beta);
    const double r = rsum / SA_PATCH_NUM;
    if (std::fabs(r) > cfg_.abs_res_max) continue;
    if (r * r / sigma2 > cfg_.chi2_thres) continue;
    PhotoResid pr;
    pr.r = r;
    pr.w = 1.0 / sigma2;
    for (int dof = 0; dof < 6; dof++)
    {
      StatesGroup xp = x, xm = x;
      V3D e = V3D::Zero();
      if (dof < 3)
      {
        e[dof] = 1.0;
        xp.rot_end = x.rot_end * Exp(e, eps);
        xm.rot_end = x.rot_end * Exp(e, -eps);
      }
      else
      {
        e[dof - 3] = 1.0;
        xp.pos_end += eps * e;
        xm.pos_end -= eps * e;
      }
      pr.J(0, dof) = (resid(xp, obs) - resid(xm, obs)) / (2.0 * eps);
    }
    out.push_back(pr);
  }
}

void SAVioManager::computeProjectionJacobian(const V3D &pc, MD(2, 3) &J) const
{
  // Eq. 24 requires the Jacobian of the ACTUAL camera model.  world2cam()
  // applies radial/tangential distortion for this dataset (and may implement
  // equidistant/fisheye models elsewhere), while the old closed form assumed
  // an undistorted pinhole.  A centred numerical derivative keeps projection
  // and Jacobian exactly consistent for every vk::AbstractCamera model.
  for (int k = 0; k < 3; k++)
  {
    const double eps = 1e-6 * std::max(1.0, std::fabs(pc[k]));
    V3D p_plus = pc;
    V3D p_minus = pc;
    p_plus[k] += eps;
    p_minus[k] -= eps;
    J.col(k) = (cam_->world2cam(p_plus) - cam_->world2cam(p_minus)) / (2.0 * eps);
  }
}

// ---------------------------------------------------------------------------
// Affine brightness model (Eq. 21)
// ---------------------------------------------------------------------------

bool SAVioManager::estimateAffine(const cv::Mat &img_cur, const StatesGroup &x0, SAWindowFrame &f,
                                  double &rrms_for_q) const
{
  // Accumulate the 2×2 normal equations over every pixel of every
  // observation's sparse cross pattern (Eq. 21), weighted by the frozen
  // reference gradient magnitude (constant for the observation).
  double A11 = 0, A12 = 0, A22 = 0, c1 = 0, c2 = 0;
  int used = 0;
  for (const auto &obs : f.obs)
  {
    V3D pc;
    V2D px;
    if (!projectPoint(x0, obs.pos_w, pc, px)) continue;  // gate (i) for affine

    std::array<float, SA_PATCH_NUM> cur;
    sampleCross(img_cur, px, cur);
    const double w = cfg_.affine_equal_weight ? 1.0 : obs.ref_grad_mag;
    for (int k = 0; k < SA_PATCH_NUM; k++)
    {
      const double iref = obs.ref_intensities[k];
      const double icur = cur[k];
      A11 += w * iref * iref;
      A12 += w * iref;
      A22 += w;
      c1 += w * icur * iref;
      c2 += w * icur;
      used++;
    }
  }
  if (used == 0) return false;

  const double det = A11 * A22 - A12 * A12;
  if (std::fabs(det) < 1e-9) return false;

  const double alpha = clampVal((A22 * c1 - A12 * c2) / det, cfg_.alpha_min, cfg_.alpha_max);
  const double beta = clampVal((A11 * c2 - A12 * c1) / det, cfg_.beta_min, cfg_.beta_max);
  f.alpha = alpha;
  f.beta = beta;
  f.wcam = std::exp(-std::max(std::fabs(std::log(alpha)) - cfg_.delta_alpha, 0.0));

  // RRMS of the current residuals of this frame (for the absolute gate
  // and the scene-level qrms factor)
  double sum_sq = 0.0;
  int n = 0;
  for (const auto &obs : f.obs)
  {
    V3D pc;
    V2D px;
    if (!projectPoint(x0, obs.pos_w, pc, px)) continue;
    std::array<float, SA_PATCH_NUM> cur;
    sampleCross(img_cur, px, cur);
    for (int k = 0; k < SA_PATCH_NUM; k++)
    {
      const double e = cur[k] - (alpha * obs.ref_intensities[k] + beta);
      sum_sq += e * e;
    }
    n += SA_PATCH_NUM;
  }
  rrms_for_q = (n > 0) ? std::sqrt(sum_sq / n) : 0.0;
  return true;
}

// ---------------------------------------------------------------------------
// Pre-loop visual information form (Sect. VI-D/G)
// ---------------------------------------------------------------------------

bool SAVioManager::buildVisualInfoForm(const cv::Mat &img_in, const StatesGroup &x0, InfoForm6 &out, double &q)
{
  out.reset();
  q = 0.0;
  n_valid_ = n_rej_ = 0;
  int g_proj = 0, g_view = 0, g_plx = 0, g_abs = 0, g_chi2 = 0;
  res_mean_ = 0.0;
  res_cnt_ = 0;

  if (cam_ == nullptr) return false;

  // Resize to the camera-model resolution: all projection coordinates come
  // from the scaled camera model (e.g. 1280x1024 -> 640x512 at scale 0.5).
  // Sampling the raw full-res image with scaled coordinates would offset
  // every pixel by 2x (this corrupted the point-cloud colors).
  // NOTE: dst must be a fresh Mat — cv::resize(src, dst) with dst aliasing
  // src (shallow copy of the same buffer) is UB and crashes inside TBB
  // (observed via ASAN: SEGV in libopencv_imgproc resize).
  cv::Mat img;
  if (img_in.cols != cam_->width() || img_in.rows != cam_->height())
    cv::resize(img_in, img, cv::Size(cam_->width(), cam_->height()), 0, 0, cv::INTER_LINEAR);
  else
    img = img_in;

  img_cp_ = img.clone();
  toGray(img, img_gray_);
  computeGradientFields(img_gray_);

  // The first frame has no reference window yet, but the exact current image
  // must still be available to the explicit colour path.
  if (window_.empty()) return false;

  const double cos_theta_max = std::cos(cfg_.theta_max_deg * M_PI / 180.0);

  // --- 1. Per-frame affine brightness models (Eq. 20–21) ------------------
  for (auto &f : window_)
  {
    double rrms_frame = 0.0;
    estimateAffine(img_gray_, x0, f, rrms_frame);
  }

  // --- 2. Residuals, Jacobians (frozen), gates, accumulation (Eq. 22–32) --
  // Per-observation temporaries (projections evaluated once at x0)
  struct ObsTmp
  {
    V3D pc;
    V2D px;
    bool valid = false;
    std::array<float, SA_PATCH_NUM> cur;
  };
  std::vector<ObsTmp> tmp;
  tmp.reserve(1024);
  for (const auto &f : window_)
  {
    for (const auto &obs : f.obs)
    {
      ObsTmp t;
      t.valid = projectPoint(x0, obs.pos_w, t.pc, t.px);
      if (t.valid)
      {
        if (cfg_.patch_search_px > 0)
        {
          // Robust photometric matching: search a small window around the
          // projected pixel for the best affine-compensated patch match.
          // Tolerates a few pixels of pose error (paper VI-D patch tracking).
          const int r = cfg_.patch_search_px;
          float best_cost = 1e30f;
          V2D best_px = t.px;
          std::array<float, SA_PATCH_NUM> best_cur;
          for (int dy = -r; dy <= r; dy++)
          {
            for (int dx = -r; dx <= r; dx++)
            {
              const V2D p(t.px[0] + dx, t.px[1] + dy);
              std::array<float, SA_PATCH_NUM> cur;
              sampleCross(img_gray_, p, cur);
              float cost = 0.0f;
              for (int k = 0; k < SA_PATCH_NUM; k++)
              {
                const float e = cur[k] - static_cast<float>(f.alpha * obs.ref_intensities[k] + f.beta);
                cost += e * e;
              }
              if (cost < best_cost)
              {
                best_cost = cost;
                best_px = p;
                best_cur = cur;
              }
            }
          }
          t.px = best_px;
          t.cur = best_cur;
        }
        else
        {
          sampleCross(img_gray_, t.px, t.cur);
        }
      }
      tmp.push_back(t);
    }
  }

  double sum_rrms = 0.0;
  int sum_rrms_n = 0;
  double dbg_sigma_sum = 0.0, dbg_chi_sum = 0.0;
  int dbg_sigma_n = 0;

  // Empirical per-frame residual scale (verifiable noise-model correction):
  //   s_emp = 1.4826 · MAD(r) over observations passing the spatial gates
  //   (proj/view/parallax/absolute) but BEFORE the χ² gate.  With
  //   adaptive_noise_en the innovation gate and information weights use
  //   σ²_eff = max(σ²_paper, s_emp²) so the gate measures relative
  //   outlier-ness instead of an absolute noise floor.  This recovers
  //   visual information in motion segments where the frozen paper σ²
  //   underestimates the true residual variance, while still rejecting
  //   the corridor's outlier tail.
  double s_emp = 0.0;
  if (cfg_.adaptive_noise_en)
  {
    std::vector<double> resid;
    resid.reserve(1024);
    int idx2 = 0;
    for (const auto &f : window_)
    {
      for (const auto &obs : f.obs)
      {
        const ObsTmp &t = tmp[idx2++];
        if (!t.valid) continue;
        const V3D n_c = Rci_ * x0.rot_end.transpose() * obs.normal_w;
        if (std::fabs(n_c.dot(t.pc.normalized())) <= cos_theta_max) continue;
        if ((t.px - obs.px_ref).norm() < cfg_.parallax_min_px) continue;
        const double alpha = f.alpha, beta = f.beta;
        double rsum = 0.0, ssq = 0.0;
        for (int k = 0; k < SA_PATCH_NUM; k++)
        {
          const double e = t.cur[k] - (alpha * obs.ref_intensities[k] + beta);
          rsum += e; ssq += e * e;
        }
        const double rrms = std::sqrt(ssq / SA_PATCH_NUM);
        if (rrms > cfg_.abs_res_max) continue;
        resid.push_back(rsum / SA_PATCH_NUM);
      }
    }
    if (resid.size() >= 8)
    {
      std::sort(resid.begin(), resid.end());
      const double med = resid[resid.size() / 2];
      std::vector<double> adev;
      adev.reserve(resid.size());
      for (double r : resid) adev.push_back(std::fabs(r - med));
      std::sort(adev.begin(), adev.end());
      const double mad = adev[adev.size() / 2];
      s_emp = std::max(cfg_.adaptive_noise_min_sigma, 1.4826 * mad);
    }
  }

  std::mutex mtx;
  int idx = 0;
  for (const auto &f : window_)
  {
    const int frame_id = f.id;
    for (const auto &obs : f.obs)
    {
      const ObsTmp &t = tmp[idx++];
      bool ok = true;

      // gate (i): projection validity (depth > dmin, in frame)
      if (!t.valid) { n_rej_++; g_proj++; continue; }

      const double alpha = f.alpha, beta = f.beta;
      double rrms = 0.0;
      if (!cfg_.rrms_after_gates)
      {
        // Scene-level RRMS is evaluated for every successful projection before
        // any view/parallax/residual gate (current default).
        double sum_sq = 0.0;
        for (int k = 0; k < SA_PATCH_NUM; k++)
        {
          const double e = t.cur[k] - (alpha * obs.ref_intensities[k] + beta);
          sum_sq += e * e;
        }
        rrms = std::sqrt(sum_sq / SA_PATCH_NUM);
        sum_rrms += rrms;
        sum_rrms_n++;
      }

      // gate (ii): view angle |n·d̂| > cos θmax
      {
        const V3D n_c = Rci_ * x0.rot_end.transpose() * obs.normal_w;
        const V3D d_hat = t.pc.normalized();
        if (std::fabs(n_c.dot(d_hat)) <= cos_theta_max) { n_rej_++; g_view++; continue; }
      }

      // gate (iii): parallax ‖px − px_ref‖ ≥ δplx
      {
        const double plx = (t.px - obs.px_ref).norm();
        if (plx < cfg_.parallax_min_px) { n_rej_++; g_plx++; continue; }
      }

      // gate (iv): absolute residual rrms ≤ δabs
      if (rrms > cfg_.abs_res_max) { n_rej_++; g_abs++; continue; }

      // Photometric residual (Eq. 23): mean over the 9-pixel cross
      double res_sum = 0.0;
      for (int k = 0; k < SA_PATCH_NUM; k++)
      {
        res_sum += t.cur[k] - (alpha * obs.ref_intensities[k] + beta);
      }
      const double res = res_sum / SA_PATCH_NUM;
      res_mean_ += res;
      res_cnt_++;

      // Jacobian of the scalar residual in Eq. 23.  The residual is the
      // MEAN of the 9 cross-pattern pixel residuals, so its image gradient
      // must be the mean gradient over the same 9 samples.  Using only the
      // centre-pixel gradient (the old code) is not the derivative of the
      // accumulated residual and produces biased high-leverage rotation
      // updates on text/edge-heavy patches.
      const double rho = static_cast<double>(std::max(1, static_cast<int>(std::floor(cfg_.patch_spacing / 2.0)) - 1));
      const double du[SA_PATCH_NUM] = {0, rho, -rho, 0, 0, rho, -rho, rho, -rho};
      const double dv[SA_PATCH_NUM] = {0, 0, 0, rho, -rho, rho, rho, -rho, -rho};
      double gx_v = 0.0, gy_v = 0.0;
      for (int k = 0; k < SA_PATCH_NUM; k++)
      {
        double gx_k, gy_k;
        gradientAt(t.px[0] + du[k], t.px[1] + dv[k], gx_k, gy_k);
        gx_v += gx_k;
        gy_v += gy_k;
      }
      gx_v /= SA_PATCH_NUM;
      gy_v /= SA_PATCH_NUM;

      // Image-to-3D gradient chain: ∇Ij = [Ix, Iy]·Jπ ∈ R^{1×3}
      MD(2, 3) Jpi;
      computeProjectionJacobian(t.pc, Jpi);
      Eigen::Matrix<double, 1, 3> grad_3d;
      grad_3d(0) = gx_v * Jpi(0, 0) + gy_v * Jpi(1, 0);
      grad_3d(1) = gx_v * Jpi(0, 1) + gy_v * Jpi(1, 1);
      grad_3d(2) = gx_v * Jpi(0, 2) + gy_v * Jpi(1, 2);

      // Depth-conditioned photometric noise (Eq. 25). The map point and its
      // full world covariance are frozen together at observation creation;
      // rebuilding a historical point's covariance from the current pose
      // would mix coordinate frames.  sigma2_live_cov instead rebuilds Σw
      // from the *current* state covariance via Eq. 12, so degenerate-scene
      // covariance growth is reflected in the innovation gate.
      const M3D R0 = x0.rot_end;
      const M3D Rcw = Rci_ * R0.transpose();
      M3D cov_w_used = obs.sigma_w;
      if (cfg_.sigma2_live_cov)
      {
        const M3D cross = hat(obs.point_i);
        const M3D Rwl = R0 * Rli_.transpose();
        cov_w_used = Rwl * obs.sigma_b * Rwl.transpose()
                     + R0 * cross * x0.cov.block<3, 3>(0, 0) * cross.transpose() * R0.transpose()
                     + x0.cov.block<3, 3>(3, 3);
      }
      else if (cfg_.sigma2_sensor_only)
      {
        const M3D Rwl = R0 * Rli_.transpose();
        cov_w_used = Rwl * obs.sigma_b * Rwl.transpose();
      }
      const M3D cov_c = Rcw * cov_w_used * Rcw.transpose();
      const double sigpx2 = cfg_.sigma_px * cfg_.sigma_px *
                            (cfg_.patch_mean_noise_en ? (1.0 / SA_PATCH_NUM) : 1.0);
      const double sigma2 = sigpx2 + (grad_3d * cov_c * grad_3d.transpose()).value();
      const double sigma2_eff = (cfg_.adaptive_noise_en && s_emp > 0.0)
          ? std::max(sigma2, s_emp * s_emp)
          : sigma2;
      dbg_sigma_sum += std::sqrt(std::max(sigma2, 0.0));
      dbg_chi_sum += res * res / std::max(sigma2_eff, 1e-6);
      dbg_sigma_n++;

      // gate (v): χ² innovation gate r²/σ² ≤ τV
      if (res * res / std::max(sigma2_eff, 1e-6) > cfg_.chi2_thres) { n_rej_++; g_chi2++; continue; }

      if (cfg_.rrms_after_gates)
      {
        double sum_sq = 0.0;
        for (int k = 0; k < SA_PATCH_NUM; k++)
        {
          const double e = t.cur[k] - (alpha * obs.ref_intensities[k] + beta);
          sum_sq += e * e;
        }
        rrms = std::sqrt(sum_sq / SA_PATCH_NUM);
        sum_rrms += rrms;
        sum_rrms_n++;
      }

      // Frozen Jacobian (Eq. 24, body-frame error basis; code convention
      // h = +Jᵀ with measurement sign folded into the accumulation):
      //   J_rot   = ∇Ij·⌊pc − Pci⌋_×·Rci
      //   J_trans = ∇Ij·(−Rci·Rᵀ)
      Eigen::Matrix<double, 1, 6> h;
      const V3D pc_minus_ci = t.pc - Pci_;
      // Eq.24 right-invariant (paper): J_rot = ∇Ij·⌊pc−Pci⌋_×·Rci·Rᵀ.
      if (g_right_invariant_en)
        h.block<1, 3>(0, 0) = grad_3d * hat(pc_minus_ci) * Rci_ * R0.transpose();
      else
        h.block<1, 3>(0, 0) = grad_3d * hat(pc_minus_ci) * Rci_;
      h.block<1, 3>(0, 3) = -grad_3d * Rci_ * R0.transpose();

      // Composite weight (Eq. 30): affine confidence × decorrelation.
      // A Huber loss is a non-paper diagnostic and is disabled by default.
      const double whub = cfg_.photometric_huber_en
          ? ((std::fabs(res) > 1e-12)
                 ? std::min(1.0, cfg_.abs_res_max / (2.0 * std::fabs(res)))
                 : 1.0)
          : 1.0;
      const double wdec = 1.0 / std::max(obs.nused, 1);
      const double w = whub * f.wcam * wdec / sigma2_eff;

      // Accumulate (ΛV, bV) — sign convention: b += w·hᵀ·(−r)
      mtx.lock();
      out.Lambda += w * h.transpose() * h;
      out.b -= w * h.transpose() * res;
      mtx.unlock();
      n_valid_++;

      // Decorrelation counter: incremented each time the observation
      // contributes to ΛV (Eq. 26)
      const_cast<SAObservation &>(obs).nused++;
    }
  }

  // --- 3. Scene-level quality factor q (Eq. 27–29) ------------------------
  rrms_last_ = (sum_rrms_n > 0) ? (sum_rrms / sum_rrms_n) : 0.0;

  // Diagnostic per-frame information cap: prevents a handful of observations
  // with extreme photometric gradients (near-range, high-contrast) from
  // dominating the joint solve. Scales ΛV and bV isotropically so the
  // directional structure is preserved.
  if (cfg_.lamv_max > 0.0)
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es_cap(out.Lambda);
    const double lam_max = es_cap.eigenvalues()(5);
    if (lam_max > cfg_.lamv_max)
    {
      const double s = cfg_.lamv_max / lam_max;
      out.Lambda *= s;
      out.b *= s;
    }
  }

  // Diagnostic / tuning gain: keep the information form's directional
  // structure intact while scaling its magnitude relative to ΛL.
  out.Lambda *= cfg_.info_scale;
  out.b *= cfg_.info_scale;

  double qrms = 1.0;
  if (rrms_last_ > cfg_.delta_rms)
  {
    qrms = std::exp(-(rrms_last_ - cfg_.delta_rms) / cfg_.sigma_rms);
  }
  const double qcnt = std::min(static_cast<double>(n_valid_) / std::max(cfg_.min_obs, 1), 1.0);
  const int n_tot = n_valid_ + n_rej_;
  double qrej = 1.0;
  if (n_tot > 0)
  {
    const double rho = static_cast<double>(n_rej_) / n_tot;
    if (rho > cfg_.rho_max) qrej = 0.0;
    else if (rho > cfg_.rho_min)
      qrej = (cfg_.rho_max - rho) / std::max(cfg_.rho_max - cfg_.rho_min, 1e-6);
  }
  q = qrms * qcnt * qrej;
  q_last_ = q;

  // Per-frame diagnostics are required by the fail-closed replay harness: they
  // bind q to the individual gate counts and information spectrum.
  double sum_res = 0.0, sum_alpha = 0.0, sum_beta = 0.0;
  int nn = 0;
  for (const auto &f : window_)
  {
    sum_alpha += f.alpha;
    sum_beta += f.beta;
    nn++;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(out.Lambda);
  printf("[ SA-VIO ] valid=%d rej: proj=%d view=%d plx=%d abs=%d chi2=%d | q=%.3f rrms=%.2f s_emp=%.2f"
         " | mean-res=%.3f | alpha=%.2f beta=%.1f | mean-sigma=%.2f mean-chi=%.2f"
         " | lamV=[%.1f %.1f %.1f %.1f %.1f %.1f] | |bV|=%.3f\n",
         n_valid_, g_proj, g_view, g_plx, g_abs, g_chi2, q, rrms_last_,
         s_emp,
         res_cnt_ > 0 ? res_mean_ / res_cnt_ : 0.0,
         nn > 0 ? sum_alpha / nn : 1.0, nn > 0 ? sum_beta / nn : 0.0,
         dbg_sigma_n > 0 ? dbg_sigma_sum / dbg_sigma_n : 0.0,
         dbg_sigma_n > 0 ? dbg_chi_sum / dbg_sigma_n : 0.0,
         es.eigenvalues()(0), es.eigenvalues()(1), es.eigenvalues()(2),
         es.eigenvalues()(3), es.eigenvalues()(4), es.eigenvalues()(5),
         out.b.norm());

  return n_valid_ > 0;
}

// ---------------------------------------------------------------------------
// Observation selection for a new window frame (Sect. VI-B)
// ---------------------------------------------------------------------------

void SAVioManager::selectObservations(const cv::Mat &img, const StatesGroup &x,
                                      const std::vector<pointWithVar> &pv_list,
                                      std::vector<SAObservation> &new_obs)
{
  new_obs.clear();
  if (cam_ == nullptr || pv_list.empty()) return;

  const int w = img.cols, h = img.rows;
  const double cos_theta_max = std::cos(cfg_.theta_max_deg * M_PI / 180.0);
  const int cont_r = static_cast<int>(std::ceil(cfg_.depth_cont_radius_px));

  // 1. Sparse depth image from all LiDAR-anchored points (min depth wins)
  depth_img_ = cv::Mat::zeros(h, w, CV_32FC1);
  std::vector<std::pair<V2D, double>> projections;
  projections.reserve(pv_list.size());
  for (const auto &pv : pv_list)
  {
    V3D pc;
    V2D px;
    if (!projectPoint(x, pv.point_w, pc, px)) continue;
    projections.emplace_back(px, pc[2]);
    const int u = static_cast<int>(px[0]), v = static_cast<int>(px[1]);
    if (u < 0 || v < 0 || u >= w || v >= h) continue;
    float &d = depth_img_.at<float>(v, u);
    if (d <= 0.0f || pc[2] < d) d = static_cast<float>(pc[2]);
  }

  // 2. Gradient energy per point + geometric gates + one-per-cell selection
  struct Cand
  {
    V2D px;
    double depth;
    double sgrad;
    const pointWithVar *pv;
  };
  std::unordered_map<int64_t, Cand> best;  // image cell → best candidate
  auto image_cell_key = [](double u, double v, double cell) -> int64_t {
    const int64_t x = static_cast<int64_t>(std::floor(u / cell));
    const int64_t y = static_cast<int64_t>(std::floor(v / cell));
    return (x * 73856093) ^ (y * 19349663);
  };
  auto world_cell_key = [](const V3D &pw, double gs) -> int64_t {
    const int64_t x = static_cast<int64_t>(std::floor(pw[0] / gs));
    const int64_t y = static_cast<int64_t>(std::floor(pw[1] / gs));
    const int64_t z = static_cast<int64_t>(std::floor(pw[2] / gs));
    return ((x * 73856093) ^ (y * 19349663) ^ (z * 83492791));
  };

  size_t npts = 0;
  int dbg_norm = 0, dbg_proj = 0, dbg_view = 0, dbg_depth = 0;
  for (const auto &pv : pv_list)
  {
    if (pv.normal.norm() < 0.5) continue;  // need a surface normal
    dbg_norm++;

    V3D pc;
    V2D px;
    if (!projectPoint(x, pv.point_w, pc, px)) continue;
    dbg_proj++;
    const double depth = pc[2];

    // view-angle gate (Eq. 19): |n·d̂| > cos θmax
    const V3D n_c = Rci_ * x.rot_end.transpose() * pv.normal;
    const V3D d_hat = pc.normalized();
    if (std::fabs(n_c.dot(d_hat)) <= cos_theta_max) continue;
    dbg_view++;

    // depth continuity: no valid depth in the 9×9 neighbourhood differing
    // by more than δd from the point's own depth
    const int u0 = static_cast<int>(px[0]), v0 = static_cast<int>(px[1]);
    bool occluded = false;
    for (int dv = -cont_r; dv <= cont_r && !occluded; dv++)
    {
      const int vv = v0 + dv;
      if (vv < 0 || vv >= h) continue;
      const float *row = depth_img_.ptr<float>(vv);
      for (int du = -cont_r; du <= cont_r; du++)
      {
        const int uu = u0 + du;
        if (uu < 0 || uu >= w) continue;
        const float d = row[uu];
        if (d > 0.0f && std::fabs(d - depth) > cfg_.depth_cont_thresh_m)
        {
          occluded = true;
          break;
        }
      }
    }
    if (occluded) continue;
    dbg_depth++;

    // gradient energy (Eq. 18): sgrad = Ix² + Iy² at the projected location
    double gx_v, gy_v;
    gradientAt(px[0], px[1], gx_v, gy_v);
    const double sgrad = gx_v * gx_v + gy_v * gy_v;

    const int64_t key = cfg_.point_select_image_grid
                            ? image_cell_key(px[0], px[1], cfg_.point_image_cell_px)
                            : world_cell_key(pv.point_w, cfg_.point_grid_size);
    auto it = best.find(key);
    if (it == best.end() || sgrad > it->second.sgrad)
    {
      best[key] = Cand{px, depth, sgrad, &pv};
    }
  }
  if (cfg_.debug_obs_gates)
  {
    printf("[ SA-VIO-DBG ] pv=%zu norm=%d proj=%d view=%d depth=%d cand=%zu\n",
           pv_list.size(), dbg_norm, dbg_proj, dbg_view, dbg_depth, best.size());
  }

  // 3. Cap with uniform stride downsampling (preserve spatial coverage)
  std::vector<Cand> cands;
  cands.reserve(best.size());
  for (auto &kv : best) cands.push_back(kv.second);
  if (static_cast<int>(cands.size()) > cfg_.max_obs_per_frame)
  {
    const int stride = static_cast<int>(std::ceil(static_cast<double>(cands.size()) / cfg_.max_obs_per_frame));
    std::vector<Cand> stride_sampled;
    stride_sampled.reserve(cands.size() / stride + 1);
    for (size_t i = 0; i < cands.size(); i += stride) stride_sampled.push_back(cands[i]);
    cands.swap(stride_sampled);
  }

  // 4. Create observations with frozen reference patches
  for (const auto &c : cands)
  {
    SAObservation obs;
    obs.pos_w = c.pv->point_w;
    obs.normal_w = c.pv->normal.normalized();
    // p_L = Rli_ * p_I + Pli_  =>  p_I = Rli_^T * (p_L - Pli_)
    obs.point_i = Rli_.transpose() * (c.pv->point_b - Pli_);
    obs.sigma_b = c.pv->body_var;
    obs.sigma_w = c.pv->var;
    obs.px_ref = c.px;
    obs.depth_ref = c.depth;
    sampleCross(img, c.px, obs.ref_intensities);

    // mean reference gradient magnitude over the patch pixels
    double gm = 0.0;
    const double rho = static_cast<double>(std::max(1, static_cast<int>(std::floor(cfg_.patch_spacing / 2.0)) - 1));
    const double du[SA_PATCH_NUM] = {0, rho, -rho, 0, 0, rho, -rho, rho, -rho};
    const double dv[SA_PATCH_NUM] = {0, 0, 0, rho, -rho, rho, rho, -rho, -rho};
    for (int k = 0; k < SA_PATCH_NUM; k++)
    {
      double gxx, gyy;
      gradientAt(c.px[0] + du[k], c.px[1] + dv[k], gxx, gyy);
      gm += std::sqrt(gxx * gxx + gyy * gyy);
    }
    obs.ref_grad_mag = gm / SA_PATCH_NUM;

    obs.nused = 1;
    obs.frame_id = frame_counter_;
    new_obs.push_back(obs);
  }
}

// ---------------------------------------------------------------------------
// Sliding-window advancement
// ---------------------------------------------------------------------------

void SAVioManager::advanceWindow(const cv::Mat &img, const StatesGroup &x,
                                 const std::vector<pointWithVar> &pv_list, double frame_time)
{
  // Duplicate-frame suppression: reject near-identical frames
  if (!window_.empty())
  {
    const SAWindowFrame &newest = window_.back();
    const M3D dR = newest.R_w.transpose() * x.rot_end;
    const double ang = (Eigen::AngleAxisd(dR).angle()) * 180.0 / M_PI;
    const double dt = (x.pos_end - newest.p_w).norm();
    if (ang < cfg_.dup_R_deg && dt < cfg_.dup_t_m)
    {
      if (cfg_.debug_obs_gates)
        printf("[ SA-VIO-DBG ] dup-suppress: ang=%.4f deg dt=%.4f m (window=%zu)\n",
               ang, dt, window_.size());
      return;
    }
  }

  // Resize to the camera-model resolution (see buildVisualInfoForm note).
  // Fresh dst Mat: cv::resize with an aliased dst is UB (TBB crash).
  cv::Mat img_resized;
  if (img.cols != cam_->width() || img.rows != cam_->height())
    cv::resize(img, img_resized, cv::Size(cam_->width(), cam_->height()), 0, 0, cv::INTER_LINEAR);
  else
    img_resized = img;

  toGray(img_resized, img_gray_);
  computeGradientFields(img_gray_);

  SAWindowFrame f;
  f.id = frame_counter_++;
  f.t = frame_time;
  f.R_w = x.rot_end;
  f.p_w = x.pos_end;
  f.img = img_gray_.clone();
  f.alpha = 1.0;
  f.beta = 0.0;
  f.wcam = 1.0;

  selectObservations(img_gray_, x, pv_list, f.obs);

  window_.push_back(std::move(f));

  // Evict the oldest frame beyond W
  while (static_cast<int>(window_.size()) > cfg_.window_size)
  {
    window_.pop_front();
  }
}

void SAVioManager::printStatus() const
{
  printf("[ SA-VIO ] window: %zu frames, valid: %d, rejected: %d, q: %.3f, rrms: %.2f\n",
         window_.size(), n_valid_, n_rej_, q_last_, rrms_last_);
}

V3F SAVioManager::getInterpolatedPixelForViz(const cv::Mat &img, const V2D &pc) const
{
  V3F pixel(0, 0, 0);
  if (img.empty()) return pixel;
  const int w = img.cols, h = img.rows;
  const int x0 = static_cast<int>(std::floor(pc[0]));
  const int y0 = static_cast<int>(std::floor(pc[1]));
  if (x0 < 0 || y0 < 0 || x0 >= w - 1 || y0 >= h - 1) return pixel;
  const double fx = pc[0] - x0, fy = pc[1] - y0;
  if (img.channels() == 1)
  {
    const double v00 = img.at<uint8_t>(y0, x0), v01 = img.at<uint8_t>(y0, x0 + 1);
    const double v10 = img.at<uint8_t>(y0 + 1, x0), v11 = img.at<uint8_t>(y0 + 1, x0 + 1);
    const double v = (1 - fy) * ((1 - fx) * v00 + fx * v01) + fy * ((1 - fx) * v10 + fx * v11);
    pixel = V3F(v, v, v);
  }
  else if (img.channels() == 3)
  {
    const cv::Vec3b &p00 = img.at<cv::Vec3b>(y0, x0), &p01 = img.at<cv::Vec3b>(y0, x0 + 1);
    const cv::Vec3b &p10 = img.at<cv::Vec3b>(y0 + 1, x0), &p11 = img.at<cv::Vec3b>(y0 + 1, x0 + 1);
    for (int c = 0; c < 3; c++)
    {
      const double v = (1 - fy) * ((1 - fx) * p00[c] + fx * p01[c]) + fy * ((1 - fx) * p10[c] + fx * p11[c]);
      pixel[2 - c] = static_cast<float>(v);  // BGR → RGB order
    }
  }
  return pixel;
}
