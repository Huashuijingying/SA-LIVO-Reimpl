// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

struct AdaptiveVoxelKey
{
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;
  bool operator==(const AdaptiveVoxelKey &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct AdaptiveVoxelKeyHash
{
  static std::uint64_t mix(std::uint64_t x) noexcept
  {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31U);
  }

  std::size_t operator()(const AdaptiveVoxelKey &k) const noexcept
  {
    return static_cast<std::size_t>(mix(static_cast<std::uint64_t>(k.x)) ^
                                    (mix(static_cast<std::uint64_t>(k.y)) << 1U) ^
                                    (mix(static_cast<std::uint64_t>(k.z)) << 2U));
  }
};

struct AdaptiveVoxelCell
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d s1 = Eigen::Vector3d::Zero();
  Eigen::Matrix3d S2 = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d covariance_sum = Eigen::Matrix3d::Zero();
  std::vector<Eigen::Matrix3d> covariance_obs;
  std::vector<Eigen::Vector3d> points_obs;
  std::uint32_t n = 0;

  void add(const Eigen::Vector3d &point, const Eigen::Matrix3d &full_covariance,
           std::uint32_t n_cap)
  {
    if (n >= n_cap) return;
    if (!full_covariance.allFinite()) throw std::invalid_argument("non-finite map point covariance");
    s1 += point;
    // Eq.9 geometry remains the exact unweighted point sufficient statistic.
    // Full Eq.11 covariance is retained separately by the map-update path so
    // it cannot distort kappa/lambda_min while remaining auditable per cell.
    S2 += point * point.transpose();
    covariance_sum += 0.5 * (full_covariance + full_covariance.transpose());
    covariance_obs.push_back(0.5 * (full_covariance + full_covariance.transpose()));
    points_obs.push_back(point);
    ++n;
  }
};

struct AdaptivePlaneFit
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  double d = 0.0;
  double lambda_min = 0.0;
  double radius = 0.0;
  double residual = 0.0;
  double residual_variance = 0.0;
  double kappa = 1.0 / 3.0;
  std::uint32_t support_count = 0;
  int shell = -1;
};

struct AdaptiveVoxelConfig
{
  double voxel_size = 1.0;
  double support_radius_m = 2.0;
  double kappa_threshold = 0.05;
  double association_chi2 = 25.0;
  double epsilon0 = 1e-3;
  double epsilon_abs = 0.01;
  std::uint32_t n_pca = 25;
  std::uint32_t n_cap = 50;
  bool use_map_covariance = false;
};

class AdaptiveVoxelGrid
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Map = std::unordered_map<AdaptiveVoxelKey, AdaptiveVoxelCell, AdaptiveVoxelKeyHash>;

  explicit AdaptiveVoxelGrid(const AdaptiveVoxelConfig &config) : config_(config)
  {
    if (!(config_.voxel_size > 0.0) || !std::isfinite(config_.voxel_size) ||
        config_.n_pca == 0 || config_.n_cap == 0)
      throw std::invalid_argument("invalid adaptive voxel configuration");
    max_shell_ = std::clamp(
        static_cast<int>(std::ceil(config_.support_radius_m / config_.voxel_size)), 1, 6);
  }

  AdaptiveVoxelKey keyOf(const Eigen::Vector3d &point) const
  {
    if (!point.allFinite()) throw std::invalid_argument("non-finite adaptive voxel point");
    AdaptiveVoxelKey key;
    std::int64_t *out[3] = {&key.x, &key.y, &key.z};
    for (int axis = 0; axis < 3; ++axis)
    {
      const long double scaled = std::floor(static_cast<long double>(point[axis]) /
                                            static_cast<long double>(config_.voxel_size));
      const long double lower = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
      const long double upper = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
      if (scaled < lower || scaled > upper)
      {
        throw std::overflow_error("adaptive voxel key overflow");
      }
      *out[axis] = static_cast<std::int64_t>(scaled);
    }
    return key;
  }

  void addPoint(const Eigen::Vector3d &point,
                const Eigen::Matrix3d &full_covariance = Eigen::Matrix3d::Zero())
  {
    cells_[keyOf(point)].add(point, full_covariance, config_.n_cap);
  }

  bool queryPlane(const Eigen::Vector3d &query,
                  const Eigen::Matrix3d &full_world_covariance,
                  AdaptivePlaneFit &result) const
  {
    result = AdaptivePlaneFit();
    const AdaptiveVoxelKey center_key = keyOf(query);
    Eigen::Vector3d s1 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d S2 = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d cov_sum = Eigen::Matrix3d::Zero();
    std::vector<Eigen::Matrix3d> cov_obs;
    std::vector<Eigen::Vector3d> points_obs;
    std::uint32_t n = 0;

    for (int shell = 0; shell <= max_shell_; ++shell)
    {
      addShell(center_key, shell, s1, S2, cov_sum, cov_obs, points_obs, n);
      if (n < config_.n_pca) continue;

      const Eigen::Vector3d center = s1 / static_cast<double>(n);
      Eigen::Matrix3d covariance = S2 / static_cast<double>(n) - center * center.transpose();
      covariance = 0.5 * (covariance + covariance.transpose());
      if (!center.allFinite() || !covariance.allFinite()) return false;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
      if (solver.info() != Eigen::Success) return false;
      const Eigen::Vector3d eigenvalues = solver.eigenvalues();
      if (!eigenvalues.allFinite()) return false;
      const double trace = eigenvalues.sum();
      const double kappa = trace > 1e-12 ? std::max(eigenvalues[0], 0.0) / trace : 1.0 / 3.0;
      // Eq. 9: dual planarity test (scale-invariant ratio + absolute guard)
      if (!(kappa < config_.kappa_threshold) ||
          !(std::max(eigenvalues[0], 0.0) < config_.epsilon_abs))
        continue;

      AdaptivePlaneFit candidate;
      candidate.normal = solver.eigenvectors().col(0).normalized();
      candidate.center = center;
      candidate.lambda_min = std::max(eigenvalues[0], 0.0);
      candidate.radius = std::sqrt(std::max(eigenvalues[2], 0.0));
      candidate.kappa = kappa;
      candidate.support_count = n;
      candidate.shell = shell;
      candidate.d = -candidate.normal.dot(candidate.center);

      const Eigen::Vector3d offset = query - candidate.center;
      const Eigen::Vector3d lateral = offset - candidate.normal * candidate.normal.dot(offset);
      if (lateral.norm() > 3.0 * candidate.radius) return false;

      // Eq. 13/14: first-order eigenvector perturbation plane covariance.
      candidate.covariance.setZero();
      const int min_idx = 0;  // SelfAdjointEigenSolver returns ascending λ
      for (std::size_t i = 0; i < cov_obs.size(); ++i)
      {
        Eigen::Matrix<double, 6, 3> Ji;
        Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
        for (int m = 0; m < 3; ++m)
        {
          if (m == min_idx) continue;
          const double denom = static_cast<double>(n) *
                               (std::max(eigenvalues[min_idx], 0.0) - eigenvalues[m]);
          if (std::fabs(denom) < 1e-12) continue;
          const Eigen::Vector3d vm = solver.eigenvectors().col(m);
          const Eigen::Vector3d vmin = solver.eigenvectors().col(min_idx);
          F.row(m) = ((points_obs[i] - center).transpose() / denom) *
                     (vm * vmin.transpose() + vmin * vm.transpose());
        }
        Ji.block<3, 3>(0, 0) = solver.eigenvectors() * F;
        Ji.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() / static_cast<double>(n);
        candidate.covariance += Ji * cov_obs[i] * Ji.transpose();
      }
      Eigen::Matrix<double, 1, 6> Jnq;
      Jnq.block<1, 3>(0, 0) = offset.transpose();
      Jnq.block<1, 3>(0, 3) = -candidate.normal.transpose();
      candidate.residual = candidate.normal.dot(offset);
      candidate.residual_variance = config_.epsilon0 +
          (Jnq * candidate.covariance * Jnq.transpose()).value() +
          (candidate.normal.transpose() *
           (config_.use_map_covariance && n > 0
                ? cov_sum / static_cast<double>(n)
                : full_world_covariance) *
           candidate.normal).value();
      if (!(candidate.residual_variance > 0.0) ||
          candidate.residual * candidate.residual >
              config_.association_chi2 * candidate.residual_variance)
      {
        return false;
      }
      candidate.valid = true;
      result = candidate;
      return true;  // v2: accept the first certifiably planar support scale.
    }
    return false;
  }

  const AdaptiveVoxelCell *findCell(const AdaptiveVoxelKey &key) const
  {
    const auto it = cells_.find(key);
    return it == cells_.end() ? nullptr : &it->second;
  }

  std::size_t size() const { return cells_.size(); }
  void clear() { cells_.clear(); }
  std::size_t eraseOutside(const AdaptiveVoxelKey &minimum, const AdaptiveVoxelKey &maximum)
  {
    std::size_t erased = 0;
    for (auto it = cells_.begin(); it != cells_.end();)
    {
      const auto &k = it->first;
      if (k.x < minimum.x || k.x > maximum.x || k.y < minimum.y || k.y > maximum.y ||
          k.z < minimum.z || k.z > maximum.z)
      {
        it = cells_.erase(it);
        ++erased;
      }
      else ++it;
    }
    return erased;
  }
  int maxShell() const { return max_shell_; }
  const Map &cells() const { return cells_; }
  Map &cells() { return cells_; }

private:
  void addShell(const AdaptiveVoxelKey &center, int shell,
                Eigen::Vector3d &s1, Eigen::Matrix3d &S2,
                Eigen::Matrix3d &cov_sum,
                std::vector<Eigen::Matrix3d> &cov_obs,
                std::vector<Eigen::Vector3d> &points_obs,
                std::uint32_t &n) const
  {
    for (int dx = -shell; dx <= shell; ++dx)
      for (int dy = -shell; dy <= shell; ++dy)
        for (int dz = -shell; dz <= shell; ++dz)
        {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != shell) continue;
          const AdaptiveVoxelKey key{center.x + dx, center.y + dy, center.z + dz};
          const auto it = cells_.find(key);
          if (it == cells_.end()) continue;
          s1 += it->second.s1;
          S2 += it->second.S2;
          cov_sum += it->second.covariance_sum;
          cov_obs.insert(cov_obs.end(), it->second.covariance_obs.begin(),
                         it->second.covariance_obs.end());
          points_obs.insert(points_obs.end(), it->second.points_obs.begin(),
                            it->second.points_obs.end());
          n += it->second.n;
        }
  }

  AdaptiveVoxelConfig config_;
  int max_shell_ = 1;
  Map cells_;
};
