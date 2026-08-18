#include "adaptive_voxel_map_v2.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;

static void addPlanePatch(AdaptiveVoxelGrid &grid, double x0, double x_span,
                          double y0, double y_span, double z, int nx = 5, int ny = 5)
{
  for (int ix = 0; ix < nx; ++ix)
    for (int iy = 0; iy < ny; ++iy)
      grid.addPoint(V3(x0 + x_span * ix / (nx - 1),
                       y0 + y_span * iy / (ny - 1), z));
}

int main()
{
  AdaptiveVoxelConfig cfg;
  cfg.voxel_size = 1.0;
  cfg.n_cap = 50;
  cfg.n_pca = 25;
  cfg.kappa_threshold = 0.05;
  cfg.association_chi2 = 25.0;

  {
    AdaptiveVoxelGrid grid(cfg);
    for (int i = 0; i < 80; ++i) grid.addPoint(V3(0.1 + i * 1e-5, 0.2, 0.3));
    const auto *cell = grid.findCell({0, 0, 0});
    assert(cell != nullptr && cell->n == 50);
    assert(grid.keyOf(V3(-0.01, -1.01, 2.99)) == (AdaptiveVoxelKey{-1, -2, 2}));
    std::cout << "[ok] exact per-cell Ncap and negative floor key\n";
  }

  {
    AdaptiveVoxelGrid grid(cfg);
    const V3 p(0.2, 0.3, 0.4);
    M3 cov = M3::Zero();
    cov.diagonal() << 0.01, 0.02, 0.03;
    grid.addPoint(p, cov);
    const auto *cell = grid.findCell({0, 0, 0});
    assert(cell != nullptr && (cell->S2 - p * p.transpose()).norm() < 1e-12);
    assert((cell->covariance_sum - cov).norm() < 1e-12);
    std::cout << "[ok] full Eq.11 covariance is retained without polluting Eq.9 PCA\n";
  }

  {
    AdaptiveVoxelGrid grid(cfg);
    bool threw = false;
    try { (void)grid.keyOf(V3(std::numeric_limits<double>::max(), 0.0, 0.0)); }
    catch (const std::overflow_error &) { threw = true; }
    assert(threw);
    std::cout << "[ok] voxel-key overflow is rejected before integer conversion\n";
  }

  {
    AdaptiveVoxelGrid grid(cfg);
    addPlanePatch(grid, 0.1, 0.8, 0.1, 0.8, 0.2);
    AdaptivePlaneFit fit;
    assert(grid.queryPlane(V3(0.5, 0.5, 0.2), 1e-4 * M3::Identity(), fit));
    assert(fit.valid && fit.shell == 0 && fit.support_count == 25);
    assert(fit.kappa < cfg.kappa_threshold);
    assert((fit.covariance - (fit.lambda_min / 25.0) *
            Eigen::Matrix<double, 6, 6>::Identity()).norm() < 1e-12);
    std::cout << "[ok] first planar support accepted at shell 0\n";
  }

  {
    AdaptiveVoxelGrid grid(cfg);
    addPlanePatch(grid, 1.05, 0.8, 0.1, 0.8, 0.2);
    AdaptivePlaneFit fit;
    assert(grid.queryPlane(V3(0.9, 0.5, 0.2), 1e-4 * M3::Identity(), fit));
    assert(fit.shell == 1 && fit.support_count == 25);
    std::cout << "[ok] Chebyshev shell grows once and accepts first planar scale\n";
  }

  {
    AdaptiveVoxelGrid grid(cfg);
    addPlanePatch(grid, 0.08, 0.04, 0.08, 0.04, 0.2);
    AdaptivePlaneFit fit;
    assert(!grid.queryPlane(V3(0.9, 0.9, 0.2), 1e-4 * M3::Identity(), fit));
    std::cout << "[ok] 3*rho lateral support gate rejects plane extension\n";
  }

  {
    AdaptiveVoxelGrid grid(cfg);
    addPlanePatch(grid, 0.1, 0.8, 0.1, 0.8, 0.1);
    AdaptivePlaneFit fit;
    M3 low = 1e-6 * M3::Identity();
    assert(!grid.queryPlane(V3(0.5, 0.5, 0.3), low, fit));
    M3 high = low;
    high(2, 2) = 1e-2;
    assert(grid.queryPlane(V3(0.5, 0.5, 0.3), high, fit));
    assert(fit.residual_variance > 1e-2);
    std::cout << "[ok] association gate uses full world covariance\n";
  }

  {
    AdaptiveVoxelConfig sparse_cfg = cfg;
    sparse_cfg.n_pca = 25;
    AdaptiveVoxelGrid grid(sparse_cfg);
    for (int i = 0; i < 24; ++i) grid.addPoint(V3(0.1 + 0.01 * i, 0.2, 0.1));
    AdaptivePlaneFit fit;
    assert(!grid.queryPlane(V3(0.2, 0.2, 0.1), 1e-4 * M3::Identity(), fit));
    std::cout << "[ok] support below N_pca is never fitted\n";
  }

  std::cout << "ALL ADAPTIVE VOXEL V2 TESTS PASSED\n";
  return 0;
}
