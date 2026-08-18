// SPDX-License-Identifier: GPL-2.0-only
//
// saif_test.cpp — unit test for the SAIF fusion module (Algorithm 1 of the
// SA-LIVO paper). Verifies:
//   1. Fused matrix is PSD by construction
//   2. Strong directions pass at full weight (g(k) = 1), weak directions are
//      linearly attenuated (g(k) = sqrt(λk)/σmin)
//   3. VIO rescues a degenerate LiDAR direction (joint eigenvalue rises
//      above the gate threshold)
//   4. Joint solve produces the correct Newton step on a toy problem
//   5. Degenerate limit: ΛL ≈ 0, q ≈ 0 → fused form ≈ 0 (IMU prior governs)
//
// Build: g++ -I../include saif_test.cpp ../src/saif.cpp -o saif_test && ./saif_test

#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include "saif.h"

static int failures = 0;
#define CHECK(cond, msg)                                                          \
  do                                                                              \
  {                                                                               \
    if (!(cond))                                                                  \
    {                                                                             \
      printf("  [FAIL] %s\n", msg);                                               \
      failures++;                                                                 \
    }                                                                             \
    else                                                                          \
    {                                                                             \
      printf("  [ok]   %s\n", msg);                                               \
    }                                                                             \
  } while (0)

static double minEig(const Eigen::Matrix<double, 6, 6> &M)
{
  return Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>>(M).eigenvalues().minCoeff();
}

int main()
{
  const double sigma_min = 1.0;

  // --- Case 1: well-constrained LiDAR, no vision ---------------------------
  {
    printf("Case 1: well-constrained LiDAR only (gate must be inactive)\n");
    InfoForm6 L;
    for (int i = 0; i < 300; i++)
    {
      Eigen::Matrix<double, 6, 1> h;
      h.setRandom();
      h *= 0.5;
      L.accumulate(h, 1.0, 0.02 * (i % 3 - 1));  // w = 1, small residuals
    }
    InfoForm6 V;
    FusedInfo6 f = saifFuse(L, V, 0.0, sigma_min);
    // 300 observations with |h| ~ 0.5 → λk ~ 300*0.25 = 75 → sqrt(75) >> 1
    CHECK(minEig(f.Lambda_f) >= 0.0, "Λf is PSD");
    double gmin = 1e9;
    for (double g : f.gate_weights) gmin = std::min(gmin, g);
    CHECK(gmin > 0.99, "all gate weights = 1 (fully observable)");
  }

  // --- Case 2: degenerate direction (pure translation z unobserved) ---------
  {
    printf("Case 2: LiDAR degenerate along z (linear-clamp attenuation)\n");
    InfoForm6 L;
    // strong independent constraints in x, y translation and rotation;
    // only a tiny residual information in z
    for (int i = 0; i < 200; i++)
    {
      Eigen::Matrix<double, 6, 1> h;
      h.setZero();
      h(0) = 0.5 * std::sin(i);  // rot x
      h(1) = 0.5 * std::cos(i);  // rot y
      h(3) = 0.8 * std::sin(0.31 * i);  // trans x
      h(4) = 0.8 * std::cos(0.31 * i);  // trans y
      h(5) = 0.03;                      // trans z: weak but nonzero
      h(2) = 0.02 * std::sin(0.13 * i); // rot z: weak but nonzero
      L.accumulate(h, 1.0, 0.01);
    }
    InfoForm6 V;
    FusedInfo6 f = saifFuse(L, V, 0.0, sigma_min);
    CHECK(minEig(f.Lambda_f) >= 0.0, "Λf is PSD");
    // eigenvalues sorted ascending; the weakest (z-translation) should be
    // linearly clamped: g = sqrt(λz)/σmin ∈ (0, 1)
    const double lam_min = f.eigenvals[0];
    const double g_weak = f.gate_weights[0];
    printf("    λmin = %.4f, g(weakest) = %.4f\n", lam_min, g_weak);
    CHECK(lam_min < 1.0, "degenerate eigenvalue below σmin²");
    CHECK(g_weak < 1.0 && g_weak > 0.0, "weak direction linearly attenuated (0 < g < 1)");
    CHECK(std::fabs(g_weak - std::sqrt(lam_min) / sigma_min) < 1e-9,
          "gate weight equals sqrt(λ)/σmin (linear-clamp)");
    // strong directions keep full weight
    double gmax = 0.0;
    for (double g : f.gate_weights) gmax = std::max(gmax, g);
    CHECK(gmax == 1.0, "strong directions at full weight");
  }

  // --- Case 3: VIO rescues the degenerate LiDAR direction -------------------
  {
    printf("Case 3: VIO lifts a degenerate LiDAR direction above the gate\n");
    InfoForm6 L;
    for (int i = 0; i < 200; i++)
    {
      Eigen::Matrix<double, 6, 1> h;
      h.setZero();
      h(0) = 0.5 * std::sin(i);
      h(1) = 0.5 * std::cos(i);
      h(3) = 0.8 * std::sin(0.31 * i);
      h(4) = 0.8 * std::cos(0.31 * i);
      L.accumulate(h, 1.0, 0.01);
    }
    InfoForm6 V;  // visual info constrains the two LiDAR-degenerate directions
    for (int i = 0; i < 100; i++)
    {
      Eigen::Matrix<double, 6, 1> h;
      h.setZero();
      h(5) = 0.6 * std::sin(0.7 * i) + 0.1;  // trans z
      h(2) = 0.3 * std::cos(0.3 * i);        // rot z
      V.accumulate(h, 0.05, 0.01);           // w = 0.05
    }
    const double q = 1.0;
    FusedInfo6 f_lidar_only = saifFuse(L, InfoForm6(), 0.0, sigma_min);
    FusedInfo6 f_joint = saifFuse(L, V, q, sigma_min);
    const double lam_weak_l = f_lidar_only.eigenvals[0];
    const double lam_weak_j = f_joint.eigenvals[0];
    printf("    λmin(LiDAR only) = %.4f → g = %.3f\n", lam_weak_l, f_lidar_only.gate_weights[0]);
    printf("    λmin(joint)      = %.4f → g = %.3f\n", lam_weak_j, f_joint.gate_weights[0]);
    CHECK(minEig(f_lidar_only.Lambda_f) >= 0.0, "Λf (LiDAR only) is PSD");
    CHECK(minEig(f_joint.Lambda_f) >= 0.0, "Λf (joint) is PSD");
    CHECK(lam_weak_j > lam_weak_l, "joint matrix eigenvalue rises above LiDAR-only");
    CHECK(f_joint.gate_weights[0] > f_lidar_only.gate_weights[0],
          "visual contribution rescues the degenerate direction (higher gate weight)");
  }

  // --- Case 4: Newton-step correctness of the joint solve -------------------
  {
    printf("Case 4: joint solve produces the Newton step of the combined cost\n");
    // Six independent residuals spanning R^6: r_i = a_iᵀx, x* = 0.
    // Three residuals feed the LiDAR form, three the visual form.
    Eigen::Matrix<double, 6, 6> A;
    A << 0.7, -0.3, 0.2, 0.9, -0.4, 0.5,
         -0.2, 0.8, 0.6, -0.5, 0.3, 0.9,
         0.4, 0.1, -0.7, 0.2, 0.8, -0.6,
         -0.6, 0.5, 0.3, 0.7, 0.1, 0.4,
         0.9, 0.2, -0.4, -0.3, 0.6, 0.7,
         0.3, -0.6, 0.5, 0.4, -0.2, 0.1;
    Eigen::Matrix<double, 6, 1> r;
    r << 1.7, -0.9, 0.4, -1.2, 0.8, -0.5;
    const double w1 = 2.0, w2 = 0.5;
    InfoForm6 L, V;
    for (int i = 0; i < 3; i++) L.accumulate(A.row(i).transpose(), w1, r(i));
    for (int i = 3; i < 6; i++) V.accumulate(A.row(i).transpose(), w2, r(i));
    const double q = 1.0;
    FusedInfo6 f = saifFuse(L, V, q, 1e-9);  // sigma_min tiny → gate inactive
    CHECK(minEig(f.Lambda_f) > -1e-9, "Λf is PSD");

    // exact Newton step of ½Σ w r²: δx = (Σ JᵀWJ)⁻¹ Σ JᵀW(−r)
    Eigen::Matrix<double, 6, 6> HtWH = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    for (int i = 0; i < 3; i++)
    {
      HtWH += w1 * A.row(i).transpose() * A.row(i);
      g += w1 * A.row(i).transpose() * (-r(i));
    }
    for (int i = 3; i < 6; i++)
    {
      HtWH += q * w2 * A.row(i).transpose() * A.row(i);
      g += q * w2 * A.row(i).transpose() * (-r(i));
    }
    const Eigen::Matrix<double, 6, 1> dx_exact = HtWH.inverse() * g;
    // solve via the joint update. A tiny positive-definite prior keeps K
    // invertible (the real system's covariance is always PD); the prior
    // offset is zero so the prior does not bias the step.
    const double eps_prior = 1e-12;
    Eigen::Matrix<double, 19, 19> P_inv = eps_prior * Eigen::Matrix<double, 19, 19>::Identity();
    Eigen::Matrix<double, 19, 1> vec = Eigen::Matrix<double, 19, 1>::Zero();
    auto js = solveJointUpdate<19>(f, P_inv, vec);
    const Eigen::Matrix<double, 6, 1> dx_code = js.dx.block<6, 1>(0, 0);
    printf("    dx_exact = %.4f %.4f %.4f ...  dx_code = %.4f %.4f %.4f ...\n",
           dx_exact(0), dx_exact(1), dx_exact(2), dx_code(0), dx_code(1), dx_code(2));
    CHECK((dx_code - dx_exact).norm() < 1e-6, "joint solve matches the analytic Newton step");
  }

  // --- Case 5: concurrent degradation → graceful LIO-only / prior fallback --
  {
    printf("Case 5: concurrent degradation (q = 0, weak ΛL) → near-zero Λf\n");
    InfoForm6 L;
    for (int i = 0; i < 10; i++)
    {
      Eigen::Matrix<double, 6, 1> h;
      h.setRandom();
      h *= 0.02;
      L.accumulate(h, 1.0, 0.01);
    }
    InfoForm6 V;
    FusedInfo6 f = saifFuse(L, V, 0.0, sigma_min);
    CHECK(minEig(f.Lambda_f) >= 0.0, "Λf is PSD");
    CHECK(f.Lambda_f.norm() < L.Lambda.norm(), "fused form attenuated below LiDAR-only");
    CHECK(f.gate_weights[5] < 0.2, "weakest direction strongly attenuated toward the IMU prior");
  }

  // --- Case 6: frozen information is not applied repeatedly ----------------
  {
    printf("Case 6: frozen b is rebased across non-zero multi-iteration vec\n");
    InfoForm6 frozen;
    frozen.Lambda = 100.0 * Eigen::Matrix<double, 6, 6>::Identity();
    frozen.b.setZero();
    frozen.b(0) = 10.0;
    const Eigen::Matrix<double, 6, 1> expected =
        (Eigen::Matrix<double, 6, 6>::Identity() + frozen.Lambda).inverse() * frozen.b;
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Matrix<double, 19, 19> P_inv =
        Eigen::Matrix<double, 19, 19>::Identity();
    for (int iter = 0; iter < 5; ++iter)
    {
      const InfoForm6 current = rebaseFrozenInfo(frozen, delta);
      const FusedInfo6 fused = saifFuse(InfoForm6(), current, 1.0, 1e-9);
      Eigen::Matrix<double, 19, 1> vec = Eigen::Matrix<double, 19, 1>::Zero();
      vec.block<6, 1>(0, 0) = -delta;
      const auto js = solveJointUpdate<19>(fused, P_inv, vec);
      delta += js.dx.block<6, 1>(0, 0);
    }
    CHECK((delta - expected).norm() < 1e-10,
          "rebased frozen information converges once instead of Kmax-fold application");
  }

  printf("\n%s (%d failures)\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
