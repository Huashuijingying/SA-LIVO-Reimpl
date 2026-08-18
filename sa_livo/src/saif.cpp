/*
 * saif.cpp — Subspace-Aware Information Fusion (SAIF) implementation.
 * See saif.h for the algorithm description and conventions.
 */

#include "saif.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

FusedInfo6 saifFuse(const InfoForm6 &LambdaL, const InfoForm6 &LambdaV, const double q, const double sigma_min,
                    const Eigen::Matrix<double, 6, 6> *prior_pose_info)
{
  FusedInfo6 out;
  out.eigenvals.resize(6);
  out.gate_weights.resize(6);
  out.Lambda_f.setZero();
  out.b_f.setZero();
  if (!LambdaL.Lambda.allFinite() || !LambdaL.b.allFinite() ||
      !LambdaV.Lambda.allFinite() || !LambdaV.b.allFinite() ||
      !std::isfinite(q) || !std::isfinite(sigma_min)) return out;

  // Joint information matrix / vector (Eq. 33–34)
  InfoForm6 sum = LambdaL;
  sum.addScaled(LambdaV, q);

  // SelfAdjointEig of the 6×6 joint information matrix (ascending λk)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(sum.Lambda);
  if (es.info() != Eigen::Success) return out;
  const Eigen::Matrix<double, 6, 6> U = es.eigenvectors();
  const Eigen::Matrix<double, 6, 1> lambda = es.eigenvalues();

  // Project the information vector into the joint eigenbasis (Eq. 34)
  Eigen::Matrix<double, 6, 1> b_prime = U.transpose() * sum.b;

  const double sigma_min_safe = std::max(sigma_min, 1e-6);
  Eigen::Matrix<double, 6, 1> lambda_tilde;
  Eigen::Matrix<double, 6, 1> b_tilde_prime;

  for (int k = 0; k < 6; k++)
  {
    // Clamp tiny negative eigenvalues from numerical noise
    const double lam = std::max(lambda(k), 0.0);
    const double sqrt_lam = std::sqrt(lam);

    // Linear-clamp soft gate (Eq. 35).  Diagnostic: relative-prior gate —
    // attenuate the measurement where its information is weaker than the IMU
    // prior's in the same direction (the prior supplies the natural scale).
    double g = std::min(sqrt_lam / sigma_min_safe, 1.0);
    if (prior_pose_info != nullptr)
    {
      const double lam_prior = std::max((U.transpose() * (*prior_pose_info) * U)(k, k), 0.0);
      const double rel = (lam_prior > 1e-12) ? std::sqrt(lam / lam_prior) : 1.0;
      g = std::min(rel, 1.0);
    }

    // Gate the eigenvalue and the projected information vector (Eq. 36)
    lambda_tilde(k) = g * lam;
    b_tilde_prime(k) = g * b_prime(k);

    out.eigenvals[k] = lam;
    out.gate_weights[k] = g;
  }

  // Rotate back to the pose error basis (Eq. 37) — PSD by construction
  out.Lambda_f = U * lambda_tilde.asDiagonal() * U.transpose();
  out.b_f = U * b_tilde_prime;
  out.ok = out.Lambda_f.allFinite() && out.b_f.allFinite();

  return out;
}
