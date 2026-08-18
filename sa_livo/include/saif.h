/*
 * saif.h — Subspace-Aware Information Fusion (SAIF)
 *
 * SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with Subspace-Aware
 * Degeneracy Handling (arXiv:2606.25699, IEEE T-RO 2026 submission).
 *
 * Implements Algorithm 1 of the paper:
 *   Λsum = ΛL + q·ΛV, bsum = bL + q·bV
 *   [U, diag(λk)] = SelfAdjointEig(Λsum)            // joint eigenbasis
 *   b′ = Uᵀ bsum
 *   g(k) = min(√λk / σmin, 1)                       // linear-clamp soft gate
 *   λ̃k  = g(k)·λk,  b̃′(k) = g(k)·b′(k)
 *   Λf  = U diag(λ̃) Uᵀ,  bf = U b̃′                 // guaranteed PSD
 *
 * Also provides the unified joint InEKF update solve (Sect. VII-E):
 *   K  = (Λ̃f + P⁻¹)⁻¹                    (Λ̃f = 18×18 embedding of Λf)
 *   G6 = K[:,0:6]·Λf
 *   δx = K[:,0:6]·bf + (x̂0 ⊟ x̂κ) − G6·(x̂0 ⊟ x̂κ)[0:6]
 *
 * Error-state convention (consistent with the FAST-LIVO2 code base):
 *   rotation error is applied as R ← R·Exp(δϕ) (right multiplication),
 *   position/velocity errors are Euclidean.  All information forms live in
 *   the 6-dim pose block [δϕ; δp].
 *
 * Sign convention: for a scalar residual r with Jacobian h = ∂r/∂δx, we accumulate
 *   Λ += w·h·hᵀ,   b += w·h·(−r),
 * which reproduces FAST-LIVO2's proven HᵀR⁻¹H / HᵀR⁻¹(−r) information form.
 */

#ifndef SA_LIVO_SAIF_H
#define SA_LIVO_SAIF_H

#include <Eigen/Dense>
#include <vector>

// 6-DoF information form: pose-block information matrix + information vector
struct InfoForm6
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, 6, 6> Lambda;  // information matrix
  Eigen::Matrix<double, 6, 1> b;       // information vector

  InfoForm6() { reset(); }
  void reset()
  {
    Lambda.setZero();
    b.setZero();
  }

  // Accumulate one scalar residual.
  //   h: 6×1 Jacobian column (h = ∂r/∂δx)
  //   w: scalar weight (1/σ² or w̃ for visual)
  //   r: signed residual
  inline void accumulate(const Eigen::Matrix<double, 6, 1> &h, const double w, const double r)
  {
    Lambda += w * h * h.transpose();
    b -= w * h * r;
  }

  // Sum with quality-gated visual form: (Λsum, bsum) = this + q·(ΛV, bV)
  inline void addScaled(const InfoForm6 &other, const double q)
  {
    Lambda += q * other.Lambda;
    b += q * other.b;
  }
};

// Rebase an information vector frozen at x0 to the current accumulated
// increment delta0, with both represented in the same fixed x0 tangent.
inline InfoForm6 rebaseFrozenInfo(const InfoForm6 &frozen,
                                  const Eigen::Matrix<double, 6, 1> &delta0)
{
  InfoForm6 current = frozen;
  current.b -= frozen.Lambda * delta0;
  return current;
}

// Fused information form output of SAIF
struct FusedInfo6
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, 6, 6> Lambda_f;  // fused, guaranteed PSD
  Eigen::Matrix<double, 6, 1> b_f;
  std::vector<double> eigenvals;         // λk before gating (ascending)
  std::vector<double> gate_weights;      // g(k)
  bool ok = false;
};

// Algorithm 1: Subspace-Aware Information Fusion (linear-clamp soft gate).
//   ΛL, bL : LiDAR information form
//   ΛV, bV : visual information form (already pre-scaled by quality factor q
//            when q is folded in by the caller; pass q here to fold it in)
//   σmin   : information-amplitude threshold (paper: σmin = 1)
FusedInfo6 saifFuse(const InfoForm6 &LambdaL, const InfoForm6 &LambdaV, const double q, const double sigma_min,
                    const Eigen::Matrix<double, 6, 6> *prior_pose_info = nullptr);

// Unified joint InEKF update (Sect. VII-E / Algorithm 2 step 6).
//   fused   : SAIF output (Λf, bf)
//   P_inv   : N×N inverse prior covariance (N = DIM_STATE; the FAST-LIVO2
//             code base carries a 19-dim state with an inert exposure entry,
//             the paper's state is 18-dim — the solve works for either)
//   x0_minus_xk : (x̂0 ⊟ x̂κ) ∈ R^N — displacement of the current iterate
//                 from the propagated state (StatesGroup::operator-)
// Returns the N-dim correction δx with x̂κ+1 = x̂κ ⊞ δx, plus the compact
// gain G6 = K[:,0:6]·Λf needed for the covariance update (I − Ḡ)P.
template <int N>
struct JointSolveT
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, N, 1> dx;          // state correction
  Eigen::Matrix<double, N, 6> G6;          // compact gain for covariance update
  Eigen::Matrix<double, N, N> K;           // precision-gain (optional use)
  bool ok = false;
};

template <int N>
JointSolveT<N> solveJointUpdate(const FusedInfo6 &fused,
                                const Eigen::Matrix<double, N, N> &P_inv,
                                const Eigen::Matrix<double, N, 1> &x0_minus_xk)
{
  JointSolveT<N> out;

  // Embed the fused 6×6 information matrix in the N-dim state space (Eq. 38)
  Eigen::Matrix<double, N, N> Lambda_tilde = Eigen::Matrix<double, N, N>::Zero();
  Lambda_tilde.template block<6, 6>(0, 0) = fused.Lambda_f;

  const Eigen::Matrix<double, N, N> precision =
      0.5 * (Lambda_tilde + P_inv + (Lambda_tilde + P_inv).transpose());
  if (!precision.allFinite() || !fused.ok) return out;
  Eigen::LDLT<Eigen::Matrix<double, N, N>> ldlt(precision);
  if (ldlt.info() != Eigen::Success) return out;
  out.K = ldlt.solve(Eigen::Matrix<double, N, N>::Identity());
  if (ldlt.info() != Eigen::Success || !out.K.allFinite()) return out;

  // Compact gain (Eq. 39): G6 = K[:,0:6]·Λf
  out.G6 = out.K.template block<N, 6>(0, 0) * fused.Lambda_f;

  // Joint update (Eq. 40)
  //   δx = K[:,0:6]·bf + (x̂0 ⊟ x̂κ) − G6·(x̂0 ⊟ x̂κ)[0:6]
  out.dx = out.K.template block<N, 6>(0, 0) * fused.b_f + x0_minus_xk -
           out.G6 * x0_minus_xk.template block<6, 1>(0, 0);
  out.ok = true;

  return out;
}

#endif  // SA_LIVO_SAIF_H
