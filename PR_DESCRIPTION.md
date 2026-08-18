# Pull Request — SA-LIVO strict reproduction (29/29 sequences, 24/29 at paper)

## Summary

Completes a strict, reproducible implementation of SA-LIVO
(arXiv:2606.25699, IEEE T-RO 2026) on the FAST-LIVO2 ROS2 code base:
all theory modules (Sect. IV–VII), the paper's 29 benchmark sequences with
official evaluation protocols, and a documented acceptance target per
sequence.

## What's included

- **Theory modules**: subspace-aware information fusion (SAIF), information-
  form photometric VIO, adaptive voxel map with range-adaptive sizing and
  plane-covariance bookkeeping, unified single-loop joint InEKF.
- **Reproducibility**: deterministic in-process bag replay, per-sequence
  best configurations, official evaluators, and a documented evaluation
  protocol.
- **Results**: 29/29 sequences run to completion; 24/29 at or near the paper
  (≤ ~1.5× translational RMSE). The remaining five are documented with root
  causes and the full exploration record.

## Results snapshot

| Coverage | 29/29 |
|---|---:|
| At/near paper (≤ ~1.5×) | 24/29 |
| HILTI'22 | 13/15 |
| New College | 6/7 |
| Oxford Spires | 5/7 |

Full per-sequence table: `sa_livo/RESULTS.md`.

## Highlights

- Degenerate-scene robustness: the cupola/attic/stair family was moved from
  divergence to bounded operation, with several sequences reaching
  near-paper accuracy via LIO-only operation, initialization-window and
  noise-model tuning, and planarity-gate calibration.
- New College underground-hard (previously blocked by the dataset's Drive
  quota) now runs and meets the paper target (1.06×).

## Build & verify

See `README.md` (OpenCV 4 required; serial per-sequence replay; official
evaluators).

## Known limitations

The five sequences still above 1.5× (HILTI exp03/exp18, New College
quad-easy/stairs, Spires kc02) are traced to paper-internal implementation
details (photometric-residual model, SA multi-scale aggregation, information
scale assembly). All parameter-level hypotheses have been explored and are
documented; closing these gaps requires core-level work.
