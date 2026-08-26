#!/usr/bin/env python3
"""HILTI'22 official sparse-survey-point APE (headless replica of
evaluation.py's evaluate() protocol):

  - estimate is TUM (t x y z qx qy qz qw)
  - apply poletip calibration to the estimate (identity when the GT file
    name ends in 'imu.txt')
  - time-sync (max_diff = 2 s), Umeyama SE(3) align (no scale)
  - report translation APE RMSE

Usage: eval_hilti_ape.py est_tum.txt gt_tum.txt
"""
import argparse
import sys

import numpy as np


def load_tum(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            rows.append(
                (
                    float(p[0]),
                    np.array([float(p[1]), float(p[2]), float(p[3])]),
                    np.array([float(p[7]), float(p[4]), float(p[5]), float(p[6])]),
                )
            )
    rows.sort(key=lambda r: r[0])
    return rows


def quat_to_matrix(qwqxqyqz):
    w, x, y, z = qwqxqyqz
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def umeyama(P, Q):
    mu_p = P.mean(axis=0)
    mu_q = Q.mean(axis=0)
    H = (P - mu_p).T @ (Q - mu_q)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = mu_q - R @ mu_p
    return R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("est")
    ap.add_argument("gt")
    args = ap.parse_args()

    est = load_tum(args.est)
    gt = load_tum(args.gt)
    if not est or not gt:
        print("empty trajectory")
        sys.exit(2)

    calibration_type = args.gt.split("_")[-1].lower()
    if calibration_type == "imu.txt":
        T_imu_ref = np.eye(4)
    else:
        T_imu_ref = np.array(
            [
                [1, 0, 0, 0.059],
                [0, 1, 0, -0.00855],
                [0, 0, 1, 0.1964],
                [0, 0, 0, 1.0],
            ]
        )

    est_t = np.array([r[0] for r in est])
    est_p = np.zeros((len(est), 3))
    est_q = np.zeros((len(est), 4))
    for i, (_, p, q) in enumerate(est):
        R = quat_to_matrix(q)
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = p
        H = H @ T_imu_ref
        est_p[i] = H[:3, 3]
        est_q[i] = np.array(
            [
                np.trace(H[:3, :3]) + 1,
                0.0,
                0.0,
                0.0,
            ]
        )

    gt_t = np.array([r[0] for r in gt])
    gt_p = np.array([r[1] for r in gt])

    # time-sync: match each GT pose to the closest estimate pose within 2 s
    est_idx = np.searchsorted(est_t, gt_t)
    matched = []
    for gi, t in enumerate(gt_t):
        best = -1
        for cand in range(max(0, est_idx[gi] - 1), min(len(est_t), est_idx[gi] + 2)):
            if abs(est_t[cand] - t) <= 2.0:
                if best < 0 or abs(est_t[cand] - t) < abs(est_t[best] - t):
                    best = cand
        if best >= 0:
            matched.append((gi, best))
    if len(matched) < 3:
        print("too few synchronized poses")
        sys.exit(2)

    P = np.array([est_p[b] for _, b in matched])
    Q = np.array([gt_p[g] for g, _ in matched])
    R, t = umeyama(P, Q)
    errs = np.linalg.norm((R @ P.T).T + t - Q, axis=1)
    rmse = float(np.sqrt(np.mean(errs**2)))
    print(f"poses={len(matched)}")
    print(f"APE trans: rmse={rmse:.4f} mean={errs.mean():.4f} "
          f"median={np.median(errs):.4f} max={errs.max():.4f} (m)")


if __name__ == "__main__":
    main()
