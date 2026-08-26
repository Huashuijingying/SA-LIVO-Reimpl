#!/usr/bin/env python3
"""Oxford Spires APE evaluation: rigid SE(3) (Umeyama) alignment, translation RMSE.
Usage: eval_spires_ape.py est_tum.txt gt_tum.txt
"""
import argparse, sys
import numpy as np

def load_tum(path):
    rows=[]
    with open(path) as f:
        for line in f:
            line=line.strip()
            if not line or line.startswith('#'): continue
            p=line.split()
            if len(p)<8: continue
            rows.append((float(p[0]), np.array([float(p[1]),float(p[2]),float(p[3])])))
    rows.sort(key=lambda r:r[0])
    return rows

def umeyama_rigid(P,Q):
    muP=P.mean(0); muQ=Q.mean(0)
    Pc=P-muP; Qc=Q-muQ
    H=Pc.T@Qc
    U,S,Vt=np.linalg.svd(H)
    d=np.sign(np.linalg.det(Vt.T@U.T))
    R=Vt.T@np.diag([1,1,d])@U.T
    t=muQ-R@muP
    return R,t

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('est'); ap.add_argument('gt')
    a=ap.parse_args()
    est=load_tum(a.est); gt=load_tum(a.gt)
    gt_t=np.array([r[0] for r in gt]); gt_p=np.array([r[1] for r in gt])
    t=np.array([r[0] for r in est]); P=np.array([r[1] for r in est])
    lo,hi=gt_t[0],gt_t[-1]
    m=(t>=lo)&(t<=hi); t=t[m]; P=P[m]
    if len(t)<10:
        print('too few matched poses'); sys.exit(2)
    Q=np.array([np.interp(t,gt_t,gt_p[:,j]) for j in range(3)]).T
    R,s=umeyama_rigid(P,Q)
    errs=np.linalg.norm((R@P.T).T+s-Q,axis=1)
    print(f'poses={len(t)} overlap={t[0]:.3f}..{t[-1]:.3f}')
    print(f'APE trans: rmse={np.sqrt(np.mean(errs**2)):.4f} mean={errs.mean():.4f} median={np.median(errs):.4f} max={errs.max():.4f} (m)')

if __name__=='__main__':
    main()
