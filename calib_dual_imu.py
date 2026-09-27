#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calib_dual_imu.py — CJ02-IMU 与 Mid-360 内置 IMU 双 IMU 外参标定 (全 6-DoF, 不预设安装关系)

原理:
  1) 两路 IMU 时间对齐 (陀螺模长互相关) + 单位自检 (g/m·s⁻², deg/s/rad/s)
  2) 陀螺 Kabsch:  w_builtin = R * w_cj02        -> 旋转 R (CJ02 -> 内置IMU)
  3) 杆臂方程 LS:  f1 - R*f2 = ([a×] + [w×][w×]) p + bias  -> 平移 p (CJ02 原点在内置IMU系)
  4) 复合出厂外参 (内置IMU->雷达: T=[-0.011,-0.02329,0.04412], R=I) 得 FAST-LIO 外参

用法:
  python3 calib_dual_imu.py <bag> [--t1 TOPIC1] [--t2 TOPIC2] [--apply YAML]

参数:
  bag      必填  rosbag 文件路径
  --t1     可选  内置 IMU 话题 (默认 /drone_0/livox/imu, 200 Hz)
  --t2     可选  CJ02 话题    (默认 /drone_0/cj02/imu/data_raw, 800 Hz)
  --apply  可选  标定结果直接写入该 yaml (默认只打印)
"""

import argparse
import sys
import numpy as np

G = 9.80665
# Mid-360 内置 IMU -> 雷达 出厂外参 (与 mid360.yaml 一致)
T_FACTORY = np.array([-0.011, -0.02329, 0.04412])
R_FACTORY = np.eye(3)

def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])

def read_imu(bagpath, topic):
    import rosbag
    t, w, a = [], [], []
    with rosbag.Bag(bagpath, 'r') as bag:
        for _, msg, _ts in bag.read_messages(topics=[topic]):
            t.append(msg.header.stamp.to_sec())
            w.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            a.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    if not t:
        print("[E] 话题 %s 在 bag 中无消息: %s" % (topic, bagpath)); sys.exit(1)
    return np.asarray(t), np.asarray(w), np.asarray(a)

def est_time_offset(t1, s1, t2, s2, fs=100.0):
    """互相关估计 t2 相对 t1 的时间偏移 (秒)"""
    t0 = max(t1[0], t2[0]); t1e = min(t1[-1], t2[-1])
    if t1e - t0 < 2.0:
        print("[E] 两路数据重叠时间不足 2s"); sys.exit(1)
    grid = np.arange(t0, t1e, 1.0 / fs)
    g1 = np.interp(grid, t1, np.linalg.norm(s1, axis=1))
    g2 = np.interp(grid, t2, np.linalg.norm(s2, axis=1))
    g1 -= g1.mean(); g2 -= g2.mean()
    n = min(len(g1), len(g2))
    corr = np.correlate(g1[:n], g2[:n], mode='full')
    lag = np.argmax(corr) - (n - 1)
    return lag / fs

def euler_deg(R):
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    pitch = np.degrees(np.arcsin(max(-1.0, min(1.0, -R[2, 0]))))
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return roll, pitch, yaw

def smooth(x, win):
    if win < 3: return x
    k = np.ones(win) / win
    return np.convolve(x, k, mode='same')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--t1", default="/drone_0/livox/imu")
    ap.add_argument("--t2", default="/drone_0/cj02/imu/data_raw")
    ap.add_argument("--apply", default=None, help="把结果写入此 yaml 文件")
    args = ap.parse_args()

    print("[1] 读取 bag ...")
    t1, w1, a1 = read_imu(args.bag, args.t1)
    t2, w2, a2 = read_imu(args.bag, args.t2)
    dur = min(t1[-1], t2[-1]) - max(t1[0], t2[0])
    print("    内置IMU: %d 条 (%.0f Hz)  CJ02: %d 条 (%.0f Hz)  重叠 %.1f s"
          % (len(t1), (len(t1) - 1) / (t1[-1] - t1[0]), len(t2),
             (len(t2) - 1) / (t2[-1] - t2[0]), dur))

    # ---- 单位自检: 静止段加速度模长 ----
    q = np.linalg.norm(a1, axis=1)
    # 用 |w| 找静止段 (前后各找)
    wnorm1 = np.linalg.norm(w1, axis=1)
    static_mask = wnorm1 < 0.02
    w2_static = np.linalg.norm(w2, axis=1) < 0.02   # CJ02 用自身陀螺判静止
    if static_mask.sum() > 50:
        m1 = np.median(np.linalg.norm(a1[static_mask], axis=1))
    else:
        m1 = np.median(q)
    if w2_static.sum() > 50:
        m2 = np.median(np.linalg.norm(a2[w2_static], axis=1))
    else:
        m2 = np.median(np.linalg.norm(a2, axis=1))
    for name, m in (("内置IMU", m1), ("CJ02", m2)):
        if 0.8 < m < 1.4:
            print("    [单位] %s 加速度疑似 g 单位 (%.2f) — 请检查驱动!" % (name, m))
        elif 8.0 < m < 11.5:
            print("    [单位] %s 加速度 m/s^2 OK (静止模长 %.2f)" % (name, m))
        else:
            print("    [W] %s 静止加速度模长异常: %.2f" % (name, m))

    # ---- 时间偏移 + 最近邻配对 ----
    off = est_time_offset(t1, w1, t2, w2)
    print("[2] 时间偏移 (CJ02 相对内置): %+.1f ms" % (off * 1000))
    t2c = t2 + off
    idx = np.searchsorted(t2c, t1)
    idx = np.clip(idx, 1, len(t2c) - 1)
    left = np.abs(t1 - t2c[idx - 1]); right = np.abs(t2c[idx] - t1)
    idx = np.where(left < right, idx - 1, idx)
    gap = np.abs(t1 - t2c[idx])
    ok = gap < 0.005  # 配对阈值 5 ms
    print("    有效配对 %d / %d" % (ok.sum(), len(t1)))
    if ok.sum() < 500:
        print("[E] 有效配对样本过少, 标定无法进行"); sys.exit(1)
    w2p = w2[idx[ok]]; a2p = a2[idx[ok]]; w1p = w1[ok]; a1p = a1[ok]; t1p = t1[ok]

    # ---- 陀螺去偏 (静止段) ----
    w1b = w1p - np.median(w1p[wnorm1[ok] < 0.02], axis=0) if (wnorm1[ok] < 0.02).sum() > 50 else w1p
    w2n = np.linalg.norm(w2p, axis=1)
    w2b = w2p - np.median(w2p[w2n < np.deg2rad(2)], axis=0) if (w2n < np.deg2rad(2)).sum() > 50 else w2p

    # ---- 旋转: Kabsch (w1 = R w2) ----
    sel = np.linalg.norm(w1b, axis=1) > 0.05  # rad/s
    if sel.sum() < 200:
        print("[E] 旋转激励不足 (|w|>0.05rad/s 的样本仅 %d) — 请按动作要领绕三轴大幅慢转" % sel.sum())
        sys.exit(1)
    A_, B_ = w1b[sel], w2b[sel]
    H = A_.T @ B_
    U, S, Vt = np.linalg.svd(H)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1; R = U @ Vt
    res_rot = np.sqrt(np.mean(np.sum((A_ - (R @ B_.T).T) ** 2, axis=1)))
    r, p, y = euler_deg(R)
    ang = np.degrees(np.arccos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))
    print("[3] 旋转 R (CJ02->内置IMU): roll=%+.2f pitch=%+.2f yaw=%+.2f deg (总偏差 %.2f deg), 拟合残差 %.4f rad/s, 样本 %d"
          % (r, p, y, ang, res_rot, sel.sum()))

    # ---- 平移: 杆臂方程 LS ----
    # alpha = dw1/dt (平滑后中心差分)
    dt = np.gradient(t1p)
    w1s = np.vstack([smooth(w1b[:, k], 7) for k in range(3)]).T
    alpha = np.vstack([np.gradient(w1s[:, k], t1p) for k in range(3)]).T
    y = a1p - (R @ a2p.T).T   # f1 - R f2
    rows, ys = [], []
    for i in range(len(t1p)):
        Gi = skew(alpha[i]) + skew(w1b[i]) @ skew(w1b[i])
        rows.append(Gi); ys.append(y[i])
    Gmat = np.vstack(rows)                      # 3N x 3
    Amat = np.hstack([Gmat, np.tile(np.eye(3), (len(ys), 1))])  # 3N x 6 (p + bias)
    yvec = np.vstack(ys)
    exc = np.linalg.norm(Gmat, axis=1)
    if np.median(exc) < 1e-3:
        print("[E] 平移激励不足 (杆臂回归量过小) — 需要更多带角加速度的晃动/翻转")
        sys.exit(1)
    yflat = np.concatenate(ys)
    sol, res, rank, sv = np.linalg.lstsq(Amat, yflat, rcond=None)
    if rank < 6:
        print("[W] LS 秩亏 (%d/6), 平移结果可能不可靠" % rank)
    tvec = sol[:3]; bias = sol[3:]
    res_a = yflat - Amat @ sol
    print("[4] 平移 p (CJ02 原点在内置IMU系): [%.4f, %.4f, %.4f] m" % tuple(tvec))
    print("    陀螺模长激励中位数 %.3f rad/s, 加速度 LS 残差 RMS %.3f m/s^2" % (np.median(exc), np.sqrt(np.mean(res_a ** 2))))

    # ---- 复合出厂外参 -> FAST-LIO (IMU 在雷达系中的位姿) ----
    ext_R = R_FACTORY @ R
    ext_T = R_FACTORY @ tvec + T_FACTORY
    er, ep, ey = euler_deg(ext_R)
    print("[5] FAST-LIO 外参 (CJ02 在雷达系):")
    print("    extrinsic_T: [ %.5f, %.5f, %.5f ]" % tuple(ext_T))
    print("    extrinsic_R: [ %.5f, %.5f, %.5f," % tuple(ext_R[0]))
    print("                   %.5f, %.5f, %.5f," % tuple(ext_R[1]))
    print("                   %.5f, %.5f, %.5f ]" % tuple(ext_R[2]))
    print("    (相对雷达 rpy: %+.2f / %+.2f / %+.2f deg)" % (er, ep, ey))
    if abs(tvec[0]) > 0.5 or abs(tvec[1]) > 0.5 or abs(tvec[2]) > 0.5:
        print("[W] 平移超过 0.5 m, 与'装在雷达附近'不符, 结果可疑 — 建议加大旋转激励后重标")
    if ang > 45:
        print("[i] 旋转偏差较大 (>45 deg), 属于斜装; FAST-LIO 初始旋转需准确, 该值将直接写入配置")

    if args.apply:
        import re
        with open(args.apply) as f: txt = f.read()
        txt = re.sub(r"extrinsic_T:\s*\[.*?\]", "extrinsic_T: [%.5f, %.5f, %.5f]" % tuple(ext_T), txt, flags=re.S)
        txt = re.sub(r"extrinsic_R:\s*\[\s*[^\]]*\]",
                     "extrinsic_R: [ %.5f, %.5f, %.5f,\n                   %.5f, %.5f, %.5f,\n                   %.5f, %.5f, %.5f ]"
                     % (ext_R[0,0], ext_R[0,1], ext_R[0,2], ext_R[1,0], ext_R[1,1], ext_R[1,2], ext_R[2,0], ext_R[2,1], ext_R[2,2]), txt, flags=re.S)
        with open(args.apply, "w") as f: f.write(txt)
        print("[6] 已写入 %s (extrinsic_est_en 保持 true 供在线精修)" % args.apply)

if __name__ == "__main__":
    main()
