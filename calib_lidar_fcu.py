#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
静态重力对齐标定: 求 雷达IMU系 -> 飞控IMU系 的旋转 R_L2F.
原理: 静止时两个IMU的 linear_acceleration 都指向"天"(重力反力).
      同一物理方向在两坐标系下满足 g_F = R_L2F * g_L.
      多姿态采样后用 Kabsch/SVD 解最优旋转.
用法: 摆一个姿态 -> 回车采样; 采>=4个不同姿态后输入 q 解算.
"""
import rospy, sys, threading, numpy as np
from sensor_msgs.msg import Imu

LIDAR_TOPIC = "/drone_0/livox/imu"
FCU_TOPIC   = "/drone_0/mavros/imu/data"
SAMPLE_SEC  = 2.0

class Collector:
    def __init__(self):
        self.lidar_buf = []
        self.fcu_buf = []
        self.collecting = False
        self.lock = threading.Lock()
        rospy.Subscriber(LIDAR_TOPIC, Imu, self.cb_lidar, queue_size=200, tcp_nodelay=True)
        rospy.Subscriber(FCU_TOPIC,   Imu, self.cb_fcu,   queue_size=200, tcp_nodelay=True)

    def cb_lidar(self, m):
        if self.collecting:
            a = m.linear_acceleration
            with self.lock: self.lidar_buf.append([a.x, a.y, a.z])
    def cb_fcu(self, m):
        if self.collecting:
            a = m.linear_acceleration
            with self.lock: self.fcu_buf.append([a.x, a.y, a.z])

    def grab(self):
        with self.lock:
            self.lidar_buf = []; self.fcu_buf = []
        self.collecting = True
        rospy.sleep(SAMPLE_SEC)
        self.collecting = False
        with self.lock:
            L = np.array(self.lidar_buf); F = np.array(self.fcu_buf)
        if len(L) < 5 or len(F) < 5:
            print(f"  [!] 样本太少 lidar={len(L)} fcu={len(F)}, 重试"); return None
        gL = L.mean(axis=0); gF = F.mean(axis=0)
        # 标准差检查: 静止时应很小
        sL = L.std(axis=0).mean(); sF = F.std(axis=0).mean()
        gLn = gL/np.linalg.norm(gL); gFn = gF/np.linalg.norm(gF)
        print(f"  lidar g={gL.round(3)} |{np.linalg.norm(gL):.2f}| std={sL:.3f}  (n={len(L)})")
        print(f"  fcu   g={gF.round(3)} |{np.linalg.norm(gF):.2f}| std={sF:.3f}  (n={len(F)})")
        if sL > 0.5 or sF > 0.5:
            print("  [!] 抖动过大(可能没静止), 建议重采")
        return gLn, gFn

def kabsch(L, F):
    """求 R 使 F ~= R @ L (列向量), L,F: Nx3 单位向量"""
    H = L.T @ F
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1,1,d])
    R = Vt.T @ D @ U.T
    return R

def rot_to_euler_deg(R):
    # ZYX (yaw-pitch-roll)
    sy = -R[2,0]
    sy = max(-1,min(1,sy))
    pitch = np.degrees(np.arcsin(sy))
    roll  = np.degrees(np.arctan2(R[2,1], R[2,2]))
    yaw   = np.degrees(np.arctan2(R[1,0], R[0,0]))
    return roll, pitch, yaw

def main():
    rospy.init_node("calib_lidar_fcu", anonymous=True)
    c = Collector()
    print("="*60)
    print("静态重力标定 (雷达IMU -> 飞控IMU)")
    print("把飞机摆成不同姿态(水平/抬头/低头/左倾/右倾...), 每次回车采样")
    print("采 >=4 个明显不同姿态后, 输入 q 解算")
    print("="*60)
    rospy.sleep(1.0)
    Ls=[]; Fs=[]
    while not rospy.is_shutdown():
        cmd = input(f"[已采 {len(Ls)}] 摆好姿态后回车采样 (q=解算, r=清空): ").strip().lower()
        if cmd == 'q': break
        if cmd == 'r': Ls=[]; Fs=[]; print("  已清空"); continue
        res = c.grab()
        if res is None: continue
        gL, gF = res
        Ls.append(gL); Fs.append(gF)
    if len(Ls) < 3:
        print("样本不足3个, 退出"); return
    L = np.array(Ls); F = np.array(Fs)
    R = kabsch(L, F)
    # 残差: 把 gL 旋转后与 gF 的夹角
    errs=[]
    for i in range(len(L)):
        p = R @ L[i]
        cang = np.clip(np.dot(p, F[i]), -1, 1)
        errs.append(np.degrees(np.arccos(cang)))
    errs=np.array(errs)
    roll,pitch,yaw = rot_to_euler_deg(R)
    print("\n"+"="*60)
    print("标定结果 R_L2F (雷达 -> 飞控):")
    for row in R: print("  [{: .6f}, {: .6f}, {: .6f}]".format(*row))
    print(f"\n欧拉角(ZYX, deg): roll={roll:.2f}  pitch={pitch:.2f}  yaw={yaw:.2f}")
    print(f"重投影残差(deg): mean={errs.mean():.3f} max={errs.max():.3f}")
    print("  (注意: 静态重力法 yaw 不可观, yaw值仅供参考, 需动态法才准)")
    print("\n--- FAST-LIO extrinsic_R 格式 (行主序) ---")
    flat = R.flatten()
    print("    extrinsic_R: [ {:.6f}, {:.6f}, {:.6f},".format(*flat[0:3]))
    print("                   {:.6f}, {:.6f}, {:.6f},".format(*flat[3:6]))
    print("                   {:.6f}, {:.6f}, {:.6f}]".format(*flat[6:9]))
    print("="*60)

if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass
