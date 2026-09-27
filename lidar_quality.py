#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动捕飞行 + 雷达数据质量评估
================================================
前提: 飞机由动捕稳住(实际近似静止)
推论: 此时 FAST-LIO 输出的任何"运动"都是误差,不需要坐标系对齐

同时测三件事:
  1. 雷达位姿误差  = FAST-LIO 测出的位移 - 动捕真值位移
  2. 雷达抖动      = 帧间跳变 RMS(等效速度噪声)
  3. 振动水平      = IMU 加速度模长的波动

用法:
  动捕起飞、悬停稳定后运行,采 30 秒:
    python3 ~/lidar_quality.py
"""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

IMU_TOPIC = '/drone_0/livox/imu'
LIO_TOPIC = '/drone_0/Odometry'


def rms(vals):
    if not vals:
        return 0.0
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def mean(vals):
    return sum(vals) / float(len(vals)) if vals else 0.0


def std(vals):
    if len(vals) < 2:
        return 0.0
    m = mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


class LidarQuality(object):
    def __init__(self):
        rospy.init_node('lidar_quality', anonymous=True)

        self.lio = []      # (t, x, y, z, roll, pitch, yaw)
        self.mocap = []    # (t, x, y, z)
        self.imu = []      # (t, ax, ay, az)
        self.done = False

        # 自动探测动捕话题
        self.mocap_topic = None
        try:
            for t, ty in rospy.get_published_topics():
                if ('vrpn' in t or 'mocap' in t) and 'PoseStamped' in ty:
                    self.mocap_topic = t
                    break
        except Exception:
            pass

        rospy.Subscriber(LIO_TOPIC, Odometry, self.lio_cb, queue_size=100)
        rospy.Subscriber(IMU_TOPIC, Imu, self.imu_cb, queue_size=200)
        if self.mocap_topic:
            rospy.Subscriber(self.mocap_topic, PoseStamped, self.mocap_cb, queue_size=100)

        rospy.Timer(rospy.Duration(2.0), self.report)
        rospy.on_shutdown(self.finish)

        print("=" * 72)
        print("雷达数据质量评估 (动捕飞行中)")
        print("=" * 72)
        print("雷达里程计: %s" % LIO_TOPIC)
        print("雷达IMU   : %s" % IMU_TOPIC)
        print("动捕真值  : %s" % (self.mocap_topic or '未发现动捕话题!'))
        print("")
        print("前提: 飞机被动捕稳住 -> FAST-LIO 测出的位移就是它的误差")
        print("采 30 秒左右后按 Ctrl+C")
        print("=" * 72)

    def lio_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        r, pt, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.lio.append((msg.header.stamp.to_sec(), p.x, p.y, p.z,
                         math.degrees(r), math.degrees(pt), math.degrees(y)))

    def mocap_cb(self, msg):
        p = msg.pose.position
        self.mocap.append((msg.header.stamp.to_sec(), p.x, p.y, p.z))

    def imu_cb(self, msg):
        a = msg.linear_acceleration
        self.imu.append((msg.header.stamp.to_sec(), a.x, a.y, a.z))

    def report(self, _evt):
        if len(self.lio) < 2:
            print("[等待] 没收到 %s" % LIO_TOPIC)
            return
        s0, s1 = self.lio[0], self.lio[-1]
        d = math.sqrt((s1[1] - s0[1]) ** 2 + (s1[2] - s0[2]) ** 2 + (s1[3] - s0[3]) ** 2)
        t = s1[0] - s0[0]
        mtxt = ''
        if len(self.mocap) >= 2:
            m0, m1 = self.mocap[0], self.mocap[-1]
            md = math.sqrt((m1[1] - m0[1]) ** 2 + (m1[2] - m0[2]) ** 2 + (m1[3] - m0[3]) ** 2)
            mtxt = ' | 动捕真值位移 %.3fm' % md
        print("雷达累计位移 %.3fm%s | 用时 %.1fs | 帧数 %d/%d"
              % (d, mtxt, t, len(self.lio), len(self.imu)))

    def finish(self):
        if self.done:
            return
        self.done = True
        print("")
        print("=" * 72)
        print("雷达数据质量汇总")
        print("=" * 72)

        # ---------- 1. 雷达位姿误差 ----------
        print("")
        print("[1] 雷达位姿误差 (飞机实际静止,以下全是误差)")
        if len(self.lio) >= 2:
            a, b = self.lio[0], self.lio[-1]
            dur = b[0] - a[0]
            dx = math.sqrt((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2 + (b[3] - a[3]) ** 2)
            dxy = math.sqrt((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)
            dz = b[3] - a[3]
            print("    时长        : %.1f 秒 (%d 帧)" % (dur, len(self.lio)))
            print("    FAST-LIO 位移: %.3f m  (水平 %.3f / 垂直 %+.3f)" % (dx, dxy, dz))
            print("    漂移速率    : %.3f m/s" % (dx / dur if dur > 0 else 0))
            yaw_drift = b[6] - a[6]
            print("    yaw 漂移    : %+.2f 度" % yaw_drift)
        else:
            print("    数据不足")
            dx = None

        # ---------- 2. 动捕真值对照 ----------
        if len(self.mocap) >= 2:
            a, b = self.mocap[0], self.mocap[-1]
            md = math.sqrt((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2 + (b[3] - a[3]) ** 2)
            print("")
            print("[2] 动捕真值 (飞机真实位移)")
            print("    实际位移    : %.3f m   <- 飞机基本没动,说明动捕确实稳住了" % md)
            if dx is not None:
                print("    雷达误差 / 真实位移 = %.1f 倍" % (dx / md if md > 0.001 else float('inf')))

        # ---------- 3. 雷达抖动 ----------
        print("")
        print("[3] 雷达抖动 (帧间跳变)")
        if len(self.lio) >= 3:
            steps = []
            for i in range(1, len(self.lio)):
                p0, p1 = self.lio[i - 1], self.lio[i]
                dt = p1[0] - p0[0]
                if dt <= 0:
                    continue
                d = math.sqrt((p1[1] - p0[1]) ** 2 + (p1[2] - p0[2]) ** 2 + (p1[3] - p0[3]) ** 2)
                steps.append(d / dt)   # 等效速度 m/s
            if steps:
                print("    帧间等效速度 RMS: %.3f m/s" % rms(steps))
                print("    最大瞬时跳变    : %.3f m/s" % max(steps))
                print("    (静止悬停时,这个值应该 < 0.05 m/s)")

        # ---------- 4. 振动 ----------
        print("")
        print("[4] 振动水平 (雷达IMU)")
        if len(self.imu) >= 10:
            mags = [math.sqrt(x[1] ** 2 + x[2] ** 2 + x[3] ** 2) for x in self.imu]
            m = mean(mags)
            s = std(mags)
            print("    加速度模长  : 均值 %.2f m/s^2 (静止应接近 9.81)" % m)
            print("    波动(标准差): %.2f m/s^2   <-- 振动强度" % s)
            print("    (室内悬停,这个值 < 1.0 算正常, > 2.0 就是振动很大)")

        # ---------- 结论 ----------
        print("")
        print("=" * 72)
        print("结论判断:")
        print("=" * 72)
        if dx is not None and len(self.mocap) >= 2:
            md = math.sqrt((self.mocap[-1][1] - self.mocap[0][1]) ** 2 +
                           (self.mocap[-1][2] - self.mocap[0][2]) ** 2 +
                           (self.mocap[-1][3] - self.mocap[0][3]) ** 2)
            if dx > 0.15 and md < 0.05:
                print("  >>> [实锤] 飞机几乎没动,但雷达测出漂了 %.3f m" % dx)
                print("      雷达数据在空中的质量确实不行。")
                print("      下一步结合 [3]抖动 和 [4]振动 判断:")
                print("        - 抖动大 + 振动大 -> 振动导致 FAST-LIO 配准退化(要减震/加固)")
                print("        - 抖动大 + 振动正常 -> FAST-LIO 参数或外参问题(调参/重新标定)")
                print("        - 抖动小但持续单向漂 -> 算法累积误差(查外参/回环)")
            elif dx < 0.05:
                print("  >>> [意外] 雷达在空中也很稳 (%.3f m)" % dx)
                print("      那雷达数据没问题,要回到 EKF2 融合环节重新查。")
            else:
                print("  >>> 雷达位移 %.3f m,动捕位移 %.3f m,介于两者之间" % (dx, md))
                print("      雷达有一定误差但不算严重,需要结合抖动/振动综合判断。")
        print("")
        print("=" * 72)


if __name__ == '__main__':
    try:
        LidarQuality()
        rospy.spin()
    except KeyboardInterrupt:
        pass
