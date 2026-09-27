#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空中漂移诊断工具 (在飞机"飘"的时候采集,越飘越好抓)
================================================
核心诊断逻辑:
  飞机在空中飘,要么是"被自己的倾斜推着走",要么是"被外力/估计误差带偏"。
  本脚本把两个方向都算出来对比:
    1. 漂移方向角 = 相对起始点位移的方位角
    2. 倾斜方向角 = 机身推力轴(Z轴)偏离竖直方向的水平投影方位角
  两者一致 -> 倾斜是原因(查控制环/重心/EKF姿态)
  两者相反 -> 被反向带偏(查外力或估计反向)
  倾斜极小却猛飘 -> 不是倾斜导致(查振动/延迟/雷达退化)

同时监控 EKF2 状态标志位,抓 accel_error(振动超标) 等异常翻转。

用法:
  python3 ~/air_diag.py
"""

import sys
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_matrix

LIO_TOPIC = '/drone_0/Odometry'
POSE_TOPIC = '/drone_0/mavros/local_position/pose'
VEL_TOPIC = '/drone_0/mavros/local_position/velocity'
EST_TOPIC = '/drone_0/mavros/estimator_status'


def wrap180(a):
    """把角度归一化到 -180~180"""
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def mean(vals):
    return sum(vals) / float(len(vals)) if vals else 0.0


class AirDiag(object):
    def __init__(self):
        rospy.init_node('air_diag', anonymous=True)
        self.samples = []   # (t, x, y, z, roll, pitch, yaw, vx, vy, vz, tilt_deg, tilt_az, drift_az)
        self.lio = []       # (t, roll, pitch)
        self.est_flags = {}  # 标志位 -> 变化次数
        self.est_bad = []    # 异常记录
        self.done = False

        rospy.Subscriber(POSE_TOPIC, PoseStamped, self.pose_cb, queue_size=10)
        rospy.Subscriber(VEL_TOPIC, TwistStamped, self.vel_cb, queue_size=10)
        rospy.Subscriber(LIO_TOPIC, Odometry, self.lio_cb, queue_size=10)

        # 尝试订阅 EKF2 状态,失败不致命
        try:
            from mavros_msgs.msg import EstimatorStatus
            rospy.Subscriber(EST_TOPIC, EstimatorStatus, self.est_cb, queue_size=10)
            self.has_est = True
        except Exception as e:
            self.has_est = False
            print("[提示] 无法订阅 EKF2 状态: %s" % e)

        self.last_vel = (0.0, 0.0, 0.0)
        rospy.Timer(rospy.Duration(1.5), self.report)
        rospy.on_shutdown(self.finish)

        print("=" * 80)
        print("空中漂移诊断 (飞机飘着时采集)")
        print("=" * 80)
        print("  漂移方位 = 相对起点位移的方向角")
        print("  倾斜方位 = 机身推力轴偏离竖直的方向角")
        print("  夹角     = 漂移方位 - 倾斜方位")
        print("           接近 0度   -> 被自己的倾斜推着飘")
        print("           接近 180度 -> 被反向带偏")
        print("           倾斜极小却猛飘 -> 与倾斜无关(振动/延迟/雷达退化)")
        print("=" * 80)

    def pose_cb(self, msg):
        q = msg.pose.orientation
        r, p, y = euler_from_quaternion((q.x, q.y, q.z, q.w))
        roll, pitch, yaw = math.degrees(r), math.degrees(p), math.degrees(y)

        # 机身Z轴(推力轴)在世界系的朝向 = 旋转矩阵第三列
        M = quaternion_matrix([q.x, q.y, q.z, q.w])
        zx, zy, zz = M[0, 2], M[1, 2], M[2, 2]
        horiz = math.sqrt(zx * zx + zy * zy)          # 水平分量 = sin(倾角)
        tilt_deg = math.degrees(math.asin(min(1.0, horiz)))
        tilt_az = math.degrees(math.atan2(zy, zx)) if horiz > 1e-6 else 0.0

        pos = msg.pose.position
        t = msg.header.stamp.to_sec()

        # 漂移方位:相对本次记录起点
        if self.samples:
            x0, y0 = self.samples[0][1], self.samples[0][2]
            dx, dy = pos.x - x0, pos.y - y0
        else:
            dx, dy = 0.0, 0.0
        drift_az = math.degrees(math.atan2(dy, dx)) if (abs(dx) > 1e-4 or abs(dy) > 1e-4) else 0.0

        vx, vy, vz = self.last_vel
        self.samples.append((t, pos.x, pos.y, pos.z, roll, pitch, yaw,
                             vx, vy, vz, tilt_deg, tilt_az, drift_az))

    def vel_cb(self, msg):
        v = msg.twist.linear
        self.last_vel = (v.x, v.y, v.z)

    def lio_cb(self, msg):
        q = msg.pose.pose.orientation
        r, p, _y = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.lio.append((msg.header.stamp.to_sec(), math.degrees(r), math.degrees(p)))

    def est_cb(self, msg):
        flags = {
            'attitude': msg.attitude_status_flag,
            'vel_horiz': msg.velocity_horiz_status_flag,
            'vel_vert': msg.velocity_vert_status_flag,
            'posH_rel': msg.pos_horiz_rel_status_flag,
            'posH_abs': msg.pos_horiz_abs_status_flag,
            'posV_abs': msg.pos_vert_abs_status_flag,
            'accel_err': msg.accel_error_status_flag,
            'const_pos': msg.const_pos_mode_status_flag,
            'gps_glitch': msg.gps_glitch_status_flag,
        }
        for k, v in flags.items():
            prev = self.est_flags.get(k)
            if prev is None:
                self.est_flags[k] = v
            elif prev != v:
                self.est_flags[k] = v
                self.est_bad.append("%s: %s -> %s" % (k, prev, v))

    def report(self, _evt):
        if not self.samples:
            print("[等待] 没收到 %s" % POSE_TOPIC)
            return
        s = self.samples[-1]
        s0 = self.samples[0]
        dist = math.sqrt((s[1] - s0[1]) ** 2 + (s[2] - s0[2]) ** 2)
        t = s[0] - s0[0]
        rate = dist / t if t > 0 else 0.0
        diff = wrap180(s[12] - s[11]) if dist > 0.05 else 0.0

        l = self.lio[-1] if self.lio else (0, 0, 0)
        dr = s[4] - l[1]
        dp = s[5] - l[2]

        bad = (" | EKF异常:%s" % ';'.join(self.est_bad[-2:])) if self.est_bad else ""
        print("pos(%6.2f,%6.2f,%6.2f) 漂移%5.2fm @%6.1f° | 倾斜%4.1f° @%6.1f° | 夹角%+6.1f° | "
              "速率%5.2fm/s | 雷达差(%+4.1f,%+4.1f)%s"
              % (s[1], s[2], s[3], dist, s[12], s[10], s[11], diff, rate, dr, dp, bad))

    def finish(self):
        if self.done:
            return
        self.done = True
        print("")
        print("=" * 80)
        print("空中漂移诊断汇总")
        print("=" * 80)

        if len(self.samples) < 2:
            print("样本不足")
            return

        s0, s1 = self.samples[0], self.samples[-1]
        dur = s1[0] - s0[0]
        dist = math.sqrt((s1[1] - s0[1]) ** 2 + (s1[2] - s0[2]) ** 2)
        rate = dist / dur if dur > 0 else 0.0

        tilt_avg = mean([s[10] for s in self.samples])
        tilt_max = max(s[10] for s in self.samples)
        vz_avg = mean([s[9] for s in self.samples])
        horiz_speed = mean([math.sqrt(s[7] ** 2 + s[8] ** 2) for s in self.samples])

        # 只用漂移明显(>10cm)的样本算方位
        valid = [s for s in self.samples
                 if math.sqrt((s[1] - s0[1]) ** 2 + (s[2] - s0[2]) ** 2) > 0.10]
        diff_avg = None
        if valid:
            diffs = [wrap180(s[12] - s[11]) for s in valid]
            diff_avg = mean(diffs)

        print("")
        print("[1] 漂移")
        print("    时长        : %.1f 秒 (%d 帧)" % (dur, len(self.samples)))
        print("    水平总漂移  : %.3f m" % dist)
        print("    高度变化    : %+.3f m" % (s1[3] - s0[3]))
        print("    平均漂移速率: %.3f m/s" % rate)
        print("    平均水平速度: %.3f m/s" % horiz_speed)
        print("    平均垂直速度: %+.3f m/s" % vz_avg)

        print("")
        print("[2] 机身倾斜")
        print("    平均倾角: %.2f 度   最大倾角: %.2f 度" % (tilt_avg, tilt_max))

        if self.lio:
            lr = mean([x[1] for x in self.lio])
            lp = mean([x[2] for x in self.lio])
            fr = mean([s[4] for s in self.samples])
            fp = mean([s[5] for s in self.samples])
            print("")
            print("[3] 雷达 vs 飞控 姿态对齐")
            print("    融合 roll %+.2f / pitch %+.2f" % (fr, fp))
            print("    雷达 roll %+.2f / pitch %+.2f" % (lr, lp))
            print("    偏差 roll %+.2f / pitch %+.2f" % (fr - lr, fp - lp))

        print("")
        print("=" * 80)
        print("结论判断:")
        print("=" * 80)

        if diff_avg is None:
            print("  [正常] 漂移量太小(始终<10cm),没抓到有效漂移")
        else:
            ad = abs(diff_avg)
            print("  漂移方向与倾斜方向夹角: %+.1f 度" % diff_avg)
            print("")
            if ad < 45:
                print("  >>> [倾斜推着飘] 飞机往自己倒的方向飘")
                print("      说明: 机身确实在持续倾斜,升力的水平分量把它推走了。")
                print("      排查: 位置环增益太低 / 重心偏移 / 桨不平衡 / EKF姿态有偏")
            elif ad > 135:
                print("  >>> [反向带偏] 飞机在往倾斜的反方向飘")
                print("      说明: 倾斜是在「抵抗」漂移,有外力或估计误差在推它。")
                print("      排查: 风的扰动 / 雷达观测被污染 / 延迟补偿不准(EKF2_EV_DELAY)")
            else:
                print("  >>> [无关/斜向] 漂移方向与倾斜方向无明显对应")
                print("      说明: 不是单纯的倾斜问题。")
                print("      排查: 雷达退化跳变 / EKF2观测量被拒绝 / 振动")

        print("")
        if tilt_avg < 1.0 and rate > 0.05:
            print("  >>> [注意] 机身几乎水平(平均倾角%.2f度)却明显在飘(%.3f m/s)" % (tilt_avg, rate))
            print("      这基本排除了「倾斜导致」,问题在估计或外力。")
        elif tilt_avg >= 1.0:
            print("  >>> 机身存在持续倾斜 %.2f 度 -> 虚假水平加速度约 %.3f m/s^2"
                  % (tilt_avg, 9.81 * math.sin(math.radians(tilt_avg))))

        print("")
        if self.has_est:
            if self.est_bad:
                print("  [EKF2 异常] 状态标志位发生翻转:")
                for b in self.est_bad[:15]:
                    print("      %s" % b)
                if any('accel_err: False -> True' in b for b in self.est_bad):
                    print("")
                    print("  >>> [重要] accel_error 置位 = 加速度计量测误差超标,")
                    print("      通常是螺旋桨振动过大 -> 污染EKF2 -> 漂移。")
                    print("      排查: 飞控减震 / 桨平衡 / IMU安装")
            else:
                print("  [EKF2 正常] 状态标志位全程无翻转")
        else:
            print("  [跳过] 没有 EKF2 状态数据")

        print("")
        print("=" * 80)


if __name__ == '__main__':
    try:
        AirDiag()
        rospy.spin()
    except KeyboardInterrupt:
        pass
