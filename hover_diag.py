#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
悬停漂移综合诊断工具
================================================
同时监控两路数据:
  1. EKF2 融合后的位姿 (PX4 最终用于控制的那个)
  2. FAST-LIO 原始雷达里程计

实时显示: 位置、姿态、两者姿态差、漂移距离与速率
Ctrl+C 结束后输出诊断汇总 + 判断结论

用法:
  python3 ~/hover_diag.py
  或指定话题名:
  python3 ~/hover_diag.py /drone_0/fast_lio/odometry /drone_0/mavros/local_position/pose

用法建议:
  A. 地面静止测试: 把飞机水平放地上,跑 60~120 秒
  B. 空中定点测试: 起飞悬停,跑 60 秒
"""

import sys
import math
import time
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# 默认话题名(可用命令行参数覆盖)
LIO_TOPIC = '/drone_0/fast_lio/odometry'
FUSED_TOPIC = '/drone_0/mavros/local_position/pose'

if len(sys.argv) > 1:
    LIO_TOPIC = sys.argv[1]
if len(sys.argv) > 2:
    FUSED_TOPIC = sys.argv[2]


def mean(vals):
    return sum(vals) / float(len(vals)) if vals else 0.0


class HoverDiag(object):
    def __init__(self):
        rospy.init_node('hover_diag', anonymous=True)

        self.fused = []      # (t, x, y, z, roll, pitch, yaw)
        self.lio = []        # (t, x, y, z, roll, pitch, yaw)
        self.done = False

        rospy.Subscriber(FUSED_TOPIC, PoseStamped, self.fused_cb, queue_size=10)
        rospy.Subscriber(LIO_TOPIC, Odometry, self.lio_cb, queue_size=10)
        rospy.Timer(rospy.Duration(2.0), self.report)
        rospy.on_shutdown(self.finish)

        print("=" * 78)
        print("悬停漂移综合诊断")
        print("=" * 78)
        print("融合位姿话题 (PX4用): %s" % FUSED_TOPIC)
        print("雷达里程计话题      : %s" % LIO_TOPIC)
        print("")
        print("实时输出说明:")
        print("  漂移 = 距离本次记录起点的水平位移")
        print("  姿态差 = 融合姿态 - 雷达姿态 (关键指标,正常应 < 0.3 度)")
        print("")
        print("飞机放好/起飞后开始记录, 60~120 秒后按 Ctrl+C 看结论")
        print("=" * 78)

    def fused_cb(self, msg):
        q = msg.pose.orientation
        r, p, y = euler_from_quaternion((q.x, q.y, q.z, q.w))
        pos = msg.pose.position
        self.fused.append((msg.header.stamp.to_sec(), pos.x, pos.y, pos.z,
                           math.degrees(r), math.degrees(p), math.degrees(y)))

    def lio_cb(self, msg):
        q = msg.pose.pose.orientation
        r, p, y = euler_from_quaternion((q.x, q.y, q.z, q.w))
        pos = msg.pose.pose.position
        self.lio.append((msg.header.stamp.to_sec(), pos.x, pos.y, pos.z,
                         math.degrees(r), math.degrees(p), math.degrees(y)))

    def report(self, _evt):
        if not self.fused:
            print("[等待] 还没收到 %s 的数据,请检查话题名或 ROS 是否已启动" % FUSED_TOPIC)
            return
        if not self.lio:
            print("[等待] 还没收到 %s 的数据,请检查话题名" % LIO_TOPIC)
            return

        f = self.fused[-1]
        f0 = self.fused[0]
        horiz = math.sqrt((f[1] - f0[1]) ** 2 + (f[2] - f0[2]) ** 2)
        t = f[0] - f0[0]
        rate = horiz / t if t > 0 else 0.0

        l = self.lio[-1]
        droll = f[4] - l[4]
        dpitch = f[5] - l[5]

        print("pos(%6.2f,%6.2f,%6.2f) 姿态(r%5.1f p%5.1f y%6.1f) | "
              "雷达(r%5.1f p%5.1f) 姿态差(%+4.1f,%+4.1f) | 漂移 %5.2fm 速率 %5.2fm/s"
              % (f[1], f[2], f[3], f[4], f[5], f[6],
                 l[4], l[5], droll, dpitch, horiz, rate))

    def finish(self):
        if self.done:
            return
        self.done = True
        print("")
        print("=" * 78)
        print("诊断汇总")
        print("=" * 78)

        if len(self.fused) < 2:
            print("融合位姿样本不足,无法分析")
            return

        f0, f1 = self.fused[0], self.fused[-1]
        dur = f1[0] - f0[0]
        horiz = math.sqrt((f1[1] - f0[1]) ** 2 + (f1[2] - f0[2]) ** 2)
        vert = f1[3] - f0[3]
        rate = horiz / dur if dur > 0 else 0.0

        f_roll = mean([s[4] for s in self.fused])
        f_pitch = mean([s[5] for s in self.fused])

        print("")
        print("[1] 漂移量")
        print("    时长      : %.1f 秒 (%d 帧)" % (dur, len(self.fused)))
        print("    水平总漂移: %.3f m" % horiz)
        print("    高度变化  : %+.3f m" % vert)
        print("    平均速率  : %.3f m/s" % rate)

        print("")
        print("[2] 稳态姿态 (悬停时理论上应接近 0)")
        print("    融合后平均 roll : %+.2f 度" % f_roll)
        print("    融合后平均 pitch: %+.2f 度" % f_pitch)

        if self.lio and len(self.lio) > 1:
            l_roll = mean([s[4] for s in self.lio])
            l_pitch = mean([s[5] for s in self.lio])
            print("")
            print("[3] 雷达 vs 飞控 姿态对齐 (关键!)")
            print("    雷达平均 roll : %+.2f 度" % l_roll)
            print("    雷达平均 pitch: %+.2f 度" % l_pitch)
            print("    >>> roll  偏差: %+.2f 度" % (f_roll - l_roll))
            print("    >>> pitch 偏差: %+.2f 度" % (f_pitch - l_pitch))

            d_roll = abs(f_roll - l_roll)
            d_pitch = abs(f_pitch - l_pitch)
            max_d = max(d_roll, d_pitch)

            print("")
            print("=" * 78)
            print("结论判断:")
            print("=" * 78)
            if max_d > 0.5:
                print("  [高度可疑] 雷达与飞控姿态未对齐 (最大偏差 %.2f 度)" % max_d)
                print("  >>> 这就是漂移的最大嫌疑: 两者水平面不一致,EKF2 融合时")
                print("      会折算出一个虚假的水平加速度,飞机被持续推向一边。")
                print("  >>> 解决: 在 odom_to_mavros.py 里填 CALIB_ROLL_DEG /")
                print("      CALIB_PITCH_DEG 补偿这个偏差(方向可能要试正负)。")
            elif max_d > 0.3:
                print("  [轻微可疑] 雷达与飞控姿态偏差 %.2f 度,偏大但不致命" % max_d)
            else:
                print("  [正常] 雷达与飞控姿态对齐良好 (最大偏差 %.2f 度)" % max_d)

            if abs(f_roll) > 1.5 or abs(f_pitch) > 1.5:
                print("")
                print("  [可疑] 悬停时飞机持续保持倾角 (roll %+.2f / pitch %+.2f)" % (f_roll, f_pitch))
                print("  >>> 持续倾斜 = 持续的水平加速度分量,必然漂移。")
                print("  >>> 排查: 重心偏移 / 桨不平衡 / 飞控加速度计校准 / 姿态零偏")
            else:
                print("")
                print("  [正常] 稳态姿态基本水平")

            if rate > 0.05:
                print("")
                print("  [确认] 存在明显漂移: %.3f m/s" % rate)
            else:
                print("")
                print("  [正常] 漂移速率很小 (%.3f m/s)" % rate)
        else:
            print("")
            print("[跳过] 没收到雷达里程计数据,无法做姿态对齐分析")

        print("")
        print("=" * 78)


if __name__ == '__main__':
    try:
        HoverDiag()
        rospy.spin()
    except KeyboardInterrupt:
        pass
