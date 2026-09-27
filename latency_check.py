#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达数据 延迟/抖动 诊断
================================================
背景: 动捕和雷达都发 /drone_0/mavros/vision_pose/pose,用同一套 EKF2_EV_* 参数,
      动捕飞得稳、雷达飘 -> 说明参数不是主因,差别在数据本身的特性。

关键指标:
  数据年龄 age    = 收到时刻 - 消息里的时间戳
  抖动     jitter = age 的标准差

为什么抖动致命:
  EKF2_EV_DELAY 只能补偿一个"固定的"延迟。如果数据延迟忽大忽小(抖动大),
  固定补偿就对不上,错位的时间直接变成位置误差 -> 飞机飘。
  动捕是实时的(延迟小且几乎不抖);雷达要经过 FAST-LIO 解算,延迟大且可能抖动。

本脚本同时对比三路:
  1. FAST-LIO 原始 /drone_0/Odometry
  2. 转换后     /drone_0/mavros/vision_pose/pose
  3. 动捕       /vrpn_client_node/*/pose  (自动探测,作为对照基准)

用法: python3 ~/latency_check.py
"""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class Stat(object):
    def __init__(self, name):
        self.name = name
        self.ages = []      # 毫秒
        self.t0 = None
        self.t1 = None
        self.n = 0
        self.neg = 0        # 时间戳在未来(时钟不同步)的次数

    def add(self, stamp_sec, recv_sec):
        age = (recv_sec - stamp_sec) * 1000.0
        self.ages.append(age)
        self.n += 1
        if age < 0:
            self.neg += 1
        if self.t0 is None:
            self.t0 = recv_sec
        self.t1 = recv_sec

    def summary(self):
        if not self.ages:
            print("[%s] 没收到数据" % self.name)
            return None
        n = len(self.ages)
        avg = sum(self.ages) / n
        var = sum((a - avg) ** 2 for a in self.ages) / n
        std = math.sqrt(var)
        dur = (self.t1 - self.t0) if (self.t1 and self.t0) else 0.0
        hz = n / dur if dur > 0 else 0.0
        print("")
        print("[%s]" % self.name)
        print("  样本数    : %d 帧, 频率 %.1f Hz" % (n, hz))
        print("  平均年龄  : %.1f ms" % avg)
        print("  最小/最大 : %.1f / %.1f ms" % (min(self.ages), max(self.ages)))
        print("  抖动(标准差): %.1f ms   <-- 关键指标" % std)
        if self.neg:
            print("  [警告] 有 %d 帧时间戳超前于接收时刻(时钟可能不同步)" % self.neg)
        return avg, std


class LatencyCheck(object):
    def __init__(self):
        rospy.init_node('latency_check', anonymous=True)
        self.lio = Stat('FAST-LIO 原始 /drone_0/Odometry')
        self.vp = Stat('转换后 /drone_0/mavros/vision_pose/pose')
        self.mocap = None

        rospy.Subscriber('/drone_0/Odometry', Odometry, self.lio_cb, queue_size=100)
        rospy.Subscriber('/drone_0/mavros/vision_pose/pose', PoseStamped,
                         self.vp_cb, queue_size=100)

        # 自动探测动捕话题作为对照
        try:
            for t, ty in rospy.get_published_topics():
                if ('vrpn' in t or 'mocap' in t) and 'PoseStamped' in ty:
                    self.mocap = Stat('动捕(对照) %s' % t)
                    rospy.Subscriber(t, PoseStamped, self.mocap_cb, queue_size=100)
                    break
        except Exception:
            pass

        rospy.Timer(rospy.Duration(3.0), self.report)
        rospy.on_shutdown(self.finish)
        self.done = False

        print("=" * 70)
        print("雷达数据 延迟/抖动 诊断")
        print("=" * 70)
        print("正在采集... 20~30 秒后按 Ctrl+C")
        print("重点看「抖动」: 动捕的抖动是基准,雷达抖动明显更大就是问题")
        print("=" * 70)

    def lio_cb(self, msg):
        self.lio.add(msg.header.stamp.to_sec(), rospy.get_time())

    def vp_cb(self, msg):
        self.vp.add(msg.header.stamp.to_sec(), rospy.get_time())

    def mocap_cb(self, msg):
        if self.mocap:
            self.mocap.add(msg.header.stamp.to_sec(), rospy.get_time())

    def report(self, _evt):
        if self.lio.n % 60 == 0 and self.lio.n > 0:
            print("  已采集 %d 帧 (LIO) / %d 帧 (vision_pose)"
                  % (self.lio.n, self.vp.n))

    def finish(self):
        if self.done:
            return
        self.done = True
        print("")
        print("=" * 70)
        print("延迟/抖动 汇总")
        print("=" * 70)

        r_lio = self.lio.summary()
        r_vp = self.vp.summary()
        r_mc = self.mocap.summary() if self.mocap else None

        print("")
        print("=" * 70)
        print("结论判断:")
        print("=" * 70)

        if r_mc and r_vp:
            mc_std = r_mc[1]
            vp_std = r_vp[1]
            print("  雷达链路抖动 %.1f ms  vs  动捕抖动 %.1f ms" % (vp_std, mc_std))
            print("")
            if vp_std > mc_std * 3 and vp_std > 20:
                print("  >>> [高度可疑] 雷达数据抖动远大于动捕,且超过 20ms")
                print("      EKF2_EV_DELAY 只能补固定延迟,补不了抖动。")
                print("      错位的时间会直接变成位置误差 -> 飘。")
                print("      >>> 修复方向: 让时间戳稳定(见下面说明)")
            elif vp_std > 20:
                print("  >>> [可疑] 雷达数据抖动 %.1f ms 偏大(>20ms)" % vp_std)
                print("      会影响融合精度,建议优化。")
            else:
                print("  >>> [正常] 雷达数据抖动 %.1f ms,在可接受范围" % vp_std)
                print("      那抖动不是主因,要往「振动导致FAST-LIO精度下降」方向查。")

        if r_vp:
            avg, std = r_vp
            print("")
            print("  当前 EKF2_EV_DELAY 应该设成 ≈ 平均年龄 %.0f ms" % avg)
            print("  (抖动 %.1f ms 是固定补偿补不掉的残留误差)" % std)

        print("")
        print("如果抖动大,修复思路:")
        print("  1. 检查 FAST-LIO 的时间戳是否用的是 ROS 时钟(而非雷达硬件时钟)")
        print("  2. odom_to_mavros.py 里目前是透传 msg.header.stamp;")
        print("     若该时间戳不稳,可改为用接收时刻 rospy.Time.now(),")
        print("     这样延迟变成固定的, EKF2_EV_DELAY 就能补准。")
        print("  3. 降低 FAST-LIO 计算负载(点云滤波/特征点数),减少解算时间波动")
        print("=" * 70)


if __name__ == '__main__':
    try:
        LatencyCheck()
        rospy.spin()
    except KeyboardInterrupt:
        pass
