#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import TakeoffLand


class AutoTakeoffNode:
    def __init__(self):
        # 中文注释：这里直接订阅 MAVROS 转换后的位姿，和你现有的 odom_to_mavros.py 流程一致
        self.pose_topic = rospy.get_param("~pose_topic", "/drone_0/mavros/local_position/pose")
        self.takeoff_topic = rospy.get_param("~takeoff_topic", "/takeoff_land")
        self.takeoff_height = rospy.get_param("~takeoff_height", 1.0)
        self.stable_speed_thresh = rospy.get_param("~stable_speed_thresh", 0.15)
        self.stable_duration = rospy.get_param("~stable_duration", 2.0)

        self.last_pose_time = None
        self.last_pose = None
        self.last_vel = None
        self.stable_since = None
        self.sent_takeoff = False

        self.takeoff_pub = rospy.Publisher(self.takeoff_topic, TakeoffLand, queue_size=1, latch=True)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.loginfo("[auto_takeoff] waiting pose on %s", self.pose_topic)

    def pose_callback(self, msg):
        now = rospy.Time.now()
        if self.last_pose is not None and self.last_pose_time is not None:
            dt = (now - self.last_pose_time).to_sec()
            if dt > 1e-3:
                dx = msg.pose.position.x - self.last_pose.pose.position.x
                dy = msg.pose.position.y - self.last_pose.pose.position.y
                dz = msg.pose.position.z - self.last_pose.pose.position.z
                speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
            else:
                speed = 0.0
        else:
            speed = 0.0

        self.last_pose_time = now
        self.last_pose = msg
        self.last_vel = speed

        # 中文注释：位姿更新后，若速度连续足够小，就允许触发起飞
        if speed <= self.stable_speed_thresh:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None

    def timer_callback(self, _event):
        if self.sent_takeoff:
            return

        if self.last_pose_time is None or self.last_pose is None:
            return

        if (rospy.Time.now() - self.last_pose_time).to_sec() > 0.5:
            self.stable_since = None
            return

        if self.stable_since is None:
            return

        if (rospy.Time.now() - self.stable_since).to_sec() < self.stable_duration:
            return

        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
        self.takeoff_pub.publish(msg)
        self.sent_takeoff = True

        # 中文注释：实际起飞高度由 px4ctrl 的参数文件决定，这里只负责发送起飞触发
        rospy.loginfo(
            "[auto_takeoff] published TAKEOFF to %s after stable pose, configured takeoff_height=%.2f",
            self.takeoff_topic,
            self.takeoff_height,
        )


if __name__ == "__main__":
    rospy.init_node("auto_takeoff")
    AutoTakeoffNode()
    rospy.spin()
