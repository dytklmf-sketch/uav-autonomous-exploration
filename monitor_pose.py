#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时监控无人机姿态 (Roll/Pitch/Yaw)
订阅 /drone_0/mavros/local_position/pose 话题,转换四元数为欧拉角并显示
"""

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math

class PoseMonitor:
    def __init__(self):
        rospy.init_node('pose_monitor', anonymous=True)
        
        # 订阅姿态话题
        self.pose_sub = rospy.Subscriber(
            '/drone_0/mavros/local_position/pose',
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )
        
        print("=" * 60)
        print("姿态监控已启动")
        print("订阅话题: /drone_0/mavros/local_position/pose")
        print("=" * 60)
        print(f"{'时间戳':<12} {'Roll (°)':<10} {'Pitch (°)':<10} {'Yaw (°)':<10}")
        print("-" * 60)
        
    def pose_callback(self, msg):
        """处理姿态消息"""
        # 提取四元数
        q = msg.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        
        # 转换为欧拉角 (弧度)
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)
        
        # 转换为角度
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        yaw_deg = math.degrees(yaw_rad)
        
        # 获取时间戳
        timestamp = msg.header.stamp.to_sec()
        
        # 打印格式化数据
        print(f"{timestamp:<12.2f} {roll_deg:<10.2f} {pitch_deg:<10.2f} {yaw_deg:<10.2f}")

    def run(self):
        """保持节点运行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = PoseMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        print("\n监控已停止")
    except KeyboardInterrupt:
        print("\n用户中断,监控已停止")
