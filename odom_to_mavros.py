#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomFusion:
    def __init__(self):
        rospy.init_node('odom_fusion_node', anonymous=True)

        # 简单版本: 只做偏移, 杆臂补偿交给 PX4 的 EKF2_EV_POS_X/Y/Z
        self.offset_x = rospy.get_param('~offset_x', 0.0)
        self.offset_y = rospy.get_param('~offset_y', 0.0)
        self.offset_z = rospy.get_param('~offset_z', 0.0)
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.target_frame = "world"

        rospy.loginfo(f"🚀 [Fusion] Simple version! Drone_{self.drone_id} Offset: x={self.offset_x}, y={self.offset_y}, z={self.offset_z}")

        self.pub_mavros = rospy.Publisher(
            f"/drone_{self.drone_id}/mavros/vision_pose/pose", 
            PoseStamped, 
            queue_size=10
        )

        self.pub_planner = rospy.Publisher(
            f"/drone_{self.drone_id}/odom_world", 
            Odometry, 
            queue_size=10
        )

        odom_topic = f"/drone_{self.drone_id}/Odometry"
        self.sub = rospy.Subscriber(
            odom_topic, 
            Odometry, 
            self.odom_cb, 
            queue_size=10, 
            tcp_nodelay=True
        )

    def odom_cb(self, msg):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        raw_z = msg.pose.pose.position.z

        world_x = raw_x + self.offset_x
        world_y = raw_y + self.offset_y
        world_z = raw_z + self.offset_z

        # 输出1: MAVROS
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose.position.x = world_x
        pose_msg.pose.position.y = world_y
        pose_msg.pose.position.z = world_z
        pose_msg.pose.orientation = msg.pose.pose.orientation
        self.pub_mavros.publish(pose_msg)

        # 输出2: EGO-Planner
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.target_frame
        odom_msg.child_frame_id = msg.child_frame_id
        odom_msg.pose.pose.position.x = world_x
        odom_msg.pose.pose.position.y = world_y
        odom_msg.pose.pose.position.z = world_z
        odom_msg.pose.pose.orientation = msg.pose.pose.orientation
        odom_msg.twist = msg.twist
        self.pub_planner.publish(odom_msg)

if __name__ == '__main__':
    try:
        OdomFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
