#!/usr/bin/env python3
import os
import signal
import subprocess
import time
from collections import defaultdict
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, RCIn, State, PositionTarget
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rosgraph_msgs.msg import Log
from rospy import AnyMsg


TOPICS = [
    "/drone_0/mavros/state",
    "/drone_0/mavros/extended_state",
    "/drone_0/mavros/local_position/odom",
    "/drone_0/mavros/rc/in",
    "/drone_0/mavros/setpoint_raw/local",
    "/drone_0/mavros/setpoint_position/local",
    "/drone_0/mavros/setpoint_raw/target_local",
    "/drone_0/planning/pos_cmd",
    "/drone_0/planning/bspline",
    "/drone_0/px4ctrl/force_hover",
    "/rosout",
]


class FlightDebugLogger:
    def __init__(self):
        root = rospy.get_param("~log_root", "/home/nvidia/fuel_debug_logs")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = os.path.join(root, stamp)
        os.makedirs(self.log_dir, exist_ok=True)

        self.events = open(os.path.join(self.log_dir, "events.log"), "a", buffering=1)
        self.summary = open(os.path.join(self.log_dir, "topics_summary.log"), "a", buffering=1)
        self.rosout = open(os.path.join(self.log_dir, "rosout_px4ctrl.log"), "a", buffering=1)
        self.meta = open(os.path.join(self.log_dir, "metadata.txt"), "a", buffering=1)

        self.counts = defaultdict(int)
        self.last_counts = defaultdict(int)
        self.last_state = None
        self.last_landed_state = None
        self.last_odom = None
        self.last_rc = None
        self.last_pos_cmd = None
        self.last_target = None
        self.last_raw_local = None
        self.last_pose_setpoint = None
        self.bag_proc = None
        self.start_time = time.time()
        self.duration_sec = float(rospy.get_param("~duration_sec", 0.0))

        self.write_meta()
        self.start_bag_if_enabled()
        self.subscribe()

        rospy.Timer(rospy.Duration(1.0), self.write_summary)
        rospy.on_shutdown(self.shutdown)
        self.log_event("logger_started dir=%s" % self.log_dir)

    def write_meta(self):
        self.meta.write("started: %s\n" % datetime.now().isoformat())
        self.meta.write("node: %s\n" % rospy.get_name())
        self.meta.write("topics:\n")
        for topic in TOPICS:
            self.meta.write("  %s\n" % topic)

    def start_bag_if_enabled(self):
        if not bool(rospy.get_param("~record_bag", True)):
            self.meta.write("record_bag: false\n")
            return
        bag_path = os.path.join(self.log_dir, "debug.bag")
        cmd = ["rosbag", "record", "-O", bag_path] + TOPICS
        self.meta.write("record_bag: true\n")
        self.meta.write("rosbag_cmd: %s\n" % " ".join(cmd))
        self.bag_proc = subprocess.Popen(
            cmd,
            stdout=open(os.path.join(self.log_dir, "rosbag_stdout.log"), "a"),
            stderr=open(os.path.join(self.log_dir, "rosbag_stderr.log"), "a"),
            preexec_fn=os.setsid,
        )

    def subscribe(self):
        rospy.Subscriber(
            "/drone_0/mavros/state",
            State,
            self.counted_cb("/drone_0/mavros/state", self.state_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/mavros/extended_state",
            ExtendedState,
            self.counted_cb("/drone_0/mavros/extended_state", self.extended_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/mavros/local_position/odom",
            Odometry,
            self.counted_cb("/drone_0/mavros/local_position/odom", self.odom_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/mavros/rc/in",
            RCIn,
            self.counted_cb("/drone_0/mavros/rc/in", self.rc_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/mavros/setpoint_raw/local",
            PositionTarget,
            self.counted_cb("/drone_0/mavros/setpoint_raw/local", self.raw_local_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/mavros/setpoint_position/local",
            PoseStamped,
            self.counted_cb("/drone_0/mavros/setpoint_position/local", self.pose_setpoint_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/mavros/setpoint_raw/target_local",
            PositionTarget,
            self.counted_cb("/drone_0/mavros/setpoint_raw/target_local", self.target_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/planning/pos_cmd",
            PositionCommand,
            self.counted_cb("/drone_0/planning/pos_cmd", self.pos_cmd_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/planning/bspline",
            AnyMsg,
            self.counted_cb("/drone_0/planning/bspline", self.ignore_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/drone_0/px4ctrl/force_hover",
            AnyMsg,
            self.counted_cb("/drone_0/px4ctrl/force_hover", self.force_hover_cb),
            queue_size=10,
        )
        rospy.Subscriber(
            "/rosout",
            Log,
            self.counted_cb("/rosout", self.rosout_cb),
            queue_size=100,
        )

    def counted_cb(self, topic, callback):
        def cb(msg):
            self.counts[topic] += 1
            callback(msg)
        return cb

    def ignore_cb(self, _msg):
        pass

    def log_event(self, text):
        self.events.write("[%8.3f] %s\n" % (time.time() - self.start_time, text))

    def state_cb(self, msg):
        sig = (msg.connected, msg.armed, msg.guided, msg.manual_input, msg.mode, msg.system_status)
        if sig != self.last_state:
            self.log_event(
                "state connected=%s armed=%s guided=%s manual_input=%s mode=%s system_status=%s"
                % sig
            )
            self.last_state = sig

    def extended_cb(self, msg):
        if msg.landed_state != self.last_landed_state:
            self.log_event("extended landed_state=%s vtol_state=%s" % (msg.landed_state, msg.vtol_state))
            self.last_landed_state = msg.landed_state

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.last_odom = (p.x, p.y, p.z, v.x, v.y, v.z)

    def rc_cb(self, msg):
        self.last_rc = list(msg.channels)

    def target_cb(self, msg):
        self.last_target = (
            msg.coordinate_frame,
            msg.type_mask,
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z,
            msg.yaw,
        )

    def raw_local_cb(self, msg):
        self.last_raw_local = (
            msg.coordinate_frame,
            msg.type_mask,
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z,
            msg.yaw,
        )

    def pose_setpoint_cb(self, msg):
        p = msg.pose.position
        self.last_pose_setpoint = (p.x, p.y, p.z)

    def pos_cmd_cb(self, msg):
        self.last_pos_cmd = (
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z,
            msg.yaw,
        )

    def force_hover_cb(self, _msg):
        self.log_event("force_hover_received")

    def rosout_cb(self, msg):
        interesting_names = ("px4ctrl", "mavros", "exploration", "traj", "waypoint", "realsense")
        if msg.level >= Log.WARN or any(name in msg.name for name in interesting_names):
            self.rosout.write(
                "[%8.3f] level=%s name=%s file=%s:%s msg=%s\n"
                % (time.time() - self.start_time, msg.level, msg.name, msg.file, msg.line, msg.msg)
            )

    def write_summary(self, _event):
        now = time.time()
        elapsed = now - self.start_time
        parts = ["[%8.3f]" % elapsed]
        for topic in TOPICS:
            delta = self.counts[topic] - self.last_counts[topic]
            self.last_counts[topic] = self.counts[topic]
            parts.append("%s=%dHz" % (topic, delta))
        self.summary.write(" ".join(parts) + "\n")

        if self.last_odom:
            x, y, z, vx, vy, vz = self.last_odom
            self.events.write(
                "[%8.3f] odom p=(%.3f,%.3f,%.3f) v=(%.3f,%.3f,%.3f)\n"
                % (elapsed, x, y, z, vx, vy, vz)
            )
        if self.last_rc:
            self.events.write("[%8.3f] rc channels=%s\n" % (elapsed, self.last_rc[:8]))
        if self.last_target:
            frame, mask, x, y, z, vx, vy, vz, yaw = self.last_target
            self.events.write(
                "[%8.3f] target_local frame=%s mask=%s p=(%.3f,%.3f,%.3f) v=(%.3f,%.3f,%.3f) yaw=%.3f\n"
                % (elapsed, frame, mask, x, y, z, vx, vy, vz, yaw)
            )
        if self.last_raw_local:
            frame, mask, x, y, z, vx, vy, vz, yaw = self.last_raw_local
            self.events.write(
                "[%8.3f] raw_local frame=%s mask=%s p=(%.3f,%.3f,%.3f) v=(%.3f,%.3f,%.3f) yaw=%.3f\n"
                % (elapsed, frame, mask, x, y, z, vx, vy, vz, yaw)
            )
        if self.last_pose_setpoint:
            x, y, z = self.last_pose_setpoint
            self.events.write("[%8.3f] pose_setpoint p=(%.3f,%.3f,%.3f)\n" % (elapsed, x, y, z))
        if self.last_pos_cmd:
            x, y, z, vx, vy, vz, yaw = self.last_pos_cmd
            self.events.write(
                "[%8.3f] pos_cmd p=(%.3f,%.3f,%.3f) v=(%.3f,%.3f,%.3f) yaw=%.3f\n"
                % (elapsed, x, y, z, vx, vy, vz, yaw)
            )

        if self.duration_sec > 0 and elapsed >= self.duration_sec:
            self.log_event("duration_reached shutting_down")
            rospy.signal_shutdown("duration reached")

    def shutdown(self):
        self.log_event("logger_stopping")
        if self.bag_proc and self.bag_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGINT)
                self.bag_proc.wait(timeout=5)
            except Exception as exc:
                self.log_event("rosbag_stop_error=%s" % exc)
        for handle in (self.events, self.summary, self.rosout, self.meta):
            try:
                handle.close()
            except Exception:
                pass


if __name__ == "__main__":
    rospy.init_node("flight_debug_logger", anonymous=False)
    logger = FlightDebugLogger()
    rospy.loginfo("[flight_debug_logger] writing logs to %s", logger.log_dir)
    rospy.spin()
