#!/usr/bin/env python3
# 同步记录发散诊断: 点云 / FAST-LIO位姿 / mavros odom / IMU
# 用法: python3 diverge_logger.py <logdir>
import rospy, numpy as np, sys, math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg

logdir = sys.argv[1] if len(sys.argv)>1 else "/tmp"
f_pc  = open(logdir+"/diag_pointcloud.log","w",1)
f_lio = open(logdir+"/diag_fastlio.log","w",1)
f_mav = open(logdir+"/diag_mavros_odom.log","w",1)
f_imu = open(logdir+"/diag_imu.log","w",1)
for f,h in [(f_pc,"# t_wall  n_pts  n_valid(>0.5m)  dmin  dmax  dmean"),
            (f_lio,"# t_wall  px  py  pz  vx  vy  vz  |p|  |v|"),
            (f_mav,"# t_wall  px  py  pz"),
            (f_imu,"# t_wall  ax  ay  az  |a|")]:
    f.write(h+"\n")

def now(): return rospy.get_time()

def pc_cb(m):
    n=len(m.points)
    if n==0:
        f_pc.write("%.3f %d %d %.3f %.3f %.3f\n"%(now(),0,0,0,0,0)); return
    x=np.array([p.x for p in m.points]);y=np.array([p.y for p in m.points]);z=np.array([p.z for p in m.points])
    d=np.sqrt(x*x+y*y+z*z); v=int((d>0.5).sum())
    f_pc.write("%.3f %d %d %.3f %.3f %.3f\n"%(now(),n,v,d.min(),d.max(),d.mean()))

def lio_cb(m):
    p=m.pose.pose.position; v=m.twist.twist.linear
    pn=math.sqrt(p.x**2+p.y**2+p.z**2); vn=math.sqrt(v.x**2+v.y**2+v.z**2)
    f_lio.write("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n"%(now(),p.x,p.y,p.z,v.x,v.y,v.z,pn,vn))

def mav_cb(m):
    p=m.pose.pose.position
    f_mav.write("%.3f %.3f %.3f %.3f\n"%(now(),p.x,p.y,p.z))

def imu_cb(m):
    a=m.linear_acceleration; an=math.sqrt(a.x**2+a.y**2+a.z**2)
    f_imu.write("%.3f %.4f %.4f %.4f %.4f\n"%(now(),a.x,a.y,a.z,an))

rospy.init_node("diverge_logger",anonymous=True)
rospy.Subscriber("/drone_0/livox/lidar",CustomMsg,pc_cb,queue_size=50)
rospy.Subscriber("/drone_0/Odometry",Odometry,lio_cb,queue_size=200)
rospy.Subscriber("/drone_0/mavros/local_position/odom",Odometry,mav_cb,queue_size=200)
rospy.Subscriber("/drone_0/livox/imu",Imu,imu_cb,queue_size=400)
rospy.loginfo("[diverge_logger] recording to %s"%logdir)
rospy.spin()
