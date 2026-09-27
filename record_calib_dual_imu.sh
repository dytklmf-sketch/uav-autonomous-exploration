#!/usr/bin/env bash
# record_calib_dual_imu.sh —— 录制 双IMU(Mid360内置 + CJ02) 外参标定数据
#
# 动作要领 (全程桨不转, 录制约 90~120 s):
#   1. 静止 10 s
#   2. 绕 X 轴 (滚转) 缓慢大幅 ±60° 来回 5 次   ~15 s
#   3. 绕 Y 轴 (俯仰) 缓慢大幅 ±60° 来回 5 次   ~15 s
#   4. 绕 Z 轴 (偏航) 缓慢大幅 ±90° 来回 5 次   ~15 s
#   5. 平移画圈 + 画 8 字, 带小幅晃动           ~20 s
#   6. 静止 5 s
#   要点: 动作"慢而大"; 翻转时尽量绕单一轴; 别怕晃, 有角加速度才有平移信息
#
# 用法:  ./record_calib_dual_imu.sh          开始录制
#        ./record_calib_dual_imu.sh stop     停止
set -u
set +u; source /opt/ros/noetic/setup.bash; set -u
set +u; source /mnt/nvme/ws/fastlio_ws/devel/setup.bash; set -u
set +u; source /mnt/nvme/ws/cj02_ws/devel/setup.bash; set -u

BAGDIR=/mnt/nvme/fastlio_bags
T_LIDAR_IMU=/drone_0/livox/imu
T_CJ02_IMU=/drone_0/cj02/imu/data_raw

if [ "${1:-}" = "stop" ]; then
  echo '[stop] closing rosbag ...'
  pkill -INT -f 'rosbag record' 2>/dev/null
  sleep 2
  pkill -f msg_MID360.launch 2>/dev/null
  pkill -f cj02_imu_node 2>/dev/null
  sleep 1
  echo '[stop] done.'
  exit 0
fi

# roscore
if ! pgrep -x rosmaster >/dev/null; then
  echo '[0] starting roscore ...'
  nohup roscore > /tmp/roscore_calib.log 2>&1 &
  for i in $(seq 1 20); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
fi

# 雷达驱动 (内置 IMU 来源; launch 在 launch_ROS1/ 子目录, 用全路径)
LIVOX_LAUNCH=/mnt/nvme/ws/fastlio_ws/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch
if ! pgrep -f 'msg_MID360.launch' >/dev/null; then
  echo '[1] starting livox driver ...'
  nohup env ROS_NAMESPACE=drone_0 roslaunch $LIVOX_LAUNCH > /tmp/livox_calib.log 2>&1 &
  sleep 4
else
  echo '[1] livox driver 已在运行'
fi

# CJ02 驱动
if ! pgrep -f cj02_imu_node >/dev/null; then
  echo '[2] starting CJ02 driver ...'
  nohup roslaunch cj02_imu_node cj02_imu.launch > /tmp/cj02_calib.log 2>&1 &
  sleep 3
else
  echo '[2] CJ02 driver 已在运行'
fi

# 等两路 IMU 就绪
echo '[3] 等待两路 IMU 数据 ...'
ok1=0; ok2=0
for i in $(seq 1 20); do
  timeout 4 rostopic hz $T_LIDAR_IMU 2>/dev/null | grep -q 'average rate' && ok1=1
  timeout 4 rostopic hz $T_CJ02_IMU   2>/dev/null | grep -q 'average rate' && ok2=1
  [ $ok1 -eq 1 ] && [ $ok2 -eq 1 ] && break
  sleep 1
done
if [ $ok1 -eq 0 ] || [ $ok2 -eq 0 ]; then
  echo "[!!] IMU 未就绪: 雷达IMU=$ok1 CJ02=$ok2 (雷达是否上电? 查 /tmp/livox_calib.log /tmp/cj02_calib.log)"
  exit 1
fi
echo "    两路 IMU 就绪 ✓"

mkdir -p "$BAGDIR"
BAG="$BAGDIR/calib_dual_imu_$(date +%Y%m%d_%H%M%S).bag"
echo "[4] 开始录制 -> $BAG"
echo "    按动作要领做标定动作; 结束后执行: ./record_calib_dual_imu.sh stop"
rosbag record -O "$BAG" $T_LIDAR_IMU $T_CJ02_IMU
