#!/usr/bin/env bash
# record_compare.sh —— 录制 A/B 对比数据 (点云 + 双IMU 同步)
#
# 动作要领 (全程桨不转, 录制约 60~90 s):
#   静止 5 s → 手持无人机正常步伐行走 (直线/转弯/上下楼梯口均可) → 回到起点 → 静止 5 s
#   要点: 走完回到起点! 终点漂移是 A/B 对比的核心指标
#
# 用法:  ./record_compare.sh          开始录制
#        ./record_compare.sh stop     停止
set -u
set +u; source /opt/ros/noetic/setup.bash; set -u
source /mnt/nvme/ws/cj02_ws/devel/setup.bash   # overlay, 已含 fastlio_ws

BAGDIR=/mnt/nvme/fastlio_bags
T_LIDAR=/drone_0/livox/lidar
T_LIDAR_IMU=/drone_0/livox/imu
T_CJ02_IMU=/drone_0/cj02/imu/data_raw

if [ "${1:-}" = "stop" ]; then
  echo '[stop] closing rosbag ...'
  pkill -INT -f 'rosbag record' 2>/dev/null
  sleep 2
  pkill -f msg_MID360.launch 2>/dev/null
  pkill -f 'cj02_imu_nod[e]' 2>/dev/null
  sleep 1
  echo '[stop] done.'
  exit 0
fi

# roscore
if ! pgrep -x rosmaster >/dev/null; then
  echo '[0] starting roscore ...'
  nohup roscore > /tmp/roscore_cmp.log 2>&1 &
  for i in $(seq 1 20); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
fi

# 雷达驱动 (点云 + 内置IMU)
if ! pgrep -f 'msg_MID360.launch' >/dev/null; then
  echo '[1] starting livox driver ...'
  nohup env ROS_NAMESPACE=drone_0 roslaunch /mnt/nvme/ws/fastlio_ws/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch > /tmp/livox_cmp.log 2>&1 &
  sleep 4
else
  echo '[1] livox driver 已在运行'
fi

# CJ02 驱动
if ! pgrep -f 'cj02_imu_nod[e]' >/dev/null; then
  echo '[2] starting CJ02 driver ...'
  nohup roslaunch cj02_imu_node cj02_imu.launch > /tmp/cj02_cmp.log 2>&1 &
  sleep 3
else
  echo '[2] CJ02 driver 已在运行'
fi

# 等数据就绪
echo '[3] 等待三路数据 ...'
ok1=0; ok2=0; ok3=0
for i in $(seq 1 20); do
  timeout 4 rostopic hz $T_LIDAR    2>/dev/null | grep -q 'average rate' && ok1=1
  timeout 4 rostopic hz $T_LIDAR_IMU 2>/dev/null | grep -q 'average rate' && ok2=1
  timeout 4 rostopic hz $T_CJ02_IMU   2>/dev/null | grep -q 'average rate' && ok3=1
  [ $ok1 -eq 1 ] && [ $ok2 -eq 1 ] && [ $ok3 -eq 1 ] && break
  sleep 1
done
if [ $ok1 -eq 0 ] || [ $ok2 -eq 0 ] || [ $ok3 -eq 0 ]; then
  echo "[!!] 数据未就绪: 点云=$ok1 雷达IMU=$ok2 CJ02=$ok3"; exit 1
fi
echo "    三路数据就绪 ✓"

mkdir -p "$BAGDIR"
BAG="$BAGDIR/ab_compare_$(date +%Y%m%d_%H%M%S).bag"
echo "[4] 开始录制 -> $BAG"
echo "    按动作要领行走; 结束后执行: ./record_compare.sh stop"
rosbag record -O "$BAG" $T_LIDAR $T_LIDAR_IMU $T_CJ02_IMU
