#!/usr/bin/env bash
# record_dual_imu.sh —— 同步录制 雷达IMU + 飞控IMU, 用于对比振动水平
#
# 用途: 怠速/飞行时对比两个 IMU 的振动 (飞控IMU通常有减振垫, 雷达IMU直连机架)
#
# 操作流程 (建议):
#   1. ./record_dual_imu.sh          # 开始录制 (自动启动雷达驱动, 不启动FAST-LIO)
#   2. 静止 10s → 怠速桨转 30s → 停桨  (或直接飞行)
#   3. ./record_dual_imu.sh stop     # 停止
#
set -u
set +u; source /opt/ros/noetic/setup.bash; set -u
set +u; source /mnt/nvme/ws/fastlio_ws/devel/setup.bash; set -u

BAGDIR=/mnt/nvme/fastlio_bags

if [ "${1:-}" = "stop" ]; then
  echo '[stop] closing rosbag ...'
  pkill -INT -f 'rosbag record' 2>/dev/null
  sleep 2
  echo '[stop] stopping livox driver ...'
  pkill -f msg_MID360.launch 2>/dev/null
  sleep 1
  echo "[stop] done. bags: $BAGDIR"
  exit 0
fi

# 前置: mavros 必须在跑 (飞控 IMU 来源)
if ! timeout 3 rostopic list 2>/dev/null | grep -q '/drone_0/mavros/imu/data_raw'; then
  echo '[!!] mavros 未运行或飞控 IMU 无数据, 请先启动 mavros (飞控上电)'
  exit 1
fi

# roscore
if ! pgrep -x rosmaster >/dev/null; then
  echo '[0] starting roscore ...'
  nohup roscore > /tmp/roscore_dual.log 2>&1 &
  for i in $(seq 1 20); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
fi

# 雷达驱动 (只为拿 /drone_0/livox/imu, 不启动 FAST-LIO)
if ! pgrep -f 'msg_MID360.launch' >/dev/null; then
  echo '[1] starting livox driver (仅驱动, 无FAST-LIO) ...'
  nohup env ROS_NAMESPACE=drone_0 roslaunch livox_ros_driver2 msg_MID360.launch > /tmp/livox_dual.log 2>&1 &
  sleep 4
else
  echo '[1] livox driver 已在运行'
fi

# 等两个 IMU 都有数据
echo '[2] 等待两个 IMU 话题就绪 ...'
ok_lidar=0; ok_fcu=0
for i in $(seq 1 15); do
  timeout 2 rostopic hz /drone_0/livox/imu 2>/dev/null | grep -q 'average rate' && ok_lidar=1
  timeout 2 rostopic hz /drone_0/mavros/imu/data_raw 2>/dev/null | grep -q 'average rate' && ok_fcu=1
  [ $ok_lidar -eq 1 ] && [ $ok_fcu -eq 1 ] && break
  sleep 1
done
if [ $ok_lidar -eq 0 ] || [ $ok_fcu -eq 0 ]; then
  echo "[!!] IMU 未就绪: 雷达IMU=$ok_lidar 飞控IMU=$ok_fcu"; exit 1
fi

# 录制
STAMP=$(date +%Y%m%d_%H%M%S)
echo "[3] recording dual IMU ..."
nohup rosbag record -O "$BAGDIR/dualimu_$STAMP" --lz4 \
  /drone_0/livox/imu /drone_0/mavros/imu/data_raw /drone_0/mavros/imu/data \
  > "$BAGDIR/record_dual_$STAMP.log" 2>&1 &
sleep 2

echo '---------------------------------------------'
echo "bag: $BAGDIR/dualimu_$STAMP.bag"
echo ''
echo '现在操作: 静止10s → 怠速桨转30s → (可选飞行) → 停桨'
echo '完毕后: ./record_dual_imu.sh stop'
