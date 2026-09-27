#!/usr/bin/env bash
# record_fastlio.sh —— 一键启动 雷达+FAST-LIO 并录制 rosbag (定位+点云, 不发给飞控)
#
# 用途: 动捕做定位源飞行时, FAST-LIO 并行运行但【只记录不使用】,
#       飞完回放 bag 分析 FAST-LIO 的 z 漂移/定位质量。
#
# 注意: 本脚本【不启动】odom_to_mavros.py (FAST-LIO 定位不会进飞控),
#       也【不启动】mavros; 动捕定位链路请另外启动, 避免双定位源冲突。
#
# 用法:
#   ./record_fastlio.sh            启动雷达+FAST-LIO+录制
#   ./record_fastlio.sh stop       停止录制(正常关闭bag)并关闭节点
#   RAW=0 ./record_fastlio.sh      不录原始雷达点云(省空间), 只录配准点云+定位+IMU
#                                  (raw 点云 + IMU 可离线重跑 FAST-LIO, 建议录)
set -u

set +u; source /opt/ros/noetic/setup.bash; set -u
set +u; source /mnt/nvme/ws/fastlio_ws/devel/setup.bash; set -u

HOME_DIR=/home/nvidia
BAGDIR=/mnt/nvme/fastlio_bags   # bag 存 NVMe 大盘(105G空闲); 系统盘 eMMC 仅 2.4G, 不要用 ~
RAW=${RAW:-1}   # 1=录原始雷达点云(可离线重跑FAST-LIO), 0=省空间

# ---------- stop ----------
if [ "${1:-}" = "stop" ]; then
  echo '[stop] closing rosbag (正常收尾) ...'
  pkill -INT -f 'rosbag record' 2>/dev/null
  sleep 2
  echo '[stop] killing fastlio / livox ...'
  pkill -f mapping_mid360.launch 2>/dev/null
  pkill -f msg_MID360.launch     2>/dev/null
  sleep 1
  echo "[stop] done. bag 文件在: $BAGDIR"
  exit 0
fi

# ---------- 前置检查: odom_to_mavros 不能在跑 (避免 FAST-LIO 定位进飞控) ----------
if pgrep -f 'odom_to_mavros.py' >/dev/null; then
  echo '[!!] 检测到 odom_to_mavros.py 正在运行 —— 它会把 FAST-LIO 定位发给飞控!'
  echo '     本脚本要求只记录不使用, 请先: ./start_sensors.sh stop  或  pkill -f odom_to_mavros.py'
  exit 1
fi

mkdir -p "$BAGDIR"

# 磁盘检查 (raw 点云全录约 300MB/min, 压缩后减半)
avail_gb=$(df --output=avail -BG "$BAGDIR" 2>/dev/null | tail -1 | tr -dc '0-9')
if [ -n "$avail_gb" ] && [ "$avail_gb" -lt 5 ]; then
  echo "[!!] 磁盘剩余 ${avail_gb}GB < 5GB, 空间不足。请先清理: ls -lh $BAGDIR"
  exit 1
fi

# ---------- 0. roscore ----------
if ! pgrep -x rosmaster >/dev/null; then
  echo '[0/3] starting roscore ...'
  nohup roscore > /tmp/roscore_record.log 2>&1 &
  for i in $(seq 1 20); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
fi
echo '[0/3] roscore ready'

# ---------- 1. Livox (已在跑则跳过, 避免与动捕链路冲突) ----------
if pgrep -f 'msg_MID360.launch' >/dev/null; then
  echo '[1/3] livox driver 已在运行, 跳过启动'
else
  echo '[1/3] livox driver ...'
  nohup env ROS_NAMESPACE=drone_0 roslaunch livox_ros_driver2 msg_MID360.launch > /tmp/livox_record.log 2>&1 &
  sleep 4
fi

# ---------- 2. FAST-LIO (已在跑则跳过) ----------
if pgrep -f 'mapping_mid360.launch' >/dev/null; then
  echo '[2/3] fast-lio 已在运行, 跳过启动'
else
  echo '[2/3] fast-lio (只记录, 不发飞控) ...'
  nohup env ROS_NAMESPACE=drone_0 roslaunch fast_lio mapping_mid360.launch > /tmp/fastlio_record.log 2>&1 &
  sleep 4
fi

# ---------- 3. rosbag record ----------
echo '[3/3] rosbag record ...'
STAMP=$(date +%Y%m%d_%H%M%S)
TOPICS="/drone_0/Odometry /drone_0/cloud_registered /drone_0/livox/imu"
if [ "$RAW" = "1" ]; then
  TOPICS="$TOPICS /drone_0/livox/lidar"
fi
# mavros 若在跑(动捕链路), 顺便录 EKF 融合输出, 方便三方对比: 动捕 vs FAST-LIO vs EKF
if rostopic list 2>/dev/null | grep -q '^/drone_0/mavros/local_position/odom$'; then
  TOPICS="$TOPICS /drone_0/mavros/local_position/odom"
  echo '      (检测到 mavros, 已加入 EKF odom 录制)'
fi

nohup rosbag record -O "$BAGDIR/fastlio_$STAMP" --lz4 $TOPICS > "$BAGDIR/record_$STAMP.log" 2>&1 &
sleep 2

echo '---------------------------------------------'
echo "bag   : $BAGDIR/fastlio_$STAMP.bag"
echo "topics: $TOPICS"
echo ''
echo '说明:'
echo '  - FAST-LIO 定位只录不发飞控 (odom_to_mavros 未启动)'
echo '  - raw lidar+IMU 可离线重跑 FAST-LIO: rosbag play <bag> 后另改 topic 重放'
echo '  - 空间参考: 全录约 300MB/min(lz4后约一半); RAW=0 约 40MB/min'
echo '检查:  rostopic hz /drone_0/Odometry   (应 ~10Hz)'
echo '       tail -f '"$BAGDIR"'/record_'"$STAMP"'.log'
echo '停止:  ./record_fastlio.sh stop   (会正常关闭 bag 文件)'
