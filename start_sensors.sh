#!/usr/bin/env bash
# start_sensors.sh —— 新机适配版 (nvidia @ /mnt/nvme/ws)
# 一键启动/停止 步骤1-4: Livox / CJ02-IMU / FAST-LIO / MAVROS / odom_to_mavros (+诊断日志)
# 2026-09-21: FAST-LIO 已切换为外部 CJ02-IMU (mid360_cj02.yaml, 外参已双IMU标定)
#   回退内置IMU: ./start_sensors.sh.bak_before_cj02_20260921
# 用法:  ./start_sensors.sh          启动
#        ./start_sensors.sh stop     停止
set -u

set +u; source /opt/ros/noetic/setup.bash; set -u
# ROS 主从机(2026-09-21): 锁定 wlan0 IP, 供远程电脑(192.168.31.136) RViz/rostopic 连接
export ROS_IP=192.168.31.141
export ROS_MASTER_URI=http://192.168.31.141:11311
set +u; source /mnt/nvme/ws/fastlio_ws/devel/setup.bash; set -u
set +u; source /mnt/nvme/ws/cj02_ws/devel/setup.bash; set -u   # overlay, 提供 cj02_imu_node

# ---------- 新机路径 ----------
FASTLIO_WS=/mnt/nvme/ws/fastlio_ws
HOME_DIR=/home/nvidia

# ---------- stop ----------
if [ "${1:-}" = "stop" ]; then
  echo '[stop] killing 1-4 ...'
  pkill -f diverge_logger.py        2>/dev/null
  pkill -f odom_to_mavros.py        2>/dev/null
  pkill -f 'mavros .*px4.launch'    2>/dev/null
  pkill -f mapping_mid360_cj02.launch 2>/dev/null
  pkill -f mapping_mid360.launch    2>/dev/null
  pkill -f cj02_imu.launch          2>/dev/null
  pkill -f msg_MID360.launch        2>/dev/null
  sleep 1
  echo '[stop] done. (roscore 保留, 如需关闭: pkill -x rosmaster)'
  exit 0
fi

# ---------- start ----------
LOGDIR=${HOME_DIR}/sensor_logs/$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOGDIR"
echo "logs -> $LOGDIR"

wait_topic () {  # $1=topic  $2=timeout_s
  local t=0
  while [ $t -lt ${2} ]; do
    rostopic list 2>/dev/null | grep -q "^$1$" && return 0
    sleep 1; t=$((t+1))
  done
  return 1
}

# 0. roscore
if ! pgrep -x rosmaster >/dev/null; then
  echo '[0/4] starting roscore ...'
  nohup roscore > "$LOGDIR/roscore.log" 2>&1 &
  for i in $(seq 1 20); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
fi
echo '[0/4] roscore ready'

# 1. Livox
echo '[1/4] livox driver ...'
nohup env ROS_NAMESPACE=drone_0 roslaunch livox_ros_driver2 msg_MID360.launch > "$LOGDIR/livox.log" 2>&1 &
sleep 4

# 1.5 CJ02-IMU (外部IMU, 必须先于 FAST-LIO 就绪)
echo '[1.5/4] cj02-imu driver ...'
nohup roslaunch cj02_imu_node cj02_imu.launch > "$LOGDIR/cj02_imu.log" 2>&1 &
if wait_topic /drone_0/cj02/imu/data_raw 10; then
  echo '  [OK] /drone_0/cj02/imu/data_raw 就绪'
else
  echo '  [!!] CJ02 未就绪, 看 $LOGDIR/cj02_imu.log (FAST-LIO 仍将启动但会退化为纯IMU)'
fi
sleep 1

# 2. FAST-LIO (CJ02 版: /drone_0/cj02/imu/data_raw + 标定外参; 原版 launch 保留可回退)
echo '[2/4] fast-lio (cj02) ...'
nohup env ROS_NAMESPACE=drone_0 roslaunch fast_lio mapping_mid360_cj02.launch > "$LOGDIR/fastlio.log" 2>&1 &
sleep 4

# 3. MAVROS (新机 sudo 密码: nvidia)
echo '[3/4] mavros ...'
echo nvidia | sudo -S chmod 777 /dev/ttyACM0 2>/dev/null
nohup env ROS_NAMESPACE=drone_0 roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 tgt_system:=1 > "$LOGDIR/mavros.log" 2>&1 &
sleep 6

# 4. odom_to_mavros (home 下标定+ROLL180 补偿版)
echo '[4/4] odom_to_mavros ...'
cd ${HOME_DIR}
nohup python3 odom_to_mavros.py > "$LOGDIR/odom_to_mavros.log" 2>&1 &
sleep 3

echo '---------------------------------------------'

# 5. diverge_logger (三流同步诊断记录; 不需要可 LOG_ALL=0 跳过)
if [ "${LOG_ALL:-1}" = "1" ]; then
  echo "[5] diverge_logger ..."
  nohup python3 ${HOME_DIR}/diverge_logger.py "$LOGDIR" > "$LOGDIR/diverge_logger.stdout" 2>&1 &
  sleep 2
fi

echo '就绪检测 (最多等 15s):'
wait_topic /drone_0/mavros/local_position/odom 15 && echo '  [OK] /drone_0/mavros/local_position/odom 有数据' || echo "  [!!] odom 未就绪, 看 $LOGDIR 下日志"
wait_topic /drone_0/Odometry 10 && echo '  [OK] /drone_0/Odometry (FAST-LIO/CJ02) 有数据' || echo '  [!!] FAST-LIO Odometry 未就绪, 看 $LOGDIR/fastlio.log'
echo "已启动. 日志: $LOGDIR"
echo '检查:  rosnode list   /   rostopic hz /drone_0/mavros/local_position/odom'
echo '停止:  ./start_sensors.sh stop'
