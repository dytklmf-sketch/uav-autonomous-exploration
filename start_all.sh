#!/usr/bin/env bash
# start_all.sh —— 一键启动全栈: 前置传感器栈(1-4) + FUEL 探索(5)
# 新机适配版 (nvidia @ /mnt/nvme/ws): FUEL 未编译时自动只跑前置栈
# 2026-09-21: 传感器栈已随 start_sensors.sh 切换 CJ02-IMU; stop 段同步补齐 CJ02 清理
#   回退内置IMU: cp ~/start_sensors.sh.bak_before_cj02_20260921 ~/start_sensors.sh
# 用法:  ./start_all.sh          启动
#        ./start_all.sh stop     停止全部
set -u

set +u; source /opt/ros/noetic/setup.bash; set -u

# ROS 主从机(2026-09-21): 锁定 wlan0 IP, 供远程电脑(192.168.31.136) RViz/rostopic 连接
export ROS_IP=192.168.31.141
export ROS_MASTER_URI=http://192.168.31.141:11311

if [ "${1:-}" = "stop" ]; then
  echo '[stop] killing FUEL + sensors ...'
  pkill -f exploration.launch       2>/dev/null
  pkill -f exploration_node         2>/dev/null
  pkill -f traj_server              2>/dev/null
  pkill -f px4ctrl                  2>/dev/null
  pkill -f rs_camera                2>/dev/null
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

# ===================== 步骤 1-4: 传感器栈 =====================
rm -f /tmp/uav_ready /tmp/uav_failed 2>/dev/null
echo '================ [PHASE 1] 传感器栈 (1-4) ================'
bash /home/nvidia/start_sensors.sh
if [ $? -ne 0 ]; then echo '[FATAL] 传感器栈启动失败, 中止'; exit 1; fi

# 等 odom 真正出数据 (最多 20s)
echo '[wait] 等待 /drone_0/mavros/local_position/odom 出数据 ...'
ok=0
for i in $(seq 1 20); do
  if rostopic echo -n1 /drone_0/mavros/local_position/odom >/dev/null 2>&1; then ok=1; break; fi
  sleep 1
done
if [ "$ok" != "1" ]; then
  echo '[FATAL] odom 未就绪, 中止 (检查 ~/sensor_logs 下日志)'; exit 1
fi
echo '[OK] odom 就绪'

# ===================== 步骤 5: FUEL 探索栈 (可选) =====================
FUEL_WS=/mnt/nvme/ws/fuel_ws
CATKIN_WS=/mnt/nvme/ws/catkin_ws
if [ ! -f ${FUEL_WS}/devel/setup.bash ]; then
  echo '[INFO] FUEL 未编译 (${FUEL_WS}/devel 不存在), 跳过探索栈'
  touch /tmp/uav_ready
  echo 'UAV_STACK_READY (sensors only)'
  exit 0
fi

echo '================ [PHASE 2] FUEL 探索栈 (5) ================'
set +u; source ${FUEL_WS}/devel/setup.bash; set -u
# realsense 依赖的 catkin_ws (存在才加载; 未接 realsense 相机时跳过)
if [ -f ${CATKIN_WS}/devel/setup.bash ]; then
  set +u; source ${CATKIN_WS}/devel/setup.bash --extend; set -u
  export LD_LIBRARY_PATH=${CATKIN_WS}/devel/lib:${FUEL_WS}/devel/lib:/opt/ros/noetic/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}
else
  echo '[WARN] catkin_ws (realsense) 未编译, 跳过加载'
  export LD_LIBRARY_PATH=${FUEL_WS}/devel/lib:/opt/ros/noetic/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}
fi

echo '[check] realsense plugin:'
if rospack plugins --attrib=plugin nodelet 2>/dev/null | grep -qi realsense; then
  echo '  OK'
else
  echo '  [WARN] realsense plugin 不可见 (未接 realsense 时正常; FUEL 探索将无深度感知输入)'
fi
echo '[check] exploration_manager:'; rospack find exploration_manager >/dev/null 2>&1 && echo '  OK' || { echo '  [FATAL] fuel 不可见, 中止'; exit 1; }

# ---- 探索 box 传参 (环境变量, 半径式; z 用绝对 min/max) ----
# 用法: BOX_X=2.5 BOX_Y=3.0 BOX_ZMIN=0.2 BOX_ZMAX=1.5 bash ~/start_all.sh
BOX_X=${BOX_X:-2.5}
BOX_Y=${BOX_Y:-2.5}
BOX_ZMIN=${BOX_ZMIN:-0.2}
BOX_ZMAX=${BOX_ZMAX:-0.8}
BOX_ARGS="box_min_x:=-${BOX_X} box_max_x:=${BOX_X} box_min_y:=-${BOX_Y} box_max_y:=${BOX_Y} box_min_z:=${BOX_ZMIN} box_max_z:=${BOX_ZMAX}"
echo "[box] x=+/-${BOX_X}  y=+/-${BOX_Y}  z=${BOX_ZMIN}~${BOX_ZMAX}"

FUELLOG=/home/nvidia/sensor_logs/fuel_$(date +%Y%m%d_%H%M%S).log
echo "[run] FUEL 后台启动, 日志 -> $FUELLOG"
nohup roslaunch exploration_manager exploration.launch debug_logger:=true debug_log_bag:=true $BOX_ARGS > "$FUELLOG" 2>&1 &

# 等 px4ctrl / FSM 就绪 (bspline 话题出现)
echo '[wait] 等待 FUEL 就绪 (最多 25s) ...'
fuel_ok=0
for i in $(seq 1 30); do
  if rostopic list 2>/dev/null | grep -q "/drone_0/planning/bspline"; then fuel_ok=1; break; fi
  sleep 1
done
echo "---------------------------------------------"
if [ "$fuel_ok" = "1" ]; then
  touch /tmp/uav_ready
  echo "  FUEL log: $FUELLOG"
  echo "UAV_STACK_READY"
  exit 0
else
  touch /tmp/uav_failed
  echo "[FATAL] FUEL not ready, see $FUELLOG"
  echo "UAV_STACK_FAILED"
  exit 1
fi
