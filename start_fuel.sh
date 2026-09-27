#!/usr/bin/env bash
# start_fuel.sh —— 步骤5: FUEL 原生探索 (含 RealSense / px4ctrl / traj_server)
# 前提: 先跑 ~/start_sensors.sh 且 odom 已就绪
# 关键: 用 --extend 叠加 catkin_ws(realsense) 和 fuel_ws, 否则 RealSense nodelet 加载失败
set -u

source /opt/ros/noetic/setup.bash
source /home/mm/fuel_ws/devel/setup.bash
source /home/mm/catkin_ws/devel/setup.bash --extend
export LD_LIBRARY_PATH=/home/mm/catkin_ws/devel/lib:/home/mm/fuel_ws/devel/lib:/opt/ros/noetic/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}

# 自检: 两个工作区的关键包必须都可见
echo '[check] realsense plugin:'; rospack plugins --attrib=plugin nodelet 2>/dev/null | grep -i realsense || { echo '  [FATAL] realsense plugin 不可见, 中止'; exit 1; }
echo '[check] exploration_manager:'; rospack find exploration_manager >/dev/null 2>&1 && echo '  OK' || { echo '  [FATAL] fuel 不可见, 中止'; exit 1; }

echo '[run] roslaunch exploration_manager exploration.launch ...'
exec roslaunch exploration_manager exploration.launch debug_logger:=true debug_log_bag:=true
