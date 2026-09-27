#!/usr/bin/env bash
# 只读监控: frontier / 地图 / FSM 状态, 不发任何控制命令
source /opt/ros/noetic/setup.bash
source /home/mm/fuel_ws/devel/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI=http://192.168.31.130:11311
DUR=${1:-180}
OUT=/home/mm/frontier_mon_$(date +%H%M%S).log
echo "monitor ${DUR}s -> $OUT"

# 1) FSM + frontier + 关键事件 (跟随 rosout)
( stdbuf -oL rostopic echo /rosout/msg 2>/dev/null \
   | stdbuf -oL grep -iE 'state:|Triggered|Coverable|Frontier|NO_FRONTIER|GO_HOME|finish|cluster|Box|new num|to visit|remove' \
) > ${OUT}.fsm 2>&1 &
P1=$!

# 2) 周期采样: 深度图/点云/地图话题频率
END=$((SECONDS+DUR))
while [ $SECONDS -lt $END ]; do
  T=$(date +%H:%M:%S)
  depth=$(timeout 3 rostopic hz -w 5 /drone_0/camera/aligned_depth_to_color/image_raw 2>/dev/null | grep -m1 average | tr -d '\n')
  occ=$(timeout 3 rostopic hz -w 5 /sdf_map/occupancy_all 2>/dev/null | grep -m1 average | tr -d '\n')
  odom=$(timeout 2 rostopic echo -n1 /drone_0/mavros/local_position/odom/pose/pose/position 2>/dev/null | tr '\n' ' ')
  echo "[$T] depth_hz=[$depth] occ_hz=[$occ] odom=[$odom]" >> ${OUT}.sample
done
kill $P1 2>/dev/null
echo DONE >> ${OUT}.sample
