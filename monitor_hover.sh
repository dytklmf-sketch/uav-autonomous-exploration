#!/usr/bin/env bash
# 只读监控: 不发任何控制命令
source /opt/ros/noetic/setup.bash
source /home/mm/fuel_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.31.130:11311
DUR=${1:-150}
OUT=/home/mm/hover_monitor_$(date +%H%M%S).log
echo "monitor $DUR s -> $OUT"

# 1) px4ctrl 状态机切换 + force_hover (后台跟随 rosout)
( stdbuf -oL rostopic echo /rosout 2>/dev/null | stdbuf -oL grep -iE 'AUTO_HOVER|MANUAL_CTRL|AUTO_TAKEOFF|CMD_CTRL|AUTO_LAND|force_hover|Reject|TRIGGER suppressed|PVA Mode' ) > ${OUT}.fsm 2>&1 &
P1=$!
# 2) mavros state (mode/armed) 变化
( stdbuf -oL rostopic echo /drone_0/mavros/state 2>/dev/null ) > ${OUT}.state 2>&1 &
P2=$!
# 3) 周期采样 RC + setpoint hz
END=$((SECONDS+DUR))
while [ $SECONDS -lt $END ]; do
  T=$(date +%H:%M:%S)
  RC=$(timeout 2 rostopic echo -n1 /drone_0/mavros/rc/in/channels 2>/dev/null | tr -d '\n')
  MODE=$(timeout 2 rostopic echo -n1 /drone_0/mavros/state/mode 2>/dev/null | tr -d '\n')
  SP=$(timeout 3 rostopic hz -w 10 /drone_0/mavros/setpoint_raw/local 2>/dev/null | grep -m1 average | tr -d '\n')
  echo "[$T] mode=$MODE rc=$RC sp_hz=[$SP]" >> ${OUT}.sample
done
kill $P1 $P2 2>/dev/null
echo 'DONE' >> ${OUT}.sample
