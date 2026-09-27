#!/usr/bin/env bash
# auto_fly.sh —— 自动任务: 起飞 -> 切板外悬停 -> 发目标点激活FUEL -> 等返航finish -> 自动降落
# 前提: 先跑 ./start_all.sh 且 odom/FUEL 就绪; 遥控器 ch5 在低位(高位=急停kill, 拨高位随时接管)
#        CH6 是 PX4 模式开关(RC_MAP_FLTMODE=6), 飞行中拨动会顶掉自动模式, 全程勿动 CH6
# 用法:  ./auto_fly.sh    (环境变量: TAKEOFF_ALT / HOLD_TIME / CONFIRM=1 / EXPLORE_TIMEOUT / GOAL_X,Y,Z)
# 20260917 修复: set_mode/arming 失败判定(本机 MAVROS set_mode 返回 mode_sent 而非 success);
#               no_settle 失败时保护性 AUTO.LAND; 起飞顺序对齐 auto_takeoff_land.sh 定稿版; 探索监听加超时兜底
set -u
set +u; source /opt/ros/noetic/setup.bash; set -u
set +u; source /mnt/nvme/ws/fuel_ws/devel/setup.bash >/dev/null 2>&1; set -u
export ROS_MASTER_URI=http://192.168.31.141:11311
export ROS_IP=192.168.31.141

NS=/drone_0
TAKEOFF_ALT=${TAKEOFF_ALT:-0.5}   # 起飞高度(m), 可环境变量覆盖; 同步写入 MIS_TAKEOFF_ALT
VZ_SETTLE=0.08         # 垂直速度阈值(m/s): 低于此值视为爬升到顶停稳
SETTLE_TICKS=1         # 需连续满足的拍数, 确认进LOITER再切板外
GOAL_X=0.5; GOAL_Y=0.0; GOAL_Z=0.5   # 探索激活目标点(z 必须 > -0.1)
EXPLORE_TIMEOUT=${EXPLORE_TIMEOUT:-600}  # 探索/返航最长等待(s), 超时强制 AUTO.LAND, 0=不限

echo '############################################################'
echo '# 自动飞行任务  —— 安全提示:'
echo '#  CH5 高位=急停(随时可拨); CH6 是模式开关, 全程勿动; 留意姿态'
echo '############################################################'
# 无人值守模式: 默认直接执行 (供无人车 SSH 调用).
# 若需人工确认, 运行前加环境变量 CONFIRM=1
if [ "${CONFIRM:-0}" = "1" ]; then
  read -r -p '确认环境就绪、空域安全, 回车开始 / Ctrl+C 取消 ... ' _
else
  echo '[auto] 无人值守模式 (CONFIRM=1 可启用人工确认)'
fi

get_z ()  { rostopic echo -n1 ${NS}/mavros/local_position/odom/pose/pose/position/z 2>/dev/null | head -1; }
get_vz () { rostopic echo -n1 ${NS}/mavros/local_position/odom/twist/twist/linear/z 2>/dev/null | head -1; }

# 失败时保护性降落再退出 (飞机可能已在空中)
fail_land () {
  echo "[!!] 警告: ${1} —— 飞机可能仍解锁, 发送 AUTO.LAND 保护性降落, 请立即手动接管!"
  rosservice call ${NS}/mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.LAND'" >/dev/null 2>&1
  echo "UAV_MISSION_FAILED ${1}"
  exit 1
}

# ---------- 步骤0: 同步起飞高度到飞控 (MIS_TAKEOFF_ALT) ----------
echo "[0/4] 设置 PX4 起飞高度 MIS_TAKEOFF_ALT=${TAKEOFF_ALT} ..."
rosrun mavros mavparam -n ${NS#/}/mavros set MIS_TAKEOFF_ALT ${TAKEOFF_ALT} >/dev/null 2>&1
gotalt=$(rosrun mavros mavparam -n ${NS#/}/mavros get MIS_TAKEOFF_ALT 2>/dev/null | tail -1)
echo "[0/4] 读回 MIS_TAKEOFF_ALT=${gotalt}"
ok=$(awk "BEGIN{d=${gotalt:-0}-${TAKEOFF_ALT}; print (d<0.01 && d>-0.01)?1:0}" 2>/dev/null)
if [ "$ok" != "1" ]; then
  echo "UAV_MISSION_FAILED set_takeoff_alt (期望 ${TAKEOFF_ALT}, 实读 ${gotalt})"; exit 1
fi

# ---------- 步骤1: POSCTL -> 解锁 -> 停稳 -> AUTO.TAKEOFF (对齐 auto_takeoff_land.sh 定稿顺序) ----------
echo '[1/4] 设置 POSCTL 定点模式 (清残留模式) ...'
resp=$(rosservice call ${NS}/mavros/set_mode "base_mode: 0
custom_mode: 'POSCTL'")
echo "$resp"
echo "$resp" | grep -q 'mode_sent: True' || { echo 'UAV_MISSION_FAILED set_posctl'; exit 1; }

echo '[1/4] 解锁 (arming) ...'
resp=$(rosservice call ${NS}/mavros/cmd/arming "value: true")
echo "$resp"
echo "$resp" | grep -q 'success: True' || { echo 'UAV_MISSION_FAILED arming (常见原因: kill开关激活/前置检查未过/定位未就绪)'; exit 1; }

z0=$(get_z); z0=${z0:-0}
tgt=$(awk "BEGIN{print ${z0}+${TAKEOFF_ALT}}" 2>/dev/null)
echo "[!] 切模式瞬间 EKF z = ${z0}  →  实际目标高度 ≈ ${tgt}m  (若 ${z0} 明显为负 = 起飞基准被记低, 高度将不足)"

echo '[1/4] 切 AUTO.TAKEOFF 自动起飞 ...'
resp=$(rosservice call ${NS}/mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.TAKEOFF'")
echo "$resp"
echo "$resp" | grep -q 'mode_sent: True' || fail_land 'takeoff_mode'

# 等爬升【到顶并停稳】: 高度达标 且 垂直速度连续多拍趋零 (PX4 已进LOITER)
# 关键: force_hover 会把切的那一刻 odom 位置/高度锁为悬停点;
#       若在上升中途切, 会锁在半空且带上升速度. 必须等停稳再切.
echo "[1/4] 等待爬升到 ${TAKEOFF_ALT}m 且停稳 (vz<${VZ_SETTLE}, 连续${SETTLE_TICKS}拍) ..."
reached=0; settle=0
for i in $(seq 1 40); do
  z=$(get_z);   z=${z:-0}
  vz=$(get_vz); vz=${vz:-9}
  vzabs=$(awk "BEGIN{v=${vz}; print (v<0)?-v:v}" 2>/dev/null)
  echo "    z=${z}  vz=${vz}  settle=${settle}"
  alt_ok=$(awk "BEGIN{print (${z} >= ${TAKEOFF_ALT}-0.1)?1:0}" 2>/dev/null)
  vz_ok=$(awk "BEGIN{print (${vzabs} < ${VZ_SETTLE})?1:0}" 2>/dev/null)
  if [ "$alt_ok" = "1" ] && [ "$vz_ok" = "1" ]; then
    settle=$((settle+1))
    if [ "$settle" -ge "${SETTLE_TICKS}" ]; then reached=1; break; fi
  else
    settle=0
  fi
  sleep 1
done
if [ "$reached" != "1" ]; then
  fail_land 'no_settle'
fi
echo '[1/4] 已到顶停稳, 可安全切板外'; echo 'UAV_TAKEOFF_OK'

# ---------- 步骤2: 切板外悬停 (force_hover -> OFFBOARD) ----------
echo '[2/4] 切板外悬停 force_hover ...'
rostopic pub -1 ${NS}/px4ctrl/force_hover std_msgs/Empty "{}"
sleep 3

# ---------- 步骤3: 发目标点激活 FUEL 探索 ----------
# --once 只发一次就退出, 订阅者未接上时消息会丢. -1 -l 挂住消息直到确认发出.
echo '[3/4] 发送目标点激活 FUEL 探索 ...'
: > /tmp/fuel_trigger_watch.log
timeout 25 rostopic echo /rosout/msg > /tmp/fuel_trigger_watch.log 2>/dev/null &
watch_pid=$!
sleep 1
rostopic pub -1 -l /waypoint_generator/waypoints nav_msgs/Path "header: {frame_id: 'world'}
poses:
- pose:
    position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}
    orientation: {w: 1.0}"
trig=0
for i in $(seq 1 20); do
  if grep -q 'Triggered!' /tmp/fuel_trigger_watch.log; then trig=1; break; fi
  if timeout 2 rostopic echo -n1 /rosout/msg 2>/dev/null | grep -q 'PLAN_TRAJ'; then trig=1; break; fi
  sleep 1
done
kill $watch_pid 2>/dev/null || true
wait $watch_pid 2>/dev/null || true
if [ "$trig" != "1" ]; then
  fail_land 'fuel_not_triggered'
fi
echo '[3/4] 已激活, FUEL 开始自主探索; 完成后将自动返航原点'; echo 'UAV_EXPLORE_STARTED'

# ---------- 步骤4: 监听 finish/返航到达, 自动降落 (带超时兜底) ----------
echo "[4/4] 监听探索/返航完成信号 (arrived origin / finish exploration), 超时 ${EXPLORE_TIMEOUT}s ..."
if [ "${EXPLORE_TIMEOUT}" = "0" ]; then
  stdbuf -oL rostopic echo /rosout/msg 2>/dev/null \
    | grep -m1 -E 'arrived origin|finish exploration' >/dev/null
else
  if ! timeout ${EXPLORE_TIMEOUT} stdbuf -oL rostopic echo /rosout/msg 2>/dev/null \
    | grep -m1 -E 'arrived origin|finish exploration' >/dev/null; then
    fail_land "explore_timeout_${EXPLORE_TIMEOUT}s"
  fi
fi
echo '[4/4] 收到完成信号 -> 发送 AUTO.LAND 自动降落'
resp=$(rosservice call ${NS}/mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.LAND'")
echo "$resp"
echo "$resp" | grep -q 'mode_sent: True' || fail_land 'land_mode'
echo '[done] 已发送降落指令. 飞机落地后会自动上锁; 留意是否需手动接管.'; echo 'UAV_MISSION_DONE'
