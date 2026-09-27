#!/bin/bash
# PX4 参数自动设置脚本
# 用于配置 FAST-LIO + FUEL 自主探索所需的飞控参数

set -e

echo "================================================"
echo "  PX4 参数配置脚本"
echo "================================================"
echo ""
echo "⚠️  确保："
echo "  1. 飞控已通过 USB 连接到 /dev/ttyACM0"
echo "  2. MAVROS 正在运行"
echo ""
read -p "按 Enter 继续，或 Ctrl+C 取消..."

# 检查 MAVROS 是否运行
if ! rostopic list 2>/dev/null | grep -q '/mavros/'; then
  echo ""
  echo "错误: MAVROS 未运行！"
  echo "请先启动 MAVROS："
  echo "  roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600"
  exit 1
fi

echo ""
echo "开始设置参数..."
echo ""

# 函数：设置参数
set_param() {
  local param=$1
  local value=$2
  echo -n "设置 $param = $value ... "
  if timeout 5 rosrun mavros mavparam set $param $value >/dev/null 2>&1; then
    echo "✓"
  else
    echo "✗ (失败)"
  fi
  sleep 0.2
}

# ============================================
# EKF2 外部定位参数
# ============================================
echo "--- EKF2 外部定位参数 ---"
set_param EKF2_EV_CTRL 15
set_param EKF2_HGT_REF 3
set_param EKF2_EV_DELAY 0
set_param EKF2_EV_POS_X 0.0
set_param EKF2_EV_POS_Y 0.0
set_param EKF2_EV_POS_Z 0.0

# ============================================
# 起飞和降落参数
# ============================================
echo ""
echo "--- 起飞和降落参数 ---"
set_param MIS_TAKEOFF_ALT 0.5
set_param COM_DISARM_LAND 3.0
set_param MPC_LAND_SPEED 0.3

# ============================================
# 遥控器映射
# ============================================
echo ""
echo "--- 遥控器映射 ---"
set_param RC_MAP_FLTMODE 6

# ============================================
# 板外控制参数
# ============================================
echo ""
echo "--- 板外控制参数 ---"
set_param COM_OBS_AVOID 0
set_param COM_RC_OVERRIDE 0

# ============================================
# 位置控制参数
# ============================================
echo ""
echo "--- 位置控制参数 ---"
set_param MPC_XY_VEL_MAX 1.5
set_param MPC_Z_VEL_MAX_UP 1.0
set_param MPC_Z_VEL_MAX_DN 0.8
set_param MPC_XY_P 1.0
set_param MPC_Z_P 1.0

# ============================================
# 安全参数
# ============================================
echo ""
echo "--- 安全参数 ---"
set_param COM_ARM_WO_GPS 1
set_param NAV_RCL_ACT 0
set_param NAV_DLL_ACT 0
set_param GF_ACTION 0

echo ""
echo "================================================"
echo "  参数设置完成！"
echo "================================================"
echo ""
echo "⚠️  重要提示："
echo "  1. 重启飞控使参数生效"
echo "  2. 在 QGroundControl 中完成传感器校准："
echo "     - 加速度计校准"
echo "     - 陀螺仪校准"
echo "     - 磁力计校准（如果使用）"
echo "     - 水平校准"
echo "     - 遥控器校准"
echo "  3. 设置遥控器飞行模式开关（CH6）："
echo "     - 高位：Offboard（允许板外控制）"
echo "     - 中位：Position"
echo "     - 低位：Manual"
echo "  4. 设置 CH5 为急停开关："
echo "     - 低位：正常"
echo "     - 高位：切换到手动模式（急停）"
echo ""
