#!/bin/bash
# 无人机编号配置脚本
# 使用方法: ./config_drone_id.sh 0  (或 1)

set -e

if [ -z "$1" ]; then
  echo "用法: $0 <drone_id>"
  echo "例如: $0 0    # 配置为 drone_0"
  echo "      $0 1    # 配置为 drone_1"
  exit 1
fi

DRONE_ID=$1
OLD_NS="drone_0"
NEW_NS="drone_${DRONE_ID}"

if [ "$DRONE_ID" != "0" ] && [ "$DRONE_ID" != "1" ]; then
  echo "错误: DRONE_ID 只能是 0 或 1"
  exit 1
fi

echo "================================================"
echo "  配置无人机编号: $NEW_NS"
echo "================================================"

# 1. 更新 ~/.bashrc 中的 DRONE_ID
if grep -q "export DRONE_ID=" ~/.bashrc; then
  sed -i "s/export DRONE_ID=.*/export DRONE_ID=${DRONE_ID}/" ~/.bashrc
  echo "✓ 更新 ~/.bashrc 中的 DRONE_ID=${DRONE_ID}"
else
  echo "export DRONE_ID=${DRONE_ID}" >> ~/.bashrc
  echo "✓ 添加 DRONE_ID=${DRONE_ID} 到 ~/.bashrc"
fi

# 2. 更新所有 shell 脚本中的命名空间
echo ""
echo "正在更新 shell 脚本..."
for f in ~/*.sh; do
  [ -f "$f" ] || continue
  if grep -q "/drone_0\|drone_0" "$f"; then
    sed -i "s|/drone_0|/${NEW_NS}|g" "$f"
    sed -i "s|drone_0|${NEW_NS}|g" "$f"
    echo "  ✓ $(basename $f)"
  fi
done

# 3. 更新所有 Python 脚本中的命名空间
echo ""
echo "正在更新 Python 脚本..."
for f in ~/*.py; do
  [ -f "$f" ] || continue
  if grep -q "/drone_0\|drone_0" "$f"; then
    sed -i "s|/drone_0|/${NEW_NS}|g" "$f"
    sed -i "s|drone_0|${NEW_NS}|g" "$f"
    echo "  ✓ $(basename $f)"
  fi
done

# 4. 更新 FAST-LIO launch 文件
FASTLIO_LAUNCH="/mnt/nvme/ws/fastlio_ws/src/FAST_LIO/launch/mapping_mid360_cj02.launch"
if [ -f "$FASTLIO_LAUNCH" ]; then
  if grep -q "drone_0" "$FASTLIO_LAUNCH"; then
    sed -i "s|drone_0|${NEW_NS}|g" "$FASTLIO_LAUNCH"
    echo ""
    echo "✓ 更新 FAST-LIO launch 文件"
  fi
fi

# 5. 更新 odom_to_mavros.py 如果在 /mnt/nvme/ws/ 下
if [ -f "/mnt/nvme/ws/odom_to_mavros.py" ]; then
  sed -i "s|/drone_0|/${NEW_NS}|g" "/mnt/nvme/ws/odom_to_mavros.py"
  echo "✓ 更新 /mnt/nvme/ws/odom_to_mavros.py"
fi

echo ""
echo "================================================"
echo "  配置完成！"
echo "================================================"
echo ""
echo "无人机命名空间已更改为: /${NEW_NS}"
echo ""
echo "下一步:"
echo "  1. 重新加载环境变量: source ~/.bashrc"
echo "  2. 重新编译工作空间（如果修改了 launch 文件）"
echo "  3. 测试启动: ./start_sensors.sh"
echo ""
