# 从零到飞 - 完整部署指南

本文档提供从全新 Jetson 板卡到实现自动飞行的完整步骤。

---

## 📋 前置条件

### 硬件要求
- ✅ NVIDIA Jetson（Orin/Xavier/TX2）
- ✅ PX4 飞控（通过 USB 连接）
- ✅ Livox MID-360 激光雷达
- ✅ CJ02 外部 IMU
- ✅ 遥控器（需要 CH5 和 CH6 通道）
- ✅ 已挂载的 NVMe 存储（/mnt/nvme）

### 软件要求
- Ubuntu 18.04 或 20.04
- 至少 32GB 存储空间
- 网络连接

---

## 🚀 第一步：克隆仓库

```bash
cd /mnt/nvme
git clone https://github.com/dytklmf-sketch/uav-autonomous-exploration.git
cd uav-autonomous-exploration
```

---

## 🔧 第二步：安装依赖

### 2.1 安装 ROS Noetic（使用小鱼一键安装）

```bash
# 使用小鱼的一键安装脚本
wget http://fishros.com/install -O fishros && . fishros

# 根据提示选择：
# 1. 选择 ROS 版本：Noetic（适用于 Ubuntu 20.04）
# 2. 选择安装类型：Desktop-Full（完整版）
# 3. 等待安装完成

# 安装 MAVROS
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras

# 下载 GeographicLib 数据集
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 2.2 安装编译工具

```bash
sudo apt install -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev
```

### 2.3 设置环境变量

编辑 `~/.bashrc`，添加以下内容（**修改 IP 为你的实际 IP**）：

```bash
# ROS 环境
source /opt/ros/noetic/setup.bash

# 网络配置（修改为你的实际 IP）
export ROS_IP=192.168.31.141
export ROS_MASTER_URI=http://192.168.31.141:11311
```

应用环境变量：
```bash
source ~/.bashrc
```

---

## 🏗️ 第三步：编译工作空间

### 3.1 创建工作空间目录结构

```bash
mkdir -p /mnt/nvme/ws
cd /mnt/nvme/ws
```

### 3.2 复制源码到工作空间

```bash
# 复制 FAST-LIO 工作空间
cp -r /mnt/nvme/uav-autonomous-exploration/workspaces/fastlio_ws_src /mnt/nvme/ws/fastlio_ws/src

# 复制 FUEL 工作空间
cp -r /mnt/nvme/uav-autonomous-exploration/workspaces/fuel_ws_src/FUEL-main/FUEL-main/* /mnt/nvme/ws/fuel_ws/src/

# 复制 CJ02 IMU 工作空间
cp -r /mnt/nvme/uav-autonomous-exploration/workspaces/cj02_ws_src /mnt/nvme/ws/cj02_ws/src

# 复制 RealSense 工作空间（可选）
cp -r /mnt/nvme/uav-autonomous-exploration/workspaces/catkin_ws_src /mnt/nvme/ws/catkin_ws/src
```

### 3.3 编译 FAST-LIO

```bash
cd /mnt/nvme/ws/fastlio_ws
catkin_make
source devel/setup.bash
```

### 3.4 编译 CJ02 IMU 驱动

```bash
cd /mnt/nvme/ws/cj02_ws
catkin_make
source devel/setup.bash
```

### 3.5 编译 FUEL

```bash
cd /mnt/nvme/ws/fuel_ws
catkin_make
source devel/setup.bash
```

### 3.6 编译 RealSense（可选）

```bash
cd /mnt/nvme/ws/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 📝 第四步：复制脚本和配置文件

### 4.1 复制启动脚本到家目录

```bash
cd /mnt/nvme/uav-autonomous-exploration
cp *.sh ~/
cp *.py ~/
chmod +x ~/*.sh
chmod +x ~/*.py
```

### 4.2 复制配置文件

```bash
# 复制 FAST-LIO 配置（如果还没有）
cp mid360_cj02.yaml /mnt/nvme/ws/fastlio_ws/src/FAST_LIO/config/
```

---

## ⚙️ 第五步：配置 PX4 飞控参数

### 5.1 连接飞控

确保飞控通过 USB 连接到 `/dev/ttyACM0`（或其他串口）。

### 5.2 设置关键参数

使用 QGroundControl 或命令行设置以下参数：

```bash
# 启动 MAVROS 连接飞控
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600

# 在另一个终端设置参数
rosrun mavros mavparam set EKF2_EV_CTRL 15
rosrun mavros mavparam set EKF2_HGT_REF 3
rosrun mavros mavparam set MIS_TAKEOFF_ALT 0.5
rosrun mavros mavparam set COM_DISARM_LAND 3.0
rosrun mavros mavparam set RC_MAP_FLTMODE 6
```

完整参数列表请参考 [UAV_CONFIG.md](UAV_CONFIG.md) 第 2 节。

---

## 🧪 第六步：测试传感器栈

### 6.1 启动传感器

```bash
cd ~
./start_sensors.sh
```

等待提示 `UAV_STACK_READY`。

### 6.2 验证数据流

在另一个终端检查话题：

```bash
# 检查里程计输出
rostopic hz /drone_0/Odometry
rostopic hz /drone_0/mavros/local_position/odom

# 检查 IMU
rostopic hz /drone_0/cj02/imu/data_raw

# 检查雷达
rostopic hz /drone_0/livox/lidar
```

### 6.3 停止传感器

```bash
./start_sensors.sh stop
```

---

## 🛫 第七步：首次飞行测试

### 7.1 启动全栈

```bash
./start_all.sh
```

等待提示 `UAV_STACK_READY`。

### 7.2 检查遥控器设置

- **CH5**：拨到**低位**（高位 = 急停）
- **CH6**：拨到**高位**（放行 command 权限）

### 7.3 运行自动飞行

⚠️ **在空旷场地测试，随时准备急停！**

```bash
# 人工确认模式（推荐首次使用）
CONFIRM=1 ./auto_fly.sh
```

系统将自动完成：
1. 起飞到 0.5m
2. 切换到板外控制
3. 发送探索目标点
4. 自主探索
5. 返航到原点
6. 降落并上锁

### 7.4 急停

随时用**遥控器 CH5 拨到高位**立即切换到手动控制。

---

## 📊 第八步：查看日志

```bash
# 查看传感器日志
ls ~/sensor_logs/

# 查看最新日志
tail -f ~/sensor_logs/$(ls -t ~/sensor_logs/ | head -1)/fastlio.log
```

---

## 🔍 常见问题排查

### 问题 1：odom 不出数据

**检查**：
```bash
rostopic hz /drone_0/cj02/imu/data_raw
cat ~/sensor_logs/$(ls -t ~/sensor_logs/ | head -1)/fastlio.log
```

**可能原因**：
- CJ02 IMU 未连接
- FAST-LIO 配置错误
- Livox 雷达未连接

### 问题 2：起飞失败

**检查**：
- 遥控器 CH5 是否在低位
- odom 是否正常输出
- MAVROS 连接状态：`rostopic echo /drone_0/mavros/state`

### 问题 3：FUEL 未激活

**检查**：
- 遥控器 CH6 是否在高位
- 目标点 Z 坐标是否 > -0.1
- 查看 FUEL 日志

### 问题 4：无法编译

**尝试**：
```bash
# 清理并重新编译
cd /mnt/nvme/ws/fuel_ws
catkin clean -y
catkin_make
```

---

## 📖 进阶配置

### 自定义探索范围

```bash
# 设置探索边界（米）
BOX_X=3.0 BOX_Y=3.0 BOX_ZMIN=0.3 BOX_ZMAX=1.0 ./start_all.sh
```

### 自定义起飞高度

```bash
TAKEOFF_ALT=1.0 ./auto_fly.sh
```

### 外参标定

如果 IMU 或雷达位置改变，需要重新标定外参：

```bash
# 录制标定数据
./record_calib_dual_imu.sh

# 运行标定
python3 calib_dual_imu.py

# 将结果写入 /mnt/nvme/ws/fastlio_ws/src/FAST_LIO/config/mid360_cj02.yaml
```

---

## ✅ 部署检查清单

- [ ] ROS Noetic 已安装
- [ ] MAVROS 已安装并下载 GeographicLib
- [ ] 工作空间已编译成功
- [ ] 脚本已复制到家目录
- [ ] PX4 参数已设置
- [ ] 传感器栈测试通过
- [ ] odom 正常输出
- [ ] 遥控器 CH5/CH6 设置正确
- [ ] 首次飞行在空旷场地测试

---

## 🆘 获取帮助

- **详细配置说明**：[UAV_CONFIG.md](UAV_CONFIG.md)
- **FUEL 使用说明**：[FUEL_AUTO_README.md](FUEL_AUTO_README.md)
- **GitHub Issues**：https://github.com/dytklmf-sketch/uav-autonomous-exploration/issues

---

## 🔐 安全提示

1. ⚠️ **首次飞行必须在空旷场地**
2. ⚠️ **遥控器随时准备急停（CH5 高位）**
3. ⚠️ **确认空域安全后再起飞**
4. ⚠️ **留意飞机姿态，异常立即接管**
5. ⚠️ **自动起降为高风险动作，谨慎操作**

---

**祝飞行顺利！🚁**
