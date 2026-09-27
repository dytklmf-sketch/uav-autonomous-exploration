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

### 2.3 设置环境变量和无人机编号

编辑 `~/.bashrc`，添加以下内容（**根据你的实际情况修改**）：

```bash
# ROS 环境
source /opt/ros/noetic/setup.bash

# 网络配置（修改为你的 Jetson 实际 IP）
export ROS_IP=192.168.31.141
export ROS_MASTER_URI=http://192.168.31.141:11311

# 无人机编号（0 或 1，根据你的设置）
# 如果你的命名空间是 /drone_0，则设置为 0
# 如果你的命名空间是 /drone_1，则设置为 1
export DRONE_ID=0
```

应用环境变量：
```bash
source ~/.bashrc
```

### 2.4 配置 Livox MID-360 雷达 IP

Livox MID-360 默认 IP 是 `192.168.1.1XX`，需要配置为与你的网络同网段。

#### 方法 1：修改配置文件（推荐，最快）

编辑雷达驱动配置文件：

```bash
# 编译工作空间后修改
vim /mnt/nvme/ws/fastlio_ws/src/livox_ros_driver2/config/MID360_config.json
```

修改两处 IP：

```json
{
  "MID360": {
    "host_net_info" : {
      "cmd_data_ip" : "192.168.31.141",    ← 改成你的 Jetson IP
      "push_msg_ip": "192.168.31.141",     ← 改成你的 Jetson IP
      "point_data_ip": "192.168.31.141",   ← 改成你的 Jetson IP
      "imu_data_ip" : "192.168.31.141",    ← 改成你的 Jetson IP
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.31.12",              ← 改成你的雷达 IP
    }
  ]
}
```

**然后配置雷达 IP**（只需要做一次）：

#### 方法 2：通过 Livox Viewer 配置雷达 IP（一次性操作）

1. 在 Windows 电脑上下载并安装 [Livox Viewer](https://www.livoxtech.com/cn/downloads)
2. 用网线将雷达连接到电脑
3. 配置电脑 IP 为 `192.168.1.50`（与雷达默认 IP 同网段）
4. 打开 Livox Viewer，点击设备
5. 修改雷达 IP 为 `192.168.31.12`（与 Jetson 同网段）
6. 修改子网掩码为 `255.255.255.0`
7. 点击应用并重启雷达

**配置完成后，雷达 IP 永久保存，以后只需修改配置文件即可。**

#### 验证雷达连接

```bash
# 将 Jetson 网口 IP 设置为 192.168.31.141
sudo ifconfig eth0 192.168.31.141

# ping 雷达 IP
ping 192.168.31.12

# 应该能够正常 ping 通
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

### 4.2 配置无人机编号

⚠️ **重要**：所有脚本默认使用 `drone_0` 命名空间。如果你需要使用 `drone_1`，运行配置脚本：

```bash
# 配置为 drone_0（默认，无需修改）
./config_drone_id.sh 0

# 或配置为 drone_1
./config_drone_id.sh 1

# 重新加载环境变量
source ~/.bashrc
```

此脚本会自动修改：
- 所有 shell 脚本中的命名空间
- 所有 Python 脚本中的命名空间
- FAST-LIO launch 文件
- ~/.bashrc 中的 DRONE_ID 环境变量

### 4.3 复制配置文件

```bash
# 复制 FAST-LIO 配置（如果还没有）
cp mid360_cj02.yaml /mnt/nvme/ws/fastlio_ws/src/FAST_LIO/config/
```

---

## ⚙️ 第五步：配置 PX4 飞控参数

### 5.1 连接飞控

确保飞控通过 USB 连接到 `/dev/ttyACM0`（或其他串口）。

### 5.2 传感器校准（必须！）

⚠️ **重要**：在首次使用或更换飞控后，必须完成所有传感器校准。

使用 QGroundControl 完成以下校准：

#### 1. 加速度计校准
- QGC → 设置 → 传感器 → Accelerometer
- 按提示将飞机放置在 6 个方向（前后左右上下）
- 每个方向保持静止直到提示音响起

#### 2. 陀螺仪校准
- QGC → 设置 → 传感器 → Gyroscope
- 将飞机放置在水平面上保持静止
- 等待校准完成（约 5-10 秒）

#### 3. 磁力计校准（如果使用指南针）
- QGC → 设置 → 传感器 → Compass
- 按提示旋转飞机，覆盖所有方向
- 如果不使用 GPS/指南针，可以跳过

#### 4. 水平校准
- QGC → 设置 → 传感器 → Level Horizon
- 将飞机放置在水平面上
- 点击校准并等待完成

#### 5. 遥控器校准
- QGC → 设置 → 遥控器 → 校准
- 按提示移动所有摇杆和开关到极限位置
- 确保所有通道正确识别

### 5.3 电机和机架配置

⚠️ **危险操作**：电机校准时会转动螺旋桨，**必须先拆除螺旋桨！**

#### 1. 设置机架类型
- QGC → 设置 → 机架
- 选择对应的机架类型（四旋翼、X 型等）
- 应用并重启

#### 2. 电调（ESC）校准
拆除所有螺旋桨后：
- QGC → 设置 → 电源
- 点击 "ESC 校准"
- 按提示操作：
  1. 拔掉电池
  2. 将油门推到最大
  3. 插上电池
  4. 听到提示音后将油门拉到最低
  5. 等待电调校准完成

#### 3. 电机方向测试
拆除所有螺旋桨后：
- QGC → 设置 → 电机
- 逐个测试每个电机：
  - 滑动滑块使电机转动
  - 确认电机编号与实际位置对应
  - 确认转动方向正确（X 型：前左/后右逆时针，前右/后左顺时针）
- ⚠️ 如果方向错误，需要调换电机连接线中的任意两根

#### 4. 电池电压校准
- QGC → 设置 → 电源
- 设置电池串数（如 6S）
- 用万用表测量实际电池电压
- 输入实际电压进行校准

### 5.4 设置 PX4 参数

使用自动脚本设置参数：

```bash
# 启动 MAVROS
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 &

# 等待 MAVROS 连接
sleep 5

# 运行参数设置脚本
cd ~
./setup_px4_params.sh
```

或手动设置关键参数（见 [px4_params.txt](px4_params.txt)）。

### 5.5 遥控器通道设置

在 QGC 中配置：
- **CH5**：Kill Switch（急停开关）
  - 低位：正常飞行
  - 高位：立即切换到手动模式（急停）
  
- **CH6**：飞行模式切换
  - 高位：Offboard（允许板外控制）
  - 中位：Position（位置保持）
  - 低位：Manual（手动模式）

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
