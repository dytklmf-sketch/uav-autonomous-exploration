# 无人机配置说明文档

本文档记录本无人机相较于初始配置的所有修改，以及重新搭建一台同款无人机所需的全部配置步骤。

---

## 📋 目录

1. [硬件配置](#1-硬件配置)
2. [飞控参数修改](#2-飞控参数修改)
3. [传感器配置](#3-传感器配置)
4. [软件环境配置](#4-软件环境配置)
5. [网络配置](#5-网络配置)
6. [启动脚本配置](#6-启动脚本配置)
7. [FUEL探索参数](#7-fuel探索参数)
8. [重新搭建步骤](#8-重新搭建步骤)

---

## 1. 硬件配置

### 1.1 核心硬件
- **机载计算机**: NVIDIA Jetson (用户名: nvidia, 密码: nvidia)
- **飞控**: PX4 (通过 /dev/ttyACM0 连接，波特率 921600)
- **激光雷达**: Livox MID-360
- **IMU**: CJ02 外部 IMU（已替换内置 IMU）
- **深度相机**: RealSense（可选，用于 FUEL 探索）

### 1.2 硬件连接
- 飞控通过 USB 连接到 /dev/ttyACM0
- Livox MID-360 通过网络连接
- CJ02 IMU 独立连接
- 机载计算机存储挂载在 `/mnt/nvme/ws`

---

## 2. 飞控参数修改

### 2.1 关键 PX4 参数

以下参数需要在 PX4 飞控中设置（相较于出厂默认值的修改）：

#### 2.1.1 外部定位（Vision Position）
```
EKF2_EV_CTRL = 15           # 启用外部位置+航向融合
EKF2_EV_DELAY = 0           # 视觉延迟（ms），已通过 odom_to_mavros 同步
EKF2_HGT_REF = 3            # 高度参考源：Vision
EKF2_EV_POS_X = 0.0         # 相机/传感器相对重心的X偏移（m）
EKF2_EV_POS_Y = 0.0         # Y偏移（m）
EKF2_EV_POS_Z = 0.0         # Z偏移（m）
```

#### 2.1.2 起飞/降落参数
```
MIS_TAKEOFF_ALT = 0.5       # 自动起飞高度（m），可通过脚本动态修改
COM_DISARM_LAND = 3.0       # 降落后自动上锁延迟（秒）
```

#### 2.1.3 遥控器映射
```
RC_MAP_FLTMODE = 6          # CH6 为飞行模式切换通道
                            # CH5 为急停开关（高位=急停）
```

#### 2.1.4 板外控制（Offboard）
```
COM_OBS_AVOID = 1           # 启用避障
COM_RCL_EXCEPT = 4          # RC丢失时的行为
```

**重要提示**: 
- 遥控器 **CH5 必须高位** 才能启用 AUTO.HOVER 和急停功能
- 遥控器 **CH6 必须高位** 才能放行 FUEL/返航指令（command 权限）
- 飞行过程中不要拨动 CH6，会顶掉自动模式

---

## 3. 传感器配置

### 3.1 FAST-LIO 配置（已切换到 CJ02 外部 IMU）

**配置文件**: `/mnt/nvme/ws/fastlio_ws/src/FAST_LIO/config/mid360_cj02.yaml`

**关键修改**:

```yaml
common:
    imu_topic: "/livox/imu"    # 映射到 /drone_0/cj02/imu/data_raw

mapping:
    # 双 IMU 标定结果 (2026-09-21)
    extrinsic_est_en: true
    extrinsic_T: [0.00205, -0.02149, 0.06168]
    extrinsic_R: [0.99993, 0.01110, 0.00398,
                  -0.01117, 0.99979, 0.01740,
                  -0.00379, -0.01745, 0.99984]
```

**回退到内置 IMU** (如需要):
```bash
cp ~/start_sensors.sh.bak_before_cj02_20260921 ~/start_sensors.sh
```

### 3.2 Livox MID-360 配置

**Launch 文件**: `/mnt/nvme/ws/fastlio_ws/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch`

- 话题命名空间: `/drone_0`
- 输出话题: `/drone_0/livox/lidar`, `/drone_0/livox/imu`

### 3.3 CJ02 IMU 配置

**Launch 文件**: `/mnt/nvme/ws/cj02_ws/src/cj02_imu_node/launch/cj02_imu.launch`

- 输出话题: `/drone_0/cj02/imu/data_raw`
- 必须在 FAST-LIO 之前启动

### 3.4 里程计融合节点（odom_to_mavros.py）

**功能**: 将 FAST-LIO 输出的里程计转发给 PX4 和规划器

**配置**:
```python
offset_x = 0.0    # X 轴偏移（m）
offset_y = 0.0    # Y 轴偏移（m）
offset_z = 0.0    # Z 轴偏移（m）
drone_id = 0      # 无人机 ID
```

**输入**: `/drone_0/Odometry` (来自 FAST-LIO)

**输出**:
- `/drone_0/mavros/vision_pose/pose` (给 PX4 EKF2)
- `/drone_0/odom_world` (给 FUEL 规划器)

---

## 4. 软件环境配置

### 4.1 ROS 工作空间

**位置**: `/mnt/nvme/ws/`

包含以下工作空间：
- `fastlio_ws/` - FAST-LIO + Livox 驱动
- `cj02_ws/` - CJ02 IMU 驱动
- `fuel_ws/` - FUEL 自主探索
- `catkin_ws/` - RealSense 相机驱动（可选）

### 4.2 环境变量设置

**在 `~/.bashrc` 中**:
```bash
# ROS 主从机配置
export ROS_IP=192.168.31.141          # 本机 IP（wlan0）
export ROS_MASTER_URI=http://192.168.31.141:11311
```

**启动脚本中的 source 顺序**:
```bash
source /opt/ros/noetic/setup.bash
source /mnt/nvme/ws/fastlio_ws/devel/setup.bash
source /mnt/nvme/ws/cj02_ws/devel/setup.bash
source /mnt/nvme/ws/fuel_ws/devel/setup.bash
source /mnt/nvme/ws/catkin_ws/devel/setup.bash --extend  # 可选
```

---

## 5. 网络配置

### 5.1 无人机网络
- **接口**: wlan0
- **IP 地址**: 192.168.31.141（固定）
- **ROS Master**: http://192.168.31.141:11311

### 5.2 远程电脑配置

如需在远程电脑（如 192.168.31.136）上运行 RViz 或 rostopic：

```bash
export ROS_MASTER_URI=http://192.168.31.141:11311
export ROS_IP=192.168.31.136  # 改为你的电脑 IP
```

### 5.3 SSH 连接

```bash
ssh -i ~/.ssh/id_ed25519_model_health nvidia@192.168.31.141
# 或使用密码: nvidia
```

---

## 6. 启动脚本配置

### 6.1 start_sensors.sh（传感器栈）

**启动顺序**:
1. roscore
2. Livox MID-360 驱动
3. CJ02 IMU 驱动（1.5 步骤）
4. FAST-LIO（CJ02 版本）
5. MAVROS（PX4 通信）
6. odom_to_mavros（里程计融合）
7. diverge_logger（三流同步诊断，可选）

**使用方法**:
```bash
./start_sensors.sh        # 启动
./start_sensors.sh stop   # 停止
```

**日志位置**: `~/sensor_logs/YYYYMMDD_HHMMSS/`

### 6.2 start_all.sh（全栈启动）

**功能**: 启动传感器栈 + FUEL 探索栈

**探索边界参数**（环境变量）:
```bash
BOX_X=2.5        # X 方向半径（m）
BOX_Y=2.5        # Y 方向半径（m）
BOX_ZMIN=0.2     # 最低高度（m，绝对值）
BOX_ZMAX=0.8     # 最高高度（m，绝对值）
```

**使用示例**:
```bash
# 默认探索范围
./start_all.sh

# 自定义探索范围
BOX_X=3.0 BOX_Y=3.0 BOX_ZMIN=0.3 BOX_ZMAX=1.0 ./start_all.sh

# 停止全部
./start_all.sh stop
```

### 6.3 auto_fly.sh（一键自动飞行）

**功能**: 自动起飞 → 切板外 → 探索 → 返航 → 降落

**关键参数**:
```bash
TAKEOFF_ALT=0.5         # 起飞高度（m）
VZ_SETTLE=0.08          # 垂直速度阈值（m/s）
SETTLE_TICKS=1          # 停稳判定拍数
GOAL_X=0.5; GOAL_Y=0.0; GOAL_Z=0.5   # 探索激活目标点
EXPLORE_TIMEOUT=600     # 探索超时时间（秒，0=不限）
```

**使用方法**:
```bash
# 无人值守模式（直接执行）
./auto_fly.sh

# 人工确认模式
CONFIRM=1 ./auto_fly.sh

# 自定义起飞高度
TAKEOFF_ALT=1.0 ./auto_fly.sh
```

**前提条件**:
1. 已运行 `./start_all.sh` 且全栈就绪
2. 遥控器 CH5 低位（高位=急停）
3. 遥控器 CH6 高位（command 权限）
4. odom 正常输出

---

## 7. FUEL探索参数

### 7.1 GO_HOME 自动返航

**功能**: 探索完成后自动飞回原点 (0, 0)

**参数**（在 exploration.launch 或代码中）:
```
fsm/go_home_enable = true         # 启用返航（false=原生行为）
fsm/home_arrive_thresh = 0.3      # 到达判定距离（m）
```

**到达判定条件**:
- 水平距离 < 0.3m
- 速度 < 0.15m/s（确保停稳）

### 7.2 探索边界

通过 `start_all.sh` 的环境变量设置（见 6.2 节）

---

## 8. 重新搭建步骤

### 8.1 硬件连接

1. **飞控连接**: USB 线连接到 Jetson 的 /dev/ttyACM0
2. **Livox MID-360**: 网络连接并配置 IP
3. **CJ02 IMU**: 按厂家说明连接
4. **电源**: 确保飞控和机载计算机供电稳定

### 8.2 Jetson 系统配置

#### 基础环境
```bash
# 用户
用户名: nvidia
密码: nvidia

# 挂载 NVMe 存储
sudo mkdir -p /mnt/nvme
sudo mount /dev/nvme0n1p1 /mnt/nvme
# 添加到 /etc/fstab 以自动挂载
```

#### 安装 ROS Noetic
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
# 下载 GeographicLib 数据集
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 8.3 编译工作空间

#### FAST-LIO + Livox
```bash
mkdir -p /mnt/nvme/ws/fastlio_ws/src
cd /mnt/nvme/ws/fastlio_ws/src
# 克隆 FAST-LIO 和 livox_ros_driver2
git clone https://github.com/hku-mars/FAST_LIO.git
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ..
catkin_make
```

#### CJ02 IMU
```bash
mkdir -p /mnt/nvme/ws/cj02_ws/src
cd /mnt/nvme/ws/cj02_ws/src
# 克隆 CJ02 驱动
# git clone [CJ02驱动仓库]
cd ..
catkin_make
```

#### FUEL
```bash
mkdir -p /mnt/nvme/ws/fuel_ws/src
cd /mnt/nvme/ws/fuel_ws/src
# 克隆 FUEL
# git clone [FUEL仓库]
cd ..
catkin_make
```

### 8.4 配置文件复制

将本仓库中的以下文件复制到无人机：

```bash
# 启动脚本
scp *.sh nvidia@192.168.31.141:~/
scp *.py nvidia@192.168.31.141:~/

# FAST-LIO 配置
scp mid360_cj02.yaml nvidia@192.168.31.141:/mnt/nvme/ws/fastlio_ws/src/FAST_LIO/config/
```

### 8.5 校准外参

#### CJ02-雷达外参校准
```bash
# 1. 录制标定包（运动激励充分）
./record_calib_dual_imu.sh

# 2. 运行标定脚本
python3 calib_dual_imu.py

# 3. 将输出的外参写入 mid360_cj02.yaml 的 extrinsic_T 和 extrinsic_R
```

### 8.6 PX4 参数配置

使用 QGroundControl 或 mavparam 设置第 2 节中的所有参数。

### 8.7 网络配置

编辑 `~/.bashrc`，添加：
```bash
export ROS_IP=192.168.31.141  # 改为实际 IP
export ROS_MASTER_URI=http://192.168.31.141:11311
```

### 8.8 权限设置

```bash
# 给启动脚本添加执行权限
chmod +x ~/*.sh

# 飞控设备权限（或在启动脚本中用 sudo）
sudo chmod 777 /dev/ttyACM0

# 可选：添加 udev 规则自动设置权限
```

### 8.9 测试验证

#### 测试传感器栈
```bash
./start_sensors.sh

# 检查话题
rostopic list
rostopic hz /drone_0/mavros/local_position/odom
rostopic hz /drone_0/Odometry

# 停止
./start_sensors.sh stop
```

#### 测试全栈
```bash
./start_all.sh

# 检查 FUEL 是否就绪
rostopic list | grep bspline

# 停止
./start_all.sh stop
```

#### 首次飞行测试
```bash
# 1. 启动全栈
./start_all.sh

# 2. 确认遥控器设置（CH5 低位，CH6 高位）

# 3. 空旷场地测试自动飞行
CONFIRM=1 ./auto_fly.sh

# 随时准备用 CH5 急停接管！
```

---

## 9. 常见问题排查

### 9.1 odom 不出数据

**检查**:
- CJ02 IMU 是否就绪: `rostopic hz /drone_0/cj02/imu/data_raw`
- FAST-LIO 日志: `~/sensor_logs/[最新时间]/fastlio.log`
- Livox 连接: `rostopic hz /drone_0/livox/lidar`

### 9.2 自动起飞失败

**常见原因**:
- 飞控定位未就绪（EKF2 未收敛）
- 遥控器急停开关（CH5）在高位
- 飞控前置检查未通过
- MIS_TAKEOFF_ALT 参数未正确设置

### 9.3 FUEL 未激活

**检查**:
- 目标点 Z 坐标是否 > -0.1
- 遥控器 CH6 是否在高位（command 权限）
- FUEL 日志: `~/sensor_logs/fuel_*.log`

### 9.4 返航不回原点

**检查**:
- `fsm/go_home_enable` 是否为 true
- FUEL 日志中是否有 "replan to origin"
- 探索过程中 odom 是否漂移

### 9.5 时钟问题导致编译失败

板卡时钟可能倒退，导致增量编译失败。**强制重编译**:
```bash
rm -f /mnt/nvme/ws/fuel_ws/devel/lib/exploration_manager/exploration_node
find /mnt/nvme/ws/fuel_ws/build -path '*exploration_node.dir*' -name '*.o' -delete
cd /mnt/nvme/ws/fuel_ws && catkin_make --pkg exploration_manager -j2
```

---

## 10. 重要提醒

### 10.1 安全注意事项

⚠️ **首次测试必读**:
1. 在空旷场地测试
2. 遥控器 CH5 随时准备急停
3. 自动起飞/降落为高风险动作，确认空域安全
4. 返航走避障轨迹，但仍建议留足接管余地
5. 留意飞机姿态，异常立即接管

### 10.2 遥控器设置总结

- **CH5**: 急停开关
  - **高位**: AUTO.HOVER 钥匙 + 急停（板外失控时接管）
  - **低位**: 正常飞行
  
- **CH6**: Command 权限
  - **高位**: 放行 FUEL/返航轨迹（必须）
  - **低位**: 阻断指令
  - **飞行中切勿拨动**（会顶掉自动模式）

### 10.3 日志位置

所有运行日志保存在：
- 传感器: `~/sensor_logs/YYYYMMDD_HHMMSS/`
- FUEL 调试: `~/fuel_debug_logs/`

---

## 11. 文件清单

### 启动脚本
- `start_sensors.sh` - 传感器栈启动
- `start_all.sh` - 全栈启动（传感器 + FUEL）
- `start_fuel.sh` - 仅启动 FUEL
- `auto_fly.sh` - 一键自动飞行
- `auto_takeoff_land.sh` - 自动起降测试

### Python 脚本
- `odom_to_mavros.py` - 里程计融合节点
- `air_diag.py` - 空中诊断
- `hover_diag.py` - 悬停诊断
- `latency_check.py` - 延迟检查
- `lidar_quality.py` - 雷达质量检查
- `diverge_logger.py` - 三流同步诊断
- `monitor_pose.py` - 位姿监控
- `calib_dual_imu.py` - 双 IMU 标定
- `calib_lidar_fcu.py` - 雷达-飞控标定

### 数据记录脚本
- `record_dual_imu.sh` - 录制双 IMU 数据
- `record_calib_dual_imu.sh` - 录制标定数据
- `record_fastlio.sh` - 录制 FAST-LIO 数据
- `record_compare.sh` - 对比测试录制
- `compare_ab_fastlio.sh` - A/B 测试对比

### 监控脚本
- `monitor_hover.sh` - 悬停监控
- `mon_frontier.sh` - 前沿点监控

### 配置文件
- `mid360_cj02.yaml` - FAST-LIO CJ02 配置
- `FUEL_AUTO_README.md` - FUEL 自动飞行说明

---

## 12. 参考资料

- [FAST-LIO GitHub](https://github.com/hku-mars/FAST_LIO)
- [Livox SDK](https://github.com/Livox-SDK/livox_ros_driver2)
- [PX4 Autopilot](https://docs.px4.io/)
- [MAVROS](http://wiki.ros.org/mavros)
- FUEL 探索: 原生 + GO_HOME 返航（见 FUEL_AUTO_README.md）

---

## 13. 版本信息

- **文档版本**: 1.0
- **最后更新**: 2026-09-27
- **适配无人机**: nvidia@192.168.31.141
- **ROS 版本**: Noetic
- **PX4 版本**: (运行 `rosrun mavros mavparam get SYS_AUTOSTART` 查看)
- **最后标定日期**: 2026-09-21（双 IMU 外参）

---

**维护建议**:
- 每次修改重要参数后更新此文档
- 每次外参标定后记录标定日期和数据包
- 备份 PX4 参数: `rosrun mavros mavparam dump params_backup_YYYYMMDD.txt`
