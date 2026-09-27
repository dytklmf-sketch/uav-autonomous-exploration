# 无人机自主探索系统

基于 FUEL + FAST-LIO + PX4 的自主探索无人机完整代码包。

## 🚁 系统特性

- ✅ **FAST-LIO**: 使用 Livox MID-360 + CJ02 外部 IMU 进行实时建图定位
- ✅ **FUEL 自主探索**: 带自动返航原点功能
- ✅ **一键启动**: 自动化启动脚本，全栈启动只需一条命令
- ✅ **一键自动飞行**: 起飞 → 探索 → 返航 → 降落全自动
- ✅ **完整诊断工具**: 延迟检查、雷达质量、悬停诊断等

## 📁 项目结构

```
.
├── README.md                    # 本文件
├── UAV_CONFIG.md               # 完整配置说明文档（必读！）
├── FUEL_AUTO_README.md         # FUEL 自动飞行详细说明
│
├── start_sensors.sh            # 启动传感器栈（Livox/IMU/FAST-LIO/MAVROS）
├── start_all.sh                # 启动全栈（传感器 + FUEL）
├── start_fuel.sh               # 仅启动 FUEL
├── auto_fly.sh                 # 一键自动飞行
├── auto_takeoff_land.sh        # 自动起降测试
│
├── odom_to_mavros.py           # 里程计融合节点
├── air_diag.py                 # 空中诊断
├── hover_diag.py               # 悬停诊断
├── latency_check.py            # 延迟检查
├── lidar_quality.py            # 雷达质量检查
├── diverge_logger.py           # 三流同步诊断
├── monitor_pose.py             # 位姿监控
├── calib_dual_imu.py           # 双 IMU 标定
├── calib_lidar_fcu.py          # 雷达-飞控标定
│
├── record_dual_imu.sh          # 录制双 IMU 数据
├── record_calib_dual_imu.sh    # 录制标定数据
├── record_fastlio.sh           # 录制 FAST-LIO 数据
├── record_compare.sh           # 对比测试录制
├── compare_ab_fastlio.sh       # A/B 测试对比
│
├── monitor_hover.sh            # 悬停监控
├── mon_frontier.sh             # 前沿点监控
├── run_odom.sh                 # 运行里程计节点
│
└── mid360_cj02.yaml            # FAST-LIO CJ02 配置文件
```

## 🚀 快速开始

### 1. 启动系统

在无人机上运行：

```bash
# 启动全栈（传感器 + FUEL）
./start_all.sh

# 等待提示 "UAV_STACK_READY"
```

### 2. 自动飞行

```bash
# 确认遥控器设置：CH5 低位，CH6 高位
./auto_fly.sh

# 系统将自动完成：起飞 → 探索 → 返航 → 降落
```

### 3. 急停

**遥控器 CH5 拨到低位** → 立即切换到手动控制

### 4. 停止系统

```bash
./start_all.sh stop
```

## 📖 详细文档

**⚠️ 重要**: 首次使用前，请务必阅读 [UAV_CONFIG.md](UAV_CONFIG.md)，了解：
- 飞控参数配置
- 传感器标定步骤
- 遥控器设置要求
- 安全注意事项
- 重新搭建完整步骤

## ⚙️ 系统要求

### 硬件
- NVIDIA Jetson（机载计算机）
- PX4 飞控
- Livox MID-360 激光雷达
- CJ02 外部 IMU
- RealSense 深度相机（可选）

### 软件
- Ubuntu 18.04/20.04
- ROS Noetic
- FAST-LIO
- FUEL
- MAVROS

## 🛠️ 配置要点

### 网络配置
- 无人机 IP: `192.168.31.141`
- ROS Master: `http://192.168.31.141:11311`

### 遥控器设置
- **CH5**: 急停开关（高位=急停）
- **CH6**: Command 权限（必须高位，飞行中勿动）

### 探索范围
可通过环境变量设置：
```bash
BOX_X=2.5 BOX_Y=2.5 BOX_ZMIN=0.2 BOX_ZMAX=0.8 ./start_all.sh
```

## 📊 日志位置

- 传感器日志: `~/sensor_logs/YYYYMMDD_HHMMSS/`
- FUEL 调试日志: `~/fuel_debug_logs/`

## 🔧 常用命令

```bash
# 检查话题
rostopic list
rostopic hz /drone_0/mavros/local_position/odom
rostopic hz /drone_0/Odometry

# 查看参数
rosrun mavros mavparam get MIS_TAKEOFF_ALT
rosrun mavros mavparam get EKF2_EV_CTRL

# 导出参数
rosrun mavros mavparam dump px4_params.txt

# 查看最新日志
tail -f ~/sensor_logs/$(ls -t ~/sensor_logs/ | head -1)/fastlio.log
```

## 🐛 故障排查

### odom 不出数据
```bash
# 检查 CJ02 IMU
rostopic hz /drone_0/cj02/imu/data_raw

# 查看 FAST-LIO 日志
cat ~/sensor_logs/$(ls -t ~/sensor_logs/ | head -1)/fastlio.log
```

### 起飞失败
- 检查遥控器 CH5 是否在低位
- 确认 odom 正常输出
- 查看 MAVROS 连接: `rostopic echo /drone_0/mavros/state`

### FUEL 未激活
- 确认 CH6 在高位
- 检查目标点 Z 坐标 > -0.1
- 查看 FUEL 日志

更多问题请参考 [UAV_CONFIG.md](UAV_CONFIG.md) 第 9 节。

## 📝 版本信息

- **版本**: 1.0
- **最后更新**: 2026-09-27
- **ROS**: Noetic
- **适配无人机**: nvidia@192.168.31.141

## 🔗 相关资源

- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [Livox SDK](https://github.com/Livox-SDK/livox_ros_driver2)
- [PX4 Documentation](https://docs.px4.io/)
- [MAVROS](http://wiki.ros.org/mavros)

## ⚠️ 安全提示

1. **首次飞行必须在空旷场地测试**
2. **遥控器随时准备急停接管**
3. **确认空域安全后再起飞**
4. **留意飞机姿态，异常立即接管**
5. **自动起飞/降落为高风险动作，谨慎操作**

## 📄 许可证

本项目仅供学习和研究使用。

## 👤 联系方式

- 无人机 IP: nvidia@192.168.31.141
- 密码: nvidia

---

**如有任何问题，请先查阅 [UAV_CONFIG.md](UAV_CONFIG.md) 配置文档。**
