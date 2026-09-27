# 运行命令整理

本文档整理了当前对话中用到的主要运行、编译、排查与录制命令，方便后续直接查用。

## 1. 编译与环境加载

### 1.1 编译 `px4ctrl`
```bash
cd /home/nvidia/fuel_ws
catkin_make --pkg px4ctrl
source /home/nvidia/fuel_ws/devel/setup.bash
```

### 1.2 加载工作区环境
```bash
source /home/nvidia/fuel_ws/devel/setup.bash
```

### 1.3 加载 Fast-LIO 工作区环境
```bash
source ~/fastlio_ws/devel/setup.bash
```

---

## 2. 真机链路启动命令

### 2.1 启动 Livox 雷达
```bash
source ~/fastlio_ws/devel/setup.bash
ROS_NAMESPACE=drone_0 roslaunch livox_ros_driver2 msg_MID360.launch
```

### 2.2 启动 Fast-LIO
```bash
source ~/fastlio_ws/devel/setup.bash
ROS_NAMESPACE=drone_0 roslaunch fast_lio mapping_mid360.launch
```

### 2.3 启动 MAVROS / PX4
```bash
source /home/nvidia/fuel_ws/devel/setup.bash
sudo chmod 777 /dev/ttyACM0
ROS_NAMESPACE=drone_0 roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 tgt_system:=1
```

### 2.4 启动位姿转换脚本
```bash
source /home/nvidia/fuel_ws/devel/setup.bash
python3 odom_to_mavros.py
```

### 2.5 启动 `px4ctrl`
```bash
source /home/nvidia/fuel_ws/devel/setup.bash
roslaunch px4ctrl run_ctrl.launch auto_takeoff:=false
```

---

## 3. 自动起飞 / 降落相关命令

### 3.1 手动发送起飞命令
```bash
source /home/nvidia/fuel_ws/devel/setup.bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "{takeoff_land_cmd: 1}"
```

### 3.2 手动发送降落命令
```bash
source /home/nvidia/fuel_ws/devel/setup.bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "{takeoff_land_cmd: 2}"
```

### 3.3 查看 `TakeoffLand` 消息是否可用
```bash
source /home/nvidia/fuel_ws/devel/setup.bash
rosmsg show quadrotor_msgs/TakeoffLand
```

---

## 4. 状态与话题排查命令

### 4.1 查看 `px4ctrl` 节点连接情况
```bash
rosnode info /px4ctrl
```

### 4.2 查看关键话题
```bash
rostopic list | grep -E "takeoff_land|local_position|rc/in|mavros/state"
```

### 4.3 查看 RC 输入
```bash
rostopic echo /drone_0/mavros/rc/in
```

### 4.4 查看里程计频率
```bash
rostopic hz /drone_0/mavros/local_position/odom
```

### 4.5 查看位姿频率
```bash
rostopic hz /drone_0/mavros/local_position/pose
```

### 4.6 查看位姿内容
```bash
rostopic echo /drone_0/mavros/local_position/pose
```

### 4.7 查看规划命令是否在持续发布
```bash
rostopic hz /planning/pos_cmd
```

---

## 5. USB 摄像头录制命令

脚本位置：
```bash
/home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/px4ctrl/scripts/record_usb_camera.py
```

### 5.1 直接录制 KS-WDR 摄像头视频（无图形界面）
```bash
python3 /home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/px4ctrl/scripts/record_usb_camera.py \
  --device /dev/video6 \
  --output ~/ks_wdr_record.mp4 \
  --width 1920 \
  --height 1080 \
  --fps 30 \
  --no-preview
```

### 5.2 默认参数录制
```bash
python3 /home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/px4ctrl/scripts/record_usb_camera.py \
  --device /dev/video6 \
  --output ~/ks_wdr_record.mp4 \
  --no-preview
```

### 5.3 如果画面倒置，增加翻转参数
```bash
python3 /home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/px4ctrl/scripts/record_usb_camera.py \
  --device /dev/video6 \
  --output ~/ks_wdr_record.mp4 \
  --width 1920 \
  --height 1080 \
  --fps 30 \
  --flip \
  --no-preview
```

### 5.4 结束录制
```bash
Ctrl+C
```

---

## 6. 摄像头设备排查命令

### 6.1 查看所有视频设备
```bash
ls /dev/video*
```

### 6.2 查看摄像头设备列表
```bash
v4l2-ctl --list-devices
```

### 6.3 查看 `/dev/video6` 支持的格式与分辨率
```bash
v4l2-ctl -d /dev/video6 --list-formats-ext
```

### 6.4 查看 `/dev/video7` 支持的格式与分辨率
```bash
v4l2-ctl -d /dev/video7 --list-formats-ext
```

### 6.5 如果没有 `v4l2-ctl`，安装工具
```bash
sudo apt-get install v4l-utils
```

---

## 7. 当前排查中确认过的设备结论

### 7.1 KS-WDR 对应设备
```text
KS-WDR: KS-WDR (usb-3610000.xhci-3.2):
    /dev/video6
    /dev/video7
    /dev/media3
```

### 7.2 当前测试结论
- `/dev/video6`：可正常读帧。
- `/dev/video7`：当前不可用，不建议用于录制。

---

## 8. 常用备注

- 真机测试前，优先确认：
  - `ROS_NAMESPACE=drone_0`
  - `/drone_0/mavros/...` 话题存在
  - `px4ctrl` 已重新编译并重启
- 发送 `TakeoffLand` 命令前，记得先：
  ```bash
  source /home/nvidia/fuel_ws/devel/setup.bash
  ```
- 当前自动起飞高度配置已改为：
  - `takeoff_height: 0.5`
  - `takeoff_land_speed: 0.2`
