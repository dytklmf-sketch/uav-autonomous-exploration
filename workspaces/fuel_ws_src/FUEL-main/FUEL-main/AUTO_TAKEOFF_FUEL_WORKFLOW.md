# FUEL Real-Flight Auto Takeoff Workflow

This note records the current Jetson/FUEL setup for real-flight startup, auto takeoff, locked hover, landing, and debugging.

## 1. Current Changes

### 1.1 Locked AUTO_HOVER

Source file:

```bash
/home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/px4ctrl/src/PX4CtrlFSM.cpp
```

`AUTO_HOVER` now keeps the `hover_pose` captured when the controller enters hover. It no longer integrates RC stick input in hover.

Expected behavior:

- After `AUTO_TAKEOFF --> AUTO_HOVER(L2)`, the vehicle should hold the position and altitude at the moment it entered hover.
- Throttle stick offset should not keep increasing hover altitude.
- RC hover/command mode checks are still active.
- If `/planning/pos_cmd` is active and command mode is enabled, the controller can still enter `CMD_CTRL` and follow planner commands.

Backup:

```bash
/home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/px4ctrl/src/PX4CtrlFSM.cpp.bak_20260531_hover_lock
```

### 1.2 Auto takeoff integrated into the total launch

Launch file:

```bash
/home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/fuel_planner/exploration_manager/launch/exploration.launch
```

Added args:

```xml
<arg name="pose_topic" value="/drone_$(arg drone_id)/mavros/local_position/pose" />
<arg name="takeoff_topic" value="/drone_$(arg drone_id)/px4ctrl/takeoff_land" />
<arg name="auto_takeoff" default="false" />
<arg name="takeoff_height" default="0.5" />
<arg name="stable_speed_thresh" default="0.15" />
<arg name="stable_duration" default="2.0" />
```

When `auto_takeoff:=true`, the total launch starts `px4ctrl/scripts/auto_takeoff.py`. The script waits for stable pose, then publishes `TAKEOFF=1` to the takeoff/land topic.

Backup:

```bash
/home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/fuel_planner/exploration_manager/launch/exploration.launch.bak_20260531_auto_takeoff
```

## 2. Real-Flight Startup Order

### 2.1 Start Livox

```bash
source ~/fastlio_ws/devel/setup.bash
ROS_NAMESPACE=drone_0 roslaunch livox_ros_driver2 msg_MID360.launch
```

### 2.2 Start FAST-LIO

```bash
source ~/fastlio_ws/devel/setup.bash
ROS_NAMESPACE=drone_0 roslaunch fast_lio mapping_mid360.launch
```

### 2.3 Start MAVROS

```bash
source ~/fuel_ws/devel/setup.bash
sudo chmod 777 /dev/ttyACM0
ROS_NAMESPACE=drone_0 roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 tgt_system:=1
```

### 2.4 Start odom conversion

```bash
source ~/fuel_ws/devel/setup.bash
cd ~
python3 odom_to_mavros.py
```

### 2.5 Start the FUEL total launch

Without auto takeoff:

```bash
source ~/fuel_ws/devel/setup.bash
roslaunch exploration_manager exploration.launch
```

With auto takeoff:

```bash
source ~/fuel_ws/devel/setup.bash
roslaunch exploration_manager exploration.launch auto_takeoff:=true
```

## 3. Takeoff and Hover Behavior

The auto takeoff node listens to:

```bash
/drone_0/mavros/local_position/pose
```

After pose is stable for about `2.0s`, it publishes to:

```bash
/drone_0/px4ctrl/takeoff_land
```

Message:

```yaml
takeoff_land_cmd: 1
```

The actual takeoff height is controlled by px4ctrl params:

```bash
/home/nvidia/fuel_ws/src/FUEL-main/FUEL-main/px4ctrl/config/ctrl_param_fpv.yaml
```

Current key params:

```yaml
auto_takeoff_land:
    enable: true
    enable_auto_arm: true
    no_RC: false
    takeoff_height: 0.5
    takeoff_land_speed: 0.2
```

## 4. RC Requirements

Current config uses `no_RC: false`, so px4ctrl still checks RC mode switches.

Before takeoff, inspect RC input:

```bash
rostopic echo /drone_0/mavros/rc/in
```

Expected values:

```text
channels[0] ~ channels[3] around 1500
channels[4] > 1750    # hover mode
channels[5] > 1750    # command mode
```

Important: `AUTO_HOVER` no longer uses RC stick input to change hover altitude, but RC mode switches can still make px4ctrl leave `AUTO_HOVER`.

## 5. Manual Takeoff and Landing

If not using auto takeoff, publish commands manually.

Load the FUEL workspace first:

```bash
source ~/fuel_ws/devel/setup.bash
rosmsg show quadrotor_msgs/TakeoffLand
```

Takeoff:

```bash
rostopic pub -1 /drone_0/px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "{takeoff_land_cmd: 1}"
```

Landing:

```bash
rostopic pub -1 /drone_0/px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "{takeoff_land_cmd: 2}"
```

Command values:

```text
TAKEOFF = 1
LAND = 2
```

## 6. Start FUEL Exploration

The exploration FSM waits for a trigger. A simple trigger can be sent to:

```bash
/move_base_simple/goal
```

Example:

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"header:
  frame_id: 'world'
pose:
  position: {x: 0.0, y: 0.0, z: 0.5}
  orientation: {w: 1.0}"
```

For the first test, only verify this sequence:

```text
auto takeoff -> AUTO_HOVER(L2) -> locked hover
```

Do not trigger FUEL exploration until hover is stable.

## 7. Debug Commands

Check takeoff/land subscriber:

```bash
rostopic info /drone_0/px4ctrl/takeoff_land
```

Check odom:

```bash
rostopic hz /drone_0/mavros/local_position/odom
rostopic echo -n 1 /drone_0/mavros/local_position/odom
```

Check MAVROS state:

```bash
rostopic echo /drone_0/mavros/state
```

Check whether planner commands are taking over:

```bash
rostopic hz /planning/pos_cmd
rostopic echo /planning/pos_cmd
```

If logs show:

```text
[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)
```

but the vehicle still moves, first check whether px4ctrl entered `CMD_CTRL` or whether `/planning/pos_cmd` is being published.

## 8. Build px4ctrl

After changing px4ctrl source, rebuild it:

```bash
source /opt/ros/noetic/setup.bash
cd ~/fuel_ws
catkin_make --pkg px4ctrl
source ~/fuel_ws/devel/setup.bash
```

After rebuilding, restart the running px4ctrl process or restart the total launch. A running old process will not use the new binary.
