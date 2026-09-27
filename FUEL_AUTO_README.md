# FUEL 自主探索 + 自动返航 + 一键飞行

本文档记录在原生 FUEL 基础上新增的 **探索完成后自动返航原点 (GO_HOME)** 功能,
以及配套的 **一键启动 / 一键自动飞行** 脚本和完整操作流程。

---

## 1. 新增功能概述

### 1.1 GO_HOME 自动返航
原生 FUEL 探索完成 (规划器返回 `NO_FRONTIER`) 后直接进 `FINISH` 悬停在探索终点。
现在新增 `GO_HOME` 状态:探索完成后**自动规划避障轨迹飞回起飞原点 (0,0)**,
到达并停稳后再 `FINISH`。

特点:
- **目标硬编码为原点 (0,0, 当前飞行高度)**,水平飞回原点正上方,不会撞地。
- **迭代返航**:一段轨迹飞完后若未真正到达,自动重新规划再飞一段,直到贴近原点。
- **到达判定**:水平距离 < 0.3m **且** 速度 < 0.15m/s (真正停稳) 才判定到达。
- **可开关**:参数 `fsm/go_home_enable` (默认 true);设 false 则完全复现原生行为。
- 探索流程其余部分与原生完全一致。

### 1.2 改动的源码文件 (fuel_ws)
- `fast_exploration_fsm.h` — 新增 `GO_HOME` 枚举、`packHomeTraj()` 声明
- `expl_data.h` — FSMData 加 `home_pos_`/`go_home_planned_`;FSMParam 加 `go_home_enable_`/`home_arrive_thresh_`
- `fast_exploration_fsm.cpp` — `NO_FRONTIER` 分支转 GO_HOME;新增 GO_HOME 迭代返航逻辑 + 打包函数

---

## 2. 一键脚本

| 脚本 | 作用 |
|------|------|
| `~/start_all.sh`     | 一键启动全栈:传感器 (Livox/FAST-LIO/MAVROS/odom) + FUEL 探索 |
| `~/start_all.sh stop`| 一键停止全部节点 |
| `~/auto_fly.sh`      | 一键自动飞行:起飞→切板外→发目标点→探索→返航→自动降落 |

> 原有的 `~/start_sensors.sh` (步骤1-4) 和 `~/start_fuel.sh` (步骤5) 仍保留,
> `start_all.sh` 内部即调用 `start_sensors.sh` 再叠加 FUEL。

---

## 3. 完整操作流程

### 3.0 遥控器准备
- 摇杆全部居中
- **ch5 高位** (>1750):auto hover 钥匙 + 急停
- **ch6 高位**:command 权限 (放行 FUEL / 返航轨迹)

### 3.1 启动全栈 (一条命令)
```bash
~/start_all.sh
```
自动完成:传感器栈 → 等 odom 出数据 → 后台启 FUEL → 等 bspline 就绪。
看到 `全栈已启动.` 即可。

### 3.2 一键自动飞行 (一条命令)
```bash
~/auto_fly.sh
```
回车确认空域安全后,全自动执行 4 步:
1. **解锁 + AUTO.TAKEOFF** 自动起飞
2. **等爬到顶停稳** (高度达标 且 |vz|<0.08) → `force_hover` 切板外悬停
3. **发目标点** 激活 FUEL 自主探索
4. **监听完成信号** (`arrived origin` / `finish exploration`) → 自动 `AUTO.LAND` 降落

中途无需手动操作。探索完自动返航原点,落地后自动上锁。

### 3.3 急停 (任何时刻)
**遥控器 ch5 拨低位** → 立即回 MANUAL_CTRL 手动接管。

### 3.4 停止全栈
```bash
~/start_all.sh stop
```

---

## 4. 关键时机:为什么等"停稳"才切板外?

`force_hover` 会把**切的那一刻 odom 位置/高度锁成悬停点** (`set_hov_with_odom`)。
px4ctrl 只在速度 >3m/s 时拒绝,所以爬升中途也能切 —— 但会把飞机
**锁在半空中途高度、且带上升速度**,悬停点不干净。

因此 `auto_fly.sh` 在 force_hover 前要求:
**高度达标 且 |vz| < `VZ_SETTLE` 连续 `SETTLE_TICKS` 拍** (确认 PX4 爬到顶进 LOITER)。

---

## 5. 可调参数 (auto_fly.sh 顶部)

```bash
TAKEOFF_ALT=0.5      # 期望起飞高度(m), 须与 PX4 参数 MIS_TAKEOFF_ALT 一致
VZ_SETTLE=0.08       # 垂直速度阈值(m/s): 低于此值视为爬升到顶停稳
SETTLE_TICKS=1       # 需连续满足的拍数
GOAL_X=0.5; GOAL_Y=0.0; GOAL_Z=0.5   # 探索激活目标点 (z 必须 > -0.1)
```

GO_HOME 相关参数 (FUEL launch / 代码默认值):
```
fsm/go_home_enable     = true   # 是否启用返航
fsm/home_arrive_thresh = 0.3    # 到达判定水平距离阈值(m)
```

---

## 6. 首次运行前的核对

栈起来后 (3.1 之后)、起飞前,确认两项环境假设:

```bash
# 1. 话题命名空间是否与脚本一致
rostopic list | grep -E 'force_hover|waypoint_generator/waypoints'
#   脚本用: /drone_0/px4ctrl/force_hover  和  /waypoint_generator/waypoints
#   不一致 -> 改 auto_fly.sh

# 2. PX4 自动起飞高度, 须与脚本 TAKEOFF_ALT 一致
rosrun mavros mavparam get MIS_TAKEOFF_ALT
#   不一致 -> 改 auto_fly.sh 顶部 TAKEOFF_ALT
```

---

## 7. 验证返航效果 (可选)

```bash
latest=$(ls -dt /home/nvidia/fuel_debug_logs/*/ | head -1)
grep 'GO_HOME' $latest/rosout_px4ctrl.log
```
正常应看到:
- 多次 `replan to origin: traj end=(~0,~0,..)` — 一段段往回飞
- 最后 `arrived origin (dist=0.xx)` — dist 真 <0.3 且停稳才判到达

---

## 8. 重要提醒:板卡时钟问题与编译

板卡时钟会**倒退/跳变**,导致 `catkin_make` 靠 mtime 的增量判断失效,
会**静默跳过实际编译**(仍打印 "Built target",但二进制是旧的)。

**改完源码后,务必用强制重建**:
```bash
rm -f /home/nvidia/fuel_ws/devel/lib/exploration_manager/exploration_node
find /home/nvidia/fuel_ws/build -path '*exploration_node.dir*' -name '*.o' -delete
cd /home/nvidia/fuel_ws && catkin_make --pkg exploration_manager -j2
# 必须看到 "Linking CXX executable" 才算真编译了
```
验证二进制确含新逻辑:
```bash
strings /home/nvidia/fuel_ws/devel/lib/exploration_manager/exploration_node | grep 'replan to origin'
```

---

## 9. 安全注意事项

- 首次测试务必在**空旷场地**,遥控器 ch5 随时可急停接管。
- 自动起飞/降落为高风险动作,起飞前确认 odom 正常、飞机静止、空域安全。
- 返航走 kinodynamic 避障轨迹,但仍建议低速、留足接管余地。
- 源码改动均有 `.bak_gohome` 备份 (在各源文件同目录)。
