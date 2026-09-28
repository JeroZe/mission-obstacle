# mid360_obstacle_stop

Mission 模式下 MID360 检测到障碍物 → 请求 PX4 自己停车并 Hold 的 ROS 2 package。

- ROS 2 只负责：感知 / 决策 / 请求 Mission Pause
- 减速、停车、悬停全部由 PX4 完成（不使用 Offboard，不发送 setpoint，不发速度 0）
- 第一阶段只做 360° 固定安全圆柱，不做避障/路径规划

---

## 0. 必须先知道的接口差异（已逐条核对源代码）

你给的方案里有两个假设与 **PX4 v1.16 / px4_msgs release/1.16** 的真实情况不一致。
我没有偷偷绕过，而是按核实结果实现，并把你的方案作为可切换选项保留下来。

| 项目 | 原始假设 | 核实结果（PX4 v1.16.0 / px4_msgs release/1.16） | 本 package 的做法 |
| --- | --- | --- | --- |
| `NAVIGATION_STATE_AUTO_MISSION` | 4 | **3**（`NAVIGATION_STATE_AUTO_LOITER` 才是 4；1.15 及更早版本 AUTO_MISSION=4） | 代码里只用符号常量 `px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION`，不硬编码数字，因此不同 px4_msgs 版本都能编译且语义正确 |
| `/fmu/vehicle_command` service | 存在 | **存在**，类型 `px4_msgs/srv/VehicleCommand`，字段为 `request`(VehicleCommand) / `reply`(VehicleCommandAck) | 使用该 service（首选方案，未退回 `/fmu/in/vehicle_command`） |
| `VEHICLE_CMD_DO_PAUSE_CONTINUE`(193) | 用它暂停 Mission | 常量存在，但 **PX4 v1.16 飞行栈没有实现它**。`Commander::handle_command()` 里没有 193 的分支，会落到 `default:` → 回 `VEHICLE_CMD_RESULT_UNSUPPORTED (3)`，飞机继续飞 | 提供 `hold_method: "do_pause_continue"` 选项（并在启动时 WARN），但**默认不用** |
| PX4 原生“暂停任务并 Hold” | — | 是 `VEHICLE_CMD_DO_REPOSITION`(192)：`param2 = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE(1)`，`param5/6/7 = NaN` → commander 切到 `AUTO_LOITER`(Hold)，navigator 用 `preproject_stop_point()` 生成**带刹车的停点**。这正是 QGroundControl “Pause” 按钮发的命令 | `hold_method: "do_reposition_hold"`（**默认**） |
| 恢复任务 | — | `VEHICLE_CMD_SET_NAV_STATE`(100001)，`param1 = 3`(AUTO_MISSION)，PX4 从被打断的航点继续 | 自动恢复关闭；开启时用该命令 |

核对依据（PX4 v1.16.0 tag / px4_msgs release/1.16）：

- `px4_msgs`：`msg/VehicleStatus.msg`（`NAVIGATION_STATE_AUTO_MISSION = 3`、`AUTO_LOITER = 4`）、`msg/VehicleCommand.msg`（`VEHICLE_CMD_DO_REPOSITION = 192`、`VEHICLE_CMD_DO_PAUSE_CONTINUE = 193`、`VEHICLE_CMD_SET_NAV_STATE = 100001`；`param5/param6` 是 **float64**，`param1..4/param7` 是 float32）、`msg/VehicleCommandAck.msg`、`srv/VehicleCommand.srv`
- PX4：`src/modules/commander/Commander.cpp`（DO_REPOSITION 分支、SET_NAV_STATE 分支、`default:` → UNSUPPORTED）、`src/modules/navigator/navigator_main.cpp`（DO_REPOSITION 处理，含注释 “All three set to NaN - pause vehicle”）、`src/modules/navigator/mission_base.cpp`（恢复航点）、`src/modules/uxrce_dds_client/{vehicle_command_srv.cpp, srv_base.cpp, utilities.hpp}`（service 名 `/fmu/vehicle_command`、QoS）
- QGroundControl：`src/FirmwarePlugin/PX4/PX4FirmwarePlugin.cc` 的 `PX4FirmwarePlugin::pauseVehicle()` 发的就是 `MAV_CMD_DO_REPOSITION + MAV_DO_REPOSITION_FLAGS_CHANGE_MODE + NaN`

其他被核实的细节：

- PX4 **发布** `/fmu/out/*` 的 QoS 是 `BEST_EFFORT + KEEP_LAST(depth 0)`（`uxrce_dds_client/utilities.hpp`），所以本节点用 PX4 官方文档推荐的 sensor-data QoS 订阅；
- PX4 的 service replier 是 `RELIABLE + KEEP_LAST(1)`，ROS 2 service client 默认 QoS 兼容；
- `source_component` 必须 < `COMPONENT_MODE_EXECUTOR_START(1000)`，否则 PX4 当成 mode executor 处理。

---

## 1. 包结构

```text
mid360_obstacle_stop/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── obstacle_stop.yaml                 # 所有参数（无 magic number）
├── launch/
│   └── obstacle_stop.launch.py
├── include/mid360_obstacle_stop/
│   ├── obstacle_detector.hpp              # 点云 → ROI / 危险判定（不含 ROS 通信，可单测）
│   ├── px4_command_interface.hpp          # /fmu/vehicle_command service 客户端 + ACK 解析
│   └── obstacle_stop_node.hpp             # 参数、订阅、状态机、watchdog
└── src/
    ├── obstacle_detector.cpp
    └── obstacle_stop_node.cpp
```

分层是刻意的：`PointCloud preprocessing / obstacle detector / state machine / PX4 command interface`
分别在 detector / interface / node 三处，后续升级“沿速度方向的安全走廊 + 动态刹车距离”时只需要改 detector 与一处调用。

---

## 2. 数据流与状态机

```text
Livox MID360 → Livox ROS Driver 2 → sensor_msgs/PointCloud2 (/livox/lidar)
      ↓
ObstacleStopNode: ROI(360° 圆柱) → 帧确认 → 状态机 → Px4CommandInterface
      ↓
/fmu/vehicle_command (px4_msgs/srv/VehicleCommand)
      ↓  VEHICLE_CMD_DO_REPOSITION(192) + CHANGE_MODE + NaN lat/lon/alt
PX4 commander → nav_state = AUTO_LOITER(Hold) → 减速/停车/悬停由 PX4 完成
```

状态机（`StopState`）：

```text
CLEAR ── obstacle_frames >= confirm_frames, nav_state == AUTO_MISSION ──▶ PAUSE_PENDING
PAUSE_PENDING ── ACK ACCEPTED ──▶ PAUSED            (ACK 非 ACCEPTED / 超时 → 回 CLEAR，冷却后重试)
PAUSED ── 观察到 nav_state == AUTO_LOITER ──▶ 保持 Hold（第一阶段一直保持）
PAUSED ── auto_resume=true 且连续 clear_frames 帧安全 ──▶ RESUME_PENDING ── ACK ACCEPTED ──▶ CLEAR
```

额外的安全规则（在需求之外补的，但都是“不和人抢控制权”方向）：

1. **PAUSED 只在 ACK = ACCEPTED 时进入**；被拒绝或超时绝不进入 PAUSED，日志明确。
2. **`paused_by_obstacle_` 归属标记**：只有本节点触发的 pause 才会自动 Continue；
   人工在 QGC 手动 Pause（或任何外部切模式）时本节点根本不会进入 PAUSED。
3. **外部接管保护**：如果 Hold 期间 `nav_state` 被外部改变（人工恢复任务 / 切 POSCTL / RTL / failsafe），
   本节点立即放弃归属，并静默 `rearm_grace`（默认 5 s）：这段时间内不会自动 Pause，
   不会出现“用户刚恢复、ROS 立刻又把它停住”的来回拉锯；静默结束后重新武装，
   若障碍仍在危险圆柱内会再次 Pause。

   > 这里**故意用时间而不是“等危险区清空”**。旧版用 `suppress_until_clear`（要求整个圆柱
   > `resume_distance` 内无点），在走廊、树林、贴墙飞行或 `stop_distance` 设得较大时该条件
   > 可能**永远不成立**，结果是飞手手动恢复一次之后，剩余航程完全失去保护，且没有任何提示。
   > 时间窗是有界的，保护一定会回来。
4. **ACK 之后的二次确认**：ACK ACCEPTED 后 `hold_confirm_timeout` 内若 `nav_state` 一直是 AUTO_MISSION，
   输出 ERROR 提示 PX4 可能没有真正进入 Hold（判定仍以 ACK 为准，不会误报 PAUSED）。
5. **LiDAR watchdog**：超过 `lidar_timeout` 没有点云 → ERROR + 周期性 WARN；
   `lidar_timeout_stop_enabled=true` 时才会像障碍物一样触发 Pause（第一阶段默认 false）。
6. **Pause 已发出但障碍在 ACK 返回前消失**：仍会进入 PAUSED（保守做法，也符合“一个障碍事件只发一次命令”）。
   `auto_resume=false` 时需要人工继续任务；`auto_resume=true` 时会在安全持续 `clear_frames` 帧后自动继续。
7. **线程安全**：所有回调（点云、状态、定时器、service 响应）都在同一个 `MutuallyExclusive`
   callback group + 默认 SingleThreadedExecutor 下执行，状态机无需加锁；将来换 MultiThreadedExecutor 也不会并发进入状态机。

---

## 3. 参数（`config/obstacle_stop.yaml`）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `pointcloud_topic` | `/livox/lidar` | MID360 点云话题（**以实测为准**） |
| `vehicle_status_topic` | `/fmu/out/vehicle_status` | px4_msgs/msg/VehicleStatus |
| `vehicle_command_service` | `/fmu/vehicle_command` | px4_msgs/srv/VehicleCommand |
| `stop_distance` | `3.0` | 危险半径（水平距离，360°） |
| `resume_distance` | `4.0` | 恢复半径（迟滞，避免反复 pause/resume） |
| `min_range` | `0.5` | 小于该水平距离的点忽略（机体自身/起落架/桨叶） |
| `z_min` / `z_max` | `-0.6` / `0.8` | 高度 ROI（机体系） |
| `min_obstacle_points` | `10` | 单帧危险点数下限（滤噪） |
| `confirm_frames` | `3` | 连续多少帧才触发 Pause（中间一帧 clear 即清零） |
| `clear_frames` | `10` | 连续多少帧安全才算恢复 |
| `auto_resume` | `false` | 第一阶段默认关闭 |
| `lidar_timeout` | `0.5` | 点云超时时间 [s] |
| `lidar_timeout_stop_enabled` | `false` | 超时是否触发 Pause |
| `hold_method` | `do_reposition_hold` | `do_reposition_hold`（PX4 1.16 可用）/ `do_pause_continue`（PX4 1.16 不支持） |
| `command_timeout` | `2.0` | 等待 VehicleCommandAck 的超时 [s] |
| `command_retry_cooldown` | `2.0` | 失败后的重试冷却 [s] |
| `hold_confirm_timeout` | `2.0` | ACK 后确认 nav_state 进入 Hold 的时间 [s] |
| `rearm_grace` | `5.0` | 失去控制权（人工恢复 / 命令被拒）后静默多久再重新武装 [s]，`0` = 不静默 |
| `diagnostic_period` | `0.0` | > 0 时每隔该秒数打印一行检测摘要（地面调试用） |

除话题名和 `hold_method` 外，其余参数都支持运行时 `ros2 param set` 动态生效（非法值会被拒绝并给出原因）。

---

## 4. 编译 / source / 运行

```bash
# 1) 放进工作空间的 src/
cd ~/ros2_ws/src
cp -r /path/to/mid360_obstacle_stop .

# 2) 编译（px4_msgs 必须在同一个工作空间或已 source）
cd ~/ros2_ws
colcon build --symlink-install --packages-select mid360_obstacle_stop

# 3) source
source /opt/ros/humble/setup.bash      # 你的 ROS 2 发行版
source ~/ros2_ws/install/setup.bash

# 4) 运行（livox_ros_driver2 和 PX4 的 uXRCE-DDS 客户端请先起好）
ros2 launch mid360_obstacle_stop obstacle_stop.launch.py

# 或不用 launch，手动带参数运行
ros2 run mid360_obstacle_stop obstacle_stop_node \
  --ros-args --params-file ~/ros2_ws/src/mid360_obstacle_stop/config/obstacle_stop.yaml
```

---

## 5. 运行前检查（对应交付项 11–14）

```bash
# 11) 话题/服务清单
ros2 topic list | grep -E "livox|fmu"
ros2 service list | grep vehicle_command

# 12) 确认 /livox/lidar 是 sensor_msgs/msg/PointCloud2
ros2 topic info /livox/lidar --verbose        # Type: sensor_msgs/msg/PointCloud2，并显示 QoS
ros2 topic hz /livox/lidar                    # 应有稳定的点云频率
# 如果不是 PointCloud2（例如自定义 livox_ros_driver2/msg/CustomMsg），
# 请把 livox_ros_driver2 的 xfer_format 设为 0 输出 PointCloud2

# 13) 确认 /fmu/vehicle_command service 存在
ros2 service type /fmu/vehicle_command        # px4_msgs/srv/VehicleCommand
ros2 interface show px4_msgs/srv/VehicleCommand
# 期望输出（px4_msgs 1.16）:
#   px4_msgs/VehicleCommand request
#   ---
#   px4_msgs/VehicleCommandAck reply

# 14) 确认 PX4 当前 nav_state（注意 QoS：PX4 发布是 best_effort）
ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --field nav_state
#   也可以： --qos-profile sensor_data
#   数值含义（px4_msgs 1.16）：3 = AUTO_MISSION，4 = AUTO_LOITER(Hold)
ros2 interface show px4_msgs/msg/VehicleStatus | grep -E "NAVIGATION_STATE_(AUTO_MISSION|AUTO_LOITER) ="

# 顺便确认 px4_msgs 版本
ros2 pkg prefix px4_msgs
```

启动节点后应看到：

```text
[INFO] Obstacle stop node started | point cloud: /livox/lidar | vehicle_status: /fmu/out/vehicle_status | service: /fmu/vehicle_command
[INFO] ROI: 0.50 m < horizontal < 3.00 m, -0.60 m < z < 0.80 m, >= 10 danger points, 3 confirm frames, 10 clear frames
[INFO] hold_method: do_reposition_hold | auto_resume: false | lidar_timeout: 0.50 s (stop on timeout: false)
```

---

## 6. 测试步骤

> 先做地面测试，**不要一开始就装桨实飞**。

> **重要前提**：本节点只在该 PX4 处于 `AUTO_MISSION` 时才发暂停命令。
> 地面未 arm 时 `nav_state` 是 MANUAL/POSCTL，**永远不会触发 Pause**（设计如此，不是 bug）。
> 因此“真正发出暂停命令、看到 ACK”必须在 Test 2 里验证：SITL，或拆桨 arm 后上传任务进入 AUTO_MISSION。

### Test 0：接口核对 + 静态点云（拆桨）

1. 确认话题/服务名与 `config/obstacle_stop.yaml` 一致（不一致就改 yaml，不要改代码）：

```bash
ros2 topic list | grep -i livox      # 期望 /livox/lidar
ros2 topic list | grep fmu           # 期望 /fmu/out/vehicle_status
ros2 service list | grep vehicle     # 期望 /fmu/vehicle_command
ros2 topic hz /livox/lidar           # 期望 ~10 Hz
```

> PX4 的话题名可能带版本后缀（例如 `/fmu/out/vehicle_status_v1`），**以 `ros2 topic list` 的实际输出为准**。

2. 打开诊断日志：

```bash
ros2 param set /mid360_obstacle_stop diagnostic_period 2.0
```

3. 无遮挡时每 2 s 应看到 `danger=0`：

```text
[INFO] detect: roi=1832, danger=0, nearest=5.41 m | frames obstacle=0/3 clear=12/10 | nav_state=0 (MANUAL), state=CLEAR
```

   **`danger` 必须恒为 0。** 如果无遮挡时它一直非 0，说明机身上的东西（起落架、天线、GPS 杆、桨叶）
   落在 `min_range`~`stop_distance` 之间：调大 `min_range`，或调 `z_min/z_max` 把机身那段高度排掉。
   这一步不做，后面一定会频繁误停。

4. 把纸箱放进 `stop_distance` 以内并保持：`danger` 应立刻变成几千，`nearest` 变成纸箱距离；拿开后回到 0。
   这证明检测链路（点云解析 → ROI → 连续帧确认）是通的。

5. 关掉诊断日志：`ros2 param set /mid360_obstacle_stop diagnostic_period 0.0`

### Test 0b：无雷达台架测试（合成点云）

手边没有 MID360、或者不想等雷达装好，可以用合成点云跑通整条链路：

```bash
ros2 run mid360_obstacle_stop fake_obstacle_publisher.py
# 运行时"放/收"障碍（0.0 = 无障碍）
ros2 param set /fake_obstacle_publisher obstacle_distance 2.0
ros2 param set /fake_obstacle_publisher obstacle_distance 0.0
```

它替代 Test 1/2/3/4 里"把纸箱靠近 MID360"那一步：

| 能验证 | 不能验证 |
| --- | --- |
| 点云解析、ROI 判定、连续帧确认 | Livox 驱动的真实字段与坐标系 |
| 状态机 CLEAR → PAUSE_PENDING → PAUSED | 机体自身点云误报（起落架 / 天线 / 桨叶） |
| PX4 命令、ACK、nav_state 3 → 4 | 真实点云的帧率、延迟、丢帧 |
| 防重复命令、`rearm_grace`、人工接管保护 | 刹车距离（必须带桨台架或实机） |

> 要验证"真的发出暂停命令并进入 Hold"，**仍然需要 PX4**：真机飞控通电（拆桨）且任务处于
> `AUTO_MISSION`，或者 SITL。没有 PX4 时只会看到 `no PX4 VehicleStatus ... inhibited` 这类日志。

连飞控也不想接的话，还有一个**假 PX4 桩**，它模仿 PX4 v1.16 对命令的反应：

```bash
ros2 run mid360_obstacle_stop fake_px4_stub.py
# 模拟 PX4 拒绝命令（验证 rearm_grace 静默与重试）
ros2 run mid360_obstacle_stop fake_px4_stub.py --ros-args -p reject_pause:=true
# 模拟 ACK 超时（验证 command_timeout 后回到 CLEAR）
ros2 run mid360_obstacle_stop fake_px4_stub.py --ros-args -p ack_delay_s:=5.0
# 模拟 ACK 了但迟迟不进 Hold（验证 hold_confirm_timeout 的 ERROR）
ros2 run mid360_obstacle_stop fake_px4_stub.py --ros-args -p hold_delay_s:=10.0
```

配合上面的假点云，板卡上不需要雷达、不需要飞控就能跑完 Test 2 / 3 / 4 的逻辑部分：

```bash
ros2 run mid360_obstacle_stop fake_px4_stub.py        # 终端 1
ros2 launch mid360_obstacle_stop obstacle_stop.launch.py   # 终端 2
ros2 run mid360_obstacle_stop fake_obstacle_publisher.py   # 终端 3
ros2 param set /fake_obstacle_publisher obstacle_distance 2.0   # 终端 4
```

> 桩只能验证**本节点的逻辑**（状态机、ACK 处理、命令去重）。PX4 真实的 ACK 行为、
> 刹车距离、模式切换时序仍然必须在 SITL 和真机上验证。

### Test 1：纯点云测试（不接 PX4）

1. 启动 `livox_ros_driver2` 与本节点（PX4 可以不连）。
2. 无障碍时应无多余日志（点云正常时不打印）。
3. 用纸箱靠近 MID360 到 `stop_distance` 以内并保持几帧，观察：

```text
[WARN] Obstacle/timeout condition present but no PX4 VehicleStatus received on '/fmu/out/vehicle_status' yet - automatic Mission PAUSE inhibited (is PX4 running?)
```

   或（PX4 在但没有 VehicleStatus 时）服务不可用日志：

```text
[ERROR] Mission PAUSE requested (...) but the VehicleCommand service '/fmu/vehicle_command' is NOT available - no command sent (is PX4 connected?)
```

4. 节点不得崩溃、不得每帧刷屏（日志有 2 s throttle，命令有 2 s 冷却）。
5. 把纸箱移开，日志回到安静状态。

### Test 2：PX4 无桨测试（Mission → Hold）

1. **拆桨**。启动 PX4（SITL 或真机），起 uXRCE-DDS 客户端。
2. QGC 上传一个 Mission，让飞机进入 `AUTO_MISSION`（未 arm 也可以，本节点不要求 armed，方便台架测试）。
3. 确认 `ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --field nav_state` 输出 3。
4. 把纸箱放进安全圆柱内，期望日志：

```text
[WARN] Requesting Mission PAUSE (obstacle): 42 danger point(s), nearest 1.83 m, nav_state=AUTO_MISSION, armed=false, method=do_reposition_hold
[WARN] PX4 accepted the Mission PAUSE (vehicle_command=192, ack=ACCEPTED) - PX4 now owns the deceleration, the stop and the hold
[INFO] PX4 left AUTO_MISSION: nav_state 3 -> 4 (AUTO_LOITER(Hold))
[INFO] PX4 is holding: nav_state=AUTO_LOITER - Mission PAUSE complete (vehicle stopped by PX4)
```

5. QGC 上应显示 Hold/Loiter（真机无桨时没有实际位移，重点看模式与 ACK）。

### Test 3：防重复命令

1. 障碍物持续放在危险区内 10 s 以上。
2. 日志中 `Requesting Mission PAUSE` 与 `accepted the Mission PAUSE` **各只出现一次**
   （状态机从 CLEAR 进入 PAUSED 后不再发命令；`obstacle_frames` 到达 `confirm_frames` 后饱和）。

### Test 4：人工 Pause 保护

1. 无障碍时，人工在 QGC 手动 Pause（PX4 会切到 Hold）。
2. 本节点应**不进入 PAUSED**：只会看到 `PX4 left AUTO_MISSION: nav_state 3 -> 4 (AUTO_LOITER(Hold))`，
   不会有 `PX4 accepted the Mission PAUSE` 日志。
3. 把 `auto_resume` 设为 `true`（yaml 或 `ros2 param set /mid360_obstacle_stop auto_resume true`），
   即使障碍区是空的，也不会有任何 `requesting Mission CONTINUE`：
   因为 `paused_by_obstacle_ == false`，`requestMissionContinue()` 会直接拒绝。

### Test 5：低速低高度实飞

1. 开阔场地、Mission 速度很低、高度较低、`stop_distance` 适当放大（例如 5 m）、人手放在模式开关上。
2. 依次复现：无障碍正常飞 → 放入障碍 → Mission Pause → PX4 Hold → 人工决定是否继续任务。
3. 若打开 `auto_resume: true`，先确认 Hold 状态下把障碍移开 `resume_distance` 以外并持续 `clear_frames` 帧后，
   PX4 收到 `VEHICLE_CMD_SET_NAV_STATE` 回到 AUTO_MISSION 并从被打断的航点继续。

---

## 7. 实际改动 / 未能验证的部分

**本 package 全部是新增文件**（未修改任何既有工程代码，也没有改 PX4 源码）：

| 文件 | 作用 |
| --- | --- |
| `CMakeLists.txt` | ament 目标 `obstacle_stop_node`，依赖 `rclcpp / rcl_interfaces / sensor_msgs / px4_msgs`，安装 launch+config |
| `package.xml` | 上述依赖声明 |
| `include/.../obstacle_detector.hpp` + `src/obstacle_detector.cpp` | 360° 圆柱 ROI 判定，`PointCloud2ConstIterator<float>` 遍历，`isfinite` 过滤 |
| `include/.../px4_command_interface.hpp` | `/fmu/vehicle_command` service 客户端，pause/continue 两种 method，ACK 解析 |
| `include/.../obstacle_stop_node.hpp` + `src/obstacle_stop_node.cpp` | 参数、订阅、状态机、watchdog、日志、动态参数 |
| `config/obstacle_stop.yaml` | 全部参数 |
| `launch/obstacle_stop.launch.py` | 加载参数并启动节点 |
| `README.md` | 本文档 |

**尚未验证（需要你在实机上确认）：**

1. **没有编译过**：本机没有 ROS 2 环境（macOS），`colcon build` 尚未执行；请先编译，如有 include/字段名问题按报错修（字段名已按 px4_msgs release/1.16 核对）。
2. **没有访问你的工程**：无法检查你现有的 launch/topic/QoS 配置；`/livox/lidar` 与 `/fmu/*` 请在实机用第 5 节命令确认，不一致就改 yaml。
3. **未做真实飞行/台架验证**：ACK 行为、`preproject_stop_point` 的实际刹车距离都需要 Test 2/5 验证。
4. **px4_msgs 版本**：如果你的 px4_msgs 不是 release/1.16（例如更新版本把 `DO_PAUSE_CONTINUE` 实现了），把 `hold_method` 切到 `do_pause_continue` 即可，无需改代码。
