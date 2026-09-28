# mission-obstacle

Livox MID360 检测到障碍物 → 请求 PX4 自己把 Mission 暂停并 Hold 的 ROS 2 包。

**职责划分**：ROS 2 只负责感知 / 决策 / 请求 Mission Pause；减速、停车、悬停全部由 PX4 完成。
不使用 Offboard，不发送 setpoint，不发速度 0 —— 因此这条链路的可靠性和 QGroundControl
上的 Pause 按钮完全一致。

## 工作原理

```text
MID360 → livox_ros_driver2 → /livox/lidar (PointCloud2)
   ↓  360° 水平安全圆柱 ROI + 高度 ROI + 连续帧确认
ObstacleStopNode  状态机：CLEAR → PAUSE_PENDING → PAUSED → (RESUME_PENDING)
   ↓  /fmu/vehicle_command (px4_msgs/srv/VehicleCommand)
PX4  VEHICLE_CMD_DO_REPOSITION(192) + NaN  →  AUTO_LOITER (Hold)
```

只有当飞机处于 `AUTO_MISSION` 时才会发暂停请求；ACK 不是 `ACCEPTED` 就绝不进入 PAUSED。
自动恢复默认关闭，检测到障碍后保持 Hold，由人决定是否继续任务。

## 目录结构

```text
mission-obstacle_ws/                ← 仓库根 = colcon workspace 根（目录名随意，示例用 xxx_ws）
├── README.md
├── .gitignore                     # build/ install/ log/ bags/ *.bag src/px4_msgs/
├── scripts/push.sh                # 一次推送到 GitHub + Gitee（任一失败不影响另一个）
└── src/                           # ★ 本仓库只跟踪这里的内容
    ├── px4_msgs/                  # 外部依赖，自己 clone，不进本仓库
    └── mid360_obstacle_stop/      ← ROS 2 package，详细文档见包内 README
        ├── config/obstacle_stop.yaml  # 全部参数（无 magic number）
        ├── launch/                    # obstacle_stop.launch.py
        ├── include/ src/              # detector / PX4 接口 / 状态机
        └── README.md                  # ★ 接口核对依据、状态机、完整测试步骤
```

`build/`、`install/`、`log/`、`bags/`（录制的 rosbag）和 `src/px4_msgs/` 都由 `.gitignore` 排除，
仓库里只有 `src/` 下的源码。

## 远端与镜像

| 远端 | 地址 | 用途 |
| --- | --- | --- |
| `origin` | `https://github.com/JeroZe/mission-obstacle.git` | 主仓库（已启用） |
| `gitee` | `https://gitee.com/JeroZhang/mission-obstacle.git` | 国内镜像（已启用） |

`scripts/push.sh` 会推送到**所有已配置的远端**，任一远端失败不影响其它远端；
没有配置 `gitee` 时会自动跳过。启用镜像远端：

```bash
git remote add gitee https://gitee.com/JeroZhang/mission-obstacle.git
bash scripts/push.sh "chore: 启用 Gitee 镜像"
```

机载端从 Gitee 拉（国内速度更快）：

```bash
git clone https://gitee.com/JeroZhang/mission-obstacle.git ~/mission-obstacle
```

> Gitee 的 https 推送若开了两步验证，密码栏要填**私人令牌**而不是登录密码。
> 若 Gitee 仓库首页显示的默认分支仍是 `master`，去「仓库设置 → 默认分支」改成 `main`。

`px4_msgs` 不在本仓库里，需要单独获取。它必须与飞控固件版本对应（v1.16 → `release/1.16`），
**第三方镜像不保证同步到正确的分支**，所以优先用官方源；GitHub 拉不动时，可以用 Gitee 的
「导入仓库」把 `PX4/px4_msgs` 导到自己账号下再 clone，或者直接从已经拉好的机器上拷 `src/px4_msgs` 整个目录。

## 依赖

| 组件 | 要求 | 说明 |
| --- | --- | --- |
| ROS 2 | Humble | 本仓库在 Humble 上编译通过 |
| `px4_msgs` | `release/1.16` 分支，**必须源码编译** | 从未发布到 ROS apt 源，`ros-humble-px4-msgs` 不存在 |
| PX4 | v1.16（v1.15 需重新核对） | v1.14 及更早没有 `/fmu/vehicle_command` service，需要改代码 |
| 雷达 | Livox MID360 + `livox_ros_driver2` | 输出 `sensor_msgs/PointCloud2` |

## 编译

```bash
git clone https://github.com/JeroZe/mission-obstacle.git ~/mission-obstacle_ws
cd ~/mission-obstacle_ws/src
git clone -b release/1.16 https://github.com/PX4/px4_msgs.git px4_msgs

cd ~/mission-obstacle_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mid360_obstacle_stop
source install/setup.bash
```

> `px4_msgs` 是接口包（约 270 个 msg + 35 个 srv），首次编译在低配机器上可能要 **30 分钟**，
> 属正常现象。之后改自己的代码只用 `--packages-select`，几秒钟完成，**不要**
> `rm -rf build install`，否则 `px4_msgs` 要重来一遍。

## 运行

```bash
# 1) PX4（真机通电后 uXRCE-DDS 随固件启动；SITL 另需 MicroXRCEAgent udp4 -p 8888）
# 2) 雷达
ros2 launch livox_ros_driver2 msg_MID360_launch.py
# 3) 本节点
ros2 launch mid360_obstacle_stop obstacle_stop.launch.py
```

启动后务必核对接口名（`config/obstacle_stop.yaml` 里的默认值不一定和你的环境一致）：

```bash
ros2 topic type /livox/lidar                    # sensor_msgs/msg/PointCloud2
ros2 topic list | grep fmu                      # 确认 vehicle_status 的真实话题名
ros2 service list | grep vehicle_command        # 必须存在
```

打开诊断日志，确认**无遮挡时 `danger=0`**（否则起落架/天线/桨叶会被当成障碍，实飞必误停）：

```bash
ros2 param set /mid360_obstacle_stop diagnostic_period 2.0
# [INFO] detect: roi=1832, danger=0, nearest=5.41 m | frames obstacle=0/3 clear=12/10 | ...
```

## 已知限制（第一阶段）

- **只支持 `AUTO_MISSION`**：POSCTL / ALTCTL / Offboard / RTL / LAND 模式下不会暂停。
  地面未 arm 时也不会触发，这是设计如此。
- **固定 360° 圆柱，不做避障**：窄小环境（树、墙）会频繁误停，且不区分运动方向。
- **不感知速度**：`stop_distance` 必须大于「刹车距离 + 检测/通信延迟」，否则刹不住。
  按 3 帧确认 + 服务往返 ≈0.3~0.5 s 估算，默认 3 m 只适合 2 m/s 左右的低速任务。
- **点云坐标系未做 TF 变换**：直接按点云的 z 当机体高度，雷达安装有俯仰/滚转时需要重新标定 ROI。
- **请求式停车，不是硬安全链**：PX4 若拒绝该命令，飞机继续飞，本节点只报错。

## 测试

**先地面、后 SITL、最后才是实飞。** 完整的分步测试（Test 0 ~ Test 5，含期望日志）
见 [`src/mid360_obstacle_stop/README.md`](src/mid360_obstacle_stop/README.md)。

> 本包已在 ROS 2 Humble 上编译通过，但**尚未完成 SITL 与实机飞行验证**，
> ACK 行为与刹车距离需要在台架上确认后再实飞。
