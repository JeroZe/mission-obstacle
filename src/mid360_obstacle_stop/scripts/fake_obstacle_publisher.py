#!/usr/bin/env python3
"""合成点云发布器：在没有 MID360 的情况下驱动 mid360_obstacle_stop。

用途：台架 / SITL 上验证 感知 → 状态机 → PX4 命令 → ACK → Hold 的整条链路，
不需要雷达、不需要真机起飞。

用法：
    # 背景（10 m 一圈点，danger=0）
    ros2 run mid360_obstacle_stop fake_obstacle_publisher.py

    # 启动时就放一个 2 m 处的障碍
    ros2 run mid360_obstacle_stop fake_obstacle_publisher.py --ros-args -p obstacle_distance:=2.0

    # 运行时切换（0.0 = 无障碍）
    ros2 param set /fake_obstacle_publisher obstacle_distance 2.0
    ros2 param set /fake_obstacle_publisher obstacle_distance 0.0

场景：
    * 背景：background_radius 处一圈点，z ∈ [-0.3, 0.3] → ROI 有点但 danger=0
    * 障碍：obstacle_distance 处一段 ±obstacle_arc_deg/2 的弧形点墙（正前方）→ danger>0
    z 范围都落在节点默认的 z ROI (-0.6, 0.8) 内，正前方意味着危险点全部落在
    stop_distance 圆柱里，因此能稳定触发 Pause。

注意：本工具只替代雷达。要验证"真的发出暂停命令并进 Hold"，
仍然需要 PX4（真机飞控通电，或 SITL），且飞机要处于 AUTO_MISSION。
"""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(points, frame_id, stamp):
    """把 (x, y, z) 列表打包成只有 x/y/z 三个 FLOAT32 字段的 PointCloud2。"""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    flat = []
    for x, y, z in points:
        flat.extend((x, y, z))
    msg.data = struct.pack('<%df' % len(flat), *flat)
    return msg


class FakeObstaclePublisher(Node):
    def __init__(self):
        super().__init__('fake_obstacle_publisher')

        self.declare_parameter('pointcloud_topic', '/livox/lidar')
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('background_radius', 10.0)
        self.declare_parameter('background_points', 720)
        self.declare_parameter('obstacle_distance', 0.0)
        self.declare_parameter('obstacle_points', 400)
        self.declare_parameter('obstacle_arc_deg', 30.0)

        topic = self.get_parameter('pointcloud_topic').value
        # 与节点侧的 SensorDataQoS（best effort）兼容。
        qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(PointCloud2, topic, qos)

        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.last_obstacle_distance = None  # 只在变化时打印，避免刷屏

        self.get_logger().info(
            "fake MID360 cloud on '%s' at %.1f Hz (background %.1f m), frame_id='%s'",
            topic, rate, self.get_parameter('background_radius').value,
            self.get_parameter('frame_id').value)
        self.get_logger().info(
            'obstacle: %s | 切换障碍距离: ros2 param set /fake_obstacle_publisher '
            'obstacle_distance <m>   (0.0 = 无障碍)',
            self._obstacle_description())

    def _obstacle_description(self):
        distance = float(self.get_parameter('obstacle_distance').value)
        if distance <= 0.0:
            return 'none (danger=0)'
        return '%.2f m ahead, %d points' % (
            distance, int(self.get_parameter('obstacle_points').value))

    def on_timer(self):
        background_radius = max(0.0, float(self.get_parameter('background_radius').value))
        background_points = max(0, int(self.get_parameter('background_points').value))
        distance = float(self.get_parameter('obstacle_distance').value)
        obstacle_points = max(0, int(self.get_parameter('obstacle_points').value))
        arc = math.radians(max(1.0, float(self.get_parameter('obstacle_arc_deg').value)))

        if distance != self.last_obstacle_distance:
            self.last_obstacle_distance = distance
            self.get_logger().info('obstacle changed: %s', self._obstacle_description())

        points = []

        # 背景圈：用确定性抖动而不是随机数，方便复现
        for i in range(background_points):
            angle = 2.0 * math.pi * i / background_points
            z = -0.3 + 0.6 * ((i % 7) / 6.0)
            points.append(
                (background_radius * math.cos(angle), background_radius * math.sin(angle), z))

        # 正前方的弧形点墙
        if distance > 0.0 and obstacle_points > 0:
            for i in range(obstacle_points):
                angle = -arc / 2.0 + arc * (i / max(1, obstacle_points - 1))
                z = -0.3 + 0.6 * ((i % 5) / 4.0)
                points.append((distance * math.cos(angle), distance * math.sin(angle), z))

        self.publisher.publish(
            make_cloud(points, self.get_parameter('frame_id').value,
                       self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = FakeObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
