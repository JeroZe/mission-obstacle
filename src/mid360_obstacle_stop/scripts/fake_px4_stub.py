#!/usr/bin/env python3
"""假 PX4 桩：不接飞控、不装 SITL 也能验证本节点的命令链路。

它模仿 PX4 v1.16 Commander 对被本节点使用的两条命令的处理：
    VEHICLE_CMD_DO_REPOSITION(192)     → 切 AUTO_LOITER(4)，回 ACCEPTED
    VEHICLE_CMD_SET_NAV_STATE(100001)  → 切 param1 指定的模式，回 ACCEPTED
    其它（含 DO_PAUSE_CONTINUE(193)）  → 回 UNSUPPORTED(3)

用法：
    ros2 run mid360_obstacle_stop fake_px4_stub.py

    # 想测"PX4 拒绝"的路径（应看到 rearm_grace 静默，然后重试）
    ros2 run mid360_obstacle_stop fake_px4_stub.py --ros-args -p reject_pause:=true

    # 想测 ACK 超时（节点应在 command_timeout 后回到 CLEAR）
    ros2 run mid360_obstacle_stop fake_px4_stub.py --ros-args -p ack_delay_s:=5.0

    # 想测"ACK 了但一直没进 Hold"
    ros2 run mid360_obstacle_stop fake_px4_stub.py --ros-args -p hold_delay_s:=10.0

它验证的是**本节点的逻辑**：状态机、ACK 处理、命令去重、rearm_grace、外部接管保护。
不能替代真机 / SITL 上对 PX4 真实 ACK 行为的验证 —— 那一步仍然必须做。
每次收到命令都会打印序号，可以直接用来确认"一个障碍事件只发一次命令"。
"""

import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv


def _const(msg_type, name, fallback):
    """优先用 px4_msgs 里的符号常量，换版本时也不至于直接崩。"""
    return getattr(msg_type, name, fallback)


NAV_AUTO_MISSION = _const(VehicleStatus, 'NAVIGATION_STATE_AUTO_MISSION', 3)
NAV_AUTO_LOITER = _const(VehicleStatus, 'NAVIGATION_STATE_AUTO_LOITER', 4)
ARMING_ARMED = _const(VehicleStatus, 'ARMING_STATE_ARMED', 2)

CMD_DO_REPOSITION = _const(VehicleCommand, 'VEHICLE_CMD_DO_REPOSITION', 192)
CMD_DO_PAUSE_CONTINUE = _const(VehicleCommand, 'VEHICLE_CMD_DO_PAUSE_CONTINUE', 193)
CMD_SET_NAV_STATE = _const(VehicleCommand, 'VEHICLE_CMD_SET_NAV_STATE', 100001)

RESULT_ACCEPTED = _const(VehicleCommandAck, 'VEHICLE_CMD_RESULT_ACCEPTED', 0)
RESULT_UNSUPPORTED = _const(VehicleCommandAck, 'VEHICLE_CMD_RESULT_UNSUPPORTED', 3)


class FakePx4Stub(Node):
    def __init__(self):
        super().__init__('fake_px4_stub')

        self.declare_parameter('vehicle_status_topic', '/fmu/out/vehicle_status')
        self.declare_parameter('vehicle_command_service', '/fmu/vehicle_command')
        self.declare_parameter('initial_nav_state', NAV_AUTO_MISSION)
        self.declare_parameter('status_rate_hz', 5.0)
        self.declare_parameter('reject_pause', False)
        self.declare_parameter('ack_delay_s', 0.0)
        self.declare_parameter('hold_delay_s', 0.0)

        self.nav_state = int(self.get_parameter('initial_nav_state').value)
        self.pending_nav_state = None
        self.pending_time = 0.0
        self.command_count = 0

        topic = self.get_parameter('vehicle_status_topic').value
        service = self.get_parameter('vehicle_command_service').value

        # PX4 发布 /fmu/out/* 用的是 BEST_EFFORT + KEEP_LAST，保持一致的 QoS。
        qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self.status_pub = self.create_publisher(VehicleStatus, topic, qos)
        self.command_srv = self.create_service(
            VehicleCommandSrv, service, self.handle_command)

        rate = max(1.0, float(self.get_parameter('status_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.publish_status)

        self.get_logger().warn(
            "假 PX4 桩已启动：'%s' (%.1f Hz) + 服务 '%s'，初始 nav_state=%d",
            topic, rate, service, self.nav_state)
        self.get_logger().warn(
            'reject_pause=%s ack_delay_s=%.1f hold_delay_s=%.1f | 这是测试桩，不是真飞控！',
            self.get_parameter('reject_pause').value,
            self.get_parameter('ack_delay_s').value,
            self.get_parameter('hold_delay_s').value)

    # ---------------------------------------------------------------- status

    def publish_status(self):
        if self.pending_nav_state is not None and time.monotonic() >= self.pending_time:
            self.set_nav_state(self.pending_nav_state)
            self.pending_nav_state = None

        msg = VehicleStatus()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.nav_state = self.nav_state
        msg.arming_state = ARMING_ARMED
        self.status_pub.publish(msg)

    def set_nav_state(self, state):
        if state == self.nav_state:
            return
        self.get_logger().info('nav_state %d -> %d', self.nav_state, state)
        self.nav_state = state

    # --------------------------------------------------------------- command

    def handle_command(self, request, response):
        command = request.request.command
        delay = max(0.0, float(self.get_parameter('ack_delay_s').value))
        if delay > 0.0:
            self.get_logger().warn('模拟 ACK 延迟：%.1f s 后才回复', delay)
            time.sleep(delay)

        self.command_count += 1
        result = RESULT_UNSUPPORTED

        if command == CMD_DO_REPOSITION:
            if self.get_parameter('reject_pause').value:
                self.get_logger().error(
                    '#%d DO_REPOSITION(192) → 按 reject_pause 回 UNSUPPORTED',
                    self.command_count)
            else:
                result = RESULT_ACCEPTED
                hold_delay = max(0.0, float(self.get_parameter('hold_delay_s').value))
                if hold_delay > 0.0:
                    self.pending_nav_state = NAV_AUTO_LOITER
                    self.pending_time = time.monotonic() + hold_delay
                    self.get_logger().warn(
                        '#%d DO_REPOSITION(192) → ACCEPTED，%.1f s 后才切 AUTO_LOITER',
                        self.command_count, hold_delay)
                else:
                    self.get_logger().warn(
                        '#%d DO_REPOSITION(192) → ACCEPTED，切 AUTO_LOITER(Hold)',
                        self.command_count)
                    self.set_nav_state(NAV_AUTO_LOITER)

        elif command == CMD_SET_NAV_STATE:
            target = int(round(request.request.param1))
            result = RESULT_ACCEPTED
            self.get_logger().warn(
                '#%d SET_NAV_STATE(100001) → ACCEPTED，切 nav_state=%d',
                self.command_count, target)
            self.set_nav_state(target)

        elif command == CMD_DO_PAUSE_CONTINUE:
            self.get_logger().error(
                '#%d DO_PAUSE_CONTINUE(193) → PX4 v1.16 未实现，回 UNSUPPORTED',
                self.command_count)

        else:
            self.get_logger().warn(
                '#%d 未知命令 %d → UNSUPPORTED', self.command_count, command)

        response.reply = VehicleCommandAck()
        response.reply.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        response.reply.command = command
        response.reply.result = result
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FakePx4Stub()
    # ACK 延迟是用 sleep 模拟的，必须多线程，否则状态发布会被卡住。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
