#!/usr/bin/env python3
"""
Talks to the 4 daisy-chained Dynamixel steering servos over the U2D2.

Subscribes:  /servo_cmd  (std_msgs/Float64MultiArray) -- 4 target angles in radians,
             order: [front_left, front_right, back_left, back_right]
Publishes:   /servo_state (std_msgs/Float64MultiArray) -- 4 current angles in radians

Requires: pip install dynamixel-sdk   (or apt: ros-<distro>-dynamixel-sdk)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ---------------------------------------------------------------------
# Edit with actual servo IDs (using Dynamixel Wizard 2.0 tool during setup) 
# and model's control table (assuming X-series servo e.g. XL430 right now)
# ---------------------------------------------------------------------
DEVICE_NAME = '/dev/ttyU2D2'   # set by udev rule, see config/99-robot-usb.rules
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

SERVO_IDS = {'front_left': 1, 'front_right': 2, 'back_left': 3, 'back_right': 4}
ORDER = ['front_left', 'front_right', 'back_left', 'back_right']

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE = 1

# X-series: 0-4095 ticks = 0-360 degrees, center (0 rad) = 2048
TICKS_PER_REV = 4096


def rad_to_ticks(rad):
    ticks = int(2048 + (rad / (2 * 3.14159265)) * TICKS_PER_REV)
    return max(0, min(TICKS_PER_REV - 1, ticks))


def ticks_to_rad(ticks):
    return (ticks - 2048) / TICKS_PER_REV * (2 * 3.14159265)


class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')

        self.port = PortHandler(DEVICE_NAME)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        if not self.port.openPort():
            self.get_logger().error(f'Failed to open port {DEVICE_NAME}')
            raise RuntimeError('Could not open U2D2 serial port')
        if not self.port.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set baudrate')
            raise RuntimeError('Could not set U2D2 baudrate')

        for name, sid in SERVO_IDS.items():
            result, error = self.packet.write1ByteTxRx(
                self.port, sid, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if result != COMM_SUCCESS:
                self.get_logger().warn(f'Could not enable torque on servo "{name}" (id {sid})')
            else:
                self.get_logger().info(f'Servo "{name}" (id {sid}) torque enabled')

        self.cmd_sub = self.create_subscription(
            Float64MultiArray, 'servo_cmd', self.on_cmd, 10)
        self.state_pub = self.create_publisher(Float64MultiArray, 'servo_state', 10)

        self.timer = self.create_timer(0.05, self.publish_state)  # 20 Hz

    def on_cmd(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('servo_cmd needs 4 values, got %d' % len(msg.data))
            return
        for name, angle in zip(ORDER, msg.data):
            sid = SERVO_IDS[name]
            ticks = rad_to_ticks(angle)
            result, error = self.packet.write4ByteTxRx(
                self.port, sid, ADDR_GOAL_POSITION, ticks)
            if result != COMM_SUCCESS:
                self.get_logger().warn(f'Write failed for servo {name}: {self.packet.getTxRxResult(result)}')

    def publish_state(self):
        out = Float64MultiArray()
        out.data = []
        for name in ORDER:
            sid = SERVO_IDS[name]
            ticks, result, error = self.packet.read4ByteTxRx(
                self.port, sid, ADDR_PRESENT_POSITION)
            out.data.append(ticks_to_rad(ticks) if result == COMM_SUCCESS else 0.0)
        self.state_pub.publish(out)

    def destroy_node(self):
        self.port.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
