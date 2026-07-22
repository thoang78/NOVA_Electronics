#!/usr/bin/env python3
"""
Converts a geometry_msgs/Twist on /cmd_vel into:
  - /wheel_cmd  (std_msgs/Float32MultiArray, 4 duty cycles -1..1) -> goes to the Pico
  - /servo_cmd  (std_msgs/Float64MultiArray, 4 angles in radians) -> goes to dynamixel_node

*** STUB KINEMATICS ***
This implements simple "point turn or straight line" logic, NOT true swerve
kinematics. Real independent-steering kinematics needs your actual track
width and wheelbase, and per-wheel angle/speed solving so wheels don't fight
each other during a turn. Replace `compute_wheel_targets()` with real swerve
math once you've confirmed the transport/plumbing works end-to-end.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float64MultiArray

ORDER = ['front_left', 'front_right', 'back_left', 'back_right']

MAX_LINEAR_DUTY = 0.8   # duty cycle at "full speed" cmd_vel.linear.x
MAX_STEER_RAD = math.radians(45)  # servo angle at max angular.z, purely illustrative


class WheelKinematicsNode(Node):
    def __init__(self):
        super().__init__('wheel_kinematics_node')

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.wheel_cmd_pub = self.create_publisher(Float32MultiArray, 'wheel_cmd', 10)
        self.servo_cmd_pub = self.create_publisher(Float64MultiArray, 'servo_cmd', 10)

    def compute_wheel_targets(self, linear_x: float, angular_z: float):
        """Returns (wheel_duties[4], servo_angles[4]) in ORDER."""
        # Placeholder: all wheels same speed/angle (straight-line + point-turn only)
        duty = max(-1.0, min(1.0, linear_x)) * MAX_LINEAR_DUTY
        steer = max(-1.0, min(1.0, angular_z)) * MAX_STEER_RAD

        if abs(angular_z) > 0.05 and abs(linear_x) < 0.05:
            # pure rotation in place: point wheels to spin the chassis, not drive it
            duties = [duty if i in (0, 2) else -duty for i in range(4)]  # placeholder
            angles = [0.0] * 4
        else:
            duties = [duty] * 4
            angles = [steer] * 4

        return duties, angles

    def on_cmd_vel(self, msg: Twist):
        duties, angles = self.compute_wheel_targets(msg.linear.x, msg.angular.z)

        wheel_msg = Float32MultiArray()
        wheel_msg.data = [float(d) for d in duties]
        self.wheel_cmd_pub.publish(wheel_msg)

        servo_msg = Float64MultiArray()
        servo_msg.data = [float(a) for a in angles]
        self.servo_cmd_pub.publish(servo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
