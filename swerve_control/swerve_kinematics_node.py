#!/usr/bin/env python3
# subscribes to /cmd_vel publishes /target_angles and /wheel_speeds

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from .swerve_kinematics import (
    WHEEL_ORDER,
    compute_wheel_commands,
    optimize_wheel,
)

CMD_VEL_TIMEOUT_SEC = 0.3
PUBLISH_RATE_HZ = 20.0
MAX_WHEEL_SPEED = 1.0

class SwerveKinematicsNode(Node):
    def __init__(self):
        super().__init__('swerve_kinematics_node')

        self.declare_parameter('max_wheel_speed', MAX_WHEEL_SPEED)
        self.declare_parameter('cmd_vel_timeout', CMD_VEL_TIMEOUT_SEC)

        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.last_angles = {name: 0.0 for name in WHEEL_ORDER}

        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.pub_angles = self.create_publisher(
            Float32MultiArray, 'target_angles', 10)
        self.pub_wheels = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, self.publish_wheel_commands)

        self.get_logger().info(
            'swerve_kinematics_node started, publishing at '
            f'{PUBLISH_RATE_HZ} Hz (cmd_vel timeout {self.cmd_vel_timeout}s)')

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def publish_wheel_commands(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_vel_timeout:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self.vx, self.vy, self.wz

        raw_commands = compute_wheel_commands(vx, vy, wz)

        angles_deg = []
        speeds_norm = []
        for name, (raw_angle, raw_speed) in zip(WHEEL_ORDER, raw_commands):
            angle, speed = optimize_wheel(
                raw_angle, raw_speed, self.last_angles[name])
            self.last_angles[name] = angle

            angles_deg.append(angle)
            normalized = speed / self.max_wheel_speed
            speeds_norm.append(max(-1.0, min(1.0, normalized)))

        # angles_deg[1] = -angles_deg[1]

        for i, name in enumerate(WHEEL_ORDER):
            if name in ('LF', 'LR'):
                speeds_norm[i] = -speeds_norm[i]

        angles_msg = Float32MultiArray()
        angles_msg.data = [float(a) for a in angles_deg]
        self.pub_angles.publish(angles_msg)

        wheels_msg = Float32MultiArray()
        wheels_msg.data = [float(s) for s in speeds_norm] + [0.0]
        self.pub_wheels.publish(wheels_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()