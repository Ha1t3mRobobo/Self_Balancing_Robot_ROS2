#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


class SelfBalancingController(Node):

    def __init__(self):
        super().__init__('self_balancing_controller')

        # ===== PID ÉQUILIBRE =====
        self.Kp = -150.0
        self.Ki = -30.0
        self.Kd = -10.0

        # ===== COMMANDES HAUT NIVEAU =====
        self.setpoint = 0.0        # angle cible (rad)
        self.yaw_cmd = 0.0         # différence de couple

        self.max_tilt = 0.08       # rad (~5°)
        self.k_vel = 0.35          # linear.x -> angle
        self.k_yaw = 20.0          # angular.z -> couple

        # ===== ÉTAT PID =====
        self.offset = 0.0
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        # ===== QoS (Gazebo IMU) =====
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ===== SUBSCRIBERS =====
        self.create_subscription(Imu, '/imu', self.imu_callback, qos)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ===== PUBLISHERS =====
        self.l_pub = self.create_publisher(
            Float64,
            '/model/my_bot/joint/base_Lwheel_joint/cmd_force',
            10
        )
        self.r_pub = self.create_publisher(
            Float64,
            '/model/my_bot/joint/base_Rwheel_joint/cmd_force',
            10
        )

        self.get_logger().info("Self Balancing Controller STARTED")

    # =========================================================
    # CMD_VEL → ANGLE + YAW
    # =========================================================
    def cmd_vel_callback(self, msg: Twist):
        # Avancer / reculer → angle
        desired_angle = self.k_vel * msg.linear.x
        desired_angle = max(min(desired_angle, self.max_tilt), -self.max_tilt)
        self.setpoint = desired_angle

        # Tourner → différence de couple
        self.yaw_cmd = self.k_yaw * msg.angular.z

    # =========================================================
    # IMU → PID ÉQUILIBRE
    # =========================================================
    def imu_callback(self, msg: Imu):

        # Quaternion -> pitch
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        current_angle = pitch - self.offset

        # Sécurité : robot tombé
        if abs(current_angle) > 1.3:
            self.integral = 0.0
            self.publish_torque(0.0)
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        error = self.setpoint - current_angle
        self.integral += error * dt
        self.integral = max(min(self.integral, 10.0), -10.0)
        derivative = (error - self.last_error) / dt

        torque = (
            self.Kp * error +
            self.Ki * self.integral +
            self.Kd * derivative
        )

        torque = max(min(torque, 150.0), -150.0)

        self.publish_torque(torque)

        self.last_error = error
        self.last_time = now
        self.get_logger().info(f"Current angle: {current_angle:.3f}")

    # =========================================================
    # COUPLE → ROUES
    # =========================================================
    def publish_torque(self, torque):
        left = Float64()
        right = Float64()

        left.data = float(torque - self.yaw_cmd)
        right.data = float(torque + self.yaw_cmd)

        self.l_pub.publish(left)
        self.r_pub.publish(right)


def main():
    rclpy.init()
    node = SelfBalancingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
