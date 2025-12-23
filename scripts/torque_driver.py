#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TorqueDriver(Node):
    def __init__(self):
        super().__init__('torque_driver')

        # 1. Subscribe to cmd_vel (from keyboard or nav)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        # 2. Publish Torque (Effort) to the wheels
        # UPDATE THESE TOPICS to match exactly what worked in your terminal
        self.left_pub = self.create_publisher(Float64, '/model/my_bot/joint/base_Lwheel_joint/cmd_force', 10)
        self.right_pub = self.create_publisher(Float64, '/model/my_bot/joint/base_Rwheel_joint/cmd_force', 10)

        # Robot Parameters (Tune these!)
        self.wheel_separation = 0.36  # Meters between wheels
        self.wheel_radius = 0.05      # Meters
        self.mass_gain = 10.0        # Multiplier to convert "speed" to "force" (Simple P-Controller)

        self.get_logger().info('Torque Driver Started! Waiting for cmd_vel...')

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential Drive Kinematics (converting speed to wheel speed)
        left_speed_target = linear - (angular * self.wheel_separation / 2.0)
        right_speed_target = linear + (angular * self.wheel_separation / 2.0)

        # VERY SIMPLE LOGIC: Applying Force proportional to desired speed
        # In a real robot, you would use a PID controller here reading joint_states velocity.
        # For now, we just multiply by a gain to get it moving.
        
        left_force = Float64()
        left_force.data = left_speed_target * self.mass_gain

        right_force = Float64()
        right_force.data = right_speed_target * self.mass_gain

        # Publish the forces
        self.left_pub.publish(left_force)
        self.right_pub.publish(right_force)

        # self.get_logger().info(f'Pub Torque -> L: {left_force.data:.2f}, R: {right_force.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TorqueDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()