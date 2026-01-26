#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class SelfBalancingController(Node):

    def __init__(self):
        super().__init__('self_balancing_controller')

        # ==========================================
        # 1. TUNING
        # ==========================================
        self.Kp = -50.0   
        self.Ki = -170.0     
        self.Kd = -2.0    

        # ==========================================
        # 2. MOVEMENT
        # ==========================================
        self.k_vel = 0.35          
        self.k_yaw = 10.0          
        self.max_tilt = 0.01       

        # ==========================================
        # 3. BRAKING
        # ==========================================
        self.k_brake = 4.0        
        self.min_speed_to_brake = 0.01 

        # VARIABLES
        self.setpoint = 0.0
        self.yaw_cmd = 0.0
        self.current_speed = 0.0   
        self.is_commanding = False 

        self.offset = 0.0
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subs
        self.create_subscription(Imu, '/imu', self.imu_callback, qos)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Pubs
        self.l_pub = self.create_publisher(Float64, '/model/my_bot/joint/base_Lwheel_joint/cmd_force', 10)
        self.r_pub = self.create_publisher(Float64, '/model/my_bot/joint/base_Rwheel_joint/cmd_force', 10)

        # Debug Pubs
        self.pub_dbg_angle = self.create_publisher(Float64, '/debug/angle', 10)
        self.pub_dbg_setpoint = self.create_publisher(Float64, '/debug/setpoint', 10)
        self.pub_dbg_torque = self.create_publisher(Float64, '/debug/torque', 10)
        self.pub_dbg_error = self.create_publisher(Float64, '/debug/error', 10)
        self.pub_dbg_speed = self.create_publisher(Float64, '/debug/velocity', 10)

        self.get_logger().info("Debug Controller Fixed STARTED")

    def joint_state_callback(self, msg: JointState):
        vels = []
        for i, name in enumerate(msg.name):
            if "wheel" in name.lower() and i < len(msg.velocity):
                vels.append(msg.velocity[i])
        if len(vels) >= 2:
            raw_speed = (vels[0] + vels[1]) / 2.0
            self.current_speed = 0.7 * self.current_speed + 0.3 * raw_speed

    def cmd_vel_callback(self, msg: Twist):
        self.yaw_cmd = self.k_yaw * msg.angular.z
        if msg.linear.x != 0.0:
            self.is_commanding = True
            target = self.k_vel * msg.linear.x
            self.setpoint = max(min(target, 263.0*self.max_tilt), -263.0*self.max_tilt)
        else:
            self.is_commanding = False

    def imu_callback(self, msg: Imu):
        # 1. Pitch
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        current_angle = pitch - self.offset

        # 2. Safety
        if abs(current_angle) > 1.3:
            self.publish_torque(0.0)
            self.integral = 0.0
            return

        # 3. Setpoint logic
        if not self.is_commanding:
            if abs(self.current_speed) > self.min_speed_to_brake:
                brake_angle = -1.0 * (self.current_speed * self.k_brake)
                self.setpoint = max(min(brake_angle, self.max_tilt), -self.max_tilt)
            else:
                self.setpoint = 0.0

        # 4. PID Calculation (RESTE EN RADIANS)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        error = self.setpoint - current_angle
        
        self.integral += error * dt
        self.integral = max(min(self.integral, 20.0), -20.0)

        # Gyro Derivative
        derivative = -msg.angular_velocity.y

        torque = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        torque = max(min(torque, 100.0), -100.0)

        self.publish_torque(torque)

        # ==========================================
        # DEBUG PUBLISH 
        # ==========================================
        msg_float = Float64() 

        # 1. Error (Converti en degrés pour cohérence visuelle)
        msg_float.data = math.degrees(error)
        self.pub_dbg_error.publish(msg_float)

        # 2. Angle (Converti en degrés)
        msg_float.data = math.degrees(current_angle)
        self.pub_dbg_angle.publish(msg_float)

        # 3. Setpoint (Converti en degrés)
        msg_float.data = math.degrees(self.setpoint)
        self.pub_dbg_setpoint.publish(msg_float)

        # 4. Torque (/100) - Pas d'angle ici, on laisse tel quel
        msg_float.data = float(-1.0*torque/10)
        self.pub_dbg_torque.publish(msg_float)
        
        # 5. Speed (/10) - Pas d'angle ici, on laisse tel quel
        msg_float.data = math.degrees(-1.0 * self.current_speed/10)
        self.pub_dbg_speed.publish(msg_float)
        # ==========================================

        self.last_error = error
        self.last_time = now

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