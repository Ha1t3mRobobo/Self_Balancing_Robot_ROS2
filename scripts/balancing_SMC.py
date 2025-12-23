#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class SelfBalancingSMC(Node):

    def __init__(self):
        super().__init__('self_balancing_smc')

        # ===== PARAMÈTRES SMC (SLIDING MODE CONTROL) =====
        # Surface de glissement s = lambda * erreur + derivee_erreur
        self.lambda_s = 5.0   # Contrôle la vitesse de convergence de l'erreur
        self.K_smc = 100.0    # Gain max (Amplitude de la commande)
        self.boundary = 0.1   # Épaisseur de la couche limite (pour éviter le "chattering")
        
        # ===== COMMANDES HAUT NIVEAU =====
        self.setpoint = 0.0        # angle cible (rad)
        self.yaw_cmd = 0.0         # différence de couple

        self.max_tilt = 0.08       # rad (~5°)
        self.k_vel = 0.35          # linear.x -> angle
        self.k_yaw = 20.0          # angular.z -> couple

        # ===== ÉTAT =====
        self.offset = 0.0

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

        self.get_logger().info("SMC Controller STARTED")

    # =========================================================
    # CMD_VEL → ANGLE CIBLE + YAW
    # =========================================================
    def cmd_vel_callback(self, msg: Twist):
        # Avancer / reculer → on penche le robot
        desired_angle = self.k_vel * msg.linear.x
        desired_angle = max(min(desired_angle, self.max_tilt), -self.max_tilt)
        self.setpoint = desired_angle

        # Tourner → différence de couple gauche/droite
        self.yaw_cmd = self.k_yaw * msg.angular.z

    # =========================================================
    # IMU → ALGORITHME SMC
    # =========================================================
    def imu_callback(self, msg: Imu):
        # 1. Calcul de l'angle actuel (Pitch)
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        theta = pitch - self.offset
        
        # 2. Vitesse angulaire (Pitch Rate)
        # On utilise directement le gyroscope de l'IMU (c'est plus propre que (error-last)/dt)
        theta_dot = msg.angular_velocity.y

        # Sécurité : Arrêt si le robot est tombé
        if abs(theta) > 1.3:
            self.publish_torque(0.0)
            return

        # ================= ALGORITHME SMC =================
        
        # Erreur de position (e)
        error = self.setpoint - theta
        
        # Erreur de vitesse (e_dot)
        # Si setpoint est constant, sa dérivée est 0, donc e_dot = 0 - theta_dot
        error_dot = 0.0 - theta_dot

        # Surface de glissement (Sliding Surface) : s
        # s représente la distance par rapport à la trajectoire idéale
        s = self.lambda_s * error + error_dot

        # Loi de commande u
        # u = K * tanh(s / boundary)
        # tanh remplace sign(s) pour éviter les vibrations violentes (chattering)
        
        # Note: Le signe dépend de votre robot. 
        # Si s > 0 (on est "en retard"), on doit accélérer.
        # Ici j'applique un signe négatif car généralement pour redresser un pendule
        # qui penche en avant (theta > 0, donc error < 0), il faut avancer les roues.
        
        u = self.K_smc * math.tanh(s / self.boundary)
        
        # Inversion du signe nécessaire selon le sens des moteurs dans Gazebo
        # Si le robot part dans le mauvais sens, enlevez ou ajoutez le signe "-" ci-dessous
        torque = -u 

        # Saturation finale pour protéger la simulation
        torque = max(min(torque, 200.0), -200.0)

        self.publish_torque(torque)
        
        # Debug optionnel
        # self.get_logger().info(f"Angle: {theta:.2f} | s: {s:.2f} | Torque: {torque:.2f}")

    # =========================================================
    # ENVOI AUX MOTEURS
    # =========================================================
    def publish_torque(self, torque):
        left = Float64()
        right = Float64()

        # On applique le couple calculé +/- la commande de rotation (yaw)
        left.data = float(torque - self.yaw_cmd)
        right.data = float(torque + self.yaw_cmd)

        self.l_pub.publish(left)
        self.r_pub.publish(right)


def main():
    rclpy.init()
    node = SelfBalancingSMC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()