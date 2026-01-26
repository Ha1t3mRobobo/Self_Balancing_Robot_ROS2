#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class SelfBalancingADRC(Node):

    def __init__(self):
        super().__init__('self_balancing_adrc')

        # ==========================================
        # 1. PARAMÈTRES ADRC (TUNING)
        # ==========================================
        # b0 : Estimation du gain du système (Force -> Accélération)
        # IMPORTANT : Comme votre PID avait des gains NÉGATIFS, b0 doit être NÉGATIF
        # Relation approx : Kp_pid ~= wc^2 / b0. Avec Kp=-50 et wc=10 => b0 ~= -2.0
        self.b0 = -1.0 

        # wc : Bande passante du contrôleur (Réactivité)
        # Similaire à la "force" du ressort (Kp). Augmenter rend le robot plus "raide".
        self.wc = 3.0  

        # wo : Bande passante de l'observateur (Qualité de l'estimation)
        # Généralement wo = 3*wc à 5*wc.
        self.wo = 20.0  

        # Calcul automatique des gains de l'observateur (Luenberger)
        self.beta1 = 3.0 * self.wo
        self.beta2 = 3.0 * (self.wo * self.wo)
        self.beta3 = (self.wo * self.wo * self.wo)

        # Calcul automatique des gains du contrôleur PD
        self.kp_adrc =  self.wc * self.wc
        self.kd_adrc = 2.0 * self.wc

        # ==========================================
        # 2. VARIABLES D'ÉTAT ESO (Extended State Observer)
        # ==========================================
        self.z1 = 0.0 # Estimation Position (Angle)
        self.z2 = 0.0 # Estimation Vitesse (Vitesse angulaire)
        self.z3 = 0.0 # Estimation Perturbation totale (Disturbance)
        
        self.u_last = 0.0 # Dernière commande envoyée
        

        # ==========================================
        # 3. MOVEMENT & BRAKING (Conservé de votre code)
        # ==========================================
        self.k_vel = 0.35          
        self.k_yaw = 20.0          
        self.max_tilt = 0.01       
        self.k_brake = 0.25        
        self.min_speed_to_brake = 0.2 

        self.setpoint = 0.0
        self.yaw_cmd = 0.0
        self.current_speed = 0.0   
        self.is_commanding = False 

        self.offset = 0.0
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
        self.pub_dbg_z1 = self.create_publisher(Float64, '/adrc/z1_angle_est', 10)
        self.pub_dbg_z2 = self.create_publisher(Float64, '/adrc/z2_speed_est', 10)
        self.pub_dbg_z3 = self.create_publisher(Float64, '/adrc/z3_disturbance', 10)
        self.pub_dbg_u0 = self.create_publisher(Float64, '/adrc/u0_control', 10)
        self.pub_dbg_raw = self.create_publisher(Float64, '/adrc/raw_angle_sensor', 10)
        self.pub_dbg_setpoint = self.create_publisher(Float64, '/adrc/setpoint', 10)


        self.get_logger().info("ADRC Controller STARTED with b0=" + str(self.b0))

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
            self.setpoint = max(min(target, 34.9*self.max_tilt), -34.9*self.max_tilt)
        else:
            self.is_commanding = False

    def imu_callback(self, msg: Imu):
        # 1. Pitch Calculation (Measurement y)
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        y = pitch - self.offset # y est la mesure (angle réel)

        # 2. Safety Cutoff
        if abs(y) > 1.3:
            self.publish_torque(0.0)
            self.reset_eso()
            return

        # 3. Setpoint Logic (Braking / Movement)
        if not self.is_commanding:
            if abs(self.current_speed) > self.min_speed_to_brake:
                brake_angle = -1.0 * (self.current_speed * self.k_brake)
                self.setpoint = max(min(brake_angle, self.max_tilt), -self.max_tilt)
            else:
                self.setpoint = 0.0

        # 4. ADRC Calculation
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = now

        # --- A. Extended State Observer (ESO) ---
        # L'ESO estime l'état interne (z1, z2) et la perturbation (z3)
        # e_eso = estimation - mesure
        e_eso = self.z1 - y 
        
        # Mise à jour des états (Intégration d'Euler)
        # z1_dot = z2 - beta1 * e
        # z2_dot = z3 - beta2 * e + b0 * u
        # z3_dot = -beta3 * e
        
        self.z1 += (self.z2 - self.beta1 * e_eso) * dt
        self.z2 += (self.z3 - self.beta2 * e_eso + self.b0 * self.u_last) * dt
        self.z3 += (- self.beta3 * e_eso) * dt

        # Note: On peut utiliser le Gyroscope réel (msg.angular_velocity.y) pour aider z2
        # Pour un robot balancier, le gyro est souvent plus propre que l'estimation dérivée.
        # Décommentez la ligne ci-dessous pour injecter le gyro réel :
        # self.z2 = -msg.angular_velocity.y  # (Attention au signe du gyro selon votre IMU)

        # --- B. Control Law (Loi de commande) ---
        # u0 est la commande idéale PD (sans perturbation)
        # u = (u0 - perturbation_estimée) / b0
        
        error = self.setpoint - self.z1
        
        # Calcul du PD "virtuel"
        u0 = (self.kp_adrc * error) - (self.kd_adrc * self.z2)
        
        # Rejet de perturbation : on soustrait z3 pour l'annuler
        u = (u0 - self.z3) / self.b0

        # Saturation
        u = max(min(u, 200.0), -200.0)
        
        self.u_last = u # Stocker pour la prochaine itération de l'ESO
        self.publish_torque(u)

        # ==========================================
        # DEBUG PUBLISH
        # ==========================================
        z1_deg = math.degrees(self.z1)
        setpoint_deg = math.degrees(self.setpoint) # Utile pour comparer

        # On publie les degrés sur le topic existant
        self.pub_float(self.pub_dbg_z1, z1_deg)
        self.pub_float(self.pub_dbg_setpoint, setpoint_deg)
        
        # Si vous voulez aussi voir la vitesse en deg/s :
        z2_deg_s = math.degrees(self.z2)
        self.pub_float(self.pub_dbg_z2, z2_deg_s)

        # La perturbation z3 est une accélération/force abstraite, 
        # on la laisse souvent telle quelle, ou on la multiplie pour l'échelle.
        self.pub_float(self.pub_dbg_z3, self.z3)
        self.pub_float(self.pub_dbg_u0, u0)
        self.pub_float(self.pub_dbg_raw, math.degrees(y))

        

        
    def reset_eso(self):
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self.u_last = 0.0

    def pub_float(self, publisher, value):
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    def publish_torque(self, torque):
        left = Float64()
        right = Float64()
        # Application de la commande aux roues
        left.data = float(torque - self.yaw_cmd)
        right.data = float(torque + self.yaw_cmd)
        self.l_pub.publish(left)
        self.r_pub.publish(right)

def main():
    rclpy.init()
    node = SelfBalancingADRC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()