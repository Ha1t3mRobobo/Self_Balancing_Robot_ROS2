#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class ADRCPerformanceLogger(Node):

    def __init__(self):
        super().__init__('adrc_performance_logger')

        # ==========================================================
        # 1. CORRECTION DES TOPICS (Pour correspondre à ton ADRC)
        # ==========================================================
        # On écoute l'angle brut du capteur et la consigne de l'ADRC
        self.create_subscription(Float64, '/adrc/raw_angle_sensor', self.angle_callback, 10)
        self.create_subscription(Float64, '/adrc/setpoint', self.setpoint_callback, 10)

        # --- PARAMÈTRES ---
        self.trigger_threshold = 2.0  # Seuil de départ (deg)
        self.settling_band = 1.0      # Seuil de stabilité (deg)
        self.stable_duration = 2.0    # Secondes de stabilité requises

        # --- VARIABLES ---
        self.state = "WAITING"
        self.start_time = 0.0
        self.last_msg_time = 0.0
        self.ise, self.iae, self.itae, self.max_peak = 0.0, 0.0, 0.0, 0.0
        self.last_time_out_of_band = 0.0
        
        self.current_angle = 0.0
        self.current_setpoint = 0.0
        self.msg_received = False

        self.create_timer(0.1, self.display_monitor)

    def setpoint_callback(self, msg):
        self.current_setpoint = msg.data

    def angle_callback(self, msg):
        self.msg_received = True
        now = self.get_clock().now().nanoseconds / 1e9
        
        if self.last_msg_time == 0:
            self.last_msg_time = now
            return
        dt = now - self.last_msg_time
        self.last_msg_time = now

        self.current_angle = msg.data
        error_deg = self.current_setpoint - self.current_angle
        abs_err = abs(error_deg)
        error_rad = math.radians(error_deg)

        if self.state == "WAITING":
            if abs_err > self.trigger_threshold:
                self.state = "RECORDING"
                self.start_time = now
                self.last_time_out_of_band = now
                self.ise, self.iae, self.itae, self.max_peak = 0.0, 0.0, 0.0, 0.0

        elif self.state == "RECORDING":
            t = now - self.start_time
            self.ise += (error_rad ** 2) * dt
            self.iae += abs(error_rad) * dt
            self.itae += t * abs(error_rad) * dt
            if abs_err > self.max_peak: self.max_peak = abs_err
            
            if abs_err > self.settling_band:
                self.last_time_out_of_band = now
            
            if (now - self.last_time_out_of_band) > self.stable_duration:
                self.settling_time = self.last_time_out_of_band - self.start_time
                self.state = "FINISHED"

    def display_monitor(self):
        if not self.msg_received:
            print("\r⚠️ ATTENTE DE CONNEXION... Vérifiez que le robot tourne", end="")
            return

        abs_err = abs(self.current_setpoint - self.current_angle)
        if self.state == "WAITING":
            print(f"\r[PRET] Erreur: {abs_err:5.2f}rad | Pousse > {self.trigger_threshold}°", end="")
        elif self.state == "RECORDING":
            t_stable = (self.get_clock().now().nanoseconds / 1e9) - self.last_time_out_of_band
            print(f"\r[CALCUL...] Erreur: {abs_err:5.2f}° | Stabilité: {t_stable:3.1f}s / {self.stable_duration}s", end="")
        elif self.state == "FINISHED":
            self.print_final_report()
            self.state = "WAITING"

    def print_final_report(self):
        print("\n\n" + "\033[92m" + "*"*50)
        print("          RÉSULTATS ADRC (EN RADIANS)")
        print("*"*50 + "\033[0m")
        print(f" > Angle Max Peak (Dépassement) : {self.max_peak:.2f} deg")
        print(f" > Temps de réponse (Settling)  : {self.settling_time:.3f} sec")
        print("-" * 50)
        print(f" > ISE  : {self.ise:.6f}")
        print(f" > IAE  : {self.iae:.6f}")
        print(f" > ITAE : {self.itae:.6f}")
        print("*"*50 + "\n")

def main():
    rclpy.init()
    node = ADRCPerformanceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()