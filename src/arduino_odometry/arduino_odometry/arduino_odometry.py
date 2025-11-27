#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import serial
import math
import time

# ===================== CONFIG =====================
PORT = "/dev/ttyUSB1"
BAUD = 115200
WHEEL_DIAM = 0.1651          # 16.51 cm
WHEEL_BASE = 0.32            # 32 cm separación hoverboard
TICKS_PER_REV = 4093.0
UPDATE_HZ = 20.0
# ===================================================


class ArduinoOdometry(Node):

    def __init__(self):
        super().__init__('arduino_odometry')

        # Intentar abrir puerto
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.05)
            self.get_logger().info("🔥 Odometer: Conectado a Arduino.")
            self.sim_mode = False
        except:
            self.get_logger().warn("⚠️ Arduino NO encontrado. Activando modo SIMULADO.")
            self.ser = None
            self.sim_mode = True

        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.lastL = None
        self.lastR = None

        # Timer
        self.create_timer(1.0 / UPDATE_HZ, self.update_odom)

    # ================================================================
    #                      LOOP PRINCIPAL DE ODOMETRÍA
    # ================================================================
    def update_odom(self):

        # =========================
        #     MODO SIMULADO
        # =========================
        if self.sim_mode:
            now = self.get_clock().now().to_msg()

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            # Pose estática
            odom.pose.pose.orientation.w = 1.0

            self.odom_pub.publish(odom)

            # TF estático
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)

            return

        # =========================
        #     MODO REAL
        # =========================
        line = self.ser.readline().decode("utf-8").strip()

        # No llegó nada
        if not line:
            return

        if not line.startswith("E "):
            return

        try:
            _, L, R = line.split()
            L = int(L)
            R = int(R)
        except:
            return

        # Primer tick
        if self.lastL is None:
            self.lastL = L
            self.lastR = R
            return

        # Deltas
        dL = L - self.lastL
        dR = R - self.lastR

        self.lastL = L
        self.lastR = R

        # Distancias individuales
        dl = math.pi * WHEEL_DIAM * (dL / TICKS_PER_REV)
        dr = math.pi * WHEEL_DIAM * (dR / TICKS_PER_REV)

        # Distancia lineal
        dc = (dl + dr) / 2.0

        # Cambio angular
        dth = (dr - dl) / WHEEL_BASE

        # Integración
        self.th += dth
        self.x += dc * math.cos(self.th)
        self.y += dc * math.sin(self.th)

        now = self.get_clock().now().to_msg()

        # ==========================================================
        #                         ODOM MESSAGE
        # ==========================================================
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = dc * UPDATE_HZ
        odom.twist.twist.angular.z = dth * UPDATE_HZ

        self.odom_pub.publish(odom)

        # ==========================================================
        #                     TF odom → base_link
        # ==========================================================
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
