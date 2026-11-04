#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import numpy as np

def main():
    rclpy.init()
    node = Node("chequeo_sensores_en_vivo")

    state = {"scan": None, "odom": None, "batt": None}

    def on_scan(msg):
        r = np.array(msg.ranges)
        valid = np.isfinite(r) & (r > 0.05) & (r < 12.0)
        state["scan"] = {
            "puntos_totales": len(r),
            "puntos_validos": int(valid.sum()),
            "dist_min_m": round(float(np.min(r[valid])), 3) if valid.any() else -1.0,
            "dist_max_m": round(float(np.max(r[valid])), 3) if valid.any() else -1.0,
            "libre_frente_m": round(float(r[0]), 3) if len(r) > 0 and np.isfinite(r[0]) else -1.0
        }

    def on_odom(msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        state["odom"] = {
            "x": round(pos.x, 3),
            "y": round(pos.y, 3),
            "z": round(pos.z, 3),
            "quat_z": round(ori.z, 3),
            "quat_w": round(ori.w, 3)
        }

    def on_batt(msg):
        state["batt"] = {
            "porcentaje": round(msg.percentage * 100, 1),
            "voltaje": round(msg.voltage, 2)
        }

    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)
    node.create_subscription(BatteryState, "/battery_state", on_batt, qos_profile_sensor_data)

    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(state.values()):
            break

    print("--- DIAGNOSTICO EN VIVO ---")
    for k, v in state.items():
        print(f"{k.upper()}: {v}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
