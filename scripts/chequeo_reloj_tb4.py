"""
Mide el desfase de reloj entre esta laptop y los sensores del TurtleBot 4:
  /scan = reloj de la Raspberry Pi (lidar)      /odom = reloj del Create 3 (base)
Lo que importa es que Pi y Create 3 COINCIDAN entre si: si difieren, SLAM descarta los escaneos
("Message Filter dropping ... queue is full") y no hay mapa. El desfase con la laptop da igual.

Uso (terminal WSL conectada al robot):
  python3 chequeo_reloj_tb4.py             # una medida
  python3 chequeo_reloj_tb4.py --esperar   # repite cada 20 s hasta que coincidan (max 15 min)
Solo LEE; espera hasta 15 s a que lleguen los datos en cada medida.
"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def medir(n):
    r = {}
    subs = [
        n.create_subscription(LaserScan, "/scan", lambda m: r.setdefault("scan", m.header.stamp.sec),
                              qos_profile_sensor_data),
        n.create_subscription(Odometry, "/odom", lambda m: r.setdefault("odom", m.header.stamp.sec),
                              qos_profile_sensor_data),
    ]
    t = time.time()
    while len(r) < 2 and time.time() - t < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
    now = time.time()
    for s in subs:
        n.destroy_subscription(s)
    return {k: now - v for k, v in r.items()}


def informe(off):
    for k, who in (("scan", "Pi"), ("odom", "Create 3")):
        if k in off:
            print(f"/{k:4} ({who:8}): desfase vs laptop {off[k]:.0f} s ({off[k] / 86400:.1f} dias)")
        else:
            print(f"/{k:4} ({who:8}): sin datos")
    if len(off) == 2:
        d = abs(off["scan"] - off["odom"])
        if d < 2:
            print("OK: los dos relojes coinciden.")
            return True
        print(f"PROBLEMA: Pi y Create 3 difieren {d:.0f} s ({d / 86400:.1f} dias). "
              f"{'El Create 3 va ATRASADO.' if off['odom'] > off['scan'] else 'El Create 3 va ADELANTADO.'}")
    return False


def main():
    esperar = "--esperar" in sys.argv
    rclpy.init()
    n = rclpy.create_node("chequeo_reloj_tb4")
    t0 = time.time()
    while True:
        ok = informe(medir(n))
        if ok or not esperar or time.time() - t0 > 900:
            break
        print("   ...reintentando en 20 s (Ctrl+C para salir)")
        time.sleep(20)
    if esperar and not ok:
        print("Sigue sin coincidir tras 15 min: reinicia la aplicacion del Create 3 (web 192.168.186.2) o el robot.")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
