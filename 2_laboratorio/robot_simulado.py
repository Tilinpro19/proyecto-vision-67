"""
Robot simulado para probar mision.py / navegacion.py sin el TurtleBot: publica /scan
(girado 90 grados como el RPLIDAR del TB4) y /odom, y obedece /cmd_vel (Twist). Usa el
simulador de 1_competencia/tests/sim.py. No hay camara: el escaneo termina por tiempo.

Correr en un dominio aislado y solo localhost para no mezclarse con un robot real:
  export ROS_DOMAIN_ID=99 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  python3 robot_simulado.py --circuito obstaculo &
  cd ../1_competencia && python3 mision.py --tarea A --goal 3 0 --voz ninguna
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "1_competencia", "tests"))
from sim import Sim, box, scan_from  # noqa: E402

import nav_core as nc  # noqa: E402

CIRCUITOS = {
    "vacio": box(2.0, 0.0, 6.0, 4.0),
    "obstaculo": box(2.0, 0.0, 6.0, 4.0) + box(1.5, 0.0, 0.4, 0.4) + box(2.6, 1.0, 0.3, 0.6),
    "laberinto": [
        ((-0.8, -0.6), (-0.8, 0.6)),
        ((-0.8, -0.6), (1.4, -0.6)),
        ((-0.8, 0.6), (1.4, 0.6)),
        ((1.4, -0.6), (1.4, -1.8)),
        ((1.4, -1.8), (2.6, -1.8)),
        ((2.6, -1.8), (2.6, 1.6)),
        ((1.4, 0.6), (1.4, 2.8)),
        ((1.4, 2.8), (4.4, 2.8)),
        ((2.6, 1.6), (4.4, 1.6)),
        ((4.4, 1.6), (4.4, 2.8)),
    ],
}
N = 720


class RobotSimulado:
    """Nodo de prueba: timer a 20 Hz que integra /cmd_vel y publica /odom y /scan."""

    def __init__(self, node, circuito="obstaculo", deriva=0.03, verbose=True):
        import cv2
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, LaserScan

        self.cv2 = cv2
        self.Odometry, self.LaserScan, self.CompressedImage = Odometry, LaserScan, CompressedImage
        self.circuito, self.verbose = circuito, verbose
        self.sim = Sim(CIRCUITOS[circuito], yaw_drift=deriva)
        self.pub_scan = node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.pub_odom = node.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
        self.pub_cam = node.create_publisher(
            CompressedImage, "/oakd/rgb/preview/image_raw/compressed", qos_profile_sensor_data
        )
        self.imgs_senales = self._cargar_senales() if circuito == "laberinto" else {}
        self.cmd = (0.0, 0.0, 0.0)
        node.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.dt, self.k, self.last_print = 0.05, 0, 0.0
        node.create_timer(self.dt, self._tick)

    def _cargar_senales(self):
        base_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "1_competencia", "tests", "data", "senales",
        )
        out = {}
        for k, fn in (("izq", "flecha_izq.png"), ("der", "flecha_der.png"), ("bandera", "bandera_meta.png")):
            p = os.path.join(base_dir, fn)
            if os.path.isfile(p):
                out[k] = self.cv2.imread(p)
        return out

    def _frame_laberinto(self):
        x, y, th = self.sim.true
        canvas = np.full((240, 320, 3), 185, dtype=np.uint8)
        elegida = None
        if 0.6 < x < 1.95 and abs(y) < 0.55 and abs(th) < 0.65:
            elegida = self.imgs_senales.get("izq")
        elif 1.4 < x < 2.6 and 0.8 < y < 2.15 and abs(th - math.pi / 2) < 0.65:
            elegida = self.imgs_senales.get("der")
        elif x > 3.1 and 1.6 < y < 2.8:
            elegida = self.imgs_senales.get("bandera")
        if elegida is not None:
            patch = self.cv2.resize(elegida, (110, 110), interpolation=self.cv2.INTER_AREA)
            canvas[65:175, 105:215] = patch
        return canvas

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z, time.monotonic())

    def _tick(self):
        sim, t = self.sim, time.monotonic()
        v, w = self.cmd[:2] if t - self.cmd[2] < 0.5 else (0.0, 0.0)
        sim.step(v, w, self.dt)
        od = self.Odometry()
        od.header.frame_id = "odom"
        od.pose.pose.position.x, od.pose.pose.position.y = sim.odom[0], sim.odom[1]
        od.pose.pose.orientation.z = math.sin(sim.odom[2] / 2)
        od.pose.pose.orientation.w = math.cos(sim.odom[2] / 2)
        self.pub_odom.publish(od)
        if self.k % 3 == 0:  # ~7 Hz como el RPLIDAR A1
            rot, inc = math.pi / 2, 2 * math.pi / N  # el driver del TB4: 0 = izquierda
            r, a = scan_from(sim.walls, sim.true)
            ranges = np.full(N, np.inf)
            ranges[np.round((nc.wrap(a - rot) + math.pi) / inc).astype(int) % N] = r
            sc = self.LaserScan()
            sc.header.frame_id = "rplidar_link"
            sc.angle_min, sc.angle_increment = -math.pi, inc
            sc.angle_max = -math.pi + inc * (N - 1)
            sc.range_min, sc.range_max = 0.15, 12.0
            sc.ranges = ranges.astype(np.float32).tolist()
            self.pub_scan.publish(sc)
        if self.circuito == "laberinto" and self.k % 4 == 0:  # ~5 Hz camara preview
            ok, buf = self.cv2.imencode(".jpg", self._frame_laberinto(), [self.cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                cm = self.CompressedImage()
                cm.format = "jpeg"
                cm.data = buf.tobytes()
                self.pub_cam.publish(cm)
        self.k += 1
        if self.verbose and t - self.last_print > 2.0:
            self.last_print = t
            x, y, th = sim.true
            print(f"  [sim] real ({x:+.2f}, {y:+.2f}, {math.degrees(th):+4.0f} deg) "
                  f"cmd ({v:+.2f}, {w:+.2f}) choques {sim.collisions} "
                  f"holgura min {sim.min_clear:.2f} m", flush=True)


def main():
    import rclpy

    ap = argparse.ArgumentParser()
    ap.add_argument("--circuito", default="obstaculo", choices=CIRCUITOS)
    ap.add_argument("--deriva", type=float, default=0.03, help="error de escala del giro en /odom")
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("robot_simulado")
    RobotSimulado(node, args.circuito, args.deriva)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
