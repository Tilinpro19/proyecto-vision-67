"""Suscriptores a los sensores del robot. Cada uno guarda solo el ultimo mensaje
(una sola asignacion: seguro aunque el nodo gire en otro hilo) y sabe si esta fresco."""
import math
import time

import numpy as np

from .limites import LIDAR_ROT_DEG, TOPIC_CAMARA, TOPIC_HAZARD, TOPIC_ODOM, TOPIC_SCAN


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def yaw_de_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Lidar:
    """Ultimo /scan, limpio (sin NaN/inf ni fuera de rango) y con los angulos en el
    marco del robot: 0 = frente, + = izquierda. data = (ranges, angles) o None."""

    def __init__(self, node, topic=TOPIC_SCAN, rot_deg=LIDAR_ROT_DEG):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan

        self.rot = math.radians(rot_deg)
        self.data = None
        self.msg = None   # mensaje crudo, por si hace falta el stamp (reloj)
        self.last_t = 0.0
        node.create_subscription(LaserScan, topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        r = np.asarray(msg.ranges, np.float64)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment + self.rot
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        self.data = r[ok], _wrap(a[ok])
        self.msg, self.last_t = msg, time.monotonic()

    def fresh(self, max_age=0.6):
        return self.data is not None and time.monotonic() - self.last_t < max_age

    def distancia(self, rumbo=0.0, medio_ancho=math.radians(15), percentil=20):
        """Distancia (m) a lo mas cercano en [rumbo +- medio_ancho] (rad), o None.
        Percentil bajo y no el minimo: ignora un punto suelto de ruido."""
        if not self.fresh():
            return None
        ranges, angles = self.data
        r = ranges[np.abs(_wrap(angles - rumbo)) < medio_ancho]
        return float(np.percentile(r, percentil)) if len(r) else None

    distance_at = distancia  # nombre usado en sigueme.py

    def puntos_xy(self):
        """Nube (N, 2) en metros, x = frente, y = izquierda."""
        if self.data is None:
            return np.zeros((0, 2))
        r, a = self.data
        return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


class Odom:
    """Pose (x, y, yaw) del /odom del Create 3 y conversiones robot <-> odom."""

    def __init__(self, node, topic=TOPIC_ODOM):
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.pose = None
        self.vel = (0.0, 0.0)
        self.msg = None
        self.last_t = 0.0
        node.create_subscription(Odometry, topic, self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_de_quaternion(msg.pose.pose.orientation))
        t = msg.twist.twist
        self.vel = (t.linear.x, t.angular.z)
        self.msg, self.last_t = msg, time.monotonic()

    def fresh(self, max_age=0.6):
        return self.pose is not None and time.monotonic() - self.last_t < max_age

    ready = fresh  # nombre usado en sigueme.py

    def to_odom(self, p):
        x, y, t = self.pose
        c, s = math.cos(t), math.sin(t)
        return np.array([x + c * p[0] - s * p[1], y + s * p[0] + c * p[1]])

    def to_robot(self, q):
        x, y, t = self.pose
        c, s = math.cos(t), math.sin(t)
        dx, dy = q[0] - x, q[1] - y
        return np.array([c * dx + s * dy, -s * dx + c * dy])


class Camara:
    """Ultima imagen de la OAK-D. Por defecto el topico comprimido (no satura el Wi-Fi).
    Decodifica solo cuando se pide (latest/ultima), no en cada mensaje."""

    def __init__(self, node, topic=TOPIC_CAMARA):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, Image

        self.topic = topic
        self.comprimida = topic.endswith("compressed")
        self.msg = None
        self.img = None
        self.last_t = 0.0
        self._entregado_t = 0.0
        node.create_subscription(CompressedImage if self.comprimida else Image, topic,
                                 self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg):
        self.msg = msg

    def latest(self):
        """Decodifica la imagen nueva (si llego) y devuelve la ultima valida (BGR) o None."""
        msg, self.msg = self.msg, None
        if msg is not None:
            img = self._decodificar(msg)
            if img is not None:  # None = JPEG corrupto (paquete perdido por Wi-Fi)
                self.img, self.last_t = img, time.monotonic()
        return self.img

    ultima = latest

    def nueva(self):
        """Como latest(), pero None si no llego nada desde la ultima llamada."""
        img = self.latest()
        if img is None or self.last_t <= self._entregado_t:
            return None
        self._entregado_t = self.last_t
        return img

    def fresh(self, max_age=1.0):
        return self.img is not None and time.monotonic() - self.last_t < max_age

    @staticmethod
    def _decodificar(msg):
        import cv2
        if hasattr(msg, "format"):
            return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding)
        if ch is None:
            return None
        h, w = msg.height, msg.width
        arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(h, msg.step)[:, :w * ch].reshape(h, w, ch)
        if msg.encoding == "rgb8":
            arr = arr[:, :, ::-1]
        elif msg.encoding == "rgba8":
            arr = arr[:, :, 2::-1]
        elif msg.encoding == "bgra8":
            arr = arr[:, :, :3]
        elif msg.encoding == "mono8":
            arr = cv2.cvtColor(arr[:, :, 0], cv2.COLOR_GRAY2BGR)
        return np.ascontiguousarray(arr)


class Bumper:
    """Choques del Create 3 (/hazard_detection). disponible=False si faltan los
    mensajes irobot_create_msgs en esta PC (en el Docker vienen instalados)."""

    BUMP = 1

    def __init__(self, node, topic=TOPIC_HAZARD):
        self.t = 0.0
        self.choques = 0
        try:
            from irobot_create_msgs.msg import HazardDetectionVector
            from rclpy.qos import qos_profile_sensor_data
        except ImportError:
            self.disponible = False
            return
        self.disponible = True
        node.create_subscription(HazardDetectionVector, topic, self._on_hazard,
                                 qos_profile_sensor_data)

    def _on_hazard(self, msg):
        if any(d.type == self.BUMP for d in msg.detections):
            if time.monotonic() - self.t > 0.5:
                self.choques += 1
            self.t = time.monotonic()

    def activo(self, ventana=0.3):
        return time.monotonic() - self.t < ventana
