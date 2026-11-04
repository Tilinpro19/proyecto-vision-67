"""
Modo "Pokemon": el TurtleBot 4 te sigue a ~0.9 m, como un companero.

- Camara OAK-D -> cajas de "persona": YOLOX (si esta el .onnx; ve personas aunque solo
  se vean las piernas) o, si no, MediaPipe Pose (necesita verte el torso).
- LiDAR (/scan) -> piernas: bultos del ancho de una pierna. Te engancha cuando unas
  piernas caen dentro de una caja de persona, y desde ahi las sigue aunque la camara te
  pierda. Para no confundirte con sillas:
  * forma: bultos del ancho de una pierna, delante de lo que los rodea, no concavos;
  * movimiento (/odom): un bulto quieto en el mismo sitio del suelo mas de STATIC_TIME
    es un mueble y no se usa (salvo que la camara lo vea como persona);
  * color de la ropa: al engancharte guarda el color de tus pantalones; si el objetivo
    deja de parecerse te suelta, y si te pierde te vuelve a encontrar por ese color
    (y entre varias personas, elige la de tu ropa).
- Gira para tenerte centrado y avanza hasta quedar a FOLLOW_DIST.
- Si te pierde, gira despacio hacia el lado donde te vio por ultima vez y,
  si no te encuentra en SEARCH_TIME, se detiene.

Todo corre en la PC (como mano_teleop.py); el robot solo manda camara y lidar.

Uso, despues de 'source tb4_connect.sh <IP_ROBOT> <DOMAIN_ID>':
  python3 sigueme.py --dry      # solo ver deteccion y distancia (no publica)
  python3 sigueme.py            # robot real

Teclas: espacio armar/pausar (ARRANCA EN PAUSA) | u/j vel. lineal +/- |
        i/k vel. angular +/- | [ / ] hfov -/+ | r soltar y volver a engancharte | q/Esc salir
"""
import argparse
import math
import os
import threading
import time

import cv2
import numpy as np

from comun import (LIMIT_ANGULAR, LIMIT_LINEAR, STEP_ANGULAR, STEP_LINEAR, Sender, clamp)
from mano_teleop import PANEL_H, RobotCamera, fit_height

HERE = os.path.dirname(os.path.abspath(__file__))
POSE_MODEL = os.path.join(HERE, "pose_landmarker_lite.task")
# https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task
MIN_POSE_CONFIDENCE = 0.5
MIN_VISIBILITY = 0.5

# YOLOX-s de OpenCV Zoo (COCO, 80 clases; usamos solo 'person'). Si el archivo esta, se
# usa en vez de MediaPipe: detecta personas aunque solo se vean las piernas.
YOLO_MODEL = os.path.join(HERE, "object_detection_yolox_2022nov.onnx")
# https://huggingface.co/opencv/object_detection_yolox/resolve/main/object_detection_yolox_2022nov.onnx
YOLO_SIZE = 320        # el preview mide 250 px: 640 seria 4x mas lento sin ganar nada
YOLO_SCORE = 0.45      # objectness * P(persona); 0.40 daba falsos en postes
YOLO_NMS = 0.45
BOX_MARGIN_DEG = 4     # +- grados extra al comprobar si una pierna cae dentro de una caja

FOLLOW_DIST = 0.9      # m   a esta distancia la camara aun ve algo de torso
DIST_DEADBAND = 0.1    # m   +- alrededor de FOLLOW_DIST sin moverse
STOP_DIST = 0.3        # m   algo mas cerca que esto en el frente -> no avanza
K_LINEAR = 0.6         # (m/s) por m de error de distancia
K_ANGULAR = 2.5        # (rad/s) por unidad de error horizontal (-0.5..0.5)
CENTER_DEADBAND = 0.06 # error horizontal sin girar
LOST_HOLD = 0.4        # s   sigue con el ultimo comando tras perderte (frames perdidos)
SEARCH_TIME = 6.0      # s   girando para buscarte antes de rendirse
SEARCH_ANGULAR = 0.6   # rad/s
START_LINEAR = 0.2     # m/s
START_ANGULAR = 1.0    # rad/s
LOOP_DT = 1 / 30       # s   periodo del bucle de control

# Piernas en el LiDAR (el del TB4 corta a la altura de las espinillas)
CLUSTER_GAP = 0.10     # m   salto entre puntos seguidos que separa dos objetos
LEG_MIN_W = 0.05       # m   ancho minimo de una pierna (patas de silla finas quedan fuera)
LEG_MAX_W = 0.45       # m   maximo (las dos piernas juntas se ven como un solo bulto)
LEG_MIN_PTS = 3
FAR_RANGE = 2.0        # m   mas lejos, el lidar da solo ~2 puntos por pierna: aceptar 2
CONCAVE_MAX = 0.01     # m   si el centro del bulto queda mas lejos que sus bordes = esquina
                       #     o respaldo, no pierna (una pierna es convexa hacia el robot)
PAIR_DIST = 0.45       # m   dos piernas mas cerca que esto = una persona
PEOPLE_RANGE = 3.0     # m   no buscar piernas mas lejos
TRACK_GATE = 0.45      # m   cuanto puede moverse la persona entre dos scans
TRACK_TIMEOUT = 0.6    # s   sin ver tus piernas -> te da por perdido
CAM_FRESH = 0.5        # s   antiguedad maxima de la deteccion de camara

# Muebles: bultos quietos respecto al suelo (con /odom)
STATIC_CELL = 0.15     # m   tamano de celda
STATIC_TIME = 4.0      # s   quieto mas que esto en la misma celda = mueble
STATIC_FORGET = 2.0    # s   celda a la vista y vacia este tiempo -> se olvida (se movio)

# Color de la ropa (parche de piernas en la imagen, donde el lidar dice que estas)
CAM_HEIGHT = 0.20      # m   altura aprox. de la camara del TB4 Lite (medir si falla)
PANTS_Z = (0.15, 0.60) # m   alturas del parche (espinilla a muslo)
PANTS_HALF_W = 0.10    # m   medio ancho del parche
COLOR_MATCH = 0.40     # distancia Bhattacharyya: por debajo = misma ropa
COLOR_REJECT = 0.65    # por encima durante COLOR_REJECT_TIME -> no eres tu, soltar
COLOR_REJECT_TIME = 1.0
COLOR_LEARN = 0.05     # cuanto se actualiza el color de referencia (cambios de luz)
REACQ_RANGE = 2.0      # m   re-enganche solo por color hasta esta distancia

TORSO = (11, 12, 23, 24)  # hombros y caderas
SKELETON = [(11, 12), (11, 23), (12, 24), (23, 24), (11, 13), (13, 15), (12, 14), (14, 16),
            (23, 25), (25, 27), (24, 26), (26, 28)]


class PersonDetector:
    """MediaPipe Pose en modo video. find(img) -> (puntos (33,2) px, visibles (33,) bool) o None."""

    def __init__(self):
        if not os.path.isfile(POSE_MODEL):
            raise SystemExit(f"Falta el modelo {POSE_MODEL}. Descargalo (URL en sigueme.py) "
                             "con internet y dejalo junto al script.")
        try:
            import mediapipe as mp
            from mediapipe.tasks.python import BaseOptions, vision
        except ImportError:
            raise SystemExit("Falta mediapipe: ver 'Instalacion' en seguimiento_vision/README.md.")
        self.mp = mp
        options = vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=POSE_MODEL),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=MIN_POSE_CONFIDENCE,
            min_pose_presence_confidence=MIN_POSE_CONFIDENCE,
            min_tracking_confidence=0.5)
        self.landmarker = vision.PoseLandmarker.create_from_options(options)
        self.last_ts = -1
        self.det = None

    def find(self, img):
        h, w = img.shape[:2]
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.last_ts = max(int(time.monotonic() * 1000), self.last_ts + 1)
        result = self.landmarker.detect_for_video(
            self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), self.last_ts)
        if not result.pose_landmarks:
            return None
        lms = result.pose_landmarks[0]
        pts = np.array([[lm.x * w, lm.y * h] for lm in lms], np.float32)
        vis = np.array([lm.visibility > MIN_VISIBILITY and 0 <= lm.x <= 1 for lm in lms])
        if not vis.any():
            return None
        return pts, vis

    def boxes(self, img):
        """Misma interfaz que YoloPersons: caja de los puntos visibles, solo con torso
        (MediaPipe pone "esqueletos" en sillas, pero casi nunca con hombros y caderas)."""
        self.det = det = self.find(img)
        if det is None or sum(det[1][i] for i in TORSO) < 2:
            return []
        pts = det[0][det[1]]
        return [(*pts.min(axis=0), *pts.max(axis=0), 1.0)]

    def close(self):
        self.landmarker.close()


class YoloPersons:
    """YOLOX-s (COCO) con cv2.dnn, solo la clase 'person'. Al estar entrenado con gente
    cortada por el borde de la foto, detecta personas de las que solo se ven las piernas.
    boxes(img) -> [(x0, y0, x1, y1, score)] en pixeles de img."""

    det = None  # no hay esqueleto que dibujar

    def __init__(self):
        self.net = cv2.dnn.readNet(YOLO_MODEL)
        grids, strides = [], []
        for s in (8, 16, 32):
            n = YOLO_SIZE // s
            gy, gx = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
            grids.append(np.stack([gx, gy], -1).reshape(-1, 2))
            strides.append(np.full((n * n, 1), s))
        self.grids = np.concatenate(grids).astype(np.float32)
        self.strides = np.concatenate(strides).astype(np.float32)

    def boxes(self, img):
        h, w = img.shape[:2]
        k = min(YOLO_SIZE / h, YOLO_SIZE / w)
        nw, nh = int(w * k), int(h * k)
        canvas = np.full((YOLO_SIZE, YOLO_SIZE, 3), 114, np.float32)  # letterbox arriba-izq
        canvas[:nh, :nw] = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), (nw, nh))
        self.net.setInput(canvas.transpose(2, 0, 1)[None])
        out = self.net.forward().reshape(-1, 85)  # cx, cy, w, h, obj, 80 clases
        score = out[:, 4] * out[:, 5]             # clase 0 = person
        m = score > YOLO_SCORE
        if not m.any():
            return []
        xy = (out[m, :2] + self.grids[m]) * self.strides[m]
        wh = np.exp(out[m, 2:4]) * self.strides[m]
        xywh = np.concatenate([xy - wh / 2, wh], axis=1) / k
        keep = np.array(cv2.dnn.NMSBoxes(xywh.tolist(), score[m].tolist(),
                                         YOLO_SCORE, YOLO_NMS)).flatten()
        return [(*xywh[i, :2], *(xywh[i, :2] + xywh[i, 2:]), float(score[m][i])) for i in keep]

    def close(self):
        pass


class AsyncDetector:
    """Corre el detector en su propio hilo para que el control (lidar, ~30 Hz) no se
    congele mientras YOLO/MediaPipe procesan una imagen. Siempre procesa la ultima."""

    def __init__(self, detector):
        self.detector = detector
        self.pending = None  # (img, yaw del robot al recibirla)
        self.result = None   # (boxes, det, yaw, t)
        self.wake = threading.Event()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def submit(self, img, yaw):
        self.pending = (img, yaw)
        self.wake.set()

    def _loop(self):
        while self.running:
            self.wake.wait(0.5)
            self.wake.clear()
            job, self.pending = self.pending, None
            if job is not None:
                boxes = self.detector.boxes(job[0])
                self.result = (boxes, self.detector.det, job[1], time.monotonic())

    def close(self):
        self.running = False
        self.wake.set()
        self.thread.join(timeout=2.0)
        self.detector.close()


class Lidar:
    """Ultimo /scan, con los angulos ya en el marco del robot (0 = frente, + = izquierda)."""

    def __init__(self, node, topic, rot_deg):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan

        self.rot = math.radians(rot_deg)
        self.data = None  # (ranges, angles): una sola asignacion, segura entre hilos
        self.last_t = 0.0
        node.create_subscription(LaserScan, topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        r = np.asarray(msg.ranges, np.float32)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment + self.rot
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        self.data = r[ok], np.arctan2(np.sin(a[ok]), np.cos(a[ok]))
        self.last_t = time.monotonic()

    def fresh(self):
        return self.data is not None and time.monotonic() - self.last_t < 1.0

    def distance_at(self, bearing, half_width):
        """Distancia al objeto mas cercano en [bearing +- half_width] (rad), o None."""
        if not self.fresh():
            return None
        ranges, angles = self.data
        diff = np.abs(np.arctan2(np.sin(angles - bearing), np.cos(angles - bearing)))
        r = ranges[diff < half_width]
        # percentil bajo y no el minimo: ignora un punto suelto de ruido
        return float(np.percentile(r, 20)) if len(r) else None

    def people(self, half_fov):
        """Posibles personas (x, y) en m (x = frente, y = izquierda), segun bultos del
        ancho de una pierna delante del robot."""
        if not self.fresh():
            return []
        ranges, angles = self.data
        m = np.abs(angles) < half_fov
        order = np.argsort(angles[m])
        a, r = angles[m][order], ranges[m][order]
        xy = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
        if len(xy) < LEG_MIN_PTS:
            return []
        cuts = np.where(np.linalg.norm(np.diff(xy, axis=0), axis=1) > CLUSTER_GAP)[0] + 1
        legs = []
        for i0, i1 in zip(np.r_[0, cuts], np.r_[cuts, len(xy)]):
            c = xy[i0:i1]
            min_pts = LEG_MIN_PTS if r[i0:i1].min() < FAR_RANGE else 2
            if (len(c) < min_pts or r[i0:i1].min() > PEOPLE_RANGE
                    or not LEG_MIN_W <= np.linalg.norm(c[-1] - c[0]) <= LEG_MAX_W):
                continue
            # una pierna esta DELANTE de lo que la rodea: si un vecino esta mas cerca es
            # un trozo de pared/mueble tapado (p. ej. la pared detras de tus piernas)
            if i0 == 0 or i1 == len(xy) or r[i0 - 1] < r[i0] or r[i1] < r[i1 - 1]:
                continue
            # forma: el centro del bulto no puede quedar detras de la cuerda entre sus
            # bordes (esquinas, respaldos y huecos son concavos; una pierna es convexa)
            u = (c[-1] - c[0]) / np.linalg.norm(c[-1] - c[0])
            n = np.array([-u[1], u[0]])
            if n @ -c[0] < 0:
                n = -n  # normal apuntando hacia el robot
            if len(c) > 2 and np.mean((c[1:-1] - c[0]) @ n) < -CONCAVE_MAX:
                continue
            legs.append(c.mean(axis=0))
        # junta pares de piernas; una pierna sola tambien vale (la otra puede taparse)
        people, used = [], set()
        for i, p in enumerate(legs):
            if i in used:
                continue
            for j in range(i + 1, len(legs)):
                if j not in used and np.linalg.norm(legs[j] - p) < PAIR_DIST:
                    p = (p + legs[j]) / 2
                    used.add(j)
                    break
            people.append(p)
        return people


class Odom:
    """Pose del robot en el marco odom (/odom del Create 3) para saber que bultos se
    mueven respecto al suelo y cuales no."""

    def __init__(self, node, topic):
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.pose = None  # (x, y, yaw)
        self.last_t = 0.0
        node.create_subscription(Odometry, topic, self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose, self.last_t = (p.x, p.y, yaw), time.monotonic()

    def ready(self):
        return self.pose is not None and time.monotonic() - self.last_t < 1.0

    def to_odom(self, p):
        x, y, t = self.pose
        c, s = math.cos(t), math.sin(t)
        return np.array([x + c * p[0] - s * p[1], y + s * p[0] + c * p[1]])

    def to_robot(self, q):
        x, y, t = self.pose
        c, s = math.cos(t), math.sin(t)
        dx, dy = q[0] - x, q[1] - y
        return np.array([c * dx + s * dy, -s * dx + c * dy])


class StaticMap:
    """Celdas del suelo (marco odom) donde hay un bulto quieto desde hace rato: muebles.
    Tu celda nunca se marca mientras te sigue (te puedes quedar quieto)."""

    def __init__(self):
        self.cells = {}  # (i, j) -> [primera vez, ultima vez]

    def _find(self, q):
        """Celda existente en la vecindad de q (el centro del bulto tiembla entre celdas)."""
        i, j = int(round(q[0] / STATIC_CELL)), int(round(q[1] / STATIC_CELL))
        for di in (0, -1, 1):
            for dj in (0, -1, 1):
                if (i + di, j + dj) in self.cells:
                    return i + di, j + dj
        return i, j

    def is_static(self, q, now):
        c = self.cells.get(self._find(q))
        return c is not None and c[1] - c[0] > STATIC_TIME and now - c[1] < STATIC_FORGET

    def update(self, qs, target_q, odom, half_fov, now):
        for q in qs:
            if target_q is not None and np.linalg.norm(q - target_q) < TRACK_GATE:
                continue
            k = self._find(q)
            self.cells[k] = [self.cells.get(k, [now])[0], now]
        # olvidar celdas vacias que el lidar esta mirando: lo que habia se fue
        for k, (_, last) in list(self.cells.items()):
            p = odom.to_robot(np.array(k) * STATIC_CELL)
            in_view = abs(math.atan2(p[1], p[0])) < half_fov and np.hypot(*p) < PEOPLE_RANGE
            if now - last > (STATIC_FORGET if in_view else 60.0):
                del self.cells[k]


class Appearance:
    """Color de la ropa (pantalones) como histograma HSV de un parche de la imagen en la
    direccion y distancia que da el lidar. No depende de MediaPipe: sirve de cerca."""

    def __init__(self, hfov):
        self.hfov = hfov
        self.ref = None

    def patch(self, img, p):
        h, w = img.shape[:2]
        vfov = self.hfov * h / w
        d = float(np.hypot(*p))
        u = (0.5 - math.atan2(p[1], p[0]) / self.hfov) * w
        hw = math.atan(PANTS_HALF_W / d) / self.hfov * w
        row = lambda z: h * (0.5 + math.atan((CAM_HEIGHT - z) / d) / vfov)
        x0, x1 = int(max(0, u - hw)), int(min(w, u + hw))
        y0, y1 = int(max(0, row(PANTS_Z[1]))), int(min(h, row(PANTS_Z[0])))
        return (x0, y0, x1, y1) if x1 - x0 >= 6 and y1 - y0 >= 6 else None

    def hist(self, img, p):
        box = self.patch(img, p)
        if box is None:
            return None
        x0, y0, x1, y1 = box
        hsv = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        hst = cv2.calcHist([hsv], [0, 1, 2], None, [12, 4, 4], [0, 180, 0, 256, 0, 256])
        return cv2.normalize(hst, None).flatten()

    def dist(self, img, p):
        """0 = misma ropa .. 1 = nada que ver; None si no se puede medir."""
        hst = self.hist(img, p)
        if self.ref is None or hst is None:
            return None
        return float(cv2.compareHist(self.ref, hst, cv2.HISTCMP_BHATTACHARYYA))

    def learn(self, img, p, rate):
        hst = self.hist(img, p)
        if hst is not None:
            self.ref = hst if self.ref is None else (1 - rate) * self.ref + rate * hst


class LegTracker:
    """Sigue a UNA persona en el lidar. Se engancha cuando la camara te ve (torso) y hay
    piernas en esa direccion, o -si ya te conocia- por el color de tu ropa. Despues sigue
    la pierna mas cercana a donde estabas, aunque la camara ya no te detecte. Los bultos
    quietos (muebles) nunca se usan para engancharse ni para saltar a ellos."""

    def __init__(self):
        self.target = None  # (x, y) en el marco del robot
        self.last_t = 0.0
        self.scan_t = 0.0
        self.people = []    # candidatos validos del ultimo scan
        self.furniture = [] # descartados por quietos
        self.static = StaticMap()

    def update(self, lidar, odom, cam_boxes, color_dist, half_fov, now):
        """cam_boxes: [(angulo_izq, angulo_der)] de las personas que ve la camara (rad,
        + = izquierda), o None si no hay imagen reciente.
        color_dist(p) -> distancia de color al objetivo conocido, o None."""
        if lidar.last_t != self.scan_t:  # solo con scan nuevo
            self.scan_t = lidar.last_t
            people = lidar.people(half_fov)
            boxes = cam_boxes or []
            margin = math.radians(BOX_MARGIN_DEG)
            in_box = [any(right - margin <= math.atan2(p[1], p[0]) <= left + margin
                          for left, right in boxes) for p in people]
            use_odom = odom is not None and odom.ready()
            qs = [odom.to_odom(p) for p in people] if use_odom else []
            # quieto pero la camara dice que es una persona -> no es mueble
            still = ([self.static.is_static(q, now) and not b for q, b in zip(qs, in_box)]
                     if use_odom else [False] * len(people))
            self.people = [p for p, s in zip(people, still) if not s]
            self.furniture = [p for p, s in zip(people, still) if s]
            seen = [p for p, s, b in zip(people, still, in_box) if b and not s]
            color_of = color_dist or (lambda p: None)

            if self.target is not None:
                d = [np.linalg.norm(p - self.target) for p in self.people]
                if d and min(d) < TRACK_GATE:
                    self.target, self.last_t = self.people[int(np.argmin(d))], now
            elif seen:
                # piernas dentro de una caja de persona. Si ya conocemos tu ropa, solo
                # te re-engancha a ti; si no, la persona mas cercana
                scored = [(color_of(p), p) for p in seen]
                if any(c is not None for c, _ in scored):
                    scored = [(c, p) for c, p in scored if c is not None and c < COLOR_MATCH]
                    best = min(scored, key=lambda cp: cp[0])[1] if scored else None
                else:
                    best = min(seen, key=lambda p: float(np.hypot(*p)))
                if best is not None:
                    self.target, self.last_t = best, now
            elif color_dist is not None:
                # la camara no ve a nadie: buscar tu ropa entre las piernas del lidar
                scored = [(color_dist(p), p) for p in self.people if np.hypot(*p) < REACQ_RANGE]
                scored = [(c, p) for c, p in scored if c is not None and c < COLOR_MATCH]
                if scored:
                    self.target = min(scored, key=lambda cp: cp[0])[1]
                    self.last_t = now

            if use_odom:
                tq = odom.to_odom(self.target) if self.target is not None else None
                # lo que la camara ve como persona tampoco se aprende como mueble
                self.static.update([q for q, b in zip(qs, in_box) if not b], tq, odom,
                                   half_fov, now)
        if self.target is not None and now - self.last_t > TRACK_TIMEOUT:
            self.target = None
        return self.target

    def reset(self):
        self.target = None


def draw(img, det, cx, dist, state, armed, linear, angular, max_l, max_a, legs_cx=(),
         furniture_cx=(), color=None, patch=None, boxes=(), hfov_hint=None, hfov_deg=None):
    scale = PANEL_H / img.shape[0]
    img = fit_height(img, PANEL_H)
    h, w = img.shape[:2]
    if det is not None:
        pts, vis = det
        p = (pts * scale).astype(int)
        for a, b in SKELETON:
            if vis[a] and vis[b]:
                cv2.line(img, tuple(p[a]), tuple(p[b]), (255, 100, 0), 2)
    for x0, y0, x1, y1, score in boxes:  # personas que ve la camara
        cv2.rectangle(img, (int(x0 * scale), int(y0 * scale)), (int(x1 * scale), int(y1 * scale)),
                      (255, 200, 0), 2)
        cv2.putText(img, f"{score:.2f}", (int(x0 * scale) + 3, int(y1 * scale) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
    if patch is not None:  # zona donde mide el color de tu ropa
        x0, y0, x1, y1 = (int(v * scale) for v in patch)
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 0, 255), 1)
    for lx in legs_cx:  # piernas candidatas del lidar (marcas abajo)
        cv2.drawMarker(img, (int(lx * w), h - 15), (0, 200, 255), cv2.MARKER_TRIANGLE_UP, 14, 2)
    for lx in furniture_cx:  # bultos quietos = muebles, descartados
        cv2.drawMarker(img, (int(lx * w), h - 15), (90, 90, 90), cv2.MARKER_TILTED_CROSS, 12, 2)
    if cx is not None:
        x = int(cx * w)
        cv2.line(img, (x, 60), (x, h), (0, 255, 0), 2)
    cv2.line(img, (w // 2, 60), (w // 2, h), (120, 120, 120), 1)

    cv2.rectangle(img, (0, 0), (w, 60), (0, 0, 0), -1)
    d = f"{dist:.2f} m" if dist is not None else "--"
    c = f" | color {color:.2f}" if color is not None else ""
    cv2.putText(img, f"{state} | dist {d}{c}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 0) if state.startswith("SIGUIENDO") else (0, 200, 255), 2)
    if hfov_hint is not None and abs(hfov_hint - hfov_deg) > 4:
        cv2.putText(img, f"hfov {hfov_deg:.0f} -> sugerido {hfov_hint:.0f} ([ ])", (10, h - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    arm = "ARMADO" if armed else "PAUSA (espacio)"
    cv2.putText(img, f"{arm} | v {linear:+.2f}/{max_l:.2f} w {angular:+.1f}/{max_a:.1f}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 255) if armed else (200, 200, 200), 1)
    if not armed:  # en pausa no se mueve aunque te vea: que se note
        cv2.putText(img, "PAUSA: clic aqui y ESPACIO para que se mueva", (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)
    return img


def spin_quietly(executor):
    try:
        executor.spin()
    except Exception:  # al cerrar rclpy el executor puede quejarse: ya da igual
        pass


def main():
    ap = argparse.ArgumentParser(description="El TurtleBot 4 te sigue (modo Pokemon)")
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--dry", action="store_true", help="no publicar, solo mostrar")
    ap.add_argument("--robot-cam", default="/oakd/rgb/preview/image_raw/compressed")
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--detector", choices=("auto", "yolo", "mediapipe"), default="auto",
                    help="auto = YOLOX si esta el .onnx, si no MediaPipe")
    ap.add_argument("--lidar-rot", type=float, default=90.0,
                    help="grados a sumar al /scan para que 0 sea el frente (TB4: 90)")
    ap.add_argument("--hfov", type=float, default=37.0,
                    help="campo de vision horizontal de la imagen (grados). El preview 250x250 "
                         "del TB4 recorta el centro: ~37 (medido en el lab: 36), no los 69 del sensor")
    ap.add_argument("--dist", type=float, default=FOLLOW_DIST, help="distancia de seguimiento (m)")
    args = ap.parse_args()

    import rclpy
    rclpy.init()
    node = rclpy.create_node("sigueme")

    win = "Sigueme TurtleBot 4"
    sender = detector = executor = spin_thread = None
    try:
        use_yolo = args.detector == "yolo" or (args.detector == "auto" and os.path.isfile(YOLO_MODEL))
        if use_yolo and not os.path.isfile(YOLO_MODEL):
            raise SystemExit(f"Falta {YOLO_MODEL}. Descargalo (URL en sigueme.py) con internet.")
        detector = YoloPersons() if use_yolo else PersonDetector()
        print("Detector de personas:", "YOLOX (ve piernas sueltas)" if use_yolo
              else "MediaPipe Pose (necesita verte el torso)")
        sender = Sender(node, args.topic, args.dry)
        cam = RobotCamera(node, args.robot_cam)
        lidar = Lidar(node, args.scan, args.lidar_rot)
        odom = Odom(node, args.odom)
        hfov = math.radians(args.hfov)
        look = Appearance(hfov)

        # ROS en su propio hilo: atiende TODOS los mensajes al llegar. Antes el bucle hacia
        # un spin_once (= 1 mensaje) por vuelta y /odom+/scan+camara se acumulaban (lag)
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=spin_quietly, args=(executor,), daemon=True)
        spin_thread.start()
        detector = AsyncDetector(detector)

        print("Haz CLIC en la ventana y pulsa ESPACIO para armar. q para salir.")
        print(f"hfov {args.hfov:.0f} grados: si los triangulos naranjas no caen sobre las piernas, "
              "ajustalo con [ y ]")
        if args.dry:
            print("[MODO DRY: no se publica nada]")

        armed = False
        max_l, max_a = START_LINEAR, START_ANGULAR
        tracker = LegTracker()
        half_fov = hfov / 2 + math.radians(15)  # algo mas que la camara: no perderte al girar
        last_seen_t, last_side = 0.0, 1.0
        linear = angular = 0.0
        det, cx, dist = None, 0.5, None
        boxes, boxes_t, boxes_yaw, img_w = [], 0.0, None, 1
        res_t = 0.0
        hfov_samples = []  # calibracion: hfov que haria coincidir caja y piernas
        color, color_bad_t = None, None
        odom_warned = False
        cam_t = 0.0
        view = np.zeros((PANEL_H, PANEL_H, 3), np.uint8)
        sent_moving = False

        next_t = 0.0
        while True:
            # ~30 Hz: sin esto el bucle giraria sin pausa e inundaria /cmd_vel
            time.sleep(max(0.0, next_t - time.monotonic()))
            next_t = time.monotonic() + LOOP_DT
            img = cam.latest()
            now = time.monotonic()

            new_img = img is not None and cam.last_t != cam_t
            if new_img:
                cam_t = cam.last_t
                img_w = img.shape[1]
                detector.submit(img, odom.pose[2] if odom.ready() else None)
            res = detector.result
            if res is not None and res[3] != res_t:  # resultado nuevo del detector
                boxes, det, boxes_yaw, res_t = res
                boxes_t = now
                # 1 persona en camara + 1 en lidar, fuera del centro -> muestra de hfov
                front = [p for p in tracker.people + tracker.furniture
                         if abs(math.atan2(p[1], p[0])) < math.radians(45)]
                if len(boxes) == 1 and len(front) == 1:
                    u = (boxes[0][0] + boxes[0][2]) / 2 / img_w
                    if abs(u - 0.5) > 0.15:
                        est = math.degrees(math.atan2(front[0][1], front[0][0]) / (0.5 - u))
                        if 20 < est < 120:
                            hfov_samples = (hfov_samples + [est])[-30:]
            cam_fresh = now - boxes_t < CAM_FRESH
            # cajas -> angulos (+ = izquierda; izquierda de la imagen = izquierda del robot).
            # La imagen es de hace un rato: descontar lo que giro el robot desde entonces
            turn = 0.0
            if boxes_yaw is not None and odom.ready():
                turn = math.atan2(math.sin(odom.pose[2] - boxes_yaw), math.cos(odom.pose[2] - boxes_yaw))
            cam_boxes = ([((0.5 - b[0] / img_w) * hfov - turn, (0.5 - b[2] / img_w) * hfov - turn)
                          for b in boxes] if cam_fresh else None)
            img_ok = img is not None and now - cam.last_t < CAM_FRESH
            color_dist = (lambda p: look.dist(img, p)) if img_ok and look.ref is not None else None
            if not odom.ready() and not odom_warned and now - lidar.last_t < 1.0:
                odom_warned = True
                print(f"Aviso: no llega {args.odom}; sin filtro de muebles quietos")
            target = tracker.update(lidar, odom, cam_boxes, color_dist, half_fov, now)

            # color de la ropa: aprender al engancharte, soltar si deja de parecerse
            color = None
            if target is not None and new_img:
                color = look.dist(img, target)
                if look.ref is None or (color is not None and color < COLOR_MATCH):
                    look.learn(img, target, 1.0 if look.ref is None else COLOR_LEARN)
                    color_bad_t = None
                elif color is not None and color > COLOR_REJECT:
                    color_bad_t = color_bad_t or now
                    if now - color_bad_t > COLOR_REJECT_TIME:
                        tracker.reset()
                        target, color_bad_t = None, None
                        print("Soltado: el objetivo ya no tiene el color de tu ropa")

            approach_only = False
            if target is not None:
                bearing, dist = math.atan2(target[1], target[0]), float(np.hypot(*target))
                state = "SIGUIENDO (piernas)"
            elif cam_boxes:
                # la camara ve a alguien pero aun no hay piernas validas ahi (p. ej. lejos):
                # ir hacia la persona mas grande con la distancia del lidar en su caja
                left, right = max(cam_boxes, key=lambda lr: lr[0] - lr[1])
                bearing = (left + right) / 2
                dist = lidar.distance_at(bearing, max((left - right) / 2, math.radians(3)))
                approach_only = True
                state = "VIENDOTE (acercandose)"
            else:
                bearing = dist = None
                state = "BUSCANDO"

            if bearing is not None:
                cx = 0.5 - bearing / hfov
                last_seen_t, last_side = now, (1.0 if bearing > 0 else -1.0)
                err = cx - 0.5
                angular = 0.0 if abs(err) < CENTER_DEADBAND else -K_ANGULAR * err * max_a
                if dist is None:
                    linear = 0.0
                else:
                    e = dist - args.dist
                    linear = 0.0 if abs(e) < DIST_DEADBAND else K_LINEAR * e
                    # retrocede poco y despacio: el Create 3 no tiene sensores atras
                    linear = clamp(linear, -0.05, max_l)
                    if approach_only:
                        # sin piernas confirmadas: solo acercarse, a media velocidad, hasta
                        # algo mas lejos de lo normal (la distancia puede ser de otra cosa)
                        linear = clamp(linear, 0.0, max_l / 2) if e > 0.3 else 0.0
                    # no avanzar fuerte mientras aun no estas centrado
                    linear *= max(0.0, 1.0 - 2.5 * abs(err))
            else:
                lost_for = now - last_seen_t
                if lost_for < LOST_HOLD:
                    pass  # mantener el ultimo comando unos frames
                elif lost_for < LOST_HOLD + SEARCH_TIME and last_seen_t > 0:
                    linear, angular = 0.0, last_side * min(SEARCH_ANGULAR, max_a)
                else:
                    linear = angular = 0.0

            if new_img:
                to_cx = lambda ps: [0.5 - math.atan2(p[1], p[0]) / hfov for p in ps]
                view = draw(img, det, cx if bearing is not None else None, dist, state,
                            armed, linear, angular, max_l, max_a,
                            to_cx(tracker.people), to_cx(tracker.furniture), color,
                            look.patch(img, target) if target is not None else None, boxes,
                            float(np.median(hfov_samples)) if len(hfov_samples) >= 5 else None,
                            args.hfov)
            if now - cam.last_t > 1.0:  # sin camara: no moverse a ciegas
                linear = angular = 0.0

            # Seguridad: obstaculo justo delante -> no avanzar
            front = lidar.distance_at(0.0, math.radians(25))
            if linear > 0 and front is not None and front < STOP_DIST:
                linear = 0.0

            angular = clamp(angular, -max_a, max_a)
            if armed and (linear or angular):
                sender.send(linear, angular)
                sent_moving = True
            elif sent_moving:
                sender.stop()
                sent_moving = False

            cv2.imshow(win, view)
            key = chr(cv2.waitKey(1) & 0xFF).lower()
            if key in ("q", "\x1b"):
                break
            elif key == " ":
                armed = not armed
                print("ARMADO" if armed else "PAUSA")
            elif key == "r":
                tracker.reset()
                look.ref = None  # olvidar tambien el color: puede engancharse otra persona
                print("Objetivo soltado: ponte delante con el torso visible y camina un poco")
            elif key in "[]":
                args.hfov = clamp(args.hfov + (1 if key == "]" else -1), 30, 100)
                hfov = math.radians(args.hfov)
                look.hfov = hfov
                half_fov = hfov / 2 + math.radians(15)
                print(f"hfov = {args.hfov:.0f} grados  (arranca con --hfov {args.hfov:.0f})")
            elif key in "ujik":
                if key == "u":
                    max_l = clamp(max_l + STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
                elif key == "j":
                    max_l = clamp(max_l - STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
                elif key == "i":
                    max_a = clamp(max_a + STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
                else:
                    max_a = clamp(max_a - STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
                print(f"Velocidad max: lineal {max_l:.2f} m/s | angular {max_a:.1f} rad/s")
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if sender is not None:
                sender.stop()
        finally:
            if detector is not None:
                detector.close()
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
                spin_thread.join(timeout=2.0)  # sin esto el hilo muere a medias al salir (core dump)
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.try_shutdown()
            print("\nDetenido. Adios.")


if __name__ == "__main__":
    main()
