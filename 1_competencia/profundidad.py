"""Guardian de profundidad (OAK-D): ve obstaculos BAJOS que el LiDAR no ve (bases/patas de silla).

Convierte el mapa de profundidad en un scan virtual (r, a) en el marco del robot y lo une al del
LiDAR, asi nav_core/pasillo/escape lo usan sin cambios.

Costo en la Pi: suscribirse a /oakd/stereo/image_raw cuesta ~36 % de un nucleo (es el transporte
DDS, no el calculo; la OAK calcula la profundidad). Por eso el guardian se suscribe SOLO bajo
demanda (zona estrecha, algo cerca al frente o atasco), con histeresis de `hold` s, y vuelve a
soltar la suscripcion. El calculo (reducir 1/4, plano del piso, 24 columnas) es ~1 ms.

Calibracion (cuadro real, TB4 + OAK-D, fx=564, 640x400): el piso es un plano
y = floor_h + floor_slope * z en el marco de la camara (y hacia abajo): camara a ~0.10 m del piso,
inclinada ~3.3 grados. Un punto es obstaculo si queda entre hmin y hmax por ENCIMA de ese plano.
"""
import math
import threading
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class ProfParams:
    fx: float = 564.0            # px  (se reemplaza por camera_info si llega)
    cx: float = 327.3
    cy: float = 202.8
    floor_h: float = 0.103       # m   plano del piso bajo la camara (calibrado)
    floor_slope: float = 0.057   # m/m aumento de y con la distancia (inclinacion ~3.3 grados)
    hmin: float = 0.05           # m   sobre el piso: por debajo = piso/ruido
    hmin_z: float = 0.025        # m/m el umbral sube con la distancia (error de pendiente del piso)
    hmax: float = 0.45           # m   sobre el piso: por encima el robot pasa (o lo ve el LiDAR)
    zmin: float = 0.36           # m   zona ciega de la OAK-D
    zmax: float = 2.5            # m   mas lejos no hace falta (el LiDAR cubre)
    cam_x: float = 0.10          # m   camara adelante del centro del robot (SIN MEDIR: ajustar)
    step: int = 4                # reduccion de la imagen (640x400 -> 160x100)
    nbins: int = 24              # columnas del scan virtual
    pct: float = 10.0            # percentil de z por columna (robusto a pixeles sueltos)
    min_pix: int = 6             # pixeles de obstaculo por columna para aceptar


def scan_virtual(depth_mm, p=ProfParams()):
    """depth_mm: uint16 (alto, ancho) en mm. Devuelve (r, a) float arrays (marco del robot,
    a=0 al frente, + izquierda). Vacio si no hay obstaculos bajos."""
    D = np.asarray(depth_mm)[::p.step, ::p.step].astype(np.float32) * 0.001
    h, w = D.shape
    v = (np.arange(h, dtype=np.float32) * p.step)[:, None]
    u = (np.arange(w, dtype=np.float32) * p.step)[None, :]
    y = (v - p.cy) / p.fx * D
    hf = (p.floor_h + p.floor_slope * D) - y          # altura sobre el piso
    ok = (D > p.zmin) & (D < p.zmax) & (hf > p.hmin + p.hmin_z * D) & (hf < p.hmax)
    if not ok.any():
        return np.empty(0), np.empty(0)
    edges = np.linspace(0, w, p.nbins + 1).astype(int)
    rs, as_ = [], []
    for i in range(p.nbins):
        sel = ok[:, edges[i]:edges[i + 1]]
        if sel.sum() < p.min_pix:
            continue
        z = float(np.percentile(D[:, edges[i]:edges[i + 1]][sel], p.pct))
        uc = 0.5 * (edges[i] + edges[i + 1] - 1) * p.step
        py = -(uc - p.cx) / p.fx * z                   # + izquierda
        px = z + p.cam_x
        rs.append(math.hypot(px, py))
        as_.append(math.atan2(py, px))
    return np.array(rs), np.array(as_)


class GuardianProfundidad:
    """Une el scan virtual de profundidad al del LiDAR, con suscripcion bajo demanda."""

    def __init__(self, topic="/oakd/stereo/image_raw", params=None, hold=8.0,
                 periodo=0.2, lado_cerca=0.70, frente_cerca=1.0, log=None):
        self.topic, self.p = topic, params or ProfParams()
        self.hold, self.periodo = hold, periodo
        self.lado_cerca, self.frente_cerca = lado_cerca, frente_cerca
        self.log = log or (lambda txt: None)
        self.activo_hasta = -1e9
        self.virtual = (np.empty(0), np.empty(0))
        self.t_virtual = -1e9
        self.n_activaciones = 0
        self._thread = None
        self._quiere = False
        self._lock = threading.Lock()
        self._t_proc = -1e9

    # --- decision de encender -------------------------------------------------------
    def _necesita(self, r, a):
        """Zona estrecha (paredes cerca a ambos lados) o algo cerca al frente."""
        if len(r) == 0:
            return False
        ang = np.degrees(a)
        izq = r[(ang > 60) & (ang < 120)]
        der = r[(ang < -60) & (ang > -120)]
        fr = r[np.abs(ang) < 25]
        estrecho = len(izq) > 0 and len(der) > 0 and izq.min() < self.lado_cerca and der.min() < self.lado_cerca
        cerca = len(fr) > 0 and fr.min() < self.frente_cerca
        return bool(estrecho or cerca)

    def forzar(self, t):
        """Mantener encendido hold s (p. ej. tras un atasco con frente libre)."""
        self.activo_hasta = max(self.activo_hasta, t + self.hold)

    def mezclar(self, t, r, a):
        """Llamar con cada scan del LiDAR. Devuelve (r, a) con los puntos virtuales anadidos."""
        if self._necesita(r, a):
            self.forzar(t)
        if t < self.activo_hasta:
            self._arrancar()
        else:
            self._detener()
        vr, va = self.virtual
        if len(vr) and t - self.t_virtual < 0.6:
            return np.r_[r, vr], np.r_[a, va]
        return r, a

    # --- suscripcion bajo demanda ---------------------------------------------------
    # Crear/destruir el nodo ROS tarda ~1.5 s (discovery DDS). Va todo en un hilo propio para que
    # NUNCA bloquee el ciclo de control (mezclar() corre dentro del callback del LiDAR).
    def _arrancar(self):
        with self._lock:
            self._quiere = True
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, daemon=True)
                self._thread.start()

    def _detener(self):
        self._quiere = False
        self.virtual = (np.empty(0), np.empty(0))

    @property
    def encendido(self):
        return self._thread is not None and self._quiere

    def _run(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        while True:
            node = ex = None
            try:
                node = rclpy.create_node("guardian_profundidad")
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                node.create_subscription(Image, self.topic, self._on_depth, qos)
                ex = SingleThreadedExecutor()
                ex.add_node(node)
                self.n_activaciones += 1
                self.log("profundidad ON (zona estrecha / obstaculo cerca)")
                while self._quiere:
                    ex.spin_once(timeout_sec=0.1)
            except Exception as e:  # sin romper la mision si falla la camara
                self.log(f"profundidad no disponible: {e}")
                self._quiere = False
                self.activo_hasta = -1e9
            finally:
                try:
                    if ex is not None:
                        ex.shutdown()
                    if node is not None:
                        node.destroy_node()
                except Exception:
                    pass
                self.virtual = (np.empty(0), np.empty(0))
                self.log("profundidad OFF")
            with self._lock:
                if not self._quiere:
                    self._thread = None
                    return

    def _on_depth(self, msg):
        now = time.monotonic()
        if now - self._t_proc < self.periodo:      # descarta rapido: ~5 Hz bastan
            return
        self._t_proc = now
        if msg.encoding != "16UC1":
            return
        d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
        self.virtual = scan_virtual(d, self.p)
        self.t_virtual = now

    def cerrar(self):
        self._detener()
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
