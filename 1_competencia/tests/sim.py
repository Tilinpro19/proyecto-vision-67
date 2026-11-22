"""Simulador minimo para las pruebas: scans ideales por trazado de rayos contra
segmentos ((x1, y1), (x2, y2)) y un robot uniciclo con odometria que deriva."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import nav_core as nc  # noqa: E402


def raycast(segments, n=720, max_r=12.0):
    """Scan ideal desde el origen contra segmentos ((x1, y1), (x2, y2))."""
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    ranges = np.full(n, np.inf)
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    for (x1, y1), (x2, y2) in segments:
        a, e = np.array([x1, y1]), np.array([x2 - x1, y2 - y1])
        den = d[:, 0] * -e[1] - d[:, 1] * -e[0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (a[0] * -e[1] - a[1] * -e[0]) / den   # distancia por el rayo
            u = (d[:, 0] * a[1] - d[:, 1] * a[0]) / den  # posicion en el segmento
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        ranges = np.where(hit & (t < ranges), t, ranges)
    ranges[ranges > max_r] = np.inf
    return nc.clean_scan(ranges, angles)


def box(cx, cy, w, h):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def corridor(y_left, y_right, length=6.0):
    return [((-1.0, y_left), (length, y_left)), ((-1.0, y_right), (length, y_right))]


def seg_dist(p, seg):
    a, b = np.array(seg[0], float), np.array(seg[1], float)
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def scan_from(segments, pose):
    """Scan visto desde pose (x, y, yaw) en el marco del robot."""
    inv = nc.inverse(pose)
    local = [(nc.compose(inv, (*s[0], 0.0))[:2], nc.compose(inv, (*s[1], 0.0))[:2])
             for s in segments]
    return raycast(local)


class Sim:
    """walls: los ve el LiDAR. low: segmentos que chocan pero el LiDAR no ve.
    yaw_drift: error relativo de la odometria al girar (deriva).
    scan_retardo: pasos (dt) de atraso del LiDAR (en la Pi el ciclo llega a 0.3 s)."""

    def __init__(self, walls, low=(), radius=0.17, yaw_drift=0.0, start=(0.0, 0.0, 0.0),
                 scan_retardo=0):
        self.walls, self.low, self.radius = list(walls), list(low), radius
        self.true = start
        self.odom = (5.0, -2.0, 0.7)  # el /odom no arranca en A
        self.yaw_drift = yaw_drift
        self.collisions = 0
        self.min_clear = math.inf
        self._hist = [start] * (scan_retardo + 1)

    def scan(self):
        return scan_from(self.walls, self._hist[0])

    def clearance(self, xy, segs):
        return min((seg_dist(np.asarray(xy), s) for s in segs), default=math.inf)

    def step(self, lin, ang, dt):
        x, y, th = self.true
        nx, ny = x + lin * math.cos(th) * dt, y + lin * math.sin(th) * dt
        blocked = (self.clearance((nx, ny), self.walls + self.low) < self.radius)
        if blocked:
            if self.clearance((nx, ny), self.walls) < self.radius:
                self.collisions += 1
            lin = 0.0
            nx, ny = x, y
        self.true = (nx, ny, float(nc.wrap(th + ang * dt)))
        self._hist = self._hist[1:] + [self.true]
        self.min_clear = min(self.min_clear, self.clearance((nx, ny), self.walls))
        # odometria: integra con un error de escala en el giro
        self.odom = nc.compose(self.odom, (lin * dt, 0.0, ang * dt * (1 + self.yaw_drift)))


def run(sim, piloto, goal, t_max=180.0, dt=0.1, pause=0.0,
        latencia=0, lidar_cada=1, ruido=0.0, seed=0):
    """Mision completa A -> B -> A en el simulador. Devuelve dict con resultados.
    Modo realista (por defecto apagado, asi los tests de siempre no cambian):
      latencia   ciclos que tarda un cmd_vel en llegar a las ruedas (DDS por Wi-Fi)
      lidar_cada el scan se renueva cada tantos ciclos (RPLIDAR A1 ~5-7 Hz + DDS)
      ruido      desviacion tipica (m) del ruido de rango del LiDAR"""
    rng = np.random.default_rng(seed)
    cola = [(0.0, 0.0)] * latencia
    scan, edad = sim.scan(), 0

    def leer():
        r, a = scan
        return (r + rng.normal(0.0, ruido, r.shape), a) if ruido else (r, a)

    piloto.fijar_origen(sim.odom, *leer())
    piloto.ir_a(goal)
    t, phase, t_b, reached_b = 0.0, "ida", None, False
    while t < t_max:
        lin, ang, info = piloto.paso(t, sim.odom, *leer())
        cola.append((lin, ang))
        lin, ang = cola.pop(0)
        edad += 1
        if edad >= lidar_cada:
            scan, edad = sim.scan(), 0
        if phase == "ida" and info["dist"] < piloto.pp.goal_tol:
            phase, t_b, reached_b = "vuelta", t, True
            b_true = sim.true
            piloto.volver()
            continue
        if phase == "vuelta" and info["llegada"]:
            return dict(ok=True, t=t, t_b=t_b, true=sim.true, b_true=b_true, est=info["pose"],
                        collisions=sim.collisions, min_clear=sim.min_clear)
        sim.step(lin, ang, dt)
        t += dt
    return dict(ok=False, t=t, reached_b=reached_b, true=sim.true,
                collisions=sim.collisions, min_clear=sim.min_clear)
