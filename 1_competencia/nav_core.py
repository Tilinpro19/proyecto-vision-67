"""
Logica pura de navegacion reactiva (VFH-lite) para la Tarea A. Sin ROS: solo numpy,
para poder probarla con pytest y con scans grabados (ver 2_laboratorio/grabar_scan.py).

Convencion (la misma que la clase Lidar de sigueme.py): angulos en rad en el marco del
robot, 0 = frente, + = izquierda; x = frente, y = izquierda.

Idea:
1. free_distance: para cada rumbo candidato, cuanto puede avanzar en linea recta un
   disco del radio del robot (+ margen) antes de tocar un punto del scan. Esto es el
   "inflado" de obstaculos por el radio del robot.
2. side_margin: holgura lateral a lo largo de ese rumbo (distancia al punto mas cercano
   a la trayectoria un poco mas adelante). Sirve para ir por el centro de un pasillo.
3. choose_heading: entre los rumbos con suficiente espacio, el que mejor combina ir a la
   meta, holgura amplia y no girar bruscamente respecto al rumbo anterior.
4. speed_command: lineal baja cerca del obstaculo y se anula bajo STOP_DIST; angular
   sigue al rumbo elegido.
5. Extras para la mision A->B->A (usados por navegacion.py): repulsion de paredes
   laterales, monitor de atasco, obstaculos virtuales (lo bajo que el LiDAR no ve),
   ICP 2D para corregir la deriva de odometria y eleccion de migas en el regreso.
"""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class NavParams:
    robot_radius: float = 0.17   # m   TB4 Lite (base Create 3, ~0.34 m de diametro)
    safety_margin: float = 0.09  # m   se suma al radio al inflar (choque = penalidad)
    lateral_margin: float = 0.06 # m   holgura LATERAL dura sobre el radio (al frente rige safety_margin)
    max_range: float = 3.0       # m   mas lejos que esto cuenta como "libre"
    n_headings: int = 73         # rumbos candidatos de -180 a +180 (cada 5 grados)
    min_free: float = 0.45       # m   un rumbo es admisible si deja avanzar al menos esto
    margin_from: float = 0.40    # m   la holgura lateral se mide desde aqui...
    margin_to: float = 1.20      # m   ...hasta aqui hacia adelante
    margin_cap: float = 0.40     # m   holgura por encima de esto ya no suma puntos
    w_goal: float = 1.0          # peso: desviarse de la meta (por rad)
    w_turn: float = 0.4          # peso: cambiar respecto al rumbo anterior (por rad)
    w_clear: float = 5.0         # peso: holgura lateral (por m, hasta margin_cap)
    stop_dist: float = 0.30      # m   libre al frente por debajo de esto -> lineal 0
    slow_dist: float = 0.90      # m   libre al frente por encima de esto -> lineal maxima
    turn_in_place: float = math.radians(60)  # rumbo mas abierto que esto -> girar quieto
    k_angular: float = 1.5       # rad/s por rad de error de rumbo
    scan_min: float = 0.12       # m   lecturas mas cercanas = el propio robot / ruido
    wall_min: float = 0.32       # m   distancia lateral (desde el centro) minima a paredes
    k_wall: float = 2.0          # rad/s por m que se invade wall_min
    near_dist: float = 0.45      # m   con algo mas cerca que esto se reduce la angular...
    near_angular: float = 0.5    # ...hasta este factor
    side_free: float = 0.60      # m   holgura lateral por encima de esto -> velocidad maxima
    side_tight: float = 0.28     # m   holgura lateral por debajo de esto -> side_min_scale
    side_min_scale: float = 0.5  # factor de la lineal con holgura lateral minima
    side_ahead: float = 0.80     # m   se mira la holgura lateral hasta esta distancia al frente

    @property
    def inflate(self):
        return self.robot_radius + self.safety_margin


def wrap(a):
    """Angulo(s) a (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


def clean_scan(ranges, angles, range_min=0.05, range_max=12.0):
    """Quita NaN/inf y lecturas fuera de rango. Devuelve (ranges, angles) como arrays."""
    r = np.asarray(ranges, float)
    a = np.asarray(angles, float)
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    return r[ok], wrap(a[ok])


def mask_sector(ranges, angles, center_deg, half_deg, rmax):
    """Descarta las lecturas dentro de +-half_deg de center_deg (marco del robot, 0 = frente)
    Y a menos de rmax m. Sirve para ignorar una pieza del propio robot que el LiDAR ve fija
    (soporte, tornillo, cable); un obstaculo real mas ancho o mas lejos se sigue viendo."""
    ranges, angles = np.asarray(ranges, float), np.asarray(angles, float)
    d = np.abs(wrap(angles - math.radians(center_deg)))
    ok = ~((d <= math.radians(half_deg)) & (ranges < rmax))
    return ranges[ok], angles[ok]


def headings(p=NavParams()):
    return np.linspace(-math.pi, math.pi, p.n_headings)


def _project(ranges, angles, bearings):
    """Coordenadas de cada punto en el marco de cada rumbo: (adelante, lateral), KxN."""
    d = angles[None, :] - np.asarray(bearings, float)[:, None]
    return ranges[None, :] * np.cos(d), ranges[None, :] * np.sin(d)


def free_distance(ranges, angles, bearings, p=NavParams()):
    """Distancia (m) que puede recorrer el disco inflado por cada rumbo, hasta max_range."""
    bearings = np.atleast_1d(bearings)
    if len(ranges) == 0:
        return np.full(len(bearings), p.max_range)
    fwd, lat = _project(ranges, angles, bearings)
    # Elipse: margen completo AL FRENTE (R, para frenar a tiempo) pero solo `side_margin_hard`
    # de holgura LATERAL (Rl). Con el disco inflado una pared paralela a < inflate (carton a 0.25 m,
    # inflate 0.31) bloqueaba todos los rumbos (free=0, sin opciones) aunque el robot cabia.
    # La holgura lateral "blanda" la dan side_margin (costo), wall_repulsion y side_scale.
    R = p.inflate
    Rl = min(R, p.robot_radius + p.lateral_margin)
    # solo puntos por delante: alejarse de algo que ya esta muy cerca debe estar permitido
    hit = (np.abs(lat) < Rl) & (fwd > 0)
    contact = fwd - R * np.sqrt(np.maximum(1.0 - (lat / Rl) ** 2, 0.0))
    contact = np.where(hit, contact, np.inf)
    return np.clip(contact.min(axis=1), 0.0, p.max_range)


def side_margin(ranges, angles, bearings, p=NavParams()):
    """Holgura lateral (m) por rumbo: distancia minima de la trayectoria a los puntos entre
    margin_from y margin_to por delante. Sin puntos ahi -> margin_cap."""
    bearings = np.atleast_1d(bearings)
    if len(ranges) == 0:
        return np.full(len(bearings), p.margin_cap)
    fwd, lat = _project(ranges, angles, bearings)
    near = (fwd > p.margin_from) & (fwd < p.margin_to)
    m = np.where(near, np.abs(lat), np.inf).min(axis=1)
    return np.minimum(m, p.margin_cap)


def choose_heading(ranges, angles, goal, prev=0.0, p=NavParams(), avoid_concavity=False):
    """Rumbo elegido (rad, marco del robot) o None si ningun rumbo es admisible.

    goal: rumbo hacia la meta en el marco del robot (rad).
    prev: rumbo elegido en el ciclo anterior (rad), para no zigzaguear.
    avoid_concavity: si es True, detecta trampas concavas y sesga el rumbo hacia la salida libre.
    """
    hs = headings(p)
    free = free_distance(ranges, angles, hs, p)
    ok = free >= p.min_free
    if not ok.any():
        return None

    effective_goal = goal
    if avoid_concavity:
        is_trap, escape_h = detect_concavity(ranges, angles, goal, p=p)
        if is_trap and escape_h is not None:
            effective_goal = escape_h

    margin = side_margin(ranges, angles, hs, p)
    cost = (p.w_goal * np.abs(wrap(hs - effective_goal))
            + p.w_turn * np.abs(wrap(hs - prev))
            - p.w_clear * margin)
    cost = np.where(ok, cost, np.inf)
    return float(hs[np.argmin(cost)])


def detect_concavity(ranges, angles, goal=0.0, trap_dist=0.75, p=NavParams()):
    """Detecta si el robot se aproxima a una trampa concava (callejon en U o frente cerrado).
    Devuelve (is_concave, escape_heading):
    - is_concave: True si el sector frontal hacia la meta esta bloqueado a menos de trap_dist.
    - escape_heading: Rumbo optimo para rodear o salir de la concavidad."""
    hs = headings(p)
    free = free_distance(ranges, angles, hs, p)
    goal_free = float(free_distance(ranges, angles, goal, p)[0])
    if goal_free >= trap_dist:
        return False, None

    err = np.abs(wrap(hs - goal))
    front_mask = err < math.radians(40)
    front_free = free[front_mask]
    if len(front_free) == 0 or np.any(front_free >= trap_dist):
        return False, None

    exit_mask = err > math.radians(60)
    exit_free = free[exit_mask]
    if len(exit_free) > 0 and np.max(exit_free) >= p.min_free:
        best_exit_idx = np.where(exit_mask)[0][np.argmax(exit_free)]
        return True, float(hs[best_exit_idx])
    return False, None


def most_open(ranges, angles, p=NavParams()):
    """Rumbo con mas distancia libre (para recuperarse cuando no hay rumbo admisible)."""
    hs = headings(p)
    return float(hs[np.argmax(free_distance(ranges, angles, hs, p))])


def speed_command(heading, front_free, max_linear, max_angular, p=NavParams()):
    """(lineal, angular) para seguir 'heading' con 'front_free' m libres al frente."""
    angular = float(np.clip(p.k_angular * heading, -max_angular, max_angular))
    if abs(heading) > p.turn_in_place:
        return 0.0, angular
    k = (front_free - p.stop_dist) / (p.slow_dist - p.stop_dist)
    cos_tip = math.cos(p.turn_in_place)
    turn_scale = max(0.0, (math.cos(heading) - cos_tip) / (1.0 - cos_tip))
    linear = max_linear * min(max(k, 0.0), 1.0) * turn_scale
    return float(linear), angular


def side_scale(r, a, p=NavParams()):
    """Factor (side_min_scale..1) para la velocidad lineal segun la holgura LATERAL minima de lo
    que esta a los lados y hasta side_ahead al frente (patas de silla, marcos, esquinas).
    Cuesta un par de operaciones numpy sobre el scan que ya se lee."""
    r, a = np.asarray(r), np.asarray(a)
    if r.size == 0:
        return 1.0
    x, y = r * np.cos(a), np.abs(r * np.sin(a))
    sel = (x > -0.05) & (x < p.side_ahead) & (y > 0.02)
    if not sel.any():
        return 1.0
    d = float(y[sel].min())
    k = (d - p.side_tight) / (p.side_free - p.side_tight)
    return float(p.side_min_scale + (1.0 - p.side_min_scale) * min(max(k, 0.0), 1.0))


def goal_bearing(pose, goal):
    """Rumbo (rad, marco del robot) y distancia (m) a la meta (x, y) en el marco odom.
    pose = (x, y, yaw) del robot en odom."""
    x, y, yaw = pose
    dx, dy = goal[0] - x, goal[1] - y
    return float(wrap(math.atan2(dy, dx) - yaw)), math.hypot(dx, dy)


# --- extras para la mision -------------------------------------------------------

def wall_repulsion(ranges, angles, p=NavParams()):
    """Correccion angular (rad/s) que aleja de paredes laterales mas cerca que wall_min.
    Solo mira puntos al costado (|x| < 0.4 m); dos paredes iguales se cancelan."""
    if len(ranges) == 0:
        return 0.0
    x, y = ranges * np.cos(angles), ranges * np.sin(angles)
    side = (np.abs(x) < 0.4) & (np.abs(y) > 0.05)
    left = y[side & (y > 0)]
    right = -y[side & (y < 0)]
    pen_l = max(p.wall_min - left.min(), 0.0) if len(left) else 0.0
    pen_r = max(p.wall_min - right.min(), 0.0) if len(right) else 0.0
    return float(p.k_wall * (pen_r - pen_l))


def nearest(ranges):
    return float(ranges.min()) if len(ranges) else math.inf


def angular_limit(max_angular, nearest_dist, p=NavParams()):
    """Angular maxima reducida cuando hay algo muy cerca (giros suaves junto a paredes)."""
    k = min(max(nearest_dist / p.near_dist, p.near_angular), 1.0)
    return max_angular * k


class ProgressMonitor:
    """Atasco = en los ultimos 'window' s el robot no se alejo 'min_progress' m del punto
    donde estaba al inicio de la ventana NI giro en neto 'min_turn' rad (girar 180 grados
    quieto para volver no es atasco; oscilar izquierda-derecha en el sitio si)."""

    def __init__(self, window=5.0, min_progress=0.08, min_turn=math.radians(60)):
        self.window, self.min_progress, self.min_turn = window, min_progress, min_turn
        self.hist = deque()

    def reset(self):
        self.hist.clear()

    def update(self, t, xy, yaw=None):
        giro = 0.0
        if yaw is not None and self.hist and self.hist[-1][2] is not None:
            giro = float(wrap(yaw - self.hist[-1][3]))
        acum = (self.hist[-1][2] if self.hist else 0.0) or 0.0
        self.hist.append((t, np.asarray(xy, float), acum + giro if yaw is not None else None,
                          yaw))
        while len(self.hist) > 1 and t - self.hist[1][0] >= self.window:
            self.hist.popleft()
        t0, p0, g0, _ = self.hist[0]
        if t - t0 < self.window:
            return False
        if g0 is not None and abs(self.hist[-1][2] - g0) >= self.min_turn:
            return False
        return max(np.linalg.norm(q[1] - p0) for q in self.hist) < self.min_progress


# --- poses 2D (x, y, yaw) -------------------------------------------------------

def compose(a, b):
    """Pose b (expresada en el marco de a) llevada al marco donde esta a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            float(wrap(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def relative(a, b):
    """Pose b vista desde la pose a."""
    return compose(inverse(a), b)


def to_xy(ranges, angles):
    return np.stack([ranges * np.cos(angles), ranges * np.sin(angles)], axis=1)


def to_polar(xy):
    xy = np.asarray(xy, float).reshape(-1, 2)
    return np.hypot(xy[:, 0], xy[:, 1]), np.arctan2(xy[:, 1], xy[:, 0])


class SpatialKDTree2D:
    """KD-Tree 2D puro en Python + NumPy para busqueda espacial rapida O(log N).
    Sin dependencias externas (no requiere scipy), ideal para ejecucion on-board en Pi 4."""

    class _Node:
        __slots__ = ("idx", "left", "right", "axis")

        def __init__(self, idx, left, right, axis):
            self.idx = idx
            self.left = left
            self.right = right
            self.axis = axis

    def __init__(self, points):
        self.points = np.asarray(points, float).reshape(-1, 2)
        if len(self.points) > 0:
            self.root = self._build(np.arange(len(self.points)), depth=0)
        else:
            self.root = None

    def _build(self, indices, depth):
        if len(indices) == 0:
            return None
        axis = depth % 2
        coords = self.points[indices, axis]
        order = np.argsort(coords)
        mid = len(indices) // 2
        sorted_indices = indices[order]
        node_idx = sorted_indices[mid]
        left_indices = sorted_indices[:mid]
        right_indices = sorted_indices[mid + 1:]
        return self._Node(
            node_idx,
            self._build(left_indices, depth + 1),
            self._build(right_indices, depth + 1),
            axis
        )

    def nearest(self, query):
        """Devuelve (idx, dist) del punto mas cercano al query (x, y)."""
        if self.root is None:
            return -1, float("inf")
        q = np.asarray(query, float)
        best_idx = [self.root.idx]
        best_d2 = [float(np.sum((self.points[self.root.idx] - q) ** 2))]

        def _search(node):
            if node is None:
                return
            p = self.points[node.idx]
            d2 = float(np.sum((p - q) ** 2))
            if d2 < best_d2[0]:
                best_d2[0] = d2
                best_idx[0] = node.idx
            axis = node.axis
            diff = q[axis] - p[axis]
            first = node.left if diff < 0 else node.right
            second = node.right if diff < 0 else node.left
            _search(first)
            if diff * diff < best_d2[0]:
                _search(second)

        _search(self.root)
        return best_idx[0], math.sqrt(best_d2[0])

    def query_radius(self, query, radius):
        """Devuelve indices de los puntos dentro de 'radius' de query (x, y)."""
        if self.root is None:
            return []
        q = np.asarray(query, float)
        r2 = radius * radius
        res = []

        def _search(node):
            if node is None:
                return
            p = self.points[node.idx]
            d2 = float(np.sum((p - q) ** 2))
            if d2 <= r2:
                res.append(node.idx)
            axis = node.axis
            diff = q[axis] - p[axis]
            if diff - radius <= 0:
                _search(node.left)
            if diff + radius >= 0:
                _search(node.right)

        _search(self.root)
        return res


class BreadcrumbGraph(list):
    """Grafo topologico de migas de pan con soporte para poda de callejones (dead-ends)
    y busqueda espacial eficiente. Es 100% compatible con una lista de tuplas [(x, y), ...]."""

    def __init__(self, iterable=None):
        super().__init__(iterable if iterable is not None else [(0.0, 0.0)])
        self.pruned_indices = set()
        self._kd = None

    def append(self, item):
        super().append(item)
        self._kd = None

    def prune_branch(self, start_idx, end_idx=None):
        """Poda el rango [start_idx, end_idx) de migas pertenecientes a ramas muertas o callejones."""
        if end_idx is None:
            end_idx = len(self)
        for i in range(max(1, start_idx), min(len(self), end_idx)):
            self.pruned_indices.add(i)

    def is_pruned(self, idx):
        return idx in self.pruned_indices

    @property
    def kd_tree(self):
        if self._kd is None and len(self) > 0:
            self._kd = SpatialKDTree2D(self)
        return self._kd

    def nearest_active(self, xy):
        """Encuentra el indice de la miga activa (no podada) mas cercana a xy."""
        if not self.pruned_indices and self.kd_tree is not None:
            return self.kd_tree.nearest(xy)
        pts = np.asarray(self, float)
        d2 = np.sum((pts - np.asarray(xy, float)) ** 2, axis=1)
        for idx in self.pruned_indices:
            if idx < len(d2):
                d2[idx] = np.inf
        best = int(np.argmin(d2))
        return best, math.sqrt(d2[best])


class VirtualObstacles:
    """Obstaculos que el LiDAR no ve (bajos): se marcan donde el robot se trabo empujando
    hacia adelante y se agregan al scan como puntos fijos en el marco de A."""

    def __init__(self):
        self.points = np.empty((0, 2))
        self.times = np.empty(0, float)

    def add_front(self, pose, dist, half_width, step=0.04, dedup_radius=0.03, t=None):
        ys = np.arange(-half_width, half_width + 1e-9, step)
        local = np.stack([np.full_like(ys, dist), ys], axis=1)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = local @ np.array([[c, s], [-s, c]]) + np.array(pose[:2])
        stamp = np.nan if t is None else float(t)
        if len(self.points) > 0 and dedup_radius > 0:
            kd = SpatialKDTree2D(self.points)
            new_pts = [pt for pt in world if kd.nearest(pt)[1] > dedup_radius]
            if new_pts:
                arr = np.array(new_pts)
                self.points = np.vstack([self.points, arr])
                self.times = np.r_[self.times, np.full(len(arr), stamp)]
        else:
            self.points = np.vstack([self.points, world])
            self.times = np.r_[self.times, np.full(len(world), stamp)]

    def prune_expired(self, t, ttl):
        """Elimina puntos virtuales con marca de tiempo mas vieja que ttl segundos."""
        if not len(self.points) or ttl is None or ttl <= 0:
            return
        keep = np.isnan(self.times) | ((t - self.times) <= ttl)
        if not np.all(keep):
            self.points = self.points[keep]
            self.times = self.times[keep]

    def clear_near(self, xy, radius):
        """Elimina puntos virtuales dentro de `radius` metros de `xy`."""
        if not len(self.points):
            return
        d = np.hypot(self.points[:, 0] - xy[0], self.points[:, 1] - xy[1])
        keep = d > radius
        if not np.all(keep):
            self.points = self.points[keep]
            self.times = self.times[keep]

    def in_robot(self, pose, max_r=3.0):
        """(ranges, angles) de los puntos virtuales vistos desde pose."""
        if not len(self.points):
            return np.empty(0), np.empty(0)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        d = self.points - np.array(pose[:2])
        local = d @ np.array([[c, -s], [s, c]])
        r, a = to_polar(local)
        ok = r < max_r
        return r[ok], a[ok]


def _nearest(pts, ref, ref2):
    """Vecino mas cercano de cada punto de pts en ref: (indices, distancias). Usa
    |a-b|^2 = |a|^2 + |b|^2 - 2a.b (una multiplicacion de matrices) en vez de armar el
    tensor N x N x 2: mismo resultado, ~10x mas rapido y sin memoria temporal grande."""
    d2 = (pts ** 2).sum(1)[:, None] + ref2[None, :] - 2.0 * (pts @ ref.T)
    j = d2.argmin(1)
    return j, np.sqrt(np.maximum(d2[np.arange(len(pts)), j], 0.0))


def icp_2d(ref, cur, init=(0.0, 0.0, 0.0), iters=40, max_pair=0.3, tol=1e-4):
    """ICP punto a punto. ref, cur: (N, 2) en m. Devuelve (pose, inliers, rmse) donde pose
    es la del marco 'cur' dentro del marco 'ref' (ref ~ R*cur + t), o None si no encaja."""
    ref, cur = np.asarray(ref, float), np.asarray(cur, float)
    if len(ref) < 10 or len(cur) < 10:
        return None
    ref2 = (ref ** 2).sum(1)
    x, y, th = init
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        moved = cur @ np.array([[c, s], [-s, c]]) + (x, y)
        j, dist = _nearest(moved, ref, ref2)
        m = dist < max_pair
        if m.sum() < 10:
            return None
        P, Q = cur[m], ref[j[m]]
        pc, qc = P.mean(0), Q.mean(0)
        H = (P - pc).T @ (Q - qc)
        th_new = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        c, s = math.cos(th_new), math.sin(th_new)
        t = qc - np.array([[c, -s], [s, c]]) @ pc
        done = abs(th_new - th) < tol and math.hypot(t[0] - x, t[1] - y) < tol
        x, y, th = float(t[0]), float(t[1]), th_new
        if done:
            break
    c, s = math.cos(th), math.sin(th)
    moved = cur @ np.array([[c, s], [-s, c]]) + (x, y)
    dist = _nearest(moved, ref, ref2)[1]
    inl = dist < max_pair / 3
    rmse = float(np.sqrt(np.mean(dist[inl] ** 2))) if inl.any() else math.inf
    return (x, y, float(wrap(th))), float(inl.mean()), rmse


def subsample(xy, n=240):
    if len(xy) <= n:
        return xy
    return xy[np.linspace(0, len(xy) - 1, n).astype(int)]


def pick_waypoint(crumbs, pose, ranges, angles, upto, lookahead=2.5, p=NavParams()):
    """Regreso por las migas de la ida. crumbs[0] = A. Devuelve el indice de la miga mas
    cercana a A (<= upto) que se ve libre en linea recta desde pose; si ninguna, la miga
    anterior a la mas cercana. Asi se recorta camino sin salir del trayecto conocido."""
    pts = np.asarray(crumbs[:upto + 1], float)
    d = np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])
    bear = wrap(np.arctan2(pts[:, 1] - pose[1], pts[:, 0] - pose[0]) - pose[2])
    cand = np.where(d <= lookahead)[0]

    pruned = getattr(crumbs, "pruned_indices", None)
    if pruned:
        cand = np.array([c for c in cand if c not in pruned], dtype=int)

    if len(cand):
        free = free_distance(ranges, angles, bear[cand], p)
        ok = cand[free >= np.minimum(d[cand], p.max_range) - 1e-6]
        if len(ok):
            return int(ok.min())

    if pruned:
        for p_idx in pruned:
            if p_idx < len(d):
                d[p_idx] = np.inf
    return max(int(np.argmin(d)) - 1, 0)
