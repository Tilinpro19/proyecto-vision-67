"""
Exploracion autonoma por fronteras: recorre toda el area alcanzable y vuelve a A.

Construye una malla de ocupacion con el LiDAR (marco de A, odometria), busca "fronteras"
(celdas libres vecinas de lo desconocido), planifica (Dijkstra sobre celdas libres e infladas)
y le da a Piloto (navegacion.py) un punto a ~1 m por el camino; Piloto aporta la evasion
reactiva, el escape y el regreso por migas. Termina cuando no queda ninguna frontera
alcanzable, al agotar --max-min o con Ctrl+C, vuelve a A y guarda mapa PNG + resumen JSON.

POR DEFECTO NO MUEVE EL ROBOT (--dry): solo calcula y muestra. Para moverlo: --mover.

  python3 explorar.py                    # prueba sin mover (20 s): ve el mapa y la frontera elegida
  python3 explorar.py --mover --max-min 6 --radio-max 6
"""
import argparse
import heapq
import json
import math
import os
import time
from dataclasses import dataclass

import numpy as np

import nav_core as nc
from navegacion import Piloto

L_FREE, L_OCC, L_MIN, L_MAX = 0.4, 0.85, -3.0, 3.0   # log-odds por scan
FREE_T, OCC_T = -0.3, 0.3
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
INF = 1e9


@dataclass
class ExplorarParams:
    res: float = 0.10          # m/celda
    size: float = 50.0         # m, lado de la malla (centrada en A)
    max_range: float = 4.0     # m, mas lejos que esto el LiDAR no marca obstaculos
    soft: float = 0.45         # m, cerca de paredes (mas que 'hard') el camino cuesta mas
    soft_cost: float = 2.0
    lookahead: float = 1.0     # m, punto del camino que se le da a Piloto
    replan_s: float = 1.0      # s entre replanificaciones
    min_cluster: int = 3       # celdas (0.3 m): fronteras menores son ruido / rendijas
    blacklist_r: float = 0.6   # m, radio descartado alrededor de una meta fallida
    reach_tol: float = 0.35    # m, "llegue" a la frontera
    stall_s: float = 25.0      # s sin acercarse a la meta -> descartarla
    stall_dist: float = 0.15   # m de avance que cuenta como progreso
    done_confirm: int = 3      # replanes seguidos sin fronteras para dar por terminado
    radio_max: float = 12.0    # m desde A: valla virtual (no se planifica mas lejos)
    map_every: float = 0.25    # s entre actualizaciones del mapa
    bat_min_inicio: float = 0.30   # carga minima para aceptar --mover
    bat_abort: float = 0.22        # umbral de bateria para abortar y volver a A
    bat_hysteresis_s: float = 5.0  # s consecutivos bajo bat_abort antes de abortar
    mapa_vivo_s: float = 5.0       # s entre escrituras de --mapa-vivo


class MonitorBateria:
    """Histeresis de bateria: exige `hysteresis_s` seguidos bajo `umbral` antes de abortar."""

    def __init__(self, umbral=0.22, hysteresis_s=5.0):
        self.umbral = umbral
        self.hysteresis_s = hysteresis_s
        self._t_baja = None

    def update(self, t, pct):
        if pct is None or pct >= self.umbral:
            self._t_baja = None
            return False
        if self._t_baja is None:
            self._t_baja = t
            return False
        return (t - self._t_baja) >= self.hysteresis_s


def _shift_or(out, m, dy, dx):
    H, W = m.shape
    out[max(0, dy):H - max(0, -dy), max(0, dx):W - max(0, -dx)] |= \
        m[max(0, -dy):H - max(0, dy), max(0, -dx):W - max(0, dx)]


def dilatar(m, k):
    """Dilata la mascara booleana m con un disco de k celdas."""
    out = m.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if (dy or dx) and dy * dy + dx * dx <= k * k + 0.5:
                _shift_or(out, m, dy, dx)
    return out


class Mapa:
    """Malla de ocupacion log-odds en el marco de A. Fila = y, columna = x."""

    def __init__(self, ep):
        self.ep = ep
        self.n = int(ep.size / ep.res)
        self.c = self.n // 2
        self.lo = np.zeros((self.n, self.n), np.float32)
        steps = np.arange(0.0, ep.max_range, ep.res * 0.5)
        self.steps = steps

    def celda(self, x, y):
        return int(round(y / self.ep.res)) + self.c, int(round(x / self.ep.res)) + self.c

    def mundo(self, row, col):
        return (col - self.c) * self.ep.res, (row - self.c) * self.ep.res

    def actualizar(self, pose, r, a):
        ep, n = self.ep, self.n
        if len(r) == 0:
            return
        x, y, th = pose
        ang = np.asarray(a) + th
        rr = np.minimum(np.asarray(r), ep.max_range)
        hit = np.asarray(r) < ep.max_range - 0.05
        m = self.steps[None, :] < (rr[:, None] - ep.res * 0.5)
        cols = np.rint((x + self.steps[None, :] * np.cos(ang)[:, None]) / ep.res).astype(int) + self.c
        rows = np.rint((y + self.steps[None, :] * np.sin(ang)[:, None]) / ep.res).astype(int) + self.c
        ok = m & (cols >= 0) & (cols < n) & (rows >= 0) & (rows < n)
        free = np.unique(rows[ok] * n + cols[ok])
        self.lo.flat[free] = np.maximum(self.lo.flat[free] - L_FREE, L_MIN)
        ex = np.rint((x + rr[hit] * np.cos(ang[hit])) / ep.res).astype(int) + self.c
        ey = np.rint((y + rr[hit] * np.sin(ang[hit])) / ep.res).astype(int) + self.c
        ok = (ex >= 0) & (ex < n) & (ey >= 0) & (ey < n)
        occ = np.unique(ey[ok] * n + ex[ok])
        self.lo.flat[occ] = np.minimum(self.lo.flat[occ] + L_OCC, L_MAX)
        np.clip(self.lo, L_MIN, L_MAX, out=self.lo)

    def libre_en(self, x, y):
        r, c = self.celda(x, y)
        return 0 <= r < self.n and 0 <= c < self.n and self.lo[r, c] < FREE_T

    def area_libre(self):
        return float((self.lo < FREE_T).sum()) * self.ep.res ** 2


class Explorador:
    """Misma interfaz que Piloto: paso(t, odom, ranges, angles) -> (lin, ang, info)."""

    def __init__(self, piloto=None, ep=None, volver=True):
        self.piloto = piloto or Piloto()
        self.ep = ep or ExplorarParams()
        self.mapa = Mapa(self.ep)
        self.volver_a_A = volver
        self.estado = "explora"          # explora | vuelta | fin
        self.target = None
        self.path = []
        self.t_plan = -1e9
        self.t_map = -1e9
        self.sin_frontera = 0
        self.blacklist = []              # [(x, y)]
        self.visitas = {}
        self.best_d = INF
        self.t_best = 0.0
        self.inalcanzables = []          # centros de fronteras sin camino
        self.razon_fin = ""
        self.trayecto = []
        self.eventos = self.piloto.eventos
        self.t0 = None

    # --- API -------------------------------------------------------------------
    def iniciar(self, odom_pose, ranges, angles):
        self.piloto.fijar_origen(odom_pose, ranges, angles)
        r, a = nc.clean_scan(ranges, angles, self.piloto.nav.scan_min)
        self.mapa.actualizar((0.0, 0.0, 0.0), r, a)

    def terminar(self, razon):
        """Detiene la exploracion y vuelve a A (o termina)."""
        if self.estado != "explora":
            return
        self.razon_fin = razon
        self._log(self.t_plan, f"exploracion terminada: {razon}")
        if self.volver_a_A:
            self.piloto.volver()
            self.estado = "vuelta"
        else:
            self.estado = "fin"

    def paso(self, t, odom_pose, ranges, angles, bumper_hit=False):
        pl, ep = self.piloto, self.ep
        if self.t0 is None:
            self.t0 = t
        if self.estado == "fin":
            return 0.0, 0.0, dict(pose=pl.pose(odom_pose), dist=0.0, front=9.0, nearest=9.0,
                                  estado="fin", llegada=True)
        if self.estado == "vuelta":
            lin, ang, info = pl.paso(t, odom_pose, ranges, angles, bumper_hit)
            info["estado"] = "vuelta"
            if info["llegada"]:
                self.estado = "fin"
            return lin, ang, info

        pose = pl.pose(odom_pose)
        if t - self.t_map >= ep.map_every:
            self.t_map = t
            r, a = nc.clean_scan(ranges, angles, pl.nav.scan_min)
            self.mapa.actualizar(pose, r, a)
            self.trayecto.append(pose[:2])
        if t - self.t_plan >= ep.replan_s:
            self.t_plan = t
            self._replanificar(t, pose)
            if self.estado != "explora":
                return self.paso(t, odom_pose, ranges, angles, bumper_hit)

        if self.target is None:
            # esperando confirmar que no quedan fronteras: parado
            pl._reset_motion()
            return 0.0, 0.0, dict(pose=pose, dist=0.0, front=9.0, nearest=9.0,
                                  estado="espera", llegada=False)
        carrot = self._zanahoria(pose)
        if pl.meta != carrot:
            pl.meta = carrot
        lin, ang, info = pl.paso(t, odom_pose, ranges, angles, bumper_hit)
        if info.get("atasco_repetido") and self.target is not None:
            self._log(t, f"atasco repetido hacia ({self.target[0]:+.2f}, {self.target[1]:+.2f}): se descarta")
            self.blacklist.append(self.target)
            self.target, self.path = None, []
            self.best_d, self.t_best = INF, t
            self.t_plan = -INF
        info["estado"] = "explora" if info["estado"] == "navega" else info["estado"]
        info["target"] = self.target
        info["dist"] = math.hypot(self.target[0] - pose[0], self.target[1] - pose[1]) if self.target else 0.0
        return lin, ang, info

    # --- planificacion ---------------------------------------------------------
    def _log(self, t, texto):
        self.piloto.eventos.append((t, texto))

    def _descartada(self, x, y):
        r2 = self.ep.blacklist_r ** 2
        return any((x - bx) ** 2 + (y - by) ** 2 < r2 for bx, by in self.blacklist)

    def _replanificar(self, t, pose):
        ep, mp = self.ep, self.mapa
        # llegada a la meta
        if self.target is not None and math.hypot(self.target[0] - pose[0],
                                                  self.target[1] - pose[1]) < ep.reach_tol:
            k = (round(self.target[0] / 0.5), round(self.target[1] / 0.5))
            self.visitas[k] = self.visitas.get(k, 0) + 1
            if self.visitas[k] >= 2:
                self.blacklist.append(self.target)
            self.target = None
        res = self._planear(pose)
        if res is None:
            self.target, self.path = None, []
            self.sin_frontera += 1
            if self.sin_frontera >= ep.done_confirm:
                self.terminar("no quedan fronteras alcanzables")
            return
        self.sin_frontera = 0
        target, path, dist = res
        if self.target is None or math.hypot(target[0] - self.target[0], target[1] - self.target[1]) > 0.5:
            self._log(t, f"nueva frontera ({target[0]:+.2f}, {target[1]:+.2f}) a {dist:.1f} m "
                         f"| libre {mp.area_libre():.1f} m2")
            self.best_d, self.t_best = dist, t
        self.target, self.path = target, path
        # sin progreso hacia la meta -> descartarla
        if dist < self.best_d - ep.stall_dist:
            self.best_d, self.t_best = dist, t
        elif t - self.t_best > ep.stall_s:
            self._log(t, f"sin progreso hacia ({target[0]:+.2f}, {target[1]:+.2f}): se descarta")
            self.blacklist.append(target)
            self.target, self.path = None, []
            self.best_d, self.t_best = INF, t

    def _zanahoria(self, pose):
        path = self.path
        if not path:
            return self.target
        i = min(range(len(path)), key=lambda k: (path[k][0] - pose[0]) ** 2 + (path[k][1] - pose[1]) ** 2)
        acc, prev = 0.0, path[i]
        for p in path[i + 1:]:
            acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
            if acc >= self.ep.lookahead:
                return p
        return path[-1]

    def _planear(self, pose):
        """Devuelve (meta_xy, camino_xy, distancia) a la mejor frontera alcanzable, o None."""
        ep, mp, nav = self.ep, self.mapa, self.piloto.nav
        r_row, r_col = mp.celda(pose[0], pose[1])
        nz_r, nz_c = np.nonzero(mp.lo)
        if len(nz_r) == 0:
            return None
        pad = 10
        r0 = max(min(nz_r.min(), r_row) - pad, 0)
        r1 = min(max(nz_r.max(), r_row) + pad + 1, mp.n)
        c0 = max(min(nz_c.min(), r_col) - pad, 0)
        c1 = min(max(nz_c.max(), r_col) + pad + 1, mp.n)
        sub = mp.lo[r0:r1, c0:c1]
        H, W = sub.shape
        occ, free = sub > OCC_T, sub < FREE_T
        # Incluye obstaculos virtuales (bajos/trampas) en la planificacion global
        vpts = self.piloto.virtual.points
        if len(vpts):
            for vx, vy in vpts:
                vr, vc = mp.celda(vx, vy)
                if r0 <= vr < r1 and c0 <= vc < c1:
                    occ[vr - r0, vc - c0] = True
                    free[vr - r0, vc - c0] = False
        unk = ~(occ | free)
        hard = dilatar(occ, int(math.ceil((nav.inflate + 0.01) / ep.res)))
        soft = dilatar(occ, int(math.ceil(ep.soft / ep.res))) & ~hard
        trav = free & ~hard
        # valla virtual
        yy, xx = np.mgrid[r0:r1, c0:c1]
        trav &= ((xx - mp.c) ** 2 + (yy - mp.c) ** 2) * ep.res ** 2 <= ep.radio_max ** 2
        si_r, si_c = r_row - r0, r_col - c0
        k = int(math.ceil(nav.inflate / ep.res))
        trav[max(si_r - k, 0):si_r + k + 1, max(si_c - k, 0):si_c + k + 1] |= \
            free[max(si_r - k, 0):si_r + k + 1, max(si_c - k, 0):si_c + k + 1]
        trav[si_r, si_c] = True

        front = np.zeros_like(trav)
        un = np.zeros_like(trav)
        un[1:, :] |= unk[:-1, :]
        un[:-1, :] |= unk[1:, :]
        un[:, 1:] |= unk[:, :-1]
        un[:, :-1] |= unk[:, 1:]
        front = trav & un
        front[0, :] = front[-1, :] = False
        front[:, 0] = front[:, -1] = False

        # Dijkstra desde el robot
        travf = trav.ravel().tolist()
        costf = (1.0 + ep.soft_cost * soft.ravel()).tolist()
        dist = [INF] * (H * W)
        par = [-1] * (H * W)
        s = si_r * W + si_c
        dist[s] = 0.0
        heap = [(0.0, s)]
        res = ep.res
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue
            uy, ux = divmod(u, W)
            for dy, dx, w in MOVES:
                vy, vx = uy + dy, ux + dx
                if vy < 0 or vy >= H or vx < 0 or vx >= W:
                    continue
                v = vy * W + vx
                if not travf[v]:
                    continue
                if dy and dx and not (travf[uy * W + vx] and travf[vy * W + ux]):
                    continue
                nd = d + w * res * costf[v]
                if nd < dist[v]:
                    dist[v] = nd
                    par[v] = u
                    heapq.heappush(heap, (nd, v))

        # clusters de frontera (8-conexos)
        cells = set(map(int, np.flatnonzero(front.ravel())))
        best = None
        prev = None
        self.inalcanzables = []
        while cells:
            seed = cells.pop()
            comp, stack = [seed], [seed]
            while stack:
                u = stack.pop()
                uy, ux = divmod(u, W)
                for dy, dx, _ in MOVES:
                    v = (uy + dy) * W + ux + dx
                    if v in cells:
                        cells.discard(v)
                        comp.append(v)
                        stack.append(v)
            if len(comp) < ep.min_cluster:
                continue
            alc = [u for u in comp if dist[u] < INF]
            cy, cx = divmod(comp[0], W)
            cxy = mp.mundo(cy + r0, cx + c0)
            if not alc:
                self.inalcanzables.append((round(cxy[0], 2), round(cxy[1], 2), len(comp)))
                continue
            u = min(alc, key=lambda q: dist[q])
            uy, ux = divmod(u, W)
            xy = mp.mundo(uy + r0, ux + c0)
            if self._descartada(*xy):
                continue
            score = dist[u] - 0.03 * len(comp)
            cand = (score, u, xy, len(comp))
            if best is None or score < best[0]:
                best = cand
            if self.target is not None and math.hypot(xy[0] - self.target[0], xy[1] - self.target[1]) < 0.5:
                prev = cand
        if best is None:
            return None
        if prev is not None and prev[0] <= best[0] + 1.0:
            best = prev           # histeresis: no cambiar de meta por poco
        _, u, xy, _ = best
        chain = []
        while u != -1:
            chain.append(u)
            u = par[u]
        chain.reverse()
        path = [mp.mundo(q // W + r0, q % W + c0) for q in chain]
        return xy, path, dist[chain[-1]]

    # --- salidas ---------------------------------------------------------------
    def resumen(self, t=None):
        return dict(area_libre_m2=round(self.mapa.area_libre(), 1),
                    razon_fin=self.razon_fin or "en curso",
                    metas_descartadas=[(round(x, 2), round(y, 2)) for x, y in self.blacklist],
                    fronteras_sin_camino=self.inalcanzables,
                    segundos=None if t is None or self.t0 is None else round(t - self.t0, 1))

    def guardar(self, prefijo, pose=None):
        """Escribe <prefijo>.json y <prefijo>.png (si hay cv2)."""
        os.makedirs(os.path.dirname(os.path.abspath(prefijo)), exist_ok=True)
        with open(prefijo + ".json", "w", encoding="utf-8") as f:
            json.dump(self.resumen(), f, indent=1, ensure_ascii=False)
        try:
            import cv2
        except ImportError:
            return
        mp = self.mapa
        nzr, nzc = np.nonzero(mp.lo)
        if len(nzr) == 0:
            return
        r0, r1, c0, c1 = nzr.min() - 5, nzr.max() + 6, nzc.min() - 5, nzc.max() + 6
        sub = mp.lo[r0:r1, c0:c1]
        img = np.full(sub.shape + (3,), 127, np.uint8)
        img[sub < FREE_T] = 255
        img[sub > OCC_T] = 0
        for x, y in self.trayecto:
            rr, cc = mp.celda(x, y)
            if r0 <= rr < r1 and c0 <= cc < c1:
                img[rr - r0, cc - c0] = (255, 120, 0)   # BGR: azul
        for x, y in self.blacklist:
            rr, cc = mp.celda(x, y)
            if r0 <= rr < r1 and c0 <= cc < c1:
                img[rr - r0, cc - c0] = (0, 0, 255)
        img = cv2.resize(img[::-1], None, fx=5, fy=5, interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(prefijo + ".png", img)


# --- ROS ---------------------------------------------------------------------------

def main():
    import rclpy

    from comun import Sender, apply_params
    from navegacion import Lidar, Odom, add_ros_args, wait_sensors

    ap = argparse.ArgumentParser(description="Exploracion autonoma por fronteras (no mueve el robot sin --mover)")
    ap.add_argument("--mover", action="store_true", help="publica /cmd_vel (SIN esto solo calcula)")
    ap.add_argument("--max-min", type=float, default=8.0, help="minutos maximos de exploracion")
    ap.add_argument("--radio-max", type=float, default=12.0, help="valla virtual: m maximos desde A")
    ap.add_argument("--no-volver", action="store_true", help="no regresar a A al terminar")
    ap.add_argument("--dry-seg", type=float, default=20.0, help="sin --mover: s que corre la prueba")
    ap.add_argument("--salida", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
    ap.add_argument("--mapa-vivo", action="store_true", help="actualiza logs/mapa_vivo.png cada 2 s mientras explora")
    ap.add_argument("--rate", type=float, default=10.0)
    add_ros_args(ap)
    args = ap.parse_args()
    args.dry = args.dry or not args.mover

    piloto = Piloto()
    ep = ExplorarParams(radio_max=args.radio_max)
    apply_params([piloto.nav, piloto.pp, ep], args.config, args.param)
    exp = Explorador(piloto, ep, volver=not args.no_volver)

    rclpy.init()
    node = rclpy.create_node("explorar")
    sender = None
    try:
        lidar, odom = Lidar(node, args.scan, args.lidar_rot, args.lidar_mask), Odom(node, args.odom)
        sender = Sender(node, args.cmd_vel, args.dry)
        bump_t = 0.0
        battery_pct = None
        try:
            from irobot_create_msgs.msg import HazardDetectionVector
            from rclpy.qos import qos_profile_sensor_data

            def _on_hazard(msg):
                nonlocal bump_t
                if any(d.type == 1 for d in msg.detections):
                    bump_t = time.monotonic()

            node.create_subscription(HazardDetectionVector, "/hazard_detection", _on_hazard, qos_profile_sensor_data)
        except ImportError:
            pass
        try:
            from rclpy.qos import qos_profile_sensor_data as _qos_bat
            from sensor_msgs.msg import BatteryState

            def _on_battery(msg):
                nonlocal battery_pct
                if 0.0 <= msg.percentage <= 1.0:
                    battery_pct = msg.percentage

            node.create_subscription(BatteryState, "/battery_state", _on_battery, _qos_bat)
        except ImportError:
            pass
        if not wait_sensors(node, lidar, odom):
            raise SystemExit("No llegan /scan y /odom: revisa la conexion (tb4_check.sh).")
        if args.mover and battery_pct is None:
            t_wait = time.monotonic() + 1.5
            while battery_pct is None and time.monotonic() < t_wait:
                rclpy.spin_once(node, timeout_sec=0.1)
        if args.mover and battery_pct is not None and battery_pct < ep.bat_min_inicio:
            raise SystemExit(
                f"Bateria demasiado baja ({battery_pct * 100:.0f}% < {ep.bat_min_inicio * 100:.0f}%): "
                "recarga antes de usar --mover."
            )
        exp.iniciar(odom.pose, *lidar.data)
        modo = "MOVIENDO el robot" if args.mover else "SIN MOVER (prueba)"
        print(f"Exploracion {modo}: max {args.max_min:g} min, radio {args.radio_max:g} m, "
              f"{'con' if exp.volver_a_A else 'sin'} regreso a A. Ctrl+C = parar y volver.")
        t0 = time.monotonic()
        last_log = 0.0
        last_mapa_vivo = 0.0
        bat_mon = MonitorBateria(ep.bat_abort, ep.bat_hysteresis_s)
        limite = args.max_min * 60.0 if args.mover else args.dry_seg
        while exp.estado != "fin":
            fin_ciclo = time.monotonic() + 1.0 / args.rate
            while True:
                rclpy.spin_once(node, timeout_sec=max(fin_ciclo - time.monotonic(), 0.0))
                if time.monotonic() >= fin_ciclo:
                    break
            t = time.monotonic()
            if not (lidar.fresh() and odom.fresh()):
                sender.send(0.0, 0.0)
                continue
            if exp.estado == "explora" and args.mover and bat_mon.update(t, battery_pct):
                print(f"[{t - t0:5.1f}s] BATERIA BAJA ({battery_pct * 100:.0f}%): terminando y volviendo a A")
                exp.terminar("bateria baja")
            if exp.estado == "explora" and t - t0 > limite:
                exp.terminar("tiempo agotado" if args.mover else "fin de la prueba sin mover")
                if not args.mover:
                    exp.estado = "fin"
            if exp.estado == "fin":
                break
            lin, ang, info = exp.paso(t, odom.pose, *lidar.data, bumper_hit=(t - bump_t < 0.3))
            if args.mapa_vivo and t - last_mapa_vivo >= ep.mapa_vivo_s:
                last_mapa_vivo = t
                exp.guardar(os.path.join(args.salida, "mapa_vivo"))
            if t - last_log >= 2.0:
                last_log = t
                tg = info.get("target")
                print(f"[{t - t0:5.0f}s] {info['estado']:7s} libre {exp.mapa.area_libre():5.1f} m2 | "
                      f"pose ({info['pose'][0]:+.2f},{info['pose'][1]:+.2f}) | "
                      f"meta {('(%+.2f,%+.2f)' % tg) if tg else '-'} | frente {info['front']:.2f} m | "
                      f"v {lin:+.2f} w {ang:+.2f}")
            for te, txt in piloto.eventos:
                print(f"[{te - t0:6.1f}s] {txt}")
            piloto.eventos.clear()
            sender.send(lin, ang)
    except KeyboardInterrupt:
        print("\nCtrl+C: se detiene.")
    finally:
        if sender is not None:
            sender.stop()
        ruta = os.path.join(args.salida, time.strftime("exploracion_%Y%m%d_%H%M%S"))
        exp.guardar(ruta)
        print("Resumen:", json.dumps(exp.resumen(), ensure_ascii=False))
        print("Mapa guardado en", ruta + ".png / .json")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
