"""
Navegacion reactiva de la Tarea A, sin mapa: /scan + /odom + nav_core.py.

- Piloto: logica pura (sin ROS) de ida a B y regreso a A. Se prueba en tests/ con un
  simulador cinematico. Todo en el marco de A: x = frente del robot en A, y = izquierda.
  * Ida: VFH-lite hacia B (nav_core.choose_heading) con inflado radio + margen, frenado
    progresivo, repulsion de paredes laterales y angular reducida cerca de obstaculos.
    Guarda migas (camino recorrido) y keyframes (scan + pose) para el regreso.
  * Regreso: sigue las migas al reves recortando cuando hay linea libre
    (nav_core.pick_waypoint) y corrige la deriva de odometria con ICP contra los
    keyframes de la ida.
  * Anti-atasco: sin avance en stuck_time s -> retrocede, gira al rumbo mas abierto y
    reintenta. Si se trabo empujando sin ver nada al frente (obstaculo bajo que el LiDAR
    no ve) deja un obstaculo virtual en ese punto.
- Robot / Lidar / Odom: envoltorio ROS (rclpy se importa solo al usarlos).

Uso suelto para probar en el laboratorio (mision.py es el que corre la competencia):
  python3 navegacion.py --goal 2.5 0.0 --dry      # solo mira sensores, no publica
  python3 navegacion.py --goal 2.5 0.0 --pausa 3  # A -> B, espera 3 s, B -> A
  python3 navegacion.py --goal 2.5 0.5 --param max_linear=0.15 --param safety_margin=0.1
"""
import argparse
import math
import time
from dataclasses import dataclass, field

import numpy as np

import nav_core as nc


@dataclass
class PilotoParams:
    max_linear: float = 0.20      # m/s   (el Create 3 llega a 0.31)
    max_angular: float = 1.2      # rad/s
    max_accel: float = 0.4        # m/s^2 al acelerar
    max_decel: float = 0.8        # m/s^2 al frenar suave (inmediato si peligro < stop_dist)
    max_alpha: float = 2.5        # rad/s^2 aceleracion angular maxima
    goal_tol: float = 0.25        # m     llegada a B por distancia (por defecto)
    home_tol: float = 0.20        # m     llegada a A
    stuck_time: float = 5.0       # s     sin avanzar stuck_dist -> maniobra de salida
    stuck_dist: float = 0.08      # m
    stuck_wait: float = 15.0      # s     sin avanzar y SIN empujar (frena a proposito) -> salida sin barreras
    back_dist: float = 0.15       # m     retroceso de la maniobra
    back_speed: float = 0.08      # m/s
    escape_turn: float = 2.0      # s     maximo girando hacia lo mas abierto
    blind_front: float = 0.40     # m     trabado con mas libre que esto al frente = bajo
    crumb_every: float = 0.20     # m     separacion entre migas
    keyframe_every: float = 0.8   # m     separacion entre keyframes para ICP
    lookahead: float = 2.5        # m     recorte de camino en el regreso
    icp_radius: float = 0.5       # m     correr ICP al pasar a menos de esto de un keyframe
    icp_min_inliers: float = 0.6
    icp_max_rmse: float = 0.05    # m
    icp_max_jump: float = 0.35    # m     correccion mas grande que esto = ICP equivocado
    virtual_ttl: float = 45.0     # s     caducidad de barreras virtuales de escape


@dataclass
class Keyframe:
    pose: tuple
    xy: np.ndarray
    used: bool = False
    attempts: int = 0


@dataclass
class Piloto:
    nav: nc.NavParams = field(default_factory=nc.NavParams)
    pp: PilotoParams = field(default_factory=PilotoParams)

    def __post_init__(self):
        self.origin = None                 # pose de A en /odom
        self.corr = (0.0, 0.0, 0.0)        # correccion de deriva: pose = corr o raw
        self.meta = (0.0, 0.0)
        self.retorno = False
        self.crumbs = nc.BreadcrumbGraph()
        self.keyframes = []
        self.virtual = nc.VirtualObstacles()
        self.monitor = nc.ProgressMonitor(self.pp.stuck_time, self.pp.stuck_dist, self.pp.stuck_wait)
        self.prev_heading = 0.0
        self.last_t = None
        self.lin = 0.0
        self.ang = 0.0
        self.escape = None                 # (fase, t_inicio, pose_inicio, rumbo)
        self.upto = 0
        self.eventos = []                  # (t, texto) para el registro de la mision
        self._last_escape_xy = None
        self._repeat_escapes = 0

    # --- API -------------------------------------------------------------------
    def fijar_origen(self, odom_pose, ranges, angles):
        """Llamar parado en A: guarda la pose de A y su scan (keyframe 0)."""
        self.origin = tuple(odom_pose)
        r, a = nc.clean_scan(ranges, angles, self.nav.scan_min)
        self.keyframes = [Keyframe((0.0, 0.0, 0.0), nc.subsample(nc.to_xy(r, a)))]

    def ir_a(self, meta_xy):
        self.meta = tuple(meta_xy)
        self.retorno = False
        self._reset_motion()

    def volver(self):
        self.retorno = True
        self.upto = len(self.crumbs) - 1
        self._reset_motion()

    def pose(self, odom_pose):
        return nc.compose(self.corr, nc.relative(self.origin, odom_pose))

    def paso(self, t, odom_pose, ranges, angles, bumper_hit=False):
        """Un ciclo de control. Devuelve (lineal, angular, info)."""
        nav, pp = self.nav, self.pp
        dt = 0.1 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5)
        self.last_t = t
        r, a = nc.clean_scan(ranges, angles, nav.scan_min)
        raw = nc.relative(self.origin, odom_pose)
        pose = nc.compose(self.corr, raw)

        if self.retorno:
            pose = self._corregir(t, pose, raw, r, a)
        else:
            self._registrar(pose, r, a)
        self.virtual.prune_expired(t, pp.virtual_ttl)
        vr, va = self.virtual.in_robot(pose, nav.max_range)
        R, A = np.r_[r, vr], np.r_[a, va]

        if self.retorno:
            self.upto = nc.pick_waypoint(self.crumbs, pose, R, A, self.upto, pp.lookahead, nav)
            target = self.crumbs[self.upto]
            final = (0.0, 0.0)
        else:
            target = final = self.meta
        bearing, _ = nc.goal_bearing(pose, target)
        dist = math.hypot(final[0] - pose[0], final[1] - pose[1])
        info = dict(pose=pose, dist=dist, front=float(nc.free_distance(R, A, 0.0, nav)[0]),
                    nearest=nc.nearest(r), estado="escape" if self.escape else "navega",
                    llegada=self.retorno and dist < pp.home_tol, atasco_repetido=False)
        if info["llegada"]:
            return 0.0, 0.0, info

        if bumper_hit and self.escape is None:
            self._log(t, f"choque por bumper en ({pose[0]:.2f}, {pose[1]:.2f}): escape inmediato")
            self.virtual.add_front(pose, self.nav.robot_radius + 0.04, self.nav.robot_radius)
            if hasattr(self.crumbs, "prune_branch") and len(self.crumbs) > 2:
                self.crumbs.prune_branch(len(self.crumbs) - 2, len(self.crumbs))
            self.escape = ("atras", t, pose, None)

        if self.escape is None and self.monitor.update(t, pose[:2], self.lin):
            info["atasco_repetido"] = self._iniciar_escape(
                t, pose, R, A, info["front"], espera=self.monitor.motivo == "espera")
        if self.escape is not None:
            lin, ang = self._escape(t, pose, R, A, r, a)
        else:
            h = nc.choose_heading(R, A, bearing, self.prev_heading, nav)
            if h is None:
                h = nc.most_open(R, A, nav)
            self.prev_heading = h
            free = float(nc.free_distance(R, A, h, nav)[0])
            # Salvaguarda anti-colision frontal: si el frente fisico esta obstruido,
            # no avanzar mientras se gira; girar en el sitio hasta despejar el frente.
            eff_free = min(free, info["front"])
            lin, ang = nc.speed_command(h, eff_free, pp.max_linear, pp.max_angular, nav)
            lin *= nc.side_scale(r, a, nav)   # mas despacio con poca holgura lateral

            # Reduccion suave muy cerca de la meta final (ultimos 40 cm) para no pasarse
            if dist < 0.40 and not self.escape:
                lin *= min(1.0, max(0.60, dist / 0.40))

            ang += nc.wall_repulsion(r, a, nav)
            lim = nc.angular_limit(pp.max_angular, info["nearest"], nav)
            ang = float(np.clip(ang, -lim, lim))

        # Rampa de aceleracion / frenado suave
        if lin > self.lin:
            lin = min(lin, self.lin + pp.max_accel * dt)
        elif lin < self.lin:
            if info["front"] > nav.stop_dist and not self.escape:
                lin = max(lin, self.lin - pp.max_decel * dt)
        self.lin = lin

        # Rampa angular para evitar sacudidas violentas de las ruedas
        d_ang = ang - self.ang
        max_d_ang = pp.max_alpha * dt
        if abs(d_ang) > max_d_ang:
            ang = self.ang + math.copysign(max_d_ang, d_ang)
        self.ang = ang

        return float(lin), float(ang), info

    # --- internos --------------------------------------------------------------
    def _log(self, t, texto):
        self.eventos.append((t, texto))

    def _reset_motion(self):
        self.monitor.reset()
        self.escape = None
        self.prev_heading = 0.0
        self.lin = 0.0
        self.ang = 0.0

    def _registrar(self, pose, r, a):
        x, y = pose[:2]
        cx, cy = self.crumbs[-1]
        if math.hypot(x - cx, y - cy) >= self.pp.crumb_every:
            self.crumbs.append((x, y))
        kp = self.keyframes[-1].pose if self.keyframes else None
        if kp is None or math.hypot(x - kp[0], y - kp[1]) >= self.pp.keyframe_every:
            self.keyframes.append(Keyframe(pose, nc.subsample(nc.to_xy(r, a))))

    def _corregir(self, t, pose, raw, r, a):
        """ICP contra el keyframe de la ida mas cercano (una vez por keyframe)."""
        pp = self.pp
        cerca = [k for k in self.keyframes if not k.used
                 and math.hypot(k.pose[0] - pose[0], k.pose[1] - pose[1]) < pp.icp_radius]
        if not cerca or len(r) < 30:
            return pose
        kf = min(cerca, key=lambda k: math.hypot(k.pose[0] - pose[0], k.pose[1] - pose[1]))
        kf.attempts += 1
        res = nc.icp_2d(kf.xy, nc.subsample(nc.to_xy(r, a)), nc.relative(kf.pose, pose))
        if res is None:
            if kf.attempts >= 3:
                kf.used = True
            return pose
        rel, inl, rmse = res
        true = nc.compose(kf.pose, rel)
        jump = math.hypot(true[0] - pose[0], true[1] - pose[1])
        if inl < pp.icp_min_inliers or rmse > pp.icp_max_rmse or jump > pp.icp_max_jump:
            self._log(t, f"ICP descartado (inl {inl:.2f} rmse {rmse:.3f} salto {jump:.2f}) intento {kf.attempts}")
            if kf.attempts >= 3:
                kf.used = True
            return pose
        kf.used = True
        self.corr = nc.compose(true, nc.inverse(raw))
        self._log(t, f"ICP corrige {jump * 100:.0f} cm / {math.degrees(nc.wrap(true[2] - pose[2])):+.1f} deg")
        return true

    def _iniciar_escape(self, t, pose, R, A, front, espera=False):
        if self._last_escape_xy is not None and math.hypot(
                pose[0] - self._last_escape_xy[0], pose[1] - self._last_escape_xy[1]) < 0.40:
            self._repeat_escapes += 1
        else:
            self._repeat_escapes = 1
        self._last_escape_xy = (pose[0], pose[1])

        repetido = self._repeat_escapes >= 2
        if espera:
            # No empujaba (frenaba ante algo cercano): no es un obstaculo bajo ni una trampa,
            # asi que no se siembran barreras; solo se reposiciona con la maniobra de salida.
            self._log(t, f"sin avance en ({pose[0]:.2f}, {pose[1]:.2f}) esperando con obstaculo cerca: reposiciono sin barreras")
        elif self.lin > 0.03 and front > self.pp.blind_front:
            # Barrera permanente (t=None): los obstaculos bajos no desaparecen con el tiempo
            self.virtual.add_front(pose, self.nav.robot_radius + 0.04, self.nav.robot_radius)
            self._log(t, f"atasco con frente libre: obstaculo bajo en ({pose[0]:.2f}, {pose[1]:.2f})")
        elif not repetido and (self.lin > 0.01 or front > self.nav.stop_dist):
            # Sella la trampa con un ancho acotado al radio del robot y caducidad virtual_ttl
            dist_barrera = max(front, self.nav.robot_radius + 0.12)
            self.virtual.add_front(pose, dist_barrera, self.nav.robot_radius, t=t)
            self._log(t, f"atasco en ({pose[0]:.2f}, {pose[1]:.2f}): sellando trampa con barrera virtual")
        if repetido:
            self._log(t, f"atasco repetido en ({pose[0]:.2f}, {pose[1]:.2f}): escapando sin borrar barreras")
        if hasattr(self.crumbs, "prune_branch") and len(self.crumbs) > 2:
            self.crumbs.prune_branch(len(self.crumbs) - 2, len(self.crumbs))
        self.escape = ("atras", t, pose, None)
        return repetido

    def _escape(self, t, pose, R, A, r=None, a=None):
        fase, t0, p0, rumbo = self.escape
        r_real = R if r is None else r
        a_real = A if a is None else a
        if fase == "atras":
            # Usa el LiDAR real para retroceder: una barrera virtual vieja atras no debe bloquear la marcha atras
            back_free = nc.free_distance(r_real, a_real, math.pi, self.nav)[0]
            moved = math.hypot(pose[0] - p0[0], pose[1] - p0[1])
            if moved < self.pp.back_dist and back_free > 0.20 and t - t0 < 3.0:
                return -self.pp.back_speed, 0.0
            # gira hacia lo mas abierto, evitando volver de frente a donde se trabo
            hs = nc.headings(self.nav)
            free = nc.free_distance(R, A, hs, self.nav)
            free[np.abs(hs) < math.radians(30)] = 0.0
            if np.max(free) < 0.15:
                free = nc.free_distance(r_real, a_real, hs, self.nav)
                free[np.abs(hs) < math.radians(30)] = 0.0
            rumbo = float(nc.wrap(pose[2] + hs[np.argmax(free)]))
            self.escape = fase, t0, p0, rumbo = ("giro", t, pose, rumbo)
        err = float(nc.wrap(rumbo - pose[2]))
        if abs(err) > math.radians(10) and t - t0 < self.pp.escape_turn:
            return 0.0, float(np.clip(1.5 * err, -self.pp.max_angular, self.pp.max_angular))
        self.escape = None
        self.monitor.reset()
        # Mantiene inercia direccional hacia la salida en lugar de reiniciar a 0
        self.prev_heading = float(nc.wrap(rumbo - pose[2]))
        return 0.0, 0.0


def llegada_por_distancia(info, tol):
    """Llegada a B por defecto: a menos de tol de la meta (pose corregida)."""
    return info["dist"] < tol


# --- ROS ---------------------------------------------------------------------------

class Lidar:
    """Ultimo /scan con angulos en el marco del robot (0 = frente, + = izquierda).
    En el TB4 el RPLIDAR esta girado: rot_deg = 90 (ver 3_referencia/sigueme.py)."""

    def __init__(self, node, topic, rot_deg, mask=None):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan

        self.rot = math.radians(rot_deg)
        self.mask = mask  # (centro_deg, medio_ancho_deg, r_max_m) o None; ver nc.mask_sector
        self.data = None
        self.extra = None   # f(t, r, a) -> (r, a): une puntos extra (guardian de profundidad)
        self.last_t = 0.0
        node.create_subscription(LaserScan, topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        r = np.asarray(msg.ranges, np.float64)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment + self.rot
        self.data = nc.clean_scan(r, a, msg.range_min, msg.range_max)
        if self.mask:
            self.data = nc.mask_sector(*self.data, *self.mask)
        self.last_t = time.monotonic()
        if self.extra is not None:
            self.data = self.extra(self.last_t, *self.data)

    def fresh(self, max_age=0.6):
        return self.data is not None and time.monotonic() - self.last_t < max_age


class Odom:
    """Pose (x, y, yaw) del /odom del Create 3."""

    def __init__(self, node, topic):
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.pose = None
        self.last_t = 0.0
        node.create_subscription(Odometry, topic, self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose, self.last_t = (p.x, p.y, yaw), time.monotonic()

    def fresh(self, max_age=0.6):
        return self.pose is not None and time.monotonic() - self.last_t < max_age


def add_ros_args(ap):
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--cmd-vel", default="/cmd_vel")
    ap.add_argument("--lidar-rot", type=float, default=90.0,
                    help="grados a sumar al /scan para que 0 sea el frente (TB4: 90)")
    ap.add_argument("--lidar-mask", type=float, nargs=3, metavar=("CENTRO", "MEDIO", "RMAX"),
                    help="ignorar una pieza fija del robot: lecturas a +-MEDIO grados de CENTRO "
                         "(marco del robot, 0 = frente) y a menos de RMAX m. Ej.: 180 8 0.25")
    ap.add_argument("--dry", action="store_true", help="no publica /cmd_vel")
    ap.add_argument("--config", help="JSON con parametros (ver config.json)")
    ap.add_argument("--param", action="append", default=[], metavar="nombre=valor")


def wait_sensors(node, lidar, odom, timeout=15.0):
    import rclpy
    # /start_motor solo si NO llega /scan: con el LiDAR girando, el driver rplidar falla
    # ('Cannot start scan: 80008000') y muere (el bringup no lo relanza).
    fin = time.monotonic() + 1.5
    while time.monotonic() < fin and not lidar.fresh():
        rclpy.spin_once(node, timeout_sec=0.05)
    if not lidar.fresh():
        try:
            from std_srvs.srv import Empty
            cli = node.create_client(Empty, "/start_motor")
            if cli.wait_for_service(timeout_sec=0.5):
                print("[arranque] no llega /scan: pido /start_motor")
                cli.call_async(Empty.Request())
        except Exception:
            pass
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if lidar.fresh() and odom.fresh():
            return True
    return False


def main():
    import rclpy

    from comun import Sender, apply_params

    ap = argparse.ArgumentParser(description="Navegacion reactiva A -> B -> A (prueba)")
    ap.add_argument("--goal", type=float, nargs=2, required=True, metavar=("X", "Y"),
                    help="B en m respecto a A (x = frente del robot en A, y = izquierda)")
    ap.add_argument("--pausa", type=float, default=2.0, help="s parado en B")
    ap.add_argument("--rate", type=float, default=10.0)
    add_ros_args(ap)
    args = ap.parse_args()

    piloto = Piloto()
    apply_params([piloto.nav, piloto.pp], args.config, args.param)

    rclpy.init()
    node = rclpy.create_node("navegacion")
    sender = None
    try:
        lidar, odom = Lidar(node, args.scan, args.lidar_rot, args.lidar_mask), Odom(node, args.odom)
        sender = Sender(node, args.cmd_vel, args.dry)
        bump_t = 0.0
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

        if not wait_sensors(node, lidar, odom):
            raise SystemExit("No llegan /scan y /odom: revisa la conexion (tb4_check.sh).")
        piloto.fijar_origen(odom.pose, *lidar.data)
        piloto.ir_a(args.goal)
        t0, fase, t_b = time.monotonic(), "ida", None
        last_log = 0.0
        print(f"Ida a B {args.goal}")
        while fase != "fin":
            # un ciclo a 'rate' Hz atendiendo todos los callbacks (spin_once atiende solo uno)
            fin_ciclo = time.monotonic() + 1.0 / args.rate
            while True:
                rclpy.spin_once(node, timeout_sec=max(fin_ciclo - time.monotonic(), 0.0))
                if time.monotonic() >= fin_ciclo:
                    break
            t = time.monotonic()
            if not (lidar.fresh() and odom.fresh()):
                sender.send(0.0, 0.0)  # sin sensores no se mueve
                continue
            if fase == "pausa":
                sender.send(0.0, 0.0)
                if t - t_b > args.pausa:
                    piloto.volver()
                    fase = "vuelta"
                    print("Regreso a A")
                continue
            bumper_hit = (time.monotonic() - bump_t < 0.3)
            lin, ang, info = piloto.paso(t, odom.pose, *lidar.data, bumper_hit=bumper_hit)
            if t - last_log >= 1.0:
                last_log = t
                target_str = f"B={args.goal}" if fase == "ida" else "A=(0,0)"
                print(f"[{t - t0:5.1f}s] {fase.upper():6s} | dist: {info['dist']:.2f}m a {target_str} | frente: {info['front']:.2f}m | v: {lin:+.2f}m/s w: {ang:+.2f}rad/s")
            for te, txt in piloto.eventos:
                print(f"[{te - t0:6.1f}s] {txt}")
            piloto.eventos.clear()
            if fase == "ida" and llegada_por_distancia(info, piloto.pp.goal_tol):
                fase, t_b = "pausa", t
                print(f"[{t - t0:6.1f}s] llegue a B")
                continue
            if info["llegada"]:
                fase = "fin"
                print(f"[{t - t0:6.1f}s] llegue a A")
                break
            sender.send(lin, ang)
    except KeyboardInterrupt:
        pass
    finally:
        if sender is not None:
            sender.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
