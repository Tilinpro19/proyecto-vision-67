"""
Mision completa de la competencia: ESPERA -> IR_A_B -> ESCANEAR -> VOLVER_A_A -> ANUNCIAR -> FIN
(PARADA = parada de emergencia). Cronometro global desde que sale de A, registro de
eventos en logs/ y resultado en JSON (lo usa 2_laboratorio/calibrar.py).

  Tarea A (autonoma):  ./tarea_a.sh --goal 3.0 0.5
  Tarea B (gestos):    ./tarea_b.sh
  (o directo: python3 mision.py --tarea A --goal 3.0 0.5 [--dry] [--ver])

Tarea A: B se da como (x, y) en m respecto a A (x = hacia donde mira el robot en A,
y = izquierda), medido con cinta el dia de la competencia. La llegada a B es enchufable
(--param llegada=distancia|foto|cualquiera, ver llegada_b()).
Tarea B: palma 1 s arma y arranca el cronometro; V 1.5 s en B = escanear la foto;
de vuelta en A, V 1.5 s (o automatico al estar a menos de home_tol_b de A) = anunciar.

Parada de emergencia: Ctrl+C, 'ros2 topic pub --once /mision/parar std_msgs/msg/Empty',
superar t_max, o (Tarea B) puno cerrado = desarmar.
"""
import argparse
import json
import math
import os
import time
from dataclasses import asdict, dataclass

import numpy as np

import nav_core as nc
from navegacion import Piloto, llegada_por_distancia

HERE = os.path.dirname(os.path.abspath(__file__))
ESPERA, IR_A_B, ESCANEAR, VOLVER_A_A, ANUNCIAR, FIN, PARADA = (
    "ESPERA", "IR_A_B", "ESCANEAR", "VOLVER_A_A", "ANUNCIAR", "FIN", "PARADA")


@dataclass
class MisionParams:
    t_max: float = 300.0          # s   tiempo maximo (aun no anunciado): pasado esto, para
    choque_dist: float = 0.20     # m   algo a menos de esto del centro = choque (radio 0.17)
    llegada: str = "distancia"    # llegada a B: distancia | foto | cualquiera
    foto_ancho: float = 0.10      # cara >= esta fraccion del ancho de imagen = "llegue (foto)"
    foto_frente: float = 0.70     # m   ...y con menos que esto libre al frente
    escaneo_frames: int = 12      # frames con cara para decidir el nombre
    escaneo_max: float = 15.0     # s   maximo escaneando/buscando la cara en B
    busqueda_angular: float = 0.4 # rad/s girando para buscar la foto
    home_tol_b: float = 0.35      # m   Tarea B: anuncio automatico cerca de A
    cuenta: float = 0.0           # s   espera antes de salir de A
    rate: float = 10.0            # Hz  ciclo de control


def drenar(spin_once, periodo, reloj=time.monotonic):
    """Un ciclo de control de 'periodo' s: atiende TODOS los callbacks que lleguen y vuelve
    cuando termina el periodo. rclpy.spin_once atiende uno solo por llamada; sin esto el
    control corria una vez por mensaje recibido (scan + odom + camara = decenas de veces
    por segundo) en vez de a 'rate' Hz, y gastaba CPU de la Pi sin ganar nada."""
    fin = reloj() + periodo
    spin_once(periodo)
    while True:
        resto = fin - reloj()
        if resto <= 0:
            return
        spin_once(resto)


class Perfil:
    """Cuanto cuesta la mision en la Pi: tiempo de cada ciclo de navegacion y CPU total del
    proceso (en % de UN nucleo; la Pi 4 tiene 4, asi que 100 % = un nucleo entero)."""

    def __init__(self, reloj=time.perf_counter, cpu=time.process_time):
        self.reloj, self.cpu = reloj, cpu
        self.t_ini, self.cpu_ini = reloj(), cpu()
        self.ms = []

    def registrar(self, t0):
        """t0 = self.reloj() justo antes del calculo que se mide."""
        self.ms.append((self.reloj() - t0) * 1000.0)

    def resumen(self):
        wall = self.reloj() - self.t_ini
        ms = sorted(self.ms)
        return dict(ciclos=len(ms),
                    ciclo_ms_prom=round(sum(ms) / len(ms), 2) if ms else 0.0,
                    ciclo_ms_p95=round(ms[min(int(len(ms) * 0.95), len(ms) - 1)], 2) if ms else 0.0,
                    ciclo_ms_max=round(ms[-1], 2) if ms else 0.0,
                    cpu_pct=round(100.0 * (self.cpu() - self.cpu_ini) / wall, 1) if wall > 0 else 0.0)


class Mision:
    """Estado, cronometro y registro. Sin ROS."""

    def __init__(self, tarea, log_dir=None, clock=time.monotonic, quiet=False):
        self.tarea, self.clock, self.quiet = tarea, clock, quiet
        self.perfil = Perfil()
        self.estado, self.t0, self.t_fin = ESPERA, None, None
        self.eventos, self.choques = [], 0
        self.nombre, self.score, self.seguro = None, 0.0, False
        self.log = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.log_path = os.path.join(log_dir, f"mision_{tarea}_{stamp}.log")
            self.log = open(self.log_path, "w", encoding="utf-8")

    def tiempo(self):
        if self.t0 is None:
            return 0.0
        return (self.t_fin or self.clock()) - self.t0

    def evento(self, texto):
        line = f"[{self.tiempo():7.2f}s] {self.estado:10s} {texto}"
        self.eventos.append(line)
        if not self.quiet:
            print(line, flush=True)
        if self.log:
            self.log.write(line + "\n")
            self.log.flush()

    def ir(self, estado, motivo=""):
        if estado == IR_A_B and self.t0 is None:
            self.t0 = self.clock()
            self.perfil = Perfil(self.perfil.reloj, self.perfil.cpu)  # sin la carga de modelos
        if estado in (FIN, PARADA) and self.t_fin is None and self.t0 is not None:
            self.t_fin = self.clock()
        anterior, self.estado = self.estado, estado
        self.evento(f"{anterior} -> {estado}" + (f" ({motivo})" if motivo else ""))

    def activa(self):
        return self.estado not in (FIN, PARADA)

    def choque(self, origen):
        self.choques += 1
        self.evento(f"CHOQUE #{self.choques} ({origen})")

    def resultado(self):
        return dict(tarea=self.tarea, ok=self.estado == FIN, estado=self.estado,
                    tiempo=round(self.tiempo(), 2), choques=self.choques, nombre=self.nombre,
                    score=round(self.score, 3), seguro=self.seguro,
                    perfil=self.perfil.resumen())

    def cerrar(self):
        if self.log:
            self.log.close()


class ContadorChoques:
    """Un choque por contacto: entra con algo a < umbral (o bumper) y se rearma al
    alejarse umbral + histeresis durante al menos 'rearme' s."""

    def __init__(self, umbral, histeresis=0.03, rearme=1.0):
        self.umbral, self.hist, self.rearme = umbral, histeresis, rearme
        self.contacto, self.t_libre = False, None

    def update(self, t, nearest, bump=False):
        if nearest < self.umbral or bump:
            self.t_libre = None
            if not self.contacto:
                self.contacto = True
                return True
        elif self.contacto and nearest > self.umbral + self.hist:
            self.t_libre = self.t_libre if self.t_libre is not None else t
            if t - self.t_libre >= self.rearme:
                self.contacto = False
        return False


def llegada_b(info, vista, mp, goal_tol):
    """Condicion de llegada a B (enchufable). info: del Piloto (dist, front, ...).
    vista: fraccion del ancho de imagen que ocupa la cara mas grande (0 si no hay)."""
    por_dist = llegada_por_distancia(info, goal_tol)
    por_foto = vista >= mp.foto_ancho and info["front"] < mp.foto_frente
    return {"distancia": por_dist, "foto": por_foto, "cualquiera": por_dist or por_foto}[mp.llegada]


class Escaneo:
    """ESCANEAR sin bloquear: junta frames con cara; si no ve ninguna, barre girando.
    paso(t, img_nueva) -> (angular, frames listos o None)."""

    def __init__(self, rostros, mp, t0, buscar=True, limite=None):
        self.rostros, self.mp, self.t0, self.buscar = rostros, mp, t0, buscar
        self.limite = limite or mp.escaneo_max
        self.frames, self.visto = [], False

    def paso(self, t, img):
        if img is not None:
            _, faces = self.rostros.detectar(img)
            if len(faces):
                self.frames.append(img)
                self.visto = True
        listo = len(self.frames) >= self.mp.escaneo_frames or t - self.t0 > self.limite
        if listo:
            return 0.0, self.frames
        if self.visto or not self.buscar or t - self.t0 < 1.5:
            return 0.0, None
        # barrido que se abre: izq 1.5 s, der 3 s, izq 4.5 s, ... (+-35, +-70 grados a 0.4 rad/s)
        tau, n, seg = t - self.t0 - 1.5, 0, 1.5
        while tau >= seg:
            tau, n, seg = tau - seg, n + 1, seg + 1.5
        return (self.mp.busqueda_angular if n % 2 == 0 else -self.mp.busqueda_angular), None


# --- ROS ---------------------------------------------------------------------------

def args_parser():
    from gestos_teleop import add_gesture_args
    from navegacion import add_ros_args

    ap = argparse.ArgumentParser(description="Mision completa A -> B -> A")
    ap.add_argument("--tarea", choices=("A", "B"), required=True)
    ap.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"),
                    help="Tarea A: B en m respecto a A (x frente, y izquierda)")
    ap.add_argument("--galeria", default=os.path.join(HERE, "galeria"))
    ap.add_argument("--umbral", type=float, default=None, help="similitud SFace (0.363)")
    ap.add_argument("--voz", default="auto", choices=("auto", "windows", "linux", "ninguna"))
    ap.add_argument("--resultado", help="guardar el resultado en este JSON")
    ap.add_argument("--ver", action="store_true", help="Tarea A: ventana con la camara")
    add_ros_args(ap)
    add_gesture_args(ap)
    return ap


class Sistema:
    """Nodo, sensores, actuadores, vision y anuncio, compartidos por las dos tareas."""

    def __init__(self, args, mp):
        import rclpy
        from std_msgs.msg import Empty

        import facial
        from comun import Sender
        from mano_teleop import RobotCamera
        from navegacion import Lidar, Odom

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("mision")
        self.lidar = Lidar(self.node, args.scan, args.lidar_rot, args.lidar_mask)
        self.odom = Odom(self.node, args.odom)
        self.cam = RobotCamera(self.node, args.robot_cam)
        self.cam_t = 0.0
        self.parar = False
        self.node.create_subscription(Empty, "/mision/parar", self._on_parar, 10)
        self.bump_t = 0.0
        try:  # bumper del Create 3 si los mensajes estan instalados en esta PC
            from irobot_create_msgs.msg import HazardDetectionVector
            from rclpy.qos import qos_profile_sensor_data
            self.node.create_subscription(HazardDetectionVector, "/hazard_detection",
                                          self._on_hazard, qos_profile_sensor_data)
            self.bumper = True
        except ImportError:
            self.bumper = False
        self.anunciador = facial.Anunciador(self.node, voz=args.voz)
        self.rostros = facial.Rostros()
        self.rostros.calentar()
        self.galeria = facial.Galeria(self.rostros, args.galeria)
        print(f"Galeria: {self.galeria.resumen() or 'VACIA (ver galeria/README.md)'}"
              f" | bumper: {'si' if self.bumper else 'no (solo LiDAR)'}")
        self.umbral = args.umbral or facial.UMBRAL
        self.sender = Sender(self.node, args.cmd_vel, args.dry)
        self.mp = mp
        self._despertar_sensores()

    def _despertar_sensores(self):
        """Enciende el LiDAR y la camara SOLO si no llegan datos. Llamar a /start_motor con el
        LiDAR ya girando hace que el driver rplidar falle ('Cannot start scan: 80008000') y
        muera, y el bringup no lo relanza: la segunda corrida se queda sin /scan."""
        try:
            from std_srvs.srv import Empty, Trigger
            fin = time.monotonic() + 1.5
            while time.monotonic() < fin and not (self.lidar.fresh() and self.cam.msg is not None):
                self.spin(0.1)
            if not self.lidar.fresh():
                cli_lidar = self.node.create_client(Empty, "/start_motor")
                if cli_lidar.wait_for_service(timeout_sec=0.5):
                    self.evento_arranque("no llega /scan: pido /start_motor")
                    cli_lidar.call_async(Empty.Request())
            if self.cam.msg is None and self.cam.img is None:
                cli_cam = self.node.create_client(Trigger, "/oakd/start_camera")
                if cli_cam.wait_for_service(timeout_sec=0.5):
                    self.evento_arranque("no llega la camara: pido /oakd/start_camera")
                    cli_cam.call_async(Trigger.Request())
        except Exception:
            pass

    @staticmethod
    def evento_arranque(texto):
        print(f"[arranque] {texto}", flush=True)

    def _on_parar(self, _msg):
        self.parar = True

    def _on_hazard(self, msg):
        if any(d.type == 1 for d in msg.detections):  # 1 = BUMP
            self.bump_t = time.monotonic()

    def spin(self, timeout):
        """Un ciclo de control de 'timeout' s (ver drenar)."""
        drenar(lambda t: self.rclpy.spin_once(self.node, timeout_sec=t), timeout)

    def bump(self):
        return time.monotonic() - self.bump_t < 0.3

    def frame_nuevo(self):
        img = self.cam.latest()
        if img is None or self.cam.last_t <= self.cam_t:
            return None
        self.cam_t = self.cam.last_t
        return img

    def vista_cara(self, img):
        if img is None:
            return 0.0
        big, faces = self.rostros.detectar(img)
        return float(faces[0][2] / big.shape[1]) if len(faces) else 0.0

    def identificar(self, frames, m):
        import facial
        res = facial.identificar(frames, self.rostros, self.galeria, self.umbral)
        if res.nombre is None and self.galeria.nombres:
            res.nombre = self.galeria.nombres[0]  # sin cara: algo hay que decir
            m.evento("no vi ninguna cara: digo el primero de la galeria")
        m.nombre, m.score, m.seguro = res.nombre, res.score, res.seguro
        top = ", ".join(f"{n} {s:.2f}" for n, s in res.ranking[:3])
        m.evento(f"identificado: {res.nombre} ({res.score:.2f}, "
                 f"{'seguro' if res.seguro else 'DUDOSO'}, {res.n_caras} frames) [{top}]")

    def cerrar(self):
        try:
            self.sender.stop()
        finally:
            self.anunciador.cerrar()
            self.node.destroy_node()
            self.rclpy.try_shutdown()


def esperar_sensores(s, m, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s.spin(0.05)
        if s.lidar.fresh() and s.odom.fresh():
            if s.cam.latest() is None:
                m.evento("AVISO: aun no llega la camara del robot")
            return True
    return False


def comunes(s, m, cont, t):
    """Chequeos de cada ciclo. Devuelve False si el robot debe quedarse quieto."""
    if s.parar:
        m.ir(PARADA, "/mision/parar")
        return False
    if m.t0 is not None and m.tiempo() > s.mp.t_max:
        m.ir(PARADA, f"supero t_max = {s.mp.t_max:.0f} s")
        return False
    if not (s.lidar.fresh() and s.odom.fresh()):
        s.sender.send(0.0, 0.0)
        return False
    if cont.update(t, nc.nearest(s.lidar.data[0]), s.bump()):
        m.choque("bumper" if s.bump() else "LiDAR")
    return True


def tarea_a(args, s, m, piloto):
    import cv2
    mp = s.mp
    if args.goal is None:
        raise SystemExit("Tarea A: falta --goal X Y (posicion de B respecto a A, en m)")
    cont = ContadorChoques(mp.choque_dist)
    piloto.fijar_origen(s.odom.pose, *s.lidar.data)
    # La camara ya se comprobo en esperar_sensores. Mientras solo navega no se necesita:
    # sin suscripcion no llegan (ni se deserializan) imagenes que gastan CPU de la Pi.
    solo_navega = mp.llegada == "distancia" and not args.ver
    if solo_navega:
        s.cam.pausar()
    if mp.cuenta:
        m.evento(f"salgo en {mp.cuenta:.0f} s")
        time.sleep(mp.cuenta)
    piloto.ir_a(args.goal)
    m.ir(IR_A_B, f"B = ({args.goal[0]:.2f}, {args.goal[1]:.2f})")
    escaneo, t_vista, vista = None, 0.0, 0.0
    while m.activa():
        s.spin(1.0 / mp.rate)
        t = time.monotonic()
        if not comunes(s, m, cont, t):
            continue
        for _, txt in piloto.eventos:
            m.evento(txt)
        piloto.eventos.clear()
        # La camara solo se decodifica cuando hace falta (en la Pi cada JPEG cuesta CPU que
        # el LiDAR y el control necesitan): al escanear, con --ver, o si la llegada usa la foto.
        img = s.frame_nuevo() if (m.estado == ESCANEAR or args.ver
                                  or (m.estado == IR_A_B and mp.llegada != "distancia")) else None
        if m.estado == IR_A_B:
            t_p = m.perfil.reloj()
            lin, ang, info = piloto.paso(t, s.odom.pose, *s.lidar.data, bumper_hit=s.bump())
            m.perfil.registrar(t_p)
            if mp.llegada != "distancia" and img is not None and t - t_vista > 0.3:
                vista, t_vista = s.vista_cara(img), t
            if llegada_b(info, vista, mp, piloto.pp.goal_tol):
                s.sender.send(0.0, 0.0)
                s.cam.reanudar()
                m.ir(ESCANEAR, f"a {info['dist']:.2f} m de B")
                escaneo = Escaneo(s.rostros, mp, t)
                continue
            s.sender.send(lin, ang)
        elif m.estado == ESCANEAR:
            ang, frames = escaneo.paso(t, img)
            s.sender.send(0.0, ang)
            if frames is not None:
                s.sender.send(0.0, 0.0)
                s.identificar(frames, m)
                if solo_navega:
                    s.cam.pausar()
                piloto.volver()
                m.ir(VOLVER_A_A)
        elif m.estado == VOLVER_A_A:
            t_p = m.perfil.reloj()
            lin, ang, info = piloto.paso(t, s.odom.pose, *s.lidar.data, bumper_hit=s.bump())
            m.perfil.registrar(t_p)
            if info["llegada"]:
                s.sender.stop()
                m.ir(ANUNCIAR, f"a {info['dist']:.2f} m de A")
                continue
            s.sender.send(lin, ang)
        elif m.estado == ANUNCIAR:
            s.sender.stop()
            s.anunciador.anunciar(m.nombre or "desconocido")
            m.ir(FIN)
        if args.ver and img is not None:
            cv2.putText(img, f"{m.estado} {m.tiempo():.1f}s", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("mision A", img)
            cv2.waitKey(1)


def tarea_b(args, s, m):
    import cv2

    from comun import LIMIT_ANGULAR, LIMIT_LINEAR, clamp
    from gestos_teleop import Gestos, draw_hud, front_limit
    from mano_teleop import PANEL_H, fit_height, robot_panel

    mp, nav = s.mp, nc.NavParams()
    max_l, max_a = clamp(args.lineal, 0, LIMIT_LINEAR), clamp(args.angular, 0, LIMIT_ANGULAR)
    cont = ContadorChoques(mp.choque_dist)
    gestos = Gestos(args.cam, args.clahe)
    win = "Tarea B"
    origen, escaneo, moving = None, None, False
    m.evento("palma abierta 1 s para armar y salir")
    try:
        while m.activa():
            s.spin(0)
            t = time.monotonic()
            out = gestos.leer(t)
            if out is None:
                m.ir(PARADA, "la webcam dejo de entregar imagenes")
                break
            frame, direction, evento, pose = out
            ok = comunes(s, m, cont, t)
            if evento:
                m.evento(f"gesto: {evento}")
            img = s.frame_nuevo()
            lin = front_limit(direction[0] * max_l, s.lidar, nav)
            ang = direction[1] * max_a
            if m.estado == ESPERA and evento == "armado":
                origen = s.odom.pose
                m.ir(IR_A_B)
            elif m.estado == IR_A_B and evento == "accion":
                escaneo = Escaneo(s.rostros, mp, t, buscar=False, limite=5.0)
                m.ir(ESCANEAR)
            elif m.estado == ESCANEAR:
                lin = ang = 0.0
                _, frames = escaneo.paso(t, img)
                if frames is not None:
                    if frames:
                        s.identificar(frames, m)
                        m.ir(VOLVER_A_A)
                    else:
                        m.evento("no veo la cara: apunta la camara a la foto y repite la V")
                        m.ir(IR_A_B)
            elif m.estado == VOLVER_A_A:
                d = math.hypot(*nc.relative(origen, s.odom.pose)[:2])
                if evento == "accion" or d < mp.home_tol_b:
                    lin = ang = 0.0
                    s.sender.stop()
                    m.ir(ANUNCIAR, "gesto V" if evento == "accion" else f"a {d:.2f} m de A")
                    s.anunciador.anunciar(m.nombre or "desconocido")
                    m.ir(FIN)
            if not ok or not m.activa() or m.estado in (ESPERA, ESCANEAR):
                lin = ang = 0.0
            if lin or ang:
                s.sender.send(lin, ang)
                moving = True
            elif moving:
                s.sender.stop()
                moving = False
            draw_hud(frame, pose, gestos.ctrl, t, lin, ang, f"| {m.estado} {m.tiempo():.1f}s")
            cv2.imshow(win, np.hstack([fit_height(frame, PANEL_H), robot_panel(s.cam)]))
            cv2.waitKey(1)
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    m.ir(PARADA, "ventana cerrada")
            except cv2.error:
                pass
    finally:
        gestos.close()
        cv2.destroyAllWindows()


def main():
    from comun import apply_params

    args = args_parser().parse_args()
    piloto, mp = Piloto(), MisionParams()
    config = args.config or (os.path.join(HERE, "config.json")
                             if os.path.isfile(os.path.join(HERE, "config.json")) else None)
    valores = apply_params([piloto.nav, piloto.pp, mp], config, args.param)

    m = Mision(args.tarea, os.path.join(HERE, "logs"))
    m.evento(f"tarea {args.tarea} | config {config} | ajustes {valores}")
    s = None
    try:
        s = Sistema(args, mp)
        if not esperar_sensores(s, m):
            raise SystemExit("No llegan /scan y /odom: revisa la conexion (tb4_check.sh).")
        if args.tarea == "A":
            tarea_a(args, s, m, piloto)
        else:
            tarea_b(args, s, m)
    except KeyboardInterrupt:
        m.ir(PARADA, "Ctrl+C")
    finally:
        if s is not None:
            s.cerrar()
        res = m.resultado()
        res["params"] = dict(nav=asdict(piloto.nav), piloto=asdict(piloto.pp), mision=asdict(mp))
        m.evento(f"RESULTADO: {json.dumps({k: v for k, v in res.items() if k != 'params'})}")
        if args.resultado:
            with open(args.resultado, "w", encoding="utf-8") as f:
                json.dump(res, f, indent=2, ensure_ascii=False)
        m.cerrar()


if __name__ == "__main__":
    main()
