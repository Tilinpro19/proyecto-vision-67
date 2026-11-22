"""
Mision completa de la competencia: ESPERA -> IR_A_B -> ESCANEAR -> VOLVER_A_A -> ANUNCIAR -> FIN
(PARADA = parada de emergencia). Cronometro global desde que sale de A, registro de
eventos en logs/ y resultado en JSON (lo usa 2_laboratorio/calibrar.py).

  Tarea A (autonoma):  ./tarea_a.sh                 (modo directo, sin meta fija)
                       ./tarea_a.sh --goal 3.0 0.5  (B medido con cinta)
  Tarea B (gestos):    ./tarea_b.sh
  (o: python3 mision.py --tarea A [--modo directo|guiado|goal] [--goal X Y] [--ver])

Tarea A sin --goal (por defecto, --modo directo, ver directo.py): recto hasta la flecha,
giro en el sitio, recto ... hasta las caras (bandera cercana, caras al frente o callejon
con caras); ignora huecos laterales sin flecha. Ajustes: --param v_recto=, d_giro=, ...
--modo guiado: explorador de pasillos/cruces (PilotoGuiado) para pistas con cruces sin flecha.
--goal X Y: B en m respecto a A (x = hacia donde mira el robot en A, y = izquierda).
La llegada a B con --goal es enchufable (--param llegada=distancia|foto|cualquiera).
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
ESPERA, IR_A_B, SEGUIR, DECIDIR, LLEGADA_B, ESCANEAR, VOLVER_A_A, ANUNCIAR, FIN, PARADA = (
    "ESPERA", "IR_A_B", "SEGUIR", "DECIDIR", "LLEGADA_B",
    "ESCANEAR", "VOLVER_A_A", "ANUNCIAR", "FIN", "PARADA"
)


@dataclass
class MisionParams:
    t_max: float = 300.0          # s   tiempo maximo (aun no anunciado): pasado esto, para
    choque_dist: float = 0.20     # m   algo a menos de esto del centro = choque (radio 0.17)
    llegada: str = "distancia"    # llegada a B: distancia | foto | cualquiera
    foto_ancho: float = 0.10      # cara >= esta fraccion del ancho de imagen = "llegue (foto)"
    foto_frente: float = 0.70     # m   ...y con menos que esto libre al frente
    escaneo_frames: int = 12      # frames con cara para decidir el nombre
    escaneo_frames_min: int = 4   # desde aqui, si la identificacion ya es clara, no espera mas
    escaneo_margen_rapido: float = 0.10  # ...clara = seguro y con esta ventaja sobre el segundo
    escaneo_max: float = 15.0     # s   maximo escaneando/buscando la cara en B
    busqueda_angular: float = 0.4 # rad/s girando para buscar la foto
    home_tol_b: float = 0.35      # m   Tarea B: anuncio automatico cerca de A
    cuenta: float = 0.0           # s   espera antes de salir de A
    rate: float = 10.0            # Hz  ciclo de control
    # Supuestos configurables para Tarea A guiada por senales (Seccion 7 de PLAN_ANTIGRAVITY.md)
    senal_prohibido: str = "buscar_otro"  # buscar_otro | retroceder
    senal_bandera: str = "llegada_b"      # llegada_b | ignorar
    senal_qr: str = "registrar"           # registrar | ignorar
    regreso_modo: str = "migas"           # migas (desanda por migas sin obedecer flechas al volver)
    flechas_marco: str = "robot"          # robot (izq/der relativas al robot)
    bandera_frames: int = 2               # frames seguidos con bandera para confirmar llegada a B
    depth_guard: bool = False             # guardian de profundidad OAK-D (bajo demanda; ver profundidad.py).
                                          # Apagado: en la 1a prueba real dejo al robot atascado; activar con --param depth_guard=1
    cam_periodo: float = 0.20             # s entre lecturas de senales en modo guiado (ahorro CPU Pi)
    # Busqueda multi-persona + QR en B (cuando --persona se indica al inicio del circuito)
    persona: str = ""                     # nombre objetivo a buscar en B (vacio = modo clasico de 1 cara)
    escaneo_frames_persona: int = 6       # frames con cara por cada persona al barrer en B
    escaneo_objetivo_max: float = 45.0    # s   maximo barriendo 360 y leyendo QR en B
    qr_barrido_max: float = 8.0           # s   maximo buscando el QR al costado de la persona objetivo
    sector_visto_rad: float = 0.65        # rad (~37 deg) margen angular para no re-escanear la misma cara


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
        self.senales, self.decisiones, self.hitos_qr = [], [], []
        self.nombre, self.score, self.seguro = None, 0.0, False
        self.objetivo, self.qr_objetivo = None, None
        self.log_dir = log_dir
        self._diag_seq = 0
        self.log_path = None
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
        if estado in (IR_A_B, SEGUIR) and self.t0 is None:
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

    def guardar_diagnostico(self, motivo, pose, ranges, angles, crumbs=None, guardian=None,
                            imagen=None, nota=None):
        """Guarda scan, pose y migas en un .npz dentro de log_dir para depurar atascos/choques.
        imagen: ultimo cuadro de la camara (se guarda como JPEG en 'jpg'); nota: texto libre."""
        if not self.log_dir:
            return None
        self._diag_seq += 1
        base = os.path.splitext(os.path.basename(self.log_path))[0] if self.log_path else f"mision_{self.tarea}"
        nombre = f"{base}_{self._diag_seq:03d}_{motivo}.npz"
        ruta = os.path.join(self.log_dir, nombre)
        extra = {}
        if guardian is not None and guardian.ultimo_cuadro is not None:
            # separa lo que vio la camara: puntos virtuales + cuadro de profundidad reducido (1/4)
            extra = dict(virt_r=np.asarray(guardian.virtual[0], dtype=np.float32),
                         virt_a=np.asarray(guardian.virtual[1], dtype=np.float32),
                         depth4=guardian.ultimo_cuadro)
        if imagen is not None:
            try:
                import cv2
                ok, buf = cv2.imencode(".jpg", imagen, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    extra["jpg"] = buf.reshape(-1)
            except Exception:   # sin cv2 o imagen rara: el diagnostico sale igual sin foto
                pass
        if nota:
            extra["nota"] = np.array(str(nota))
        np.savez_compressed(
            ruta,
            **extra,
            t=np.array([self.tiempo()], dtype=np.float64),
            pose=np.asarray(pose, dtype=np.float64),
            ranges=np.asarray(ranges, dtype=np.float32),
            angles=np.asarray(angles, dtype=np.float32),
            crumbs=np.asarray(list(crumbs) if crumbs else [], dtype=np.float64),
        )
        self.evento(f"diagnostico guardado: {nombre}")
        return ruta

    def resultado(self):
        return dict(tarea=self.tarea, ok=self.estado == FIN, estado=self.estado,
                    tiempo=round(self.tiempo(), 2), choques=self.choques, nombre=self.nombre,
                    score=round(self.score, 3), seguro=self.seguro,
                    objetivo=self.objetivo, qr_objetivo=self.qr_objetivo,
                    senales=self.senales, decisiones=self.decisiones, hitos_qr=self.hitos_qr,
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

    def __init__(self, rostros, mp, t0, buscar=True, limite=None, identificar_fn=None):
        self.rostros, self.mp, self.t0, self.buscar = rostros, mp, t0, buscar
        self.limite = limite or mp.escaneo_max
        self.frames, self.visto = [], False
        self._identificar_fn = identificar_fn   # frames -> facial.Resultado (salida temprana)

    def _claro(self):
        """Con escaneo_frames_min frames ya hay un nombre seguro con ventaja amplia: identificar
        promedia los 3 mejores frames, esperar a 12 (~8 s en la Pi) no cambia el resultado."""
        n0 = max(1, self.mp.escaneo_frames_min)
        # identificar re-detecta todos los frames (~0.1 s c/u en la Pi): probar en n0, n0+3, ...
        if self._identificar_fn is None or len(self.frames) < n0 or (len(self.frames) - n0) % 3:
            return False
        res = self._identificar_fn(self.frames)
        segundo = res.ranking[1][1] if len(res.ranking) > 1 else -1.0
        return bool(res.seguro and res.score - segundo >= self.mp.escaneo_margen_rapido)

    def paso(self, t, img):
        nuevo = False
        if img is not None:
            _, faces = self.rostros.detectar(img)
            if len(faces):
                self.frames.append(img)
                self.visto = nuevo = True
        listo = (len(self.frames) >= self.mp.escaneo_frames or t - self.t0 > self.limite
                 or (nuevo and self._claro()))
        if listo:
            return 0.0, self.frames
        if self.visto or not self.buscar or t - self.t0 < 1.5:
            return 0.0, None
        # barrido que se abre: izq 1.5 s, der 3 s, izq 4.5 s, ... (+-35, +-70 grados a 0.4 rad/s)
        tau, n, seg = t - self.t0 - 1.5, 0, 1.5
        while tau >= seg:
            tau, n, seg = tau - seg, n + 1, seg + 1.5
        return (self.mp.busqueda_angular if n % 2 == 0 else -self.mp.busqueda_angular), None


class EscaneoObjetivo:
    """Barrido 360 en B cuando se busca una persona especifica y su QR al costado.
    Gira revisando cada cara en sectores no visitados; al hallar al objetivo,
    lee el QR cercano (o hace un pequeno barrido local izq/der) antes de retornar."""

    def __init__(self, objetivo, rostros, galeria, mp, t0, yaw0=0.0, umbral=None,
                 qr_fn=None, identificar_fn=None):
        self.objetivo = str(objetivo).strip()
        self.objetivo_norm = self.objetivo.lower()
        self.rostros, self.galeria, self.mp = rostros, galeria, mp
        self.t0 = t0
        self.limite = mp.escaneo_objetivo_max
        self.frames_por_cara = max(1, mp.escaneo_frames_persona)
        self.yaw_prev = yaw0
        self.yaw_abs = 0.0
        self.umbral = umbral
        self._qr_fn = qr_fn
        self._identificar_fn = identificar_fn

        self.subestado = "BUSCAR"  # BUSCAR | RECOLECTAR | BUSCAR_QR
        self.t_sub = t0
        self.t_qr = t0
        self.qr_yaw0 = yaw0
        self.qr_dir = 1
        self.sector_yaw = yaw0
        self.sectores_vistos = []
        self.frames = []
        self.qrs_sector = []
        self.vistos = []            # lista de (yaw, Resultado, qrs)
        self.mejor_objetivo = None  # (yaw, Resultado, qrs) si hubo match no-seguro
        self.eventos = []           # mensajes informativos para Mision.evento()

    def _qr_en_imagen(self, img):
        if img is None:
            return []
        if self._qr_fn is not None:
            return [str(q) for q in self._qr_fn(img) if q]
        import senales
        return [texto for texto, _ in senales.decodificar_qr(img, esfuerzo=2) if texto]

    def _identificar(self, frames):
        if self._identificar_fn is not None:
            return self._identificar_fn(frames)
        import facial
        umbral = facial.UMBRAL if self.umbral is None else self.umbral
        return facial.identificar(frames, self.rostros, self.galeria, umbral)

    def _sector_ya_visto(self, yaw):
        return any(abs(nc.wrap(yaw - v)) < self.mp.sector_visto_rad for v in self.sectores_vistos)

    def _revisar_qr(self, img):
        qrs = self._qr_en_imagen(img)
        for q in qrs:
            if q not in self.qrs_sector:
                self.qrs_sector.append(q)
        return qrs

    def _es_objetivo(self, res):
        return bool(res.nombre and res.nombre.strip().lower() == self.objetivo_norm and res.n_caras > 0)

    def _armar_resultado(self, yaw, res, qrs, encontrado):
        qr = qrs[0] if qrs else None
        return dict(
            nombre=res.nombre,
            score=res.score,
            seguro=res.seguro,
            n_caras=res.n_caras,
            ranking=res.ranking,
            qr=qr,
            qrs=list(qrs),
            encontrado=encontrado,
            yaw=yaw,
            vistos=[(y, r.nombre, r.score, r.seguro, qs) for y, r, qs in self.vistos],
        )

    def _resultado_final(self):
        if self.mejor_objetivo is not None:
            yaw, res, qrs = self.mejor_objetivo
            return self._armar_resultado(yaw, res, qrs, True)
        if self.vistos:
            yaw, res, qrs = max(self.vistos, key=lambda item: item[1].score)
            return self._armar_resultado(yaw, res, qrs, False)
        import facial
        vacio = facial.Resultado(nombre=(self.galeria.nombres[0] if getattr(self.galeria, "nombres", None) else None))
        return self._armar_resultado(self.yaw_prev, vacio, [], False)

    def paso(self, t, yaw, img):
        """Avanza un ciclo. Devuelve (angular, dict_resultado | None)."""
        dy = nc.wrap(yaw - self.yaw_prev)
        self.yaw_abs += abs(dy)
        self.yaw_prev = yaw

        if self.subestado == "BUSCAR":
            if t - self.t0 > self.limite or self.yaw_abs >= 2.0 * math.pi + 0.35:
                if self.mejor_objetivo is not None and not self.mejor_objetivo[2] and t - self.t0 < self.limite:
                    myaw, _, mqrs = self.mejor_objetivo
                    self.subestado = "BUSCAR_QR"
                    self.t_qr = t
                    self.qr_yaw0 = myaw
                    self.qr_dir = 1
                    self.qrs_sector = list(mqrs)
                    self.eventos.append(
                        f"vuelta 360 completa: busco QR para candidato {self.objetivo} (yaw={myaw:.2f})"
                    )
                    return 0.0, None
                self.eventos.append(f"fin barrido 360 (vistos={len(self.vistos)})")
                return 0.0, self._resultado_final()
            if img is not None:
                _, faces = self.rostros.detectar(img)
                if len(faces) and not self._sector_ya_visto(yaw):
                    self.subestado = "RECOLECTAR"
                    self.t_sub = t
                    self.sector_yaw = yaw
                    self.frames = [img]
                    self.qrs_sector = []
                    self._revisar_qr(img)
                    return 0.0, None
            return self.mp.busqueda_angular, None

        if self.subestado == "RECOLECTAR":
            if img is not None:
                _, faces = self.rostros.detectar(img)
                if len(faces):
                    self.frames.append(img)
                    self._revisar_qr(img)
            if len(self.frames) >= self.frames_por_cara or (t - self.t_sub) >= 3.0:
                res = self._identificar(self.frames)
                self.sectores_vistos.append(self.sector_yaw)
                self.vistos.append((self.sector_yaw, res, list(self.qrs_sector)))
                if self._es_objetivo(res):
                    if res.seguro:
                        self.eventos.append(
                            f"objetivo {res.nombre} confirmado ({res.score:.2f}) en yaw={self.sector_yaw:.2f}"
                        )
                        if self.qrs_sector:
                            return 0.0, self._armar_resultado(self.sector_yaw, res, self.qrs_sector, True)
                        self.subestado = "BUSCAR_QR"
                        self.t_qr = t
                        self.qr_yaw0 = yaw
                        self.qr_dir = 1
                        self.mejor_objetivo = (self.sector_yaw, res, list(self.qrs_sector))
                        return 0.0, None
                    if self.mejor_objetivo is None or res.score > self.mejor_objetivo[1].score:
                        self.mejor_objetivo = (self.sector_yaw, res, list(self.qrs_sector))
                    self.eventos.append(
                        f"candidato {res.nombre} dudoso ({res.score:.2f}); sigo revisando caras"
                    )
                else:
                    self.eventos.append(
                        f"vista {res.nombre or 'desconocido'} ({res.score:.2f}) en yaw={self.sector_yaw:.2f}; "
                        f"busco a {self.objetivo}"
                    )
                self.frames = []
                self.qrs_sector = []
                self.subestado = "BUSCAR"
            return 0.0, None

        # subestado == "BUSCAR_QR"
        if img is not None and self._revisar_qr(img):
            yaw_obj, res_obj, _ = self.mejor_objetivo
            self.eventos.append(f"QR de {res_obj.nombre} leido: {self.qrs_sector[0]}")
            return 0.0, self._armar_resultado(yaw_obj, res_obj, self.qrs_sector, True)
        if (t - self.t_qr) > self.mp.qr_barrido_max or (t - self.t0) > self.limite:
            yaw_obj, res_obj, _ = self.mejor_objetivo
            self.eventos.append(f"sin QR legible junto a {res_obj.nombre} tras {self.mp.qr_barrido_max:.1f}s")
            return 0.0, self._armar_resultado(yaw_obj, res_obj, self.qrs_sector, True)
        delta_qr = nc.wrap(yaw - self.qr_yaw0)
        if self.qr_dir > 0 and delta_qr >= 0.55:
            self.qr_dir = -1
        elif self.qr_dir < 0 and delta_qr <= -0.55:
            self.qr_dir = 1
        ang_qr = min(0.35, self.mp.busqueda_angular)
        return self.qr_dir * ang_qr, None


# --- ROS ---------------------------------------------------------------------------

def args_parser():
    from gestos_teleop import add_gesture_args
    from navegacion import add_ros_args

    ap = argparse.ArgumentParser(description="Mision completa A -> B -> A")
    ap.add_argument("--tarea", choices=("A", "B"), required=True)
    ap.add_argument("--modo", choices=("auto", "goal", "directo", "guiado"), default="auto",
                    help="Tarea A: 'goal' usa --goal X Y; 'directo' (auto sin --goal) va de flecha en "
                         "flecha hasta las caras; 'guiado' explora pasillos y cruces sin flecha")
    ap.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"),
                    help="Tarea A: B en m respecto a A (x frente, y izquierda); si se omite usa modo directo")
    ap.add_argument("--galeria", default=os.path.join(HERE, "galeria"))
    ap.add_argument("--persona", default="",
                    help="Nombre de la persona objetivo a buscar en B (gira 360, valida cara y escanea su QR)")
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
        self.guardian = None
        if mp.depth_guard:
            from profundidad import GuardianProfundidad
            self.guardian = GuardianProfundidad(
                log=lambda txt: print(f"[profundidad] {txt}", flush=True))
            self.lidar.extra = self.guardian.mezclar
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
        if not self.rclpy.ok():
            self.parar = True
            return
        try:
            drenar(lambda t: self.rclpy.spin_once(self.node, timeout_sec=t), timeout)
        except Exception:
            if not self.rclpy.ok():
                self.parar = True
            else:
                raise

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
            if self.guardian is not None:
                self.guardian.cerrar()
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


def comunes(s, m, cont, t, piloto=None):
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
        origen = "bumper" if s.bump() else "LiDAR"
        m.choque(origen)
        r, a = s.lidar.data
        pose = piloto.pose(s.odom.pose) if (piloto is not None and piloto.origin is not None) else s.odom.pose
        crumbs = piloto.crumbs if piloto is not None else None
        m.guardar_diagnostico(f"choque_{origen}", pose, r, a, crumbs, getattr(s, "guardian", None))
    return True


def _volcar_eventos_piloto(s, m, piloto):
    """Vuelca eventos del Piloto al log de la mision y guarda scan si hubo atasco."""
    atasco = False
    for _, txt in piloto.eventos:
        m.evento(txt)
        if "atasco" in txt:
            atasco = True
    piloto.eventos.clear()
    if atasco:
        if getattr(s, "guardian", None) is not None:
            s.guardian.forzar(time.monotonic())
        r, a = s.lidar.data
        pose = piloto.pose(s.odom.pose) if piloto.origin is not None else s.odom.pose
        m.guardar_diagnostico("atasco", pose, r, a, piloto.crumbs, getattr(s, "guardian", None))


def _crear_escaneo(s, mp, t):
    if mp.persona:
        return EscaneoObjetivo(
            mp.persona, s.rostros, s.galeria, mp, t,
            yaw0=s.odom.pose[2], umbral=s.umbral,
        )
    fn = None
    if getattr(s, "galeria", None) is not None:
        import facial
        umbral = getattr(s, "umbral", None) or facial.UMBRAL
        fn = lambda fr: facial.identificar(fr, s.rostros, s.galeria, umbral)  # noqa: E731
    return Escaneo(s.rostros, mp, t, identificar_fn=fn)


def _procesar_escaneo(escaneo, s, m, t, img):
    if isinstance(escaneo, EscaneoObjetivo):
        ang, res = escaneo.paso(t, s.odom.pose[2], img)
        while escaneo.eventos:
            m.evento(escaneo.eventos.pop(0))
        if res is None:
            return ang, False
        m.nombre = res["nombre"]
        m.score = res["score"]
        m.seguro = res["seguro"]
        m.qr_objetivo = res["qr"]
        for q in res["qrs"]:
            if q not in m.hitos_qr:
                m.hitos_qr.append(q)
                m.senales.append(dict(t=round(m.tiempo(), 2), tipo="qr", texto=q))
        m.evento(
            f"resultado escaneo objetivo ({m.objetivo or s.mp.persona}): "
            f"{m.nombre} ({m.score:.2f}, {'seguro' if m.seguro else 'DUDOSO'}), "
            f"QR={m.qr_objetivo or 'ninguno'}, encontrado={res['encontrado']}"
        )
        return 0.0, True
    ang, frames = escaneo.paso(t, img)
    if frames is None:
        return ang, False
    s.identificar(frames, m)
    return 0.0, True


def _texto_anuncio(m):
    nombre = m.nombre or "desconocido"
    if m.qr_objetivo:
        return f"{nombre}, QR {m.qr_objetivo}"
    return nombre


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
        if not comunes(s, m, cont, t, piloto):
            continue
        _volcar_eventos_piloto(s, m, piloto)
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
                escaneo = _crear_escaneo(s, mp, t)
                continue
            s.sender.send(lin, ang)
        elif m.estado == ESCANEAR:
            ang, listo = _procesar_escaneo(escaneo, s, m, t, img)
            s.sender.send(0.0, ang)
            if listo:
                s.sender.send(0.0, 0.0)
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
            s.anunciador.anunciar(_texto_anuncio(m))
            m.ir(FIN)
        if args.ver and img is not None:
            cv2.putText(img, f"{m.estado} {m.tiempo():.1f}s", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("mision A", img)
            cv2.waitKey(1)


class PilotoGuiado:
    """Coordinador de navegacion por pasillos + senales en B/N + exploracion de cruces
    para la Tarea A sin --goal. Sin ROS: opera sobre Piloto, pasillo.py y decision.py."""

    def __init__(self, piloto=None, mp=None, pp_pasillo=None):
        import decision as dc
        import pasillo as ps

        self.piloto = piloto or Piloto()
        self.mp = mp or MisionParams()
        self.pp_pasillo = pp_pasillo or ps.PasilloParams()
        self.dp = dc.DecisionParams(
            senal_prohibido=self.mp.senal_prohibido,
            senal_bandera=self.mp.senal_bandera,
            senal_qr=self.mp.senal_qr,
            regreso_modo=self.mp.regreso_modo,
            flechas_marco=self.mp.flechas_marco,
        )
        self.rumbo_pasillo = 0.0
        self.ultimo_cruce_xy = None
        self.ultima_accion = None
        self.t_ultima_decision = -1e9
        self.cruce_anterior_key = None
        self.cruce_anterior_miga_idx = 0
        self.retrocediendo_hasta_idx = None
        self._retroceso_upto = 0
        self._retroceso_fin_idx = 0
        self.dir_entrada = None
        self.visitas_cruce = {}     # key_xy -> {dir_cardinal: conteo}
        self.agotadas_cruce = {}    # key_xy -> set(dir_cardinal)
        self.senal_pendiente = None
        self.t_senal = -1e9
        self.cont_bandera = 0
        self.min_dist_cruce = 0.85

    def iniciar(self, odom_pose, ranges, angles):
        self.piloto.fijar_origen(odom_pose, ranges, angles)
        self.rumbo_pasillo = 0.0
        self.ultimo_cruce_xy = (0.0, 0.0)
        self.cruce_anterior_miga_idx = 0
        self.retrocediendo_hasta_idx = None

    @staticmethod
    def _dir_cardinal(rad):
        return int(round(nc.wrap(rad) / (math.pi / 2.0))) % 4

    def _clave_cruce(self, x, y, radio=0.80):
        """Reutiliza el cruce guardado mas cercano a < radio m; si no hay, crea uno nuevo."""
        mejor_k, mejor_d = None, radio
        for k in set(self.visitas_cruce) | set(self.agotadas_cruce):
            d = math.hypot(x - k[0], y - k[1])
            if d < mejor_d:
                mejor_k, mejor_d = k, d
        return mejor_k if mejor_k is not None else (round(x, 2), round(y, 2))

    @classmethod
    def _mapa_dirs(cls, rumbo):
        return {
            "recto": cls._dir_cardinal(rumbo),
            "izq": cls._dir_cardinal(rumbo + math.pi / 2.0),
            "der": cls._dir_cardinal(rumbo - math.pi / 2.0),
        }

    def registrar_senales(self, t, senales_vistas, mision=None):
        """Procesa las senales detectadas en un frame de camara. Devuelve True si se confirmo
        la llegada a B por bandera (`bandera_frames` seguidos)."""
        vio_bandera = False
        tomo_nav = False
        for s in senales_vistas or ():
            if s.tipo == "qr":
                if self.mp.senal_qr == "registrar" and mision is not None and s.texto not in mision.hitos_qr:
                    mision.hitos_qr.append(s.texto)
                    mision.senales.append(dict(t=round(mision.tiempo(), 2), tipo="qr", texto=s.texto))
                    mision.evento(f"hito QR registrado: {s.texto}")
            elif s.tipo == "bandera":
                vio_bandera = True
            elif s.tipo in ("izq", "der", "prohibido") and not tomo_nav:
                # detectar() ordena de mayor a menor area: priorizar la senal mas cercana
                tomo_nav = True
                prev = getattr(self.senal_pendiente, "tipo", None)
                if prev != s.tipo or (t - self.t_senal) > 2.0:
                    if mision is not None:
                        mision.senales.append(dict(t=round(mision.tiempo(), 2), tipo=s.tipo,
                                                   confianza=round(s.confianza, 2)))
                        mision.evento(f"senal detectada: {s.tipo} (conf={s.confianza:.2f})")
                self.senal_pendiente = s
                self.t_senal = t

        if vio_bandera and self.mp.senal_bandera == "llegada_b":
            self.cont_bandera += 1
            if self.cont_bandera >= self.mp.bandera_frames:
                if mision is not None:
                    mision.senales.append(dict(t=round(mision.tiempo(), 2), tipo="bandera"))
                return True
        else:
            self.cont_bandera = 0
        return False

    def paso(self, t, odom_pose, ranges, angles, mision=None, bumper_hit=False):
        """Un ciclo de navegacion guiada en estado SEGUIR / DECIDIR.
        Devuelve (lin, ang, info)."""
        import decision as dc
        import pasillo as ps

        pose = self.piloto.pose(odom_pose)
        r, a = nc.clean_scan(ranges, angles, self.piloto.nav.scan_min)
        pto = ps.analizar_pasillo(r, a, self.pp_pasillo, self.piloto.nav)

        # Caducar senal pendiente si quedo muy atras
        if self.senal_pendiente is not None and (t - self.t_senal) > 6.5:
            self.senal_pendiente = None

        alineado = abs(nc.wrap(pose[2] - self.rumbo_pasillo)) < math.radians(28.0)
        dist_ultimo = (math.inf if self.ultimo_cruce_xy is None
                       else math.hypot(pose[0] - self.ultimo_cruce_xy[0],
                                       pose[1] - self.ultimo_cruce_xy[1]))
        lejos_de_ultimo = dist_ultimo >= self.min_dist_cruce
        frente_bloqueado = pto.dist_frente < self.pp_pasillo.block_front
        # Tras retroceder en un callejon, permitir salir >= min_dist_cruce; tras girar, esperar >= 2.5 s
        puede_frente = (
            frente_bloqueado
            and (t - self.t_ultima_decision) >= 2.5
            and not (self.ultima_accion == "retroceder" and not lejos_de_ultimo)
        )

        # Si estamos retrocediendo por migas, comprobar llegada al cruce objetivo antes de decidir
        if self.retrocediendo_hasta_idx is not None:
            if len(self.piloto.crumbs) <= self.retrocediendo_hasta_idx:
                self.retrocediendo_hasta_idx = None
            else:
                objetivo_cruce = self.piloto.crumbs[self.retrocediendo_hasta_idx]
                if math.hypot(pose[0] - objetivo_cruce[0], pose[1] - objetivo_cruce[1]) < 0.45:
                    self.piloto.crumbs.prune_branch(self.retrocediendo_hasta_idx + 1, self._retroceso_fin_idx)
                    self.retrocediendo_hasta_idx = None

        # Si hay cruce nuevo o pared frontal y ya estamos alineados en el pasillo, decidir
        puede_cruce = pto.en_cruce and lejos_de_ultimo and self.retrocediendo_hasta_idx is None
        if alineado and (puede_cruce or puede_frente):
            key = self._clave_cruce(pose[0], pose[1])
            vis_mundo = self.visitas_cruce.setdefault(key, {})
            ago_mundo = self.agotadas_cruce.setdefault(key, set())
            mapa_d = self._mapa_dirs(self.rumbo_pasillo)

            vis_rel = {op: vis_mundo.get(d_m, 0) for op, d_m in mapa_d.items()}
            ago_rel = {op for op, d_m in mapa_d.items() if d_m in ago_mundo}
            avisos = []

            if mision is not None and mision.estado != DECIDIR:
                mision.ir(DECIDIR, f"opciones={sorted(pto.opciones)}")

            accion = dc.decidir(
                pto.opciones,
                senal=self.senal_pendiente,
                visitado=vis_rel,
                avisos=avisos,
                agotadas=ago_rel,
                params=self.dp,
            )
            for op in ago_rel:
                ago_mundo.add(mapa_d[op])
            self.senal_pendiente = None

            if mision is not None:
                for av in avisos:
                    mision.evento(av)
                mision.decisiones.append(dict(
                    t=round(mision.tiempo(), 2),
                    pose=[round(pose[0], 2), round(pose[1], 2), round(pose[2], 2)],
                    opciones=sorted(pto.opciones),
                    accion=accion,
                ))

            if accion == "llegada_b":
                if mision is not None:
                    mision.ir(LLEGADA_B, "senal bandera en cruce")
                return 0.0, 0.0, dict(pose=pose, dist=0.0, front=pto.dist_frente,
                                      nearest=nc.nearest(r), estado="llegada_b", llegada=False, llegada_b=True)

            if accion == "retroceder":
                # Marcar la rama por la que entramos al cruce anterior como agotada
                if self.cruce_anterior_key is not None and self.dir_entrada is not None:
                    self.agotadas_cruce.setdefault(self.cruce_anterior_key, set()).add(self.dir_entrada)
                self.rumbo_pasillo = nc.wrap(self.rumbo_pasillo + math.pi)
                if len(self.piloto.crumbs) > 1:
                    self.retrocediendo_hasta_idx = min(self.cruce_anterior_miga_idx, len(self.piloto.crumbs) - 2)
                    self._retroceso_upto = len(self.piloto.crumbs) - 2
                    self._retroceso_fin_idx = len(self.piloto.crumbs)
            else:
                if self.retrocediendo_hasta_idx is not None:
                    self.piloto.crumbs.prune_branch(self.retrocediendo_hasta_idx + 1, self._retroceso_fin_idx)
                    self.retrocediendo_hasta_idx = None
                d_elegida = mapa_d[accion]
                d_origen = self._dir_cardinal(self.rumbo_pasillo + math.pi)
                vis_mundo[d_origen] = max(vis_mundo.get(d_origen, 0), 1)
                vis_mundo[d_elegida] = vis_mundo.get(d_elegida, 0) + 1
                self.cruce_anterior_key = key
                self.cruce_anterior_miga_idx = max(0, len(self.piloto.crumbs) - 1)
                self.dir_entrada = d_elegida
                if accion == "izq":
                    self.rumbo_pasillo = nc.wrap(self.rumbo_pasillo + math.pi / 2.0)
                elif accion == "der":
                    self.rumbo_pasillo = nc.wrap(self.rumbo_pasillo - math.pi / 2.0)

            self.ultimo_cruce_xy = (pose[0], pose[1])
            self.ultima_accion = accion
            self.t_ultima_decision = t
            if mision is not None and mision.estado != SEGUIR:
                mision.ir(SEGUIR, f"accion={accion}")

        # Si estamos retrocediendo de un callejon, seguir las migas de vuelta al cruce anterior
        self.piloto.retorno = False
        if self.retrocediendo_hasta_idx is not None and len(self.piloto.crumbs) > self.retrocediendo_hasta_idx:
            wp_idx = nc.pick_waypoint(
                self.piloto.crumbs, pose, r, a,
                upto=min(self._retroceso_upto, len(self.piloto.crumbs) - 1),
                lookahead=2.0, p=self.piloto.nav,
            )
            wp_idx = max(wp_idx, self.retrocediendo_hasta_idx)
            while (wp_idx > self.retrocediendo_hasta_idx
                   and math.hypot(pose[0] - self.piloto.crumbs[wp_idx][0],
                                  pose[1] - self.piloto.crumbs[wp_idx][1]) < 0.25):
                wp_idx -= 1
            self._retroceso_upto = wp_idx
            self.piloto.meta = self.piloto.crumbs[wp_idx]
        else:
            # Zanahoria local a 1.4 m en la direccion del pasillo elegido; Piloto aporta VFH-lite y migas
            self.piloto.meta = (
                pose[0] + 1.4 * math.cos(self.rumbo_pasillo),
                pose[1] + 1.4 * math.sin(self.rumbo_pasillo),
            )
        lin, ang, info = self.piloto.paso(t, odom_pose, ranges, angles, bumper_hit=bumper_hit)
        info["llegada_b"] = False
        return lin, ang, info


def tarea_a_guiada(args, s, m, piloto):
    """Tarea A sin --goal: sigue pasillos y senales hasta ver la bandera en B,
    escanea el rostro con facial.py, regresa a A por migas+ICP y anuncia."""
    import cv2

    import senales_forma as sf

    mp = s.mp
    cont = ContadorChoques(mp.choque_dist)
    guiado = PilotoGuiado(piloto, mp)
    guiado.iniciar(s.odom.pose, *s.lidar.data)
    if mp.cuenta:
        m.evento(f"salgo en {mp.cuenta:.0f} s")
        time.sleep(mp.cuenta)
    m.ir(SEGUIR, "modo guiado por senales + LiDAR")
    escaneo, t_cam = None, -1e9

    while m.activa():
        s.spin(1.0 / mp.rate)
        t = time.monotonic()
        if not comunes(s, m, cont, t, piloto):
            continue
        _volcar_eventos_piloto(s, m, piloto)

        img = s.frame_nuevo() if m.estado in (SEGUIR, DECIDIR, LLEGADA_B, ESCANEAR) or args.ver else None

        if m.estado in (SEGUIR, DECIDIR):
            t_p = m.perfil.reloj()
            if img is not None and (t - t_cam) >= mp.cam_periodo:
                t_cam = t
                # Filtro rapido en preview antes de correr el detector completo (ahorro CPU Pi)
                senales_vistas = sf.detectar(img, esfuerzo_qr=1) if sf.buscar_candidatos(img) else []
                if guiado.registrar_senales(t, senales_vistas, m):
                    s.sender.send(0.0, 0.0)
                    m.ir(LLEGADA_B, "bandera de cuadros confirmada")
                    m.ir(ESCANEAR, "analizando rostros en B")
                    escaneo = _crear_escaneo(s, mp, t)
                    m.perfil.registrar(t_p)
                    continue
            lin, ang, info = guiado.paso(t, s.odom.pose, *s.lidar.data, mision=m, bumper_hit=s.bump())
            m.perfil.registrar(t_p)
            if info.get("llegada_b"):
                s.sender.send(0.0, 0.0)
                m.ir(ESCANEAR, "analizando rostros en B")
                escaneo = _crear_escaneo(s, mp, t)
                continue
            s.sender.send(lin, ang)
        elif m.estado == ESCANEAR:
            ang, listo = _procesar_escaneo(escaneo, s, m, t, img)
            s.sender.send(0.0, ang)
            if listo:
                s.sender.send(0.0, 0.0)
                if not args.ver:
                    s.cam.pausar()
                piloto.volver()
                m.ir(VOLVER_A_A, f"regreso={mp.regreso_modo}")
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
            s.anunciador.anunciar(_texto_anuncio(m))
            m.ir(FIN)
        if args.ver and img is not None:
            cv2.putText(img, f"{m.estado} {m.tiempo():.1f}s", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("mision A guiada", img)
            cv2.waitKey(1)


def _volcar_directo(d, m):
    """Eventos, decisiones y QR de PilotoDirecto al registro de la mision."""
    for txt in d.eventos:
        m.evento(txt)
    d.eventos.clear()
    for dec in d.decisiones[len(m.decisiones):]:
        m.decisiones.append(dict(dec, t=round(m.tiempo(), 2)))
    for q in d.qrs:
        if q not in m.hitos_qr:
            m.hitos_qr.append(q)
            m.senales.append(dict(t=round(m.tiempo(), 2), tipo="qr", texto=q))


def tarea_a_directa(args, s, m, piloto, dp=None):
    """Tarea A sin --goal, modo directo (directo.py): recto hasta la flecha, giro en el sitio,
    recto ... hasta las caras (bandera cercana, caras al frente o callejon con caras);
    escanea, vuelve a A por migas+ICP a v_regreso y anuncia."""
    import cv2

    import directo as dr
    import senales_forma as sf

    mp = s.mp
    dp = dp or dr.DirectoParams()
    cont = ContadorChoques(mp.choque_dist)
    d = dr.PilotoDirecto(piloto, mp, dp)
    d.iniciar(s.odom.pose, *s.lidar.data)
    if mp.cuenta:
        m.evento(f"salgo en {mp.cuenta:.0f} s")
        time.sleep(mp.cuenta)
    m.ir(SEGUIR, f"modo directo (v_recto {dp.v_recto:.2f} m/s)")
    escaneo, t_cam, frente = None, -1e9, math.inf
    ultimo_img, n_dec = None, 0     # foto de cada decision para depurar (npz en logs/)

    while m.activa():
        s.spin(1.0 / mp.rate)
        t = time.monotonic()
        if not comunes(s, m, cont, t, piloto):
            continue
        _volcar_eventos_piloto(s, m, piloto)
        _volcar_directo(d, m)

        img = s.frame_nuevo() if m.estado in (SEGUIR, ESCANEAR) or args.ver else None
        if img is not None:
            ultimo_img = img

        if m.estado == SEGUIR:
            t_p = m.perfil.reloj()
            r, a = s.lidar.data
            if img is not None and (t - t_cam) >= mp.cam_periodo:
                t_cam = t
                senales = sf.detectar(img, esfuerzo_qr=1) if sf.buscar_candidatos(img) else []
                d.ver_senales(t, senales, img.shape[1], s.odom.pose, r, a)
                # Caras solo cerca del fondo o mirando un callejon: en marcha cuestan CPU de la
                # Pi y una cara del publico a un costado no debe cortar el recorrido.
                if d.estado == dr.MIRAR or frente < dp.d_lento:
                    d.ver_caras(t, s.vista_cara(img))
            lin, ang, info = d.paso(t, s.odom.pose, r, a, bumper_hit=s.bump())
            if len(d.decisiones) > n_dec or info["accion"] == "llegada_b":
                n_dec = len(d.decisiones)
                dec = d.decisiones[-1] if d.decisiones else {}
                motivo = "llegada" if info["accion"] == "llegada_b" else f"decision_{dec.get('accion')}"
                m.guardar_diagnostico(motivo, info["pose"], r, a, imagen=ultimo_img,
                                      nota=json.dumps(dec, default=str))
            if "front" in info:   # libre inflado de Piloto -> distancia aproximada a la pared
                frente = info.get("frente_crudo", info["front"] + piloto.nav.inflate)
            m.perfil.registrar(t_p)
            if info["accion"] == "llegada_b":
                s.sender.send(0.0, 0.0)
                _volcar_directo(d, m)
                m.ir(LLEGADA_B, d.motivo_llegada or "")
                m.ir(ESCANEAR, "analizando rostros en B")
                escaneo = _crear_escaneo(s, mp, t)
                continue
            s.sender.send(lin, ang)
        elif m.estado == ESCANEAR:
            ang, listo = _procesar_escaneo(escaneo, s, m, t, img)
            s.sender.send(0.0, ang)
            if listo:
                s.sender.send(0.0, 0.0)
                if not args.ver:
                    s.cam.pausar()
                piloto.pp.max_linear = dp.v_regreso
                piloto.volver()
                m.ir(VOLVER_A_A, f"regreso por migas a {dp.v_regreso:.2f} m/s")
        elif m.estado == VOLVER_A_A:
            t_p = m.perfil.reloj()
            lin, ang, info = d.regreso(t, s.odom.pose, *s.lidar.data, bumper_hit=s.bump())
            m.perfil.registrar(t_p)
            if info["llegada"]:
                s.sender.stop()
                m.ir(ANUNCIAR, f"a {info['dist']:.2f} m de A")
                continue
            s.sender.send(lin, ang)
        elif m.estado == ANUNCIAR:
            s.sender.stop()
            s.anunciador.anunciar(_texto_anuncio(m))
            m.ir(FIN)
        if args.ver and img is not None:
            cv2.putText(img, f"{m.estado}/{d.estado} {m.tiempo():.1f}s", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("mision A directa", img)
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
    from directo import DirectoParams

    piloto, mp, dp = Piloto(), MisionParams(), DirectoParams()
    config = args.config or (os.path.join(HERE, "config.json")
                             if os.path.isfile(os.path.join(HERE, "config.json")) else None)
    valores = apply_params([piloto.nav, piloto.pp, mp, dp], config, args.param)
    if args.persona:
        mp.persona = args.persona.strip()

    m = Mision(args.tarea, os.path.join(HERE, "logs"))
    m.objetivo = mp.persona or None
    m.evento(f"tarea {args.tarea} | config {config} | ajustes {valores}"
             + (f" | objetivo {m.objetivo}" if m.objetivo else ""))
    s = None
    try:
        s = Sistema(args, mp)
        if not esperar_sensores(s, m):
            raise SystemExit("No llegan /scan y /odom: revisa la conexion (tb4_check.sh).")
        if args.tarea == "A":
            if args.modo == "guiado":
                tarea_a_guiada(args, s, m, piloto)
            elif args.modo == "directo" or (args.modo == "auto" and args.goal is None):
                tarea_a_directa(args, s, m, piloto, dp)
            else:
                tarea_a(args, s, m, piloto)
        else:
            tarea_b(args, s, m)
    except KeyboardInterrupt:
        m.ir(PARADA, "Ctrl+C")
    finally:
        if s is not None:
            s.cerrar()
        res = m.resultado()
        res["params"] = dict(nav=asdict(piloto.nav), piloto=asdict(piloto.pp), mision=asdict(mp),
                             directo=asdict(dp))
        m.evento(f"RESULTADO: {json.dumps({k: v for k, v in res.items() if k != 'params'})}")
        if args.resultado:
            with open(args.resultado, "w", encoding="utf-8") as f:
                json.dump(res, f, indent=2, ensure_ascii=False)
        m.cerrar()


if __name__ == "__main__":
    main()
