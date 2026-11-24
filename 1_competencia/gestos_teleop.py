"""
Teleoperacion del TurtleBot 4 con POSES de la mano, SIN teclado (Tarea B):

  palma abierta (mantener ARM_HOLD s)  -> ARMAR
  puno cerrado                         -> PARAR (al instante); mantenido PUNO_HOLD s -> DESARMAR
  like  (pulgar arriba)                -> adelante
  pistola apuntando a la izq. / der.   -> girar izquierda / derecha
  dislike (pulgar abajo)               -> retroceder (con freno del LiDAR atras)
  V con dos dedos (mantener ACTION_HOLD s) -> ACCION (en mision.py: escanear la foto en
                                          B y, despues, anunciar el nombre en A)
  sin mano / pose dudosa / mano lejos  -> parar

El robot se mueve MIENTRAS mantienes la pose y ARRANCA DESARMADO. Con --lidar (por
defecto si no es --dry) no avanza si hay algo a menos de stop_dist al frente.
Usa MediaPipe Hand Landmarker (21 puntos) de mano_teleop.py.

Uso:
  python3 gestos_teleop.py --dry --no-robot-cam   # solo ver poses (sin ROS: corre en Mac)
  python3 gestos_teleop.py --dry --no-robot-cam --gestos-mejorados
                                                  # ademas muestra por frame confianza,
                                                  # estabilidad y el comando que enviaria
  python3 gestos_teleop.py                        # robot real
  python3 gestos_teleop.py --lineal 0.2 --angular 1.0 --clahe

Para salir: cerrar la ventana o Ctrl+C en la terminal (no se usa el teclado en la
ventana). La Tarea B completa (con reconocimiento y anuncio) es: ./tarea_b.sh
"""
import argparse
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from comun import LIMIT_ANGULAR, LIMIT_LINEAR, Sender, clamp
from mano_teleop import (HAND_BONES, PANEL_H, HandDetector, RobotCamera, fit_height,
                         open_camera, robot_panel)

STABLE_FRAMES = 3      # frames seguidos con la misma pose antes de aceptarla
THUMB_OPEN = 0.55      # pulgar estirado si |punta - base del indice| > 0.55 * tamano mano
MIN_HAND_PX = 25       # mano mas chica (muneca -> base del medio) = muy lejos, no se confia
ARM_HOLD = 1.0         # s con la palma para armar
ACTION_HOLD = 1.5      # s con la V para la accion
PUNO_HOLD = 0.5        # s con el puno para DESARMAR (frena enseguida; un puno "de paso" al
                       # cambiar de like a dislike cruza ~0.1-0.2 s por puno y desarmaba)
GRACIA = 0.15          # s que se sostiene el ultimo movimiento si MediaPipe pierde la mano
                       # 1-2 cuadros (antes cada parpadeo frenaba el robot a tirones)

# --- Gestos mejorados (--gestos-mejorados). Por defecto APAGADOS: sin la opcion todo
# se comporta exactamente como antes. Se SUMAN a PUNO_HOLD y GRACIA, no los reemplazan.
LIKE_PULGAR_MIN = 0.45  # M1: like exige |punta pulgar - base indice| >= 0.45 * tamano mano
                        #     (un puno con el pulgar apoyado encima queda en ~0.15-0.32)
T_MOVER = 0.30          # M2: s seguidos con la misma pose para aceptarla (salvo puno)
T_PARAR = 0.15          # M2: s seguidos de puno para darlo por ESTABLE (o STABLE_FRAMES,
                        #     lo que llegue antes). El robot frena desde el 1er cuadro de puno
                        #     igual que siempre; DESARMAR sigue pidiendo PUNO_HOLD s desde ahi.
MIN_FRAMES_PARAR = 2    # M2: y al menos estos frames de puno


@dataclass
class ParamsGestos:
    """Opciones de las mejoras. Los valores por defecto (None/False) = comportamiento
    original. ParamsGestos.mejorados() = M1 + M2. M7 (rechazar_parcial) va aparte
    con --rechazar-mano-parcial, desactivado por defecto."""
    like_pulgar_min: Optional[float] = None  # M1 (None = regla original del like)
    t_mover: Optional[float] = None          # M2 (None = STABLE_FRAMES como antes)
    t_parar: Optional[float] = None          # M2 (None = igual que t_mover)
    min_frames_parar: int = MIN_FRAMES_PARAR
    rechazar_parcial: bool = False           # M7: puntos fuera del cuadro -> 'nada'

    @classmethod
    def mejorados(cls, **cambios):
        base = dict(like_pulgar_min=LIKE_PULGAR_MIN, t_mover=T_MOVER, t_parar=T_PARAR)
        base.update({k: v for k, v in cambios.items() if v is not None})
        return cls(**base)

    def activo(self):
        return self != ParamsGestos()

# pose -> (texto en pantalla, (factor lineal, factor angular))
POSES = {
    "like": ("LIKE -> ADELANTE", (1, 0)),
    "pistola_izq": ("<-- PISTOLA IZQUIERDA", (0, 1)),
    "pistola_der": ("PISTOLA DERECHA -->", (0, -1)),
    "dislike": ("DISLIKE -> RETROCESO", (-1, 0)),
    "palma": ("PALMA (mantener: ARMAR)", (0, 0)),
    "puno": ("PUNO -> PARAR Y DESARMAR", (0, 0)),
    "victoria": ("V (mantener: ACCION)", (0, 0)),
    "nada": ("(ninguna pose)", (0, 0)),
}

# (punta, articulacion media) de indice, medio, anular, menique
FINGERS = [(8, 6), (12, 10), (16, 14), (20, 18)]
MCP_JOINTS = [5, 9, 13, 17]


def classify_pose(lm, like_pulgar_min=None):
    """lm: (21, 2) en px (frame ya espejado). Retorna una clave de POSES.
    like_pulgar_min (M1): si no es None, el like exige ademas el pulgar separado de la
    mano; si no lo esta, la mano cerrada cuenta como puno (parar)."""
    wrist = lm[0]
    size = np.linalg.norm(lm[9] - wrist)  # muneca -> base del dedo medio
    if size < MIN_HAND_PX:
        return "nada"

    # Evaluacion robusta de flexion de cada dedo (tolerancia angular y oclusion)
    folded = []
    for (tip, pip), mcp in zip(FINGERS, MCP_JOINTS):
        d_tip_w = np.linalg.norm(lm[tip] - wrist)
        d_pip_w = np.linalg.norm(lm[pip] - wrist)
        d_tip_mcp = np.linalg.norm(lm[tip] - lm[mcp])
        d_pip_mcp = np.linalg.norm(lm[pip] - lm[mcp])
        folded.append(d_tip_w < d_pip_w * 1.15 or d_tip_mcp < d_pip_mcp * 1.30)

    # Para puño / like / dislike: indice, medio y anular doblados (meñique flexible por oclusion)
    all_folded = folded[0] and folded[1] and folded[2] and (folded[3] or sum(folded) >= 3)

    # Analisis direccional del pulgar
    dx, dy = lm[4] - lm[2]  # base (2) -> punta (4) del pulgar
    thumb_up = (dy < -0.25 * size) and (lm[4][1] < lm[5][1]) and (abs(dy) > abs(dx) * 0.6)
    thumb_down = (dy > 0.25 * size) and (lm[4][1] > lm[2][1]) and (abs(dy) > abs(dx) * 0.6)
    thumb_open = np.linalg.norm(lm[4] - lm[5]) >= THUMB_OPEN * size
    if like_pulgar_min is not None:  # M1
        thumb_up = thumb_up and np.linalg.norm(lm[4] - lm[5]) >= like_pulgar_min * size

    if all_folded:
        if thumb_up:
            return "like"
        if thumb_down:
            return "dislike"
        # Si los dedos estan doblados y el pulgar no apunta ni arriba ni abajo: es PUÑO
        return "puno"
    elif not folded[0] and all(folded[1:]):
        # Pistola: indice estirado, apuntando de lado
        p_dx, p_dy = lm[8] - lm[5]
        if abs(p_dx) > abs(p_dy):
            return "pistola_der" if p_dx > 0 else "pistola_izq"
    elif not folded[0] and not folded[1] and folded[2] and folded[3]:
        return "victoria"
    elif not any(folded) and thumb_open:
        return "palma"
    return "nada"


def fuera_de_cuadro(lm, shape):
    """M7: algun punto de la mano cae fuera de la imagen (MediaPipe los inventa)."""
    h, w = shape[:2]
    return bool((lm[:, 0] < 0).any() or (lm[:, 0] >= w).any()
                or (lm[:, 1] < 0).any() or (lm[:, 1] >= h).any())


class ControlGestos:
    """Pose por frame -> (direccion, evento). Eventos: 'armado', 'desarmado', 'accion'.
    Solo se mueve armado y con una pose estable; cualquier duda = quieto.

    t_mover=None: confirmacion original (STABLE_FRAMES frames seguidos).
    t_mover (M2): una pose se acepta tras t_mover s seguidos Y la confirmacion original
      (`stable` frames): nunca es mas facil que antes, a ningun fps. El puno se da por
      estable con t_parar s O `stable` frames (nunca mas tarde que antes) y como minimo
      min_frames_parar frames. Frenar sigue siendo inmediato y desarmar sigue pidiendo
      puno_hold s de puno estable. Un parpadeo dentro de GRACIA (sin mano / 'nada' mientras
      se mueve) no reinicia los t_mover s, igual que no corta el movimiento en el original."""

    def __init__(self, stable=STABLE_FRAMES, arm_hold=ARM_HOLD, action_hold=ACTION_HOLD,
                 puno_hold=PUNO_HOLD, gracia=GRACIA, t_mover=None, t_parar=None,
                 min_frames_parar=MIN_FRAMES_PARAR):
        self.stable, self.arm_hold, self.action_hold = stable, arm_hold, action_hold
        self.puno_hold, self.gracia = puno_hold, gracia
        self.t_mover = t_mover
        self.t_parar = t_mover if t_parar is None else t_parar
        self.min_frames_parar = min_frames_parar
        self.armed = False
        self.last, self.count = None, 0
        self.run_pose, self.run_t0, self.t = None, 0.0, 0.0   # M2: racha medida en s
        self.held, self.held_t0, self.fired = None, 0.0, False
        self.mov, self.t_mov = (0, 0), -1e9      # ultimo movimiento y cuando se vio

    def _seguir_racha(self, pose, t):
        """M2: desde cuando se ve la pose actual. Un parpadeo que la GRACIA ya cubre (sin mano
        o 'nada' 1-2 cuadros mientras se mueve) no reinicia la racha: si no, con t_mover el
        robot volveria a frenar a tirones (lo que arreglo GRACIA)."""
        parpadeo = (pose in (None, "nada") and self.armed and self.mov != (0, 0)
                    and t - self.t_mov <= self.gracia)
        if not parpadeo and (pose != self.run_pose or self.count == 0):
            self.run_pose, self.run_t0 = pose, t

    def _confirmada(self, pose, t):
        if pose is None:
            return False
        if self.t_mover is None:
            return self.count >= self.stable
        if pose == "puno":  # nunca mas tarde que antes (OR con los frames de siempre)
            return self.count >= self.min_frames_parar and (
                t - self.run_t0 >= self.t_parar or self.count >= self.stable)
        # mover/armar/accion: nunca mas facil que antes (AND con los frames de siempre)
        return self.count >= self.stable and t - self.run_t0 >= self.t_mover

    def update(self, pose, t):
        if self.t_mover is not None:
            self._seguir_racha(pose, t)
        self.count = self.count + 1 if pose == self.last else 1
        self.last = pose
        self.t = t
        stable = pose if self._confirmada(pose, t) else None
        if stable != self.held:
            self.held, self.held_t0, self.fired = stable, t, False
        held_for = t - self.held_t0
        evento = None
        if (stable == "puno" and self.armed and not self.fired
                and held_for >= self.puno_hold):
            self.armed, self.fired, evento = False, True, "desarmado"
        elif (stable == "palma" and not self.armed and not self.fired
              and held_for >= self.arm_hold):
            self.armed, self.fired, evento = True, True, "armado"
        elif (stable == "victoria" and self.armed and not self.fired
              and held_for >= self.action_hold):
            self.fired, evento = True, "accion"
        direction = POSES[stable][1] if self.armed and stable in POSES else (0, 0)
        if direction != (0, 0):
            self.mov, self.t_mov = direction, t
        elif (self.armed and self.mov != (0, 0) and t - self.t_mov <= self.gracia
              and (pose in (None, "nada") or (pose in POSES and POSES[pose][1] == self.mov))):
            # parpadeo del detector (sin mano o pose dudosa 1-2 cuadros, o la MISMA pose que
            # vuelve y todavia no junta STABLE_FRAMES): sigue igual un instante. Un puno /
            # dislike / otra pose NO entra aca: frena al momento.
            direction = self.mov
        else:
            self.mov = (0, 0)
        return direction, evento

    def progreso(self, t):
        """0..1 de cuanto falta para disparar la pose mantenida (para el HUD)."""
        need = {"palma": self.arm_hold, "victoria": self.action_hold,
                "puno": self.puno_hold if self.armed else None}.get(self.held)
        if need is None or self.fired:
            return 0.0
        return min((t - self.held_t0) / need, 1.0)


def apply_clahe(frame):
    """Ecualiza la luminancia: ayuda con contraluz o laboratorio oscuro."""
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lab[:, :, 0] = cv2.createCLAHE(2.0, (8, 8)).apply(lab[:, :, 0])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


class DetectorMano(HandDetector):
    """HandDetector de mano_teleop.py + dos extras, sin cambiar lo que detecta:
      - guarda la confianza (score de handedness de MediaPipe) y el lado de la ultima mano;
      - find(frame, ts_ms) acepta un timestamp externo (para reproducir videos grabados).
    Con ts_ms=None hace exactamente lo mismo que HandDetector.find (reloj time.monotonic)."""

    def __init__(self):
        super().__init__()
        self.score, self.lado = None, None
        self.lm = None  # ultimos 21 puntos devueltos (para el banco de evaluacion)

    def find(self, frame, ts_ms=None):
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        now_ms = int(time.monotonic() * 1000) if ts_ms is None else int(ts_ms)
        # el modo video exige timestamps (ms) estrictamente crecientes
        self.last_ts = max(now_ms, self.last_ts + 1)
        result = self.landmarker.detect_for_video(
            self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), self.last_ts)
        if not result.hand_landmarks:
            self.score, self.lado, self.lm = None, None, None
            return None
        if result.handedness and result.handedness[0]:
            cat = result.handedness[0][0]
            self.score, self.lado = float(cat.score), cat.category_name
        else:
            self.score, self.lado = None, None
        self.lm = np.array([[lm.x * w, lm.y * h] for lm in result.hand_landmarks[0]], np.float32)
        return self.lm


def nuevo_control(params=None):
    """ControlGestos configurado segun ParamsGestos (None = original). PUNO_HOLD y GRACIA
    quedan siempre con sus valores de siempre."""
    p = params or ParamsGestos()
    return ControlGestos(t_mover=p.t_mover, t_parar=p.t_parar,
                         min_frames_parar=p.min_frames_parar)


class Gestos:
    """Webcam + MediaPipe + ControlGestos. leer(t) -> (frame, direccion, evento, pose).

    cap: fuente de frames ya abierta (p. ej. cv2.VideoCapture de un video grabado);
         None = abrir la webcam cam_index como siempre.
    reloj_t: True = MediaPipe usa t (s) como timestamp en vez de time.monotonic(), para
             que un video reproducido avance con SU tiempo. Por defecto False (como antes).
    mejorados / params: gestos mejorados (M1 like con pulgar separado, M2 confirmacion en
             segundos asimetrica). Por defecto APAGADOS. M7 (mano parcial = nada) solo
             con params.rechazar_parcial / --rechazar-mano-parcial.
             params (ParamsGestos) tiene prioridad sobre mejorados."""

    def __init__(self, cam_index, clahe=False, cap=None, reloj_t=False, mejorados=False,
                 params=None):
        self.cap = open_camera(cam_index) if cap is None else cap
        if self.cap is None:
            raise SystemExit(f"No se pudo abrir la webcam {cam_index}. En WSL hay que pasarla con "
                             "usbipd (ver seguimiento_vision/README.md); prueba tambien --cam 1.")
        self.clahe = clahe
        self.reloj_t = reloj_t
        if params is None:
            params = ParamsGestos.mejorados() if mejorados else ParamsGestos()
        self.params = params
        self.parcial = False
        self.copia_limpia = False  # --v-rostro en modo prueba: guardar el frame sin dibujos
        self.limpio = None
        self.detector = DetectorMano()
        self.ctrl = nuevo_control(params)

    def leer(self, t):
        ret, frame = self.cap.read()
        if not ret:
            return None
        frame = cv2.flip(frame, 1)
        lm = self.detector.find(apply_clahe(frame) if self.clahe else frame,
                                ts_ms=t * 1000 if self.reloj_t else None)
        pose = classify_pose(lm, self.params.like_pulgar_min) if lm is not None else None
        self.parcial = bool(self.params.rechazar_parcial and lm is not None
                            and fuera_de_cuadro(lm, frame.shape))
        if self.parcial:
            pose = "nada"  # M7: mano cortada por el borde, no se confia
        direction, evento = self.ctrl.update(pose, t)
        self.limpio = frame.copy() if self.copia_limpia else None
        if lm is not None:
            for a, b in HAND_BONES:
                cv2.line(frame, tuple(lm[a].astype(int)), tuple(lm[b].astype(int)),
                         (255, 100, 0), 2)
        return frame, direction, evento, pose

    def close(self):
        self.cap.release()
        self.detector.close()


def front_limit(linear, lidar, nav):
    """Frena el avance o retroceso segun la holgura con obstaculos medida por el LiDAR."""
    if linear == 0.0 or lidar is None:
        return linear, ""
    import math
    import nav_core as nc
    if not lidar.fresh():
        return 0.0, "[ESPERANDO LIDAR]"
    if linear > 0:
        free = float(nc.free_distance(*lidar.data, 0.0, nav)[0])
        cmd = float(nc.speed_command(0.0, free, linear, 0.0, nav)[0])
        if cmd == 0.0:
            return 0.0, f"[FRENO: libre {free:.2f}m]"
        return cmd, ""
    else:
        back_free = float(nc.free_distance(*lidar.data, math.pi, nav)[0])
        if back_free < nav.stop_dist:
            return 0.0, f"[FRENO ATRAS: libre {back_free:.2f}m]"
        return float(max(linear, -0.12)), ""


def comando(direction, max_l, max_a, lidar, nav):
    """Direccion de ControlGestos -> (lineal, angular, aviso) en numeros, con el freno del
    LiDAR aplicado al lineal. Lo usan gestos_teleop y mision.tarea_b."""
    lin, aviso = front_limit(float(direction[0]) * max_l, lidar, nav)
    return float(lin), float(direction[1]) * max_a, aviso


def draw_hud(frame, pose, ctrl, t, linear, angular, extra=""):
    w = frame.shape[1]
    cv2.rectangle(frame, (0, 0), (w, 64), (0, 0, 0), -1)
    text = "BUSCANDO MANO..." if pose is None else POSES[pose][0]
    cv2.putText(frame, f"Pose: {text}", (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0) if linear or angular else (200, 200, 200), 2)
    state = "ARMADO" if ctrl.armed else "DESARMADO (palma abierta para armar)"
    cv2.putText(frame, f"{state} | lin {linear:+.2f} ang {angular:+.2f} {extra}",
                (15, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 0) if ctrl.armed else (0, 165, 255), 1)
    p = ctrl.progreso(t)
    if p > 0:
        cv2.rectangle(frame, (0, 60), (int(w * p), 64), (0, 255, 255), -1)


def accion_dry(linear, angular, sent_moving):
    """Lo que el bucle de main() haria con el Sender en este frame (misma logica)."""
    if linear or angular:
        return f"send(lin={linear:+.2f}, ang={angular:+.2f})"
    if sent_moving:
        return "stop()"
    return "nada (no publica)"


def info_dry(pose, ctrl, detector, accion):
    """Texto de diagnostico por frame para --dry: confianza, estabilidad y comando."""
    conf = "-" if detector.score is None else f"{detector.score:.2f} ({detector.lado})"
    crudo = "SIN MANO" if pose is None else pose
    estable = ctrl.held if ctrl.held is not None else "-"
    if ctrl.t_mover is None:
        lleva = f"{ctrl.count}/{ctrl.stable} frames"
    else:
        need = ctrl.t_parar if pose == "puno" else ctrl.t_mover
        lleva = f"{ctrl.t - ctrl.run_t0:.2f}/{need:.2f} s ({ctrl.count} fr)"
    l1 = (f"conf {conf} | pose {crudo} {lleva}"
          f" | estable: {estable} | {'ARMADO' if ctrl.armed else 'desarmado'}")
    l2 = f"[DRY] enviaria: {accion}"
    return l1, l2


def draw_dry(frame, l1, l2):
    """Franja extra bajo el HUD normal (solo en --dry con alguna opcion nueva)."""
    w = frame.shape[1]
    cv2.rectangle(frame, (0, 64), (w, 110), (40, 40, 40), -1)
    cv2.putText(frame, l1, (15, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(frame, l2, (15, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


def add_gesture_args(ap):
    ap.add_argument("--cam", type=int, default=0, help="indice de la webcam (/dev/videoN)")
    ap.add_argument("--robot-cam", default="/oakd/rgb/preview/image_raw/compressed")
    ap.add_argument("--lineal", type=float, default=0.15, help="m/s al avanzar")
    ap.add_argument("--angular", type=float, default=0.8, help="rad/s al girar")
    ap.add_argument("--clahe", action="store_true", help="ecualizar luz (contraluz/oscuro)")
    ap.add_argument("--gestos-mejorados", action="store_true",
                    help="M1 like con pulgar separado + M2 confirmacion en segundos (el puno "
                         "se confirma antes que el movimiento). Por defecto apagado")
    ap.add_argument("--rechazar-mano-parcial", action="store_true",
                    help="M7 (aparte, experimental): mano con puntos fuera del cuadro = ninguna "
                         "pose (para). Por defecto apagado")
    ap.add_argument("--like-pulgar-min", type=float, default=None,
                    help=f"(con --gestos-mejorados) M1, por defecto {LIKE_PULGAR_MIN}")
    ap.add_argument("--t-mover", type=float, default=None,
                    help=f"(con --gestos-mejorados) M2 s para aceptar una pose, por defecto {T_MOVER}")
    ap.add_argument("--t-parar", type=float, default=None,
                    help=f"(con --gestos-mejorados) M2 s de puno para darlo por estable, por defecto {T_PARAR}")


def params_desde_args(args):
    """ParamsGestos segun la linea de comandos (sin opciones: original)."""
    parcial = bool(getattr(args, "rechazar_mano_parcial", False))  # M7, independiente
    ajustes = dict(like_pulgar_min=getattr(args, "like_pulgar_min", None),
                   t_mover=getattr(args, "t_mover", None), t_parar=getattr(args, "t_parar", None))
    if not getattr(args, "gestos_mejorados", False):
        if any(v is not None for v in ajustes.values()):
            print("AVISO: --like-pulgar-min/--t-mover/--t-parar solo valen con --gestos-mejorados; "
                  "se ignoran")
        return ParamsGestos(rechazar_parcial=parcial)
    return ParamsGestos.mejorados(**ajustes, rechazar_parcial=parcial)


def main():
    ap = argparse.ArgumentParser(description="Teleop por poses de la mano - TurtleBot 4")
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--dry", action="store_true", help="no publicar, solo mostrar")
    ap.add_argument("--no-robot-cam", action="store_true")
    ap.add_argument("--no-lidar", action="store_true", help="no frenar con el LiDAR")
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--lidar-rot", type=float, default=90.0)
    ap.add_argument("--v-rostro", action="store_true",
                    help="V (armado) = reconocer rostro con la camara del robot (en --dry "
                         "--no-robot-cam: webcam como prueba). Por defecto apagado")
    ap.add_argument("--voz", default="ninguna", choices=("auto", "windows", "linux", "ninguna"),
                    help="(con --v-rostro, sin --dry) decir el nombre en voz alta")
    ap.add_argument("--galeria", default=None, help="(con --v-rostro) carpeta de la galeria")
    add_gesture_args(ap)
    args = ap.parse_args()
    max_l = clamp(args.lineal, 0.0, LIMIT_LINEAR)
    max_a = clamp(args.angular, 0.0, LIMIT_ANGULAR)

    node = None
    use_lidar = not (args.dry or args.no_lidar)
    if not (args.dry and args.no_robot_cam):
        import rclpy
        rclpy.init()
        node = rclpy.create_node("gestos_teleop")
        # Despertar motor del LiDAR y camara en caso esten en reposo
        try:
            from std_srvs.srv import Empty, Trigger
            # (sin /start_motor: con el LiDAR ya girando tumba el driver rplidar)
            cli_cam = node.create_client(Trigger, "/oakd/start_camera")
            if cli_cam.wait_for_service(timeout_sec=0.5):
                cli_cam.call_async(Trigger.Request())
        except Exception:
            pass

    win = "Gestos teleop TurtleBot 4"
    sender = gestos = rostro = None
    try:
        gestos = Gestos(args.cam, args.clahe, params=params_desde_args(args))
        sender = Sender(node, args.topic, args.dry)
        robot_cam = None if args.no_robot_cam else RobotCamera(node, args.robot_cam)
        lidar = nav = None
        if use_lidar:
            import nav_core as nc
            from navegacion import Lidar
            lidar, nav = Lidar(node, args.scan, args.lidar_rot), nc.NavParams()

        print("Palma abierta 1 s = armar | puno = parar y desarmar | like adelante | "
              "pistola izq/der girar | cerrar ventana o Ctrl+C para salir")
        if args.dry:
            print("[MODO DRY: no se publica nada]")
        if gestos.params.activo():
            print(f"[GESTOS MEJORADOS: {gestos.params}]")
        # diagnostico por frame en --dry solo con alguna opcion nueva (sin ellas, --dry igual que antes)
        diag_dry = args.dry and (gestos.params.activo() or args.v_rostro)
        if args.v_rostro:
            from accion_rostro import FUENTE_PRUEBA, AccionRostro, fuente_rostro
            fuente = fuente_rostro(args.dry, args.no_robot_cam)
            rostro = AccionRostro.crear(node=None if args.dry else node, voz=args.voz,
                                        fuente=fuente, galeria_dir=args.galeria)
            gestos.copia_limpia = fuente == FUENTE_PRUEBA
            robot_img_t = 0.0
        sent_moving = False
        while True:
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0)
            t = time.monotonic()
            out = gestos.leer(t)
            if out is None:
                print("La webcam dejo de entregar imagenes.")
                break
            frame, direction, evento, pose = out
            if evento:
                print(evento.upper())
            linear, warn = front_limit(direction[0] * max_l, lidar, nav)
            angular = direction[1] * max_a
            if rostro is not None:   # --v-rostro: V armado = reconocer; quieto mientras dura
                if evento == "accion":
                    rostro.iniciar(t)
                if rostro.activa():
                    img = None
                    if rostro.fuente == FUENTE_PRUEBA:
                        img = gestos.limpio
                    elif robot_cam is not None:
                        cand = robot_cam.latest()
                        if cand is not None and robot_cam.last_t != robot_img_t:
                            img, robot_img_t = cand, robot_cam.last_t
                    rostro.paso(t, img)
                linear, angular = rostro.limitar(linear, angular)
            dry_lines = None
            if diag_dry:
                dry_lines = info_dry(pose, gestos.ctrl, gestos.detector,
                                     accion_dry(linear, angular, sent_moving))
                print(f"{t:.3f}s | {dry_lines[0]} | {dry_lines[1]}")
            if linear or angular:
                sender.send(linear, angular)
                sent_moving = True
            elif sent_moving:
                sender.stop()
                sent_moving = False

            draw_hud(frame, pose, gestos.ctrl, t, linear, angular, extra=warn)
            if dry_lines is not None:
                draw_dry(frame, *dry_lines)
            if rostro is not None:
                rostro.dibujar(frame, t, cajas=rostro.fuente == FUENTE_PRUEBA)
            cv2.imshow(win, np.hstack([fit_height(frame, PANEL_H), robot_panel(robot_cam)]))
            cv2.waitKey(1)  # necesario para refrescar la ventana; las teclas se ignoran
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
            if gestos is not None:
                gestos.close()
            if rostro is not None:
                rostro.cerrar()
            cv2.destroyAllWindows()
            if node is not None:
                node.destroy_node()
                rclpy.try_shutdown()
            print("\nDetenido. Adios.")


if __name__ == "__main__":
    main()
