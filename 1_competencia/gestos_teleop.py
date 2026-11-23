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
  python3 gestos_teleop.py --dry --no-robot-cam   # solo ver poses
  python3 gestos_teleop.py                        # robot real
  python3 gestos_teleop.py --lineal 0.2 --angular 1.0 --clahe

Para salir: cerrar la ventana o Ctrl+C en la terminal (no se usa el teclado en la
ventana). La Tarea B completa (con reconocimiento y anuncio) es: ./tarea_b.sh
"""
import argparse
import time

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


def classify_pose(lm):
    """lm: (21, 2) en px (frame ya espejado). Retorna una clave de POSES."""
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


class ControlGestos:
    """Pose por frame -> (direccion, evento). Eventos: 'armado', 'desarmado', 'accion'.
    Solo se mueve armado y con una pose estable; cualquier duda = quieto."""

    def __init__(self, stable=STABLE_FRAMES, arm_hold=ARM_HOLD, action_hold=ACTION_HOLD,
                 puno_hold=PUNO_HOLD, gracia=GRACIA):
        self.stable, self.arm_hold, self.action_hold = stable, arm_hold, action_hold
        self.puno_hold, self.gracia = puno_hold, gracia
        self.armed = False
        self.last, self.count = None, 0
        self.held, self.held_t0, self.fired = None, 0.0, False
        self.mov, self.t_mov = (0, 0), -1e9      # ultimo movimiento y cuando se vio

    def update(self, pose, t):
        self.count = self.count + 1 if pose == self.last else 1
        self.last = pose
        stable = pose if pose is not None and self.count >= self.stable else None
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


class Gestos:
    """Webcam + MediaPipe + ControlGestos. leer(t) -> (frame, direccion, evento, pose)."""

    def __init__(self, cam_index, clahe=False):
        self.cap = open_camera(cam_index)
        if self.cap is None:
            raise SystemExit(f"No se pudo abrir la webcam {cam_index}. En WSL hay que pasarla con "
                             "usbipd (ver seguimiento_vision/README.md); prueba tambien --cam 1.")
        self.clahe = clahe
        self.detector = HandDetector()
        self.ctrl = ControlGestos()

    def leer(self, t):
        ret, frame = self.cap.read()
        if not ret:
            return None
        frame = cv2.flip(frame, 1)
        lm = self.detector.find(apply_clahe(frame) if self.clahe else frame)
        pose = classify_pose(lm) if lm is not None else None
        direction, evento = self.ctrl.update(pose, t)
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


def add_gesture_args(ap):
    ap.add_argument("--cam", type=int, default=0, help="indice de la webcam (/dev/videoN)")
    ap.add_argument("--robot-cam", default="/oakd/rgb/preview/image_raw/compressed")
    ap.add_argument("--lineal", type=float, default=0.15, help="m/s al avanzar")
    ap.add_argument("--angular", type=float, default=0.8, help="rad/s al girar")
    ap.add_argument("--clahe", action="store_true", help="ecualizar luz (contraluz/oscuro)")


def main():
    ap = argparse.ArgumentParser(description="Teleop por poses de la mano - TurtleBot 4")
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--dry", action="store_true", help="no publicar, solo mostrar")
    ap.add_argument("--no-robot-cam", action="store_true")
    ap.add_argument("--no-lidar", action="store_true", help="no frenar con el LiDAR")
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--lidar-rot", type=float, default=90.0)
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
    sender = gestos = None
    try:
        gestos = Gestos(args.cam, args.clahe)
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
            if linear or angular:
                sender.send(linear, angular)
                sent_moving = True
            elif sent_moving:
                sender.stop()
                sent_moving = False

            draw_hud(frame, pose, gestos.ctrl, t, linear, angular, extra=warn)
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
            cv2.destroyAllWindows()
            if node is not None:
                node.destroy_node()
                rclpy.try_shutdown()
            print("\nDetenido. Adios.")


if __name__ == "__main__":
    main()
