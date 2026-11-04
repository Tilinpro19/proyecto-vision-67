"""
Teleoperacion del TurtleBot 4 moviendo la mano frente a la webcam de la laptop.
Misma idea que teleop_wasd.py, pero cada gesto hace de tecla:

  acercar la mano  -> w (adelante)        alejar la mano -> s (atras)
  mano a la izq.   -> a (girar izq.)      mano a la der. -> d (girar der.)
  arriba / abajo / quieto -> nada

Pipeline: MediaPipe Hand Landmarker (21 puntos de la mano; solo manos, nunca
caras) -> mascara de la mano -> esquinas Shi-Tomasi -> flujo optico
Lucas-Kanade -> similitud RANSAC (traslacion + escala) acumulada en unos pocos
frames -> gesto.

La ventana muestra tu webcam (izquierda) y la camara OAK-D del robot (derecha).

Requiere mediapipe (pip) y el modelo hand_landmarker.task junto a este script.

Uso (WSL o Ubuntu, laptop en la red del robot, tras 'source tb4_connect.sh'):
  python3 mano_teleop.py --dry           # solo ver gestos, NO publica (probar primero)
  python3 mano_teleop.py                 # controla el robot real
  python3 mano_teleop.py --topic /turtle1/cmd_vel     # turtlesim
  python3 mano_teleop.py --cam 1 --no-robot-cam

Teclas (con la ventana enfocada):
  espacio : armar / pausar (ARRANCA EN PAUSA: el robot no se mueve hasta armarlo)
  u / j : subir / bajar velocidad lineal     i / k : subir / bajar velocidad angular
  r : volver a buscar puntos en la mano      q / Esc : salir

Seguridad:
  - Velocidad fija como en WASD (arranca lenta, tope LIMIT_* de comun.py).
  - El gesto dura HOLD_TIMEOUT s; si dejas de mover la mano, el robot para.
  - Si se pierde la mano (o quedan pocos puntos) el robot para.
  - Solo una mano (la de mayor confianza) mueve el robot; la cara se ignora.
  - Al salir (o si falla algo) se envia velocidad cero.
"""
import argparse
import os
import time
from collections import deque

import cv2
import numpy as np

from comun import (LIMIT_ANGULAR, LIMIT_LINEAR, START_ANGULAR, START_LINEAR,
                   STEP_ANGULAR, STEP_LINEAR, Sender, clamp)

HOLD_TIMEOUT = 0.6     # s que dura un gesto (como el auto-repeat de una tecla)
PANEL_H = 480          # alto de cada panel en la ventana

# Ventana temporal de decision: el acercamiento real cambia la escala ~1-2% por
# frame, asi que se acumula el movimiento de los ultimos N frames.
HISTORY_LEN = 6
MIN_TRANS_PX = 12.0     # traslacion acumulada en la ventana (px)
MIN_LOG_SCALE = 0.05    # cambio de escala acumulado en la ventana (~5%)
MIN_RADIAL_PX = 1.5     # desplazamiento radial acumulado (px); bajo para que
                        # ATRAS siga detectandose cuando la mano ya se ve pequena
RADIAL_VS_TRANS = 0.25  # la escala gana si radial_px >= 0.25 * trans_px
                        # (al acercar la mano casi siempre deriva un poco de lado)
MIN_POINTS = 8          # menos puntos que esto -> no hay tracking
RESEED_EVERY = 30       # frames entre re-sembrados aunque el tracking vaya bien
LOST_FRAMES_MAX = 3     # frames sin mano (p. ej. desenfoque al moverla rapido)
                        # en los que el flujo sigue con la ultima mascara

# Modelo de MediaPipe junto al script: el laboratorio no tiene internet.
# https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
HAND_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")
MIN_HAND_CONFIDENCE = 0.6  # sube si aparecen manos falsas, baja si la pierde mucho
HAND_MARGIN_PX = 20        # los puntos de MediaPipe son articulaciones: se agranda
                           # el contorno para cubrir el borde de los dedos

FEATURE_PARAMS = dict(maxCorners=40, qualityLevel=0.05, minDistance=10, blockSize=7)
LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Huesos de la mano (indices de los 21 landmarks) para dibujarla
HAND_BONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
              (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
              (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]

# gesto -> (texto en pantalla, (factor lineal, factor angular))
GESTURES = {
    "adelante": (">> ADELANTE (Acercando) <<", (1, 0)),
    "atras": ("<< ATRAS (Alejando) >>", (-1, 0)),
    "izquierda": ("<-- IZQUIERDA", (0, 1)),
    "derecha": ("--> DERECHA", (0, -1)),
    "arriba": ("^ ARRIBA ^", (0, 0)),
    "abajo": ("v ABAJO v", (0, 0)),
    "quieto": ("QUIETO", (0, 0)),
}


def estimate_motion(good_old, good_new):
    """
    Ajusta una transformacion de similitud (traslacion + escala + rotacion)
    con RANSAC entre los puntos de dos frames. Retorna (dx, dy, log_scale, spread)
    o None. spread = radio medio de los puntos, para pasar la escala a pixeles.
    """
    M, inliers = cv2.estimateAffinePartial2D(good_old, good_new, method=cv2.RANSAC,
                                             ransacReprojThreshold=3.0)
    if M is None:
        return None
    inl = inliers.ravel().astype(bool)
    if inl.sum() < 3:
        return None
    scale = float(np.hypot(M[0, 0], M[1, 0]))
    c_old = good_old[inl].mean(axis=0)
    c_new = good_new[inl].mean(axis=0)
    spread = float(np.mean(np.linalg.norm(good_old[inl] - c_old, axis=1)))
    dx, dy = c_new - c_old
    return float(dx), float(dy), float(np.log(scale)), spread


def classify_motion(history):
    """
    history: secuencia de (dx, dy, log_scale, spread) de los ultimos frames.
    Compara traslacion y escala en las MISMAS unidades (pixeles).
    Retorna (gesto, trans_px, escala_acumulada); gesto es una clave de GESTURES.
    """
    sum_dx = sum(m[0] for m in history)
    sum_dy = sum(m[1] for m in history)
    sum_ls = sum(m[2] for m in history)
    spread = history[-1][3]

    trans_px = float(np.hypot(sum_dx, sum_dy))
    radial_px = abs(sum_ls) * spread
    scale_acc = float(np.exp(sum_ls))

    if (abs(sum_ls) > MIN_LOG_SCALE and radial_px > MIN_RADIAL_PX
            and radial_px >= RADIAL_VS_TRANS * trans_px):
        gesture = "adelante" if sum_ls > 0 else "atras"
    elif trans_px > MIN_TRANS_PX:
        if abs(sum_dx) > abs(sum_dy):
            gesture = "derecha" if sum_dx > 0 else "izquierda"
        else:
            gesture = "abajo" if sum_dy > 0 else "arriba"
    else:
        gesture = "quieto"
    return gesture, trans_px, scale_acc


class HandDetector:
    """MediaPipe Hand Landmarker en modo video. find(frame) -> (21, 2) en px, o None."""

    def __init__(self):
        if not os.path.isfile(HAND_MODEL):
            raise SystemExit(f"Falta el modelo {HAND_MODEL}. Descargalo (URL en mano_teleop.py) "
                             "y dejalo junto al script.")
        try:
            import mediapipe as mp
            from mediapipe.tasks.python import BaseOptions, vision
        except ImportError:
            raise SystemExit("Falta mediapipe: ver 'Instalacion' en seguimiento_vision/README.md.")
        self.mp = mp
        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=HAND_MODEL),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=MIN_HAND_CONFIDENCE,
            min_hand_presence_confidence=MIN_HAND_CONFIDENCE,
            min_tracking_confidence=0.5)
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self.last_ts = -1

    def find(self, frame):
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # el modo video exige timestamps (ms) estrictamente crecientes
        self.last_ts = max(int(time.monotonic() * 1000), self.last_ts + 1)
        result = self.landmarker.detect_for_video(
            self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), self.last_ts)
        if not result.hand_landmarks:
            return None
        return np.array([[lm.x * w, lm.y * h] for lm in result.hand_landmarks[0]], np.float32)

    def close(self):
        self.landmarker.close()


def hand_mask_from_landmarks(landmarks, shape):
    """Mascara rellena: contorno convexo de los 21 puntos, agrandado HAND_MARGIN_PX."""
    mask = np.zeros(shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(landmarks.astype(np.int32)), 255)
    size = 2 * HAND_MARGIN_PX + 1
    return cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size)))


class HandTracker:
    """Estado del tracking de la mano entre frames. update(frame) -> gesto o None."""

    def __init__(self):
        self.detector = HandDetector()
        self.reset()

    def reset(self):
        self.p0 = None
        self.prev_gray = None
        self.hand_mask = None
        self.frames_since_reseed = 0
        self.lost_frames = 0
        self.history = deque(maxlen=HISTORY_LEN)
        self.trans_px = 0.0
        self.scale_acc = 1.0

    def points(self):
        return 0 if self.p0 is None else len(self.p0)

    def update(self, frame):
        """Procesa un frame (ya espejado) y dibuja sobre el.
        Retorna la clave del gesto, o None si no hay tracking de la mano."""
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # MediaPipe dice DONDE esta la mano; el flujo optico mide COMO se mueve.
        # Si MediaPipe la pierde 1-3 frames (desenfoque) se usa la ultima mascara.
        landmarks = self.detector.find(frame)
        if landmarks is not None:
            self.hand_mask = hand_mask_from_landmarks(landmarks, frame.shape)
            self.lost_frames = 0
        else:
            self.lost_frames += 1
            if self.lost_frames > LOST_FRAMES_MAX:
                self.hand_mask, self.p0 = None, None

        gesture = None
        if self.hand_mask is not None:
            # Los puntos nuevos son de ESTE frame: el flujo se mide desde el siguiente
            needs_reseed = (self.p0 is None or len(self.p0) < MIN_POINTS
                            or self.frames_since_reseed > RESEED_EVERY)
            if needs_reseed and landmarks is not None:
                corners = cv2.goodFeaturesToTrack(curr_gray, mask=self.hand_mask, **FEATURE_PARAMS)
                if corners is not None and len(corners) >= MIN_POINTS:
                    self.p0 = corners
                    self.frames_since_reseed = 0
                    gesture = classify_motion(self.history)[0] if self.history else "quieto"
            elif self.p0 is not None and self.prev_gray is not None:
                gesture = self._flow(frame, curr_gray)

        # Sin tracking, el historial ya no describe la mano actual
        if gesture is None:
            self.history.clear()

        if landmarks is not None:
            for a, b in HAND_BONES:
                cv2.line(frame, tuple(landmarks[a].astype(int)), tuple(landmarks[b].astype(int)),
                         (255, 100, 0), 2)
        if self.hand_mask is not None:
            contours, _ = cv2.findContours(self.hand_mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame, contours, -1, (0, 200, 255), 1)

        self.prev_gray = curr_gray
        return gesture

    def _flow(self, frame, curr_gray):
        """Lucas-Kanade desde el frame anterior. Retorna el gesto o None."""
        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, curr_gray, self.p0, None,
                                             **LK_PARAMS)
        p_old, self.p0 = self.p0, None
        if p1 is None or st is None:
            return None
        good_new = p1[st == 1]
        good_old = p_old[st == 1]

        # Solo puntos que siguen dentro de la mano (los que saltaron al fondo, fuera)
        mh, mw = self.hand_mask.shape
        inside = np.array([0 <= int(y) < mh and 0 <= int(x) < mw
                           and self.hand_mask[int(y), int(x)] > 0
                           for x, y in good_new], dtype=bool)
        good_new, good_old = good_new[inside], good_old[inside]
        if len(good_new) < MIN_POINTS:
            return None  # se re-siembra en el proximo frame con mano

        self.frames_since_reseed += 1
        motion = estimate_motion(good_old, good_new)
        if motion is not None:
            self.history.append(motion)
        gesture = "quieto"
        if self.history:
            gesture, self.trans_px, self.scale_acc = classify_motion(self.history)

        for (xn, yn), (xo, yo) in zip(good_new, good_old):
            cv2.circle(frame, (int(xn), int(yn)), 4, (0, 255, 0), -1)
            cv2.line(frame, (int(xn), int(yn)), (int(xo), int(yo)), (0, 255, 255), 2)

        self.p0 = good_new.reshape(-1, 1, 2)
        return gesture


class RobotCamera:
    """Ultima imagen de la camara del robot (CompressedImage, QoS sensor data)."""

    def __init__(self, node, topic):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        self.topic = topic
        self.msg = None
        self.img = None
        self.last_t = 0.0
        node.create_subscription(CompressedImage, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg):
        self.msg = msg

    def latest(self):
        """Decodifica la imagen nueva (si llego) y devuelve la ultima valida o None."""
        if self.msg is not None:
            img = cv2.imdecode(np.frombuffer(bytes(self.msg.data), np.uint8), cv2.IMREAD_COLOR)
            self.msg = None
            if img is not None:  # None = JPEG corrupto (paquete perdido por Wi-Fi)
                self.img, self.last_t = img, time.monotonic()
        return self.img


def fit_height(img, height):
    h, w = img.shape[:2]
    return cv2.resize(img, (max(1, round(w * height / h)), height), interpolation=cv2.INTER_LINEAR)


def robot_panel(robot_cam):
    """Panel derecho: imagen del robot, o un aviso si no llega."""
    img = robot_cam.latest() if robot_cam else None
    if img is None:
        panel = np.zeros((PANEL_H, PANEL_H, 3), np.uint8)
        text = "Esperando camara del robot..." if robot_cam else "Camara del robot desactivada"
        cv2.putText(panel, text, (15, PANEL_H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (160, 160, 160), 1)
        return panel
    panel = fit_height(img, PANEL_H)
    if time.monotonic() - robot_cam.last_t > 2.0:
        cv2.putText(panel, "SIN SENAL", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return panel


def draw_hud(frame, text, ok, info, armed, linear, angular, max_l, max_a):
    w = frame.shape[1]
    cv2.rectangle(frame, (0, 0), (w, 82), (0, 0, 0), -1)
    cv2.putText(frame, f"Gesto: {text}", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (0, 255, 0) if ok else (200, 200, 200), 2)
    cv2.putText(frame, info, (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
    state = "ARMADO" if armed else "PAUSA (espacio)"
    cv2.putText(frame, f"{state} | lin {linear:+.2f} ang {angular:+.2f} "
                       f"| max {max_l:.2f} m/s {max_a:.1f} rad/s",
                (15, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if armed else (0, 165, 255), 1)


def attach_wsl_camera(index):
    """En WSL la webcam se desconecta al reiniciar: la conecta con usbipd.exe de Windows.
    Requiere haber hecho una vez 'usbipd bind' (como administrador) en Windows."""
    import re
    import shutil
    import subprocess

    dev = f"/dev/video{index}"
    usbipd = shutil.which("usbipd.exe") or "/mnt/c/Program Files/usbipd-win/usbipd.exe"
    if os.path.exists(dev) or "WSL_DISTRO_NAME" not in os.environ or not os.path.exists(usbipd):
        return
    try:
        listing = subprocess.run([usbipd, "list"], capture_output=True, text=True,
                                 timeout=15).stdout
    except (OSError, subprocess.TimeoutExpired):
        return
    m = re.search(r"^(\d+-\d+)\s+\S+\s+.*(camera|webcam|cam).*$", listing, re.I | re.M)
    if not m:
        print("usbipd no muestra ninguna camara.")
        return
    print(f"Conectando la webcam {m.group(1)} a WSL (usbipd)...")
    subprocess.run([usbipd, "attach", "--wsl", "--busid", m.group(1)],
                   capture_output=True, timeout=30)
    for _ in range(50):  # el /dev/videoN tarda un poco en aparecer
        if os.path.exists(dev):
            time.sleep(1.0)
            return
        time.sleep(0.2)


def open_camera(index):
    attach_wsl_camera(index)
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None
    # MJPG + 640x480: menos ancho de banda USB (importa con usbipd en WSL)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


def main():
    ap = argparse.ArgumentParser(description="Teleop por gestos de la mano - TurtleBot 4")
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--dry", action="store_true", help="no publicar, solo mostrar")
    ap.add_argument("--cam", type=int, default=0, help="indice de la webcam (/dev/videoN)")
    ap.add_argument("--robot-cam", default="/oakd/rgb/preview/image_raw/compressed",
                    help="topic CompressedImage de la camara del robot")
    ap.add_argument("--no-robot-cam", action="store_true", help="no mostrar la camara del robot")
    args = ap.parse_args()

    cap = open_camera(args.cam)
    if cap is None:
        raise SystemExit(f"No se pudo abrir la webcam {args.cam}. En WSL hay que pasarla con "
                         "usbipd (ver seguimiento_vision/README.md); prueba tambien --cam 1.")

    # ROS solo hace falta para publicar o para ver la camara del robot
    node = None
    if not (args.dry and args.no_robot_cam):
        import rclpy
        rclpy.init()
        node = rclpy.create_node("mano_teleop")

    win = "Mano teleop TurtleBot 4"
    sender = None
    tracker = None
    try:
        tracker = HandTracker()
        sender = Sender(node, args.topic, args.dry)
        robot_cam = None if args.no_robot_cam else RobotCamera(node, args.robot_cam)

        print("Haz CLIC en la ventana (las teclas no van a esta terminal) y pulsa ESPACIO para armar.")
        print("espacio armar/pausar | u/j vel. lineal +/- | i/k vel. angular +/- "
              "| r re-buscar mano | q/Esc salir")
        if args.dry:
            print("[MODO DRY: no se publica nada]")

        armed = False
        max_l, max_a = START_LINEAR, START_ANGULAR
        direction, last_gesture_t = (0, 0), 0.0
        sent_moving = False

        while True:
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0)
            ret, frame = cap.read()
            if not ret:
                print("La webcam dejo de entregar imagenes.")
                break
            frame = cv2.flip(frame, 1)
            now = time.monotonic()

            gesture = tracker.update(frame)
            if gesture is None:
                direction = (0, 0)  # deadman: sin mano no hay movimiento
            elif GESTURES[gesture][1] != (0, 0):
                direction, last_gesture_t = GESTURES[gesture][1], now
            elif now - last_gesture_t > HOLD_TIMEOUT:
                direction = (0, 0)

            linear = direction[0] * max_l if armed else 0.0
            angular = direction[1] * max_a if armed else 0.0
            if linear or angular:
                sender.send(linear, angular)
                sent_moving = True
            elif sent_moving:
                sender.stop()
                sent_moving = False

            if gesture is None:
                text, info = "BUSCANDO MANO...", "Buscando mano..."
            else:
                text = GESTURES[gesture][0]
                info = (f"Puntos: {tracker.points()} | Traslacion ({HISTORY_LEN}f): "
                        f"{tracker.trans_px:.1f}px | Escala ({HISTORY_LEN}f): "
                        f"{(tracker.scale_acc - 1.0) * 100:+.1f}%")
            draw_hud(frame, text, direction != (0, 0), info, armed, linear, angular, max_l, max_a)

            cv2.imshow(win, np.hstack([fit_height(frame, PANEL_H), robot_panel(robot_cam)]))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord(" "):
                armed = not armed
                print("ARMADO: el robot sigue tu mano" if armed else "PAUSA")
            elif key == ord("u"):
                max_l = clamp(max_l + STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
            elif key == ord("j"):
                max_l = clamp(max_l - STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
            elif key == ord("i"):
                max_a = clamp(max_a + STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
            elif key == ord("k"):
                max_a = clamp(max_a - STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
            elif key == ord("r"):
                tracker.reset()
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
            cap.release()
            if tracker is not None:
                tracker.detector.close()
            cv2.destroyAllWindows()
            if node is not None:
                node.destroy_node()
                rclpy.try_shutdown()
            print("\nDetenido. Adios.")


if __name__ == "__main__":
    main()
