"""
Prueba en vivo de reconocimiento facial + teleoperacion manual (WASD) del TurtleBot 4.

Combina la camara OAK-D + YuNet/SFace (facial.py) con el control manual por teclado
(comun.py) en una sola ventana de OpenCV: enfoca la ventana de video para ver las
detecciones en tiempo real y mover el robot con el teclado.

Uso (en WSL, tras 'source tb4_connect.sh'):
  python3 probar_facial_teleop.py --dry               # solo ver camara y caras, NO mueve el robot
  python3 probar_facial_teleop.py                     # camara + reconocimiento + control manual
  python3 probar_facial_teleop.py --capturar Juan     # permite pulsar 'c' para guardar fotos en galeria/Juan/
  python3 probar_facial_teleop.py --anunciar          # anuncia por voz cuando reconoce con seguridad

Controles (con la ventana de video enfocada):
  w / s       : adelante / atras
  a / d       : girar izquierda / derecha
  q / e       : adelante girando izquierda / derecha
  espacio / x : detener inmediatamente
  u / j       : subir / bajar velocidad lineal
  i / k       : subir / bajar velocidad angular
  c           : capturar foto actual en galeria (si usaste --capturar NOMBRE)
  r           : recargar galeria desde disco
  v           : anunciar en voz alta el ultimo nombre reconocido
  Esc         : salir (o cerrar la ventana / Ctrl+C)
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

from comun import (LIMIT_ANGULAR, LIMIT_LINEAR, START_ANGULAR, START_LINEAR,
                   STEP_ANGULAR, STEP_LINEAR, Sender, clamp)
from facial import GALERIA, UMBRAL, Anunciador, Galeria, Rostros, dibujar, identificar
from mano_teleop import RobotCamera

HOLD_TIMEOUT = 0.5      # s sin tecla de movimiento -> detener robot
RATE_HZ = 20            # frecuencia del bucle principal y de envio de /cmd_vel
ID_INTERVAL = 0.4       # s entre evaluaciones de identidad sobre el buffer de frames
ANNOUNCE_COOLDOWN = 5.0 # s minimos entre anuncios automaticos de la misma persona

# tecla -> (factor lineal, factor angular)
KEYS = {
    ord("w"): (1, 0),
    ord("s"): (-1, 0),
    ord("a"): (0, 1),
    ord("d"): (0, -1),
    ord("q"): (1, 1),
    ord("e"): (1, -1),
}
STOP_KEYS = (ord(" "), ord("x"))


def draw_hud(img, linear, angular, max_l, max_a, dry, res, galeria, capturar_nombre, n_capturas):
    """Dibuja barra de estado de teleop, ranking facial y ayuda de teclas."""
    h, w = img.shape[:2]

    # Ranking facial en la esquina superior derecha
    if not galeria.nombres:
        cv2.putText(img, "Galeria vacia (solo deteccion)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    elif res is not None and res.ranking:
        for idx, (nombre, score) in enumerate(res.ranking[:3]):
            txt = f"{idx + 1}. {nombre}: {score:.2f}"
            color = (0, 255, 0) if idx == 0 and res.seguro else (200, 200, 200)
            cv2.putText(img, txt, (max(10, w - 230), 28 + idx * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # Barra inferior oscura para teleop y controles
    bar_h = 56
    overlay = img.copy()
    cv2.rectangle(overlay, (0, h - bar_h), (w, h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)

    modo = " [DRY]" if dry else ""
    estado = (f"lin={linear:+.2f} m/s  ang={angular:+.2f} rad/s  "
              f"[max: {max_l:.2f}, {max_a:.2f}]{modo}")
    if capturar_nombre:
        estado += f"  |  C: {capturar_nombre} ({n_capturas})"
    cv2.putText(img, estado, (10, h - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 255) if (linear or angular) else (230, 230, 230), 1)

    ayuda = "WASD/QE: mover | Espacio: parar | U/J lin | I/K ang | R: recargar | V: voz | Esc: salir"
    cv2.putText(img, ayuda, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.43,
                (180, 180, 180), 1)
    return img


def main():
    ap = argparse.ArgumentParser(description="Prueba en vivo: reconocimiento facial + teleop WASD")
    ap.add_argument("--topic-cmd", default="/cmd_vel", help="topic de velocidad")
    ap.add_argument("--topic-cam", default="/oakd/rgb/preview/image_raw/compressed",
                    help="topic de la camara OAK-D")
    ap.add_argument("--galeria", default=GALERIA, help="carpeta de la galeria de rostros")
    ap.add_argument("--umbral", type=float, default=UMBRAL, help="umbral coseno SFace")
    ap.add_argument("--capturar", metavar="NOMBRE",
                    help="habilita tecla 'c' para guardar fotos en galeria/NOMBRE")
    ap.add_argument("--anunciar", action="store_true",
                    help="anunciar por voz cuando reconozca con seguridad")
    ap.add_argument("--dry", action="store_true",
                    help="modo seguro: no publica en /cmd_vel, solo muestra camara y caras")
    args = ap.parse_args()

    rostros = Rostros()
    rostros.calentar()
    os.makedirs(args.galeria, exist_ok=True)
    print(f"[facial] Cargando galeria desde {args.galeria}...", flush=True)
    galeria = Galeria(rostros, args.galeria)
    print(f"[facial] Galeria cargada: {galeria.resumen()}")

    destino_captura = None
    if args.capturar:
        destino_captura = os.path.join(args.galeria, args.capturar)
        os.makedirs(destino_captura, exist_ok=True)
        print(f"[captura] Pulsa 'c' en la ventana para guardar fotos en {destino_captura}")

    import rclpy
    rclpy.init()
    node = rclpy.create_node("probar_facial_teleop")
    sender = Sender(node, args.topic_cmd, args.dry)
    cam = RobotCamera(node, args.topic_cam)
    anunciador = Anunciador(node) if args.anunciar else None

    win = "Facial + Teleop TB4"
    cv2.namedWindow(win)

    max_l, max_a = START_LINEAR, START_ANGULAR
    direction = (0, 0)
    last_move_key = 0.0
    prev_vel = (0.0, 0.0)

    frames_buf = []
    res = None
    last_id_t = 0.0
    last_announced = (None, 0.0)
    n_capturas = 0
    warned_no_cam = False
    t_start = time.monotonic()
    period = 1.0 / RATE_HZ

    print("\nEnfoca la ventana 'Facial + Teleop TB4' para controlar el robot.")
    print("Controles: WASD/QE mover | Espacio/X parar | U/J lin | I/K ang | Esc salir")
    if args.dry:
        print("[MODO DRY: no se publican comandos de movimiento]")

    try:
        while rclpy.ok():
            t0 = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.01)

            img = cam.latest()
            if img is not None and (t0 - cam.last_t) <= 1.5:
                big, faces = rostros.detectar(img)
                if len(faces):
                    frames_buf = (frames_buf + [img])[-6:]
                else:
                    frames_buf.clear()
                    res = None

                if len(frames_buf) and galeria.nombres and (t0 - last_id_t) >= ID_INTERVAL:
                    last_id_t = t0
                    res = identificar(frames_buf, rostros, galeria, args.umbral)
                    if (anunciador is not None and res.seguro and res.nombre
                            and (res.nombre != last_announced[0]
                                 or (t0 - last_announced[1]) >= ANNOUNCE_COOLDOWN)):
                        anunciador.anunciar(res.nombre)
                        last_announced = (res.nombre, t0)

                canvas = dibujar(big.copy(), faces, res)
                last_img = img
            else:
                last_img = None
                if not warned_no_cam and (t0 - t_start) > 8.0:
                    warned_no_cam = True
                    print(f"[camara] Sin imagen en {args.topic_cam}. "
                          "Revisa ROS_DOMAIN_ID, ROS_STATIC_PEERS y red.")
                canvas = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(canvas, f"Esperando camara: {args.topic_cam}", (20, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            # Leer teclado desde la ventana OpenCV
            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                if key == 27:  # Esc
                    break
                k_lower = ord(chr(key).lower()) if 32 <= key < 127 else key
                if k_lower in KEYS:
                    direction = KEYS[k_lower]
                    last_move_key = t0
                elif k_lower in STOP_KEYS:
                    direction = (0, 0)
                elif k_lower == ord("u"):
                    max_l = clamp(max_l + STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
                elif k_lower == ord("j"):
                    max_l = clamp(max_l - STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
                elif k_lower == ord("i"):
                    max_a = clamp(max_a + STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
                elif k_lower == ord("k"):
                    max_a = clamp(max_a - STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
                elif k_lower == ord("r"):
                    galeria = Galeria(rostros, args.galeria)
                    print(f"[facial] Galeria recargada: {galeria.resumen()}")
                elif k_lower == ord("v"):
                    if res is not None and res.nombre:
                        if anunciador is None:
                            anunciador = Anunciador(node)
                        anunciador.anunciar(res.nombre)
                    else:
                        print("[anuncio] No hay ninguna cara reconocida todavia.")
                elif k_lower == ord("c") and destino_captura is not None and last_img is not None:
                    path = os.path.join(destino_captura, f"{int(time.time() * 1000)}.jpg")
                    cv2.imwrite(path, last_img)
                    n_capturas += 1
                    print(f"[captura] Guardada {path} ({n_capturas})")
                    galeria = Galeria(rostros, args.galeria)

            if direction != (0, 0) and (t0 - last_move_key) > HOLD_TIMEOUT:
                direction = (0, 0)

            linear = direction[0] * max_l
            angular = direction[1] * max_a

            if linear or angular:
                sender.send(linear, angular)
            elif prev_vel != (0.0, 0.0):
                sender.stop()
            prev_vel = (linear, angular)

            draw_hud(canvas, linear, angular, max_l, max_a, args.dry, res,
                     galeria, args.capturar, n_capturas)
            cv2.imshow(win, canvas)

            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break

            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sender.stop()
        finally:
            if anunciador is not None:
                anunciador.cerrar()
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.try_shutdown()
            print("\nDetenido. Adios.")


if __name__ == "__main__":
    sys.exit(main())
