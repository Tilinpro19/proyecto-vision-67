"""
Teleop + reconocimiento facial en vivo con la camara del TurtleBot 4 (desde la laptop, en WSL).

Mueves el robot con el teclado (ventana de video enfocada) y ves en pantalla las caras detectadas
y el nombre identificado. Sirve para probar facial.py con la camara real y para capturar fotos
de la galeria con el mismo lente/luz de la competencia.

  python3 probar_facial_teleop.py                    # mover + detectar + identificar
  python3 probar_facial_teleop.py --capturar Juan    # ademas: tecla c guarda foto en galeria/Juan/
  python3 probar_facial_teleop.py --dry              # sin publicar velocidades

Teclas (ventana de video):
  w/s adelante/atras   a/d girar   q/e adelante+giro   espacio o x parar
  u/j velocidad lineal +/-         i/k velocidad angular +/-
  f   seguir cara: gira solo para centrar la cara mas grande (no avanza)
  r   activar/desactivar identificacion      c  guardar foto (con --capturar)
  ESC o Ctrl+C salir (siempre envia velocidad cero)

Seguridad: sin tecla durante HOLD_TIMEOUT s el robot para; si la camara se congela (>1 s) para
y deja de seguir cara; velocidades con los mismos topes de teleop_wasd.py.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "2_laboratorio"))

import facial  # noqa: E402

START_LINEAR, START_ANGULAR = 0.1, 0.8
STEP_LINEAR, STEP_ANGULAR = 0.05, 0.2
LIMIT_LINEAR, LIMIT_ANGULAR = 0.31, 1.9
HOLD_TIMEOUT = 0.35      # s sin tecla -> parar (el auto-repeat de la ventana llega cada ~30 ms)
IDENT_EVERY = 0.5        # s entre identificaciones
IDENT_FRAMES = 6         # frames recientes que se promedian
FOLLOW_KP = 1.2          # rad/s por unidad de error horizontal normalizado (-1..1)
FOLLOW_DEADBAND = 0.08
STALE = 1.0              # s sin imagen nueva -> se considera congelada

KEYS = {ord("w"): (1, 0), ord("s"): (-1, 0), ord("a"): (0, 1), ord("d"): (0, -1),
        ord("q"): (1, 1), ord("e"): (1, -1)}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def error_horizontal(face, ancho):
    """Cara -> error normalizado en [-1, 1]; positivo = cara a la derecha de la imagen."""
    cx = float(face[0] + face[2] / 2)
    return clamp((cx - ancho / 2) / (ancho / 2), -1.0, 1.0)


def giro_para_centrar(err, max_a):
    """Velocidad angular (rad/s, + = izquierda) que centra la cara; 0 dentro de la zona muerta."""
    if abs(err) < FOLLOW_DEADBAND:
        return 0.0
    return clamp(-FOLLOW_KP * err, -max_a, max_a)


def hud(img, lineal, angular, max_l, max_a, ident, seguir, res, guardadas, capturar, stale):
    h = img.shape[0]
    cv2.rectangle(img, (0, h - 46), (img.shape[1], h), (0, 0, 0), -1)
    cv2.putText(img, f"v={lineal:+.2f} m/s w={angular:+.2f} rad/s  max {max_l:.2f}/{max_a:.1f}",
                (6, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    estado = f"ident:{'ON' if ident else 'off'}  seguir:{'ON' if seguir else 'off'}"
    if capturar:
        estado += f"  fotos {capturar}: {guardadas}"
    cv2.putText(img, estado, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    if stale:
        cv2.putText(img, "CAMARA CONGELADA", (6, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return img


def main():
    ap = argparse.ArgumentParser(description="Teleop + reconocimiento facial en vivo")
    ap.add_argument("--topic", default="/oakd/rgb/preview/image_raw/compressed")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--galeria", default=facial.GALERIA)
    ap.add_argument("--umbral", type=float, default=facial.UMBRAL)
    ap.add_argument("--capturar", metavar="NOMBRE", help="habilita la tecla c: fotos en galeria/NOMBRE/")
    ap.add_argument("--dry", action="store_true", help="no publicar velocidades")
    ap.add_argument("--anunciar", action="store_true", help="decir el nombre al salir")
    args = ap.parse_args()

    import rclpy

    from mano_teleop import RobotCamera
    from teleop_wasd import Sender

    sender = Sender(args.cmd_topic, args.dry)       # inicia rclpy
    node = rclpy.create_node("probar_facial_teleop")
    cam = RobotCamera(node, args.topic)
    rostros = facial.Rostros()
    rostros.calentar()
    os.makedirs(args.galeria, exist_ok=True)
    galeria = facial.Galeria(rostros, args.galeria)
    print(f"Galeria {galeria.resumen()}")

    destino = None
    if args.capturar:
        destino = os.path.join(args.galeria, args.capturar)
        os.makedirs(destino, exist_ok=True)

    max_l, max_a = START_LINEAR, START_ANGULAR
    direccion, last_key = (0, 0), 0.0
    ident, seguir = True, False
    frames, res, next_ident, guardadas = [], None, 0.0, 0
    ultimo_envio = (0.0, 0.0)
    last_fid = None
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            img = cam.latest()
            stale = img is None or now - cam.last_t > STALE
            if img is None:
                cv2.waitKey(30)
                continue

            big, faces = rostros.detectar(img)
            if cam.last_t != last_fid:                    # solo frames nuevos
                last_fid = cam.last_t
                frames = (frames + [img])[-IDENT_FRAMES:]
            if ident and not stale and now >= next_ident and len(faces):
                next_ident = now + IDENT_EVERY
                res = facial.identificar(frames, rostros, galeria, args.umbral)
            elif not len(faces):
                res = None

            # --- teclado
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            if k in KEYS:
                direccion, last_key = KEYS[k], now
            elif k in (ord(" "), ord("x")):
                direccion = (0, 0)
            elif k == ord("u"):
                max_l = clamp(max_l + STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
            elif k == ord("j"):
                max_l = clamp(max_l - STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
            elif k == ord("i"):
                max_a = clamp(max_a + STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
            elif k == ord("k"):
                max_a = clamp(max_a - STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
            elif k == ord("f"):
                seguir = not seguir
            elif k == ord("r"):
                ident, res = not ident, None
            elif k == ord("c") and destino:
                if len(faces):
                    cv2.imwrite(os.path.join(destino, f"{int(time.time() * 1000)}.jpg"), img)
                    guardadas += 1
                    print(f"guardada {guardadas} en {destino}")
                else:
                    print("sin cara: no se guarda")
            if direccion != (0, 0) and now - last_key > HOLD_TIMEOUT:
                direccion = (0, 0)

            # --- velocidad: teclado manda; si no, seguir cara (solo gira)
            lineal, angular = direccion[0] * max_l, direccion[1] * max_a
            if direccion == (0, 0) and seguir and len(faces) and not stale:
                angular = giro_para_centrar(error_horizontal(faces[0], big.shape[1]), max_a)
            if stale:
                lineal = angular = 0.0
            if lineal or angular:
                sender.send(lineal, angular)
            elif ultimo_envio != (0.0, 0.0):
                sender.stop()
            ultimo_envio = (lineal, angular)

            vista = facial.dibujar(big.copy(), faces, res)
            cv2.imshow("probar_facial_teleop", hud(vista, lineal, angular, max_l, max_a,
                                                   ident, seguir, res, guardadas, args.capturar, stale))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sender.stop()
        finally:
            if args.anunciar and res is not None and res.nombre:
                facial.anunciar(res.nombre, node)
            cv2.destroyAllWindows()
            node.destroy_node()
            sender.close()
            print("Detenido.")


if __name__ == "__main__":
    main()
