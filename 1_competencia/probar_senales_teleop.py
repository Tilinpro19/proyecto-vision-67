"""
Prueba en vivo de deteccion de senales (flechas, prohibido, bandera y QR) + teleop WASD del TurtleBot 4.

Combina la camara OAK-D con `senales_forma.py` (formas B/N y QR) y `senales.py` (color azul/rojo en QR)
mas el control manual por teclado (`comun.py`) en una sola ventana de OpenCV.
La deteccion corre en un hilo separado para que el video y el teclado no tengan lag.

Uso (en WSL, tras 'source ../tb4_connect.sh'):
  python3 probar_senales_teleop.py --dry                # solo ver camara y senales, NO mueve el robot
  python3 probar_senales_teleop.py                      # camara + senales + control manual
  python3 probar_senales_teleop.py --topic-cam /oakd/rgb/image_raw/compressed --dry
                                                        # alta resolucion (mejor alcance para QRs)

Controles (con la ventana de video enfocada):
  w / s       : adelante / atras
  a / d       : girar izquierda / derecha
  q / e       : adelante girando izquierda / derecha
  espacio / x : detener inmediatamente
  u / j       : subir / bajar velocidad lineal
  i / k       : subir / bajar velocidad angular
  Esc         : salir (o cerrar la ventana / Ctrl+C)
"""
import argparse
import sys
import threading
import time

import cv2
import numpy as np

import senales
import senales_forma
from comun import (LIMIT_ANGULAR, LIMIT_LINEAR, START_ANGULAR, START_LINEAR,
                   STEP_ANGULAR, STEP_LINEAR, Sender, clamp)
from mano_teleop import RobotCamera

HOLD_TIMEOUT = 0.5
RATE_HZ = 30

KEYS = {
    ord("w"): (1, 0),
    ord("s"): (-1, 0),
    ord("a"): (0, 1),
    ord("d"): (0, -1),
    ord("q"): (1, 1),
    ord("e"): (1, -1),
}
STOP_KEYS = (ord(" "), ord("x"))

COLOR_TIPO = {
    "izq": (0, 220, 255),        # amarillo
    "der": (0, 220, 255),        # amarillo
    "prohibido": (0, 0, 255),    # rojo
    "bandera": (255, 180, 0),    # azul claro
    "qr": (0, 255, 0),           # verde
}


def dibujar_senales(img, encontradas, color_qrs):
    """Dibuja cajas, etiquetas, distancia estimada y texto QR sobre una copia de la imagen."""
    out = img.copy()
    qr_color_por_texto = {q.texto: q for q in color_qrs}

    for s in encontradas:
        x, y, w, h = s.bbox
        col = COLOR_TIPO.get(s.tipo, (255, 255, 255))
        cv2.rectangle(out, (x, y), (x + w, y + h), col, 2)

        dist = f" {s.distancia_m:.2f}m" if s.distancia_m is not None else ""
        if s.tipo == "qr":
            q = qr_color_por_texto.get(s.texto)
            extra = f" [{q.color}:{q.accion}]" if (q and q.color) else ""
            etq = f"QR: {s.texto}{extra}{dist}"
        else:
            etq = f"{s.tipo.upper()} ({s.confianza:.2f}){dist}"

        ty = max(18, y - 6)
        cv2.putText(out, etq, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

    return out


def draw_hud(img, linear, angular, max_l, max_a, dry, encontradas):
    """Dibuja resumen de senales detectadas y barra inferior de teleop."""
    h, w = img.shape[:2]

    if encontradas:
        resumen = " | ".join(
            (f"QR:{s.texto}" if s.tipo == "qr" else s.tipo)
            + (f" ({s.distancia_m:.1f}m)" if s.distancia_m is not None else "")
            for s in encontradas[:4]
        )
        cv2.putText(img, resumen, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(img, "Sin senales detectadas", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 2)

    bar_h = 56
    overlay = img.copy()
    cv2.rectangle(overlay, (0, h - bar_h), (w, h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)

    modo = " [DRY]" if dry else ""
    estado = (f"lin={linear:+.2f} m/s  ang={angular:+.2f} rad/s  "
              f"[max: {max_l:.2f}, {max_a:.2f}]{modo}")
    cv2.putText(img, estado, (10, h - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 255) if (linear or angular) else (230, 230, 230), 1)

    ayuda = "WASD/QE: mover | Espacio/X: parar | U/J lin | I/K ang | Esc: salir"
    cv2.putText(img, ayuda, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (180, 180, 180), 1)
    return img


class DetectorAsincrono:
    """Ejecuta `senales_forma.detectar` en segundo plano siempre sobre el frame mas reciente."""

    def __init__(self, lado_m=0.10, esfuerzo_qr=1):
        self.lado_m = lado_m
        self.esfuerzo_qr = esfuerzo_qr
        self._lock = threading.Lock()
        self._evento = threading.Event()
        self._stop = False
        self._frame = None
        self._encontradas = []
        self._color_qrs = []
        self._hilo = threading.Thread(target=self._loop, daemon=True)
        self._hilo.start()

    def enviar(self, img):
        with self._lock:
            self._frame = img
        self._evento.set()

    def resultado(self):
        with self._lock:
            return list(self._encontradas), list(self._color_qrs)

    def cerrar(self):
        self._stop = True
        self._evento.set()

    def _loop(self):
        while not self._stop:
            self._evento.wait(timeout=0.2)
            self._evento.clear()
            if self._stop:
                break
            with self._lock:
                img = self._frame
                self._frame = None
            if img is None:
                continue
            enc = senales_forma.detectar(img, lado_m=self.lado_m, esfuerzo_qr=self.esfuerzo_qr)
            cqrs = senales.leer(img, lado_m=self.lado_m) if any(
                s.tipo == "qr" for s in enc
            ) else []
            with self._lock:
                self._encontradas = enc
                self._color_qrs = cqrs


def main():
    ap = argparse.ArgumentParser(description="Prueba en vivo: QRs, flechas, bandera + teleop WASD")
    ap.add_argument("--topic-cmd", default="/cmd_vel", help="topic de velocidad")
    ap.add_argument("--topic-cam", default="/oakd/rgb/preview/image_raw/compressed",
                    help="topic de la camara OAK-D (usar image_raw/compressed para mayor alcance QR)")
    ap.add_argument("--lado", type=float, default=0.10, help="tamano real del lado de la senal en metros")
    ap.add_argument("--esfuerzo-qr", type=int, default=1, choices=(1, 2),
                    help="1 = rapido en vivo (default), 2 = mas alcance pero mas lento")
    ap.add_argument("--dry", action="store_true",
                    help="modo seguro: no publica en /cmd_vel, solo muestra camara y senales")
    args = ap.parse_args()

    import rclpy
    rclpy.init()
    node = rclpy.create_node("probar_senales_teleop")
    sender = Sender(node, args.topic_cmd, args.dry)
    cam = RobotCamera(node, args.topic_cam)
    detector = DetectorAsincrono(lado_m=args.lado, esfuerzo_qr=args.esfuerzo_qr)

    win = "Senales + Teleop TB4"
    cv2.namedWindow(win)

    max_l, max_a = START_LINEAR, START_ANGULAR
    direction = (0, 0)
    last_move_key = 0.0
    prev_vel = (0.0, 0.0)

    warned_no_cam = False
    last_sent_cam_t = 0.0
    t_start = time.monotonic()
    period = 1.0 / RATE_HZ

    print("\nEnfoca la ventana 'Senales + Teleop TB4' para controlar el robot.")
    print("Controles: WASD/QE mover | Espacio/X parar | U/J lin | I/K ang | Esc salir")
    if args.dry:
        print("[MODO DRY: no se publican comandos de movimiento]")

    try:
        while rclpy.ok():
            t0 = time.monotonic()
            # Vaciar mensajes acumulados para quedarnos siempre con el frame mas reciente
            rclpy.spin_once(node, timeout_sec=0.005)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.0)

            img = cam.latest()
            if img is not None and (t0 - cam.last_t) <= 1.5:
                if cam.last_t != last_sent_cam_t:
                    last_sent_cam_t = cam.last_t
                    detector.enviar(img)
                encontradas, color_qrs = detector.resultado()
                canvas = dibujar_senales(img, encontradas, color_qrs)
            else:
                encontradas = []
                if not warned_no_cam and (t0 - t_start) > 8.0:
                    warned_no_cam = True
                    print(f"[camara] Sin imagen en {args.topic_cam}. "
                          "Revisa ROS_DOMAIN_ID, ROS_STATIC_PEERS y red.")
                canvas = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(canvas, f"Esperando camara: {args.topic_cam}", (20, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                if key == 27:
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

            if direction != (0, 0) and (t0 - last_move_key) > HOLD_TIMEOUT:
                direction = (0, 0)

            linear = direction[0] * max_l
            angular = direction[1] * max_a

            if linear or angular:
                sender.send(linear, angular)
            elif prev_vel != (0.0, 0.0):
                sender.stop()
            prev_vel = (linear, angular)

            draw_hud(canvas, linear, angular, max_l, max_a, args.dry, encontradas)
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
        detector.cerrar()
        try:
            sender.stop()
        finally:
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.try_shutdown()
            print("\nDetenido. Adios.")


if __name__ == "__main__":
    sys.exit(main())
