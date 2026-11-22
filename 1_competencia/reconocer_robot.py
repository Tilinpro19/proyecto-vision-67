#!/usr/bin/env python3
"""Reconoce caras con la camara del TurtleBot y dice quien es por la terminal.

Corre en la LAPTOP/WSL (OpenCV >= 4.8; la Pi trae 4.6 y YuNet 2023mar no detecta ahi).
SOLO se suscribe a la camara: no publica nada, el robot no se mueve.

  python3 reconocer_robot.py                      # 30 s, imprime el nombre cada ~1.5 s
  python3 reconocer_robot.py --seg 60 --voz       # ademas lo dice en voz alta (una vez por persona)
  python3 reconocer_robot.py --ver                # con ventana (necesita pantalla / WSLg)

Guarda en --foto-dir la mejor foto de cada persona vista con seguridad, con la caja y el
nombre dibujados: es la "foto de evidencia" del reto.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

import facial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/oakd/rgb/preview/image_raw/compressed")
    ap.add_argument("--seg", type=float, default=30.0, help="duracion total")
    ap.add_argument("--cada", type=float, default=1.5, help="s entre identificaciones")
    ap.add_argument("--frames", type=int, default=5, help="frames por identificacion")
    ap.add_argument("--foto-dir", default="fotos_reconocidas")
    ap.add_argument("--voz", action="store_true", help="decir el nombre la primera vez que se ve")
    ap.add_argument("--ver", action="store_true", help="mostrar ventana con la camara")
    args = ap.parse_args()

    rostros = facial.Rostros()
    galeria = facial.Galeria(rostros, facial.GALERIA)
    print(f"Galeria {galeria.resumen()}")
    if not galeria.nombres:
        raise SystemExit("La galeria esta vacia: pon carpetas con fotos en galeria/")
    rostros.calentar()
    os.makedirs(args.foto_dir, exist_ok=True)
    voz = facial.Voz("auto") if args.voz else None

    rclpy.init()
    node = rclpy.create_node("reconocer_robot")
    ult = {"msg": None}
    node.create_subscription(CompressedImage, args.topic,
                             lambda m: ult.update(msg=m), qos_profile_sensor_data)
    t_fin = time.monotonic() + args.seg
    frames, mejores, dichos, t_aviso = [], {}, set(), 0.0
    try:
        while time.monotonic() < t_fin:
            rclpy.spin_once(node, timeout_sec=0.05)
            msg, ult["msg"] = ult["msg"], None
            if msg is None:
                if time.monotonic() > t_aviso:
                    print("... esperando imagen de la camara (revisa topic / DDS)", flush=True)
                    t_aviso = time.monotonic() + 3
                continue
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            if args.ver:
                cv2.imshow("reconocer_robot", img)
                cv2.waitKey(1)
            frames.append(img)
            if len(frames) < args.frames:
                continue
            res = facial.identificar(frames, rostros, galeria)
            ultimo, frames = frames[-1], []
            if res.nombre is None:
                print("-> no veo ninguna cara", flush=True)
            else:
                nombre = res.nombre.replace("_", " ")
                top = ", ".join(f"{n} {s:.2f}" for n, s in res.ranking[:3])
                print(f"-> {nombre}  score {res.score:.2f}  "
                      f"{'SEGURO' if res.seguro else 'dudoso'}  [{top}]", flush=True)
                if res.seguro:
                    if res.score > mejores.get(res.nombre, 0.0):
                        mejores[res.nombre] = res.score
                        big, faces = rostros.detectar(ultimo)
                        ruta = os.path.join(args.foto_dir, f"{res.nombre}.jpg")
                        cv2.imwrite(ruta, facial.dibujar(big.copy(), faces, res))
                    if voz and res.nombre not in dichos:
                        dichos.add(res.nombre)
                        voz.decir(f"La persona es {nombre}")
            time.sleep(args.cada)
    except KeyboardInterrupt:
        pass
    finally:
        print("Vistos con seguridad:",
              ", ".join(f"{n} ({s:.2f})" for n, s in mejores.items()) or "nadie")
        if voz:
            voz.cerrar()
        if args.ver:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
