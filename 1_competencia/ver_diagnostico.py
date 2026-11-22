"""Muestra un diagnostico de logs/*.npz: el LiDAR visto desde arriba y la foto de la camara.

    python3 ver_diagnostico.py logs/mision_A_..._001_decision_der.npz [mas.npz ...]
    python3 ver_diagnostico.py logs/            # todos los npz de la carpeta, en orden

Teclas: cualquier tecla = siguiente, q = salir. --guardar escribe un .png al lado de cada npz.
"""
import argparse
import glob
import os

import cv2
import numpy as np

ESCALA, LADO = 160, 480      # px por metro; ventana del LiDAR de 3 x 3 m


def dibujar_scan(z):
    img = np.full((LADO, LADO, 3), 255, np.uint8)
    c = LADO // 2
    for m in (0.5, 1.0, 1.5):
        cv2.circle(img, (c, c), int(m * ESCALA), (220, 220, 220), 1)
    r, a = z["ranges"].astype(float), z["angles"].astype(float)
    ok = np.isfinite(r) & (r > 0.05)
    x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
    for px, py in zip(c - y * ESCALA, c - x * ESCALA):       # adelante = arriba
        if 0 <= px < LADO and 0 <= py < LADO:
            cv2.circle(img, (int(px), int(py)), 2, (0, 0, 200), -1)
    cv2.circle(img, (c, c), int(0.17 * ESCALA), (0, 150, 0), 2)           # el robot
    cv2.line(img, (c, c), (c, c - int(0.3 * ESCALA)), (0, 150, 0), 2)
    return img


def componer(ruta):
    z = np.load(ruta)
    scan = dibujar_scan(z)
    if "jpg" in z.files:
        foto = cv2.imdecode(z["jpg"], cv2.IMREAD_COLOR)
        foto = cv2.resize(foto, (int(foto.shape[1] * LADO / foto.shape[0]), LADO))
    else:
        foto = np.full((LADO, 320, 3), 200, np.uint8)
        cv2.putText(foto, "sin foto", (100, LADO // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    out = np.hstack([scan, foto])
    pose = z["pose"] if "pose" in z.files else [0, 0, 0]
    txt = f"{os.path.basename(ruta)}  pose=({pose[0]:.2f}, {pose[1]:.2f}, {np.degrees(pose[2]):.0f} gr)"
    cv2.putText(out, txt, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    if "nota" in z.files:
        cv2.putText(out, str(z["nota"])[:110], (5, LADO - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("rutas", nargs="+")
    ap.add_argument("--guardar", action="store_true", help="escribir .png en vez de mostrar")
    args = ap.parse_args()
    rutas = []
    for r in args.rutas:
        rutas += sorted(glob.glob(os.path.join(r, "*.npz"))) if os.path.isdir(r) else [r]
    for ruta in rutas:
        img = componer(ruta)
        nota = str(np.load(ruta)["nota"]) if "nota" in np.load(ruta).files else ""
        print(os.path.basename(ruta), nota)
        if args.guardar:
            cv2.imwrite(os.path.splitext(ruta)[0] + ".png", img)
            continue
        cv2.imshow("diagnostico", img)
        if cv2.waitKey(0) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()
