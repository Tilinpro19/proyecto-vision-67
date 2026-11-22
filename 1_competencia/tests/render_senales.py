"""Escenas sinteticas de las senales del profe, para probar senales.py sin camara.

Simula: distancia, resolucion de imagen, angulo (yaw), tinta/papel/marco de color,
desenfoque de movimiento, ruido y compresion JPEG. Los QR son los del .docx del profe
(tests/data/senales/). Tambien imprime la tabla de alcance:

  python3 tests/render_senales.py            # alcance por resolucion y condiciones
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "senales")
NOMBRES = ["milestone_alfa", "milestone_beta", "milestone_gamma",
           "milestone_omega", "milestone_phi", "milestone_rho"]
AZUL, ROJO, NEGRO = (200, 60, 0), (0, 0, 200), (0, 0, 0)     # BGR
FOV = 69.0   # HFOV estimado de la OAK-D Lite (RGB); medir con camera_info


def qr_original(nombre):
    return cv2.imread(os.path.join(DATA, nombre + ".png"), 0)


def hoja(qr_gris, tinta=NEGRO, margen=0.10, marco=None, marco_ancho=0.25, papel=(255, 255, 255)):
    """Hoja impresa (BGR): QR con tinta de color, margen de papel y, opcional, marco de color."""
    h, w = qr_gris.shape
    m = int(round(w * margen))
    img = np.full((h + 2 * m, w + 2 * m, 3), papel, np.uint8)
    img[m:m + h, m:m + w][qr_gris < 128] = tinta
    if marco is not None:
        b = int(round(w * marco_ancho))
        img = cv2.copyMakeBorder(img, b, b, b, b, cv2.BORDER_CONSTANT, value=marco)
    return img


def proyectar(hoja_bgr, qr_px, qr_lado_m, dist_m, ancho_px, yaw_deg=0.0, fov_deg=FOV, fondo=190,
              desplaza=(0.0, 0.0), alto_px=None):
    """Vista de la hoja a dist_m por una camara de ancho_px, girada yaw_deg sobre su eje vertical.
    qr_px = lado del QR dentro de la hoja (px); qr_lado_m = su lado real. La hoja entera
    (margen y marco incluidos) se escala igual."""
    alto_px = alto_px or int(round(ancho_px * 0.75))
    h, w = hoja_bgr.shape[:2]
    escala_m = qr_lado_m / qr_px            # m por pixel de la hoja
    Wm, Hm = w * escala_m, h * escala_m
    f = ancho_px / (2 * math.tan(math.radians(fov_deg) / 2))
    a = math.radians(yaw_deg)
    esquinas3d = []
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        x, y = sx * Wm / 2, sy * Hm / 2
        X, Z = x * math.cos(a) + desplaza[0], dist_m + x * math.sin(a)
        esquinas3d.append((X, y + desplaza[1], Z))
    dst = np.array([[ancho_px / 2 + f * X / Z, alto_px / 2 + f * Y / Z] for X, Y, Z in esquinas3d], np.float32)
    ancho_dst = np.linalg.norm(dst[1] - dst[0])
    src_img = hoja_bgr
    if ancho_dst < w:                       # reduce antes con promedio de area (sin aliasing)
        s = max(ancho_dst / w, 1e-3)
        src_img = cv2.resize(hoja_bgr, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    sh, sw = src_img.shape[:2]
    src = np.array([[0, 0], [sw, 0], [sw, sh], [0, sh]], np.float32)
    Hm_ = cv2.getPerspectiveTransform(src, dst)
    lienzo = np.full((alto_px, ancho_px, 3), fondo, np.uint8)
    return cv2.warpPerspective(src_img, Hm_, (ancho_px, alto_px), dst=lienzo,
                               borderMode=cv2.BORDER_TRANSPARENT, flags=cv2.INTER_LINEAR)


def degradar(img, blur_px=0, jpeg=None, ruido=3.0, sigma=0.6, rng=None):
    """Desenfoque (optica + movimiento horizontal de blur_px), ruido gaussiano y JPEG."""
    rng = rng or np.random.default_rng(0)
    out = cv2.GaussianBlur(img, (0, 0), sigma) if sigma else img.copy()
    if blur_px >= 2:
        k = np.zeros((blur_px, blur_px), np.float32)
        k[blur_px // 2, :] = 1.0 / blur_px
        out = cv2.filter2D(out, -1, k)
    if ruido:
        out = np.clip(out + rng.normal(0, ruido, out.shape), 0, 255).astype(np.uint8)
    if jpeg:
        _, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg)])
        out = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return out


def qr_nuevo(texto, lado_px=300):
    """QR de cualquier texto (OpenCV QRCodeEncoder), recortado a sus modulos (sin borde,
    como el template del profe) y ampliado a lado_px."""
    g = cv2.QRCodeEncoder.create().encode(texto)
    ys, xs = np.where(g < 128)
    g = g[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    return cv2.resize(g, (lado_px, lado_px), interpolation=cv2.INTER_NEAREST)


def escena(nombre, dist_m, ancho_px, qr_lado_m=0.10, tinta=NEGRO, marco=None, yaw_deg=0.0,
           blur_px=0, jpeg=None, ruido=3.0, desplaza=(0.0, 0.0), seed=0, qr_gris=None):
    qr = qr_gris if qr_gris is not None else qr_original(nombre)
    img = proyectar(hoja(qr, tinta, marco=marco), qr.shape[1], qr_lado_m, dist_m, ancho_px,
                    yaw_deg=yaw_deg, desplaza=desplaza)
    return degradar(img, blur_px, jpeg, ruido, rng=np.random.default_rng(seed))


def alcance(ancho_px, qr_lado_m, tinta=NEGRO, yaw_deg=0.0, blur_px=0, jpeg=None, esfuerzo=2,
            dists=None, intentos=4):
    """Mayor distancia (m) con lectura correcta >= 80 % de los intentos (nombres y ruido distintos)."""
    from senales import decodificar_qr
    dists = dists or [round(0.4 + 0.2 * i, 1) for i in range(19)]
    mejor, fallos = 0.0, 0
    for d in dists:
        ok = 0
        for i in range(intentos):
            nombre = NOMBRES[i % len(NOMBRES)]
            img = escena(nombre, d, ancho_px, qr_lado_m, tinta, yaw_deg=yaw_deg, blur_px=blur_px,
                         jpeg=jpeg, seed=i)
            ok += any(t == nombre for t, _ in decodificar_qr(img, esfuerzo))
        if ok / intentos >= 0.8:
            mejor, fallos = d, 0
        else:
            fallos += 1
            if fallos >= 2:          # el alcance es monotono: no sigue probando mas lejos
                break
    return mejor


if __name__ == "__main__":
    import cv2 as _cv
    print("OpenCV", _cv.__version__, "| FOV supuesto", FOV, "grados")
    condiciones = [("ideal", dict()),
                   ("JPEG 60", dict(jpeg=60)),
                   ("JPEG 60 + mov 4px", dict(jpeg=60, blur_px=4)),
                   ("JPEG 60 + mov 4px + yaw 35", dict(jpeg=60, blur_px=4, yaw_deg=35))]
    for lado in (0.10, 0.056):
        print(f"\n== QR de {lado * 100:.1f} cm: distancia maxima (m) con >=80 % de lecturas ==")
        print(f"{'condicion':30s}" + "".join(f"{a:>9d}px" for a in (250, 640, 1280, 1920)))
        for nombre, kw in condiciones:
            fila = "".join(f"{alcance(a, lado, **kw):>11.1f}" for a in (250, 640, 1280, 1920))
            print(f"{nombre:30s}{fila}")
