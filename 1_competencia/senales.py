"""
Lectura de las senales del profe: un codigo QR (su texto) + un color (azul = avanzar,
rojo = regresar). Solo OpenCV + numpy; sin ROS (lo llama quien tenga la imagen).

Dos etapas, pensadas para la camara del TurtleBot 4:
  1. buscar_manchas(img)  barata: manchas azules/rojas en la imagen chica (preview). Dice DONDE mirar.
  2. leer(img)            precisa: decodifica el QR y clasifica el color. Pide imagen de ALTA resolucion
                          (el QR de 10 cm no se lee en un preview de ~250 px: ver tests/render_senales.py).

No fija los nombres de las senales: lee cualquier QR (el profe permite agregar mas).
El template del profe NO trae borde blanco alrededor del QR; aqui se le agrega para poder leerlo.
El color puede venir en la tinta del QR o en un marco alrededor; 'auto' prueba ambos.

  python3 senales.py foto.jpg [--lado 0.10] [--manchas]
"""
import argparse
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

AZUL, ROJO = "azul", "rojo"
ACCION = {AZUL: "avanzar", ROJO: "regresar"}     # lo que dijo el profe; configurable


@dataclass
class ParamsColor:
    s_min: int = 70                 # saturacion minima (0-255) para contar como "de color"
    v_min: int = 40                 # brillo minimo
    azul: tuple = (95, 135)         # H de OpenCV (0-179)
    rojo: tuple = (10, 170)         # rojo = H <= 10  o  H >= 170 (da la vuelta)
    frac_tinta: float = 0.30        # fraccion de pixeles de tinta que debe ser del color
    frac_marco: float = 0.20        # fraccion del anillo exterior que debe ser del color
    anillo: tuple = (1.3, 1.9)      # anillo del marco, en veces el lado del QR


@dataclass
class Senal:
    texto: str
    color: str            # 'azul' | 'rojo' | None
    modo_color: str       # 'tinta' | 'marco' | ''
    frac_color: float
    esquinas: np.ndarray  # (4, 2) en pixeles de la imagen original
    lado_px: float
    distancia_m: float    # estimada por el tamano; None si no se conoce lado_m
    accion: str = field(init=False)

    def __post_init__(self):
        self.accion = ACCION.get(self.color)


# --- color --------------------------------------------------------------------------

def mascara_color(hsv, color, p=ParamsColor()):
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    base = (s >= p.s_min) & (v >= p.v_min)
    if color == AZUL:
        return base & (h >= p.azul[0]) & (h <= p.azul[1])
    return base & ((h <= p.rojo[0]) | (h >= p.rojo[1]))


def _poligono(shape, quad, escala):
    c = quad.mean(axis=0)
    q = (c + (quad - c) * escala).astype(np.int32)
    m = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(m, [q], 255)
    return m > 0


def _dominante(hsv, region, p, frac_min):
    n = int(region.sum())
    if n < 12:
        return None, 0.0
    fr = {c: float(mascara_color(hsv, c, p)[region].sum()) / n for c in (AZUL, ROJO)}
    c = max(fr, key=fr.get)
    return (c, fr[c]) if fr[c] >= frac_min else (None, fr[c])


def clasificar_color(img, quad, p=ParamsColor(), modo="auto"):
    """(color, modo, fraccion). modo: 'tinta' (los modulos oscuros del QR), 'marco' (anillo
    alrededor del papel) o 'auto' (el que de mas fraccion)."""
    if img.ndim == 2:
        return None, "", 0.0
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    quad = np.asarray(quad, float).reshape(4, 2)
    res = {}
    if modo in ("auto", "tinta"):
        dentro = _poligono(img.shape, quad, 1.0)
        g = gray[dentro]
        if len(g) >= 16:
            thr, _ = cv2.threshold(g.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            res["tinta"] = _dominante(hsv, dentro & (gray <= thr), p, p.frac_tinta)
    if modo in ("auto", "marco"):
        anillo = _poligono(img.shape, quad, p.anillo[1]) & ~_poligono(img.shape, quad, p.anillo[0])
        res["marco"] = _dominante(hsv, anillo, p, p.frac_marco)
    hay = {m: v for m, v in res.items() if v[0]}
    if not hay:
        return None, "", max((v[1] for v in res.values()), default=0.0)
    m = max(hay, key=lambda k: hay[k][1])
    return hay[m][0], m, hay[m][1]


# --- QR -----------------------------------------------------------------------------

def _detectores():
    dets = [cv2.QRCodeDetector()]
    if hasattr(cv2, "QRCodeDetectorAruco"):        # OpenCV >= 4.8: mejor con QR chicos
        dets.insert(0, cv2.QRCodeDetectorAruco())
    return dets


def _intentar(det, im):
    """[(texto, esquinas(4,2))] que decodifica 'det' en 'im' (una o varias senales)."""
    out = []
    try:
        ok, textos, pts, _ = det.detectAndDecodeMulti(im)
        if ok and pts is not None:
            out += [(t, np.asarray(q, float).reshape(4, 2)) for t, q in zip(textos, pts) if t]
    except cv2.error:
        pass
    if not out:
        try:
            t, pts, _ = det.detectAndDecode(im)
            if t and pts is not None:
                out.append((t, np.asarray(pts, float).reshape(4, 2)))
        except cv2.error:
            pass
    return out


def decodificar_qr(img, esfuerzo=2, roi=None):
    """Lista de (texto, esquinas) de los QR legibles en img (BGR o gris).
    esfuerzo 1 = un intento rapido; 2 = tambien reescalado 2x, borde blanco y canal de tinta;
    3 = ademas binarizado. El reescalado 2x es lo que mas ayuda (medido con OpenCV 4.6 y 4.11).
    roi = (x, y, w, h): decodifica solo ese recorte (con margen) -> mucho mas rapido en alta
    resolucion; las esquinas se devuelven en coordenadas de img."""
    if roi is not None:
        x, y, w, h = roi
        m = int(0.5 * max(w, h))
        x0, y0 = max(x - m, 0), max(y - m, 0)
        x1, y1 = min(x + w + m, img.shape[1]), min(y + h + m, img.shape[0])
        if x1 - x0 < 16 or y1 - y0 < 16:
            return []
        return [(t, q + np.array([x0, y0], float))
                for t, q in decodificar_qr(img[y0:y1, x0:x1], esfuerzo)]
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bases = [g]
    if esfuerzo >= 2 and img.ndim == 3:
        bases.append(img.min(axis=2))              # tinta oscura de cualquier color sobre papel
    if esfuerzo >= 3:
        bases.append(cv2.threshold(cv2.GaussianBlur(g, (0, 0), 1.0), 0, 255,
                                   cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1])
    dets = _detectores()
    # 2x sobre un cuadro completo de alta resolucion es lento: usar roi= o esfuerzo=1 ahi
    escalas = (1.0,) if esfuerzo < 2 or max(g.shape) > 2000 else (1.0, 2.0)
    pads = (0.0,) if esfuerzo < 2 else (0.0, 0.15)     # el template no trae zona silenciosa
    for base in bases:
        for esc in escalas:
            im0 = base if esc == 1.0 else cv2.resize(base, None, fx=esc, fy=esc,
                                                     interpolation=cv2.INTER_CUBIC)
            for pad in pads:
                b = int(pad * min(im0.shape[:2]))
                im = cv2.copyMakeBorder(im0, b, b, b, b, cv2.BORDER_CONSTANT, value=255) if b else im0
                for det in dets:
                    res = _intentar(det, im)
                    if res:
                        vistos, out = set(), []
                        for t, q in res:
                            if t not in vistos:
                                vistos.add(t)
                                out.append((t, (q - b) / esc))
                        return out
    return []


# --- lectura completa ---------------------------------------------------------------

def _lado(quad):
    q = np.asarray(quad, float).reshape(4, 2)
    return float(np.mean([np.linalg.norm(q[i] - q[(i + 1) % 4]) for i in range(4)]))


def leer(img, lado_m=0.10, fov_deg=69.0, K=None, params=ParamsColor(), modo_color="auto",
         esfuerzo=2, roi=None):
    """Senales (QR + color + distancia) legibles en img. lado_m = lado del QR impreso (sin papel);
    K = matriz de la camara (camera_info) o, si no, se estima con fov_deg horizontal.
    roi = (x, y, w, h) para leer solo alrededor de una mancha (ver buscar_manchas)."""
    ancho = img.shape[1]
    f = float(K[0][0]) if K is not None else ancho / (2 * math.tan(math.radians(fov_deg) / 2))
    out = []
    for texto, quad in decodificar_qr(img, esfuerzo, roi):
        color, modo, frac = clasificar_color(img, quad, params, modo_color)
        lado = _lado(quad)
        dist = lado_m * f / lado if lado_m and lado > 0 else None
        out.append(Senal(texto, color, modo, frac, quad, lado, dist))
    return out


def buscar_manchas(img, params=ParamsColor(), area_min_frac=0.0008):
    """Etapa barata (sirve en imagen chica): [(color, (x, y, w, h), area_px)] de manchas azules
    o rojas, mas grandes primero. No lee el QR: solo dice hacia donde frenar y pedir alta resolucion."""
    if img.ndim == 2:
        return []
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    k = max(3, int(round(0.012 * max(h, w))) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    out = []
    for color in (AZUL, ROJO):
        m = cv2.morphologyEx(mascara_color(hsv, color, params).astype(np.uint8) * 255,
                             cv2.MORPH_CLOSE, kernel)
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            x, y, bw, bh = cv2.boundingRect(c)
            area = bw * bh
            if area >= area_min_frac * h * w and 0.4 <= bw / bh <= 2.5:
                out.append((color, (x, y, bw, bh), area))
    return sorted(out, key=lambda t: -t[2])


def main():
    ap = argparse.ArgumentParser(description="Lee QR + color de una o mas imagenes")
    ap.add_argument("imagenes", nargs="+")
    ap.add_argument("--lado", type=float, default=0.10, help="lado del QR impreso en m (sin papel)")
    ap.add_argument("--manchas", action="store_true", help="tambien lista las manchas de color")
    args = ap.parse_args()
    for p in args.imagenes:
        img = cv2.imread(p)
        if img is None:
            print(p, ": no se pudo leer")
            continue
        s = leer(img, args.lado, esfuerzo=3)
        print(f"{p} ({img.shape[1]}x{img.shape[0]}):", "sin senal" if not s else "")
        for x in s:
            d = f"{x.distancia_m:.2f} m" if x.distancia_m else "?"
            print(f"  QR '{x.texto}'  color {x.color or '-'} ({x.modo_color}, {x.frac_color:.0%})  "
                  f"-> {x.accion or '-'}  | lado {x.lado_px:.0f} px, ~{d}")
        if args.manchas:
            for c, bb, a in buscar_manchas(img):
                print(f"  mancha {c} bbox {bb} area {a}")


if __name__ == "__main__":
    main()
