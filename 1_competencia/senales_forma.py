"""
Detector de senales en blanco y negro por forma (Tarea A guiada, sin color).
Solo OpenCV + numpy; sin ROS.

Tipos detectados (`Senal.tipo`):
  - "izq"       : circulo con flecha doblada a la izquierda
  - "der"       : circulo con flecha doblada a la derecha
  - "prohibido" : circulo con barra diagonal (no pasar)
  - "bandera"   : bandera a cuadros (meta B -> escanear caras)
  - "qr"        : codigo QR (Senal.texto = contenido decodificado)

Alcance medido con camara simulada (FOV=69 deg, senal impresa de 10 cm de lado,
>=80 % de deteccion con JPEG 60 + desenfoque de movimiento de 4 px):
  Resolucion    izq / der / prohibido    bandera (10 cm)    QR (10 cm)
    250 px            1.2 m                  1.2 m            0.4 m
    640 px            2.6 m                  2.6 m            1.2 m
   1280 px            3.8 m                  3.8 m            2.4 m

En el robot: usar `buscar_candidatos(preview)` en el preview chico (250 px) o detectar
formas directamente; cuando aparezca un candidato o se sospeche QR, usar `detectar(img)`.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

from senales import decodificar_qr

TIPOS = ("izq", "der", "prohibido", "bandera", "qr")

# Plantillas binarias 32x32 extraidas de las formas vectoriales del .docx del profe
_ROWS_IZQ = [
    1044480, 8388096, 33554304, 66064320, 125829600, 251658480, 503316600, 1006633020,
    940572700, 1883226126, 1887434766, 1627388934, 3774873095, 3766484743, 3761250055, 3759148807,
    3758100359, 3758100359, 3758100359, 3758100359, 1610616710, 1879052174, 1879052174, 939528092,
    1006636988, 503316600, 251658480, 125829600, 65013696, 33554304, 8388096, 1044480,
]
_ROWS_PROH = [
    1044480, 8388096, 33554176, 67108800, 133701600, 264242160, 528482808, 1069547768,
    1071644796, 2112880702, 2029518878, 2021392414, 4164812831, 4028563471, 4027547663, 4027039759,
    4026785807, 4026658831, 4026595343, 4160781343, 2013281822, 2013273886, 2080378814, 1040189436,
    1056965628, 528482808, 264242160, 133701600, 67108800, 33554304, 8388096, 1044480,
]
_ROWS_BANDERA = [
    3145728, 3932160, 3670016, 7895040, 59013120, 126351360, 126322688, 108990208,
    7868160, 15793920, 15790080, 118419456, 252645376, 252644864, 536022784, 284223360,
    32537472, 33060864, 34576384, 504332800, 1041204096, 1055925216, 2045788144, 1106787888,
    32497408, 1032192, 235397312, 2114191480, 4261412988, 4227858462, 3758096390, 2147483649,
]


def _unpack32(rows):
    a = np.array(rows, dtype=np.uint32)[:, None]
    bits = (a >> np.arange(31, -1, -1, dtype=np.uint32)) & 1
    return bits.astype(np.float32)


_TMPL_IZQ = _unpack32(_ROWS_IZQ)
_TMPL_DER = np.fliplr(_TMPL_IZQ)
_TMPL_PROH = _unpack32(_ROWS_PROH)
_TMPL_BAND = _unpack32(_ROWS_BANDERA)

_YY, _XX = np.mgrid[0:32, 0:32]
_NX = (_XX - 15.5) / 15.5
_NY = (_YY - 15.5) / 15.5
_R = np.hypot(_NX, _NY)
_RING_MASK = (_R >= 0.74) & (_R <= 0.99)
_INNER_MASK = _R <= 0.68
_OUTER_CORNER_MASK = _R >= 1.08
_BOT_MASK = _INNER_MASK & (_NY >= 0.08) & (_NY <= 0.58)
_TOP_MASK = _INNER_MASK & (_NY >= -0.58) & (_NY <= -0.08)
_Q1_MASK = (_R <= 0.62) & (_NX > 0.10) & (_NY < -0.10)
_Q2_MASK = (_R <= 0.62) & (_NX < -0.10) & (_NY < -0.10)
_Q3_MASK = (_R <= 0.62) & (_NX < -0.10) & (_NY > 0.10)
_Q4_MASK = (_R <= 0.62) & (_NX > 0.10) & (_NY > 0.10)

_CIRCLE_TEMPLATES = {
    "izq": _TMPL_IZQ[_INNER_MASK],
    "der": _TMPL_DER[_INNER_MASK],
    "prohibido": _TMPL_PROH[_INNER_MASK],
}
_BAND_VARIANTS = [
    _TMPL_BAND,
    np.rot90(_TMPL_BAND, 1),
    np.rot90(_TMPL_BAND, 2),
    np.rot90(_TMPL_BAND, 3),
    np.fliplr(_TMPL_BAND),
    np.fliplr(np.rot90(_TMPL_BAND, 1)),
]


@dataclass
class Senal:
    tipo: str               # "izq" | "der" | "prohibido" | "bandera" | "qr"
    confianza: float        # 0.0 .. 1.0
    bbox: tuple             # (x, y, w, h) en pixeles enteros
    texto: str = None       # contenido para tipo == "qr"; None en el resto
    distancia_m: float = None


def _corr(a, b):
    a = a.astype(np.float32).ravel()
    b = b.astype(np.float32).ravel()
    sa, sb = float(a.std()), float(b.std())
    if sa < 1e-6 or sb < 1e-6:
        return 0.0
    return float(np.dot(a - a.mean(), b - b.mean()) / (len(a) * sa * sb))


def _iou(box1, box2):
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2
    xa, ya = max(x1, x2), max(y1, y2)
    xb, yb = min(x1 + w1, x2 + w2), min(y1 + h1, y2 + h2)
    inter = max(0, xb - xa) * max(0, yb - ya)
    if inter == 0:
        return 0.0
    return inter / float(w1 * h1 + w2 * h2 - inter)


def _contiene(box_grande, box_chica, tol=4):
    x1, y1, w1, h1 = box_grande
    x2, y2, w2, h2 = box_chica
    return (x2 >= x1 - tol and y2 >= y1 - tol
            and x2 + w2 <= x1 + w1 + tol and y2 + h2 <= y1 + h1 + tol)


def _estimar_dist(w_px, h_px, ancho_img, lado_m=0.10, fov_deg=69.0):
    lado_px = max(w_px, h_px)
    if not lado_m or lado_px <= 0 or ancho_img <= 0:
        return None
    f = ancho_img / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    return float(lado_m * f / lado_px)


def _clasificar_circulo(gray_crop):
    """Evalua si un recorte corresponde a una senal circular (izq, der, prohibido).
    Retorna (tipo, confianza) o (None, 0.0)."""
    if gray_crop.size == 0 or min(gray_crop.shape[:2]) < 16:
        return None, 0.0
    if float(gray_crop.std()) < 18.0:
        return None, 0.0

    c32 = cv2.resize(gray_crop, (32, 32), interpolation=cv2.INTER_AREA)
    _, bw_u8 = cv2.threshold(c32, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    bw = (bw_u8 > 0).astype(np.float32)

    ring_dark = float(bw[_RING_MASK].mean())
    inner_dark = float(bw[_INNER_MASK].mean())
    corner_dark = float(bw[_OUTER_CORNER_MASK].mean())
    if ring_dark < 0.45 or corner_dark > 0.35:
        return None, 0.0
    if not (0.12 <= inner_dark <= 0.52):
        return None, 0.0

    # El anillo exterior debe rodear los 4 cuadrantes (evita marcos parciales o letras)
    for qmask in (_RING_MASK & (_NX >= 0) & (_NY >= 0),
                  _RING_MASK & (_NX < 0) & (_NY >= 0),
                  _RING_MASK & (_NX >= 0) & (_NY < 0),
                  _RING_MASK & (_NX < 0) & (_NY < 0)):
        if float(bw[qmask].mean()) < 0.30:
            return None, 0.0

    inner_vec = bw[_INNER_MASK]
    scores = {k: _corr(inner_vec, tmpl) for k, tmpl in _CIRCLE_TEMPLATES.items()}
    orden = sorted(scores.items(), key=lambda kv: -kv[1])
    mejor_tipo, mejor_score = orden[0]
    segundo_score = orden[1][1]

    if mejor_score < 0.42 or (mejor_score - segundo_score) < 0.14:
        return None, 0.0

    # Verificacion geometrica invariante para nunca confundir izq, der y prohibido
    bot_pts = bw[_BOT_MASK] > 0.5
    top_pts = bw[_TOP_MASK] > 0.5
    if int(bot_pts.sum()) < 3 or int(top_pts.sum()) < 3:
        return None, 0.0
    bot_x = float(_NX[_BOT_MASK][bot_pts].mean())
    top_x = float(_NX[_TOP_MASK][top_pts].mean())
    q1_dark = float(bw[_Q1_MASK].mean())
    q2_dark = float(bw[_Q2_MASK].mean())
    q3_dark = float(bw[_Q3_MASK].mean())
    q4_dark = float(bw[_Q4_MASK].mean())

    if mejor_tipo == "izq":
        # Tallo vertical abajo a la derecha (bot_x > 0), codo en Q1 y cuadrante Q3 (abajo-izq) vacio
        if bot_x < 0.10 or q1_dark < 0.10 or q3_dark > 0.20:
            return None, 0.0
    elif mejor_tipo == "der":
        # Espejo horizontal: tallo vertical abajo a la izquierda (bot_x < 0), codo en Q2 y Q4 vacio
        if bot_x > -0.10 or q2_dark < 0.10 or q4_dark > 0.20:
            return None, 0.0
    elif mejor_tipo == "prohibido":
        # Barra diagonal de arriba-izquierda (top_x < 0) a abajo-derecha (bot_x > 0), Q1 y Q3 vacios
        if top_x > -0.08 or bot_x < 0.08 or q1_dark > 0.14 or q3_dark > 0.14:
            return None, 0.0

    conf = float(np.clip(0.5 * mejor_score + 0.3 * ring_dark + 0.2 * (mejor_score - segundo_score), 0.0, 1.0))
    return mejor_tipo, conf


def _es_bandera(gray_crop):
    """Evalua si un recorte contiene el patron de la bandera de cuadros.
    Retorna (es_valida, confianza)."""
    h, w = gray_crop.shape[:2]
    if h < 14 or w < 14 or float(gray_crop.std()) < 22.0:
        return False, 0.0
    aspect = max(w, h) / max(min(w, h), 1)
    if aspect > 3.2:
        return False, 0.0

    thr, bw = cv2.threshold(gray_crop, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    if thr > 155:
        _, bw = cv2.threshold(gray_crop, 140, 255, cv2.THRESH_BINARY_INV)
    ys, xs = np.where(bw > 0)
    if len(xs) < 25:
        return False, 0.0
    sub = bw[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    sh, sw = sub.shape[:2]
    if sh < 12 or sw < 12:
        return False, 0.0

    dark_frac = float((sub > 0).mean())
    if not (0.20 <= dark_frac <= 0.62):
        return False, 0.0

    # Transiciones blanco-negro en multiples cortes horizontales y verticales
    b01 = (sub > 0).astype(np.int8)
    cortes_h = b01[np.linspace(int(0.15 * sh), int(0.85 * sh), 7).astype(int), :]
    cortes_v = b01[:, np.linspace(int(0.15 * sw), int(0.85 * sw), 7).astype(int)]
    trans_h = float(np.mean(np.sum(np.abs(np.diff(cortes_h, axis=1)), axis=1)))
    trans_v = float(np.mean(np.sum(np.abs(np.diff(cortes_v, axis=0)), axis=0)))
    if max(trans_h, trans_v) < 3.8 or min(trans_h, trans_v) < 2.5:
        return False, 0.0

    c32 = cv2.resize((sub > 0).astype(np.float32), (32, 32), interpolation=cv2.INTER_AREA)
    best_corr = max(_corr(c32, v) for v in _BAND_VARIANTS)
    if best_corr < 0.42:
        return False, 0.0

    conf = float(np.clip(0.65 * best_corr + 0.35 * min(min(trans_h, trans_v) / 6.0, 1.0), 0.0, 1.0))
    return True, conf


def _a_gris(img):
    if img.ndim == 2:
        return img
    if img.shape[2] == 4:
        alpha = img[:, :, 3:4].astype(np.float32) / 255.0
        bgr = (img[:, :, :3].astype(np.float32) * alpha + 255.0 * (1.0 - alpha)).astype(np.uint8)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def detectar(img, lado_m=0.10, fov_deg=69.0, esfuerzo_qr=2):
    """Detecta todas las senales en `img` (BGR, BGRA o escala de grises).
    Devuelve `list[Senal]` ordenada de mayor a menor area en imagen."""
    if img is None or img.size == 0:
        return []
    gray = _a_gris(img)
    H, W = gray.shape[:2]
    out = []
    ocupados = []

    # 1. Codigos QR (reutiliza senales.decodificar_qr que agrega borde blanco y reescala)
    for texto, quad in decodificar_qr(img, esfuerzo=esfuerzo_qr):
        xs, ys = quad[:, 0], quad[:, 1]
        x0, y0 = max(int(np.floor(xs.min())), 0), max(int(np.floor(ys.min())), 0)
        x1, y1 = min(int(np.ceil(xs.max())), W), min(int(np.ceil(ys.max())), H)
        bw, bh = max(1, x1 - x0), max(1, y1 - y0)
        bbox = (x0, y0, bw, bh)
        ocupados.append(bbox)
        out.append(Senal("qr", 0.98, bbox, texto=texto,
                         distancia_m=_estimar_dist(bw, bh, W, lado_m, fov_deg)))

    # 2. Binarizacion multi-umbral para hallar contornos circulares (izq, der, prohibido)
    blur = cv2.GaussianBlur(gray, (3, 3), 0.8)
    _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    adapt = cv2.adaptiveThreshold(
        blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
        max(15, (min(H, W) // 8) | 1), 7
    )

    candidatos_circ = []
    for mask in (otsu, adapt):
        cs, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            x, y, w, h = cv2.boundingRect(c)
            if w < 18 or h < 18 or w > int(0.92 * W) or h > int(0.92 * H):
                continue
            ratio = w / float(h)
            if not (0.55 <= ratio <= 1.45):
                continue
            area = cv2.contourArea(c)
            if area < 0.42 * w * h:
                continue
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            if hull_area <= 0 or (area / hull_area) < 0.80:
                continue
            perim = cv2.arcLength(c, True)
            if perim <= 0:
                continue
            circ = 4.0 * math.pi * area / (perim * perim)
            if circ < 0.52:
                continue
            candidatos_circ.append((x, y, w, h))

    # Evaluar candidatos circulares de mayor a menor y descartar solapados solo contra aceptados
    candidatos_circ.sort(key=lambda b: -(b[2] * b[3]))
    vistos_circ = []
    for box in candidatos_circ:
        if any(_iou(box, b) > 0.85 for b in vistos_circ):
            continue
        vistos_circ.append(box)
        if any(_iou(box, b) > 0.30 or _contiene(b, box) for b in ocupados):
            continue
        x, y, w, h = box
        tipo, conf = _clasificar_circulo(gray[y:y + h, x:x + w])
        if tipo is not None:
            ocupados.append(box)
            out.append(Senal(tipo, conf, box, texto=None,
                             distancia_m=_estimar_dist(w, h, W, lado_m, fov_deg)))

    # 3. Bandera de cuadros: usar mascara de tinta oscura (<= 135) para no fusionar paredes grises
    thr_otsu, _ = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    _, mask_band = cv2.threshold(blur, min(float(thr_otsu), 135.0), 255, cv2.THRESH_BINARY_INV)
    for x, y, w, h in ocupados:
        pad = 4
        mask_band[max(0, y - pad):min(H, y + h + pad), max(0, x - pad):min(W, x + w + pad)] = 0

    # Si toda la imagen ya es una bandera suelta (como bandera_cuadros.png)
    if not ocupados:
        ys, xs = np.where(mask_band > 0)
        if len(xs) > 50:
            bx0, by0, bx1, by1 = int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1
            bw, bh = bx1 - bx0, by1 - by0
            if bw * bh >= 0.25 * W * H:
                ok_b, conf_b = _es_bandera(gray[by0:by1, bx0:bx1])
                if ok_b:
                    box = (bx0, by0, bw, bh)
                    ocupados.append(box)
                    out.append(Senal("bandera", conf_b, box, texto=None,
                                     distancia_m=_estimar_dist(bw, bh, W, lado_m, fov_deg)))

    k_close = max(5, int(round(0.015 * min(H, W))) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k_close, k_close))
    closed = cv2.morphologyEx(mask_band, cv2.MORPH_CLOSE, kernel)
    cs_b, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in sorted(cs_b, key=cv2.contourArea, reverse=True):
        x, y, w, h = cv2.boundingRect(c)
        if w < 14 or h < 14 or w * h < 220:
            continue
        box = (x, y, w, h)
        if any(_iou(box, b) > 0.25 or _contiene(b, box) or _contiene(box, b) for b in ocupados):
            continue
        ok_b, conf_b = _es_bandera(gray[y:y + h, x:x + w])
        if ok_b:
            ocupados.append(box)
            out.append(Senal("bandera", conf_b, box, texto=None,
                             distancia_m=_estimar_dist(w, h, W, lado_m, fov_deg)))

    out.sort(key=lambda s: -(s.bbox[2] * s.bbox[3]))
    return out


def buscar_candidatos(img, area_min_frac=0.0015):
    """Filtro rapido para preview de baja resolucion (250 px): devuelve lista de bboxes (x, y, w, h)
    con alto contraste local blanco/negro que justifican correr `detectar()` o pedir alta resolucion."""
    if img is None or img.size == 0:
        return []
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape[:2]
    if float(gray.std()) < 14.0:
        return []
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    if float((bw > 0).mean()) > 0.65:
        return []
    k = max(3, int(round(0.015 * max(H, W))) | 1)
    closed = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    cs, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cs:
        x, y, w, h = cv2.boundingRect(c)
        if w * h >= area_min_frac * H * W and 0.35 <= (w / float(h)) <= 2.8:
            crop = gray[y:y + h, x:x + w]
            if float(crop.std()) >= 25.0:
                out.append((x, y, w, h))
    return sorted(out, key=lambda b: -(b[2] * b[3]))
