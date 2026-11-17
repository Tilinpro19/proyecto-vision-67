"""
Detector de senales BLANCO Y NEGRO del profe por forma: flecha izquierda/derecha (circulo con
flecha doblada), prohibido (circulo con barra), bandera de cuadros y QR. Solo OpenCV + numpy;
sin ROS. Complementa a senales.py (que clasifica por color y aqui no sirve).

  detectar(img) -> list[SenalForma]      tipo en {"izq", "der", "prohibido", "bandera", "qr"}

Circulo: contorno externo muy circular con un hueco dentro. El interior se normaliza a 40x40 y se
compara (correlacion) con plantillas (modelos/plantillas_forma.npz, sacadas de tests/senales_ref;
regenerar con `python3 senales_forma.py --generar`). La flecha derecha es el espejo de la izquierda.
Bandera: varios cuadrados oscuros de area parecida agrupados (tras una erosion que separa esquinas).
QR: se delega en senales.decodificar_qr.
"""
import argparse
import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
PLANTILLAS = os.path.join(AQUI, "modelos", "plantillas_forma.npz")
REF = os.path.join(AQUI, "tests", "senales_ref", "hoja_contacto.png")
N = 40                       # lado de la normalizacion del interior del circulo
R_INT = 0.74                 # radio (fraccion del radio exterior) que se compara: dentro del aro
ANCHO_TRABAJO = 960          # imagenes mas anchas se reducen (CPU de la Pi)


@dataclass
class SenalForma:
    tipo: str                # 'izq' | 'der' | 'prohibido' | 'bandera' | 'qr'
    confianza: float         # 0-1 (correlacion; en bandera, fraccion de cuadros esperados)
    bbox: tuple              # (x, y, w, h) en la imagen original
    texto: str = None        # solo QR


_PLANT = None


def _plantillas():
    global _PLANT
    if _PLANT is None:
        if not os.path.exists(PLANTILLAS):
            generar_plantillas()
        d = np.load(PLANTILLAS)
        izq = d["izq"].astype(np.float32)
        _PLANT = {"izq": izq, "der": izq[:, ::-1].copy(), "prohibido": d["prohibido"].astype(np.float32)}
    return _PLANT


def _oscuros(gris):
    # Otsu: las senales impresas son negro sobre blanco; robusto a cambios de iluminacion global.
    _, b = cv2.threshold(cv2.GaussianBlur(gris, (3, 3), 0), 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    return b


def _disco(binaria, bbox):
    """Interior del circulo (tinta=1) normalizado a NxN, o None si el recorte no es valido."""
    x, y, w, h = bbox
    lado = max(w, h)
    cx, cy = x + w / 2.0, y + h / 2.0
    x0, y0 = int(round(cx - lado / 2.0)), int(round(cy - lado / 2.0))
    rec = binaria[max(y0, 0):y0 + lado, max(x0, 0):x0 + lado]
    if rec.size == 0 or min(rec.shape) < 8:
        return None
    rec = cv2.resize(rec, (N, N), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
    yy, xx = np.mgrid[0:N, 0:N]
    mask = (xx - (N - 1) / 2.0) ** 2 + (yy - (N - 1) / 2.0) ** 2 <= (R_INT * N / 2.0) ** 2
    return np.where(mask, rec, 0.0)


def _corr(a, b):
    a, b = a.ravel() - a.mean(), b.ravel() - b.mean()
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d > 1e-6 else 0.0


def _circulos(binaria, area_min):
    cs, jer = cv2.findContours(binaria, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if jer is None:
        return
    for i, c in enumerate(cs):
        if jer[0][i][3] != -1 or jer[0][i][2] == -1:          # solo externos con hueco
            continue
        a = cv2.contourArea(c)
        if a < area_min:
            continue
        per = cv2.arcLength(c, True)
        if per <= 0 or 4 * np.pi * a / (per * per) < 0.85:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if 0.85 < w / float(h) < 1.18:
            yield (x, y, w, h)


def _bandera(binaria, area_min):
    """Cuadros de un tablero: >= 6 manchas oscuras compactas de area parecida y juntas."""
    k = max(1, int(round(min(binaria.shape) * 0.004))) * 2 + 1
    er = cv2.erode(binaria, np.ones((k, k), np.uint8))       # separa cuadros que se tocan en la esquina
    cs, _ = cv2.findContours(er, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cand = []
    for c in cs:
        a = cv2.contourArea(c)
        if a < area_min / 20.0:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if a / float(w * h) > 0.55 and 0.4 < w / float(h) < 2.5:    # compactos (no aros, no QR gruesos)
            cand.append((a, x, y, w, h))
    if len(cand) < 6:
        return None
    med = np.median([c[0] for c in cand])
    cand = [c for c in cand if 0.4 * med <= c[0] <= 2.5 * med]
    if len(cand) < 6:
        return None
    lado = np.sqrt(med)
    # grupo conexo por cercania (<= 3 lados): el mas grande es la bandera
    pts = np.array([(c[1] + c[3] / 2.0, c[2] + c[4] / 2.0) for c in cand])
    grupo = np.zeros(len(cand), int) - 1
    g = 0
    for i in range(len(cand)):
        if grupo[i] >= 0:
            continue
        pila, grupo[i] = [i], g
        while pila:
            j = pila.pop()
            cerca = np.where((np.hypot(*(pts - pts[j]).T) < 3.2 * lado) & (grupo < 0))[0]
            grupo[cerca] = g
            pila.extend(cerca.tolist())
        g += 1
    mejor = max(range(g), key=lambda k_: int((grupo == k_).sum()))
    sel = [cand[i] for i in range(len(cand)) if grupo[i] == mejor]
    if len(sel) < 6:
        return None
    x0 = min(s[1] for s in sel); y0 = min(s[2] for s in sel)
    x1 = max(s[1] + s[3] for s in sel); y1 = max(s[2] + s[4] for s in sel)
    return (x0, y0, x1 - x0, y1 - y0), min(1.0, len(sel) / 12.0)


def detectar(img, con_qr=True, umbral=0.55, margen=0.10, area_min_frac=0.0008):
    """Senales por forma en img (BGR o gris). Devuelve lista de SenalForma ordenada por confianza.
    umbral = correlacion minima con la plantilla; margen = ventaja minima sobre la 2da mejor
    (asi izq/der y prohibido no se confunden)."""
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    esc = min(1.0, ANCHO_TRABAJO / float(gris.shape[1]))
    g = cv2.resize(gris, None, fx=esc, fy=esc, interpolation=cv2.INTER_AREA) if esc < 1 else gris
    b = _oscuros(g)
    area_min = max(60.0, g.shape[0] * g.shape[1] * area_min_frac)
    out = []
    pl = _plantillas()
    for bb in _circulos(b, area_min):
        d = _disco(b, bb)
        if d is None:
            continue
        pun = sorted(((_corr(d, p), t) for t, p in pl.items()), reverse=True)
        if pun[0][0] >= umbral and pun[0][0] - pun[1][0] >= margen:
            out.append(SenalForma(pun[0][1], pun[0][0], _a_orig(bb, esc)))
    r = _bandera(b, area_min)
    if r:
        out.append(SenalForma("bandera", r[1], _a_orig(r[0], esc)))
    if con_qr:
        import senales
        for texto, quad in senales.decodificar_qr(img):
            x, y, w, h = cv2.boundingRect(np.asarray(quad, np.float32).reshape(-1, 1, 2))
            out.append(SenalForma("qr", 1.0, (x, y, w, h), texto))
    return sorted(out, key=lambda s: -s.confianza)


def _a_orig(bb, esc):
    return tuple(int(round(v / esc)) for v in bb)


def generar_plantillas(ref=REF, destino=PLANTILLAS):
    """Plantillas desde la hoja de contacto del profe (circulos: izq y prohibido de la 1a fila)."""
    im = cv2.imread(ref, 0)
    if im is None:
        raise FileNotFoundError(ref)
    b = _oscuros(im)
    cs = sorted(_circulos(b, 20000), key=lambda t: (t[0], t[1]))
    # columna 0 = flechas (arriba izq, abajo der); columna siguiente = prohibido
    izq = min((c for c in cs if c[0] < 300), key=lambda t: t[1])
    pro = min((c for c in cs if 600 < c[0] < 800), key=lambda t: t[1])
    np.savez_compressed(destino, izq=_disco(b, izq) > 0.5, prohibido=_disco(b, pro) > 0.5)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("imagen", nargs="?")
    ap.add_argument("--generar", action="store_true", help="regenera las plantillas")
    a = ap.parse_args()
    if a.generar:
        generar_plantillas()
        print("plantillas ->", PLANTILLAS)
    if a.imagen:
        img = cv2.imread(a.imagen)
        for s in detectar(img):
            print(s)


if __name__ == "__main__":
    sys.exit(main())
