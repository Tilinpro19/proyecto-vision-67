"""Circuito de referencia reconstruido del video del grupo Carranza (00:00-01:05), para el
simulador. Medidas aproximadas (el video no da cotas); el ancho del pasillo es parametro
(en el video ~0.75-0.90 m). A = (0, 0) mirando a +x.

Tramos (centro del pasillo):
  1 sale recto y dobla a la IZQUIERDA sin senal (solo LiDAR)
  2 flecha IZQ en la caja del fondo -> gira 90 a la izquierda
  3 esquina en L sin flecha -> DERECHA por LiDAR
  4 QR en la pared del fondo -> lo lee y gira a la DERECHA (unico lado libre)
  5 esquina a la DERECHA
  6 flecha IZQ en el zocalo del fondo -> izquierda, entra al pasillo largo
  7 QR al fondo del pasillo largo -> IZQUIERDA
  8 tramo final: PROHIBIDO en la caja lateral izquierda y BANDERA al fondo (+ caras)
"""
import math

IZQ_X, IZQ_Y, DER_X, DER_Y = (-1.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.0, -1.0)  # normales

CENTRO = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.8, 2.0), (0.8, 3.6), (3.2, 3.6),
          (3.2, 1.0), (6.0, 1.0), (6.0, 4.0)]
ATRAS = 0.5      # m de pared detras de A


def _dir(p, q):
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    return ((q[0] - p[0]) / d, (q[1] - p[1]) / d)


def pasillo_poligonal(centro, ancho, atras=ATRAS):
    """Paredes de un pasillo de esquinas a 90 grados que sigue la polilinea `centro`."""
    h = ancho / 2.0
    dirs = [_dir(centro[i], centro[i + 1]) for i in range(len(centro) - 1)]
    normal = [(-dy, dx) for dx, dy in dirs]                 # izquierda de cada tramo
    p0 = (centro[0][0] - atras * dirs[0][0], centro[0][1] - atras * dirs[0][1])
    izq, der = [], []
    for i, c in enumerate(centro):
        if i == 0:
            c, ns = p0, [normal[0]]
        elif i == len(centro) - 1:
            ns = [normal[-1]]
        else:
            ns = [normal[i - 1], normal[i]]
        ox = sum(n[0] for n in ns) * h
        oy = sum(n[1] for n in ns) * h
        izq.append((c[0] + ox, c[1] + oy))
        der.append((c[0] - ox, c[1] - oy))
    segs = [(izq[i], izq[i + 1]) for i in range(len(izq) - 1)]
    segs += [(der[i], der[i + 1]) for i in range(len(der) - 1)]
    return segs + [(izq[0], der[0]), (izq[-1], der[-1])]


def objetos(ancho):
    """Senales y caras pegadas en las paredes (posicion, normal hacia el pasillo)."""
    h = ancho / 2.0
    return [
        ("izq", (2.0, 2.0 + h), DER_Y),            # tramo 2: caja al fondo
        ("qr", (0.8, 3.6 + h), DER_Y),             # tramo 4: QR en la pared del fondo
        ("izq", (3.2, 1.0 - h), IZQ_Y),            # tramo 6: flecha en el zocalo
        ("qr", (6.0 + h, 1.0), IZQ_X),             # tramo 7: QR al fondo del pasillo largo
        ("prohibido", (6.0 - h, 3.0), DER_X),      # tramo 8: caja lateral IZQUIERDA
        ("bandera", (6.1, 4.0), DER_Y),            # tramo 8: bandera en la pared del fondo
        ("cara", (5.85, 4.0), DER_Y),
    ]


def circuito(ancho=0.80):
    return pasillo_poligonal(CENTRO, ancho), objetos(ancho)
