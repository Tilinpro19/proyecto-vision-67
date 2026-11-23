"""Camara simulada para las pruebas: senales y caras puestas en el mundo (posicion + normal
de la cara de la pared), vistas con un modelo pinhole: FOV, alcance, angulo de vision y
linea de vista contra las paredes. Devuelve lo mismo que senales_forma.detectar()."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import nav_core as nc  # noqa: E402
from senales_forma import Senal  # noqa: E402

ANCHO, FOV = 640, 69.0


def _cruza(p, q, s):
    """True si el segmento p-q corta el segmento s (sin contar extremos)."""
    (x1, y1), (x2, y2) = s
    d = (q[0] - p[0]) * (y2 - y1) - (q[1] - p[1]) * (x2 - x1)
    if abs(d) < 1e-12:
        return False
    t = ((x1 - p[0]) * (y2 - y1) - (y1 - p[1]) * (x2 - x1)) / d
    u = ((x1 - p[0]) * (q[1] - p[1]) - (y1 - p[1]) * (q[0] - p[0])) / d
    return 1e-6 < t < 1 - 1e-6 and 0.0 <= u <= 1.0


def ver(pose, objetos, paredes, alcance=2.6, lado=0.12, cara_ancho=0.15, ang_max_deg=70.0,
        ancho=None, fov=None, perdida=0.0, rng=None):
    """objetos: [(tipo, (x, y), (nx, ny))], tipo en senales_forma.TIPOS o "cara".
    alcance: m, o dict {tipo: m} (alcance medido del detector por tipo). ancho/fov: imagen
    (por defecto 640 px / 69 grados; el preview del robot es 250 px / ~37). perdida: probabilidad
    de que el detector no vea una senal en un cuadro (rng = numpy Generator).
    Devuelve (senales ordenadas por area, fraccion de ancho de la cara mas grande)."""
    ANCHO_, FOV_ = ancho or ANCHO, fov or FOV
    f = (ANCHO_ / 2.0) / math.tan(math.radians(FOV_) / 2.0)
    senales, cara = [], 0.0
    for tipo, xy, n in objetos:
        rel = nc.relative(pose, (xy[0], xy[1], 0.0))
        d = math.hypot(rel[0], rel[1])
        b = math.atan2(rel[1], rel[0])
        lim = alcance.get(tipo, 2.6) if isinstance(alcance, dict) else alcance
        if d > lim or d < 0.05 or abs(b) > math.radians(FOV_) / 2.0:
            continue
        if perdida and tipo != "cara" and rng is not None and rng.random() < perdida:
            continue
        hacia = ((pose[0] - xy[0]) / d, (pose[1] - xy[1]) / d)
        if hacia[0] * n[0] + hacia[1] * n[1] < math.cos(math.radians(ang_max_deg)):
            continue
        # se corta 3 cm antes para no chocar con la pared donde esta pegada
        q = (xy[0] + 0.03 * hacia[0], xy[1] + 0.03 * hacia[1])
        if any(_cruza(pose[:2], q, s) for s in paredes):
            continue
        cx = ANCHO_ / 2.0 - f * math.tan(b)
        if tipo == "cara":
            cara = max(cara, cara_ancho * f / d / ANCHO_)
            continue
        px = max(4, int(lado * f / d))
        senales.append(Senal(tipo, 0.9, (int(cx - px / 2), 200, px, px),
                             texto=f"hito_{xy[0]:.1f}_{xy[1]:.1f}" if tipo == "qr" else None,
                             distancia_m=d))
    senales.sort(key=lambda s: -(s.bbox[2] * s.bbox[3]))
    return senales, cara
