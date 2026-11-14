import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import nav_core as nc  # noqa: E402
import profundidad as pf  # noqa: E402

P = pf.ProfParams()


def cuadro_piso():
    """Depth 640x400 (mm) de un piso plano segun la calibracion; arriba del horizonte = pared a 4 m."""
    v = np.arange(400, dtype=np.float64)[:, None] * np.ones((1, 640))
    den = (v - P.cy) / P.fx - P.floor_slope
    z = np.where(den > 0.01, P.floor_h / np.maximum(den, 0.01), 4.0)
    return np.clip(z, 0.0, 4.0) * 1000.0


def con_caja(d, z, u0, u1, v0, v1):
    d = d.copy()
    d[v0:v1, u0:u1] = z * 1000.0
    return d.astype(np.uint16)


def test_piso_vacio_no_da_obstaculos():
    r, a = pf.scan_virtual(cuadro_piso().astype(np.uint16))
    assert len(r) == 0


def test_caja_baja_al_frente_se_detecta():
    # base de silla a 0.9 m: la caja ocupa las filas que miran 0.05-0.35 m sobre el piso
    d = con_caja(cuadro_piso(), 0.9, 280, 380, 230, 330)
    r, a = pf.scan_virtual(d)
    assert len(r) >= 1
    i = int(np.argmin(r))
    assert 0.9 < r[i] < 1.1 + P.cam_x          # distancia desde el centro del robot
    assert abs(math.degrees(a[i])) < 12        # al frente


def test_objeto_a_la_izquierda_tiene_angulo_positivo():
    d = con_caja(cuadro_piso(), 1.0, 60, 140, 230, 330)   # columnas a la izquierda del centro
    r, a = pf.scan_virtual(d)
    assert len(r) >= 1 and a.mean() > 0.2


def test_objeto_alto_no_cuenta():
    # superficie de mesa muy por encima de hmax: el robot pasa por debajo, no es obstaculo
    d = con_caja(cuadro_piso(), 1.5, 280, 380, 0, 30)
    r, a = pf.scan_virtual(d)
    assert len(r) == 0


def test_zona_ciega_y_lejania_se_ignoran():
    assert len(pf.scan_virtual(con_caja(cuadro_piso(), 0.2, 280, 380, 230, 330))[0]) == 0
    assert len(pf.scan_virtual(con_caja(cuadro_piso(), 3.5, 280, 380, 230, 330))[0]) == 0


def test_costo_de_calculo_bajo():
    import time
    d = con_caja(cuadro_piso(), 0.9, 280, 380, 230, 330)
    t = time.perf_counter()
    for _ in range(20):
        pf.scan_virtual(d)
    assert (time.perf_counter() - t) / 20 < 0.02      # < 20 ms por cuadro (en la Pi ~5x la laptop)


def _scan_pasillo(izq, der, frente=3.0):
    ang = np.radians(np.arange(-180, 180, 2.0))
    r = np.full(ang.shape, 3.0)
    r[np.abs(np.degrees(ang) - 90) < 4] = izq
    r[np.abs(np.degrees(ang) + 90) < 4] = der
    r[np.abs(np.degrees(ang)) < 4] = frente
    return r, ang


def test_guardian_se_enciende_en_zona_estrecha_y_mantiene_histeresis():
    g = pf.GuardianProfundidad(hold=8.0)
    encendidos = []
    g._arrancar = lambda: encendidos.append(1)       # sin ROS
    g._detener = lambda: encendidos.append(0)
    g.mezclar(10.0, *_scan_pasillo(0.5, 0.5))        # estrecho -> ON
    g.mezclar(12.0, *_scan_pasillo(2.0, 2.0))        # ya abierto, pero dentro de hold -> sigue ON
    g.mezclar(17.0, *_scan_pasillo(2.0, 2.0))        # t=17 < 10+8: aun dentro de hold
    g.mezclar(25.0, *_scan_pasillo(2.0, 2.0))        # paso hold -> OFF
    assert encendidos == [1, 1, 1, 0]


def test_guardian_une_puntos_virtuales_al_lidar():
    g = pf.GuardianProfundidad()
    g._arrancar = g._detener = lambda: None
    import time
    now = time.monotonic()
    g.virtual = (np.array([0.8]), np.array([0.0]))
    g.t_virtual = now
    r, a = _scan_pasillo(2.0, 2.0, frente=3.0)
    r2, a2 = g.mezclar(now, r, a)
    assert len(r2) == len(r) + 1 and r2[-1] == 0.8
    g.t_virtual = now - 5.0                            # dato viejo: no se usa
    r3, _ = g.mezclar(now, r, a)
    assert len(r3) == len(r)


def test_side_scale_frena_con_poca_holgura_lateral():
    p = nc.NavParams()
    ang = np.radians(np.arange(-180, 180, 2.0))
    abierto = np.full(ang.shape, 3.0)
    assert nc.side_scale(abierto, ang, p) == 1.0
    # pared a 0.22 m a la izquierda, junto al robot
    r = abierto.copy()
    r[np.abs(np.degrees(ang) - 90) < 6] = 0.22
    assert nc.side_scale(r, ang, p) == p.side_min_scale
    # holgura intermedia -> factor intermedio
    r[np.abs(np.degrees(ang) - 90) < 6] = 0.44
    s = nc.side_scale(r, ang, p)
    assert p.side_min_scale < s < 1.0
    assert nc.side_scale(np.empty(0), np.empty(0), p) == 1.0
