"""Escaneos reales del robot (logs/*.npz de los intentos en pista) contra el modo directo.
Cada npz guarda ranges/angles (marco del robot) y la pose de odometria del momento."""
import glob
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import directo as dr  # noqa: E402
import mision as ms  # noqa: E402
import nav_core as nc  # noqa: E402
from comun import apply_params  # noqa: E402
from navegacion import Piloto  # noqa: E402

DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scans_reales")
PASILLOS = ["pasillo_cajas.npz", "r122736_pasillo.npz", "r124100_pasillo.npz"]
CHOQUES = sorted(os.path.basename(f) for f in glob.glob(os.path.join(DIR, "choque_*.npz")))


def cargar(nombre):
    z = np.load(os.path.join(DIR, nombre))
    pose = tuple(float(v) for v in z["pose"]) if "pose" in z.files else (0.0, 0.0, 0.0)
    return pose, z["ranges"].astype(float), z["angles"].astype(float)


def directo(pose, rumbo=None):
    """PilotoDirecto con config.json; A en el origen de odometria (como en la pista)."""
    piloto, mp, dp = Piloto(), ms.MisionParams(), dr.DirectoParams()
    apply_params([piloto.nav, piloto.pp, mp, dp], os.path.join(ROOT, "config.json"))
    piloto.origin = (0.0, 0.0, 0.0)
    d = dr.PilotoDirecto(piloto, mp, dp)
    d.rumbo = dr.PilotoDirecto._cardinal_rad(pose[2]) if rumbo is None else rumbo
    return d


def limpio(d, ranges, angles):
    return nc.clean_scan(ranges, angles, d.piloto.nav.scan_min)


def test_hay_escaneos_reales():
    assert len(CHOQUES) >= 4
    for n in PASILLOS + ["r144106_borde_salida.npz"]:
        assert os.path.exists(os.path.join(DIR, n)), n


def test_pared_en_rechaza_ventana_en_el_borde_de_un_hueco():
    a = np.radians(np.arange(80.0, 101.0, 0.5))
    r = np.where(a < math.radians(90.0), 0.28, 1.85)
    assert dr.pared_en(r, a, math.radians(90.0), math.radians(5.0)) is None
    assert dr.pared_en(np.full_like(a, 0.30), a, math.radians(90.0), math.radians(5.0)) == pytest.approx(0.30)


@pytest.mark.parametrize("nombre", PASILLOS)
def test_ancho_de_los_pasillos_reales(nombre):
    pose, rr, aa = cargar(nombre)
    d = directo(pose)
    r, a = limpio(d, rr, aa)
    d._medir_ancho(d.piloto.pose(pose), r, a)
    assert d._ancho_n == 1
    assert 0.65 < d.ancho < 0.85


def test_borde_de_salida_no_infla_el_ancho():
    """Intento real del 19/11: en x=0.49 la ventana de +90 cae justo en el borde de la salida
    (0.28 m de pared y 1.8 m de hueco). Con la mediana el ancho daba 1.33 m (pasillo de 0.57)
    y los umbrales de giro volvian a 0.70 m: el mismo fallo que el de ese intento."""
    pose, rr, aa = cargar("r144106_borde_salida.npz")
    d = directo(pose)
    r, a = limpio(d, rr, aa)
    d._medir_ancho(d.piloto.pose(pose), r, a)
    assert d._ancho_n == 0 and d.ancho == d.dp.ancho_inicial     # lectura descartada
    d.ancho, d._ancho_n = 0.57, 5
    d._medir_ancho(d.piloto.pose(pose), r, a)
    assert d.ancho == pytest.approx(0.57)


@pytest.mark.parametrize("nombre", PASILLOS)
def test_pasillo_real_libre_avanza_derecho(nombre):
    pose, rr, aa = cargar(nombre)
    d = directo(pose)
    r, a = limpio(d, rr, aa)
    p = d.piloto.pose(pose)
    front = dr.rango_en(r, a, nc.wrap(d.rumbo - p[2]), math.radians(8.0))
    d._lin_prev = d.dp.v_recto
    lin, ang, bloqueado = d._seguir_pasillo(0.0, p, r, a, front, d.dp.v_recto)
    assert lin > 0.15 and not bloqueado
    assert abs(ang) < 0.5


@pytest.mark.parametrize("nombre", CHOQUES)
def test_casi_choques_reales_frenan(nombre):
    """Los 4 'choque_LiDAR' de los intentos (algo a 0.16-0.20 m, al frente-diagonal): el
    seguidor de pasillo no debe avanzar hacia ahi."""
    pose, rr, aa = cargar(nombre)
    d = directo(pose, rumbo=pose[2])
    r, a = limpio(d, rr, aa)
    p = d.piloto.pose(pose)
    front = dr.rango_en(r, a, 0.0, math.radians(8.0)) or d.piloto.nav.max_range
    d._lin_prev = d.dp.v_recto
    lin, _, bloqueado = d._seguir_pasillo(0.0, p, r, a, front, d.dp.v_recto)
    assert lin == 0.0 and bloqueado


def test_borde_de_salida_no_desvia_el_centrado():
    """Mismo escaneo: la mediana de +-10 grados en +90 daba ~1.0 m (pared + hueco) y el
    centrado giraba ~20 grados hacia el hueco, rozando el borde del carton."""
    pose, rr, aa = cargar("r144106_borde_salida.npz")
    d = directo(pose)
    d.ancho, d._ancho_n = 0.57, 5
    r, a = limpio(d, rr, aa)
    p = d.piloto.pose(pose)
    d.rumbo = p[2]          # alineado: solo cuenta el centrado
    _, ang, _ = d._seguir_pasillo(0.0, p, r, a, 0.81, d.dp.v_recto)
    assert abs(ang) < 0.3
