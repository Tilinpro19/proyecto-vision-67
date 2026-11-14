"""Pruebas de seguimiento de pasillo y deteccion de cruces con LiDAR (Fase 2)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import pasillo as ps  # noqa: E402
from sim import scan_from  # noqa: E402


def _mapa_pasillo_recto():
    return [((-1.0, 0.55), (4.0, 0.55)), ((-1.0, -0.55), (4.0, -0.55))]


def _mapa_esquina_L_izq():
    # Pasillo de x=-1..2 (ancho 1.1 m), bloqueado en x=2.55 y abierto hacia +y (izquierda)
    return [
        ((-1.0, -0.55), (2.55, -0.55)),
        ((2.55, -0.55), (2.55, 3.5)),
        ((-1.0, 0.55), (1.45, 0.55)),
        ((1.45, 0.55), (1.45, 3.5)),
    ]


def _mapa_esquina_L_der():
    # Simetrico hacia -y (derecha)
    return [((x1, -y1), (x2, -y2)) for (x1, y1), (x2, y2) in _mapa_esquina_L_izq()]


def _mapa_cruce_T():
    # Pasillo bloqueado al frente en x=2.55, abierto a +y (izq) y -y (der)
    return [
        ((-1.0, 0.55), (1.45, 0.55)),
        ((1.45, 0.55), (1.45, 3.5)),
        ((-1.0, -0.55), (1.45, -0.55)),
        ((1.45, -0.55), (1.45, -3.5)),
        ((2.55, -3.5), (2.55, 3.5)),
    ]


def _mapa_cruce_cruz():
    # Abierto en las 4 direcciones alrededor de x=2.0, y=0.0
    return [
        ((-1.0, 0.55), (1.45, 0.55)),
        ((1.45, 0.55), (1.45, 3.5)),
        ((2.55, 0.55), (2.55, 3.5)),
        ((2.55, 0.55), (5.0, 0.55)),
        ((-1.0, -0.55), (1.45, -0.55)),
        ((1.45, -0.55), (1.45, -3.5)),
        ((2.55, -0.55), (2.55, -3.5)),
        ((2.55, -0.55), (5.0, -0.55)),
    ]


def _mapa_callejon():
    return [
        ((-1.0, 0.55), (2.0, 0.55)),
        ((-1.0, -0.55), (2.0, -0.55)),
        ((2.0, -0.55), (2.0, 0.55)),
    ]


def test_pasillo_recto_solo_opcion_recto_y_corrige_centro():
    walls = _mapa_pasillo_recto()
    r, a = scan_from(walls, (0.0, 0.0, 0.0))
    pto = ps.analizar_pasillo(r, a)
    assert pto.opciones == {"recto"}
    assert not pto.en_cruce

    # Si el robot esta pegado a la pared derecha (y=-0.20), debe corregir a la izquierda (ang > 0)
    r_off, a_off = scan_from(walls, (0.0, -0.20, 0.0))
    lin, ang, pto_off = ps.comando_pasillo(r_off, a_off)
    assert lin > 0.05
    assert pto_off.error_centro > 0.20
    assert ang > 0.05


def test_detecta_esquina_L_izquierda_y_derecha():
    r_izq, a_izq = scan_from(_mapa_esquina_L_izq(), (2.0, 0.0, 0.0))
    pto_izq = ps.analizar_pasillo(r_izq, a_izq)
    assert pto_izq.en_cruce
    assert pto_izq.opciones == {"izq"}

    r_der, a_der = scan_from(_mapa_esquina_L_der(), (2.0, 0.0, 0.0))
    pto_der = ps.analizar_pasillo(r_der, a_der)
    assert pto_der.en_cruce
    assert pto_der.opciones == {"der"}


def test_detecta_cruce_T_y_cruz():
    r_t, a_t = scan_from(_mapa_cruce_T(), (2.0, 0.0, 0.0))
    pto_t = ps.analizar_pasillo(r_t, a_t)
    assert pto_t.en_cruce
    assert pto_t.opciones == {"izq", "der"}

    r_c, a_c = scan_from(_mapa_cruce_cruz(), (2.0, 0.0, 0.0))
    pto_c = ps.analizar_pasillo(r_c, a_c)
    assert pto_c.en_cruce
    assert pto_c.opciones == {"izq", "recto", "der"}


def test_detecta_callejon_sin_salida():
    r, a = scan_from(_mapa_callejon(), (1.45, 0.0, 0.0))
    pto = ps.analizar_pasillo(r, a)
    assert pto.en_cruce
    assert pto.opciones == set()
