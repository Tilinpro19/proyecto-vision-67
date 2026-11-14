"""Senales del profe (QR + color) sin camara: escenas sinteticas con distancia, resolucion,
angulo, tinta/marco de color, desenfoque de movimiento, ruido y JPEG.

Correr desde 1_competencia/:  python3 -m pytest tests/test_senales.py -v
Tambien con el OpenCV del sistema:  python3 -I -m pytest tests/test_senales.py -v
"""
import os

import cv2
import numpy as np
import pytest

import render_senales as rs
import senales as sg
from senales import AZUL, ROJO

DATOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def textos(img, esfuerzo=2):
    return [t for t, _ in sg.decodificar_qr(img, esfuerzo)]


# --- el template tal cual -------------------------------------------------------------

@pytest.mark.parametrize("nombre", rs.NOMBRES)
def test_template_sin_borde_blanco_se_lee(nombre):
    """El PNG del profe no trae zona silenciosa: senales.py se la agrega."""
    img = cv2.imread(os.path.join(rs.DATA, nombre + ".png"))
    assert textos(img) == [nombre]


def test_bandera_no_es_qr_ni_falso_positivo():
    bandera = cv2.imread(os.path.join(rs.DATA, "bandera_meta.png"))
    assert sg.leer(bandera) == []
    for f in ("lena.jpg", "messi5.jpg"):
        img = cv2.imread(os.path.join(DATOS, f))
        assert sg.leer(img, esfuerzo=3) == [], f


# --- lectura, color y distancia ---------------------------------------------------------

@pytest.mark.parametrize("nombre", rs.NOMBRES)
def test_lee_a_1m_y_estima_distancia(nombre):
    s = sg.leer(rs.escena(nombre, 1.0, 1280))
    assert [x.texto for x in s] == [nombre]
    assert s[0].distancia_m == pytest.approx(1.0, rel=0.15)


@pytest.mark.parametrize("tinta,esperado", [(rs.NEGRO, None), (rs.AZUL, AZUL), (rs.ROJO, ROJO)])
def test_color_en_la_tinta(tinta, esperado):
    s = sg.leer(rs.escena("milestone_alfa", 1.0, 1280, tinta=tinta))
    assert len(s) == 1 and s[0].texto == "milestone_alfa"
    assert s[0].color == esperado
    assert s[0].accion == ({AZUL: "avanzar", ROJO: "regresar"}.get(esperado))


@pytest.mark.parametrize("marco,esperado", [(rs.AZUL, AZUL), (rs.ROJO, ROJO)])
def test_color_en_el_marco(marco, esperado):
    s = sg.leer(rs.escena("milestone_beta", 1.0, 1280, marco=marco))
    assert len(s) == 1 and s[0].color == esperado and s[0].modo_color == "marco"


def test_rojo_da_la_vuelta_en_el_matiz():
    hsv = np.zeros((1, 4, 3), np.uint8)
    hsv[..., 1:] = 255
    hsv[0, :, 0] = [2, 176, 120, 60]                       # rojo, rojo, azul, verde
    r, a = sg.mascara_color(hsv, ROJO), sg.mascara_color(hsv, AZUL)
    assert r.tolist() == [[True, True, False, False]]
    assert a.tolist() == [[False, False, True, False]]


def test_no_depende_de_los_seis_nombres():
    """El profe permite agregar senales: cualquier QR del mismo tamano debe leerse."""
    for texto in ("milestone_zeta", "hito_42", "avanzar"):
        img = rs.escena(None, 1.0, 1280, tinta=rs.AZUL, qr_gris=rs.qr_nuevo(texto))
        assert textos(img) == [texto]


# --- condiciones reales ----------------------------------------------------------------

def test_jpeg_movimiento_y_angulo_a_1m():
    """JPEG 60 + 3 px de desenfoque de movimiento + 30 grados de angulo, QR de 10 cm a 1 m."""
    ok = sum(
        textos(rs.escena(n, 1.0, 1280, yaw_deg=30, blur_px=3, jpeg=60, seed=i)) == [n]
        for i, n in enumerate(rs.NOMBRES))
    assert ok >= 5, f"solo {ok}/6"


def test_el_preview_250px_no_alcanza_para_leer_pero_si_para_ver_la_mancha():
    """Por eso hay dos etapas: en el preview solo se ve DONDE hay una senal."""
    for tinta, color in ((rs.AZUL, AZUL), (rs.ROJO, ROJO)):
        img = rs.escena("milestone_alfa", 1.5, 250, tinta=tinta)
        assert sg.leer(img) == []                              # no se puede decodificar
        manchas = sg.buscar_manchas(img)
        assert manchas and manchas[0][0] == color, manchas
    assert sg.buscar_manchas(rs.escena("milestone_alfa", 1.5, 250, tinta=rs.NEGRO)) == []


def test_mancha_apunta_a_la_senal():
    """La mancha del preview cae sobre donde esta la senal (desplazada a la derecha)."""
    img = rs.escena("milestone_alfa", 1.5, 250, tinta=rs.ROJO, desplaza=(0.4, 0.0))
    color, (x, y, w, h), _ = sg.buscar_manchas(img)[0]
    assert x + w / 2 > img.shape[1] / 2 + 20


@pytest.mark.parametrize("ancho,dist", [(1280, 1.5), (1920, 2.0)])
def test_alcance_con_resolucion_alta(ancho, dist):
    n = rs.NOMBRES[0]
    assert textos(rs.escena(n, dist, ancho, jpeg=75)) == [n]


@pytest.mark.parametrize("color,tinta", [(AZUL, rs.AZUL), (ROJO, rs.ROJO)])
def test_mancha_y_lectura_en_roi(color, tinta):
    """Flujo de dos etapas: la mancha da el recorte (ROI) y solo ahi se decodifica."""
    img = rs.escena("milestone_gamma", 1.8, 1920, tinta=tinta, jpeg=75, desplaza=(0.3, 0.1))
    c, bbox, _ = sg.buscar_manchas(img)[0]
    assert c == color
    s = sg.leer(img, roi=bbox)
    assert len(s) == 1 and s[0].texto == "milestone_gamma" and s[0].color == color
    cx, cy = s[0].esquinas.mean(axis=0)                  # coordenadas de la imagen completa
    x, y, w, h = bbox
    assert x <= cx <= x + w and y <= cy <= y + h
    assert s[0].distancia_m == pytest.approx(1.8, rel=0.15)


def test_vacio_no_falla():
    assert sg.leer(np.full((480, 640, 3), 200, np.uint8)) == []
    assert sg.buscar_manchas(np.full((480, 640, 3), 200, np.uint8)) == []
    assert sg.decodificar_qr(np.zeros((40, 40), np.uint8)) == []
