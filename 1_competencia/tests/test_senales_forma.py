"""Tests del detector por forma (senales_forma.py). Escenas sinteticas con las senales del profe
(tests/senales_ref/hoja_contacto.png): escala, rotacion pequena, desenfoque, ruido y JPEG.
  python3 tests/test_senales_forma.py      # imprime el alcance (diametro minimo en px) por senal
"""
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import senales_forma as sf

HOJA = cv2.imread(sf.REF)
# recortes (x, y, w, h) de la hoja original (2895x1638)
RECORTES = {"izq": (142, 66, 293, 286), "der": (142, 398, 293, 286),
            "prohibido": (712, 69, 305, 305), "bandera": (100, 890, 400, 700)}


def recorte(tipo):
    x, y, w, h = RECORTES[tipo]
    return HOJA[y:y + h, x:x + w].copy()


def escena(tipo, ancho_px, rot=0.0, blur=0, ruido=0, jpeg=0, shape=(480, 640), pos=(0.5, 0.5), seed=0):
    """Fondo gris claro con la senal de `ancho_px` de ancho; devuelve la imagen BGR."""
    rng = np.random.default_rng(seed)
    s = recorte(tipo)
    esc = ancho_px / s.shape[1]
    s = cv2.resize(s, None, fx=esc, fy=esc, interpolation=cv2.INTER_AREA)
    img = np.full(shape + (3,), 215, np.uint8)
    h, w = s.shape[:2]
    x, y = int((shape[1] - w) * pos[0]), int((shape[0] - h) * pos[1])
    img[y:y + h, x:x + w] = s
    if rot:
        M = cv2.getRotationMatrix2D((x + w / 2, y + h / 2), rot, 1.0)
        img = cv2.warpAffine(img, M, (shape[1], shape[0]), borderValue=(215, 215, 215))
    if blur:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    if ruido:
        img = np.clip(img + rng.normal(0, ruido, img.shape), 0, 255).astype(np.uint8)
    if jpeg:
        img = cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, jpeg])[1], 1)
    return img


def tipos(img):
    return [s.tipo for s in sf.detectar(img, con_qr=False)]


@pytest.mark.parametrize("tipo", ["izq", "der", "prohibido"])
def test_circulos_limpios(tipo):
    assert tipos(escena(tipo, 200)) == [tipo]


def test_bandera_limpia():
    assert tipos(escena("bandera", 200)) == ["bandera"]


@pytest.mark.parametrize("tipo", ["izq", "der", "prohibido"])
@pytest.mark.parametrize("rot", [-8, 8])
def test_rotacion_pequena(tipo, rot):
    assert tipos(escena(tipo, 160, rot=rot)) == [tipo]


@pytest.mark.parametrize("tipo", ["izq", "der", "prohibido", "bandera"])
def test_degradada(tipo):
    img = escena(tipo, 140, blur=1.2, ruido=8, jpeg=60, seed=3)
    assert tipos(img) == [tipo]


def test_izq_der_nunca_se_confunden():
    for tipo, otro in (("izq", "der"), ("der", "izq")):
        for ancho in (60, 90, 140, 220):
            for rot in (-6, 0, 6):
                assert otro not in tipos(escena(tipo, ancho, rot=rot, blur=0.8, ruido=6, jpeg=70))


def test_sin_falsos_positivos_en_fondos():
    rng = np.random.default_rng(1)
    fondos = [np.full((480, 640, 3), 128, np.uint8),
              rng.integers(0, 255, (480, 640, 3), dtype=np.uint8),
              cv2.GaussianBlur(rng.integers(0, 255, (480, 640, 3), dtype=np.uint8), (0, 0), 6)]
    for f in fondos:
        assert tipos(f) == []
    # un QR no es circulo ni bandera
    qr = cv2.imread(os.path.join(sf.AQUI, "tests", "senales_ref", "qr_milestone_alfa.png"))
    lienzo = np.full((480, 640, 3), 255, np.uint8)
    q = cv2.resize(qr, (200, 200), interpolation=cv2.INTER_AREA)
    lienzo[100:300, 200:400] = q
    assert tipos(lienzo) == []


def test_qr_por_texto():
    qr = cv2.imread(os.path.join(sf.AQUI, "tests", "senales_ref", "qr_milestone_beta.png"))
    lienzo = cv2.copyMakeBorder(cv2.resize(qr, (260, 260)), 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=(255,) * 3)
    r = [s for s in sf.detectar(lienzo) if s.tipo == "qr"]
    assert r and r[0].texto == "milestone_beta"


def alcance():
    print("ancho minimo (px de la senal en una imagen de 640 px) con degradacion moderada")
    for tipo in ("izq", "der", "prohibido", "bandera"):
        ok = [a for a in range(200, 20, -4)
              if all(tipos(escena(tipo, a, blur=0.8, ruido=6, jpeg=70, seed=s)) == [tipo] for s in range(3))]
        print(f"  {tipo:10s} {min(ok) if ok else '-'} px")


if __name__ == "__main__":
    alcance()
