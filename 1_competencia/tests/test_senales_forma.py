"""Pruebas del detector de senales en blanco y negro por forma (Fase 1)."""
import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import render_senales as rs  # noqa: E402
import senales_forma as sf  # noqa: E402

REF_DIR = os.path.join(HERE, "senales_ref")
DATA_SENALES = os.path.join(HERE, "data", "senales")
DATA_FACES = os.path.join(HERE, "data")


def _escena_forma(img_gris, dist_m=1.2, ancho_px=640, yaw_deg=0.0, blur_px=2, ruido=3.0, seed=0):
    hoja = rs.hoja(img_gris, tinta=(0, 0, 0), margen=0.12)
    vista = rs.proyectar(hoja, img_gris.shape[1], 0.10, dist_m, ancho_px, yaw_deg=yaw_deg)
    return rs.degradar(vista, blur_px=blur_px, jpeg=65, ruido=ruido, rng=np.random.default_rng(seed))


def test_hoja_contacto_detecta_todos_los_tipos():
    img = cv2.imread(os.path.join(REF_DIR, "hoja_contacto.png"))
    res = sf.detectar(img)
    por_tipo = {}
    for s in res:
        por_tipo.setdefault(s.tipo, []).append(s)

    assert len(por_tipo.get("izq", [])) == 4
    assert len(por_tipo.get("der", [])) == 4
    assert len(por_tipo.get("prohibido", [])) == 2
    assert len(por_tipo.get("bandera", [])) == 1
    qrs = {s.texto for s in por_tipo.get("qr", [])}
    assert qrs == {
        "milestone_alfa", "milestone_beta", "milestone_gamma",
        "milestone_omega", "milestone_phi", "milestone_rho",
    }


@pytest.mark.parametrize("tipo,archivo", [
    ("izq", "flecha_izq.png"),
    ("der", "flecha_der.png"),
    ("prohibido", "prohibido.png"),
    ("bandera", "bandera_meta.png"),
])
@pytest.mark.parametrize("yaw", [-28.0, 0.0, 28.0])
def test_formas_proyectadas_con_perspectiva_y_ruido_nunca_se_confunden(tipo, archivo, yaw):
    base = cv2.imread(os.path.join(DATA_SENALES, archivo), cv2.IMREAD_GRAYSCALE)
    escena = _escena_forma(base, dist_m=1.1, ancho_px=640, yaw_deg=yaw, blur_px=3, ruido=4.0)
    res = sf.detectar(escena)
    assert len(res) >= 1
    assert res[0].tipo == tipo
    assert res[0].confianza >= 0.45
    assert res[0].texto is None


@pytest.mark.parametrize("nombre", rs.NOMBRES)
def test_qr_individuales_en_senales_ref(nombre):
    img = cv2.imread(os.path.join(REF_DIR, f"qr_{nombre}.png"))
    res = sf.detectar(img)
    qrs = [s for s in res if s.tipo == "qr"]
    assert len(qrs) == 1
    assert qrs[0].texto == nombre


def test_bandera_horizontal_y_vertical():
    img = cv2.imread(os.path.join(REF_DIR, "bandera_cuadros.png"))
    for rot in (img, cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)):
        res = sf.detectar(rot)
        assert any(s.tipo == "bandera" for s in res)


def test_cero_falsos_positivos_en_fondos_y_rostros():
    rng = np.random.default_rng(42)
    negativos = [
        np.full((480, 640, 3), 185, dtype=np.uint8),
        rng.integers(90, 200, size=(360, 480, 3), dtype=np.uint8),
        cv2.imread(os.path.join(DATA_FACES, "lena.jpg")),
        cv2.imread(os.path.join(DATA_FACES, "messi5.jpg")),
    ]
    # Circulo vacio y disco negro solido tampoco deben disparar falsos positivos
    aro_vacio = np.full((300, 300, 3), 230, dtype=np.uint8)
    cv2.circle(aro_vacio, (150, 150), 80, (10, 10, 10), 16)
    disco_lleno = np.full((300, 300, 3), 230, dtype=np.uint8)
    cv2.circle(disco_lleno, (150, 150), 80, (10, 10, 10), -1)
    negativos.extend([aro_vacio, disco_lleno])

    for img in negativos:
        assert sf.detectar(img) == []


@pytest.mark.parametrize("tipo,archivo", [
    ("izq", "flecha_izq.png"),
    ("der", "flecha_der.png"),
    ("prohibido", "prohibido.png"),
    ("bandera", "bandera_meta.png"),
])
@pytest.mark.parametrize("condicion", ["sobreexposicion", "poca_luz", "sombra_lateral", "inclinacion_10"])
def test_condiciones_reales_camara_y_preview_250px(tipo, archivo, condicion):
    base = cv2.imread(os.path.join(DATA_SENALES, archivo), cv2.IMREAD_GRAYSCALE)
    escena = _escena_forma(base, dist_m=1.1, ancho_px=640, yaw_deg=0.0, blur_px=2, ruido=3.0)
    H, W = escena.shape[:2]
    if condicion == "sobreexposicion":
        im = np.clip(escena.astype(np.float32) * 0.45 + 140.0, 0, 255).astype(np.uint8)
    elif condicion == "poca_luz":
        im = np.clip(escena.astype(np.float32) * 0.45 + 10.0, 0, 255).astype(np.uint8)
    elif condicion == "sombra_lateral":
        grad = np.linspace(0.35, 1.0, W, dtype=np.float32)[None, :, None]
        im = np.clip(escena.astype(np.float32) * grad, 0, 255).astype(np.uint8)
    else:
        M = cv2.getRotationMatrix2D((W / 2, H / 2), 10.0, 1.0)
        im = cv2.warpAffine(escena, M, (W, H), borderValue=(180, 180, 180))

    preview = cv2.resize(im, (250, int(250 * H / W)), interpolation=cv2.INTER_AREA)
    assert len(sf.buscar_candidatos(preview)) >= 1, f"preview 250px no detecto candidato en {tipo}/{condicion}"

    res = sf.detectar(im)
    assert len(res) >= 1 and res[0].tipo == tipo, f"fallo {tipo}/{condicion}: {[s.tipo for s in res]}"


def test_cero_falsos_positivos_en_patrones_regulares():
    # Rayas verticales y horizontales (rejas/persianas) no deben confundirse con bandera ni circulos
    rayas_v = np.full((360, 480, 3), 220, dtype=np.uint8)
    for x in range(100, 380, 24):
        rayas_v[:, x:x + 10] = 25
    rayas_h = np.full((360, 480, 3), 220, dtype=np.uint8)
    for y in range(60, 300, 24):
        rayas_h[y:y + 10, :] = 25
    for im in (rayas_v, rayas_h):
        assert sf.detectar(im) == []
