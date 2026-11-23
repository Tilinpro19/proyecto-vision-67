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


def test_cero_falsos_positivos_en_texturas_pared_y_baldosas():
    rng = np.random.default_rng(7)
    gx = np.linspace(140, 200, 640, dtype=np.float32)[None, :, None]
    pared = np.clip(gx + rng.normal(0, 6.0, (480, 640, 3)), 0, 255).astype(np.uint8)
    baldosas = np.full((480, 640, 3), 210, dtype=np.uint8)
    baldosas[::70, :] = 140
    baldosas[:, ::70] = 140
    ruido = rng.integers(80, 210, size=(480, 640, 3), dtype=np.uint8)

    for im in (pared, baldosas, ruido):
        assert sf.detectar(im) == []

    pared_250 = cv2.resize(pared, (250, 187), interpolation=cv2.INTER_AREA)
    baldosas_250 = np.full((187, 250, 3), 210, dtype=np.uint8)
    baldosas_250[::35, :] = 140
    baldosas_250[:, ::35] = 140
    assert sf.buscar_candidatos(pared_250) == []
    assert sf.buscar_candidatos(baldosas_250) == []


def test_tiempo_deteccion_acotado():
    import time
    base = cv2.imread(os.path.join(DATA_SENALES, "flecha_izq.png"), cv2.IMREAD_GRAYSCALE)
    escena_640 = _escena_forma(base, dist_m=1.1, ancho_px=640, yaw_deg=0.0, blur_px=2, ruido=3.0)
    escena_250 = cv2.resize(escena_640, (250, 187), interpolation=cv2.INTER_AREA)

    t0 = time.perf_counter()
    cands = sf.buscar_candidatos(escena_250)
    dt_buscar_ms = (time.perf_counter() - t0) * 1000.0
    assert len(cands) >= 1
    assert dt_buscar_ms < 15.0

    t1 = time.perf_counter()
    res = sf.detectar(escena_640, esfuerzo_qr=1)
    dt_det_ms = (time.perf_counter() - t1) * 1000.0
    assert len(res) >= 1 and res[0].tipo == "izq"
    assert dt_det_ms < 150.0


# --- senales nuevas: flechas en circulo AZUL ("Copia de senales.docx") ---------------------

REF_AZUL = os.path.join(HERE, "senales_ref_azul")


def _hoja_color(nombre, flip=False, margen=0.12):
    im = cv2.imread(os.path.join(REF_AZUL, nombre), cv2.IMREAD_UNCHANGED)
    if im.shape[2] == 4:
        a = im[:, :, 3:4] / 255.0
        im = (im[:, :, :3] * a + 255 * (1 - a)).astype(np.uint8)
    if flip:
        im = cv2.flip(im, 1)
    p = int(margen * max(im.shape[:2]))
    return im, cv2.copyMakeBorder(im, p, p, p, p, cv2.BORDER_CONSTANT, value=(255, 255, 255))


def _luz(img, brillo, sat):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] *= sat
    hsv[:, :, 2] *= brillo
    return cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR)


@pytest.mark.parametrize("archivo", ["flecha_azul_der_a.png", "flecha_azul_der_b.png",
                                     "flecha_azul_der_vector.png", "flecha_roja_der.png"])
@pytest.mark.parametrize("flip,esperado", [(False, "der"), (True, "izq")])
@pytest.mark.parametrize("luz", [(1.0, 1.0), (0.6, 1.0), (0.6, 0.5), (1.0, 0.5)])
def test_flecha_azul_se_detecta_y_nunca_se_confunde(archivo, flip, esperado, luz):
    im, hoja = _hoja_color(archivo, flip)
    hoja = _luz(hoja, *luz)
    rng = np.random.default_rng(0)
    for d in (0.5, 0.8, 1.1, 1.4):
        for yaw in (0, 25):
            img = rs.degradar(rs.proyectar(hoja, hoja.shape[1], 0.107 * hoja.shape[1] / im.shape[1], d, 640,
                                           yaw_deg=yaw), rng=rng)
            assert sf.buscar_candidatos(img), f"sin candidato a {d} m"     # el filtro previo de mision.py
            tipos = [s.tipo for s in sf.detectar(img)]
            assert esperado in tipos, f"{archivo} {esperado} a {d} m yaw {yaw} luz {luz}: {tipos}"
            assert not [t for t in tipos if t in ("izq", "der", "prohibido") and t != esperado], tipos
            color = "rojo" if "roja" in archivo else "azul"
            assert all(x.color == color for x in sf.detectar(img) if x.tipo == esperado)


def test_prohibido_y_bandera_nuevos_se_detectan():
    for archivo, esperado in (("prohibido.png", "prohibido"), ("bandera.png", "bandera")):
        im, hoja = _hoja_color(archivo)
        img = rs.degradar(rs.proyectar(hoja, hoja.shape[1], 0.107 * hoja.shape[1] / im.shape[1], 1.0, 640),
                          rng=np.random.default_rng(0))
        assert esperado in [s.tipo for s in sf.detectar(img)]


def test_objetos_azules_sin_flecha_no_son_senales():
    # cartel azul liso, circulo azul sin flecha, cielo/cajas azules: el azul pasa a papel y no queda forma
    img = np.full((480, 640, 3), 190, np.uint8)
    cv2.rectangle(img, (40, 60), (220, 260), (192, 112, 0), -1)
    cv2.circle(img, (420, 200), 90, (192, 112, 0), -1)
    cv2.circle(img, (420, 200), 90, (0, 0, 0), 12)          # anillo negro con interior azul vacio
    img[350:] = (200, 140, 60)
    tipos = [s.tipo for s in sf.detectar(img)]
    assert not [t for t in tipos if t in ("izq", "der")], tipos


@pytest.mark.parametrize("archivo", ["circulo_azul_liso.png", "circulo_rojo_liso.png"])
def test_circulos_de_color_lisos_no_son_flechas(archivo):
    im, hoja = _hoja_color(archivo)
    rng = np.random.default_rng(0)
    for d in (0.5, 0.9, 1.3):
        img = rs.degradar(rs.proyectar(hoja, hoja.shape[1], 0.107 * hoja.shape[1] / im.shape[1], d, 640), rng=rng)
        tipos = [x.tipo for x in sf.detectar(img)]
        assert not [t for t in tipos if t in ("izq", "der", "prohibido")], (archivo, d, tipos)


def test_flecha_impresa_en_escala_de_grises_tambien_se_detecta():
    """Si la hoja se imprime en blanco y negro el azul queda gris medio (sin color que mapear)."""
    im, hoja = _hoja_color("flecha_azul_der_b.png")
    gris = cv2.cvtColor(cv2.cvtColor(hoja, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
    rng = np.random.default_rng(0)
    vistas = []
    for d in (0.5, 0.8, 1.1):
        img = rs.degradar(rs.proyectar(gris, gris.shape[1], 0.107 * gris.shape[1] / im.shape[1], d, 640), rng=rng)
        vistas.append([x.tipo for x in sf.detectar(img)])
    assert all("der" in v for v in vistas), vistas
