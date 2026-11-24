"""--v-rostro: la V (armado) dispara reconocimiento de rostro en gestos_teleop.py.
Usa los modelos reales de facial.py y fotos de tests/data. Todo se escribe en tmp_path."""
import argparse
import glob
import json
import os
import shutil
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

import accion_rostro as ar  # noqa: E402
import facial  # noqa: E402
import gestos_teleop as gt  # noqa: E402

LENA = cv2.imread(os.path.join(HERE, "tests", "data", "lena.jpg"))
MESSI = cv2.imread(os.path.join(HERE, "tests", "data", "messi5.jpg"))
NEGRO = np.zeros((480, 640, 3), np.uint8)


class AnunciadorFalso:
    def __init__(self):
        self.dichos = []

    def anunciar(self, nombre):
        self.dichos.append(nombre)

    def cerrar(self):
        pass


@pytest.fixture(scope="module")
def rostros():
    r = facial.Rostros()
    r.calentar()
    return r


def accion(rostros, tmp_path, galeria_fotos=(), anunciador=None, fuente=ar.FUENTE_ROBOT):
    gdir = tmp_path / "galeria"
    gdir.mkdir(exist_ok=True)
    for nombre, img in galeria_fotos:
        (gdir / nombre).mkdir(exist_ok=True)
        cv2.imwrite(str(gdir / nombre / "a.jpg"), img)
    gal = facial.Galeria(rostros, str(gdir))
    return ar.AccionRostro(rostros, gal, anunciador, fuente,
                           dir_resultados=str(tmp_path / "resultados"),
                           dir_logs=str(tmp_path / "logs"))


def correr(a, imgs, t0=10.0, dt=1 / 15):
    """iniciar + frames hasta terminar; devuelve los comandos (lin, ang) durante la accion."""
    a.iniciar(t0)
    t, cmds = t0, []
    for img in imgs:
        t += dt
        a.paso(t, img)
        cmds.append(a.limitar(0.15, 0.8))
        if not a.activa():
            break
    while a.activa():                 # sin mas frames: vence el limite (5 s)
        t += dt
        a.paso(t, None)
        cmds.append(a.limitar(0.15, 0.8))
    return cmds


def jsons(tmp_path):
    return [json.load(open(p)) for p in sorted(glob.glob(str(tmp_path / "resultados" / "*.json")))]


def test_reconoce_a_la_persona_de_la_galeria(rostros, tmp_path):
    anun = AnunciadorFalso()
    a = accion(rostros, tmp_path, [("Lena", LENA)], anun)
    cmds = correr(a, [LENA] * 20)
    assert a.estado == "reconocido" and "Lena" in a.mensaje and "SEGURO" in a.mensaje
    assert anun.dichos == ["Lena"]
    d = jsons(tmp_path)[0]
    assert d["nombre"] == "Lena" and d["seguro"] and d["score"] >= facial.UMBRAL
    assert d["estado"] == "reconocido" and d["hora"] and d["publicado"]
    assert os.path.isfile(os.path.join(ar.HERE, d["foto"])) or glob.glob(str(tmp_path / "logs" / "*.jpg"))
    assert set(cmds[:-1]) == {(0.0, 0.0)}           # quieto mientras busca


def test_persona_distinta_queda_dudosa(rostros, tmp_path):
    anun = AnunciadorFalso()
    a = accion(rostros, tmp_path, [("Lena", LENA)], anun)
    correr(a, [MESSI] * 20)
    assert a.estado == "dudoso" and "DUDOSO" in a.mensaje
    assert jsons(tmp_path)[0]["seguro"] is False


def test_sin_cara_no_anuncia_ningun_nombre(rostros, tmp_path):
    anun = AnunciadorFalso()
    a = accion(rostros, tmp_path, [("Lena", LENA)], anun)
    cmds = correr(a, [NEGRO] * 30)
    assert a.estado == "sin_cara" and a.mensaje.startswith("sin cara")
    assert anun.dichos == []                        # NO el primero de la galeria
    d = jsons(tmp_path)[0]
    assert d["nombre"] is None and d["estado"] == "sin_cara" and d["foto"] is None
    assert not glob.glob(str(tmp_path / "logs" / "*.jpg"))
    assert set(cmds[:-1]) == {(0.0, 0.0)}


def test_galeria_vacia_lo_dice(rostros, tmp_path):
    a = accion(rostros, tmp_path, [], AnunciadorFalso())
    correr(a, [LENA] * 20)
    assert a.estado == "galeria_vacia" and "galeria esta vacia" in a.mensaje
    assert glob.glob(str(tmp_path / "logs" / "*.jpg"))


def test_en_dry_no_se_publica(rostros, tmp_path):
    a = accion(rostros, tmp_path, [("Lena", LENA)], anunciador=None)   # --dry: sin nodo
    correr(a, [LENA] * 20)
    assert a.estado == "reconocido" and jsons(tmp_path)[0]["publicado"] is False


def test_modelo_faltante_no_tumba_nada(tmp_path, monkeypatch):
    monkeypatch.setattr(facial, "YUNET", str(tmp_path / "no_existe.onnx"))
    a = ar.AccionRostro.crear(fuente=ar.FUENTE_PRUEBA, dir_resultados=str(tmp_path / "r"),
                              dir_logs=str(tmp_path / "l"))
    a.iniciar(1.0)
    a.paso(1.1, LENA)
    assert a.estado == "error" and "no disponible" in a.mensaje and not a.activa()
    assert a.limitar(0.15, 0.8) == (0.15, 0.8)       # no bloquea ni mueve por su cuenta


def test_sin_camara_para_rostros(rostros, tmp_path):
    a = accion(rostros, tmp_path, fuente=None)
    a.iniciar(1.0)
    assert a.estado == "error" and "sin camara" in a.mensaje and not a.activa()


def test_error_del_detector_queda_como_mensaje(rostros, tmp_path):
    a = accion(rostros, tmp_path, [("Lena", LENA)])

    class Roto:
        def detectar(self, img):
            raise RuntimeError("fallo simulado")
    a.rostros = Roto()
    a.iniciar(1.0)
    a.paso(1.1, LENA)
    assert a.estado == "error" and "fallo simulado" in a.mensaje and not a.activa()


def test_no_se_reinicia_mientras_busca(rostros, tmp_path):
    a = accion(rostros, tmp_path, [("Lena", LENA)])
    a.iniciar(1.0)
    esc = a.escaneo
    a.iniciar(1.5)
    assert a.escaneo is esc


def test_texto_de_modo_prueba(rostros, tmp_path):
    a = accion(rostros, tmp_path, fuente=ar.FUENTE_PRUEBA)
    assert ar.TEXTO_PRUEBA in a.lineas(0.0)
    f = NEGRO.copy()
    a.dibujar(f, 0.0)
    assert f.any()
    assert ar.TEXTO_PRUEBA not in accion(rostros, tmp_path).lineas(0.0)


def test_fuente_segun_opciones():
    assert ar.fuente_rostro(dry=False, no_robot_cam=False) == ar.FUENTE_ROBOT
    assert ar.fuente_rostro(dry=True, no_robot_cam=False) == ar.FUENTE_ROBOT
    assert ar.fuente_rostro(dry=True, no_robot_cam=True) == ar.FUENTE_PRUEBA
    assert ar.fuente_rostro(dry=False, no_robot_cam=True) is None   # sin webcam como sustituto


def test_la_v_solo_dispara_armada_y_con_la_misma_confirmacion():
    """La accion la sigue decidiendo ControlGestos: desarmado no hay evento; armado,
    V mantenida ACTION_HOLD (y con mejorados, ademas T_MOVER)."""
    for nuevo in (gt.ControlGestos, lambda: gt.nuevo_control(gt.ParamsGestos.mejorados())):
        desarmado = nuevo()
        assert "accion" not in [desarmado.update("victoria", i / 30)[1] for i in range(150)]
        ctrl = nuevo()
        ctrl.armed = True
        ev_t = next((i / 30 for i in range(150) if ctrl.update("victoria", i / 30)[1] == "accion"),
                    None)
        assert ev_t is not None and ev_t >= gt.ACTION_HOLD


def test_opcion_apagada_por_defecto():
    import inspect
    src = inspect.getsource(gt.main)
    assert '"--v-rostro"' in src
    g = gt.Gestos.__new__(gt.Gestos)
    assert not hasattr(g, "copia_limpia")  # atributo solo existe tras __init__ (False)


def test_gestos_no_copia_el_frame_si_no_se_pide():
    class Cap:
        def read(self):
            return True, NEGRO.copy()

        def release(self):
            pass
    g = gt.Gestos(0, cap=Cap(), reloj_t=True)
    try:
        g.leer(0.0)
        assert g.copia_limpia is False and g.limpio is None
        g.copia_limpia = True
        g.leer(0.1)
        assert g.limpio is not None and g.limpio.shape == NEGRO.shape
    finally:
        g.close()
