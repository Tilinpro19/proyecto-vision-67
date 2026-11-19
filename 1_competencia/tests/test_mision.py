"""Maquina de estados, choques, llegada a B y escaneo de la mision (sin ROS)."""
import json
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "2_laboratorio"))

import mision as ms  # noqa: E402
from comun import apply_params  # noqa: E402
from navegacion import PilotoParams  # noqa: E402


class Reloj:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_cronometro_arranca_al_salir_de_a_y_para_en_fin(tmp_path):
    r = Reloj()
    m = ms.Mision("A", str(tmp_path), clock=r, quiet=True)
    r.t += 5
    assert m.tiempo() == 0.0                    # en ESPERA no corre
    m.ir(ms.IR_A_B)
    r.t += 20
    m.ir(ms.ESCANEAR)
    m.ir(ms.VOLVER_A_A)
    m.choque("LiDAR")
    r.t += 15
    m.ir(ms.ANUNCIAR)
    m.ir(ms.FIN)
    r.t += 50
    res = m.resultado()
    assert res["ok"] and res["tiempo"] == 35.0 and res["choques"] == 1
    m.cerrar()
    log = open(m.log_path, encoding="utf-8").read()
    assert "IR_A_B -> ESCANEAR" in log and "CHOQUE #1" in log


def test_parada_de_emergencia_no_es_ok():
    m = ms.Mision("A", None, clock=Reloj(), quiet=True)
    m.ir(ms.IR_A_B)
    m.ir(ms.PARADA, "Ctrl+C")
    assert not m.activa() and not m.resultado()["ok"]


def test_contador_choques_un_evento_por_contacto():
    c = ms.ContadorChoques(0.20, histeresis=0.03, rearme=1.0)
    seq = [(0.0, 0.5), (0.1, 0.19), (0.2, 0.18), (0.3, 0.21), (0.4, 0.19),   # roce: 1
           (0.5, 0.30), (1.0, 0.30), (1.6, 0.30), (1.7, 0.15),               # nuevo: 2
           (2.0, 0.40, True)]                                                # bumper seguido
    n = sum(c.update(*s) for s in seq)
    assert n == 2


def test_llegada_enchufable():
    mp = ms.MisionParams()
    lejos = dict(dist=1.0, front=0.5)
    cerca = dict(dist=0.1, front=2.0)
    assert ms.llegada_b(cerca, 0.0, mp, 0.25) and not ms.llegada_b(lejos, 0.0, mp, 0.25)
    mp.llegada = "foto"
    assert ms.llegada_b(lejos, 0.2, mp, 0.25) and not ms.llegada_b(cerca, 0.0, mp, 0.25)
    mp.llegada = "cualquiera"
    assert ms.llegada_b(lejos, 0.2, mp, 0.25) and ms.llegada_b(cerca, 0.0, mp, 0.25)


class FakeRostros:
    def detectar(self, img):
        return img, np.ones((1, 15)) if img is not None and img.mean() > 0 else np.empty((0, 15))


def test_escaneo_junta_frames_y_no_gira_si_ve_cara():
    mp = ms.MisionParams(escaneo_frames=3)
    e = ms.Escaneo(FakeRostros(), mp, t0=0.0)
    cara = np.ones((4, 4, 3))
    outs = [e.paso(t, cara) for t in (0.1, 0.2, 0.3)]
    assert outs[-1][1] is not None and len(outs[-1][1]) == 3
    assert all(a == 0.0 for a, _ in outs)


def test_escaneo_barre_cada_vez_mas_ancho_si_no_ve_nada():
    mp = ms.MisionParams(escaneo_max=20.0)
    e = ms.Escaneo(FakeRostros(), mp, t0=0.0)
    vacio = np.zeros((4, 4, 3))
    ang = [e.paso(t / 10, vacio)[0] for t in range(0, 200)]
    assert ang[0] == 0.0                         # primero mira quieto
    giro = np.cumsum(ang) * 0.1
    assert giro.max() > 1.0 and giro.min() < -1.0  # llega a mas de +-57 grados
    assert e.paso(20.5, vacio) == (0.0, [])      # al final se rinde sin frames


def test_escaneo_objetivo_descarta_otra_persona_y_lee_qr_del_objetivo():
    import math
    from types import SimpleNamespace

    mp = ms.MisionParams(escaneo_frames_persona=2, busqueda_angular=0.4, sector_visto_rad=0.6)
    galeria = SimpleNamespace(nombres=["Ana", "Mateo"])

    class RostrosPorAngulo:
        def detectar(self, img):
            # img es un dict/marcador o array; usamos shape[0] == 8 para indicar cara
            return img, (np.ones((1, 15)) if img is not None and img.shape[0] == 8 else np.empty((0, 15)))

    def fake_identificar(frames):
        tag = int(frames[0][0, 0, 0])
        if tag == 1:
            return SimpleNamespace(nombre="Ana", score=0.72, seguro=True, n_caras=len(frames), ranking=[("Ana", 0.72)])
        return SimpleNamespace(nombre="Mateo", score=0.81, seguro=True, n_caras=len(frames), ranking=[("Mateo", 0.81)])

    def fake_qr(img):
        tag = int(img[0, 0, 0])
        return ["QR_MATEO_01"] if tag == 2 else []

    esc = ms.EscaneoObjetivo("Mateo", RostrosPorAngulo(), galeria, mp, t0=0.0, yaw0=0.0,
                             qr_fn=fake_qr, identificar_fn=fake_identificar)

    vacio = np.zeros((4, 4, 3), dtype=np.uint8)
    cara_ana = np.ones((8, 8, 3), dtype=np.uint8) * 1
    cara_mateo = np.ones((8, 8, 3), dtype=np.uint8) * 2

    # Gira buscando; al ver a Ana (+1.57 rad), se detiene a recolectar pero sigue buscando
    ang, res = esc.paso(0.1, 0.2, vacio)
    assert ang == pytest.approx(0.4) and res is None

    ang, res = esc.paso(0.2, 1.57, cara_ana)
    assert ang == 0.0 and res is None
    ang, res = esc.paso(0.3, 1.57, cara_ana)
    assert ang == 0.0 and res is None and esc.subestado == "BUSCAR"

    # En el mismo sector de Ana no re-escanea
    ang, res = esc.paso(0.4, 1.65, cara_ana)
    assert ang == pytest.approx(0.4) and res is None

    # Sigue girando hasta el otro lado (-1.57 rad) donde esta Mateo + su QR
    ang, res = esc.paso(0.5, -1.57, cara_mateo)
    assert ang == 0.0 and res is None
    ang, res = esc.paso(0.6, -1.57, cara_mateo)
    assert res is not None
    assert res["encontrado"] is True
    assert res["nombre"] == "Mateo"
    assert res["qr"] == "QR_MATEO_01"


def test_escaneo_objetivo_panea_para_leer_qr_al_costado():
    from types import SimpleNamespace

    mp = ms.MisionParams(escaneo_frames_persona=1, qr_barrido_max=5.0)
    galeria = SimpleNamespace(nombres=["Mateo"])
    cara_sin_qr = np.ones((8, 8, 3), dtype=np.uint8)
    solo_qr = np.ones((4, 4, 3), dtype=np.uint8) * 9

    class Rostros:
        def detectar(self, img):
            return img, (np.ones((1, 15)) if img is not None and img.shape[0] == 8 else np.empty((0, 15)))

    def fake_identificar(frames):
        return SimpleNamespace(nombre="Mateo", score=0.75, seguro=True, n_caras=1, ranking=[("Mateo", 0.75)])

    def fake_qr(img):
        return ["QR_COSTADO"] if int(img[0, 0, 0]) == 9 else []

    esc = ms.EscaneoObjetivo("Mateo", Rostros(), galeria, mp, t0=0.0, yaw0=0.0,
                             qr_fn=fake_qr, identificar_fn=fake_identificar)

    # Ve cara sin QR -> se detiene a recolectar y luego pasa a BUSCAR_QR
    ang, res = esc.paso(0.1, 0.0, cara_sin_qr)
    assert ang == 0.0 and res is None and esc.subestado == "RECOLECTAR"
    ang, res = esc.paso(0.2, 0.0, cara_sin_qr)
    assert res is None and esc.subestado == "BUSCAR_QR"

    # Panea hacia la izquierda y luego ve el QR al costado
    ang, res = esc.paso(0.3, 0.2, np.zeros((4, 4, 3), dtype=np.uint8))
    assert ang > 0 and res is None
    ang, res = esc.paso(0.4, 0.45, solo_qr)
    assert res is not None and res["qr"] == "QR_COSTADO" and res["nombre"] == "Mateo"


def test_escaneo_objetivo_no_se_cuelga_si_no_esta_y_vuelve_tras_360():
    from types import SimpleNamespace

    mp = ms.MisionParams(escaneo_frames_persona=1, escaneo_objetivo_max=30.0)
    galeria = SimpleNamespace(nombres=["Ana", "Mateo"])

    class SinCaras:
        def detectar(self, img):
            return img, np.empty((0, 15))

    esc = ms.EscaneoObjetivo("Mateo", SinCaras(), galeria, mp, t0=0.0, yaw0=0.0)
    vacio = np.zeros((4, 4, 3), dtype=np.uint8)
    res = None
    # Simula una vuelta completa en pasos de 0.5 rad
    for i in range(1, 16):
        yaw = ((i * 0.5 + 3.14159) % (2 * 3.14159)) - 3.14159
        _, res = esc.paso(i * 0.5, yaw, vacio)
        if res is not None:
            break
    assert res is not None and res["encontrado"] is False and res["qr"] is None


def test_apply_params_json_y_overrides(tmp_path):
    pp, mp = PilotoParams(), ms.MisionParams()
    cfg = tmp_path / "c.json"
    cfg.write_text(json.dumps({"_nota": "x", "max_linear": 0.25, "llegada": "foto"}))
    apply_params([pp, mp], str(cfg), ["t_max=120", "max_linear=0.3"])
    assert pp.max_linear == pytest.approx(0.3) and mp.t_max == 120.0 and mp.llegada == "foto"


def test_config_de_competencia_es_valida():
    import nav_core as nc
    from navegacion import Piloto
    p = Piloto()
    cfg = os.path.join(os.path.dirname(HERE), "config.json")
    from directo import DirectoParams
    vals = apply_params([p.nav, p.pp, ms.MisionParams(), DirectoParams()], cfg)
    objs = (nc.NavParams(), PilotoParams(), ms.MisionParams(), DirectoParams())
    assert all(any(hasattr(o, k) for o in objs) for k in vals)


# --- 2_laboratorio/calibrar.py ---------------------------------------------------

def test_calibrar_costo_y_resumen():
    import calibrar as cb
    ok = dict(ok=True, tiempo=60.0, choques=2, nombre="Juan")
    assert cb.costo(ok, c=5) == 70.0
    assert cb.costo(ok, c=5, e=30, esperado="Ana") == 100.0
    assert cb.costo(dict(ok=False, choques=1), c=5, t_max=200) == 205.0
    assert cb.combinaciones(["a=1,2", "b=3"]) == [{"a": "1", "b": "3"}, {"a": "2", "b": "3"}]
    filas = [dict(config="x", ok=True, costo=70.0, choques=2),
             dict(config="y", ok=True, costo=50.0, choques=0),
             dict(config="y", ok=False, costo=300.0, choques=0)]
    r = cb.resumen(filas)
    assert [f["config"] for f in r] == ["x", "y"] and r[1]["exitos"] == 1
