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
    vals = apply_params([p.nav, p.pp, ms.MisionParams()], cfg)
    objs = (nc.NavParams(), PilotoParams(), ms.MisionParams())
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
