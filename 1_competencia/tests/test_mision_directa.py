"""tarea_a_directa() de mision.py de punta a punta con un Sistema falso sobre el simulador:
mismo bucle que en el robot (spin, camara, deteccion, escaneo, regreso, anuncio) sin ROS."""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import directo as dr  # noqa: E402
import mision as ms  # noqa: E402
import senales_forma as sf  # noqa: E402
from camara_sim import ANCHO, ver  # noqa: E402
from sim import Sim  # noqa: E402
from test_directo import (CARAS_ZIGZAG, FLECHAS_ZIGZAG, _piloto_config,  # noqa: E402
                          pista_zigzag)


class Reloj:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class SistemaSim:
    """Lo que tarea_a_directa usa de Sistema, sobre Sim + camara_sim."""

    def __init__(self, paredes, objetos, mp, reloj):
        self.sim, self.objetos, self.paredes, self.mp, self.reloj = Sim(paredes), objetos, paredes, mp, reloj
        self.cmd = (0.0, 0.0)
        self.parar = False
        self.anuncios, self.pausas = [], 0
        sim = self.sim
        self.lidar = SimpleNamespace(fresh=lambda: True)
        self.odom = SimpleNamespace(fresh=lambda: True)
        self.lidar.data = sim.scan()
        self.odom.pose = sim.odom
        self.cam = SimpleNamespace(pausar=self._pausar)
        self.sender = SimpleNamespace(send=self._send, stop=lambda: self._send(0.0, 0.0))
        self.anunciador = SimpleNamespace(anunciar=self.anuncios.append)
        self.rostros = SimpleNamespace(detectar=self._detectar_rostros)

    def _pausar(self):
        self.pausas += 1

    def _send(self, lin, ang):
        self.cmd = (lin, ang)

    def spin(self, dt):
        self.sim.step(*self.cmd, dt)
        self.reloj.t += dt
        self.lidar.data = self.sim.scan()
        self.odom.pose = self.sim.odom

    def bump(self):
        return False

    def frame_nuevo(self):
        senales, cara = ver(self.sim.true, self.objetos, self.paredes)
        return SimpleNamespace(shape=(480, ANCHO, 3), senales=senales, cara=cara)

    def vista_cara(self, img):
        return img.cara

    def _detectar_rostros(self, img):
        caras = np.ones((1, 15)) if img is not None and img.cara > 0.05 else np.empty((0, 15))
        return img, caras

    def identificar(self, frames, m):
        m.nombre, m.score, m.seguro = "Thiago_Frias", 0.8, True


def test_tarea_a_directa_completa_y_anuncia(monkeypatch):
    reloj = Reloj()
    monkeypatch.setattr(ms.time, "monotonic", reloj)
    monkeypatch.setattr(sf, "buscar_candidatos", lambda img: True)
    monkeypatch.setattr(sf, "detectar", lambda img, **kw: img.senales)
    piloto, mp, dp = _piloto_config()
    mp.t_max = 200.0
    s = SistemaSim(pista_zigzag(), FLECHAS_ZIGZAG + CARAS_ZIGZAG, mp, reloj)
    m = ms.Mision("A", quiet=True, clock=reloj)
    args = SimpleNamespace(ver=False)
    ms.tarea_a_directa(args, s, m, piloto, dp)

    res = m.resultado()
    assert m.estado == ms.FIN, m.eventos[-5:]
    assert s.anuncios == ["Thiago_Frias"]
    assert s.sim.collisions == 0
    assert math.hypot(s.sim.true[0], s.sim.true[1]) < 0.35
    assert [d["accion"] for d in res["decisiones"]] == ["izq", "izq"]
    assert any("llegada a B: caras al frente" in e for e in m.eventos)
    assert s.pausas == 1                         # camara pausada en el regreso
    assert piloto.pp.max_linear == dp.v_regreso
    assert res["tiempo"] < 120.0


def test_main_usa_modo_directo_sin_goal(monkeypatch):
    llamado = {}
    monkeypatch.setattr(ms, "tarea_a_directa", lambda *a, **k: llamado.setdefault("directo", a))
    monkeypatch.setattr(ms, "tarea_a_guiada", lambda *a, **k: llamado.setdefault("guiado", a))
    monkeypatch.setattr(ms, "Sistema", lambda args, mp: SimpleNamespace(cerrar=lambda: None))
    monkeypatch.setattr(ms, "esperar_sensores", lambda s, m: True)
    monkeypatch.setattr(ms.Mision, "__init__", _mision_sin_log(ms.Mision.__init__))
    for argv, esperado in ((["--tarea", "A"], "directo"),
                           (["--tarea", "A", "--modo", "guiado"], "guiado")):
        llamado.clear()
        monkeypatch.setattr(sys, "argv", ["mision.py", *argv, "--param", "v_recto=0.24"])
        ms.main()
        assert list(llamado) == [esperado]
        if esperado == "directo":
            assert llamado["directo"][4].v_recto == 0.24   # --param llega a DirectoParams


def _mision_sin_log(init):
    def nuevo(self, tarea, log_dir=None, clock=None, quiet=False):
        init(self, tarea, None, clock or ms.time.monotonic, True)
    return nuevo


def test_directo_params_en_config_json_existen():
    # cualquier clave de config.json debe existir en algun objeto de parametros
    import json
    piloto, mp, dp = _piloto_config()
    with open(os.path.join(os.path.dirname(HERE), "config.json"), encoding="utf-8") as f:
        claves = [k for k in json.load(f) if not k.startswith("_")]
    objs = [piloto.nav, piloto.pp, mp, dp]
    assert all(any(hasattr(o, k) for o in objs) for k in claves)
    assert isinstance(dp, dr.DirectoParams)
