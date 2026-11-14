"""Exploracion por fronteras en el simulador (sin robot ni ROS).

Correr:  python3 -m pytest 1_competencia/tests/test_exploracion.py -v -s
"""
import math

import numpy as np
import pytest

from sim import Sim, box

import nav_core as nc  # noqa: E402
from explorar import Explorador, ExplorarParams, MonitorBateria  # noqa: E402
from navegacion import Piloto  # noqa: E402


def S(x1, y1, x2, y2):
    return ((x1, y1), (x2, y2))


def casa():
    """Sala 1 (x -1..5, y -3..3) -> puerta 1.2 m -> pasillo -> sala 2 (x 9..14), con cajas."""
    w = [S(-1, -3, 5, -3), S(-1, 3, 5, 3), S(-1, -3, -1, 3),
         S(5, -3, 5, -0.6), S(5, 0.6, 5, 3),                       # pared con puerta
         S(5, -0.6, 9, -0.6), S(5, 0.6, 9, 0.6),                   # pasillo
         S(9, -3, 14, -3), S(9, 3, 14, 3), S(14, -3, 14, 3),
         S(9, -3, 9, -0.6), S(9, 0.6, 9, 3)]
    return w + box(2.5, 1.5, 0.6, 0.6) + box(11.0, -1.2, 0.8, 0.5)


def explorar(walls, t_max=900.0, deriva=0.0, ep=None):
    sim = Sim(walls, yaw_drift=deriva)
    exp = Explorador(Piloto(), ep or ExplorarParams(radio_max=20.0))
    exp.iniciar(sim.odom, *sim.scan())
    t, dt = 0.0, 0.1
    while t < t_max and exp.estado != "fin":
        lin, ang, info = exp.paso(t, sim.odom, *sim.scan())
        sim.step(lin, ang, dt)
        t += dt
    return sim, exp, t


def cobertura(sim, exp, x0, x1, y0, y1, margen=0.45, paso=0.25):
    """Fraccion de puntos interiores (a >= margen de las paredes) que el mapa marca libres."""
    tot = ok = 0
    for x in np.arange(x0, x1, paso):
        for y in np.arange(y0, y1, paso):
            if sim.clearance((x, y), sim.walls) < margen:
                continue
            tot += 1
            ok += exp.mapa.libre_en(x, y)
    return ok / max(tot, 1)


def test_explora_toda_la_casa_y_vuelve():
    sim, exp, t = explorar(casa())
    print("\n", exp.resumen(t), f"| {t:.0f} s simulados | choques {sim.collisions}")
    assert exp.estado == "fin" and exp.razon_fin.startswith("no quedan")
    assert sim.collisions == 0
    assert sim.min_clear > 0.17
    assert cobertura(sim, exp, -1, 5, -3, 3) > 0.95        # sala 1
    assert cobertura(sim, exp, 5, 9, -0.6, 0.6, 0.3) > 0.9  # pasillo
    assert cobertura(sim, exp, 9, 14, -3, 3) > 0.95        # sala 2
    x, y, _ = sim.true
    assert math.hypot(x, y) < 0.4                           # volvio a A


def test_con_deriva_de_odometria():
    sim, exp, t = explorar(casa(), deriva=0.03)
    print("\n", exp.resumen(t), f"| {t:.0f} s simulados | choques {sim.collisions}")
    assert exp.estado == "fin"
    assert sim.collisions == 0
    assert cobertura(sim, exp, 9, 14, -3, 3) > 0.85


def test_valla_virtual_no_pasa_del_radio():
    ep = ExplorarParams(radio_max=5.0)
    sim, exp, t = explorar(casa(), ep=ep)
    xs = [p[0] for p in exp.trayecto]
    assert max(xs) < 5.6                                    # no cruza al pasillo lejano
    assert exp.estado == "fin" and sim.collisions == 0


def test_habitacion_cerrada_termina_rapido():
    sim, exp, t = explorar(box(1.5, 0.0, 4.0, 4.0))
    assert exp.estado == "fin" and t < 120
    assert cobertura(sim, exp, -0.5, 3.5, -1.5, 1.5) > 0.9


def test_obstaculo_bajo_permanente_y_descarta_meta():
    # Obstaculo bajo invisible al LiDAR tapando la puerta hacia el pasillo
    sim = Sim(casa(), low=box(5.2, 0.0, 0.4, 1.1))
    exp = Explorador(Piloto(), ExplorarParams(radio_max=20.0))
    exp.iniciar(sim.odom, *sim.scan())
    t, dt = 0.0, 0.1
    while t < 300.0 and exp.estado != "fin":
        lin, ang, _ = exp.paso(t, sim.odom, *sim.scan())
        sim.step(lin, ang, dt)
        t += dt
    assert exp.estado == "fin"
    assert sim.collisions == 0
    # La barrera de obstaculo bajo permanece (no caduca) y sella la puerta sin bucle infinito
    assert len(exp.piloto.virtual.points) > 0
    assert cobertura(sim, exp, 9, 14, -3, 3) == 0.0
    x, y, _ = sim.true
    assert math.hypot(x, y) < 0.4


def test_atasco_repetido_descarta_meta():
    sim = Sim(casa())
    exp = Explorador(Piloto(), ExplorarParams(radio_max=20.0))
    exp.iniciar(sim.odom, *sim.scan())
    r, a = sim.scan()
    # Primer paso para fijar una meta valida
    exp.paso(0.0, sim.odom, r, a)
    assert exp.target is not None
    meta_inicial = exp.target
    # Simulamos un atasco repetido en el mismo lugar
    exp.piloto._last_escape_xy = (0.0, 0.0)
    exp.piloto._repeat_escapes = 1
    exp.piloto.monitor.update(1.0, (0.0, 0.0))
    exp.paso(1.0 + exp.piloto.pp.stuck_time + 0.5, sim.odom, r, a)
    assert meta_inicial in exp.blacklist
    assert exp.target is None


def test_monitor_bateria_histeresis():
    mon = MonitorBateria(umbral=0.22, hysteresis_s=5.0)
    # Picos breves bajo 22% que se recuperan no deben abortar
    assert not mon.update(0.0, 0.50)
    assert not mon.update(1.0, 0.20)
    assert not mon.update(4.0, 0.21)
    assert not mon.update(4.5, 0.25)
    # 5 s continuos bajo 22% si disparan el aborto
    assert not mon.update(10.0, 0.21)
    assert not mon.update(14.9, 0.20)
    assert mon.update(15.0, 0.20)
