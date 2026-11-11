"""Eficiencia en la Pi sin robot: ritmo del ciclo de control, perfil de CPU y un salon con
varios obstaculos recorrido de ida y vuelta midiendo cuanto cuesta cada ciclo de navegacion.

Correr desde 1_competencia/:  python3 -m pytest tests/test_eficiencia.py -v -s
"""
import math
import time

import pytest

from sim import Sim, box, run

import mision as ms  # noqa: E402
import nav_core as nc  # noqa: E402
from navegacion import Piloto  # noqa: E402

P = nc.NavParams()


# --- ritmo del ciclo -----------------------------------------------------------

class Reloj:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_drenar_atiende_varios_callbacks_por_ciclo():
    reloj, llamadas = Reloj(), []

    def spin_once(timeout):          # cada callback tarda 20 ms
        llamadas.append(timeout)
        reloj.t += 0.02

    ms.drenar(spin_once, 0.1, reloj)
    assert len(llamadas) == 5        # 100 ms / 20 ms: todos los mensajes del ciclo
    assert reloj.t == pytest.approx(0.1)


def test_drenar_hace_al_menos_un_spin_y_no_pasa_del_periodo():
    reloj, llamadas = Reloj(), []

    def spin_once(timeout):          # sin mensajes: espera hasta el timeout
        llamadas.append(timeout)
        reloj.t += timeout

    ms.drenar(spin_once, 0.1, reloj)
    assert llamadas == [0.1]
    assert reloj.t == pytest.approx(0.1)


def test_drenar_un_callback_lento_no_se_repite_de_mas():
    reloj, llamadas = Reloj(), []

    def spin_once(timeout):          # un callback tarda mas que el periodo
        llamadas.append(timeout)
        reloj.t += 0.25

    ms.drenar(spin_once, 0.1, reloj)
    assert len(llamadas) == 1


# --- perfil --------------------------------------------------------------------

def test_perfil_resumen():
    wall, cpu = Reloj(), Reloj()
    perfil = ms.Perfil(reloj=wall, cpu=cpu)
    for dur in (0.002, 0.004, 0.010):
        t0 = wall()
        wall.t += dur
        perfil.registrar(t0)
    wall.t, cpu.t = 10.0, 0.5        # 0.5 s de CPU en 10 s de pared = 5 % de un nucleo
    r = perfil.resumen()
    assert r["ciclos"] == 3
    assert r["ciclo_ms_max"] == pytest.approx(10.0)
    assert r["ciclo_ms_prom"] == pytest.approx(16.0 / 3, abs=0.01)
    assert r["cpu_pct"] == pytest.approx(5.0)


def test_perfil_vacio_no_falla():
    assert ms.Perfil().resumen()["ciclos"] == 0


def test_resultado_incluye_perfil():
    m = ms.Mision("A", quiet=True)
    assert "perfil" in m.resultado()


# --- salon con obstaculos --------------------------------------------------------

def salon():
    """Salon de 7 x 4.5 m con 6 obstaculos repartidos (cajas y una columna) entre A y B."""
    paredes = box(2.5, 0.0, 7.0, 4.5)
    obstaculos = (box(1.0, 0.5, 0.4, 0.5) + box(1.8, -0.9, 0.5, 0.4) + box(2.5, 0.7, 0.3, 0.3)
                  + box(3.2, -0.3, 0.6, 0.3) + box(4.0, 0.9, 0.4, 0.4) + box(4.4, -1.0, 0.3, 0.6))
    return paredes + obstaculos


def test_salon_con_obstaculos_ida_y_vuelta_sin_choques_y_barato():
    sim = Sim(salon(), yaw_drift=0.05)
    piloto = Piloto()
    perfil = ms.Perfil()
    paso_original = piloto.paso

    def paso_medido(*a, **k):
        t0 = perfil.reloj()
        r = paso_original(*a, **k)
        perfil.registrar(t0)
        return r

    piloto.paso = paso_medido
    res = run(sim, piloto, (5.2, 0.0), t_max=240)
    r = perfil.resumen()
    print(f"\nsalon: ok={res['ok']} t_sim={res['t']:.1f}s choques={res['collisions']} "
          f"holgura_min={res['min_clear']:.2f} m | ciclo prom {r['ciclo_ms_prom']} ms, "
          f"p95 {r['ciclo_ms_p95']} ms, max {r['ciclo_ms_max']} ms")
    assert res["ok"], f"no completo la mision: {res}"
    assert res["collisions"] == 0
    assert res["min_clear"] > P.robot_radius + 0.03
    assert math.hypot(*res["true"][:2]) < 0.4
    # Cota holgada para el portatil; en la Pi 4 (~5-10x mas lenta) debe seguir << 100 ms,
    # el periodo del ciclo a 10 Hz. Lo importante es detectar una regresion grande.
    assert r["ciclo_ms_prom"] < 25
