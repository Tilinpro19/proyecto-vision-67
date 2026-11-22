"""Escenarios de la Tarea A sin robot: scans simulados (pasillo, esquina, callejon,
obstaculo bajo, pared lateral) y misiones completas A -> B -> A en el simulador.

Correr desde la raiz del repo:  python3 -m pytest 1_competencia/tests -v
"""
import math

import numpy as np
import pytest

from sim import Sim, box, corridor, raycast, run, scan_from

import nav_core as nc  # noqa: E402  (sim.py ya agrega 1_competencia/ al path)
from navegacion import Piloto, PilotoParams  # noqa: E402

P = nc.NavParams()
DEG = math.radians(1)


def room(w=6.0, h=4.0, x0=-1.0, y0=-2.0):
    return box(x0 + w / 2, y0 + h / 2, w, h)


# --- escenarios estaticos ------------------------------------------------------

def test_pasillo_cerca_de_la_pared_izq_se_aleja():
    r, a = raycast(corridor(0.25, -0.45))
    assert nc.wall_repulsion(r, a, P) < 0          # gira a la derecha
    h = nc.choose_heading(r, a, goal=0.0)
    assert h is not None and h <= 0


def test_pared_lateral_reduce_angular():
    r, a = raycast(corridor(0.22, -3.0))
    assert nc.angular_limit(1.2, nc.nearest(r), P) < 1.2
    r2, a2 = raycast(corridor(2.0, -2.0))
    assert nc.angular_limit(1.2, nc.nearest(r2), P) == pytest.approx(1.2)
    assert nc.wall_repulsion(r2, a2, P) == 0.0


def test_esquina_gira_hacia_el_lado_abierto():
    # pared al frente a 0.7 m y pared a la izquierda: lo libre esta a la derecha
    r, a = raycast([((0.7, -3.0), (0.7, 0.6)), ((-1.0, 0.6), (0.7, 0.6))])
    h = nc.choose_heading(r, a, goal=0.0)
    assert h is not None and h < -math.radians(45)


def test_callejon_sin_salida_no_sigue_de_frente():
    r, a = raycast(corridor(0.35, -0.35, length=0.65) + [((0.65, -0.35), (0.65, 0.35))])
    h = nc.choose_heading(r, a, goal=0.0)
    assert h is None or abs(h) > math.radians(90)
    assert abs(nc.most_open(r, a)) > math.radians(150)   # la salida esta atras


def test_obstaculo_bajo_virtual_bloquea_el_frente():
    r, a = raycast(room())                              # el LiDAR no ve nada cerca
    vo = nc.VirtualObstacles()
    vo.add_front((0.0, 0.0, 0.0), 0.21, 0.17)
    vr, va = vo.in_robot((0.0, 0.0, 0.0))
    R, A = np.r_[r, vr], np.r_[a, va]
    assert nc.free_distance(r, a, 0.0)[0] > 1.0
    assert nc.free_distance(R, A, 0.0)[0] < 0.05
    h = nc.choose_heading(R, A, goal=0.0)
    assert h is not None and abs(h) > math.radians(30)


# --- piezas de la mision -------------------------------------------------------

def test_compose_inverse_relative():
    a, b = (1.0, 2.0, 0.5), (0.3, -0.2, -1.0)
    ab = nc.compose(a, b)
    assert nc.relative(a, ab) == pytest.approx(b)
    assert nc.compose(a, nc.inverse(a)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_progress_monitor():
    m = nc.ProgressMonitor(window=2.0, min_progress=0.1)
    assert not any(m.update(t * 0.1, (0.0, 0.0)) for t in range(19))
    assert m.update(2.0, (0.01, 0.0))
    m.reset()
    assert not any(m.update(t * 0.1, (t * 0.02, 0.0)) for t in range(40))


def test_icp_recupera_el_desplazamiento():
    walls = room() + box(1.0, 1.0, 0.4, 0.3) + box(2.5, -1.0, 0.3, 0.6)
    ref = nc.subsample(nc.to_xy(*scan_from(walls, (0.0, 0.0, 0.0))))
    true = (0.18, -0.12, math.radians(6))
    cur = nc.subsample(nc.to_xy(*scan_from(walls, true)))
    pose, inl, rmse = nc.icp_2d(ref, cur)
    assert pose[:2] == pytest.approx(true[:2], abs=0.03)
    assert pose[2] == pytest.approx(true[2], abs=math.radians(1.5))
    assert inl > 0.8 and rmse < 0.03


def test_pick_waypoint_recorta_si_hay_linea_libre():
    crumbs = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0)]
    r, a = nc.clean_scan([], [])
    assert nc.pick_waypoint(crumbs, (2.0, 1.0, 0.0), r, a, upto=3) == 0
    # con una pared entre el robot y A solo puede ir a migas visibles
    wall = [((1.4, 0.4), (1.4, 3.0)), ((-1.0, 0.4), (1.4, 0.4))]
    r, a = scan_from(wall, (2.0, 1.0, 0.0))
    assert nc.pick_waypoint(crumbs, (2.0, 1.0, 0.0), r, a, upto=3) >= 2


# --- misiones completas en el simulador ----------------------------------------

def check(res, sim, tol=0.30):
    assert res["ok"], f"no completo la mision: {res}"
    assert res["collisions"] == 0
    assert res["min_clear"] > P.robot_radius + 0.03
    assert math.hypot(*res["true"][:2]) < tol, f"termino lejos de A: {res['true']}"


def test_mision_obstaculo_en_medio():
    sim = Sim(room() + box(1.5, 0.0, 0.4, 0.4))
    res = run(sim, Piloto(), (3.0, 0.0))
    check(res, sim)
    assert math.hypot(res["b_true"][0] - 3.0, res["b_true"][1]) < 0.35


def test_mision_pasillo_en_L():
    walls = [((-0.5, -0.5), (3.0, -0.5)), ((3.0, -0.5), (3.0, 3.0)),
             ((-0.5, 0.5), (2.0, 0.5)), ((2.0, 0.5), (2.0, 3.0)), ((-0.5, -0.5), (-0.5, 0.5))]
    sim = Sim(walls)
    check(run(sim, Piloto(), (2.5, 2.5)), sim)


def test_mision_obstaculo_bajo_invisible():
    sim = Sim(room(), low=box(1.3, 0.0, 0.25, 0.5))
    piloto = Piloto()
    res = run(sim, piloto, (2.8, 0.0), t_max=240)
    check(res, sim)
    assert len(piloto.virtual.points) > 0


def test_mision_con_deriva_de_odometria_corrige_con_icp():
    walls = room() + box(1.5, 0.6, 0.4, 0.4) + box(1.0, -1.2, 0.5, 0.3) + box(3.2, 1.2, 0.3, 0.3)
    sin_icp = PilotoParams(icp_max_jump=0.0)  # ICP siempre descartado
    res_mal = run(Sim(walls, yaw_drift=0.08), Piloto(pp=sin_icp), (3.5, -0.5))
    sim = Sim(walls, yaw_drift=0.08)
    res = run(sim, Piloto(), (3.5, -0.5))
    check(res, sim)
    err_mal = math.hypot(*res_mal["true"][:2]) if res_mal["ok"] else math.inf
    assert err_mal > 0.4  # la deriva sin corregir si deja lejos de A


def test_virtual_obstacles_ttl_y_limpieza():
    vo = nc.VirtualObstacles()
    vo.add_front((0.0, 0.0, 0.0), 0.3, 0.1, t=10.0)
    vo.add_front((2.0, 0.0, 0.0), 0.3, 0.1, t=50.0)
    assert len(vo.points) > 0
    vo.prune_expired(60.0, ttl=45.0)
    assert all(p[0] > 1.0 for p in vo.points)
    vo.clear_near((2.3, 0.0), 0.5)
    assert len(vo.points) == 0


def test_escape_no_queda_en_jaula_virtual():
    r, a = raycast(room())
    piloto = Piloto()
    piloto.fijar_origen((0.0, 0.0, 0.0), r, a)
    piloto.ir_a((2.0, 0.0))
    # Simulamos barreras virtuales rodeando al robot en 4 direcciones
    for ang in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        piloto.virtual.add_front((0.0, 0.0, ang), 0.25, 0.20, t=1.0)
    n_antes = len(piloto.virtual.points)
    # Dos atascos en el mismo sitio no borran barreras (evita olvidar obstaculos bajos)
    # y aun asi permiten retroceder usando el LiDAR real
    rep1 = piloto._iniciar_escape(2.0, (0.0, 0.0, 0.0), r, a, 0.0)
    rep2 = piloto._iniciar_escape(8.0, (0.01, 0.0, 0.0), r, a, 0.0)
    assert not rep1 and rep2
    assert len(piloto.virtual.points) == n_antes
    lin, _ = piloto._escape(8.1, (0.01, 0.0, 0.0), r, a, r, a)
    assert lin < 0.0  # puede retroceder usando el LiDAR real


def test_obstaculo_bajo_y_bumper_no_caducan():
    r, a = raycast(room())
    piloto = Piloto()
    piloto.fijar_origen((0.0, 0.0, 0.0), r, a)
    piloto.ir_a((3.0, 0.0))
    # Obstaculo bajo (frente libre y avanzando): barrera permanente
    piloto.lin = 0.15
    piloto._iniciar_escape(5.0, (0.0, 0.0, 0.0), r, a, 2.0)
    n_bajo = len(piloto.virtual.points)
    assert n_bajo > 0
    # Sellar trampa (frente obstruido en otro lugar): barrera con TTL
    piloto.lin = 0.02
    piloto._iniciar_escape(10.0, (2.0, 0.0, 0.0), r, a, 0.25)
    assert len(piloto.virtual.points) > n_bajo
    # Pasado virtual_ttl, solo caduca la barrera de trampa; la de obstaculo bajo sigue viva
    piloto.virtual.prune_expired(10.0 + piloto.pp.virtual_ttl + 5.0, piloto.pp.virtual_ttl)
    assert len(piloto.virtual.points) == n_bajo
    assert all(p[0] < 1.0 for p in piloto.virtual.points)
    # Bumper tambien pone barrera permanente
    piloto.escape = None
    piloto.virtual.clear_near((0.0, 0.0), 5.0)
    piloto.paso(200.0, (0.0, 0.0, 0.0), r, a, bumper_hit=True)
    n_bumper = len(piloto.virtual.points)
    assert n_bumper > 0
    piloto.virtual.prune_expired(1000.0, piloto.pp.virtual_ttl)
    assert len(piloto.virtual.points) == n_bumper
