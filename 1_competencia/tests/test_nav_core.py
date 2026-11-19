"""Pruebas de nav_core sin robot: scans sinteticos (trazado de rayos contra segmentos) y,
si existen, los scans reales grabados en 1_competencia/scans/*.npz.

Correr desde la raiz del repo:  python3 -m pytest 1_competencia/tests -v
"""
import glob
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import nav_core as nc  # noqa: E402
from sim import box, corridor, raycast  # noqa: E402,F401

P = nc.NavParams()
DEG = math.radians(1)


# --- percepcion -------------------------------------------------------------

def test_empty_scan_everything_free():
    r, a = nc.clean_scan([], [])
    assert np.all(nc.free_distance(r, a, nc.headings()) == P.max_range)
    assert nc.choose_heading(r, a, goal=0.3) == pytest.approx(0.3, abs=5 * DEG)


def test_clean_scan_drops_invalid():
    r, a = nc.clean_scan([1.0, np.nan, np.inf, 0.0, 2.0], [0, 0.1, 0.2, 0.3, 4.0])
    assert list(r) == [1.0, 2.0]
    assert a[1] == pytest.approx(4.0 - 2 * math.pi)


def test_free_distance_box_ahead_is_inflated():
    r, a = raycast(box(1.2, 0.0, 0.2, 0.4))  # cara cercana en x = 1.1
    assert nc.free_distance(r, a, 0.0)[0] == pytest.approx(1.1 - P.inflate, abs=0.02)
    # hacia atras no hay nada
    assert nc.free_distance(r, a, math.pi)[0] == P.max_range


def test_moving_away_from_close_wall_is_allowed():
    r, a = raycast([((-2, -0.15), (2, -0.15))])  # pared a 15 cm a la derecha
    assert nc.free_distance(r, a, math.pi / 2)[0] == P.max_range  # alejarse: libre
    assert nc.free_distance(r, a, 0.0)[0] < 0.05                  # rozandola: bloqueado


# --- eleccion de rumbo ------------------------------------------------------

def test_obstacle_in_front_picks_a_free_side():
    r, a = raycast(box(1.0, 0.0, 0.3, 0.5))
    h = nc.choose_heading(r, a, goal=0.0)
    assert h is not None
    assert 10 * DEG < abs(h) < 90 * DEG
    # por ese rumbo de verdad se puede pasar
    assert nc.free_distance(r, a, h)[0] >= P.min_free


def test_obstacle_offset_left_goes_right():
    r, a = raycast(box(1.0, 0.15, 0.3, 0.5))
    assert nc.choose_heading(r, a, goal=0.0) < 0


def test_narrow_corridor_centered_goes_straight():
    r, a = raycast(corridor(0.45, -0.45))  # pasillo de 0.9 m, robot de 0.34 m
    assert nc.choose_heading(r, a, goal=0.0) == pytest.approx(0.0, abs=5 * DEG)


def test_corridor_off_center_drifts_to_center():
    r, a = raycast(corridor(0.6, -0.3))  # robot pegado a la pared derecha
    h = nc.choose_heading(r, a, goal=0.0)
    assert 0 < h < 30 * DEG  # hacia la izquierda (el centro), sin giro brusco


def test_goal_behind_wall_is_not_chosen():
    # pared perpendicular al rumbo de la meta (60 grados), a 0.4 m
    r, a = raycast([((0.633, 0.096), (-0.233, 0.596))])
    goal = math.radians(60)
    h = nc.choose_heading(r, a, goal=goal)
    assert nc.free_distance(r, a, h)[0] >= P.min_free
    assert nc.free_distance(r, a, goal)[0] < P.min_free


def test_prev_heading_breaks_ties():
    r, a = raycast(box(1.0, 0.0, 0.3, 0.5))  # simetrico: izquierda y derecha empatan
    left = nc.choose_heading(r, a, goal=0.0, prev=0.5)
    right = nc.choose_heading(r, a, goal=0.0, prev=-0.5)
    assert left > 0 > right


def test_boxed_in_returns_none_and_most_open_finds_exit():
    walls = [((x1, y1), (x2, y2)) for (x1, y1), (x2, y2) in box(0, 0, 0.7, 0.7)]
    r, a = raycast(walls)
    assert nc.choose_heading(r, a, goal=0.0) is None
    # con una salida hacia atras, most_open apunta hacia ella
    r, a = raycast([walls[0], walls[1], walls[2]])  # falta la pared de x = -0.35
    assert abs(nc.wrap(nc.most_open(r, a) - math.pi)) < 15 * DEG


# --- estructuras de datos y algoritmos optimizados ------------------------

def test_spatial_kdtree_2d_nearest_and_radius():
    pts = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 1.0],
        [2.0, 2.0],
        [5.0, 5.0]
    ])
    kd = nc.SpatialKDTree2D(pts)
    idx, dist = kd.nearest([0.1, 0.1])
    assert idx == 0
    assert dist == pytest.approx(math.hypot(0.1, 0.1))

    idx, dist = kd.nearest([2.1, 1.9])
    assert idx == 3
    assert dist == pytest.approx(math.hypot(0.1, -0.1))

    # query radius
    indices = kd.query_radius([0.0, 0.0], 1.1)
    assert set(indices) == {0, 1, 2}
    indices_small = kd.query_radius([0.0, 0.0], 0.5)
    assert set(indices_small) == {0}


def test_detect_concavity_in_u_trap():
    # En espacio abierto, no es trampa
    r_empty, a_empty = nc.clean_scan([], [])
    is_trap, _ = nc.detect_concavity(r_empty, a_empty, goal=0.0)
    assert not is_trap

    # En trampa en U (paredes a los lados y al frente a 0.5 m, pero libre atras/lateral)
    u_walls = [
        ((0.5, -0.4), (0.5, 0.4)),    # pared frontal a 0.5m
        ((-0.2, 0.4), (0.5, 0.4)),    # pared lateral izq
        ((-0.2, -0.4), (0.5, -0.4)),  # pared lateral der
    ]
    r_u, a_u = raycast(u_walls)
    is_trap, escape_h = nc.detect_concavity(r_u, a_u, goal=0.0)
    assert is_trap
    assert escape_h is not None and abs(escape_h) > math.radians(50)


def test_breadcrumb_graph_pruning_and_pick_waypoint():
    graph = nc.BreadcrumbGraph()
    # A = (0, 0), avanza por pasillo principal
    graph.append((0.5, 0.0))
    graph.append((1.0, 0.0))
    # se mete a un callejon muerto a la izquierda
    graph.append((1.0, 0.5))
    graph.append((1.0, 1.0))
    # descubre que esta cerrado y poda la rama del callejon (indices 3 y 4)
    graph.prune_branch(3, 5)

    assert graph.is_pruned(3)
    assert graph.is_pruned(4)
    assert not graph.is_pruned(2)

    # nearest_active debe ignorar los nodos podados
    best_idx, _ = graph.nearest_active((1.0, 0.9))
    assert best_idx == 2  # el nodo activo mas cercano

    # pick_waypoint no debe elegir ninguna de las migas podadas
    r_empty, a_empty = nc.clean_scan([], [])
    chosen = nc.pick_waypoint(graph, (1.0, 0.0, 0.0), r_empty, a_empty, upto=4)
    assert chosen not in (3, 4)


# --- control ----------------------------------------------------------------

def test_speed_stops_near_obstacle():
    assert nc.speed_command(0.0, P.stop_dist - 0.01, 0.3, 1.0)[0] == 0.0


def test_speed_full_when_far_and_aligned():
    assert nc.speed_command(0.0, 2.0, 0.3, 1.0) == (pytest.approx(0.3), 0.0)


def test_speed_ramps_between_stop_and_slow():
    mid = (P.stop_dist + P.slow_dist) / 2
    assert nc.speed_command(0.0, mid, 0.3, 1.0)[0] == pytest.approx(0.15)


def test_large_heading_turns_in_place_and_clamps():
    lin, ang = nc.speed_command(math.radians(120), 2.0, 0.3, 1.0)
    assert lin == 0.0 and ang == 1.0
    assert nc.speed_command(-math.radians(120), 2.0, 0.3, 1.0)[1] == -1.0


def test_goal_bearing():
    h, d = nc.goal_bearing((0.0, 0.0, 0.0), (0.0, 2.0))
    assert h == pytest.approx(math.pi / 2) and d == pytest.approx(2.0)
    # robot mirando a +y en (1, 1): el origen queda atras-izquierda... a 135 grados
    h, d = nc.goal_bearing((1.0, 1.0, math.pi / 2), (0.0, 0.0))
    assert h == pytest.approx(math.radians(135)) and d == pytest.approx(math.sqrt(2))


# --- scans reales grabados --------------------------------------------------

SCANS = sorted(glob.glob(os.path.join(os.path.dirname(HERE), "scans", "*.npz")))


@pytest.mark.skipif(not SCANS, reason="aun no hay scans grabados en 1_competencia/scans/")
@pytest.mark.parametrize("path", SCANS, ids=[os.path.basename(s) for s in SCANS])
def test_recorded_scan_runs(path):
    z = np.load(path)
    r, a = nc.clean_scan(z["ranges"], z["angles"])
    assert len(r) > 50, "scan casi vacio: revisa el LiDAR"
    h = nc.choose_heading(r, a, goal=0.0)
    if h is not None:
        assert nc.free_distance(r, a, h)[0] >= P.min_free


def test_mask_sector_ignora_solo_la_pieza_fija():
    import math
    ang = np.radians([180.0, 175.0, 120.0, 180.0, -178.0])
    r = np.array([0.20, 0.20, 0.20, 1.50, 0.20])
    r2, a2 = nc.mask_sector(r, ang, 180, 8, 0.25)
    # se van los de 180, 175 y -178 grados cerca; se quedan el de 120 grados y el lejano
    assert sorted(np.round(np.degrees(a2)).tolist()) == [120.0, 180.0]
    assert sorted(r2.tolist()) == [0.2, 1.5]


def _pared_lateral(dl, dr):
    pts = np.array([(x, y) for x in np.arange(-0.2, 3.0, 0.02) for y in (dl, -dr)])
    return np.hypot(pts[:, 0], pts[:, 1]), np.arctan2(pts[:, 1], pts[:, 0])


def test_pared_lateral_dentro_del_margen_no_bloquea_el_frente():
    # Regresion: pared de carton a 0.25 m (> radio real 0.17, < inflado 0.31) dejaba free=0 y heading=None
    p = nc.NavParams(safety_margin=0.14)
    r, a = _pared_lateral(0.25, 0.45)
    assert nc.free_distance(r, a, 0.0, p)[0] > 1.0
    assert nc.choose_heading(r, a, 0.0, 0.0, p) is not None


def test_pared_dentro_del_radio_real_si_bloquea():
    p = nc.NavParams(safety_margin=0.14)
    r, a = _pared_lateral(0.12, 0.45)
    assert nc.free_distance(r, a, 0.0, p)[0] < 0.2


def test_obstaculo_frontal_dentro_del_margen_sigue_frenando():
    p = nc.NavParams(safety_margin=0.14)
    assert nc.free_distance(np.array([0.25]), np.array([0.0]), 0.0, p)[0] < p.stop_dist
