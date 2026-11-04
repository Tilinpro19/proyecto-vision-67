"""Poses y armado por gestos (Tarea B) con manos sinteticas de 21 puntos."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import gestos_teleop as gt  # noqa: E402

MCP = {"index": (85, 140), "middle": (100, 135), "ring": (115, 140), "pinky": (128, 148)}
IDS = {"index": (5, 6, 7, 8), "middle": (9, 10, 11, 12), "ring": (13, 14, 15, 16),
       "pinky": (17, 18, 19, 20)}
THUMB = {"in": (90, 160), "up": (55, 100), "down": (65, 240), "side": (30, 160)}


def hand(extended=(), thumb="in", index_dir=(0, -1), scale=1.0):
    """Mano erguida (y crece hacia abajo). index_dir: hacia donde apunta el indice."""
    lm = np.zeros((21, 2), np.float32)
    lm[0] = (100, 200)
    lm[1], lm[2], lm[3], lm[4] = (80, 190), (65, 175), (55, 165), THUMB[thumb]
    for name, (m, pip, dip, tip) in IDS.items():
        base = np.array(MCP[name], float)
        d = np.array(index_dir if name == "index" else (0, -1), float)
        lm[m] = base
        if name in extended:
            lm[pip], lm[dip], lm[tip] = base + 25 * d, base + 40 * d, base + 55 * d
        else:  # doblado hacia la palma
            lm[pip], lm[dip], lm[tip] = base + (0, -15), base + (0, 0), base + (0, 20)
    return (lm - lm[0]) * scale + lm[0]


@pytest.mark.parametrize("lm, pose", [
    (hand(thumb="in"), "puno"),
    (hand(thumb="up"), "like"),
    (hand(thumb="down"), "dislike"),
    (hand(("index",), index_dir=(1, 0)), "pistola_der"),
    (hand(("index",), index_dir=(-1, 0)), "pistola_izq"),
    (hand(("index", "middle")), "victoria"),
    (hand(("index", "middle", "ring", "pinky"), thumb="side"), "palma"),
    (hand(("index", "middle", "ring")), "nada"),
])
def test_classify_pose(lm, pose):
    assert gt.classify_pose(lm) == pose


def test_mano_muy_lejos_no_cuenta():
    assert gt.classify_pose(hand(thumb="up", scale=0.3)) == "nada"


def run(ctrl, seq, dt=1 / 30):
    """seq: [(pose, segundos)] -> (direcciones, eventos)."""
    t, dirs, evs = 0.0, [], []
    for pose, dur in seq:
        for _ in range(int(round(dur / dt))):
            d, e = ctrl.update(pose, t)
            dirs.append(d)
            if e:
                evs.append(e)
            t += dt
    return dirs, evs


def test_arranca_desarmado_y_no_se_mueve():
    c = gt.ControlGestos()
    dirs, evs = run(c, [("like", 2.0), ("pistola_izq", 1.0)])
    assert set(dirs) == {(0, 0)} and evs == []


def test_palma_corta_no_arma_y_larga_si():
    c = gt.ControlGestos()
    _, evs = run(c, [("palma", 0.5), (None, 0.3)])
    assert not c.armed and evs == []
    _, evs = run(c, [("palma", 1.3)])
    assert c.armed and evs == ["armado"]


def test_armado_mueve_y_sin_mano_para():
    c = gt.ControlGestos()
    run(c, [("palma", 1.3)])
    dirs, _ = run(c, [("like", 0.5)])
    assert dirs[-1] == (1, 0) and dirs[0] == (0, 0)   # espera STABLE_FRAMES
    dirs, _ = run(c, [(None, 0.2)])
    assert dirs[-1] == (0, 0) and c.armed


def test_puno_desarma():
    c = gt.ControlGestos()
    run(c, [("palma", 1.3), ("like", 0.5)])
    dirs, evs = run(c, [("puno", 0.3), ("like", 0.5)])
    assert evs == ["desarmado"] and dirs[-1] == (0, 0)


def test_victoria_dispara_una_sola_vez():
    c = gt.ControlGestos()
    run(c, [("palma", 1.3)])
    _, evs = run(c, [("victoria", 4.0)])
    assert evs == ["accion"]
    _, evs = run(c, [("nada", 0.3), ("victoria", 1.8)])
    assert evs == ["accion"]


def test_victoria_desarmado_no_dispara():
    _, evs = run(gt.ControlGestos(), [("victoria", 3.0)])
    assert evs == []
