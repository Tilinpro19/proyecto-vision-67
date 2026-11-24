"""Banco de evaluacion de gestos: margenes, simulacion del control, metricas y E/S."""
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import banco_gestos as bg  # noqa: E402
import gestos_teleop as gt  # noqa: E402
from test_gestos import hand  # noqa: E402

MANOS = [hand(thumb="in"), hand(thumb="up"), hand(thumb="down"),
         hand(("index",), index_dir=(1, 0)), hand(("index",), index_dir=(-1, 0)),
         hand(("index", "middle")), hand(("index", "middle", "ring", "pinky"), thumb="side"),
         hand(("index", "middle", "ring")), hand(thumb="up", scale=0.3)]


@pytest.mark.parametrize("lm", MANOS)
def test_margenes_eligen_la_misma_pose(lm):
    m = bg.margenes(lm)
    pose = gt.classify_pose(lm)
    assert bg.pose_de_margenes(m) == pose
    assert sum(m[p] > 0 for p in bg.POSES_REALES) == (0 if pose == "nada" else 1)


def test_margenes_coinciden_con_manos_ruidosas():
    """Miles de manos deformadas al azar: el margen nunca contradice a classify_pose."""
    rnd = np.random.default_rng(0)
    n = 0
    for base in MANOS:
        for _ in range(400):
            lm = base + rnd.normal(0, 8, base.shape)
            if bg.pose_de_margenes(bg.margenes(lm)) != gt.classify_pose(lm):
                n += 1
    assert n == 0


def test_margen_crece_al_alejarse_del_umbral():
    flojo = hand(thumb="up")
    flojo[4] = (60, 140)   # pulgar apenas arriba
    _, m_flojo, _, _ = bg.margen_y_rival(bg.margenes(flojo))
    pose, m_firme, rival, _ = bg.margen_y_rival(bg.margenes(hand(thumb="up")))
    assert pose == "like" and m_firme > m_flojo and rival in gt.POSES


def ts(n, dt=1 / 30):
    return [i * dt for i in range(n)]


def test_simular_control_comando_armado_y_eventos():
    # puno 25 frames = 0.83 s a 30 fps: desarma (PUNO_HOLD 0.5 s desde que es estable)
    poses = ["like"] * 10 + ["puno"] * 25 + ["palma"] * 40 + ["victoria"] * 60
    held, dirs, ev = bg.simular_control(poses, ts(len(poses)))
    assert dirs[1] == (0, 0) and dirs[5] == (1, 0)          # espera STABLE_FRAMES
    assert all(d == (0, 0) for d in dirs[10:35])             # frena desde el 1er cuadro de puno
    assert sum("desarmar" in e for e in ev) == 1             # un solo arranque de puno
    assert sum("armar" in e for e in ev) == 1                # palma >= 1 s
    assert sum("accion" in e for e in ev) == 1               # V >= 1.5 s
    assert held[-1] == "victoria"


def test_simular_control_puno_de_paso_no_desarma():
    """13582b6: un puno de paso (~0.2 s, like -> dislike) no desarma; el banco lo cuenta igual."""
    poses = ["like"] * 10 + ["puno"] * 6 + ["dislike"] * 20
    _, dirs, ev = bg.simular_control(poses, ts(len(poses)))
    assert not any("desarmar" in e for e in ev)
    assert all(d == (0, 0) for d in dirs[10:16]) and dirs[-1] == (-1, 0)


def frames(poses, objetivo=None):
    held, dirs, ev = bg.simular_control(poses, ts(len(poses)))
    return [dict(t=t, cruda=p, estable=h, dir=d, eventos=e, margen=None, rival=None,
                 objetivo=(objetivo[i] if objetivo else None))
            for i, (t, p, h, d, e) in enumerate(zip(ts(len(poses)), poses, held, dirs, ev))]


def test_metricas_ninguno_cuenta_arranques_falsos():
    poses = [None] * 30 + ["like"] * 4 + ["nada"] * 30 + ["pistola_izq"] * 5 + [None] * 31
    m = bg.metricas_ninguno(frames(poses))
    assert m["arranques"] == 2 and m["poses_falsas"] == {"like": 2, "pistola_izq": 3}


def test_metricas_gesto_detecta_peligro():
    poses = ["like"] * 20 + ["pistola_der"] * 5 + ["like"] * 20
    m = bg.metricas_gesto(frames(poses), "like")
    assert m["arranques_peligro"] == 1 and 0 < m["cmd_peligro"] < 0.1
    assert m["latencia"] == pytest.approx(2 / 30)


def test_metricas_transiciones_latencia_y_ajenos():
    obj = ["like"] * 60 + ["puno"] * 60
    poses = ["like"] * 62 + ["pistola_izq"] * 4 + ["puno"] * 54   # pasa por pistola
    m = bg.metricas_transiciones(frames(poses, obj))
    c = m["cambios"][0]
    assert (c["de"], c["a"]) == ("like", "puno")
    assert c["arranques_ajenos"] == 1
    assert c["latencia"] == pytest.approx(8 / 30)


def test_clipwriter_escribe_y_descarta(tmp_path):
    pytest.importorskip("cv2")
    from grabar_gestos import ClipWriter
    w = ClipWriter(str(tmp_path), "like")
    f = np.zeros((48, 64, 3), np.uint8)
    for i in range(5):
        w.write(f, i / 30)
    stem = w.close(dict(variante="v"))
    meta = json.load(open(stem + ".json"))
    assert meta["etiqueta"] == "like" and meta["frames"] == 5 and "objetivo" not in meta
    assert os.path.getsize(stem + ".avi") > 0
    w2 = ClipWriter(str(tmp_path), "transiciones")
    w2.write(f, 0.0, "like")
    w2.discard()
    assert not os.path.exists(w2.stem + ".avi") and not os.path.exists(w2.stem + ".json")


def test_secuencia_transiciones_sin_repetidos():
    from grabar_gestos import secuencia_transiciones
    s = secuencia_transiciones(30, 3)
    assert len(s) == 31 and all(a != b for a, b in zip(s, s[1:]))
    assert s == secuencia_transiciones(30, 3)
