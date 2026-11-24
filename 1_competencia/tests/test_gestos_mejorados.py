"""Gestos mejorados (M1, M2; M7 aparte) y garantias de seguridad frente al comportamiento
original del repo (incluido 13582b6: puno de paso no desarma, GRACIA ante parpadeos)."""
import argparse
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import banco_gestos as bg  # noqa: E402
import gestos_teleop as gt  # noqa: E402
from test_gestos import hand  # noqa: E402

MOVIMIENTO = ("like", "dislike", "pistola_izq", "pistola_der")


def puno_pulgar_encima():
    """Puno con el pulgar apoyado ENCIMA: apunta arriba pero pegado al indice."""
    lm = hand(thumb="in")
    lm[4] = (80, 128)
    return lm


# --- M1 -----------------------------------------------------------------------------

def test_m1_puno_con_pulgar_encima_ya_no_es_like():
    lm = puno_pulgar_encima()
    assert gt.classify_pose(lm) == "like"                                   # fallo original
    assert gt.classify_pose(lm, like_pulgar_min=gt.LIKE_PULGAR_MIN) == "puno"  # M1: parar


def test_m1_like_real_sigue_siendo_like():
    for s in (0.6, 1.0, 2.0):
        assert gt.classify_pose(hand(thumb="up", scale=s), gt.LIKE_PULGAR_MIN) == "like"


@pytest.mark.parametrize("lm", [hand(thumb="in"), hand(thumb="down"),
                                hand(("index",), index_dir=(1, 0)), hand(("index", "middle")),
                                hand(("index", "middle", "ring", "pinky"), thumb="side")])
def test_m1_no_cambia_las_demas_poses(lm):
    assert gt.classify_pose(lm, gt.LIKE_PULGAR_MIN) == gt.classify_pose(lm)


def test_m1_nunca_crea_un_comando_nuevo():
    """M1 solo puede convertir like en puno/dislike... nunca crear movimiento de la nada."""
    rnd = np.random.default_rng(1)
    for base in (hand(thumb="up"), puno_pulgar_encima(), hand(thumb="in"), hand(thumb="down")):
        for _ in range(500):
            lm = base + rnd.normal(0, 8, base.shape)
            antes, despues = gt.classify_pose(lm), gt.classify_pose(lm, gt.LIKE_PULGAR_MIN)
            if despues != antes:
                assert antes == "like" and despues in ("puno", "dislike")


def test_margenes_coinciden_con_m1():
    rnd = np.random.default_rng(2)
    for base in (hand(thumb="up"), puno_pulgar_encima(), hand(thumb="down"), hand(thumb="in")):
        for _ in range(400):
            lm = base + rnd.normal(0, 8, base.shape)
            m = bg.margenes(lm, gt.LIKE_PULGAR_MIN)
            assert bg.pose_de_margenes(m) == gt.classify_pose(lm, gt.LIKE_PULGAR_MIN)


# --- M2 -----------------------------------------------------------------------------

def primer_frame(ctrl, pose, fps, armed=True, n=200):
    """Indice del primer frame en que el control acepta la pose (armado)."""
    ctrl.armed = armed
    for i in range(n):
        d, ev = ctrl.update(pose, i / fps)
        if ctrl.held == pose:
            return i
    return None


@pytest.mark.parametrize("fps", [5, 10, 15, 17, 20, 30, 60])
def test_m2_moverse_nunca_es_mas_facil_que_antes(fps):
    for pose in MOVIMIENTO:
        antes = primer_frame(gt.ControlGestos(), pose, fps)
        despues = primer_frame(gt.nuevo_control(gt.ParamsGestos.mejorados()), pose, fps)
        assert despues >= antes
        assert despues / fps >= gt.T_MOVER - 1e-9


@pytest.mark.parametrize("fps", [5, 10, 15, 17, 20, 30, 60])
def test_m2_parar_nunca_es_mas_lento_que_antes(fps):
    antes = primer_frame(gt.ControlGestos(), "puno", fps)
    despues = primer_frame(gt.nuevo_control(gt.ParamsGestos.mejorados()), "puno", fps)
    assert despues <= antes
    assert despues >= gt.MIN_FRAMES_PARAR - 1


def test_m2_puno_gana_al_like_en_una_transicion():
    """like (aun no confirmado) -> puno: nunca llega a moverse; desarma tras PUNO_HOLD s de
    puno estable (13582b6), no antes."""
    c = gt.nuevo_control(gt.ParamsGestos.mejorados())
    c.armed = True
    t, dirs, evs = 0.0, [], []
    for pose in ["like"] * 4 + ["puno"] * 17:   # 4 frames de like a 17 fps < 0.30 s; puno 1 s
        d, e = c.update(pose, t)
        dirs.append(d)
        if e:
            evs.append((round(t, 3), e))
        t += 1 / 17
    assert set(dirs) == {(0, 0)} and not c.armed
    assert len(evs) == 1 and evs[0][1] == "desarmado"
    assert evs[0][0] - 4 / 17 >= gt.PUNO_HOLD          # no desarma antes de PUNO_HOLD


def test_m2_otra_pose_para_al_instante_y_un_parpadeo_no():
    """Puno / dislike / pistola frenan en el primer frame. 'nada' o sin mano 1 frame NO
    cortan el movimiento (GRACIA, 13582b6) y si siguen mas de GRACIA s, frena."""
    for otra in ("puno", "dislike", "pistola_izq", "palma", "victoria"):
        c = gt.nuevo_control(gt.ParamsGestos.mejorados())
        c.armed = True
        for i in range(20):
            c.update("like", i / 30)
        d, _ = c.update(otra, 20 / 30)
        assert d in ((0, 0), gt.POSES[otra][1]) and d != (1, 0), otra
    for blink in ("nada", None):
        c = gt.nuevo_control(gt.ParamsGestos.mejorados())
        c.armed = True
        for i in range(20):
            c.update("like", i / 30)
        assert c.update(blink, 20 / 30)[0] == (1, 0)
        dirs = [c.update(blink, i / 30)[0] for i in range(21, 30)]
        assert dirs[-1] == (0, 0)


def test_m2_un_cuadro_sin_mano_no_corta_el_movimiento():
    """Igual que test_gestos.test_un_cuadro_sin_mano_no_corta_el_movimiento pero con M2:
    el parpadeo no reinicia los T_MOVER s (si no, frenaria a tirones). A cada fps el
    mejorado hace EXACTAMENTE lo mismo que el original del repo ante el parpadeo."""
    def correr(c, fps):
        c.armed, t = True, 0.0
        for _ in range(int(fps)):
            c.update("like", t)
            t += 1 / fps
        dirs = []
        for pose in [None] + ["like"] * int(fps):
            dirs.append(c.update(pose, t)[0])
            t += 1 / fps
        return dirs
    for fps in (10, 15, 17, 20, 30, 60):
        antes = correr(gt.ControlGestos(), fps)
        despues = correr(gt.nuevo_control(gt.ParamsGestos.mejorados()), fps)
        assert despues == antes, (fps, antes, despues)
    assert all(d == (1, 0) for d in correr(gt.nuevo_control(gt.ParamsGestos.mejorados()), 30))


@pytest.mark.parametrize("fps", [5, 10, 15, 17, 20, 30, 60])
def test_m2_nunca_mueve_cuando_el_original_no_moveria(fps):
    """Secuencias al azar (con parpadeos y transiciones): en cada frame, si el control
    mejorado manda un movimiento, el original manda ESE mismo movimiento."""
    rnd = np.random.default_rng(int(fps))
    poses = list(gt.POSES) + [None]
    for _ in range(40):
        a, b = gt.ControlGestos(), gt.nuevo_control(gt.ParamsGestos.mejorados())
        p, t = "palma", 0.0
        for i in range(600):
            r = rnd.random()
            if r < 0.08:
                p = poses[rnd.integers(len(poses))]
            elif r < 0.12:
                p = None if rnd.random() < 0.5 else "nada"
            elif i < 40:
                p = "palma"
            da, _ = a.update(p, t)
            db, _ = b.update(p, t)
            if db != (0, 0):
                assert da == db, (fps, i, p, da, db)
            assert not (b.armed and not a.armed), (fps, i)
            t += 1 / fps


@pytest.mark.parametrize("fps", [5, 10, 15, 17, 20, 30, 60])
def test_m2_puno_de_paso_no_desarma_y_el_sostenido_si(fps):
    """13582b6 con M2: un puno de paso (<= 0.3 s) no desarma; uno sostenido desarma no
    mas tarde que el original."""
    def t_desarme(ctrl, n_puno):
        ctrl.armed, t = True, 0.0
        for pose in ["like"] * int(fps) + ["puno"] * n_puno + ["dislike"] * int(fps):
            _, e = ctrl.update(pose, t)
            if e == "desarmado":
                return t
            t += 1 / fps
        return None
    n_paso = max(1, int(0.3 * fps))
    assert t_desarme(gt.nuevo_control(gt.ParamsGestos.mejorados()), n_paso) is None
    n_largo = int(1.5 * fps)
    antes = t_desarme(gt.ControlGestos(), n_largo)
    despues = t_desarme(gt.nuevo_control(gt.ParamsGestos.mejorados()), n_largo)
    assert antes is not None and despues is not None and despues <= antes + 1e-9


def test_m1_m2_transicion_real_de_like_a_dislike_no_desarma():
    """Como test_gestos.test_transicion_real_de_like_a_dislike_no_desarma, con M1+M2: M1
    lee mas cuadros de la transicion como puno, pero siguen sin llegar a PUNO_HOLD."""
    from test_gestos import THUMB as T
    p = gt.ParamsGestos.mejorados()
    for dur in (0.3, 0.45):
        c = gt.nuevo_control(p)
        c.armed, t, evs = True, 0.0, []
        for _ in range(30):
            c.update(gt.classify_pose(hand(thumb="up"), p.like_pulgar_min), t)
            t += 1 / 30
        n = int(dur * 30)
        for k in range(n):
            f = k / (n - 1)
            T["tmp"] = tuple(np.array(T["up"]) * (1 - f) + np.array(T["down"]) * f)
            _, e = c.update(gt.classify_pose(hand(thumb="tmp"), p.like_pulgar_min), t)
            t += 1 / 30
            if e:
                evs.append(e)
        assert c.armed and evs == [], dur


def test_m2_armar_no_es_mas_facil():
    for fps in (10, 17, 30):
        def t_armado(ctrl):
            for i in range(400):
                _, ev = ctrl.update("palma", i / fps)
                if ev == "armado":
                    return i / fps
        assert t_armado(gt.nuevo_control(gt.ParamsGestos.mejorados())) >= t_armado(gt.ControlGestos())


def test_original_sigue_igual_con_parametros_por_defecto():
    a, b = gt.ControlGestos(), gt.nuevo_control(gt.ParamsGestos())
    rnd = np.random.default_rng(3)
    poses = list(gt.POSES) + [None]
    for i in range(3000):
        p = poses[rnd.integers(len(poses))] if rnd.random() < 0.3 else (poses[0] if i % 50 < 25 else "palma")
        assert a.update(p, i / 17) == b.update(p, i / 17)
        assert a.held == b.held and a.armed == b.armed


# --- M7 -----------------------------------------------------------------------------

def test_m7_fuera_de_cuadro():
    lm = hand(thumb="up")
    assert not gt.fuera_de_cuadro(lm, (480, 640, 3))
    lm2 = lm.copy()
    lm2[8] = (650, 100)
    assert gt.fuera_de_cuadro(lm2, (480, 640, 3))
    lm3 = lm.copy()
    lm3[4] = (10, -1)
    assert gt.fuera_de_cuadro(lm3, (480, 640, 3))


# --- opciones -----------------------------------------------------------------------

def args_de(lista):
    ap = argparse.ArgumentParser()
    gt.add_gesture_args(ap)
    return ap.parse_args(lista)


def test_por_defecto_comportamiento_original():
    assert gt.params_desde_args(args_de([])) == gt.ParamsGestos()
    assert not gt.ParamsGestos().activo()


def test_ajustes_sin_flag_se_ignoran(capsys):
    assert gt.params_desde_args(args_de(["--t-mover", "0.1"])) == gt.ParamsGestos()
    assert "se ignoran" in capsys.readouterr().out


def test_flag_activa_m1_m2_sin_m7_y_acepta_ajustes():
    p = gt.params_desde_args(args_de(["--gestos-mejorados"]))
    assert p == gt.ParamsGestos.mejorados() and p.activo() and not p.rechazar_parcial
    assert p.like_pulgar_min == gt.LIKE_PULGAR_MIN and p.t_mover == gt.T_MOVER
    p = gt.params_desde_args(args_de(["--gestos-mejorados", "--t-mover", "0.5"]))
    assert p.t_mover == 0.5 and p.like_pulgar_min == gt.LIKE_PULGAR_MIN


def test_indices_fps_submuestrea():
    import evaluar_gestos as ev
    ts = [i / 17 for i in range(171)]            # 10 s a 17 fps
    for fps in (15, 10):
        k = ev.indices_fps(ts, fps)
        assert abs(len(k) - 10 * fps) <= 2 and k == sorted(set(k))
    assert ev.indices_fps(ts, None) == list(range(171))


def test_m7_es_opcion_aparte_y_apagada():
    assert not gt.params_desde_args(args_de([])).rechazar_parcial
    p = gt.params_desde_args(args_de(["--rechazar-mano-parcial"]))
    assert p == gt.ParamsGestos(rechazar_parcial=True)          # solo M7, sin M1/M2
    p = gt.params_desde_args(args_de(["--gestos-mejorados", "--rechazar-mano-parcial"]))
    assert p.rechazar_parcial and p.like_pulgar_min == gt.LIKE_PULGAR_MIN
