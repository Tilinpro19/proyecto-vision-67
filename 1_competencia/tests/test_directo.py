"""Tarea A modo directo (directo.py) en el simulador con camara geometrica (camara_sim.py)."""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import directo as dr  # noqa: E402
import mision as ms  # noqa: E402
from camara_sim import ANCHO, ver  # noqa: E402
from comun import apply_params  # noqa: E402
from navegacion import Piloto  # noqa: E402
from senales_forma import Senal  # noqa: E402
from sim import Sim, scan_from  # noqa: E402
from test_mision_guiada import laberinto_t_y_esquina  # noqa: E402

CONFIG = os.path.join(ROOT, "config.json")
IZQ_X, IZQ_Y, DER_X, DER_Y = (-1.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.0, -1.0)  # normales


def pista_zigzag():
    """A=(0,0) mirando a +x. Pasillo de 1.2 m hasta la pared x=3.6, con un hueco lateral
    SIN flecha en x=1.2..2.2 (cuarto ciego). Sube por x=2.4..3.6 hasta y=3.6, dobla a la
    izquierda (-x) por y=2.4..3.6 y termina en un callejon en x=0 donde estan las caras."""
    return [
        ((-0.8, -0.6), (-0.8, 0.6)),
        ((-0.8, -0.6), (3.6, -0.6)),
        ((-0.8, 0.6), (1.2, 0.6)),
        ((1.2, 0.6), (1.2, 2.0)), ((1.2, 2.0), (2.2, 2.0)), ((2.2, 2.0), (2.2, 0.6)),  # cuarto ciego
        ((2.2, 0.6), (2.4, 0.6)),
        ((3.6, -0.6), (3.6, 3.6)),
        ((2.4, 0.6), (2.4, 2.4)),
        ((0.0, 3.6), (3.6, 3.6)),
        ((0.0, 2.4), (2.4, 2.4)),
        ((0.0, 2.4), (0.0, 3.6)),
    ]


FLECHAS_ZIGZAG = [("izq", (3.6, 0.0), IZQ_X), ("izq", (3.0, 3.6), DER_Y)]
CARAS_ZIGZAG = [("cara", (0.0, 3.0), DER_X)]
LAB_FLECHAS = [("izq", (2.6, 0.0), IZQ_X), ("der", (2.0, 2.8), DER_Y)]
LAB_META = [("bandera", (4.4, 2.3), IZQ_X), ("cara", (4.4, 2.1), IZQ_X)]


def _piloto_config():
    piloto, mp, dp = Piloto(), ms.MisionParams(), dr.DirectoParams()
    apply_params([piloto.nav, piloto.pp, mp, dp], CONFIG)
    return piloto, mp, dp


def simular_directo(paredes, objetos, t_max=200.0, dt=0.1, yaw_drift=0.0, dp_extra=None):
    sim = Sim(paredes, yaw_drift=yaw_drift)
    piloto, mp, dp = _piloto_config()
    for k, v in (dp_extra or {}).items():
        setattr(dp, k, v)
    d = dr.PilotoDirecto(piloto, mp, dp)
    d.iniciar(sim.odom, *sim.scan())
    t, t_cam, fase, t_b = 0.0, -1e9, "ida", None
    while t < t_max:
        r, a = sim.scan()
        if fase == "ida":
            if t - t_cam >= mp.cam_periodo:
                t_cam = t
                senales, cara = ver(sim.true, objetos, paredes)
                d.ver_caras(t, cara)
                d.ver_senales(t, senales, ANCHO, sim.odom, r, a)
            lin, ang, info = d.paso(t, sim.odom, r, a)
            if info["accion"] == "llegada_b":
                fase, t_b, b_true = "vuelta", t, sim.true
                piloto.pp.max_linear = dp.v_regreso
                piloto.volver()
                continue
        else:
            lin, ang, info = piloto.paso(t, sim.odom, r, a)
            if info["llegada"]:
                return dict(ok=True, t=t, t_b=t_b, b_true=b_true, true=sim.true, d=d,
                            collisions=sim.collisions, min_clear=sim.min_clear)
        sim.step(lin, ang, dt)
        t += dt
    return dict(ok=False, t=t, t_b=t_b, true=sim.true, d=d,
                collisions=sim.collisions, min_clear=sim.min_clear)


def simular_guiado(paredes, objetos, t_max=200.0, dt=0.1):
    """El modo guiado actual con la misma camara, para comparar tiempos."""
    sim = Sim(paredes)
    piloto, mp, _ = _piloto_config()
    m = ms.Mision("A", quiet=True)
    g = ms.PilotoGuiado(piloto, mp)
    g.iniciar(sim.odom, *sim.scan())
    m.ir(ms.SEGUIR)
    t, t_cam, fase, t_b = 0.0, -1e9, "ida", None
    while t < t_max:
        r, a = sim.scan()
        if fase == "ida":
            llego = False
            if t - t_cam >= mp.cam_periodo:
                t_cam = t
                # bandera solo de cerca (1.3 m: el guiado decide "callejon" a ~1.1 m del fondo); si no,
                # "llega" a 2.4 m de B y escanearia las caras desde ahi
                vistas = [s for s in ver(sim.true, objetos, paredes)[0]
                          if s.tipo != "bandera" or s.distancia_m <= 1.3]
                llego = g.registrar_senales(t, vistas, m)
            lin, ang, info = g.paso(t, sim.odom, r, a, mision=m)
            if llego or info.get("llegada_b"):
                fase, t_b = "vuelta", t
                piloto.volver()
                continue
        else:
            lin, ang, info = piloto.paso(t, sim.odom, r, a)
            if info["llegada"]:
                return dict(ok=True, t=t, t_b=t_b, collisions=sim.collisions)
        sim.step(lin, ang, dt)
        t += dt
    return dict(ok=False, t=t, t_b=t_b, collisions=sim.collisions)


def _en_a(res):
    return math.hypot(res["true"][0], res["true"][1]) < 0.35


# --- funciones puras ------------------------------------------------------------------

def test_rumbo_en_imagen_centro_y_bordes():
    assert dr.rumbo_en_imagen(320, 640, 69.0) == 0.0
    assert math.isclose(dr.rumbo_en_imagen(0, 640, 69.0), math.radians(34.5), rel_tol=1e-6)
    assert dr.rumbo_en_imagen(600, 640, 69.0) < 0     # derecha de la imagen = derecha del robot


def test_camara_sim_respeta_linea_de_vista_y_orientacion():
    paredes = pista_zigzag()
    # desde A se ve la flecha del fondo (3.6 m > alcance) -> no; desde x=1.5 si
    assert ver((0.0, 0.0, 0.0), FLECHAS_ZIGZAG, paredes)[0] == []
    vistas = ver((1.5, 0.0, 0.0), FLECHAS_ZIGZAG, paredes)[0]
    assert [s.tipo for s in vistas] == ["izq"]
    # de espaldas no se ve
    assert ver((1.5, 0.0, math.pi), FLECHAS_ZIGZAG, paredes)[0] == []


def test_flecha_se_ubica_en_el_mundo_con_el_lidar():
    paredes = pista_zigzag()
    piloto, mp, dp = _piloto_config()
    d = dr.PilotoDirecto(piloto, mp, dp)
    pose = (1.5, 0.0, 0.0)
    r, a = scan_from(paredes, pose)
    d.iniciar(pose, r, a)
    for k in range(dp.confirmar):
        d.ver_senales(k * 0.2, ver(pose, FLECHAS_ZIGZAG, paredes)[0], ANCHO, pose, r, a)
    assert d.pendiente is not None and d.pendiente["tipo"] == "izq"
    # en el marco de A (origen en pose): la flecha esta 2.1 m al frente
    assert abs(d.pendiente["xy"][0] - 2.1) < 0.1 and abs(d.pendiente["xy"][1]) < 0.1


def test_una_sola_lectura_no_confirma_la_flecha():
    paredes = pista_zigzag()
    piloto, mp, dp = _piloto_config()
    d = dr.PilotoDirecto(piloto, mp, dp)
    pose = (1.5, 0.0, 0.0)
    r, a = scan_from(paredes, pose)
    d.iniciar(pose, r, a)
    d.ver_senales(0.0, [Senal("izq", 0.9, (300, 200, 30, 30))], ANCHO, pose, r, a)
    d.ver_senales(0.2, [Senal("der", 0.9, (300, 200, 30, 30))], ANCHO, pose, r, a)
    assert d.pendiente is None


# --- misiones completas ---------------------------------------------------------------

def test_zigzag_con_flechas_ignora_hueco_y_llega_a_las_caras():
    res = simular_directo(pista_zigzag(), FLECHAS_ZIGZAG + CARAS_ZIGZAG)
    assert res["ok"], res["d"].eventos
    assert res["collisions"] == 0
    assert _en_a(res)
    d = res["d"]
    # B esta en el callejon de las caras (x ~ 0.7, y ~ 3.0)
    assert res["b_true"][0] < 1.2 and abs(res["b_true"][1] - 3.0) < 0.4
    # exactamente 2 giros a la izquierda; el cuarto ciego (sin flecha) no se visito
    assert [x["accion"] for x in d.decisiones] == ["izq", "izq"]
    assert all(x["motivo"] == "flecha" for x in d.decisiones)
    assert d.motivo_llegada == "caras al frente"


def test_zigzag_sin_flechas_dobla_esquinas_con_el_lidar():
    res = simular_directo(pista_zigzag(), CARAS_ZIGZAG)
    assert res["ok"], res["d"].eventos
    assert res["collisions"] == 0
    assert _en_a(res)
    assert [x["accion"] for x in res["d"].decisiones] == ["izq", "izq"]
    assert all(x["motivo"] == "frente" for x in res["d"].decisiones)


def test_laberinto_con_bandera_llega_cerca_de_b():
    res = simular_directo(laberinto_t_y_esquina(), LAB_FLECHAS + LAB_META)
    assert res["ok"], res["d"].eventos
    assert res["collisions"] == 0
    assert _en_a(res)
    assert res["b_true"][0] > 3.2           # no "llega" a 2 m de la bandera
    assert [x["accion"] for x in res["d"].decisiones] == ["izq", "der"]


def test_callejon_sin_caras_da_media_vuelta_y_sigue():
    # la flecha de la T manda al callejon derecho (cerrado, sin caras)
    objetos = [("der", (2.6, 0.0), IZQ_X), ("der", (2.0, 2.8), DER_Y)] + LAB_META
    res = simular_directo(laberinto_t_y_esquina(), objetos, t_max=240.0)
    assert res["ok"], res["d"].eventos
    assert res["collisions"] == 0
    assert _en_a(res)
    assert res["b_true"][0] > 3.2
    assert res["d"].callejones == 1
    assert any("media vuelta" in e for e in res["d"].eventos)


def test_prohibido_al_fondo_toma_la_salida_lateral():
    # T: el pasillo recto sigue hasta x=4.6 (callejon con "prohibido" en el fondo); la salida
    # a la izquierda (x=2.0..3.2) lleva a las caras. Debe doblar SIN entrar al recto.
    paredes = [
        ((-0.8, -0.6), (-0.8, 0.6)), ((-0.8, -0.6), (4.6, -0.6)), ((4.6, -0.6), (4.6, 0.6)),
        ((-0.8, 0.6), (2.0, 0.6)), ((3.2, 0.6), (4.6, 0.6)),
        ((2.0, 0.6), (2.0, 3.0)), ((3.2, 0.6), (3.2, 3.0)), ((2.0, 3.0), (3.2, 3.0)),
    ]
    objetos = [("prohibido", (4.6, 0.0), IZQ_X), ("cara", (2.6, 3.0), DER_Y)]
    res = simular_directo(paredes, objetos)
    assert res["ok"], res["d"].eventos
    assert res["collisions"] == 0
    assert _en_a(res)
    dec = res["d"].decisiones[0]
    assert dec["senal"] == "prohibido" and dec["accion"] == "izq"
    assert 2.2 < dec["pose"][0] < 3.0          # dobla frente al hueco, no al asomar su borde
    assert res["d"].callejones == 0


def test_prohibido_en_la_unica_salida_da_media_vuelta():
    # Politica de decision.py: "prohibido" anula la unica salida -> media vuelta sin MIRAR
    paredes = [((-0.8, -0.6), (-0.8, 0.6)), ((-0.8, -0.6), (3.2, -0.6)), ((3.2, -0.6), (3.2, 0.6)),
               ((-0.8, 0.6), (2.0, 0.6)), ((2.0, 0.6), (2.0, 3.0)), ((3.2, 0.6), (3.2, 3.0)),
               ((2.0, 3.0), (3.2, 3.0))]
    piloto, mp, dp = _piloto_config()
    d = dr.PilotoDirecto(piloto, mp, dp)
    sim = Sim(paredes)
    d.iniciar(sim.odom, *sim.scan())
    t = 0.0
    while t < 40.0 and not d.decisiones:
        r, a = sim.scan()
        d.ver_senales(t, ver(sim.true, [("prohibido", (3.2, 0.0), IZQ_X)], paredes)[0],
                      ANCHO, sim.odom, r, a)
        lin, ang, _ = d.paso(t, sim.odom, r, a)
        sim.step(lin, ang, 0.1)
        t += 0.1
    assert d.decisiones and d.decisiones[0]["senal"] == "prohibido"
    assert d.estado == dr.GIRAR and math.isclose(abs(d.rumbo_obj), math.pi, abs_tol=1e-6)
    assert sim.collisions == 0


def test_con_deriva_de_odometria_vuelve_a_a():
    res = simular_directo(pista_zigzag(), FLECHAS_ZIGZAG + CARAS_ZIGZAG, yaw_drift=0.03)
    assert res["ok"], res["d"].eventos
    assert res["collisions"] == 0
    assert _en_a(res)


def test_comparacion_con_el_modo_guiado():
    """La mejora que importa: con huecos laterales sin flecha el guiado los explora (y aqui
    se pierde); el directo los ignora. En un laberinto chico sin desvios quedan parejos."""
    zz = FLECHAS_ZIGZAG + CARAS_ZIGZAG + [("bandera", (0.0, 2.8), DER_X)]
    g_zz = simular_guiado(pista_zigzag(), zz, t_max=200.0)
    d_zz = simular_directo(pista_zigzag(), zz, t_max=200.0)
    lab = LAB_FLECHAS + LAB_META
    g_lab = simular_guiado(laberinto_t_y_esquina(), lab)
    d_lab = simular_directo(laberinto_t_y_esquina(), lab)
    print(f"\n  zigzag: guiado {'ok' if g_zz['ok'] else 'NO TERMINA'} {g_zz['t']:.1f} s | "
          f"directo {d_zz['t']:.1f} s")
    print(f"  laberinto: guiado {g_lab['t']:.1f} s | directo {d_lab['t']:.1f} s")
    assert d_zz["ok"] and d_zz["collisions"] == 0
    assert (not g_zz["ok"]) or d_zz["t"] < 0.8 * g_zz["t"]
    assert g_lab["ok"] and d_lab["ok"] and d_lab["collisions"] == 0
    assert d_lab["t"] < 1.05 * g_lab["t"]


def test_giro_en_el_sitio_es_cardinal_y_no_acelera_de_golpe():
    paredes = pista_zigzag()
    piloto, mp, dp = _piloto_config()
    d = dr.PilotoDirecto(piloto, mp, dp)
    sim = Sim(paredes, start=(3.0, 0.0, 0.0))
    d.iniciar(sim.odom, *sim.scan())
    d._girar_a(math.radians(88))     # pide casi 90: se redondea al cardinal
    assert math.isclose(d.rumbo_obj, math.pi / 2, abs_tol=1e-9)
    t, lins = 0.0, []
    for _ in range(80):
        lin, ang, _ = d.paso(t, sim.odom, *sim.scan())
        lins.append(lin)
        sim.step(lin, ang, 0.1)
        t += 0.1
    assert d.estado == dr.RECTO
    assert abs(nc_wrap(sim.true[2] - math.pi / 2)) < math.radians(12)
    assert max(np.diff([0.0] + lins)) <= piloto.pp.max_accel * 0.1 + 1e-6


def nc_wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi
