"""
Banco de velocidad del modo directo (directo.py) en el simulador, en condiciones DURAS:
  - circuito del video de referencia (tests/pista_video.py) a 0.65 / 0.75 / 0.80 / 0.90 m
  - pasillo de 0.56 m medido en el primer intento real (test_directo.pista_angosta)
  - latencia del cmd_vel (DDS por Wi-Fi), LiDAR atrasado (ciclo de la Pi), ruido de rango,
    deriva de la odometria al girar
Un perfil es ACEPTABLE solo si en todos los casos: completa A->B->A, 0 choques, holgura
minima >= radio + 5 cm (el robot real cuenta choque a 0.20 m = radio + 3 cm) y termina a
< 0.35 m de A. No es un test de pytest (tarda); se corre a mano:

  python tests/bench_directo.py
  python tests/bench_directo.py --csv ../docs/bench_directo.csv
  python tests/bench_directo.py --perfil v_recto=0.30,v_estrecho=0.22

SIMULADO: no reemplaza probar en el robot (2_laboratorio/calibrar.py).
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import directo as dr  # noqa: E402
import mision as ms  # noqa: E402
import pista_video as pv  # noqa: E402
from camara_sim import ANCHO, ver  # noqa: E402
from comun import apply_params  # noqa: E402
from navegacion import Piloto  # noqa: E402
from sim import Sim  # noqa: E402
from test_directo import IZQ_X, IZQ_Y, pista_angosta  # noqa: E402

CONFIG = os.path.join(os.path.dirname(HERE), "config.json")
HOLGURA_MIN = 0.17 + 0.05

# (latencia cmd en ciclos, atraso del LiDAR en ciclos, ruido m, deriva de giro)
CONDICIONES = {"dura": (2, 3, 0.01, 0.03), "dura-": (2, 3, 0.01, -0.03)}


def escenarios():
    e = {f"video_{int(w * 100)}": pv.circuito(w) for w in (0.65, 0.75, 0.80, 0.90)}
    e["real_56"] = (pista_angosta(), [("der", (1.19, 0.0), IZQ_X), ("cara", (0.9, -2.5), IZQ_Y)])
    return e


def simular(paredes, objetos, ajustes, latencia, retardo, ruido, deriva, seed=0, t_max=400.0, dt=0.1):
    rng = np.random.default_rng(seed)
    sim = Sim(paredes, yaw_drift=deriva, scan_retardo=retardo)
    piloto, mp, dp = Piloto(), ms.MisionParams(), dr.DirectoParams()
    apply_params([piloto.nav, piloto.pp, mp, dp], CONFIG, [f"{k}={v}" for k, v in ajustes.items()])
    d = dr.PilotoDirecto(piloto, mp, dp)

    def scan():
        r, a = sim.scan()
        return (r + rng.normal(0.0, ruido, r.shape), a) if ruido else (r, a)

    d.iniciar(sim.odom, *scan())
    cola = [(0.0, 0.0)] * latencia
    t, t_cam, fase, t_b, t_estado = 0.0, -1e9, "ida", None, {}
    while t < t_max:
        r, a = scan()
        if fase == "ida":
            if t - t_cam >= mp.cam_periodo:
                t_cam = t
                senales, cara = ver(sim.true, objetos, paredes)
                d.ver_caras(t, cara)
                d.ver_senales(t, senales, ANCHO, sim.odom, r, a)
            lin, ang, info = d.paso(t, sim.odom, r, a)
            est = d.estado
            if info["accion"] == "llegada_b":
                fase, t_b = "vuelta", t
                piloto.pp.max_linear = dp.v_regreso
                piloto.volver()
                continue
        else:
            lin, ang, info = d.regreso(t, sim.odom, r, a)
            est = "regreso"
            if info["llegada"]:
                return dict(ok=True, t=t, t_b=t_b, true=sim.true, d=d, choques=sim.collisions,
                            holgura=sim.min_clear, t_estado=t_estado)
        t_estado[est] = t_estado.get(est, 0.0) + dt
        cola.append((lin, ang))
        lin, ang = cola.pop(0)
        sim.step(lin, ang, dt)
        t += dt
    return dict(ok=False, t=t, t_b=t_b, true=sim.true, d=d, choques=sim.collisions,
                holgura=sim.min_clear, t_estado=t_estado)


# Perfiles: base = config.json de guiado-directo. Cada uno cambia pocas cosas para ver su efecto.
PERFILES = {
    "base": {},
    "estrecho 0.20": dict(v_estrecho=0.20),
    "estrecho 0.20 cerca 0.14": dict(v_estrecho=0.20, v_cerca=0.14),
    "rapido A": dict(v_recto=0.30, v_regreso=0.30, v_acercar=0.22, v_estrecho=0.20, v_cerca=0.14),
    "rapido A + giro": dict(v_recto=0.30, v_regreso=0.30, v_acercar=0.22, v_estrecho=0.20,
                            v_cerca=0.14, w_giro=1.8),
    "rapido B": dict(v_recto=0.30, v_regreso=0.30, v_acercar=0.24, v_estrecho=0.24, v_cerca=0.16,
                     w_giro=1.8),
    "rapido C": dict(v_recto=0.30, v_regreso=0.30, v_acercar=0.24, v_estrecho=0.24, v_cerca=0.16,
                     w_giro=1.8, d_lento=0.9, max_accel=0.7),
    "rapido D": dict(v_recto=0.30, v_regreso=0.30, v_acercar=0.26, v_estrecho=0.26, v_cerca=0.18,
                     w_giro=2.0, k_giro=2.6, d_lento=0.8, max_accel=0.8),
}


def aceptable(f):
    return f["ok"] and f["choques"] == 0 and f["holgura"] >= HOLGURA_MIN and f["err_A"] < 0.35


def correr(nombre, ajustes, escs, condiciones):
    filas = []
    for en, (paredes, objetos) in escs.items():
        for cn, (lat, ret, ruido, deriva) in condiciones.items():
            r = simular(paredes, objetos, ajustes, lat, ret, ruido, deriva)
            te = r["t_estado"]
            filas.append(dict(perfil=nombre, escenario=en, condicion=cn, ok=r["ok"],
                              t=round(r["t"], 1), t_ida=round(r["t_b"] or 0.0, 1),
                              choques=r["choques"], holgura=round(r["holgura"], 3),
                              err_A=round(math.hypot(*r["true"][:2]), 2),
                              t_recto=round(te.get(dr.RECTO, 0), 1), t_girar=round(te.get(dr.GIRAR, 0), 1),
                              t_mirar=round(te.get(dr.MIRAR, 0), 1), t_regreso=round(te.get("regreso", 0), 1),
                              decisiones=len(r["d"].decisiones), callejones=r["d"].callejones))
    return filas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv")
    ap.add_argument("--perfil", help="k=v,k=v (corre solo este)")
    args = ap.parse_args()
    perfiles = PERFILES
    if args.perfil:
        perfiles = {args.perfil: dict(kv.split("=") for kv in args.perfil.split(","))}
    escs = escenarios()
    todas = []
    print(f"acepta: ok, 0 choques, holgura >= {HOLGURA_MIN:.2f} m, error en A < 0.35 m | "
          f"condiciones {CONDICIONES}")
    for nombre, aj in perfiles.items():
        t0 = time.time()
        filas = correr(nombre, aj, escs, CONDICIONES)
        todas += filas
        ok = all(aceptable(f) for f in filas)
        por = {}
        for f in filas:
            por.setdefault(f["escenario"], []).append(f)
        det = "  ".join(f"{k}={max(x['t'] for x in v):.0f}{'' if all(aceptable(x) for x in v) else '!'}"
                        for k, v in por.items())
        print(f"{nombre:26} {'SI' if ok else 'NO':3} holg_min={min(f['holgura'] for f in filas):.3f} "
              f"choques={sum(f['choques'] for f in filas)}  {det}  ({time.time() - t0:.0f}s)", flush=True)
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(todas[0]))
            w.writeheader()
            w.writerows(todas)
        print(f"csv: {args.csv}")


if __name__ == "__main__":
    main()
