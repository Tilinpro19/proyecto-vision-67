"""Graba corridas del modo directo (pose real, odometria, LiDAR, estado, eventos) en un JSON
para el reproductor web (sim.json).

    python tests/exportar_simulacion.py salida/sim.json
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE)]

import numpy as np  # noqa: E402

import directo as dr  # noqa: E402
import test_directo as T  # noqa: E402
from ver_simulacion import PISTAS  # noqa: E402

RAYOS = 90      # LiDAR reducido a 1 rayo cada 4 grados (sim.json < 16 MB)

ESCENARIOS = [
    ("zigzag", "Zigzag con flechas", "zigzag", {}),
    ("zigzag_sin", "Zigzag sin flechas (solo LiDAR)", "zigzag_sin_flechas", {}),
    ("laberinto", "Laberinto con bandera", "laberinto", {}),
    ("callejon", "Callejón sin caras y media vuelta", "callejon", {}),
    ("lateral", "Cruce sin flecha", "lateral", {}),
    ("angosta", "Pasillo angosto 0.57 m", "angosta", {}),
    ("angosta_deriva", "Angosto con 6 % de deriva", "angosta", dict(deriva=0.06)),
    ("angosta_sin_corr", "Angosto, 6 % de deriva, sin corregir con paredes", "angosta",
     dict(deriva=0.06, extra=dict(rumbo_paredes=False))),
    ("lateral_retardo", "Cruce con LiDAR atrasado 0.3 s a 0.30 m/s", "lateral",
     dict(deriva=-0.04, retardo=3, extra=dict(v_recto=0.30, v_regreso=0.30))),
]


def grabar(pista, deriva=0.0, retardo=0, extra=None):
    paredes, objetos, t_max = PISTAS[pista]()
    return grabar_con(paredes, objetos, retardo, deriva, lambda: T.simular_directo(
        paredes, objetos, t_max=t_max, yaw_drift=deriva, scan_retardo=retardo, dp_extra=extra))


def grabar_con(paredes, objetos, retardo, deriva, correr):
    """Graba la corrida que hace `correr()` (usa test_directo.Sim o bench_directo.Sim)."""
    import bench_directo as bd
    frames, scans, eventos = [], [], []
    ctx = {"d": None, "fase": "ida", "n_ev": 0, "t": 0.0}
    clase = T.Sim

    class SimGrabada(clase):
        def step(self, lin, ang, dt):
            d = ctx["d"]
            if d is not None:
                p = d.piloto.pose(self.odom)
                if ctx["fase"] == "ida":
                    est = d.estado
                else:
                    est = "regreso " + str(d.modo_regreso or "")
                    if d.modo_regreso == "tramos" and d._vuelta:
                        est += " · " + d._vuelta["estado"]
                for e in d.eventos[ctx["n_ev"]:]:
                    eventos.append([round(ctx["t"], 1), e])
                ctx["n_ev"] = len(d.eventos)
                r, a = self.scan()
                idx = np.linspace(0, len(r) - 1, RAYOS).astype(int)
                rr = np.where(np.isfinite(r[idx]), np.minimum(r[idx], 4.0), 4.0)
                scans.append([round(float(v), 2) for v in rr])
                if not frames:
                    ctx["ang0"] = [round(float(v), 4) for v in a[idx]]
                x, y, th = self._hist[0] if retardo else self.true
                frames.append([round(ctx["t"], 1), *(round(float(v), 3) for v in self.true),
                               *(round(float(v), 3) for v in p), round(float(lin), 3),
                               round(float(ang), 3), ctx["fase"], est,
                               *(round(float(v), 3) for v in (x, y, th))])
            ctx["t"] += dt
            super().step(lin, ang, dt)

    o_reg, o_ini = dr.PilotoDirecto.regreso, dr.PilotoDirecto.iniciar

    def regreso(self, *a, **k):
        ctx["fase"] = "vuelta"
        return o_reg(self, *a, **k)

    def iniciar(self, *a, **k):
        ctx["d"] = self
        return o_ini(self, *a, **k)

    T.Sim, bd.Sim, dr.PilotoDirecto.regreso, dr.PilotoDirecto.iniciar = SimGrabada, SimGrabada, regreso, iniciar
    try:
        res = correr()
    finally:
        T.Sim, bd.Sim, dr.PilotoDirecto.regreso, dr.PilotoDirecto.iniciar = clase, clase, o_reg, o_ini
    res.setdefault("collisions", res.get("choques", 0))
    res.setdefault("min_clear", res.get("holgura", 0.0))
    d = res["d"]
    ok = bool(res["ok"] and T._en_a(res) and res["collisions"] == 0)
    return dict(
        paredes=[[list(p0), list(p1)] for p0, p1 in paredes],
        objetos=[[tipo, list(xy)] for tipo, xy, _ in objetos],
        qrs=list(getattr(d, "qrs", [])),
        angulos=ctx.get("ang0", []),
        frames=frames, scans=scans, eventos=eventos,
        decisiones=[dict(t=round(x["t"], 1), x=round(x["pose"][0], 2), y=round(x["pose"][1], 2),
                         accion=x["accion"], motivo=x.get("motivo")) for x in d.decisiones],
        resumen=dict(ok=ok, t=round(res["t"], 1), t_b=round(res["t_b"] or 0, 1),
                     choques=res["collisions"], min_pared=round(res["min_clear"], 3),
                     dist_a=round(math.hypot(*res["true"][:2]), 2),
                     deriva=deriva, retardo=retardo * 0.1),
    )


def _perfil(nombre):
    with open(os.path.join(os.path.dirname(HERE), nombre), encoding="utf-8") as f:
        return {k: v for k, v in json.load(f).items() if not k.startswith("_")}


def grabar_bench(paredes, objetos, perfil, deriva=0.03, retardo=3, cam="real"):
    """Como tests/bench_directo.py: latencia 0.2 s, LiDAR atrasado, ruido y camara del robot."""
    import bench_directo as bd
    camara = bd.CAM_REAL if cam == "real" else bd.CAM_IDEAL
    return grabar_con(paredes, objetos, retardo, deriva, lambda: bd.simular(
        paredes, objetos, _perfil(perfil), latencia=2, retardo=retardo, ruido=0.01, deriva=deriva,
        cam=camara))


def nuevas():
    """Corridas del 2026-11-22: circuito del video por perfil y los casos de las fallas arregladas."""
    import pista_video as pv
    out = []
    for ancho, perfil, etiqueta in ((0.75, "config.json", "base"), (0.75, "config_rapido.json", "rápido"),
                                    (0.75, "config_agresivo.json", "agresivo"),
                                    (0.90, "config_rapido.json", "rápido"), (0.55, "config_rapido.json", "rápido")):
        p, o = pv.circuito(ancho)
        out.append((f"video_{int(ancho * 100)}_{etiqueta}", f"Video {ancho:.2f} m · {etiqueta}",
                    lambda p=p, o=o, f=perfil: grabar_bench(p, o, f)))
    final = T._tramo_final(True)
    out.append(("prohibido_lateral", "Prohibido lateral antes de la bandera",
                lambda: grabar_bench(final, T.OBJ_FINAL, "config_rapido.json")))
    caja = T._caja_en_el_fondo(0.35)
    out.append(("caja_sobresale", "Caja con flecha que sobresale (rescate)",
                lambda: grabar_bench(caja, [("izq", (2.65, 0.0), T.IZQ_X), ("cara", (2.6, 2.4), (0.0, -1.0))],
                                     "config_rapido.json")))
    out.append(("flecha_impresa", "Flecha impresa en caja lateral",
                lambda: grabar_bench(final, [("izq", (2.0, -0.4), T.IZQ_Y)] + T.OBJ_FINAL,
                                     "config_rapido.json")))
    return out


def main():
    salida = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "sim_png", "sim.json")
    os.makedirs(os.path.dirname(os.path.abspath(salida)), exist_ok=True)
    out = []
    for clave, nombre, correr in nuevas():
        g = correr()
        g.update(clave=clave, nombre=nombre)
        out.append(g)
        r = g["resumen"]
        print(f"{clave:20s} {'OK' if r['ok'] else 'FALLA'} t={r['t']}s B={r['t_b']}s choques={r['choques']} frames={len(g['frames'])}", flush=True)
    for clave, nombre, pista, kw in ESCENARIOS:
        g = grabar(pista, kw.get("deriva", 0.0), kw.get("retardo", 0), kw.get("extra"))
        g.update(clave=clave, nombre=nombre)
        out.append(g)
        r = g["resumen"]
        print(f"{clave:18s} {'OK' if r['ok'] else 'FALLA'} t={r['t']}s frames={len(g['frames'])}")
    with open(salida, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, separators=(",", ":"))
    print("guardado", salida, f"{os.path.getsize(salida) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
