"""Dibuja (o anima) una corrida del modo directo en el simulador.

    python tests/ver_simulacion.py                      # zigzag con flechas, sin deriva
    python tests/ver_simulacion.py angosta --deriva 0.06
    python tests/ver_simulacion.py lateral --retardo 3 --v 0.30 --animar
    python tests/ver_simulacion.py todas --guardar      # un png por pista en tests/sim_png/

Negro = paredes. Azul = ida, naranja = regreso (posicion REAL). Gris punteado = lo que cree
la odometria. Flechas verdes = senales, circulo rojo = caras. Estrella = decision tomada.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE)]

import matplotlib  # noqa: E402

if "--guardar" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402

import directo as dr  # noqa: E402
import test_directo as T  # noqa: E402

PISTAS = {
    "zigzag": lambda: (T.pista_zigzag(), T.FLECHAS_ZIGZAG + T.CARAS_ZIGZAG, 200),
    "zigzag_sin_flechas": lambda: (T.pista_zigzag(), T.CARAS_ZIGZAG, 200),
    "laberinto": lambda: (T.laberinto_t_y_esquina(), T.LAB_FLECHAS + T.LAB_META, 200),
    "angosta": lambda: (T.pista_angosta(),
                        [("der", (1.19, 0.0), T.IZQ_X), ("cara", (0.9, -2.5), T.IZQ_Y)], 200),
    "callejon": lambda: (T.laberinto_t_y_esquina(),
                         [("der", (2.6, 0.0), T.IZQ_X), ("der", (2.0, 2.8), T.DER_Y)] + T.LAB_META, 240),
    "lateral": lambda: (T._t_con_salida_lateral(), [("cara", (2.6, 3.0), T.DER_Y)], 240),
}


def correr(nombre, deriva, retardo, v):
    paredes, objetos, t_max = PISTAS[nombre]()
    tray = []          # (x_real, y_real, th_real, x_odo, y_odo, fase)
    clase = T.Sim
    estado = {"fase": "ida"}

    class SimGrabada(clase):
        def step(self, lin, ang, dt):
            super().step(lin, ang, dt)
            p = self._piloto.pose(self.odom) if getattr(self, "_piloto", None) else (math.nan,) * 3
            tray.append((*self.true, p[0], p[1], estado["fase"]))

    orig_regreso = dr.PilotoDirecto.regreso
    orig_init = dr.PilotoDirecto.iniciar

    def regreso(self, *a, **k):
        estado["fase"] = "vuelta"
        return orig_regreso(self, *a, **k)

    def iniciar(self, odom, *a, **k):
        out = orig_init(self, odom, *a, **k)
        SimGrabada.ultima._piloto = self.piloto
        return out

    orig_sim_init = SimGrabada.__init__

    def sim_init(self, *a, **k):
        orig_sim_init(self, *a, **k)
        SimGrabada.ultima = self

    SimGrabada.__init__ = sim_init
    T.Sim, dr.PilotoDirecto.regreso, dr.PilotoDirecto.iniciar = SimGrabada, regreso, iniciar
    try:
        extra = dict(v_recto=v, v_regreso=v) if v else None
        res = T.simular_directo(paredes, objetos, t_max=t_max, yaw_drift=deriva,
                                scan_retardo=retardo, dp_extra=extra)
    finally:
        T.Sim, dr.PilotoDirecto.regreso, dr.PilotoDirecto.iniciar = clase, orig_regreso, orig_init
    return paredes, objetos, tray, res


def base(ax, paredes, objetos, res, titulo):
    for (x0, y0), (x1, y1) in paredes:
        ax.plot([x0, x1], [y0, y1], "k-", lw=3)
    for tipo, (x, y), _ in objetos:
        if tipo == "cara":
            ax.plot(x, y, "o", ms=14, mfc="none", mec="red", mew=2)
            ax.annotate("caras", (x, y), color="red", xytext=(4, 6), textcoords="offset points")
        else:
            ax.plot(x, y, "s", ms=9, color="green")
            ax.annotate(tipo, (x, y), color="green", xytext=(4, 6), textcoords="offset points")
    ax.plot(0, 0, "*", ms=16, color="gold", mec="k")
    ax.annotate("A", (0, 0), xytext=(-12, -14), textcoords="offset points", weight="bold")
    for dec in res["d"].decisiones:
        x, y = dec["pose"][0], dec["pose"][1]
        ax.plot(x, y, "*", ms=12, color="purple")
        ax.annotate(dec["accion"], (x, y), color="purple", xytext=(5, -12), textcoords="offset points")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_title(titulo, fontsize=10)


def titulo_de(nombre, deriva, retardo, v, res):
    ok = res["ok"] and T._en_a(res) and res["collisions"] == 0
    return (f"{nombre}  deriva {deriva:+.0%}  retardo LiDAR {retardo * 0.1:.1f}s  v {v or 'def'}\n"
            f"{'OK' if ok else 'FALLA'}  t={res['t']:.0f}s (B en {res['t_b'] or 0:.0f}s)  "
            f"choques={res['collisions']}  min a pared={res['min_clear']:.2f} m")


def dibujar(nombre, deriva, retardo, v, animar, guardar):
    paredes, objetos, tray, res = correr(nombre, deriva, retardo, v)
    fig, ax = plt.subplots(figsize=(8, 7))
    base(ax, paredes, objetos, res, titulo_de(nombre, deriva, retardo, v, res))
    ida = [p for p in tray if p[5] == "ida"]
    vta = [p for p in tray if p[5] == "vuelta"]
    if not animar:
        ax.plot([p[0] for p in ida], [p[1] for p in ida], "-", color="tab:blue", lw=2, label="ida (real)")
        ax.plot([p[0] for p in vta], [p[1] for p in vta], "-", color="tab:orange", lw=2, label="regreso (real)")
        ax.plot([p[3] for p in tray], [p[4] for p in tray], ":", color="gray", label="odometria")
        ax.legend(loc="best", fontsize=8)
        print("eventos:", *res["d"].eventos, sep="\n  ")
        if guardar:
            os.makedirs(os.path.join(HERE, "sim_png"), exist_ok=True)
            ruta = os.path.join(HERE, "sim_png", f"{nombre}_d{deriva:+.2f}_r{retardo}.png")
            fig.savefig(ruta, dpi=110, bbox_inches="tight")
            print("guardado", ruta)
        else:
            plt.show()
        return
    linea_i, = ax.plot([], [], "-", color="tab:blue", lw=2)
    linea_v, = ax.plot([], [], "-", color="tab:orange", lw=2)
    robot = plt.Circle((0, 0), 0.17, color="tab:green", alpha=0.6)
    ax.add_patch(robot)
    nariz, = ax.plot([], [], "g-", lw=2)
    reloj = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top")
    paso = 3      # 0.3 s por cuadro

    def cuadro(k):
        i = min(k * paso, len(tray) - 1)
        hist = tray[: i + 1]
        linea_i.set_data([p[0] for p in hist if p[5] == "ida"], [p[1] for p in hist if p[5] == "ida"])
        linea_v.set_data([p[0] for p in hist if p[5] == "vuelta"], [p[1] for p in hist if p[5] == "vuelta"])
        x, y, th = tray[i][:3]
        robot.center = (x, y)
        nariz.set_data([x, x + 0.25 * math.cos(th)], [y, y + 0.25 * math.sin(th)])
        reloj.set_text(f"t = {i * 0.1:.1f} s  {tray[i][5]}")
        return linea_i, linea_v, robot, nariz, reloj

    anim = FuncAnimation(fig, cuadro, frames=len(tray) // paso + 1, interval=30, blit=True, repeat=False)
    if guardar:
        os.makedirs(os.path.join(HERE, "sim_png"), exist_ok=True)
        ruta = os.path.join(HERE, "sim_png", f"{nombre}_d{deriva:+.2f}_r{retardo}.gif")
        anim.save(ruta, writer="pillow", fps=20)
        print("guardado", ruta)
    else:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("pista", nargs="?", default="zigzag", choices=list(PISTAS) + ["todas"])
    ap.add_argument("--deriva", type=float, default=0.0, help="error de la odometria al girar (0.06 = 6%%)")
    ap.add_argument("--retardo", type=int, default=0, help="atraso del LiDAR en pasos de 0.1 s")
    ap.add_argument("--v", type=float, default=None, help="v_recto y v_regreso (m/s)")
    ap.add_argument("--animar", action="store_true", help="ver al robot moverse")
    ap.add_argument("--guardar", action="store_true", help="png (o gif con --animar) en tests/sim_png/")
    a = ap.parse_args()
    for nombre in (PISTAS if a.pista == "todas" else [a.pista]):
        dibujar(nombre, a.deriva, a.retardo, a.v, a.animar, a.guardar)


if __name__ == "__main__":
    main()
