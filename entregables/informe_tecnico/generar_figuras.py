"""Genera las figuras y tablas de resultados del informe a partir del SIMULADOR real del
proyecto (1_competencia/tests/sim.py + nav_core.py + navegacion.py). No inventa datos:
todo lo que sale aqui se calcula corriendo el codigo de navegacion.

Uso:  python generar_figuras.py        (escribe figuras/*.pdf y figuras/resultados_sim.tex)
"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
COMP = os.path.join(RAIZ, "1_competencia")
sys.path.insert(0, COMP)
sys.path.insert(0, os.path.join(COMP, "tests"))

import nav_core as nc                      # noqa: E402
from navegacion import Piloto, PilotoParams  # noqa: E402
from sim import Sim, box, raycast        # noqa: E402

OUT = os.path.join(AQUI, "figuras")
os.makedirs(OUT, exist_ok=True)
CFG = json.load(open(os.path.join(COMP, "config.json"), encoding="utf-8"))

AZUL, ROJO, VERDE, NARANJA, GRIS = "#1f4e9c", "#c0392b", "#2e8b57", "#e67e22", "#7f8c8d"
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.alpha": 0.25, "figure.dpi": 150,
                     "font.family": "serif"})


def params(max_linear=None):
    nav = nc.NavParams(safety_margin=CFG["safety_margin"], stop_dist=CFG["stop_dist"],
                       slow_dist=CFG["slow_dist"], wall_min=CFG["wall_min"])
    pp = PilotoParams(max_linear=max_linear or CFG["max_linear"], max_angular=CFG["max_angular"],
                      max_accel=CFG["max_accel"], max_decel=CFG["max_decel"],
                      max_alpha=CFG["max_alpha"], goal_tol=CFG["goal_tol"],
                      home_tol=CFG["home_tol"], stuck_time=CFG["stuck_time"])
    return nav, pp


def mision(walls, goal, low=(), yaw_drift=0.0, icp=True, max_linear=None, t_max=300.0, dt=0.1):
    """Mision A->B->A con logging. Igual que sim.run pero guardando trayectoria y comandos."""
    nav, pp = params(max_linear)
    if not icp:
        pp.icp_max_jump = 0.0           # ICP siempre descartado (igual que en test_navegacion)
    sim = Sim(walls, low=low, yaw_drift=yaw_drift)
    p = Piloto(nav=nav, pp=pp)
    p.fijar_origen(sim.odom, *sim.scan())
    p.ir_a(goal)
    tray, cmd = [], []
    t, fase, t_b, b_true = 0.0, "ida", None, None
    while t < t_max:
        lin, ang, info = p.paso(t, sim.odom, *sim.scan())
        if fase == "ida" and info["dist"] < pp.goal_tol:
            fase, t_b, b_true = "vuelta", t, sim.true
            p.volver()
            continue
        if fase == "vuelta" and info["llegada"]:
            return dict(ok=True, t=t, t_b=t_b, true=sim.true, b_true=b_true, tray=tray, cmd=cmd,
                        collisions=sim.collisions, min_clear=sim.min_clear, piloto=p,
                        err_A=math.hypot(*sim.true[:2]), t_b_idx=len([c for c in cmd if c[0] <= t_b]))
        sim.step(lin, ang, dt)
        tray.append((t, *sim.true[:2], 0 if fase == "ida" else 1))
        cmd.append((t, lin, ang))
        t += dt
    return dict(ok=False, t=t, tray=tray, cmd=cmd, collisions=sim.collisions,
                min_clear=sim.min_clear, true=sim.true, err_A=math.hypot(*sim.true[:2]),
                piloto=p)


def sala(w=6.0, h=4.0, x0=-1.0, y0=-2.0):
    return box(x0 + w / 2, y0 + h / 2, w, h)


CIRCUITOS = {
    "Obst\u00e1culos en sala": dict(walls=sala() + box(1.5, 0.6, 0.4, 0.4) + box(1.0, -1.2, 0.5, 0.3)
                                    + box(3.2, 1.2, 0.3, 0.3), goal=(3.5, -0.5)),
    "Laberinto con callej\u00f3n": dict(walls=[
        ((-0.8, -0.6), (-0.8, 0.6)), ((-0.8, -0.6), (1.4, -0.6)), ((-0.8, 0.6), (1.4, 0.6)),
        ((1.4, -0.6), (1.4, -1.8)), ((1.4, -1.8), (2.6, -1.8)), ((2.6, -1.8), (2.6, 1.6)),
        ((1.4, 0.6), (1.4, 2.8)), ((1.4, 2.8), (4.4, 2.8)), ((2.6, 1.6), (4.4, 1.6)),
        ((4.4, 1.6), (4.4, 2.8))], goal=(3.8, 2.2)),
    "Pasillo en L": dict(walls=[((-0.5, -0.5), (3.0, -0.5)), ((3.0, -0.5), (3.0, 3.0)),
                                ((-0.5, 0.5), (2.0, 0.5)), ((2.0, 0.5), (2.0, 3.0)),
                                ((-0.5, -0.5), (-0.5, 0.5))], goal=(2.5, 2.5)),
    "Obst\u00e1culo bajo (invisible)": dict(walls=sala(), low=box(1.3, 0.0, 0.25, 0.5), goal=(2.8, 0.0)),
}


def dibujar_paredes(ax, walls, low=()):
    for (x1, y1), (x2, y2) in walls:
        ax.plot([x1, x2], [y1, y2], color="#333333", lw=2)
    for (x1, y1), (x2, y2) in low:
        ax.plot([x1, x2], [y1, y2], color=NARANJA, lw=2, ls="--")


# --------------------------------------------------------------------------- F1: trayectorias
def fig_trayectorias():
    res_all = {}
    fig, axs = plt.subplots(1, 4, figsize=(11.5, 3.3))
    for ax, (nombre, c) in zip(axs, CIRCUITOS.items()):
        low = c.get("low", ())
        r = mision(c["walls"], c["goal"], low=low)
        res_all[nombre] = r
        dibujar_paredes(ax, c["walls"], low)
        tr = np.array(r["tray"])
        ida, vuelta = tr[tr[:, 3] == 0], tr[tr[:, 3] == 1]
        ax.plot(ida[:, 1], ida[:, 2], color=AZUL, lw=1.8, label="ida $A\\to B$")
        if len(vuelta):
            ax.plot(vuelta[:, 1], vuelta[:, 2], color=VERDE, lw=1.8, ls="--", label="regreso $B\\to A$")
        ax.plot(0, 0, "ks", ms=7); ax.annotate("A", (0, 0), (6, 6), textcoords="offset points", weight="bold")
        ax.plot(*c["goal"], marker="*", color=ROJO, ms=11)
        ax.annotate("B", c["goal"], (6, 6), textcoords="offset points", weight="bold", color=ROJO)
        ax.set_aspect("equal"); ax.set_title(nombre + ("" if r["ok"] else "\n(no completa con la config. final)"), fontsize=9, color="black" if r["ok"] else ROJO)
        ax.set_xlabel("x (m)")
        if not ax.get_subplotspec().is_first_col():
            pass
    axs[0].set_ylabel("y (m)")
    axs[0].legend(loc="lower right", fontsize=7, frameon=True)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "trayectorias.pdf"))
    plt.close(fig)
    return res_all


# --------------------------------------------------------------------------- F2: deriva y ICP
def exp_deriva():
    drifts = [0.0, 0.04, 0.08, 0.12]
    goals = [(3.5, -0.5), (3.0, 0.8), (2.8, -1.0), (3.6, 1.0), (2.4, 0.0)]
    walls = CIRCUITOS["Obst\u00e1culos en sala"]["walls"]
    filas = []
    for d in drifts:
        for icp in (False, True):
            errs, oks = [], 0
            for g in goals:
                r = mision(walls, g, yaw_drift=d, icp=icp)
                if r["ok"]:
                    oks += 1
                    errs.append(r["err_A"])
            filas.append(dict(drift=d, icp=icp, n=len(goals), ok=oks,
                              err_med=float(np.mean(errs)) if errs else float("nan"),
                              err_max=float(np.max(errs)) if errs else float("nan")))
    fig, ax = plt.subplots(figsize=(5.2, 3.0))
    x = np.arange(len(drifts)); w = 0.36
    sin = [f["err_med"] * 100 for f in filas if not f["icp"]]
    con = [f["err_med"] * 100 for f in filas if f["icp"]]
    b1 = ax.bar(x - w / 2, sin, w, color=ROJO, label="solo odometr\u00eda")
    b2 = ax.bar(x + w / 2, con, w, color=VERDE, label="odometr\u00eda + ICP 2D")
    ax.axhline(CFG["home_tol"] * 100, color=GRIS, ls=":", lw=1.2, label="tolerancia en $A$ (20 cm)")
    for b in list(b1) + list(b2):
        ax.annotate(f"{b.get_height():.0f}", (b.get_x() + b.get_width() / 2, b.get_height()),
                    (0, 2), textcoords="offset points", ha="center", fontsize=7)
    ax.set_xticks(x); ax.set_xticklabels([f"{int(d * 100)} %" for d in drifts])
    ax.set_xlabel("error de escala de giro en la odometr\u00eda"); ax.set_ylabel("distancia final a $A$ (cm)")
    ax.legend(fontsize=7.5, frameon=False); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "deriva_icp.pdf")); plt.close(fig)
    return filas


# --------------------------------------------------------------------------- F3: velocidad vs tiempo/riesgo
def exp_velocidad():
    walls = CIRCUITOS["Obst\u00e1culos en sala"]["walls"]
    vs = [0.08, 0.10, 0.14, 0.18, 0.22, 0.26, 0.31]
    filas = []
    for v in vs:
        r = mision(walls, (3.5, -0.5), max_linear=v)
        filas.append(dict(v=v, ok=r["ok"], t=r["t"], choques=r["collisions"], holgura=r["min_clear"]))
    fig, ax = plt.subplots(figsize=(5.2, 3.0))
    ax.plot([f["v"] for f in filas], [f["t"] for f in filas], "o-", color=AZUL, label="tiempo de misi\u00f3n")
    ax.set_xlabel("$v_{\\max}$ configurada (m/s)"); ax.set_ylabel("tiempo $A\\to B\\to A$ (s)", color=AZUL)
    ax2 = ax.twinx(); ax2.spines["right"].set_visible(True); ax2.grid(False)
    ax2.plot([f["v"] for f in filas], [(f["holgura"] - 0.17) * 100 for f in filas], "s--", color=ROJO,
             label="holgura m\u00ednima a paredes")
    ax2.set_ylabel("holgura libre m\u00ednima (cm)", color=ROJO)
    ax.axvline(CFG["max_linear"], color=GRIS, ls=":"); ax.text(CFG["max_linear"] + 0.005, ax.get_ylim()[1] * 0.90,
                                                               "elegida", fontsize=7, color=GRIS, va="top")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "velocidad.pdf")); plt.close(fig)
    return filas


# --------------------------------------------------------------------------- F4: selecci\u00f3n de rumbo
def fig_rumbo():
    nav = params()[0]
    walls = sala() + box(1.0, 0.15, 0.4, 0.4) + [((-1.0, 0.9), (0.2, 0.9))]
    r, a = raycast(walls)
    goal = 0.0
    hs = nc.headings(nav)
    free = nc.free_distance(r, a, hs, nav)
    marg = nc.side_margin(r, a, hs, nav)
    adm = free >= nav.min_free
    J = np.where(adm, nav.w_goal * np.abs(nc.wrap(hs - goal))
                 + nav.w_turn * np.abs(nc.wrap(hs - 0.0)) - nav.w_clear * marg, np.nan)
    best = hs[np.nanargmin(J)]; assert abs(best - nc.choose_heading(r, a, goal, 0.0, nav)) < 1e-9
    fig = plt.figure(figsize=(7.2, 3.1))
    ax1 = fig.add_subplot(1, 2, 1)
    dibujar_paredes(ax1, [((x1 , y1), (x2, y2)) for (x1, y1), (x2, y2) in walls])
    pts = nc.to_xy(r, a)
    ax1.plot(pts[:, 0], pts[:, 1], ".", ms=2, color=AZUL)
    ax1.add_patch(plt.Circle((0, 0), nav.inflate, color=ROJO, alpha=0.18))
    ax1.add_patch(plt.Circle((0, 0), nav.robot_radius, color=ROJO, alpha=0.35))
    L = 1.6
    ax1.annotate("", (L * math.cos(best), L * math.sin(best)), (0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=VERDE, lw=2))
    ax1.annotate("", (L * math.cos(goal), L * math.sin(goal)), (0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=GRIS, lw=1.5, ls="--"))
    ax1.text(1.65, 0.05, "meta", fontsize=7, color=GRIS); ax1.text(1.0, -1.1, "rumbo\nelegido", fontsize=7, color=VERDE)
    ax1.set_aspect("equal"); ax1.set_xlim(-1.2, 2.6); ax1.set_ylim(-1.4, 1.6)
    ax1.set_title("Escaneo LiDAR y robot inflado", fontsize=9); ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax2 = fig.add_subplot(1, 2, 2)
    deg = np.degrees(hs)
    ax2.fill_between(deg, 0, np.minimum(free, 3.0), color=AZUL, alpha=0.2, label="distancia libre $d(\\theta)$ (m)")
    ax2.axhline(nav.min_free, color=ROJO, ls=":", lw=1)
    ax2.text(-178, nav.min_free + 0.05, "$d_{\\min}$", color=ROJO, fontsize=7)
    ax2.set_xlabel("rumbo candidato $\\theta$ (\u00b0)"); ax2.set_ylabel("$d(\\theta)$ (m)")
    ax3 = ax2.twinx(); ax3.spines["right"].set_visible(True); ax3.grid(False)
    ax3.plot(deg, J, color=NARANJA, lw=1.6, label="costo $J(\\theta)$")
    ax3.plot(np.degrees(best), np.nanmin(J), "o", color=VERDE, ms=7, zorder=5)
    ax3.set_ylabel("$J(\\theta)$", color=NARANJA)
    ax2.set_title("Candidatos admisibles y costo", fontsize=9)
    h1, l1 = ax2.get_legend_handles_labels(); h2, l2 = ax3.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, fontsize=7, loc="upper center", frameon=True, framealpha=0.9)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "seleccion_rumbo.pdf")); plt.close(fig)
    return math.degrees(best)


# --------------------------------------------------------------------------- F5: perfil de velocidad
def fig_perfil(res):
    r = res["Obst\u00e1culos en sala"]
    cmd = np.array(r["cmd"])
    fig, ax = plt.subplots(figsize=(6.4, 2.6))
    ax.plot(cmd[:, 0], cmd[:, 1], color=AZUL, lw=1.4, label="$v$ (m/s)")
    ax.set_xlabel("tiempo (s)"); ax.set_ylabel("$v$ (m/s)", color=AZUL)
    ax2 = ax.twinx(); ax2.spines["right"].set_visible(True); ax2.grid(False)
    ax2.plot(cmd[:, 0], cmd[:, 2], color=NARANJA, lw=1.0, alpha=0.9)
    ax2.set_ylabel("$\\omega$ (rad/s)", color=NARANJA)
    ax.axvline(r["t_b"], color=ROJO, ls=":"); ax.text(r["t_b"] + 0.6, 0.06, "llegada a $B$", color=ROJO, fontsize=7, va="center",
                                                      bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.8))
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "perfil_velocidad.pdf")); plt.close(fig)


def main():
    res = fig_trayectorias()
    filas_c = []
    for nombre, r in res.items():
        filas_c.append((nombre, r))
    deriva = exp_deriva()
    vel = exp_velocidad()
    best = fig_rumbo()
    fig_perfil(res)

    def f1(x): return f"{x:.1f}".replace(".", ",") if x == x else "--"
    L = []
    L.append(r"% generado por generar_figuras.py -- no editar a mano")
    L.append(r"\newcommand{\TablaCircuitos}{\begin{tabular}{@{}lrrrrc@{}}\toprule")
    L.append(r"\textbf{Circuito} & $t_{A\to B}$ (s) & $t_{\text{total}}$ (s) & Choques & Holgura m\'in.\ (cm) & Error en $A$ (cm)\\\midrule")
    for nombre, r in filas_c:
        tb = r.get("t_b", float("nan"))
        L.append(f"{nombre} & {f1(tb)} & {f1(r['t'])} & {r['collisions']} & {f1((r['min_clear'] - 0.17) * 100)} & {f1(r['err_A'] * 100)}\\\\")
    L.append(r"\bottomrule\end{tabular}}")
    L.append(r"\newcommand{\TablaDeriva}{\begin{tabular}{@{}rcccc@{}}\toprule")
    L.append(r"\textbf{Deriva} & \textbf{ICP} & \textbf{Misiones OK} & \textbf{Error medio (cm)} & \textbf{Error m\'ax.\ (cm)}\\\midrule")
    for f in deriva:
        si = "s\\'i" if f["icp"] else "no"
        L.append(f"{int(f['drift'] * 100)}\\,\\% & {si} & {f['ok']}/{f['n']} & {f1(f['err_med'] * 100)} & {f1(f['err_max'] * 100)}\\\\")
    L.append(r"\bottomrule\end{tabular}}")
    L.append(r"\newcommand{\TablaVelocidad}{\begin{tabular}{@{}rrrr@{}}\toprule")
    L.append(r"$v_{\max}$ (m/s) & Tiempo (s) & Choques & Holgura m\'in.\ (cm)\\\midrule")
    for f in vel:
        L.append(f"{f['v']:.2f} & {f1(f['t'])} & {f['choques']} & {f1((f['holgura'] - 0.17) * 100)}\\\\".replace("0.", "0,"))
    L.append(r"\bottomrule\end{tabular}}")
    open(os.path.join(OUT, "resultados_sim.tex"), "w", encoding="utf-8").write("\n".join(L) + "\n")
    json.dump(dict(circuitos={n: {k: v for k, v in r.items() if k in ("ok", "t", "t_b", "collisions", "min_clear", "err_A")} for n, r in filas_c},
                   deriva=deriva, velocidad=vel, rumbo_elegido_deg=best),
              open(os.path.join(OUT, "resultados_sim.json"), "w", encoding="utf-8"), indent=1, default=float)
    print("listo", best)


if __name__ == "__main__":
    main()
