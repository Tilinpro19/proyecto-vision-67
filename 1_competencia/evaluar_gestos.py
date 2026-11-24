"""
Evalua los clips de grabaciones_gestos/ con el MISMO pipeline de gestos_teleop.py
(Gestos.leer: espejo + CLAHE opcional + MediaPipe + classify_pose + ControlGestos),
usando el tiempo grabado de cada frame. Sin ROS, sin webcam, sin ventana.

Uso (en 1_competencia/):
  ../.venv/bin/python evaluar_gestos.py --etiqueta base         # evalua y guarda
  ../.venv/bin/python evaluar_gestos.py --comparar base nuevo   # tabla antes/despues
  ../.venv/bin/python evaluar_gestos.py --modo mejorados --etiqueta mej   # con --gestos-mejorados
  ../.venv/bin/python evaluar_gestos.py --fps 10 --etiqueta base_10fps  # camara mas lenta

Resultados en grabaciones_gestos/resultados/<etiqueta>.json y .md (carpeta local).

Notas de interpretacion:
  - Los comandos se miden "como si estuviera ARMADO" (ver banco_gestos.simular_control).
  - En transiciones la latencia INCLUYE el tiempo de reaccion de la persona: sirve
    para comparar versiones sobre los mismos clips, no como latencia absoluta.
  - 'conf' = score de lado de mano de MediaPipe; 'margen' = distancia de la pose al
    umbral de otra pose segun las reglas de classify_pose (banco_gestos.margenes).
"""
import argparse
import glob
import json
import os
import sys
from collections import Counter

import cv2
import numpy as np

import banco_gestos as bg
import gestos_teleop as gt

HERE = os.path.dirname(os.path.abspath(__file__))
DIR_GRABACIONES = os.path.join(HERE, "grabaciones_gestos")


MODOS = {
    "original": dict(),
    "m1": dict(like_pulgar_min=gt.LIKE_PULGAR_MIN),
    "m1m2": dict(like_pulgar_min=gt.LIKE_PULGAR_MIN, t_mover=gt.T_MOVER, t_parar=gt.T_PARAR),
    "m1m2m7": dict(like_pulgar_min=gt.LIKE_PULGAR_MIN, t_mover=gt.T_MOVER, t_parar=gt.T_PARAR,
                   rechazar_parcial=True),
}
MODOS["mejorados"] = MODOS["m1m2"]     # = --gestos-mejorados (v1: M1+M2; M7 aparte)


def indices_fps(ts, fps):
    """Indices de frames para simular una camara a 'fps' (el frame mas cercano a cada
    instante k/fps). Solo puede BAJAR el fps grabado. fps=None: todos."""
    if not fps:
        return list(range(len(ts)))
    ta = np.asarray(ts)
    keep = []
    for k in range(int((ta[-1] - ta[0]) * fps) + 1):
        i = int(np.argmin(np.abs(ta - (ta[0] + k / fps))))
        if not keep or i > keep[-1]:
            keep.append(i)
    return keep


class CapSubmuestreo:
    """VideoCapture que solo entrega los frames de 'keep' (para --fps)."""

    def __init__(self, cap, keep):
        self.cap, self.keep, self.i = cap, set(keep), -1

    def read(self):
        while True:
            ret, f = self.cap.read()
            self.i += 1
            if not ret:
                return False, None
            if self.i in self.keep:
                return True, f

    def release(self):
        self.cap.release()


def procesar_clip(avi, meta, clahe=False, params=None, fps=None):
    """Pasa un clip por Gestos.leer con su reloj grabado -> (frames, inconsistencias).
    'dir' = comando si estuviera siempre armado (simular_control); 'dir_real' = lo que
    haria el control real de Gestos arrancando ARMADO (un puno mantenido PUNO_HOLD s desarma y luego
    nada se mueve hasta volver a armar)."""
    params = params or gt.ParamsGestos()
    cap = cv2.VideoCapture(avi)
    if not cap.isOpened():
        raise RuntimeError("no se pudo abrir " + avi)
    keep = indices_fps(meta["t"], fps)
    if fps:
        cap = CapSubmuestreo(cap, keep)
    g = gt.Gestos(0, clahe=clahe, cap=cap, reloj_t=True, params=params)
    g.ctrl.armed = True
    ts_all = [meta["t"][i] for i in keep]
    objetivos = [meta["objetivo"][i] for i in keep] if meta.get("objetivo") else None
    poses, ts, scores, marg, rivales, reales, incons = [], [], [], [], [], [], 0
    try:
        for t in ts_all:
            out = g.leer(t)
            if out is None:
                break
            pose = out[3]
            poses.append(pose)
            ts.append(t)
            reales.append(tuple(out[1]))
            scores.append(g.detector.score)
            if g.detector.lm is not None and not g.parcial:
                m = bg.margenes(g.detector.lm, params.like_pulgar_min)
                p_m, mg, rival, _ = bg.margen_y_rival(m)
                incons += p_m != pose
                marg.append(mg)
                rivales.append(rival)
            else:
                marg.append(None)
                rivales.append(None)
    finally:
        g.close()
    held, dirs, eventos = bg.simular_control(poses, ts, lambda: gt.nuevo_control(params))
    frames = [dict(t=ts[i], cruda=poses[i], estable=held[i], dir=dirs[i], eventos=eventos[i],
                   dir_real=reales[i], margen=marg[i], rival=rivales[i], conf=scores[i],
                   objetivo=objetivos[i] if objetivos else None)
              for i in range(len(poses))]
    return frames, incons


def cargar_clips(carpeta):
    clips = []
    for js in sorted(glob.glob(os.path.join(carpeta, "*", "*.json"))):
        if os.sep + "resultados" + os.sep in js:
            continue
        avi = js[:-5] + ".avi"
        if not os.path.exists(avi):
            continue
        with open(js, encoding="utf-8") as f:
            clips.append((avi, json.load(f)))
    return clips


def pct(x):
    return "-" if x is None else f"{100 * x:.1f}%"


def num(x, f="{:.2f}"):
    return "-" if x is None else f.format(x)


def evaluar(carpeta, clahe=False, params=None, fps=None):
    clips = cargar_clips(carpeta)
    if not clips:
        raise SystemExit(f"No hay clips en {carpeta}. Graba con grabar_gestos.py.")
    por_gesto, ninguno, trans, filas_cruda, filas_est, clip_rows = {}, [], [], [], [], []
    por_variante = {}
    incons_total = frames_total = 0
    for avi, meta in clips:
        etq = meta["etiqueta"]
        print(f"  {os.path.relpath(avi, carpeta)} ...", file=sys.stderr)
        fr, inc = procesar_clip(avi, meta, clahe, params, fps)
        if not fr:
            continue
        incons_total += inc
        frames_total += len(fr)
        if etq == bg.TRANSICIONES:
            mt = bg.metricas_transiciones(fr)
            trans.append(mt)
            for x in fr:   # transiciones: solo en la matriz CRUDA (la estable va con retraso)
                filas_cruda.append((x["objetivo"], x["cruda"] or bg.SIN_MANO))
            clip_rows.append((os.path.basename(avi), etq, meta.get("variante"), len(fr),
                              f"exact. estable (fuera de gracia) {pct(mt['acc_estable'])}"))
            continue
        if etq == "ninguno":
            mn = bg.metricas_ninguno(fr)
            ninguno.append(mn)
            clip_rows.append((os.path.basename(avi), etq, meta.get("variante"), len(fr),
                              f"arranques falsos {mn['arranques']} en {mn['seg']:.0f} s"))
        else:
            mg = bg.metricas_gesto(fr, etq)
            por_gesto.setdefault(etq, []).append(mg)
            por_variante.setdefault(meta.get("variante") or "-", []).append(mg)
            clip_rows.append((os.path.basename(avi), etq, meta.get("variante"), len(fr),
                              f"estable {pct(mg['acc_estable'])}; peligro {pct(mg['cmd_peligro'])}"))
        for x in fr:
            filas_cruda.append((etq, x["cruda"] or bg.SIN_MANO))
            filas_est.append((etq, x["estable"] or bg.SIN_POSE))
    return dict(por_gesto=por_gesto, ninguno=ninguno, trans=trans, por_variante=por_variante,
                conf_cruda=bg.matriz_confusion(filas_cruda),
                conf_estable=bg.matriz_confusion(filas_est),
                clips=clip_rows, inconsistencias=incons_total, frames=frames_total,
                clahe=clahe, params=params or gt.ParamsGestos(), fps=fps)


def _media(lst, key, peso="frames"):
    vals = [(d[key], d[peso]) for d in lst if d.get(key) is not None]
    if not vals:
        return None
    return sum(v * w for v, w in vals) / sum(w for _, w in vals)


def resumen(r):
    """Numeros clave (para la tabla antes/despues)."""
    s = {}
    for etq, lst in r["por_gesto"].items():
        lat = [d["latencia"] for d in lst if d["latencia"] is not None]
        ev_in = Counter()
        for d in lst:
            ev_in.update(d["eventos_inesperados"])
        s[etq] = dict(
            clips=len(lst), seg=sum(d["seg"] for d in lst),
            deteccion=_media(lst, "deteccion"), acc_cruda=_media(lst, "acc_cruda"),
            acc_estable=_media(lst, "acc_estable"), cmd_ok=_media(lst, "cmd_ok"),
            cmd_peligro=_media(lst, "cmd_peligro"),
            arranques_peligro=sum(d["arranques_peligro"] for d in lst),
            eventos_inesperados=dict(ev_in),
            latencia=float(np.median(lat)) if lat else None,
            sin_reconocer=sum(d["latencia"] is None for d in lst),
            margen=_media(lst, "margen_mediana"), cerca_umbral=_media(lst, "cerca_umbral"),
            rival=Counter(d["rival"] for d in lst if d["rival"]).most_common(1)[0][0]
            if any(d["rival"] for d in lst) else None,
            parpadeos_min=_media(lst, "parpadeos_min"),
            seg_avance_real=sum(d.get("seg_avance_real", 0) for d in lst),
            seg_mov_real=sum(d.get("seg_mov_real", 0) for d in lst))
    if r["ninguno"]:
        mins = sum(d["minutos"] for d in r["ninguno"])
        poses = Counter()
        for d in r["ninguno"]:
            poses.update(d["poses_falsas"])
        s["ninguno"] = dict(
            minutos=mins, deteccion=_media(r["ninguno"], "deteccion"),
            acc_estable=_media(r["ninguno"], "acc_estable"),
            arranques_min=sum(d["arranques"] for d in r["ninguno"]) / mins,
            seg_mov_min=sum(d["seg_mov"] for d in r["ninguno"]) / mins,
            armar_min=sum(d["armar"] for d in r["ninguno"]) / mins,
            accion_min=sum(d["accion"] for d in r["ninguno"]) / mins,
            desarmar_min=sum(d["desarmar"] for d in r["ninguno"]) / mins,
            poses_falsas=dict(poses),
            seg_mov_real=sum(d.get("seg_mov_real", 0) for d in r["ninguno"]))
    if r["trans"]:
        cambios = [c for d in r["trans"] for c in d["cambios"]]
        lat = [c["latencia"] for c in cambios if c["latencia"] is not None]
        ev = Counter(e for c in cambios for e in c["eventos_ajenos"])
        s["transiciones"] = dict(
            cambios=len(cambios),
            latencia_mediana=float(np.median(lat)) if lat else None,
            latencia_p90=float(np.percentile(lat, 90)) if lat else None,
            sin_reconocer=sum(c["latencia"] is None for c in cambios),
            arranques_ajenos=sum(c["arranques_ajenos"] for c in cambios),
            seg_ajenos=sum(c["seg_ajenos"] for c in cambios),
            eventos_ajenos=dict(ev),
            acc_estable=_media([dict(d, frames=1) for d in r["trans"]], "acc_estable"),
            peores=sorted(([f"{c['de']}->{c['a']}", c["latencia"]] for c in cambios),
                          key=lambda x: -1e9 if x[1] is None else -x[1])[:5])
    gest = [v for k, v in s.items() if k not in ("ninguno", "transiciones", "variantes")]
    s["global"] = dict(
        acc_estable_macro=float(np.mean([g["acc_estable"] for g in gest])) if gest else None,
        cmd_peligro_macro=float(np.mean([g["cmd_peligro"] for g in gest])) if gest else None,
        arranques_peligro=sum(g["arranques_peligro"] for g in gest),
        falsos_arranques_min=s.get("ninguno", {}).get("arranques_min"),
        latencia_trans=s.get("transiciones", {}).get("latencia_mediana"),
        ajenos_trans=s.get("transiciones", {}).get("arranques_ajenos"),
        inconsistencias_margen=r["inconsistencias"], frames=r["frames"],
        puno_a_like=(r["conf_estable"].get("puno", Counter())["like"] /
                     max(sum(r["conf_estable"].get("puno", Counter()).values()), 1)),
        seg_avance_puno_real=s.get("puno", {}).get("seg_avance_real"),
        desarmar_ninguno_min=s.get("ninguno", {}).get("desarmar_min"),
        desarmar_trans=s.get("transiciones", {}).get("eventos_ajenos", {}).get("desarmar", 0)
        if "transiciones" in s else None,
        armar_trans=s.get("transiciones", {}).get("eventos_ajenos", {}).get("armar", 0)
        if "transiciones" in s else None,
        accion_trans=s.get("transiciones", {}).get("eventos_ajenos", {}).get("accion", 0)
        if "transiciones" in s else None)
    s["variantes"] = {
        v: dict(clips=len(lst), acc_estable=_media(lst, "acc_estable"),
                cmd_peligro=_media(lst, "cmd_peligro"),
                seg_avance_real=sum(d.get("seg_avance_real", 0) for d in lst))
        for v, lst in r.get("por_variante", {}).items()}
    return s


def tabla_confusion(mat, titulo):
    filas = [e for e in list(bg.GESTOS_GRABACION) + bg.OBJETIVOS_TRANSICION if e in mat]
    filas = list(dict.fromkeys(filas))
    cols = [c for c in list(gt.POSES) + [bg.SIN_MANO, bg.SIN_POSE]
            if any(c in mat[f] for f in filas)]
    out = [f"\n### {titulo} (% de frames de cada fila)\n",
           "| esperado \\ detectado | " + " | ".join(cols) + " |",
           "|---|" + "---|" * len(cols)]
    for f in filas:
        tot = sum(mat[f].values())
        out.append(f"| {f} | " + " | ".join(
            (f"{100 * mat[f][c] / tot:.0f}" if mat[f][c] else "·") for c in cols) + " |")
    return "\n".join(out)


def informe(r, s, etiqueta):
    L = [f"# Banco de gestos: {etiqueta}", "",
         f"Clips: {len(r['clips'])} | frames: {r['frames']} | CLAHE: {r['clahe']} | "
         f"inconsistencias margen/pose: {r['inconsistencias']} (debe ser 0)",
         f"Modo: {r['params']} | fps simulado: {r['fps'] or 'el grabado'}", "",
         "## Por gesto (clips con una pose mantenida; comandos como si estuviera ARMADO)", "",
         "| gesto | clips | s | deteccion | exact. cruda | exact. estable | cmd correcto | "
         "cmd PELIGROSO | arranques peligrosos | eventos inesperados | latencia inicial (s) | "
         "margen mediano | % al borde | rival | parpadeos/min |",
         "|---" * 15 + "|"]
    for etq in bg.GESTOS_GRABACION:
        if etq not in s or etq == "ninguno":
            continue
        g = s[etq]
        L.append(f"| {etq} | {g['clips']} | {g['seg']:.0f} | {pct(g['deteccion'])} | "
                 f"{pct(g['acc_cruda'])} | {pct(g['acc_estable'])} | {pct(g['cmd_ok'])} | "
                 f"{pct(g['cmd_peligro'])} | {g['arranques_peligro']} | "
                 f"{g['eventos_inesperados'] or '-'} | {num(g['latencia'])}"
                 f"{' (+%d sin reconocer)' % g['sin_reconocer'] if g['sin_reconocer'] else ''} | "
                 f"{num(g['margen'])} | {pct(g['cerca_umbral'])} | {g['rival'] or '-'} | "
                 f"{num(g['parpadeos_min'], '{:.0f}')} |")
    if "ninguno" in s:
        n = s["ninguno"]
        L += ["", "## Ninguno (sin gesto)", "",
              f"- Minutos: {n['minutos']:.2f} | deteccion de mano: {pct(n['deteccion'])} | "
              f"frames sin pose (correcto): {pct(n['acc_estable'])}",
              f"- **Falsos arranques de movimiento: {n['arranques_min']:.2f}/min** "
              f"({n['seg_mov_min']:.2f} s de movimiento falso por minuto)",
              f"- Eventos falsos/min: armar {n['armar_min']:.2f} | accion {n['accion_min']:.2f} | "
              f"desarmar {n['desarmar_min']:.2f}",
              f"- Poses estables vistas por error: {n['poses_falsas'] or '-'}"]
    if "transiciones" in s:
        t = s["transiciones"]
        L += ["", "## Transiciones", "",
              f"- Cambios: {t['cambios']} | latencia mediana {num(t['latencia_mediana'])} s | "
              f"p90 {num(t['latencia_p90'])} s | sin reconocer: {t['sin_reconocer']} "
              "(incluye la reaccion humana)",
              f"- **Arranques ajenos** (comando que no es ni del gesto anterior ni del nuevo, en el "
              f"1er s): {t['arranques_ajenos']} ({t['seg_ajenos']:.2f} s en total)",
              f"- Eventos ajenos en la transicion: {t['eventos_ajenos'] or '-'}",
              f"- Exactitud estable fuera de la ventana de 1 s: {pct(t['acc_estable'])}",
              f"- Cambios mas lentos: {t['peores']}"]
    g = s["global"]
    L += ["", "## Seguridad (resumen)", "",
          f"- Puno leido como like (pose estable): {pct(g['puno_a_like'])}",
          f"- **Control REAL arrancando armado: s avanzando en clips de puno: "
          f"{num(g['seg_avance_puno_real'])}** | s moviendose en 'ninguno': "
          f"{num(s.get('ninguno', {}).get('seg_mov_real'))}",
          f"- Desarmes no deseados: 'ninguno' {num(g['desarmar_ninguno_min'])}/min | "
          f"transiciones {g['desarmar_trans']} | armados falsos en transiciones {g['armar_trans']}"]
    L += ["", "## Por variante (clips de gesto)", "",
          "| variante | clips | exact. estable | cmd peligroso | s avanzando (real) |",
          "|---|---|---|---|---|"]
    for v, d in sorted(s["variantes"].items()):
        L.append(f"| {v} | {d['clips']} | {pct(d['acc_estable'])} | {pct(d['cmd_peligro'])} | "
                 f"{num(d['seg_avance_real'])} |")
    L.append(tabla_confusion(r["conf_estable"], "Matriz de confusion, pose ESTABLE (ControlGestos)"))
    L.append(tabla_confusion(r["conf_cruda"], "Matriz de confusion, pose CRUDA por frame"))
    L += ["", "## Clips", "", "| clip | etiqueta | variante | frames | resumen |", "|---|---|---|---|---|"]
    L += [f"| {a} | {b} | {c} | {d} | {e} |" for a, b, c, d, e in r["clips"]]
    return "\n".join(L) + "\n"


FILAS_COMPARAR = [
    ("puno leido como LIKE (pose estable)", ("global", "puno_a_like"), pct, "-"),
    ("s AVANZANDO en clips de puno (control real)", ("global", "seg_avance_puno_real"), num, "-"),
    ("exactitud estable (media de gestos)", ("global", "acc_estable_macro"), pct, "+"),
    ("% frames con comando PELIGROSO (media)", ("global", "cmd_peligro_macro"), pct, "-"),
    ("arranques peligrosos en clips de gesto", ("global", "arranques_peligro"), str, "-"),
    ("falsos arranques/min en 'ninguno'", ("ninguno", "arranques_min"), num, "-"),
    ("s de movimiento falso/min en 'ninguno'", ("ninguno", "seg_mov_min"), num, "-"),
    ("armados falsos/min en 'ninguno'", ("ninguno", "armar_min"), num, "-"),
    ("acciones falsas/min en 'ninguno'", ("ninguno", "accion_min"), num, "-"),
    ("desarmes/min en 'ninguno' (seguro, molesto)", ("ninguno", "desarmar_min"), num, "-"),
    ("desarmes no deseados en transiciones", ("global", "desarmar_trans"), str, "-"),
    ("armados falsos en transiciones", ("global", "armar_trans"), str, "-"),
    ("acciones falsas en transiciones", ("global", "accion_trans"), str, "-"),
    ("latencia mediana transicion (s)", ("transiciones", "latencia_mediana"), num, "-"),
    ("latencia p90 transicion (s)", ("transiciones", "latencia_p90"), num, "-"),
    ("arranques ajenos en transiciones", ("transiciones", "arranques_ajenos"), str, "-"),
    ("cambios sin reconocer", ("transiciones", "sin_reconocer"), str, "-"),
]


def comparar(carpeta, etiquetas):
    res = []
    for e in etiquetas:
        with open(os.path.join(carpeta, "resultados", e + ".json"), encoding="utf-8") as f:
            res.append(json.load(f)["resumen"])
    L = ["| metrica | " + " | ".join(etiquetas) + " | mejor si |", "|---" * (len(etiquetas) + 2) + "|"]
    for nombre, (sec, key), fmt, mejor in FILAS_COMPARAR:
        vals = [r.get(sec, {}).get(key) for r in res]
        L.append(f"| {nombre} | " + " | ".join("-" if v is None else fmt(v) for v in vals) +
                 f" | {'mayor' if mejor == '+' else 'menor'} |")
    gestos = [g for g in bg.GESTOS_GRABACION if g != "ninguno" and any(g in r for r in res)]
    for g in gestos:
        vals = [r.get(g, {}) for r in res]
        L.append(f"| {g}: exact. estable / peligro | " + " | ".join(
            f"{pct(v.get('acc_estable'))} / {pct(v.get('cmd_peligro'))}" if v else "-"
            for v in vals) + " | mayor / menor |")
    return "\n".join(L)


def _json_ok(o):
    if isinstance(o, dict):
        return {str(k): _json_ok(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_json_ok(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o


def main():
    ap = argparse.ArgumentParser(description="Evaluar el banco de gestos")
    ap.add_argument("--dir", default=DIR_GRABACIONES)
    ap.add_argument("--etiqueta", default="base", help="nombre de esta corrida")
    ap.add_argument("--clahe", action="store_true", help="evaluar con --clahe")
    ap.add_argument("--modo", choices=list(MODOS), default="original",
                    help="original (lo de hoy), m1, m1m2 = mejorados, m1m2m7 (con --rechazar-mano-parcial)")
    ap.add_argument("--fps", type=float, default=None,
                    help="simular una camara mas lenta (solo <= fps grabado)")
    ap.add_argument("--comparar", nargs="+", metavar="ETIQUETA",
                    help="no evalua: compara corridas ya guardadas")
    args = ap.parse_args()
    if args.comparar:
        print(comparar(args.dir, args.comparar))
        return
    r = evaluar(args.dir, args.clahe, gt.ParamsGestos(**MODOS[args.modo]), args.fps)
    s = resumen(r)
    md = informe(r, s, args.etiqueta)
    os.makedirs(os.path.join(args.dir, "resultados"), exist_ok=True)
    base = os.path.join(args.dir, "resultados", args.etiqueta)
    with open(base + ".md", "w", encoding="utf-8") as f:
        f.write(md)
    with open(base + ".json", "w", encoding="utf-8") as f:
        json.dump(_json_ok(dict(resumen=s, clips=r["clips"], modo=args.modo, fps=args.fps)), f,
                  indent=1, ensure_ascii=False)
    print(md)
    print(f"Guardado en {base}.md / .json")


if __name__ == "__main__":
    main()
