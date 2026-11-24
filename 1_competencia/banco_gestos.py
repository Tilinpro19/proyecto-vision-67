"""
Banco de evaluacion de los gestos de la Tarea B (gestos_teleop.py): piezas sin camara
ni ROS que usan grabar_gestos.py, evaluar_gestos.py y los tests.

- GESTOS_GRABACION: que clips se graban y que pose se espera en cada uno.
- margenes(lm): para cada pose, un margen firmado (>0 solo para la pose que elegiria
  classify_pose). Mide cuanto le falta a la mano para cruzar el umbral de otra pose.
- simular_control(): pasa la secuencia de poses por ControlGestos como si estuviera
  ARMADO (para ver que comando saldria) y DESARMADO (para ver si armaria).
- metricas_*(): exactitud, comandos peligrosos, falsos comandos/min, latencias...
"""
from collections import Counter

import numpy as np

import gestos_teleop as gt

# etiqueta del clip -> (pose esperada, instruccion en pantalla). None = ninguna pose.
GESTOS_GRABACION = {
    "like": ("like", "Pulgar ARRIBA, resto de dedos cerrados"),
    "dislike": ("dislike", "Pulgar ABAJO, resto de dedos cerrados"),
    "pistola_izq": ("pistola_izq", "Indice estirado apuntando a TU izquierda, resto cerrados"),
    "pistola_der": ("pistola_der", "Indice estirado apuntando a TU derecha, resto cerrados"),
    "palma": ("palma", "Palma abierta a la camara, dedos JUNTOS"),
    "palma_separada": ("palma", "Palma abierta, dedos SEPARADOS y pulgar abierto"),
    "puno": ("puno", "Puno cerrado"),
    "victoria": ("victoria", "V con indice y medio, resto cerrados"),
    "ninguno": (None, "SIN gestos: mueve la mano natural, tocate la cara, sal y entra del cuadro"),
}
TRANSICIONES = "transiciones"
# objetivos posibles dentro de un clip de transiciones
OBJETIVOS_TRANSICION = ["like", "dislike", "pistola_izq", "pistola_der", "palma", "puno",
                        "victoria", "ninguno"]
VARIANTES = ["normal, a ~50 cm", "mas lejos, ~1-1.5 m", "muneca girada / mano inclinada",
             "con la OTRA mano"]

VARIANTE_MOVIMIENTO = ("con_movimiento: manten el gesto pero mueve la mano natural "
                       "(como si hablaras/senalaras), sin exagerar")

SIN_MANO = "SIN_MANO"     # pose cruda None: MediaPipe no ve mano
SIN_POSE = "-"            # pose estable None: sin mano o aun no estable
EVENTO_ESPERADO = {"palma": "armar", "puno": "desarmar", "victoria": "accion"}
CERCA_UMBRAL = 0.10       # |margen| menor que esto = la pose esta al borde de cambiar
POSES_REALES = [p for p in gt.POSES if p != "nada"]


def pose_esperada(etiqueta):
    """Etiqueta de clip u objetivo de transicion -> pose esperada (None = ninguna)."""
    return GESTOS_GRABACION[etiqueta][0]


def direccion_esperada(etiqueta):
    p = pose_esperada(etiqueta)
    return gt.POSES[p][1] if p else (0, 0)


def acierto(etiqueta, pose):
    """La pose (cruda o estable) es la correcta para esta etiqueta. En 'ninguno' vale
    sin mano o 'nada'."""
    p = pose_esperada(etiqueta)
    return pose in (None, "nada") if p is None else pose == p


# --- margenes de la clasificacion ---------------------------------------------------

def margenes(lm, like_pulgar_min=None):
    """lm (21, 2) px -> {pose: margen}. Replica las reglas de classify_pose con valores
    firmados (Y logico = min, NO = -x): solo la pose elegida queda > 0. Unidades:
    fracciones del tamano de la mano (muneca -> base del medio), aproximadas.
    like_pulgar_min: igual que en classify_pose (M1)."""
    lm = np.asarray(lm, np.float64)
    eps = 1e-9
    wrist = lm[0]
    size = float(np.linalg.norm(lm[9] - wrist))
    if size < eps:
        return {**{p: -1.0 for p in POSES_REALES}, "nada": 1.0}
    s_size = size / gt.MIN_HAND_PX - 1.0

    f = []  # >0 = dedo doblado (mismas constantes 1.15 / 1.30 que classify_pose)
    for (tip, pip), mcp in zip(gt.FINGERS, gt.MCP_JOINTS):
        r1 = np.linalg.norm(lm[tip] - wrist) / (np.linalg.norm(lm[pip] - wrist) * 1.15 + eps)
        r2 = np.linalg.norm(lm[tip] - lm[mcp]) / (np.linalg.norm(lm[pip] - lm[mcp]) * 1.30 + eps)
        f.append(1.0 - min(r1, r2))
    f0, f1, f2, f3 = f

    dx, dy = lm[4] - lm[2]
    vert = (abs(dy) - 0.6 * abs(dx)) / size
    up = min(-dy / size - 0.25, (lm[5][1] - lm[4][1]) / size, vert)
    down = min(dy / size - 0.25, (lm[4][1] - lm[2][1]) / size, vert)
    sep = np.linalg.norm(lm[4] - lm[5]) / size
    topen = sep - gt.THUMB_OPEN
    if like_pulgar_min is not None:  # M1: like exige ademas el pulgar separado
        up = min(up, sep - like_pulgar_min)
    p_dx, p_dy = lm[8] - lm[5]
    hz, dxs = (abs(p_dx) - abs(p_dy)) / size, p_dx / size

    af = min(f0, f1, f2)                        # (meñique: implicito con los otros 3)
    b2 = min(-af, -f0, f1, f2, f3)              # rama pistola
    b3 = min(-af, -b2, -f0, -f1, f2, f3)        # victoria
    b4 = min(-af, -b2, -b3, -f0, -f1, -f2, -f3, topen)  # palma
    m = {"like": min(af, up), "dislike": min(af, -up, down), "puno": min(af, -up, -down),
         "pistola_der": min(b2, hz, dxs), "pistola_izq": min(b2, hz, -dxs),
         "victoria": b3, "palma": b4}
    m = {k: float(min(v, s_size)) for k, v in m.items()}
    m["nada"] = -max(m.values())
    return m


def pose_de_margenes(m):
    best = max(POSES_REALES, key=lambda p: m[p])
    return best if m[best] > 0 else "nada"


def margen_y_rival(m):
    """-> (pose, margen de esa pose, pose rival mas cercana, margen del rival)."""
    pose = pose_de_margenes(m)
    otras = [p for p in POSES_REALES if p != pose]
    rival = max(otras, key=lambda p: m[p])
    return pose, m[pose], rival, m[rival]


# --- control temporal ---------------------------------------------------------------

def simular_control(poses, ts, nuevo_ctrl=gt.ControlGestos):
    """Secuencia de poses crudas -> por frame: pose estable, direccion SI ESTUVIERA
    ARMADO y eventos {'armar', 'desarmar', 'accion'} que dispararia."""
    armado, desarmado = nuevo_ctrl(), nuevo_ctrl()
    held, dirs, eventos = [], [], []
    prev_puno = False
    for p, t in zip(poses, ts):
        armado.armed = True
        d, e1 = armado.update(p, t)
        desarmado.armed = False
        _, e2 = desarmado.update(p, t)
        ev = set()
        if e2 == "armado":
            ev.add("armar")
        if e1 == "accion":
            ev.add("accion")
        puno = e1 == "desarmado"
        if puno and not prev_puno:
            ev.add("desarmar")
        prev_puno = puno
        held.append(armado.held)
        dirs.append(tuple(d))
        eventos.append(ev)
    return held, dirs, eventos


# --- metricas -----------------------------------------------------------------------

def _arranques(mask):
    """Numero de tramos True (cada tramo = un arranque de comando)."""
    n, prev = 0, False
    for v in mask:
        if v and not prev:
            n += 1
        prev = v
    return n


def _dur(ts):
    return max(ts[-1] - ts[0], 1e-6) if len(ts) > 1 else 1e-6


def _dt(ts):
    """Duracion de cada frame (hasta el siguiente; el ultimo copia el anterior)."""
    if len(ts) < 2:
        return [0.0] * len(ts)
    d = list(np.diff(ts))
    return d + [d[-1]]


def _seg_real(fr, cond):
    """s en que el control REAL (arrancando armado) manda un comando que cumple cond."""
    ts = [x["t"] for x in fr]
    return sum(dt for dt, x in zip(_dt(ts), fr) if x.get("dir_real") and cond(x["dir_real"]))


def _cambios(held):
    return sum(1 for a, b in zip(held, held[1:]) if a != b)


def metricas_gesto(fr, etiqueta):
    """fr: lista de dicts por frame (t, cruda, estable, dir, eventos, margen, rival).
    Clip donde se mantiene UNA pose."""
    esp, E = pose_esperada(etiqueta), direccion_esperada(etiqueta)
    ts = [x["t"] for x in fr]
    n = len(fr)
    peligro = [x["dir"] != (0, 0) and x["dir"] != E for x in fr]
    lat = next((x["t"] - ts[0] for x in fr if x["estable"] == esp), None)
    ok_m = [x["margen"] for x in fr if x["cruda"] == esp and x["margen"] is not None]
    det_m = [x["margen"] for x in fr if x["margen"] is not None]
    rivales = Counter(x["rival"] for x in fr if x["cruda"] == esp and x["rival"])
    ev = Counter(e for x in fr for e in x["eventos"])
    inesperados = {k: v for k, v in ev.items() if EVENTO_ESPERADO.get(esp) != k}
    return dict(
        frames=n, seg=_dur(ts),
        deteccion=sum(x["cruda"] is not None for x in fr) / n,
        acc_cruda=sum(x["cruda"] == esp for x in fr) / n,
        acc_estable=sum(x["estable"] == esp for x in fr) / n,
        cmd_ok=sum(x["dir"] == E for x in fr) / n,
        cmd_peligro=sum(peligro) / n,
        arranques_peligro=_arranques(peligro),
        eventos_inesperados=inesperados,
        evento_esperado=ev.get(EVENTO_ESPERADO.get(esp), 0) if esp in EVENTO_ESPERADO else None,
        latencia=lat,
        margen_mediana=float(np.median(ok_m)) if ok_m else None,
        cerca_umbral=(sum(abs(m) < CERCA_UMBRAL for m in det_m) / len(det_m)) if det_m else None,
        rival=rivales.most_common(1)[0][0] if rivales else None,
        parpadeos_min=_cambios([x["estable"] for x in fr]) / (_dur(ts) / 60),
        seg_avance_real=_seg_real(fr, lambda d: d[0] > 0),
        seg_mov_real=_seg_real(fr, lambda d: d != (0, 0)),
    )


def metricas_ninguno(fr):
    ts = [x["t"] for x in fr]
    minutos = _dur(ts) / 60
    mueve = [x["dir"] != (0, 0) for x in fr]
    ev = Counter(e for x in fr for e in x["eventos"])
    poses = Counter(x["estable"] for x in fr if x["estable"] not in (None, "nada"))
    return dict(
        frames=len(fr), seg=_dur(ts),
        deteccion=sum(x["cruda"] is not None for x in fr) / len(fr),
        acc_estable=sum(acierto("ninguno", x["estable"]) for x in fr) / len(fr),
        arranques=_arranques(mueve),
        seg_mov=sum(dt for dt, m in zip(_dt(ts), mueve) if m),
        armar=ev.get("armar", 0), accion=ev.get("accion", 0), desarmar=ev.get("desarmar", 0),
        minutos=minutos, poses_falsas=dict(poses),
        seg_mov_real=_seg_real(fr, lambda d: d != (0, 0)),
    )


def metricas_transiciones(fr, gracia=1.0):
    """fr con 'objetivo' por frame. Para cada cambio A->B: latencia hasta que la pose
    estable es B, comandos ajenos (ni de A ni de B) en los 'gracia' s siguientes y
    exactitud fuera de esa ventana."""
    ts = [x["t"] for x in fr]
    cambios, i0 = [], 0
    for i in range(1, len(fr)):
        if fr[i]["objetivo"] != fr[i - 1]["objetivo"]:
            cambios.append((i0, i))
            i0 = i
    cambios.append((i0, len(fr)))
    res, estables_ok, estables_n = [], 0, 0
    for k, (a, b) in enumerate(cambios):
        obj = fr[a]["objetivo"]
        t0 = ts[a]
        for x in fr[a:b]:
            if x["t"] >= t0 + gracia:
                estables_n += 1
                estables_ok += acierto(obj, x["estable"])
        if k == 0:
            continue
        prev = fr[a - 1]["objetivo"]
        permitidas = {direccion_esperada(prev), direccion_esperada(obj), (0, 0)}
        ventana = [x for x in fr[a:b] if x["t"] < t0 + gracia]
        ajeno = [x["dir"] not in permitidas for x in ventana]
        lat = next((x["t"] - t0 for x in fr[a:b] if acierto(obj, x["estable"])), None)
        ev = [e for x in ventana for e in x["eventos"] if EVENTO_ESPERADO.get(pose_esperada(obj)) != e]
        res.append(dict(de=prev, a=obj, latencia=lat, arranques_ajenos=_arranques(ajeno),
                        seg_ajenos=sum(dt for dt, m in zip(_dt([x["t"] for x in ventana]), ajeno) if m),
                        eventos_ajenos=ev))
    return dict(cambios=res, acc_estable=(estables_ok / estables_n) if estables_n else None,
                seg=_dur(ts))


def matriz_confusion(filas):
    """filas: [(etiqueta, pose)] -> {etiqueta: Counter(pose)}."""
    mat = {}
    for etq, pose in filas:
        mat.setdefault(etq, Counter())[pose] += 1
    return mat
