"""Compara YuNet+SFace (actual) vs SCRFD+AdaFace (turtlebot_face) con deja-uno-fuera
sobre la misma galeria y la misma regla de decision (similitud maxima por persona).

  python comparar_modelos.py --tf RUTA_A/turtlebot_face/..   # carpeta que CONTIENE turtlebot_face/
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import facial  # noqa: E402


def fotos(carpeta):
    out = []
    for n in sorted(d for d in os.listdir(carpeta) if os.path.isdir(os.path.join(carpeta, d)) and d[0] not in "._"):
        for p in sorted(glob.glob(os.path.join(carpeta, n, "*"))):
            if p.lower().endswith(facial.IMG_EXT):
                out.append((n, p))
    return out


def embs_yunet(lista):
    r = facial.Rostros()
    res, t = [], []
    for _, p in lista:
        img = cv2.imread(p)
        fl = []
        for flip in (0, 1):
            im = cv2.flip(img, 1) if flip else img
            t0 = time.perf_counter()
            im, caras = r.detectar(im)
            e = r.embedding(im, caras[0]) if len(caras) else None
            t.append(time.perf_counter() - t0)
            fl.append(e)
        res.append(fl)
    return res, float(np.mean(t))


def embs_adaface(lista, tf):
    sys.path.insert(0, tf)
    from turtlebot_face.rostro import Detector, Embedder, alinear
    det, emb = Detector(), Embedder()
    res, t = [], []
    for _, p in lista:
        img = cv2.imread(p)
        fl = []
        for flip in (0, 1):
            im = cv2.flip(img, 1) if flip else img
            t0 = time.perf_counter()
            cajas, pts = det.detectar_cascada(im)
            e = emb.embeddings([alinear(im, pts[0])])[0] if len(cajas) else None
            t.append(time.perf_counter() - t0)
            fl.append(e)
        res.append(fl)
    return res, float(np.mean(t))


def embs_hibrido(lista, tf):
    """YuNet detecta (no falla con caras grandes) y AdaFace embebe, alineado con los 5 puntos de YuNet."""
    sys.path.insert(0, tf)
    from turtlebot_face.rostro import Embedder, alinear
    r, emb = facial.Rostros(), Embedder()
    res, t = [], []
    for _, p in lista:
        img = cv2.imread(p)
        fl = []
        for flip in (0, 1):
            im = cv2.flip(img, 1) if flip else img
            t0 = time.perf_counter()
            im, caras = r.detectar(im)
            e = emb.embeddings([alinear(im, caras[0][4:14].reshape(5, 2))])[0] if len(caras) else None
            t.append(time.perf_counter() - t0)
            fl.append(e)
        res.append(fl)
    return res, float(np.mean(t))


def loo(lista, embs, margen):
    nombres = sorted(set(n for n, _ in lista))
    ok = sin = dud = 0
    malos = []
    for i, (n, p) in enumerate(lista):
        q = embs[i][0]
        if q is None:
            sin += 1
            continue
        sims = {}
        for j, (m, _) in enumerate(lista):
            if j == i:
                continue
            for e in embs[j]:
                if e is not None:
                    sims[m] = max(sims.get(m, -1), float(e @ q))
        rk = sorted(sims.items(), key=lambda kv: -kv[1])
        if rk[0][0] == n:
            ok += 1
        else:
            malos.append((os.path.basename(p), rk[0][0], round(rk[0][1], 2)))
        if len(rk) > 1 and rk[0][1] - rk[1][1] < margen:
            dud += 1
    return ok, len(lista) - sin, sin, dud, malos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tf", required=True)
    ap.add_argument("--galeria", default=facial.GALERIA)
    a = ap.parse_args()
    lista = fotos(a.galeria)
    print(f"{len(lista)} fotos, {len(set(n for n, _ in lista))} personas")
    for nombre, f in (("YuNet+SFace", lambda: embs_yunet(lista)),
                      ("SCRFD+AdaFace", lambda: embs_adaface(lista, a.tf)),
                      ("YuNet+AdaFace (hibrido)", lambda: embs_hibrido(lista, a.tf))):
        embs, t = f()
        ok, con, sin, dud, malos = loo(lista, embs, facial.MARGEN if "Yu" in nombre else 0.04)
        print(f"\n== {nombre}: {ok}/{con} aciertos ({100*ok/max(con,1):.1f}%), sin cara {sin}, "
              f"dudosos(margen<0.04) {dud}, {1000*t:.0f} ms/imagen (deteccion+embedding)")
        for m in malos:
            print("   MAL", m)


if __name__ == "__main__":
    main()
