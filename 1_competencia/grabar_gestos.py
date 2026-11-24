"""
Graba clips etiquetados de gestos con la webcam de la laptop, para el banco de
evaluacion (evaluar_gestos.py). NO usa ROS ni mueve nada: solo graba.

Cada clip queda en grabaciones_gestos/<etiqueta>/ (carpeta local, junto a este script):
  <etiqueta>_<fecha>_<n>.avi   frames CRUDOS de la webcam (sin espejar ni dibujos)
  <etiqueta>_<fecha>_<n>.json  tiempo de cada frame, etiqueta, variante, y en los clips
                               de transiciones el gesto pedido en cada frame

Uso (en 1_competencia/):
  ../.venv/bin/python grabar_gestos.py --sesion            # guiado: todos los gestos
  ../.venv/bin/python grabar_gestos.py --gesto like --reps 3
  ../.venv/bin/python grabar_gestos.py --gesto ninguno --segundos 30
  ../.venv/bin/python grabar_gestos.py --gesto transiciones --cambios 12
  ../.venv/bin/python grabar_gestos.py --sesion --sin-mp-preview      # mas fps (~camara)
  ../.venv/bin/python grabar_gestos.py --sesion --con-movimiento      # gestos moviendo la mano

Teclas (ventana enfocada): ESPACIO grabar el siguiente clip | x descartar el ultimo
clip guardado | s saltar este clip | q / Esc salir (el clip en curso se descarta).
"""
import argparse
import json
import os
import random
import time

import cv2

import banco_gestos as bg
from gestos_teleop import DetectorMano
from mano_teleop import HAND_BONES, open_camera

HERE = os.path.dirname(os.path.abspath(__file__))
DIR_GRABACIONES = os.path.join(HERE, "grabaciones_gestos")
ORDEN_SESION = ["palma", "palma_separada", "puno", "like", "dislike", "pistola_izq",
                "pistola_der", "victoria", "ninguno", "transiciones"]


class Salir(Exception):
    pass


class ClipWriter:
    """Escribe un clip (video MJPG + JSON). discard() lo borra."""

    def __init__(self, carpeta, etiqueta, fps=30.0):
        os.makedirs(os.path.join(carpeta, etiqueta), exist_ok=True)
        base = time.strftime("%Y%m%d-%H%M%S")
        n = 0
        while True:
            stem = os.path.join(carpeta, etiqueta, f"{etiqueta}_{base}_{n}")
            if not os.path.exists(stem + ".avi"):
                break
            n += 1
        self.stem, self.etiqueta = stem, etiqueta
        self.vw = None
        self.fps = fps
        self.ts, self.objetivos = [], []

    def write(self, frame, t, objetivo=None):
        if self.vw is None:
            h, w = frame.shape[:2]
            self.vw = cv2.VideoWriter(self.stem + ".avi", cv2.VideoWriter_fourcc(*"MJPG"),
                                      self.fps, (w, h))
            if not self.vw.isOpened():
                raise SystemExit("No se pudo crear el video " + self.stem + ".avi")
        self.vw.write(frame)
        self.ts.append(round(float(t), 4))
        self.objetivos.append(objetivo)

    def close(self, meta):
        if self.vw is not None:
            self.vw.release()
        data = dict(meta, etiqueta=self.etiqueta, frames=len(self.ts), t=self.ts)
        if any(o is not None for o in self.objetivos):
            data["objetivo"] = self.objetivos
        with open(self.stem + ".json", "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)
        return self.stem

    def discard(self):
        if self.vw is not None:
            self.vw.release()
        for ext in (".avi", ".json"):
            if os.path.exists(self.stem + ext):
                os.remove(self.stem + ext)


def secuencia_transiciones(n, semilla):
    """n+1 objetivos al azar, sin repetir el mismo dos veces seguidas."""
    rnd = random.Random(semilla)
    seq = [rnd.choice(bg.OBJETIVOS_TRANSICION)]
    while len(seq) < n + 1:
        o = rnd.choice(bg.OBJETIVOS_TRANSICION)
        if o != seq[-1]:
            seq.append(o)
    return seq


def texto_objetivo(o):
    return bg.GESTOS_GRABACION[o][1]


def pantalla(frame, lineas, color=(255, 255, 255), rec=False):
    """Vista previa espejada con esqueleto y texto (lo grabado NO lleva esto)."""
    w = frame.shape[1]
    cv2.rectangle(frame, (0, 0), (w, 22 + 26 * len(lineas)), (0, 0, 0), -1)
    for i, (txt, escala) in enumerate(lineas):
        cv2.putText(frame, txt, (12, 24 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, escala,
                    color if i == 0 else (220, 220, 220), 2 if i == 0 else 1)
    if rec:
        cv2.circle(frame, (w - 25, 25), 10, (0, 0, 255), -1)


class Grabador:
    def __init__(self, args):
        self.args = args
        self.cap = open_camera(args.cam)
        if self.cap is None:
            raise SystemExit(f"No se pudo abrir la webcam {args.cam} (prueba --cam 1).")
        # sin MediaPipe en la vista previa se graba mas cerca del fps de la camara
        self.det = None if args.sin_mp_preview else DetectorMano()
        self.win = "Grabar gestos (banco)"
        self.ultimo = None

    def leer(self):
        ret, raw = self.cap.read()
        if not ret:
            raise SystemExit("La webcam dejo de entregar imagenes.")
        vista = cv2.flip(raw, 1)
        lm = self.det.find(vista) if self.det is not None else None
        if lm is not None:
            for a, b in HAND_BONES:
                cv2.line(vista, tuple(lm[a].astype(int)), tuple(lm[b].astype(int)), (255, 100, 0), 2)
        return raw, vista

    def mostrar(self, vista):
        cv2.imshow(self.win, vista)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            raise Salir()
        return k

    def esperar(self, lineas):
        """Vista previa hasta ESPACIO. Devuelve 'grabar' o 'saltar'."""
        while True:
            _, vista = self.leer()
            pantalla(vista, lineas + [("ESPACIO grabar | s saltar | x borrar ultimo | q salir", 0.45)],
                     (0, 255, 255))
            k = self.mostrar(vista)
            if k == ord(" "):
                return "grabar"
            if k == ord("s"):
                return "saltar"
            if k == ord("x") and self.ultimo:
                for ext in (".avi", ".json"):
                    if os.path.exists(self.ultimo + ext):
                        os.remove(self.ultimo + ext)
                print("Descartado:", os.path.relpath(self.ultimo, HERE))
                self.ultimo = None

    def cuenta(self, lineas):
        t0 = time.monotonic()
        while (r := self.args.cuenta - (time.monotonic() - t0)) > 0:
            _, vista = self.leer()
            pantalla(vista, lineas + [(f"Empieza en {r:.1f} s", 0.7)], (0, 255, 255))
            self.mostrar(vista)

    def grabar(self, etiqueta, segundos, variante, objetivos=None, seg_cambio=None, semilla=None):
        """Graba un clip; objetivos = secuencia (solo transiciones)."""
        w = ClipWriter(self.args.dir, etiqueta)
        t0 = time.monotonic()
        try:
            while (t := time.monotonic() - t0) < segundos:
                raw, vista = self.leer()
                obj = None
                if objetivos:
                    obj = objetivos[min(int(t // seg_cambio), len(objetivos) - 1)]
                    lineas = [(f"HAZ: {obj.upper()}", 0.9), (texto_objetivo(obj), 0.5)]
                else:
                    lineas = [(f"GRABANDO {etiqueta.upper()}  {segundos - t:.1f} s", 0.8),
                              (texto_objetivo(etiqueta), 0.5), (f"variante: {variante}", 0.5)]
                w.write(raw, t, obj)
                pantalla(vista, lineas, (0, 0, 255), rec=True)
                self.mostrar(vista)
        except (Salir, KeyboardInterrupt, SystemExit):
            w.discard()
            raise
        meta = dict(variante=variante, persona=self.args.persona, cam=self.args.cam,
                    fecha=time.strftime("%Y-%m-%d %H:%M:%S"),
                    instruccion=None if objetivos else texto_objetivo(etiqueta))
        if objetivos:
            meta.update(seg_cambio=seg_cambio, semilla=semilla)
        self.ultimo = w.close(meta)
        fps = len(w.ts) / max(w.ts[-1], 1e-6) if len(w.ts) > 1 else 0
        print(f"Guardado {os.path.relpath(self.ultimo, HERE)}.avi  ({len(w.ts)} frames, ~{fps:.0f} fps)")

    def clip(self, etiqueta, rep):
        a = self.args
        if etiqueta == bg.TRANSICIONES:
            semilla = a.semilla + rep
            objetivos = secuencia_transiciones(a.cambios, semilla)
            lineas = [(f"TRANSICIONES ({rep + 1}/{a.reps_trans})", 0.8),
                      (f"Cambia al gesto que pida la pantalla (cada {a.seg_cambio:.0f} s)", 0.5),
                      ("Haz el cambio natural, sin bajar la mano entre gestos", 0.5)]
            if self.esperar(lineas) == "saltar":
                return
            self.cuenta(lineas)
            self.grabar(etiqueta, a.seg_cambio * len(objetivos), "transiciones",
                        objetivos, a.seg_cambio, semilla)
            return
        variante = bg.VARIANTES[rep % len(bg.VARIANTES)]
        if a.con_movimiento:
            variante = bg.VARIANTE_MOVIMIENTO
        seg = a.seg_ninguno if etiqueta == "ninguno" else a.segundos
        reps = 1 if etiqueta == "ninguno" else a.reps
        lineas = [(f"{etiqueta.upper()}  ({rep + 1}/{reps}, {seg:.0f} s)", 0.8),
                  (texto_objetivo(etiqueta), 0.5),
                  (f"variante: {variante if etiqueta != 'ninguno' else 'libre'}", 0.5)]
        if self.esperar(lineas) == "saltar":
            return
        self.cuenta(lineas)
        self.grabar(etiqueta, seg, variante if etiqueta != "ninguno" else "libre")

    def close(self):
        self.cap.release()
        if self.det is not None:
            self.det.close()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="Grabar clips etiquetados para el banco de gestos")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sesion", action="store_true", help="todos los gestos, guiado")
    g.add_argument("--gesto", choices=list(bg.GESTOS_GRABACION) + [bg.TRANSICIONES])
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--dir", default=DIR_GRABACIONES, help="carpeta de salida (local)")
    ap.add_argument("--reps", type=int, default=2, help="clips por gesto (variantes distintas)")
    ap.add_argument("--segundos", type=float, default=6.0, help="duracion de cada clip de gesto")
    ap.add_argument("--seg-ninguno", type=float, default=30.0)
    ap.add_argument("--cambios", type=int, default=12, help="cambios por clip de transiciones")
    ap.add_argument("--seg-cambio", type=float, default=3.0)
    ap.add_argument("--reps-trans", type=int, default=2, help="clips de transiciones")
    ap.add_argument("--semilla", type=int, default=1)
    ap.add_argument("--cuenta", type=float, default=2.0, help="s de cuenta atras antes de grabar")
    ap.add_argument("--persona", default="p1", help="etiqueta libre de quien graba (local)")
    ap.add_argument("--sin-mp-preview", action="store_true",
                    help="vista previa sin MediaPipe (graba a mas fps; no dibuja la mano)")
    ap.add_argument("--con-movimiento", action="store_true",
                    help="variante 'con_movimiento': mantener el gesto moviendo la mano natural")
    args = ap.parse_args()
    if args.segundos <= 0 or args.seg_ninguno <= 0 or args.seg_cambio <= 0:
        raise SystemExit("Las duraciones deben ser > 0")

    plan = ORDEN_SESION if args.sesion else [args.gesto]
    print("Grabaciones en:", os.path.abspath(args.dir))
    print("Haz CLIC en la ventana; ESPACIO para grabar cada clip.")
    gr = Grabador(args)
    try:
        for etq in plan:
            if etq == bg.TRANSICIONES:
                n = args.reps_trans
            elif etq == "ninguno":
                n = 1
            else:
                n = args.reps
            for rep in range(n):
                gr.clip(etq, rep)
    except (Salir, KeyboardInterrupt):
        print("\nSalida: el clip en curso (si habia) se descarto.")
    finally:
        gr.close()


if __name__ == "__main__":
    main()
