"""
Gesto V (ACCION) -> reconocimiento de rostro en gestos_teleop.py (opcion --v-rostro).

No duplica nada: usa facial.Rostros / Galeria / identificar / Anunciador y el mismo
escaneo de la Tarea B (mision.Escaneo, 5 s, sin girar).

- Fuente de las caras: la camara del ROBOT. Solo en --dry --no-robot-cam se usa la webcam
  de la laptop como SUSTITUTO DE PRUEBA (se avisa en pantalla).
- Mientras escanea, limitar() devuelve lin=0, ang=0.
- Sin cara -> "sin cara" (aqui NO se anuncia el primer nombre de la galeria).
- Cualquier fallo (modelo, galeria, camara, error) queda como mensaje: nunca lanza.
- Registro: publica en /nombre_reconocido (solo si hay nodo ROS, es decir no --dry),
  guarda un JSON en resultados/ y la foto usada (JPG) en logs/.
"""
import json
import os
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
DIR_RESULTADOS = os.path.join(HERE, "resultados")
DIR_LOGS = os.path.join(HERE, "logs")
LIMITE_S = 5.0      # como la Tarea B (mision.tarea_b: Escaneo(..., limite=5.0))
MOSTRAR_S = 6.0     # s que el resultado queda en pantalla

FUENTE_ROBOT = "robot"
FUENTE_PRUEBA = "webcam_prueba"   # solo --dry --no-robot-cam
TEXTO_PRUEBA = "MODO PRUEBA: rostro con la WEBCAM (en el robot se usa su camara)"


def fuente_rostro(dry, no_robot_cam):
    """De donde salen las caras: robot si hay su camara; webcam solo en --dry --no-robot-cam."""
    if not no_robot_cam:
        return FUENTE_ROBOT
    return FUENTE_PRUEBA if dry else None


class AccionRostro:
    """Estado: 'inactivo' | 'buscando' | 'reconocido' | 'dudoso' | 'sin_cara' |
    'galeria_vacia' | 'error'. iniciar(t) al evento 'accion'; paso(t, img) cada frame."""

    def __init__(self, rostros, galeria, anunciador=None, fuente=FUENTE_ROBOT, umbral=None,
                 dir_resultados=DIR_RESULTADOS, dir_logs=DIR_LOGS, limite=LIMITE_S,
                 error_inicial=None):
        self.rostros, self.galeria, self.anunciador = rostros, galeria, anunciador
        self.fuente, self.limite = fuente, limite
        self.umbral = umbral
        self.dir_resultados, self.dir_logs = dir_resultados, dir_logs
        self.error_inicial = error_inicial
        self.estado, self.mensaje, self.t_msg = "inactivo", "", -1e9
        self.escaneo, self.caras, self.ultimo = None, [], None

    @classmethod
    def crear(cls, node=None, voz="ninguna", fuente=FUENTE_ROBOT, galeria_dir=None, **kw):
        """Carga modelos y galeria. Si algo falla devuelve una instancia deshabilitada
        (iniciar() solo mostrara el error)."""
        try:
            import facial
            rostros = facial.Rostros()
            rostros.calentar()
            galeria = facial.Galeria(rostros, galeria_dir or facial.GALERIA)
            # Sin nodo ROS (--dry) no se publica ni se anuncia: solo se muestra
            anunciador = facial.Anunciador(node, voz=voz) if node is not None else None
            print(f"[v-rostro] fuente: {fuente or 'NINGUNA'} | galeria: "
                  f"{galeria.resumen() or 'VACIA'}")
            return cls(rostros, galeria, anunciador, fuente, **kw)
        except (SystemExit, Exception) as e:   # facial.Rostros usa SystemExit si falta un modelo
            msg = f"reconocimiento no disponible: {e}"
            print(f"[v-rostro] {msg}")
            return cls(None, None, None, fuente, error_inicial=msg, **kw)

    # --- ciclo ---------------------------------------------------------------------

    def activa(self):
        return self.estado == "buscando"

    def limitar(self, linear, angular):
        """Mientras dura la accion el robot queda quieto."""
        return (0.0, 0.0) if self.activa() else (linear, angular)

    def iniciar(self, t):
        if self.activa():
            return
        self.caras, self.ultimo = [], None
        if self.error_inicial:
            return self._fin(t, "error", self.error_inicial)
        if self.fuente is None:
            return self._fin(t, "error", "sin camara para rostros (falta la del robot)")
        try:
            import mision
            self.escaneo = mision.Escaneo(self.rostros, mision.MisionParams(), t,
                                          buscar=False, limite=self.limite)
            self.estado, self.mensaje, self.t_msg = "buscando", "buscando rostro...", t
        except Exception as e:
            self._fin(t, "error", f"error al iniciar: {e}")

    def paso(self, t, img):
        """img: frame NUEVO de la fuente (o None si no llego ninguno)."""
        if not self.activa():
            return
        try:
            if img is not None:
                self.ultimo = img
                _, self.caras = self.rostros.detectar(img)
            _, frames = self.escaneo.paso(t, img)
            n = len(self.escaneo.frames)
            self.mensaje = f"buscando rostro... {n}/{self.escaneo.mp.escaneo_frames} frames con cara"
            if frames is not None:
                self._terminar(t, frames)
        except Exception as e:
            self._fin(t, "error", f"error en el reconocimiento: {e}")

    def _terminar(self, t, frames):
        import facial
        umbral = self.umbral if self.umbral is not None else facial.UMBRAL
        res = facial.identificar(frames, self.rostros, self.galeria, umbral)
        if res.n_caras == 0:
            estado, msg = "sin_cara", "sin cara"
        elif res.nombre is None:
            estado, msg = "galeria_vacia", f"cara vista ({res.n_caras} frames) pero la galeria esta vacia"
        else:
            estado = "reconocido" if res.seguro else "dudoso"
            msg = f"{res.nombre.replace('_', ' ')}  {res.score:.2f}  {'SEGURO' if res.seguro else 'DUDOSO'}"
        foto = self._mejor_foto(frames)
        guardado = self._guardar(res, estado, foto)
        if res.nombre is not None and self.anunciador is not None:
            try:
                self.anunciador.anunciar(res.nombre)
            except Exception as e:
                msg += f" (no se pudo anunciar: {e})"
        if guardado:
            msg += f"  [{guardado}]"
        print(f"[v-rostro] {msg}")
        self._fin(t, estado, msg)

    def _fin(self, t, estado, msg):
        self.estado, self.mensaje, self.t_msg = estado, msg, t

    # --- registro ------------------------------------------------------------------

    def _mejor_foto(self, frames):
        if not frames:
            return None
        import facial
        best, best_q = frames[0], -1.0
        for f in frames:
            img, faces = self.rostros.detectar(f)
            if len(faces):
                q = facial.calidad(img, faces[0])
                if q > best_q:
                    best, best_q = f, q
        return best

    def _guardar(self, res, estado, foto):
        """JSON en resultados/ y JPG en logs/. Devuelve un texto corto o '' si fallo."""
        stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{int(time.time() * 1000) % 1000:03d}"
        foto_path = None
        try:
            if foto is not None:
                os.makedirs(self.dir_logs, exist_ok=True)
                foto_path = os.path.join(self.dir_logs, f"rostro_{stamp}.jpg")
                if not cv2.imwrite(foto_path, foto):
                    foto_path = None
            os.makedirs(self.dir_resultados, exist_ok=True)
            data = dict(nombre=res.nombre, score=round(float(res.score), 3), seguro=bool(res.seguro),
                        estado=estado, hora=time.strftime("%Y-%m-%d %H:%M:%S"),
                        n_caras=res.n_caras, ranking=[(n, round(s, 3)) for n, s in res.ranking[:3]],
                        fuente=self.fuente, publicado=self.anunciador is not None and res.nombre is not None,
                        foto=os.path.relpath(foto_path, HERE) if foto_path else None)
            with open(os.path.join(self.dir_resultados, f"rostro_{stamp}.json"), "w",
                      encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
            return f"guardado rostro_{stamp}"
        except Exception as e:
            print(f"[v-rostro] no se pudo guardar el resultado: {e}")
            return ""

    # --- pantalla ------------------------------------------------------------------

    def lineas(self, t):
        """Texto a mostrar (vacio si no hay nada reciente)."""
        out = []
        if self.fuente == FUENTE_PRUEBA:
            out.append(TEXTO_PRUEBA)
        if self.activa() or t - self.t_msg < MOSTRAR_S:
            if self.mensaje:
                out.append(f"V/ROSTRO: {self.mensaje}")
        return out

    def dibujar(self, frame, t, cajas=False):
        """Texto abajo; con cajas=True (fuente = este mismo frame) marca las caras vistas."""
        if cajas and self.activa():
            for f in self.caras:
                x, y, w, h = (int(v) for v in f[:4])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        lineas = self.lineas(t)
        if not lineas:
            return
        h, w = frame.shape[:2]
        y0 = h - 10 - 24 * len(lineas)
        cv2.rectangle(frame, (0, y0 - 6), (w, h), (0, 0, 0), -1)
        for i, txt in enumerate(lineas):
            color = (0, 165, 255) if txt == TEXTO_PRUEBA else (
                (0, 255, 0) if self.estado == "reconocido" else (0, 255, 255))
            cv2.putText(frame, txt, (10, y0 + 16 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def cerrar(self):
        if self.anunciador is not None:
            try:
                self.anunciador.cerrar()
            except Exception:
                pass
