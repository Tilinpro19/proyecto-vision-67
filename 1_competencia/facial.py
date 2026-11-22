"""
Reconocimiento facial de la foto en B y anuncio del nombre (Tareas A y B).

OpenCV YuNet (deteccion) + SFace (embedding de 128, similitud coseno). Modelos en
modelos/ (sin internet en el lab). Galeria: galeria/<nombre>/*.jpg; los embeddings se
guardan en galeria/.cache.npz, asi cargar una galeria nueva el mismo dia toma segundos.

Uso:
  python3 facial.py --probar                  # deja-uno-fuera sobre la galeria: aciertos
  python3 facial.py --img foto1.jpg foto2.jpg # identifica imagenes sueltas
  python3 facial.py --robot                   # en vivo con la camara del robot
  python3 facial.py --capturar Juan --n 15    # guarda fotos del robot en galeria/Juan/
  python3 facial.py --decir "Hola"            # prueba la voz
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
YUNET = os.path.join(HERE, "modelos", "face_detection_yunet_2023mar.onnx")
SFACE = os.path.join(HERE, "modelos", "face_recognition_sface_2021dec.onnx")
GALERIA = os.path.join(HERE, "galeria")
IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")

UMBRAL = 0.363        # coseno recomendado por OpenCV para SFace (misma persona si >=)
MARGEN = 0.04         # ventaja minima sobre el segundo para decir "seguro"
SCORE_MIN = 0.6       # confianza minima de YuNet
MIN_SIDE = 480        # imagenes mas chicas se agrandan (el preview del OAK-D mide 250 px)
MAX_SIDE = 960        # imagenes muy grandes se reducen (fotos de celular) para acelerar YuNet
TOP_FRAMES = 3        # se promedian los mejores frames


@dataclass
class Resultado:
    nombre: str = None          # None solo si no se vio ninguna cara
    score: float = 0.0          # similitud coseno media del elegido
    seguro: bool = False        # score >= umbral y con margen sobre el segundo
    ranking: list = field(default_factory=list)  # [(nombre, score)] de mayor a menor
    n_caras: int = 0            # frames con cara


class Rostros:
    """YuNet + SFace. detectar(img) -> (img_usada, caras Nx15 de mayor a menor area)."""

    def __init__(self, score_min=SCORE_MIN):
        for m in (YUNET, SFACE):
            if not os.path.isfile(m):
                raise SystemExit(f"Falta el modelo {m} (opencv_zoo, ver README).")
        self.det = cv2.FaceDetectorYN.create(YUNET, "", (320, 320), score_min, 0.3, 20)
        self.rec = cv2.FaceRecognizerSF.create(SFACE, "")

    def calentar(self, ancho=640, alto=480):
        """Una deteccion en vacio: la primera inferencia de OpenCV DNN es mucho mas lenta y,
        si cae en B, retrasa el escaneo. Mejor pagarla en A, antes de salir."""
        try:
            self.detectar(np.zeros((alto, ancho, 3), np.uint8))
        except Exception:
            pass

    def detectar(self, img):
        h, w = img.shape[:2]
        m = max(h, w)
        if m < MIN_SIDE:
            k = MIN_SIDE / m
            img = cv2.resize(img, (round(w * k), round(h * k)), interpolation=cv2.INTER_CUBIC)
        elif m > MAX_SIDE:
            k = MAX_SIDE / m
            img = cv2.resize(img, (round(w * k), round(h * k)), interpolation=cv2.INTER_AREA)
        h, w = img.shape[:2]
        self.det.setInputSize((w, h))
        _, faces = self.det.detect(img)
        if faces is None:
            return img, np.empty((0, 15), np.float32)
        return img, faces[np.argsort(-faces[:, 2] * faces[:, 3])]

    def embedding(self, img, face):
        f = self.rec.feature(self.rec.alignCrop(img, face)).ravel()
        return f / (np.linalg.norm(f) + 1e-9)


def calidad(img, face):
    """Mas alto = mejor frame: confianza, tamano de la cara y nitidez."""
    x, y, w, h = (int(v) for v in face[:4])
    crop = img[max(y, 0):y + h, max(x, 0):x + w]
    if crop.size == 0:
        return 0.0
    sharp = cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    return float(face[14] * min(w / 80.0, 1.0) * min(sharp / 100.0, 1.0))


class Galeria:
    """Embeddings de galeria/<nombre>/*.jpg (y su espejo). Cache por ruta + fecha + tamano."""

    def __init__(self, rostros, carpeta=GALERIA):
        self.rostros, self.carpeta = rostros, carpeta
        self.nombres, self.etiquetas, self.embs = [], np.empty(0, int), np.empty((0, 128))
        self.fuentes, self.sin_cara = [], []
        self.cargar()

    def cargar(self):
        cache_path = os.path.join(self.carpeta, ".cache.npz")
        cache = {}
        if os.path.isfile(cache_path):
            z = np.load(cache_path, allow_pickle=False)
            cache = {k: e for k, e in zip(z["keys"], z["embs"])}
        keys, embs, etiq, self.fuentes = [], [], [], []
        self.nombres = sorted(d for d in os.listdir(self.carpeta)
                              if os.path.isdir(os.path.join(self.carpeta, d))
                              and not d.startswith((".", "_")))
        for i, nombre in enumerate(self.nombres):
            aviso = False
            for path in sorted(glob.glob(os.path.join(self.carpeta, nombre, "*"))):
                if not path.lower().endswith(IMG_EXT):
                    continue
                st = os.stat(path)
                for flip in (0, 1):
                    key = f"{os.path.relpath(path, self.carpeta)}|{st.st_mtime_ns}|{st.st_size}|{flip}"
                    e = cache.get(key)
                    if e is None:
                        if not aviso:
                            print(f"[galeria] procesando {nombre} ({i + 1}/{len(self.nombres)})...", flush=True)
                            aviso = True
                        e = self._emb_archivo(path, flip)
                    if e is None:
                        if not flip:
                            self.sin_cara.append(path)
                        break
                    keys.append(key)
                    embs.append(e)
                    etiq.append(i)
                    self.fuentes.append(path)
        self.embs = np.array(embs).reshape(-1, 128)
        self.etiquetas = np.array(etiq, int)
        if keys:
            try:
                np.savez(cache_path, keys=np.array(keys), embs=self.embs)
            except OSError:
                pass
        for p in self.sin_cara:
            print(f"[galeria] sin cara detectable: {p}")

    def _emb_archivo(self, path, flip):
        img = cv2.imread(path)
        if img is None:
            return None
        if flip:
            img = cv2.flip(img, 1)
        img, faces = self.rostros.detectar(img)
        return self.rostros.embedding(img, faces[0]) if len(faces) else None

    def sin(self, path):
        """Copia de la galeria sin las entradas de esa foto (para deja-uno-fuera)."""
        keep = np.array([f != path for f in self.fuentes], bool)
        sub = Galeria.__new__(Galeria)
        sub.nombres, sub.embs, sub.etiquetas = self.nombres, self.embs[keep], self.etiquetas[keep]
        return sub

    def similitudes(self, emb):
        """Similitud maxima por persona (array alineado con self.nombres)."""
        sims = self.embs @ emb
        out = np.full(len(self.nombres), -1.0)
        np.maximum.at(out, self.etiquetas, sims)
        return out

    def resumen(self):
        return {n: int((self.etiquetas == i).sum()) // 2 for i, n in enumerate(self.nombres)}


def identificar(frames, rostros, galeria, umbral=UMBRAL, margen=MARGEN, top=TOP_FRAMES):
    """Toma los mejores frames con cara y decide por similitud. Si nadie supera el umbral
    devuelve igual el mas parecido (seguro=False): responder algo es mejor que nada."""
    cands = []
    for f in frames:
        if f is None:
            continue
        img, faces = rostros.detectar(f)
        if len(faces):
            cands.append((calidad(img, faces[0]), img, faces[0]))
    res = Resultado(n_caras=len(cands))
    if not cands or not galeria.nombres:
        return res
    cands.sort(key=lambda c: -c[0])
    sims = np.mean([galeria.similitudes(rostros.embedding(img, face))
                    for _, img, face in cands[:top]], axis=0)
    order = np.argsort(-sims)
    res.ranking = [(galeria.nombres[i], float(sims[i])) for i in order]
    res.nombre, res.score = res.ranking[0]
    segundo = res.ranking[1][1] if len(order) > 1 else -1.0
    res.seguro = res.score >= umbral and res.score - segundo >= margen
    return res


def _norm_nombre(n):
    """Compara nombres sin importar mayusculas ni espacio/guion bajo (Brigitte Rojas == brigitte_rojas)."""
    return "_".join(str(n or "").lower().split()).replace("-", "_")


def buscar_persona_en_frames(frames, rostros, galeria, objetivo, umbral=UMBRAL, margen=MARGEN, top=TOP_FRAMES):
    """Revisa TODAS las caras detectadas una por una en cada frame (no solo la mayor).
    Devuelve (Resultado, mejor_img, mejor_face) priorizando coincidencias con `objetivo`."""
    obj_norm = _norm_nombre(objetivo)
    hits_obj, hits_all = [], []
    for f in frames:
        if f is None:
            continue
        img, faces = rostros.detectar(f)
        for face in faces:
            q = calidad(img, face)
            emb = rostros.embedding(img, face)
            sims = galeria.similitudes(emb)
            hits_all.append((q, img, face, sims))
            if galeria.nombres:
                order = np.argsort(-sims)
                top_nombre = _norm_nombre(galeria.nombres[order[0]])
                if obj_norm and top_nombre == obj_norm:
                    hits_obj.append((q, img, face, sims))

    res = Resultado(n_caras=len(hits_all))
    if not hits_all or not galeria.nombres:
        return res, None, None

    grupo = hits_obj if hits_obj else hits_all
    grupo.sort(key=lambda c: -c[0])
    sims_mean = np.mean([c[3] for c in grupo[:top]], axis=0)
    order = np.argsort(-sims_mean)
    res.ranking = [(galeria.nombres[i], float(sims_mean[i])) for i in order]
    res.nombre, res.score = res.ranking[0]
    segundo = res.ranking[1][1] if len(order) > 1 else -1.0
    res.seguro = res.score >= umbral and res.score - segundo >= margen
    return res, grupo[0][1], grupo[0][2]


# --- voz y anuncio ---------------------------------------------------------------

_PS_VOZ = r"""
Add-Type -AssemblyName System.Speech
$s = New-Object System.Speech.Synthesis.SpeechSynthesizer
$v = $s.GetInstalledVoices() | Where-Object { $_.VoiceInfo.Culture.Name -like 'es*' } | Select-Object -First 1
if ($v) { $s.SelectVoice($v.VoiceInfo.Name) }
$s.Rate = 0
[Console]::InputEncoding = [Text.Encoding]::UTF8
[Console]::Out.WriteLine('listo')
while (($l = [Console]::In.ReadLine()) -ne $null) { $s.Speak($l); [Console]::Out.WriteLine('ok') }
"""


class Voz:
    """TTS offline. En WSL usa la voz de Windows (SAPI, 'Microsoft Helena' en espanol) por
    powershell.exe; en Linux puro, espeak-ng / espeak / spd-say si estan. Si no hay
    nada, solo imprime. El proceso queda abierto para que hablar sea inmediato."""

    def __init__(self, modo="auto"):
        self.modo, self.proc, self.cmd = "ninguna", None, None
        ps = shutil.which("powershell.exe") or "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"
        if modo in ("auto", "windows") and os.path.exists(ps):
            import base64
            enc = base64.b64encode(_PS_VOZ.encode("utf-16-le")).decode()
            try:
                self.proc = subprocess.Popen([ps, "-NoProfile", "-EncodedCommand", enc],
                                             stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                             stderr=subprocess.DEVNULL, text=True,
                                             encoding="utf-8")
                if self.proc.stdout.readline().strip() == "listo":
                    self.modo = "windows"
                    return
            except OSError:
                pass
            self.proc = None
        if modo in ("auto", "linux"):
            for exe, extra in (("espeak-ng", ["-v", "es"]), ("espeak", ["-v", "es"]),
                               ("spd-say", ["-l", "es", "-w"])):
                if shutil.which(exe):
                    self.modo, self.cmd = exe, [exe, *extra]
                    return

    def decir(self, texto, esperar=True):
        if self.modo == "windows" and self.proc and self.proc.poll() is None:
            self.proc.stdin.write(texto.replace("\n", " ") + "\n")
            self.proc.stdin.flush()
            if esperar:
                self.proc.stdout.readline()
            return True
        if self.cmd:
            p = subprocess.Popen(self.cmd + [texto], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            if esperar:
                p.wait()
            return True
        return False

    def cerrar(self):
        if self.proc:
            self.proc.stdin.close()
            self.proc.terminate()


class Anunciador:
    """Imprime, publica en /nombre_reconocido (std_msgs/String) y lo dice en voz alta.
    Crearlo al arrancar: asi el publisher ya esta descubierto al anunciar."""

    def __init__(self, node=None, topic="/nombre_reconocido", voz="auto",
                 topic_foto="/persona_encontrada/image/compressed"):
        self.pub = None
        self.img_pub = None
        self.audio_pub = None
        if node is not None:
            from std_msgs.msg import String
            self.String = String
            self.pub = node.create_publisher(String, topic, 10)
            try:
                from sensor_msgs.msg import CompressedImage
                self.CompressedImage = CompressedImage
                self.img_pub = node.create_publisher(CompressedImage, topic_foto, 10)
            except Exception:
                pass
            try:
                from irobot_create_msgs.msg import AudioNoteVector, AudioNote
                from builtin_interfaces.msg import Duration
                self.AudioNoteVector = AudioNoteVector
                self.AudioNote = AudioNote
                self.Duration = Duration
                self.audio_pub = node.create_publisher(AudioNoteVector, "/cmd_audio", 10)
            except Exception:
                pass
        self.voz = Voz(voz) if voz != "ninguna" else None
        print(f"[anuncio] voz: {self.voz.modo if self.voz else 'ninguna'}"
              f" | buzzer: {'si' if self.audio_pub else 'no'}"
              f" | topic: {topic if self.pub else '-'}")

    def anunciar(self, nombre, frase="La persona es {}"):
        legible = nombre.replace("_", " ")
        print("\n" + "=" * 40 + f"\n  NOMBRE: {legible}\n" + "=" * 40 + "\n", flush=True)
        if self.pub is not None:
            for _ in range(3):
                self.pub.publish(self.String(data=legible))
                time.sleep(0.05)
        if self.audio_pub is not None:
            try:
                # Melodia victoriosa (Do5-Mi5-Sol5) en el buzzer de la base Create 3
                notes = [
                    self.AudioNote(frequency=523, max_runtime=self.Duration(sec=0, nanosec=150000000)),
                    self.AudioNote(frequency=659, max_runtime=self.Duration(sec=0, nanosec=150000000)),
                    self.AudioNote(frequency=784, max_runtime=self.Duration(sec=0, nanosec=300000000))
                ]
                self.audio_pub.publish(self.AudioNoteVector(append=False, notes=notes))
            except Exception:
                pass
        if self.voz is not None:
            self.voz.decir(frase.format(legible))

    def enviar_foto(self, nombre, img, face=None, score=0.0, carpeta=None):
        """Guarda la foto de la persona encontrada en `evidencias/` y la publica por ROS 2."""
        if img is None:
            return None
        destino = carpeta or os.path.join(HERE, "evidencias")
        os.makedirs(destino, exist_ok=True)
        seguro_nombre = "".join(c if (c.isalnum() or c in "-_") else "_" for c in (nombre or "persona"))
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(destino, f"{seguro_nombre}_{stamp}.jpg")
        out = img.copy()
        if face is not None:
            out = dibujar(out, np.asarray([face]), Resultado(nombre=nombre, score=score, seguro=True))
        cv2.imwrite(path, out)
        if self.img_pub is not None:
            ok, buf = cv2.imencode(".jpg", out)
            if ok:
                msg = self.CompressedImage(format="jpeg", data=buf.tobytes())
                for _ in range(3):
                    self.img_pub.publish(msg)
                    time.sleep(0.05)
        print(f"[anuncio] foto guardada y enviada: {path}", flush=True)
        return path

    def cerrar(self):
        if self.voz:
            self.voz.cerrar()


def anunciar(nombre, node=None, voz="auto"):
    """Atajo de un solo uso (mision.py usa Anunciador para no esperar al arranque)."""
    a = Anunciador(node, voz=voz)
    a.anunciar(nombre)
    a.cerrar()


# --- CLI ---------------------------------------------------------------------------

def dibujar(img, faces, res=None):
    for f in faces:
        x, y, w, h = (int(v) for v in f[:4])
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
    if res is not None and res.nombre:
        txt = f"{res.nombre} {res.score:.2f} {'OK' if res.seguro else '?'}"
        cv2.putText(img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0) if res.seguro else (0, 165, 255), 2)
    return img


def probar_galeria(rostros, galeria, umbral):
    """Deja-uno-fuera: cada foto contra el resto de la galeria."""
    aciertos = total = 0
    for nombre in galeria.nombres:
        fotos = sorted(set(f for f, i in zip(galeria.fuentes, galeria.etiquetas)
                           if galeria.nombres[i] == nombre))
        if len(fotos) < 2:
            print(f"--  {nombre}: solo 1 foto, no se puede probar (agrega mas)")
            continue
        for path in fotos:
            res = identificar([cv2.imread(path)], rostros, galeria.sin(path), umbral)
            ok = res.nombre == nombre
            aciertos += ok
            total += 1
            print(f"{'OK ' if ok else 'MAL'} {os.path.relpath(path, galeria.carpeta):40s} -> "
                  f"{res.nombre} ({res.score:.2f}{'' if res.seguro else ', dudoso'})")
    print(f"\nAciertos: {aciertos}/{total}")


def main():
    ap = argparse.ArgumentParser(description="Reconocimiento facial YuNet + SFace")
    ap.add_argument("--galeria", default=GALERIA)
    ap.add_argument("--umbral", type=float, default=UMBRAL)
    ap.add_argument("--probar", action="store_true", help="deja-uno-fuera sobre la galeria")
    ap.add_argument("--img", nargs="+", help="identificar estas imagenes")
    ap.add_argument("--robot", action="store_true", help="en vivo con la camara del robot")
    ap.add_argument("--capturar", metavar="NOMBRE", help="guardar fotos del robot en la galeria")
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--topic", default="/oakd/rgb/preview/image_raw/compressed")
    ap.add_argument("--anunciar", action="store_true", help="con --img/--robot: decir el nombre")
    ap.add_argument("--decir", metavar="TEXTO", help="probar la voz")
    args = ap.parse_args()

    if args.decir:
        v = Voz()
        print("voz:", v.modo)
        v.decir(args.decir)
        v.cerrar()
        return

    rostros = Rostros()
    os.makedirs(args.galeria, exist_ok=True)
    t0 = time.monotonic()
    galeria = Galeria(rostros, args.galeria)
    print(f"Galeria {galeria.resumen()} cargada en {time.monotonic() - t0:.1f} s")

    if args.probar:
        probar_galeria(rostros, galeria, args.umbral)
    if args.img:
        res = identificar([cv2.imread(p) for p in args.img], rostros, galeria, args.umbral)
        print(f"-> {res.nombre} score {res.score:.3f} seguro={res.seguro} caras={res.n_caras}")
        print("   ranking:", ", ".join(f"{n} {s:.2f}" for n, s in res.ranking[:5]))
        if args.anunciar and res.nombre:
            anunciar(res.nombre)
    if args.robot or args.capturar:
        vivo(args, rostros, galeria)


def vivo(args, rostros, galeria):
    import rclpy

    from mano_teleop import RobotCamera

    rclpy.init()
    node = rclpy.create_node("facial")
    cam = RobotCamera(node, args.topic)
    destino = None
    if args.capturar:
        destino = os.path.join(args.galeria, args.capturar)
        os.makedirs(destino, exist_ok=True)
    frames, guardadas, res, next_t = [], 0, None, 0.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            img = cam.latest()
            if img is None or time.monotonic() - cam.last_t > 1.0:
                continue
            big, faces = rostros.detectar(img)
            if destino and len(faces) and cam.last_t > next_t:
                next_t = cam.last_t + 0.3  # fotos algo distintas entre si
                cv2.imwrite(os.path.join(destino, f"{int(time.time() * 1000)}.jpg"), img)
                guardadas += 1
                print(f"guardada {guardadas}/{args.n}")
                if guardadas >= args.n:
                    break
            frames = (frames + [img])[-10:]
            if args.robot and len(frames) == 10:
                res = identificar(frames, rostros, galeria, args.umbral)
                frames = []
                print(f"-> {res.nombre} {res.score:.2f} {'seguro' if res.seguro else 'dudoso'}")
            cv2.imshow("facial", dibujar(big.copy(), faces, res))
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    finally:
        if args.anunciar and res is not None and res.nombre:
            anunciar(res.nombre, node)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
