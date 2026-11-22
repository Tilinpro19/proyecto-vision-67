"""Reconocimiento facial offline con fotos de muestra de OpenCV (tests/data/)."""
import os
import shutil
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import facial  # noqa: E402

DATA = os.path.join(HERE, "data")
pytestmark = pytest.mark.skipif(
    not (os.path.isfile(facial.YUNET) and os.path.isfile(facial.SFACE)
         and hasattr(cv2, "FaceDetectorYN")), reason="faltan modelos o OpenCV >= 4.7")


@pytest.fixture(scope="module")
def rostros():
    return facial.Rostros()


def img(name):
    return cv2.imread(os.path.join(DATA, name))


def make_gallery(tmp, people, rostros):
    for person, files in people.items():
        os.makedirs(tmp / person)
        for f in files:
            shutil.copy(os.path.join(DATA, f), tmp / person / f)
    return facial.Galeria(rostros, str(tmp))


def degrade(im, scale=0.35, light=-50, blur=0):
    """Simula la foto vista por el robot: chica, con otra luz y algo borrosa."""
    h, w = im.shape[:2]
    out = cv2.resize(im, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    out = cv2.convertScaleAbs(out, alpha=1.0, beta=light)
    return cv2.GaussianBlur(out, (0, 0), blur) if blur else out


def test_detecta_cara_en_imagen_chica(rostros):
    _, faces = rostros.detectar(degrade(img("lena.jpg"), scale=0.3))
    assert len(faces) >= 1


def test_identifica_con_luz_y_tamano_distintos(tmp_path, rostros):
    gal = make_gallery(tmp_path, {"Lena": ["lena.jpg"], "Messi": ["messi5.jpg"]}, rostros)
    assert gal.resumen() == {"Lena": 1, "Messi": 1}
    res = facial.identificar([degrade(img("lena.jpg"))] * 3, rostros, gal)
    assert res.nombre == "Lena" and res.seguro and res.n_caras == 3
    res = facial.identificar([degrade(img("messi5.jpg"), scale=0.8, light=40)], rostros, gal)
    assert res.nombre == "Messi"


def test_sin_coincidencia_devuelve_el_mas_cercano(tmp_path, rostros):
    gal = make_gallery(tmp_path, {"Lena": ["lena.jpg"]}, rostros)
    res = facial.identificar([img("messi5.jpg")], rostros, gal)
    assert res.nombre == "Lena" and not res.seguro


def test_sin_caras_no_inventa(tmp_path, rostros):
    gal = make_gallery(tmp_path, {"Lena": ["lena.jpg"]}, rostros)
    ruido = np.random.default_rng(0).integers(0, 255, (250, 250, 3), np.uint8)
    res = facial.identificar([ruido, None], rostros, gal)
    assert res.nombre is None and res.n_caras == 0


def test_calidad_prefiere_el_frame_nitido(rostros):
    nitida = img("lena.jpg")
    borrosa = cv2.GaussianBlur(nitida, (0, 0), 4)
    q = []
    for im in (nitida, borrosa):
        big, faces = rostros.detectar(im)
        q.append(facial.calidad(big, faces[0]))
    assert q[0] > q[1]


def test_cache_de_galeria(tmp_path, rostros):
    make_gallery(tmp_path, {"Lena": ["lena.jpg"]}, rostros)
    assert (tmp_path / ".cache.npz").exists()
    gal = facial.Galeria(rostros, str(tmp_path))       # segunda carga: desde cache
    assert len(gal.embs) == 2 and gal.nombres == ["Lena"]


def test_buscar_persona_revisa_todas_las_caras_una_por_una(tmp_path, rostros):
    gal = make_gallery(tmp_path, {"Lena": ["lena.jpg"], "Messi": ["messi5.jpg"]}, rostros)
    lena_chica = cv2.resize(img("lena.jpg"), (240, 240))
    messi_grande = cv2.resize(img("messi5.jpg"), (480, 480))
    fondo = np.zeros((480, 740, 3), np.uint8)
    fondo[120:360, 0:240] = lena_chica
    fondo[:, 260:740] = messi_grande

    res, foto_img, foto_face = facial.buscar_persona_en_frames([fondo], rostros, gal, "Lena")
    assert res.nombre == "Lena" and res.n_caras >= 2
    assert foto_img is not None and foto_face is not None
    # La caja devuelta debe corresponder a la mitad izquierda (Lena), no a Messi
    assert float(foto_face[0]) < 260

