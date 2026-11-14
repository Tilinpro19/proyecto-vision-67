"""Pruebas de tabla para la politica de decision senal > exploracion (Fase 3)."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import decision as dc  # noqa: E402
from senales_forma import Senal  # noqa: E402


@pytest.mark.parametrize("opciones,senal,visitado,esperado", [
    # 1. Senal valida y la opcion existe en el LiDAR -> obedecerla
    ({"izq", "recto", "der"}, "izq", {"izq": 3, "recto": 0, "der": 0}, "izq"),
    ({"izq", "recto"}, "der", {}, "recto"),  # 2. Senal contradice al LiDAR -> gana el LiDAR (desempate recto)
    ({"der"}, "izq", {}, "der"),             # 2b. Flecha izq pero solo hay salida der -> gana LiDAR (der)
    # 3. Sin senal -> la opcion menos explorada
    ({"izq", "recto", "der"}, None, {"recto": 2, "izq": 1, "der": 0}, "der"),
    ({"izq", "recto"}, None, {"recto": 1, "izq": 0}, "izq"),
    # 4. Empate -> orden fijo: recto, izquierda, derecha
    ({"izq", "recto", "der"}, None, {}, "recto"),
    ({"izq", "der"}, None, {"izq": 1, "der": 1}, "izq"),
    # 5. Callejon -> retroceder
    (set(), None, {}, "retroceder"),
    (set(), "izq", {}, "retroceder"),
    # 6. Prohibido -> marca recto como agotada y elige otra (o retrocede si no hay otra)
    ({"recto", "der"}, "prohibido", {}, "der"),
    ({"recto"}, "prohibido", {}, "retroceder"),
    # 7. Bandera -> llegada_b
    ({"recto"}, "bandera", {}, "llegada_b"),
])
def test_tabla_decision(opciones, senal, visitado, esperado):
    avisos, agotadas = [], set()
    accion = dc.decidir(opciones, senal, visitado, avisos=avisos, agotadas=agotadas)
    assert accion == esperado


def test_contradiccion_senal_lidar_emite_aviso():
    avisos = []
    accion = dc.decidir({"recto", "der"}, Senal("izq", 0.9, (0, 0, 20, 20)), visitado={}, avisos=avisos)
    assert accion == "recto"
    assert any("contradice al LiDAR" in a for a in avisos)


def test_prohibido_marca_rama_agotada_y_qr_registra_hito():
    avisos, agotadas = [], set()
    accion = dc.decidir({"recto", "izq"}, Senal("prohibido", 0.9, (0, 0, 20, 20)),
                        visitado={}, avisos=avisos, agotadas=agotadas)
    assert accion == "izq"
    assert "recto" in agotadas

    avisos_qr = []
    accion_qr = dc.decidir({"izq", "der"}, Senal("qr", 0.98, (0, 0, 30, 30), texto="milestone_alfa"),
                           visitado={"izq": 1, "der": 0}, avisos=avisos_qr)
    assert accion_qr == "der"
    assert any("milestone_alfa" in a for a in avisos_qr)
