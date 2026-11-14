"""Pruebas de integracion de Tarea A guiada en mision.py con el simulador (Fase 4)."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import mision as ms  # noqa: E402
from senales_forma import Senal  # noqa: E402
from sim import Sim  # noqa: E402


def laberinto_t_y_esquina():
    """Laberinto hermético con pasillos de 1.2 m de ancho:
    - A=(0,0) avanza hacia +x hasta el cruce en T centrado en (2.0, 0.0).
    - A la derecha (-y) hay un callejón cerrado en y=-1.8.
    - A la izquierda (+y) sube hasta la esquina en (2.0, 2.2) y dobla a la derecha (+x)
      hasta la meta B en (3.7, 2.2).
    """
    return [
        ((-0.8, -0.6), (-0.8, 0.6)),    # pared trasera detrás de A
        ((-0.8, -0.6), (1.4, -0.6)),    # pared inferior pasillo de entrada
        ((-0.8, 0.6), (1.4, 0.6)),      # pared superior pasillo de entrada
        ((1.4, -0.6), (1.4, -1.8)),     # pared izq del callejón derecho
        ((1.4, -1.8), (2.6, -1.8)),     # fondo del callejón derecho
        ((2.6, -1.8), (2.6, 1.6)),      # pared frontal de la T y esquina interna superior
        ((1.4, 0.6), (1.4, 2.8)),       # pared exterior izq de la subida
        ((1.4, 2.8), (4.4, 2.8)),       # pared superior del tramo final hacia B
        ((2.6, 1.6), (4.4, 1.6)),       # pared inferior del tramo final hacia B
        ((4.4, 1.6), (4.4, 2.8)),       # pared de fondo detrás de B
    ]


def _simular_mision_guiada(proveer_senales, t_max=180.0, dt=0.1):
    sim = Sim(laberinto_t_y_esquina(), start=(0.0, 0.0, 0.0))
    m = ms.Mision("A", quiet=True)
    mp = ms.MisionParams(bandera_frames=2)
    guiado = ms.PilotoGuiado(mp=mp)
    guiado.iniciar(sim.odom, *sim.scan())
    m.ir(ms.SEGUIR, "inicio simulacion guiada")

    t = 0.0
    while t < t_max and m.activa():
        r, a = sim.scan()
        if m.estado in (ms.SEGUIR, ms.DECIDIR):
            senales = proveer_senales(sim.true, t)
            if guiado.registrar_senales(t, senales, m):
                m.ir(ms.LLEGADA_B, "bandera confirmada")
                m.ir(ms.ESCANEAR, "escaneo simulado en B")
                continue
            lin, ang, info = guiado.paso(t, sim.odom, r, a, mision=m)
            if info.get("llegada_b"):
                m.ir(ms.ESCANEAR, "escaneo simulado en B")
                continue
            sim.step(lin, ang, dt)
        elif m.estado == ms.ESCANEAR:
            m.nombre, m.score, m.seguro = "Juan_Perez", 0.85, True
            guiado.piloto.volver()
            m.ir(ms.VOLVER_A_A, "regreso por migas")
        elif m.estado == ms.VOLVER_A_A:
            lin, ang, info = guiado.piloto.paso(t, sim.odom, r, a)
            if info["llegada"]:
                m.ir(ms.ANUNCIAR, "en A")
                m.ir(ms.FIN)
                break
            sim.step(lin, ang, dt)
        t += dt

    return m, sim


def test_mision_guiada_con_senales_llega_a_bandera_y_vuelve_a_a():
    def senales_en_mapa(pose, _t):
        x, y, th = pose
        out = []
        # Antes de la T en (2.0, 0.0) mirando a +x: ve QR y flecha izquierda
        if 0.8 < x < 1.9 and abs(y) < 0.5 and abs(th) < 0.6:
            out.append(Senal("qr", 0.98, (10, 10, 40, 40), texto="milestone_alfa"))
            out.append(Senal("izq", 0.90, (100, 100, 50, 50)))
        # Subiendo hacia la esquina en (2.0, 2.2) mirando a +y: ve flecha derecha
        if 1.4 < x < 2.6 and 0.9 < y < 2.1 and abs(th - math.pi / 2) < 0.6:
            out.append(Senal("der", 0.90, (100, 100, 50, 50)))
        # En el tramo final cerca de B=(3.6, 2.2): ve la bandera de cuadros
        if x > 3.2 and 1.6 < y < 2.8:
            out.append(Senal("bandera", 0.92, (120, 80, 90, 120)))
        return out

    m, sim = _simular_mision_guiada(senales_en_mapa)
    res = m.resultado()
    assert res["ok"] is True
    assert res["estado"] == ms.FIN
    assert sim.collisions == 0
    assert "milestone_alfa" in res["hitos_qr"]
    assert math.hypot(sim.true[0], sim.true[1]) < 0.35


def test_mision_guiada_sin_senales_explora_y_llega_sin_bucles():
    def solo_bandera_en_b(pose, _t):
        x, y, _ = pose
        if x > 3.2 and 1.6 < y < 2.8:
            return [Senal("bandera", 0.92, (120, 80, 90, 120))]
        return []

    m, sim = _simular_mision_guiada(solo_bandera_en_b)
    res = m.resultado()
    assert res["ok"] is True
    assert sim.collisions == 0
    assert len(res["decisiones"]) >= 2
    assert math.hypot(sim.true[0], sim.true[1]) < 0.35


def test_mision_guiada_senal_contradice_pared_no_choca_y_queda_en_log():
    def senal_contradictoria_en_esquina(pose, _t):
        x, y, th = pose
        # En la esquina superior (2.0, 2.2), la unica salida es 'der' (+x), pero la senal pide 'izq'
        if 1.4 < x < 2.6 and 0.9 < y < 2.2 and abs(th - math.pi / 2) < 0.6:
            return [Senal("izq", 0.95, (100, 100, 50, 50))]
        if x > 3.2 and 1.6 < y < 2.8:
            return [Senal("bandera", 0.92, (120, 80, 90, 120))]
        return []

    m, sim = _simular_mision_guiada(senal_contradictoria_en_esquina)
    res = m.resultado()
    assert res["ok"] is True
    assert sim.collisions == 0
    assert any("contradice al LiDAR" in ev for ev in m.eventos)


def test_mision_guiada_callejon_sin_salida_retrocede_y_llega_a_b():
    def forzar_callejon_en_t(pose, _t):
        x, y, th = pose
        # En la primera pasada por la T manda al callejon derecho (-y), que esta cerrado en y=-1.8
        if 0.8 < x < 1.9 and abs(y) < 0.5 and abs(th) < 0.6:
            return [Senal("der", 0.90, (100, 100, 50, 50))]
        if x > 3.2 and 1.6 < y < 2.8:
            return [Senal("bandera", 0.92, (120, 80, 90, 120))]
        return []

    m, sim = _simular_mision_guiada(forzar_callejon_en_t, t_max=220.0)
    res = m.resultado()
    assert res["ok"] is True
    assert sim.collisions == 0
    assert any(d["accion"] == "retroceder" for d in res["decisiones"])
    assert math.hypot(sim.true[0], sim.true[1]) < 0.35


def test_mision_guiada_dos_senales_en_mismo_frame_prioriza_la_mas_cercana():
    m = ms.Mision("A", quiet=True)
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    senales = [
        Senal("izq", 0.88, (80, 80, 90, 90)),   # mas grande / cercana (primera en detectar())
        Senal("der", 0.92, (200, 120, 30, 30)), # mas pequena / lejana
    ]
    assert guiado.registrar_senales(1.0, senales, m) is False
    assert guiado.senal_pendiente is not None
    assert guiado.senal_pendiente.tipo == "izq"


def test_mision_guiada_retroceder_no_gira_en_bucle_en_el_mismo_sitio():
    # Reproduce el caso real: tras decidir "retroceder" y girar 180 deg, el LiDAR sigue viendo
    # obstaculo < block_front en el mismo sitio (< min_dist_cruce); no debe volver a retroceder.
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    guiado.iniciar((0.0, 0.0, 0.0), [2.0] * 36, [0.0] * 36)
    angles = [math.radians(i * 10 - 180) for i in range(36)]
    # Callejón cerrado por frente y lados (0.55 m): opciones=[] y frente_bloqueado=True
    ranges_cerrado = [0.55] * 36
    m = ms.Mision("A", quiet=True)
    m.ir(ms.SEGUIR)

    # 1. Primera vez en (2.40, -0.42) mirando a 0 rad: debe decidir "retroceder" e invertir rumbo a pi
    guiado.paso(10.0, (2.40, -0.42, 0.0), ranges_cerrado, angles, mision=m)
    assert guiado.ultima_accion == "retroceder"
    assert len(m.decisiones) == 1

    # 2. Ya giro 180 deg a pi rad, pero sigue en (2.44, -0.43) con < 0.80 m al frente: NO debe re-decidir
    guiado.paso(13.5, (2.44, -0.43, math.pi), ranges_cerrado, angles, mision=m)
    assert len(m.decisiones) == 1


def test_mision_guiada_retroceder_sigue_migas_de_vuelta_al_cruce():
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    angles = [math.radians(i * 10 - 180) for i in range(36)]
    guiado.iniciar((0.0, 0.0, 0.0), [3.0] * 36, angles)
    # Simular camino de entrada curvo rodeando un obstaculo (tacho) hasta (2.0, -0.7)
    guiado.piloto.crumbs.extend([(0.5, 0.0), (0.9, 0.0), (1.3, -0.35), (1.7, -0.65), (2.0, -0.7)])
    guiado.cruce_anterior_miga_idx = 2  # cruce previo en (0.9, 0.0)

    ranges_cerrado = [0.55] * 36
    m = ms.Mision("A", quiet=True)
    m.ir(ms.SEGUIR)
    guiado.paso(10.0, (2.0, -0.7, 0.0), ranges_cerrado, angles, mision=m)
    assert guiado.retrocediendo_hasta_idx == 2
    # La meta debe apuntar a una miga del camino de regreso (y >= -0.65, x <= 1.7), no a (3.4, -0.7)
    assert guiado.piloto.meta[0] <= 1.75
    assert guiado.piloto.meta[1] >= -0.65
