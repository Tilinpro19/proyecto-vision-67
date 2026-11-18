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


def _scan_cruce_izq_y_recto(angles, rumbo_izq=True, rumbo_der=False, rumbo_recto=True):
    """Genera un scan sintetico con pasillo abierto segun las direcciones indicadas."""
    ranges = []
    for ang in angles:
        if rumbo_recto and abs(ang) < 0.60:
            ranges.append(2.5)
        elif rumbo_izq and abs(ang - math.pi / 2) < 0.60:
            ranges.append(2.5)
        elif rumbo_der and abs(ang + math.pi / 2) < 0.60:
            ranges.append(2.5)
        else:
            ranges.append(0.55)
    return ranges


def test_mision_guiada_al_volver_prefiere_rama_sin_explorar_sobre_origen():
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    angles = [math.radians(i * 5 - 180) for i in range(72)]
    guiado.iniciar((0.0, 0.0, 0.0), [3.0] * 72, angles)
    m = ms.Mision("A", quiet=True)
    m.ir(ms.SEGUIR)

    # 1. En el cruce (1.0, 0.0) mirando a +x: opciones 'recto' (+x) e 'izq' (+y). Elige 'recto'.
    guiado.piloto.crumbs.append((1.0, 0.0))
    r_ida = _scan_cruce_izq_y_recto(angles, rumbo_izq=True, rumbo_der=False, rumbo_recto=True)
    guiado.paso(5.0, (1.0, 0.0, 0.0), r_ida, angles, mision=m)
    assert m.decisiones[-1]["accion"] == "recto"

    # 2. En (2.2, 0.0) encuentra callejon y retrocede hacia -x (pi).
    guiado.piloto.crumbs.extend([(1.6, 0.0), (2.2, 0.0)])
    guiado.paso(10.0, (2.2, 0.0, 0.0), [0.55] * 72, angles, mision=m)
    assert m.decisiones[-1]["accion"] == "retroceder"

    # 3. De vuelta en (1.0, 0.0) mirando a -x (pi): 'recto' vuelve a A, 'der' es la rama +y sin explorar.
    r_vuelta = _scan_cruce_izq_y_recto(angles, rumbo_izq=False, rumbo_der=True, rumbo_recto=True)
    guiado.paso(15.0, (1.0, 0.0, math.pi), r_vuelta, angles, mision=m)
    assert m.decisiones[-1]["accion"] == "der"


def test_mision_guiada_retroceso_no_se_cancela_antes_de_llegar_al_cruce():
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    angles = [math.radians(i * 5 - 180) for i in range(72)]
    guiado.iniciar((0.0, 0.0, 0.0), [3.0] * 72, angles)
    guiado.piloto.crumbs.extend([(0.5, 0.0), (0.9, 0.0), (1.5, 0.0), (2.2, 0.0), (2.8, 0.0)])
    guiado.cruce_anterior_miga_idx = 2  # cruce previo en (0.9, 0.0)

    m = ms.Mision("A", quiet=True)
    m.ir(ms.SEGUIR)
    guiado.paso(10.0, (2.8, 0.0, 0.0), [0.55] * 72, angles, mision=m)
    assert guiado.retrocediendo_hasta_idx == 2
    assert len(m.decisiones) == 1

    # En (1.6, 0.0) mirando a pi aparece un cruce intermedio (> 0.45 m de (0.9, 0.0)): NO debe decidir ni cancelar
    r_cruce = _scan_cruce_izq_y_recto(angles, rumbo_izq=False, rumbo_der=True, rumbo_recto=True)
    guiado.paso(13.0, (1.6, 0.0, math.pi), r_cruce, angles, mision=m)
    assert guiado.retrocediendo_hasta_idx == 2
    assert len(m.decisiones) == 1

    # Al llegar a < 0.45 m de (0.9, 0.0), completa el retroceso, poda migas y decide en el cruce
    guiado.paso(16.0, (0.95, 0.0, math.pi), r_cruce, angles, mision=m)
    assert guiado.retrocediendo_hasta_idx is None
    assert len(m.decisiones) == 2


def test_mision_guarda_diagnostico_npz_en_choque_y_atasco(tmp_path):
    import numpy as np

    m = ms.Mision("A", log_dir=str(tmp_path), quiet=True)
    m.ir(ms.SEGUIR)
    ruta = m.guardar_diagnostico("atasco", (1.2, -0.3, 0.5), [0.4, 1.5], [-0.5, 0.5], [(0.0, 0.0), (1.2, -0.3)])
    assert ruta is not None and os.path.isfile(ruta)
    data = np.load(ruta)
    assert list(data["pose"]) == [1.2, -0.3, 0.5]
    assert len(data["ranges"]) == 2
    assert data["crumbs"].shape == (2, 2)

    # Verificar que _volcar_eventos_piloto dispara diagnostico cuando hay evento de atasco
    class _StubLidar:
        data = ([0.5, 1.0], [-0.2, 0.2])

    class _StubOdom:
        pose = (0.5, 0.1, 0.0)

    class _StubSistema:
        lidar = _StubLidar()
        odom = _StubOdom()

    piloto = ms.Piloto()
    piloto.eventos.append((1.0, "atasco con frente libre: obstaculo bajo en (0.50, 0.10)"))
    ms._volcar_eventos_piloto(_StubSistema(), m, piloto)
    npz_files = [f for f in os.listdir(tmp_path) if f.endswith(".npz")]
    assert len(npz_files) == 2
    m.cerrar()


def test_mision_guiada_clave_cruce_reutiliza_cruce_cercano_en_borde():
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    angles = [math.radians(i * 5 - 180) for i in range(72)]
    guiado.iniciar((0.0, 0.0, 0.0), [3.0] * 72, angles)
    m = ms.Mision("A", quiet=True)
    m.ir(ms.SEGUIR)

    # Cruce en x=1.48 (antes caia en celda 1)
    r_cruce = _scan_cruce_izq_y_recto(angles, rumbo_izq=True, rumbo_der=False, rumbo_recto=True)
    guiado.piloto.crumbs.append((1.48, 0.02))
    guiado.paso(5.0, (1.48, 0.02, 0.0), r_cruce, angles, mision=m)
    assert len(guiado.visitas_cruce) == 1

    # Avanza al callejon en (2.6, 0.0) y retrocede
    guiado.piloto.crumbs.extend([(2.1, 0.0), (2.6, 0.0)])
    guiado.paso(10.0, (2.6, 0.0, 0.0), [0.55] * 72, angles, mision=m)
    assert m.decisiones[-1]["accion"] == "retroceder"

    # Al volver con drift a x=1.56 (antes caia en celda 2), debe reutilizar el mismo cruce (1.48, 0.02)
    guiado.paso(15.0, (1.56, -0.04, math.pi), _scan_cruce_izq_y_recto(angles, rumbo_izq=False, rumbo_der=True, rumbo_recto=True), angles, mision=m)
    assert (1.48, 0.02) in guiado.visitas_cruce
    assert guiado.visitas_cruce[(1.48, 0.02)].get(1) == 1
    assert len(guiado.visitas_cruce) == 2
    assert m.decisiones[-1]["accion"] == "der"


def test_escape_atras_respeta_margen_back_free_020():
    import navegacion as nv

    piloto = nv.Piloto()
    angles = [math.radians(i * 10 - 180) for i in range(36)]
    piloto.fijar_origen((0.0, 0.0, 0.0), [3.0] * 36, angles)
    piloto.escape = ("atras", 1.0, (0.0, 0.0, 0.0), None)

    # Obstaculo detras a 0.35 m -> tras restar robot_radius(0.17)+safety_margin(0.10)=0.27, back_free~0.08 <= 0.20
    # Debe cortar la marcha atras y pasar a fase "giro" (lin == 0.0)
    ranges = [0.35 if abs(abs(a) - math.pi) < 0.4 else 2.5 for a in angles]
    lin, _ang, _info = piloto.paso(1.2, (0.0, 0.0, 0.0), ranges, angles)
    assert lin == 0.0
    assert piloto.escape[0] == "giro"


def test_mision_guiada_sin_historial_no_retrocede_hacia_la_pared_trasera():
    # Caso real (mision_A_..._214219): el robot arranca en una caja de ~0.5 m (izq, der, atras)
    # con espacio al frente pero un retorno cercano al centro -> opciones=[]. Al inicio no hay
    # migas ni cruce previo: "retroceder" no puede invertir el rumbo contra la pared trasera.
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    angles = [math.radians(i * 5 - 180) for i in range(72)]
    guiado.iniciar((0.0, 0.0, 0.0), [2.0] * 72, angles)
    ranges = []
    for ang in angles:
        d = abs(math.degrees(ang))
        if d <= 5:
            ranges.append(0.70)
        elif d <= 40:
            ranges.append(1.60)
        else:
            ranges.append(0.50)
    m = ms.Mision("A", quiet=True)
    m.ir(ms.SEGUIR)
    guiado.paso(1.1, (0.0, 0.0, 0.0), ranges, angles, mision=m)
    assert not m.decisiones          # al arrancar no se decide (aun no se movio)
    guiado.paso(7.5, (0.0, 0.0, 0.0), ranges, angles, mision=m)   # atascado >6 s: ya decide
    assert m.decisiones and m.decisiones[0]["opciones"] == []
    # El rumbo elegido debe mirar hacia el lado abierto (frente), no hacia la pared trasera
    assert abs(math.degrees(ms.nc.wrap(guiado.rumbo_pasillo))) < 60.0


def test_mision_guiada_sin_historial_conserva_rumbo_si_hay_espacio_al_frente():
    # Caso real (run 214938): caja angosta, algo a 1.0 m al frente -> opciones=[] pero con 0.74 m
    # libres adelante. Una puerta lateral hace que most_open apunte a ~25 deg; el rumbo NO debe
    # desviarse (el robot terminaba de costado contra la pared y no avanzaba).
    from sim import scan_from
    paredes = [((-0.5, -0.5), (-0.5, 0.5)), ((-0.5, -0.5), (4, -0.5)),
               ((-0.5, 0.5), (0.4, 0.5)), ((1.3, 0.5), (4, 0.5)), ((1.0, -0.12), (1.0, 0.12))]
    r, a = scan_from(paredes, (0.0, 0.0, 0.0))
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    guiado.iniciar((0.0, 0.0, 0.0), r, a)
    m = ms.Mision("A", quiet=True)
    m.ir(ms.SEGUIR)
    guiado.paso(1.1, (0.0, 0.0, 0.0), r, a, mision=m)
    guiado.paso(7.5, (0.0, 0.0, 0.0), r, a, mision=m)
    assert m.decisiones and m.decisiones[0]["opciones"] == []
    assert abs(math.degrees(ms.nc.wrap(guiado.rumbo_pasillo))) < 5.0


def test_mision_guiada_flecha_izq_se_obedece_con_lateral_de_0_6_m():
    # Frente cerrado y un lateral izquierdo angosto (0.7 m, < open_side 1.15) con flecha 'izq' cercana:
    # antes caia en "callejon" y retrocedia; ahora gira a la izquierda como indica la flecha.
    from sim import scan_from
    from senales_forma import Senal
    paredes = [((-1.0, -0.5), (3.0, -0.5)), ((-1.0, 0.5), (0.0, 0.5)), ((0.7, 0.5), (0.7, 2.5)),
               ((0.0, 0.5), (0.0, 2.5)), ((1.4, -0.5), (1.4, 0.5)), ((0.0, 2.5), (0.7, 2.5))]
    pose = (0.35, 0.0, 0.0)
    r, a = scan_from(paredes, pose)
    guiado = ms.PilotoGuiado(mp=ms.MisionParams(bandera_frames=2))
    guiado.iniciar(pose, r, a)
    m = ms.Mision("A", quiet=True)
    m.ir(ms.SEGUIR)
    sen = Senal(tipo="izq", confianza=0.8, bbox=(100, 100, 60, 80))
    guiado.registrar_senales(5.0, [sen], m, alto_img=360)    # 80/360 = 0.22 >= 0.14: flecha cercana
    assert guiado.senal_cerca
    guiado.paso(10.0, (1.5, 0.0, 0.0), r, a, mision=m)
    assert m.decisiones and m.decisiones[-1]["accion"] == "izq"
