"""
Tarea A modo "directo": recto hasta la flecha -> girar en el sitio -> recto ... -> caras.
Sin ROS ni OpenCV: opera sobre Piloto (navegacion.py), pasillo.py y decision.py.

Diferencias con PilotoGuiado (explorador de laberintos, mision.py):
  - Ignora los huecos laterales sin flecha: solo decide con una flecha confirmada cerca
    o con el frente cerrado (esquina sin flecha -> decide con el LiDAR igual).
  - La flecha se ubica en el mundo (rumbo en la imagen + rango del LiDAR) y se obedece
    al llegar a ella (`d_giro`), no a los 6.5 s ni solo en cruces. Una flecha ya obedecida
    no se vuelve a obedecer (por posicion, no por tiempo).
  - Gira en el sitio a rumbos cardinales (90/180 grados) y sigue sin enfriamiento.
  - Llegada a B: bandera confirmada, o caras al frente al cerrarse el frente, o callejon
    con caras (MIRAR). Callejon sin caras = media vuelta (hasta `max_callejones`).
  - Velocidad por tramos: v_recto con camino libre, v_acercar con una flecha pendiente.

Uso (mision.py): iniciar(); por cada frame ver_senales() / ver_caras(); paso() cada ciclo.
paso() devuelve info["accion"]: None | "mirar" (frenado en un callejon: conviene pasarle
caras con ver_caras() mientras dure) | "llegada_b" (frenar y pasar a ESCANEAR).
"""
import math
from dataclasses import dataclass

import numpy as np

import decision as dc
import nav_core as nc
import pasillo as ps

RECTO, GIRAR, MIRAR, LLEGADA = "recto", "girar", "mirar", "llegada"


@dataclass
class DirectoParams:
    v_recto: float = 0.26        # m/s  tramo recto con el frente libre (Create 3: 0.31 max)
    v_acercar: float = 0.20      # m/s  con una flecha pendiente a menos de d_lento
    v_regreso: float = 0.26      # m/s  regreso a A por migas (camino conocido)
    d_lento: float = 1.2         # m    flecha a menos de esto -> v_acercar
    d_giro: float = 0.72         # m    obedecer la flecha a esta distancia (hacia adelante)
    d_frente: float = 0.70       # m    pared del fondo (LiDAR crudo) -> decidir (esquina / callejon).
                                 #      Con pasillos de 1.2 m el centro del cruce queda a 0.6; no
                                 #      bajar de inflate + stop_dist (~0.65) o Piloto frena antes.
    lado_abierto: float = 1.0    # m    libre a +-90 grados para girar hacia la flecha
    w_giro: float = 1.4          # rad/s giro en el sitio (maximo)
    w_giro_min: float = 0.35     # rad/s para no quedarse arrastrando al final del giro
    k_giro: float = 2.2          # rad/s por rad de error
    tol_giro_deg: float = 4.0
    alineado_deg: float = 25.0   # solo decide si va alineado con el rumbo del pasillo
    confirmar: int = 2           # lecturas seguidas iguales para aceptar una flecha
    radio_misma: float = 0.45    # m    dos lecturas a menos de esto = la misma flecha
    radio_obedecida: float = 0.60  # m  flecha ya obedecida: se ignora
    fov_deg: float = 69.0        # FOV horizontal de la camara (OAK-D Lite RGB)
    ventana_rango_deg: float = 3.0
    zanahoria: float = 1.4       # m    punto a seguir por el rumbo del pasillo
    w_goal_recto: float = 4.0    # peso del rumbo en choose_heading durante el tramo recto
    slow_dist_recto: float = 0.50  # m libres (inflado) desde donde frena en el tramo recto
    centrado_max: float = 0.25   # m    correccion lateral maxima de la zanahoria
    cara_min: float = 0.05       # fraccion del ancho de imagen para contar una cara
    cara_vigencia: float = 1.0   # s    una cara vista hace menos de esto cuenta
    t_mirar: float = 1.2         # s    frenado mirando al frente en un callejon
    barrido_deg: float = 60.0    # tras mirar al frente, gira +-esto buscando caras (0 = no)
    w_barrido: float = 0.8       # rad/s del barrido (lento: la camara no sale movida)
    max_callejones: int = 2      # callejones sin caras antes de escanear igual
    bandera_frames: int = 2
    d_bandera: float = 1.0       # m    la bandera confirma la llegada solo a menos de esto
    d_prohibido: float = 3.0     # m    prohibido pendiente a menos de esto: tomar la salida lateral


def cardinal(rad):
    return int(round(nc.wrap(rad) / (math.pi / 2.0))) % 4


def rango_en(ranges, angles, bearing, ventana):
    """Mediana del LiDAR en `bearing` +- ventana (rad); None si no hay rayos validos."""
    if len(ranges) == 0:
        return None
    m = np.abs(nc.wrap(angles - bearing)) < ventana
    m &= np.isfinite(ranges)
    return float(np.median(ranges[m])) if np.any(m) else None


def rumbo_en_imagen(cx, ancho, fov_deg):
    """Rumbo (rad, + = izquierda) de la columna cx en una imagen de `ancho` px (pinhole)."""
    f = (ancho / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return math.atan2((ancho / 2.0) - cx, f)


class PilotoDirecto:
    def __init__(self, piloto, mp=None, dp=None, pp_pasillo=None):
        self.piloto = piloto
        self.dp = dp or DirectoParams()
        self.pp_pasillo = pp_pasillo or ps.PasilloParams()
        self.pol = dc.DecisionParams(
            senal_prohibido=getattr(mp, "senal_prohibido", "buscar_otro"),
            senal_bandera=getattr(mp, "senal_bandera", "llegada_b"),
            senal_qr=getattr(mp, "senal_qr", "registrar"),
        )
        self.estado = RECTO
        self.rumbo = 0.0              # rumbo del pasillo en el marco de A
        self.rumbo_obj = 0.0
        self.pendiente = None         # dict(tipo, xy)
        self._cand = None             # (tipo, xy, n) lectura sin confirmar
        self.obedecidas = []          # xy de flechas ya usadas
        self.visitas = {}             # clave_xy -> {cardinal: n}
        self.agotadas = {}            # clave_xy -> set(cardinal)
        self.ultimo_cruce = None      # (clave, cardinal elegido)
        self.callejones = 0
        self.t_cara = -1e9
        self.cont_bandera = 0
        self.t_mirar0 = None
        self._barrido = []
        self.motivo_llegada = None
        self.eventos = []             # textos para el registro
        self.decisiones = []          # dict(t, pose, opciones, senal, accion)
        self.qrs = []

    # --- API ------------------------------------------------------------------
    def iniciar(self, odom_pose, ranges, angles):
        self.piloto.fijar_origen(odom_pose, ranges, angles)
        self.rumbo = self.rumbo_obj = 0.0

    def ver_senales(self, t, senales, ancho_img, odom_pose, ranges, angles):
        """Una lectura de camara. Devuelve True si la bandera quedo confirmada."""
        if self.estado != RECTO:
            return False    # girando o frenado: la camara barre senales que no son del pasillo
        pose = self.piloto.pose(odom_pose)
        r, a = nc.clean_scan(ranges, angles, self.piloto.nav.scan_min)
        vio_bandera, nav_tomada = False, False
        for s in senales or ():
            if s.tipo == "qr":
                if s.texto and s.texto not in self.qrs:
                    self.qrs.append(s.texto)
                    self.eventos.append(f"hito QR registrado: {s.texto}")
            elif s.tipo == "bandera":
                # cuenta solo cerca: de lejos (se ve a ~2.6 m) las caras salen chicas en el escaneo
                d = self._rango_senal(s, ancho_img, r, a)
                vio_bandera = d is None or d <= self.dp.d_bandera
            elif s.tipo in ("izq", "der", "prohibido") and not nav_tomada:
                nav_tomada = True    # detectar() ordena por area: la mas cercana primero
                self._leer_flecha(s, ancho_img, pose, r, a)
        if not nav_tomada:
            self._cand = None
        if vio_bandera and self.pol.senal_bandera == "llegada_b":
            self.cont_bandera += 1
            if self.cont_bandera >= self.dp.bandera_frames:
                self._llegar("bandera confirmada")
                return True
        else:
            self.cont_bandera = 0
        return False

    def ver_caras(self, t, fraccion):
        """fraccion del ancho de imagen de la cara mas grande (0 si no hay)."""
        if fraccion >= self.dp.cara_min:
            self.t_cara = t

    def resultado_mirar(self, hay_caras):
        """Tras accion "mirar": True -> llegada a B; False -> media vuelta y seguir."""
        if hay_caras:
            self._llegar("callejon con caras")
            return True
        self.callejones += 1
        if self.callejones > self.dp.max_callejones:
            self._llegar(f"{self.callejones} callejones sin caras: escaneo igual")
            return True
        if self.ultimo_cruce is not None:
            k, d = self.ultimo_cruce
            self.agotadas.setdefault(k, set()).add(d)
        self.eventos.append(f"callejon sin caras ({self.callejones}): media vuelta")
        self._girar_a(self.rumbo + math.pi)
        return False

    def paso(self, t, odom_pose, ranges, angles, bumper_hit=False):
        """Un ciclo. Devuelve (lin, ang, info) con info["accion"]."""
        pose = self.piloto.pose(odom_pose)
        if self.estado == LLEGADA:
            return 0.0, 0.0, self._info(pose, "llegada_b")
        if self.estado == MIRAR:
            if self.t_mirar0 is None:
                self.t_mirar0 = t
                b = math.radians(self.dp.barrido_deg)
                self._barrido = [self.rumbo + b, self.rumbo - b] if b > 0 else []
            if t - self.t_cara < self.dp.cara_vigencia:
                self.resultado_mirar(True)
                return 0.0, 0.0, self._info(pose, "llegada_b")
            if t - self.t_mirar0 < self.dp.t_mirar:
                return 0.0, 0.0, self._info(pose, "mirar")
            # al frente no hay caras: barrido corto (pueden estar en una pared lateral)
            while self._barrido:
                err = nc.wrap(self._barrido[0] - pose[2])
                if abs(err) > math.radians(self.dp.tol_giro_deg):
                    w = float(np.clip(self.dp.k_giro * err, -self.dp.w_barrido, self.dp.w_barrido))
                    return 0.0, w, self._info(pose, "mirar")
                self._barrido.pop(0)
            llego = self.resultado_mirar(False)
            return 0.0, 0.0, self._info(pose, "llegada_b" if llego else None)
        if self.estado == GIRAR:
            err = nc.wrap(self.rumbo_obj - pose[2])
            if abs(err) < math.radians(self.dp.tol_giro_deg):
                self.rumbo = self.rumbo_obj
                self.estado = RECTO
                self.piloto.reanudar()
            else:
                w = float(np.clip(self.dp.k_giro * err, -self.dp.w_giro, self.dp.w_giro))
                if abs(w) < self.dp.w_giro_min:
                    w = math.copysign(self.dp.w_giro_min, err)
                return 0.0, w, self._info(pose, None)

        # RECTO
        r, a = nc.clean_scan(ranges, angles, self.piloto.nav.scan_min)
        nav = self.piloto.nav
        front = rango_en(r, a, nc.wrap(self.rumbo - pose[2]), math.radians(8.0))
        front = nav.max_range if front is None else front
        alineado = abs(nc.wrap(pose[2] - self.rumbo)) < math.radians(self.dp.alineado_deg)
        fwd = self._adelante(pose)
        if alineado:
            accion = self._quizas_decidir(t, pose, r, a, front, fwd)
            if accion is not None:
                return 0.0, 0.0, self._info(pose, accion)
            if self.estado != RECTO:
                return 0.0, 0.0, self._info(pose, None)

        lento = (fwd is not None and -0.3 < fwd < self.dp.d_lento) or front < self.dp.d_lento
        self.piloto.pp.max_linear = self.dp.v_acercar if lento else self.dp.v_recto
        self.piloto.retorno = False
        # zanahoria por el rumbo (> 0.4 m: Piloto frena cerca de su meta), corrida hacia el
        # centro del pasillo: descentrado, side_scale baja la velocidad y roza las paredes
        z = self.dp.zanahoria
        err = ps.analizar_pasillo(r, a, self.pp_pasillo, nav).error_centro
        lat = float(np.clip(0.5 * err, -self.dp.centrado_max, self.dp.centrado_max))
        c, s_ = math.cos(self.rumbo), math.sin(self.rumbo)
        self.piloto.meta = (pose[0] + z * c - lat * s_, pose[1] + z * s_ + lat * c)
        # En tramo recto: pesa mas el rumbo del pasillo que la holgura (si no, Piloto, pensado
        # para ir a una meta, se abre al pasillo lateral antes de llegar a la flecha) y frena
        # mas tarde (slow_dist): la parada la decide d_frente, y stop_dist sigue igual.
        guardado = nav.w_goal, nav.slow_dist
        nav.w_goal = max(nav.w_goal, self.dp.w_goal_recto)
        nav.slow_dist = max(nav.stop_dist + 0.1, min(nav.slow_dist, self.dp.slow_dist_recto))
        try:
            lin, ang, info = self.piloto.paso(t, odom_pose, ranges, angles, bumper_hit=bumper_hit)
        finally:
            nav.w_goal, nav.slow_dist = guardado
        info["accion"] = None
        info["estado_directo"] = self.estado
        return lin, ang, info

    # --- internos -------------------------------------------------------------
    def _info(self, pose, accion):
        return dict(pose=pose, accion=accion, estado_directo=self.estado, llegada=False)

    def _llegar(self, motivo):
        self.estado = LLEGADA
        self.motivo_llegada = motivo
        self.eventos.append(f"llegada a B: {motivo}")

    def _girar_a(self, rumbo):
        # rumbos cardinales en el marco de A: no acumula error de giros anteriores
        self.rumbo_obj = nc.wrap(round(nc.wrap(rumbo) / (math.pi / 2.0)) * (math.pi / 2.0))
        self.estado = GIRAR

    def _adelante(self, pose):
        """Distancia hacia adelante (por el rumbo) a la flecha pendiente; None si no hay."""
        if self.pendiente is None or self.pendiente["xy"] is None:
            return None
        dx = self.pendiente["xy"][0] - pose[0]
        dy = self.pendiente["xy"][1] - pose[1]
        return dx * math.cos(self.rumbo) + dy * math.sin(self.rumbo)

    def _rumbo_senal(self, s, ancho):
        x, _, w, _ = s.bbox
        return rumbo_en_imagen(x + w / 2.0, ancho, self.dp.fov_deg)

    def _rango_senal(self, s, ancho, r, a):
        """Distancia a la senal: LiDAR en su rumbo (fiable) o el estimado por tamano."""
        d = rango_en(r, a, self._rumbo_senal(s, ancho), math.radians(self.dp.ventana_rango_deg))
        return d if d is not None else getattr(s, "distancia_m", None)

    def _leer_flecha(self, s, ancho, pose, r, a):
        b = self._rumbo_senal(s, ancho)
        d = self._rango_senal(s, ancho, r, a)
        xy = None if d is None else (pose[0] + d * math.cos(pose[2] + b),
                                     pose[1] + d * math.sin(pose[2] + b))
        if xy is not None and any(math.dist(xy, o) < self.dp.radio_obedecida for o in self.obedecidas):
            self._cand = None
            return
        if self.pendiente is not None and self.pendiente["tipo"] == s.tipo:
            px = self.pendiente["xy"]
            alineado = abs(nc.wrap(pose[2] - self.rumbo)) < math.radians(self.dp.alineado_deg)
            if xy is not None and alineado and (px is None or math.dist(xy, px) < self.dp.radio_misma):
                self.pendiente["xy"] = xy    # refina la posicion al acercarse
            return
        c = self._cand
        misma = (c is not None and c[0] == s.tipo
                 and (xy is None or c[1] is None or math.dist(xy, c[1]) < self.dp.radio_misma))
        n = c[2] + 1 if misma else 1
        self._cand = (s.tipo, xy, n)
        if n >= self.dp.confirmar:
            self.pendiente = dict(tipo=s.tipo, xy=xy)
            self._cand = None
            dist = "" if d is None else f" a {d:.2f} m"
            self.eventos.append(f"flecha confirmada: {s.tipo}{dist} (conf={s.confianza:.2f})")

    def _lado_libre(self, r, a, pose, lado):
        """Salida lateral abierta y ya estamos frente a ella: libres los rayos a 90 +- 15 grados
        (respecto del rumbo del pasillo). Con solo 90 se giraria al asomar el borde del hueco."""
        signo = 1.0 if lado == "izq" else -1.0
        bs = nc.wrap(self.rumbo + signo * np.radians([75.0, 90.0, 105.0]) - pose[2])
        return bool(np.all(nc.free_distance(r, a, bs, self.piloto.nav) >= self.dp.lado_abierto))

    def _quizas_decidir(self, t, pose, r, a, front, fwd):
        p = self.pendiente
        por_flecha = False
        if p is not None and fwd is not None:
            if p["tipo"] == "prohibido":
                # prohibido al frente: decide en la primera salida lateral o al llegar al letrero
                por_flecha = fwd <= self.dp.d_giro or (
                    fwd <= self.dp.d_prohibido
                    and (self._lado_libre(r, a, pose, "izq") or self._lado_libre(r, a, pose, "der")))
            else:
                por_flecha = fwd <= self.dp.d_giro and self._lado_libre(r, a, pose, p["tipo"])
        por_frente = front <= self.dp.d_frente
        if not (por_flecha or por_frente):
            return None
        if por_frente and t - self.t_cara < self.dp.cara_vigencia:
            self._llegar("caras al frente")
            return "llegada_b"

        pto = ps.analizar_pasillo(r, a, self.pp_pasillo, self.piloto.nav)
        opciones = set(pto.opciones)
        if por_frente:
            opciones.discard("recto")
        key = self._clave(pose)
        vis = self.visitas.setdefault(key, {})
        ago = self.agotadas.setdefault(key, set())
        mapa = {"recto": cardinal(self.rumbo), "izq": cardinal(self.rumbo + math.pi / 2.0),
                "der": cardinal(self.rumbo - math.pi / 2.0)}
        vis_rel = {op: vis.get(d, 0) for op, d in mapa.items()}
        ago_rel = {op for op, d in mapa.items() if d in ago}
        avisos = []
        tipo = p["tipo"] if p is not None else None
        accion = dc.decidir(opciones, senal=tipo, visitado=vis_rel, avisos=avisos,
                            agotadas=ago_rel, params=self.pol)
        for op in ago_rel:
            ago.add(mapa[op])
        self.eventos.extend(avisos)
        self.decisiones.append(dict(t=round(t, 2), pose=[round(v, 2) for v in pose],
                                    opciones=sorted(opciones), senal=tipo, accion=accion,
                                    motivo="flecha" if por_flecha else "frente"))
        if p is not None:
            if p["xy"] is not None:
                self.obedecidas.append(p["xy"])
            self.pendiente = None

        if accion == "retroceder" and tipo == "prohibido":
            self.eventos.append("prohibido sin otra salida: media vuelta")
            self._girar_a(self.rumbo + math.pi)
            return None
        if accion == "retroceder":
            self.estado = MIRAR
            self.t_mirar0 = None
            return None
        vis[cardinal(self.rumbo + math.pi)] = max(vis.get(cardinal(self.rumbo + math.pi), 0), 1)
        vis[mapa[accion]] = vis.get(mapa[accion], 0) + 1
        self.ultimo_cruce = (key, mapa[accion])
        if accion == "izq":
            self._girar_a(self.rumbo + math.pi / 2.0)
        elif accion == "der":
            self._girar_a(self.rumbo - math.pi / 2.0)
        return None

    def _clave(self, pose, radio=0.8):
        x, y = pose[0], pose[1]
        cerca = [k for k in set(self.visitas) | set(self.agotadas) if math.hypot(x - k[0], y - k[1]) < radio]
        if cerca:
            return min(cerca, key=lambda k: math.hypot(x - k[0], y - k[1]))
        return (round(x, 2), round(y, 2))
