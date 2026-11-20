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
    # Geometria adaptada al ancho del pasillo (medido con el LiDAR a +-90 grados): el centro
    # del cruce queda a ~ancho/2 de la pared del fondo. d_giro / d_frente son los TOPES.
    d_giro: float = 0.72         # m    obedecer la flecha a ancho/2 + 0.03 (tope)
    d_frente: float = 0.70       # m    pared del fondo (LiDAR crudo) a ancho/2 + 0.03 -> decidir
    d_min: float = 0.30          # m    nunca decidir mas cerca que esto de la pared del fondo
    pegarse_flecha: bool = True  # con flecha izq/der pendiente: girar recien a d_min de la pared
    ancho_inicial: float = 1.2   # m    ancho supuesto hasta medirlo
    lado_min: float = 0.55       # m    salida lateral: libre (crudo) >= max(lado_min,
    lado_extra: float = 0.35     #      ancho/2 + lado_extra) a 75, 90 y 105 grados
    # Cerca de una pared (acercandose a la flecha o llegando a A) va despacio con un margen
    # frontal chico para "pegarse" como en un pasillo angosto; lejos rige config.json.
    margen_cerca: float = 0.03   # m    safety_margin cerca
    stop_cerca: float = 0.05     # m    stop_dist cerca (libre ya inflado)
    min_free_cerca: float = 0.15  # m   min_free de choose_heading cerca (0.45 anula "recto")
    lateral_cerca: float = 0.02  # m    lateral_margin en pasillo angosto (0.57 m: 11 cm por lado)
    v_cerca: float = 0.10        # m/s  velocidad al llegar al punto de giro / a A
    cerca_a: float = 0.8         # m    a menos de esto de A, regreso con margen chico
    regreso_tramos: bool = True  # regreso deshaciendo los giros de la ida (seguidor de pasillo)
    pasado_max: float = 0.5      # m    pasarse de un punto de giro mas que esto -> girar igual
    ancho_estrecho: float = 0.8  # m    pasillo medido mas angosto: todo el regreso con margen
    v_estrecho: float = 0.15     # m/s  chico y a lo sumo a esta velocidad
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
    w_goal_recto: float = 4.0    # peso del rumbo en choose_heading (respaldo con Piloto)
    # Seguidor de pasillo propio del tramo recto (Piloto queda de respaldo para obstaculos)
    k_rumbo: float = 1.8         # rad/s por rad de error de rumbo
    k_centro: float = 1.5        # rad de correccion por m de descentrado
    centro_max: float = 0.35     # rad  correccion maxima por centrado
    w_recto: float = 0.9         # rad/s maximo en el tramo recto
    holgura_paso: float = 0.03   # m    franja (sobre el radio) que debe estar libre al frente
    d_parada: float = 0.24       # m    algo en la franja a menos de esto (crudo) -> lineal 0
    k_frenado: float = 0.8       # 1/s  lineal <= k (libre - d_parada)
    t_bloqueo: float = 2.0       # s    franja bloqueada este tiempo -> Piloto (esquiva/escape)
    t_respaldo: float = 5.0      # s    que maneja Piloto tras un bloqueo o choque
    slow_dist_recto: float = 0.50  # m libres (inflado) desde donde frena en el tramo recto
    centrado_max: float = 0.25   # m    correccion lateral maxima de la zanahoria
    cara_min: float = 0.05       # fraccion del ancho de imagen para contar una cara
    cara_vigencia: float = 1.0   # s    una cara vista hace menos de esto cuenta
    t_mirar: float = 1.2         # s    frenado mirando al frente en un callejon
    barrido_deg: float = 60.0    # tras mirar al frente, gira +-esto buscando caras (0 = no)
    w_barrido: float = 0.8       # rad/s del barrido (lento: la camara no sale movida)
    factor_timeout: float = 3.0  # giro / barrido que tarda mas que esto x lo esperado = trabado
    dist_cruce: float = 0.85     # m    entre decisiones / cruces pasados de largo
    max_callejones: int = 2      # callejones sin caras antes de escanear igual
    bandera_frames: int = 2
    d_bandera: float = 1.0       # m    la bandera confirma la llegada solo a menos de esto
    d_prohibido: float = 3.0     # m    prohibido pendiente a menos de esto: tomar la salida lateral
    # Tras cada giro: rumbo de la odometria corregido UNA vez con las dos paredes del pasillo
    # (en 0.57 m, 10-15 grados de deriva tras la media vuelta dejan al robot contra una pared)
    rumbo_paredes: bool = True
    paredes_max_deg: float = 25.0  # correccion mayor que esto = no son las paredes del pasillo
    paredes_paralelas_deg: float = 3.0
    rumbo_espera: float = 0.4    # s    tras el giro, antes de medir las paredes
    # A es una zona: a menos de a_suficiente sin acercarse mas durante t_sin_mejora -> llegada
    # (con el LiDAR atrasado Piloto quedaba 2 min escapando a 0.27 m de A sin declarar llegada)
    a_suficiente: float = 0.40   # m
    t_sin_mejora: float = 4.0    # s


def cardinal(rad):
    return int(round(nc.wrap(rad) / (math.pi / 2.0))) % 4


def rango_en(ranges, angles, bearing, ventana):
    """Mediana del LiDAR en `bearing` +- ventana (rad); None si no hay rayos validos."""
    if len(ranges) == 0:
        return None
    m = np.abs(nc.wrap(angles - bearing)) < ventana
    m &= np.isfinite(ranges)
    return float(np.median(ranges[m])) if np.any(m) else None


def pared_en(ranges, angles, bearing, ventana, dispersion=0.08):
    """Distancia a una pared en `bearing` +- ventana: la mediana, solo si los rayos de la
    ventana son parejos (p90 - p10 < dispersion). None en un hueco, su borde o sin datos."""
    if len(ranges) == 0:
        return None
    m = (np.abs(nc.wrap(angles - bearing)) < ventana) & np.isfinite(ranges)
    if np.count_nonzero(m) < 3:
        return None
    p10, p50, p90 = np.percentile(ranges[m], [10, 50, 90])
    return float(p50) if p90 - p10 < dispersion else None


def angulo_pasillo(ranges, angles, alcance=(-0.35, 0.6), lateral=(0.12, 0.9), resid=0.015,
                   extension=0.3, n_min=10):
    """Angulo de las paredes izquierda y derecha en el marco del robot, cada una ajustada con
    una recta a los puntos a +-lateral del robot. (ang_izq, d_izq, ang_der, d_der) o None si a
    algun lado no hay una pared recta (hueco, esquina, caja)."""
    if len(ranges) == 0:
        return None
    ok = np.isfinite(ranges)
    x, y = ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])
    out = []
    for signo in (1.0, -1.0):
        m = (x > alcance[0]) & (x < alcance[1]) & (signo * y > lateral[0]) & (signo * y < lateral[1])
        if np.count_nonzero(m) < n_min or np.ptp(x[m]) < extension:
            return None
        k, b = np.polyfit(x[m], y[m], 1)
        if np.std(y[m] - (k * x[m] + b)) > resid:
            return None
        ang = math.atan(k)
        out += [ang, abs(b) * math.cos(ang)]
    return tuple(out)


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
        self._xy_decision = None       # donde se tomo la ultima decision
        self._t_giro0, self._t_giro_max = None, 0.0
        self._t_prev, self._lin_prev = None, 0.0
        self._t_bloq0 = None
        self._piloto_hasta = -1e9
        self._giro_desde = (0.0, 0.0, 0.0)
        self.ruta = []                # giros de la ida: dict(xy, de, a)
        self.media_vuelta = False
        self.modo_regreso = None      # "tramos" | "migas"
        self._vuelta = None
        self._rumbo_pend = None        # t del fin del ultimo giro: corregir el rumbo con las paredes
        self._rumbo_lect = []
        self._a_mejor = (math.inf, None)   # (dist minima a A en el regreso, t)
        self._clave_pasada = None      # ultimo cruce pasado de largo
        self.ancho = self.dp.ancho_inicial
        self._ancho_n = 0
        self._dist_a = math.inf
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
            t_barrido = self.dp.factor_timeout * (2.0 * math.radians(self.dp.barrido_deg)
                                                  / self.dp.w_barrido + 1.0)
            if self._barrido and t - self.t_mirar0 - self.dp.t_mirar > t_barrido:
                self.eventos.append("barrido sin terminar: lo corto")
                self._barrido = []
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
            if self._t_giro0 is None:
                self._t_giro0 = t
                self._t_giro_max = self.dp.factor_timeout * (abs(err) / self.dp.w_giro + 1.0)
                self._giro_desde = (pose[0], pose[1], self.rumbo)
            vencido = t - self._t_giro0 > self._t_giro_max
            if vencido:
                # rueda trabada / el Create 3 no deja girar: que Piloto (escape) se encargue
                self.eventos.append(f"giro sin terminar en {t - self._t_giro0:.1f} s "
                                    f"(faltan {math.degrees(err):.0f} grados): sigo con Piloto")
            if abs(err) < math.radians(self.dp.tol_giro_deg) or vencido:
                x0, y0, de = self._giro_desde
                if abs(nc.wrap(self.rumbo_obj - de)) > 3.0:
                    self.media_vuelta = True      # ida con tramo muerto: regreso por migas
                self.ruta.append(dict(xy=(x0, y0), de=de, a=self.rumbo_obj))
                self.rumbo = self.rumbo_obj
                self.estado = RECTO
                self._rumbo_tras_giro(t)
                self.piloto.reanudar()
                self._lin_prev, self._t_prev = 0.0, None
            else:
                w = float(np.clip(self.dp.k_giro * err, -self.dp.w_giro, self.dp.w_giro))
                if abs(w) < self.dp.w_giro_min:
                    w = math.copysign(self.dp.w_giro_min, err)
                return 0.0, w, self._info(pose, None)

        # RECTO
        r, a = nc.clean_scan(ranges, angles, self.piloto.nav.scan_min)
        pose = self._corregir_rumbo(t, odom_pose, pose, r, a)
        nav = self.piloto.nav
        front = rango_en(r, a, nc.wrap(self.rumbo - pose[2]), math.radians(8.0))
        front = nav.max_range if front is None else front
        alineado = abs(nc.wrap(pose[2] - self.rumbo)) < math.radians(self.dp.alineado_deg)
        fwd = self._adelante(pose)
        if alineado:
            self._medir_ancho(pose, r, a)
            accion = self._quizas_decidir(t, pose, r, a, front, fwd)
            if accion is not None:
                return 0.0, 0.0, self._info(pose, accion)
            if self.estado != RECTO:
                return 0.0, 0.0, self._info(pose, None)

        lento = (fwd is not None and -0.3 < fwd < self.dp.d_lento) or front < self.dp.d_lento
        v = self.dp.v_acercar if lento else self.dp.v_recto
        estrecho = self.ancho < self.dp.ancho_estrecho
        if estrecho:
            v = min(v, self.dp.v_estrecho)
        cerca = front < self.dp.d_lento
        if cerca:
            # rampa hasta v_cerca en el punto de decision (no frena de golpe desde v_acercar)
            falta = front - self._d_frente()
            if fwd is not None and fwd > -0.3:
                falta = min(falta, fwd - self._d_giro())
            v = float(np.clip(self.dp.v_cerca + 0.6 * falta, self.dp.v_cerca, v))
        if bumper_hit and t >= self._piloto_hasta:
            self.eventos.append("bumper en tramo recto: maneja Piloto (escape)")
            self._a_respaldo(t)
        if t < self._piloto_hasta:
            lin, ang, info = self._paso_piloto(t, odom_pose, ranges, angles, pose, r, a, v,
                                               cerca, estrecho, bumper_hit)
        else:
            lin, ang, bloqueado = self._seguir_pasillo(t, pose, r, a, front, v)
            self.piloto.registrar(odom_pose, ranges, angles)
            if bloqueado:
                self._t_bloq0 = t if self._t_bloq0 is None else self._t_bloq0
                if t - self._t_bloq0 > self.dp.t_bloqueo:
                    self.eventos.append("paso bloqueado en el tramo recto: maneja Piloto")
                    self._a_respaldo(t)
            else:
                self._t_bloq0 = None
            info = dict(pose=pose, front=front - nav.inflate, nearest=nc.nearest(r),
                        estado="recto", llegada=False)
        info["frente_crudo"] = front
        info["accion"] = None
        info["estado_directo"] = self.estado
        return lin, ang, info

    def _a_respaldo(self, t):
        self._piloto_hasta = t + self.dp.t_respaldo
        self._t_bloq0 = None
        self.piloto.reanudar()
        self.piloto.prev_heading = 0.0

    def _seguir_pasillo(self, t, pose, r, a, front, v):
        """Recto por el rumbo del pasillo, centrado entre las paredes de +-90 grados, con la
        lineal limitada por lo que haya en la franja del ancho del robot. (lin, ang, bloqueado)"""
        dp, nav = self.dp, self.piloto.nav
        dt = 0.1 if self._t_prev is None else min(max(t - self._t_prev, 0.0), 0.5)
        self._t_prev = t
        err = nc.wrap(self.rumbo - pose[2])
        # pared_en: en el borde de una salida la mediana mezcla pared y hueco y el centrado
        # tiraria al robot hacia el hueco
        izq = pared_en(r, a, nc.wrap(self.rumbo + math.pi / 2 - pose[2]), math.radians(10.0))
        der = pared_en(r, a, nc.wrap(self.rumbo - math.pi / 2 - pose[2]), math.radians(10.0))
        corr = 0.0
        if izq is not None and der is not None and max(izq, der) < dp.ancho_inicial:
            corr = float(np.clip(dp.k_centro * (izq - der) / 2.0, -dp.centro_max, dp.centro_max))
        ang = dp.k_rumbo * (err + corr) + nc.wall_repulsion(r, a, nav)
        ang = float(np.clip(ang, -dp.w_recto, dp.w_recto))
        if len(r):
            x, y = r * np.cos(a), r * np.sin(a)
            m = (x > 0.05) & (np.abs(y) < nav.robot_radius + dp.holgura_paso)
            x_obs = float(np.min(x[m])) if np.any(m) else math.inf
        else:
            x_obs = math.inf
        lin = float(np.clip(dp.k_frenado * (x_obs - dp.d_parada), 0.0, v))
        if abs(err + corr) > 0.6:
            lin = 0.0       # muy torcido: primero corregir el rumbo
        lin = min(lin, self._lin_prev + self.piloto.pp.max_accel * dt)
        self._lin_prev = lin
        return lin, ang, x_obs < dp.d_parada + 0.04

    def _paso_piloto(self, t, odom_pose, ranges, angles, pose, r, a, v, cerca, estrecho, bumper_hit):
        """Respaldo: Piloto (VFH + escape) con la zanahoria por el rumbo del pasillo."""
        nav = self.piloto.nav
        self.piloto.pp.max_linear = v
        self.piloto.retorno = False        # zanahoria por el rumbo (> 0.4 m: Piloto frena cerca de su meta), corrida hacia el
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
        restaurar = self._margen_chico() if (cerca or estrecho) else (lambda: None)
        nav.slow_dist = max(nav.stop_dist + 0.1, min(nav.slow_dist, self.dp.slow_dist_recto))
        try:
            lin, ang, info = self.piloto.paso(t, odom_pose, ranges, angles, bumper_hit=bumper_hit)
        finally:
            restaurar()
            nav.w_goal, nav.slow_dist = guardado
        self._lin_prev = lin
        return lin, ang, info

    def _margen_chico(self):
        """Cambia los margenes de Piloto a los de 'cerca'; devuelve como restaurarlos."""
        nav = self.piloto.nav
        guardado = nav.safety_margin, nav.stop_dist, nav.min_free, nav.lateral_margin
        nav.safety_margin = min(nav.safety_margin, self.dp.margen_cerca)
        nav.stop_dist = min(nav.stop_dist, self.dp.stop_cerca)
        nav.min_free = min(nav.min_free, self.dp.min_free_cerca)
        if self.ancho < self.dp.ancho_estrecho:
            nav.lateral_margin = min(nav.lateral_margin, self.dp.lateral_cerca)

        def restaurar():
            nav.safety_margin, nav.stop_dist, nav.min_free, nav.lateral_margin = guardado
        return restaurar

    def regreso(self, t, odom_pose, ranges, angles, bumper_hit=False):
        """Regreso a A. Por tramos (por defecto): deshace los giros de la ida con el mismo
        seguidor de pasillo (en pasillos angostos Piloto no dobla las esquinas). Por migas
        (Piloto + ICP) si la ida tuvo una media vuelta, sin giros registrados utiles, o si
        el paso queda bloqueado. Devuelve (lin, ang, info) con info["llegada"], info["dist"]."""
        if self.modo_regreso is None:
            self.modo_regreso = ("tramos" if self.dp.regreso_tramos and not self.media_vuelta
                                 else "migas")
            self.eventos.append(f"regreso por {self.modo_regreso}")
            if self.modo_regreso == "tramos":
                pasos = [(g["xy"], nc.wrap(g["a"] + math.pi), nc.wrap(g["de"] + math.pi))
                         for g in reversed(self.ruta)]
                self._vuelta = dict(pasos=pasos, estado="girar",
                                    obj=self._cardinal_rad(self.rumbo + math.pi))
        out = None
        if self.modo_regreso == "tramos":
            out = self._regreso_tramos(t, odom_pose, ranges, angles, bumper_hit)
            if out is None:
                self.modo_regreso = "migas"
                self.eventos.append("regreso por tramos bloqueado: sigo por migas")
                self.piloto.volver()
        if out is None:
            out = self._regreso_migas(t, odom_pose, ranges, angles, bumper_hit)
        return self._llegada_por_zona(t, out)

    def _llegada_por_zona(self, t, out):
        lin, ang, info = out
        dist = info.get("dist", math.inf)
        if info.get("llegada"):
            return out
        if dist < self._a_mejor[0] - 0.02:
            self._a_mejor = (dist, t)
        elif (self._a_mejor[0] < self.dp.a_suficiente
              and t - self._a_mejor[1] > self.dp.t_sin_mejora):
            self.eventos.append(f"en la zona de A ({self._a_mejor[0]:.2f} m) y sin acercarme "
                                f"en {self.dp.t_sin_mejora:.0f} s: llegada")
            info = dict(info, llegada=True)
            return 0.0, 0.0, info
        return out

    @staticmethod
    def _cardinal_rad(rumbo):
        return nc.wrap(round(nc.wrap(rumbo) / (math.pi / 2.0)) * (math.pi / 2.0))

    def _regreso_tramos(self, t, odom_pose, ranges, angles, bumper_hit):
        v, dp = self._vuelta, self.dp
        pose = self.piloto.pose(odom_pose)
        r, a = nc.clean_scan(ranges, angles, self.piloto.nav.scan_min)
        dist_a = math.hypot(pose[0], pose[1])
        info = dict(pose=pose, dist=dist_a, llegada=False, estado="tramos")
        if bumper_hit:
            return None
        if v["estado"] == "girar":
            err = nc.wrap(v["obj"] - pose[2])
            if abs(err) < math.radians(dp.tol_giro_deg):
                self.rumbo = v["obj"]
                v["estado"] = "recto"
                self._rumbo_tras_giro(t)
                self._lin_prev, self._t_prev, self._t_bloq0 = 0.0, None, None
            else:
                w = float(np.clip(dp.k_giro * err, -dp.w_giro, dp.w_giro))
                if abs(w) < dp.w_giro_min:
                    w = math.copysign(dp.w_giro_min, err)
                return 0.0, w, info
        # recto hacia el proximo giro (o hacia A)
        pose = self._corregir_rumbo(t, odom_pose, pose, r, a)
        info["pose"] = pose
        c, s_ = math.cos(self.rumbo), math.sin(self.rumbo)
        front = rango_en(r, a, nc.wrap(self.rumbo - pose[2]), math.radians(8.0))
        front = self.piloto.nav.max_range if front is None else front
        if v["pasos"]:
            (px, py), rumbo_llegada, rumbo_salida = v["pasos"][0]
            falta = (px - pose[0]) * c + (py - pose[1]) * s_
            lado = "izq" if nc.wrap(rumbo_salida - self.rumbo) > 0 else "der"
            if ((falta <= 0.03 and self._lado_libre(r, a, pose, lado))
                    or falta < -dp.pasado_max or front <= self._d_frente()):
                v["pasos"].pop(0)
                v["obj"] = rumbo_salida
                v["estado"] = "girar"
                return 0.0, 0.0, info
        else:
            falta = pose[0] * -c + pose[1] * -s_          # hacia A por el rumbo
            if dist_a < self.piloto.pp.home_tol or falta <= 0.0 or (front <= dp.d_min and dist_a < 0.5):
                info["llegada"] = True
                return 0.0, 0.0, info
        vmax = dp.v_regreso if self.ancho >= dp.ancho_estrecho else min(dp.v_regreso, dp.v_estrecho)
        vmax = float(np.clip(dp.v_cerca + 0.6 * max(falta, 0.0), dp.v_cerca, vmax))
        lin, ang, bloqueado = self._seguir_pasillo(t, pose, r, a, front, vmax)
        if bloqueado:
            self._t_bloq0 = t if self._t_bloq0 is None else self._t_bloq0
            if t - self._t_bloq0 > dp.t_bloqueo:
                return None
        else:
            self._t_bloq0 = None
        return lin, ang, info

    def _regreso_migas(self, t, odom_pose, ranges, angles, bumper_hit=False):
        """Piloto por migas a v_regreso; en el ultimo tramo (o en pasillo angosto) despacio
        y con margen chico: A puede estar a ~0.3 m de una pared, mas cerca de lo que Piloto
        se acerca con los margenes de config.json (inflado + stop_dist ~0.63 m)."""
        pp = self.piloto.pp
        self.piloto.retorno = True
        pp.max_linear = self.dp.v_regreso
        restaurar = lambda: None  # noqa: E731
        estrecho = self.ancho < self.dp.ancho_estrecho
        if estrecho or self._dist_a < self.dp.cerca_a:
            restaurar = self._margen_chico()
            if estrecho:
                pp.max_linear = min(pp.max_linear, self.dp.v_estrecho)
            if self._dist_a < self.dp.cerca_a:
                pp.max_linear = min(pp.max_linear, max(self.dp.v_cerca, 0.25 * self._dist_a + 0.05))
        nav = self.piloto.nav
        slow = nav.slow_dist
        if estrecho or self._dist_a < self.dp.cerca_a:
            nav.slow_dist = max(nav.stop_dist + 0.1, min(slow, self.dp.slow_dist_recto))
        try:
            lin, ang, info = self.piloto.paso(t, odom_pose, ranges, angles, bumper_hit=bumper_hit)
        finally:
            nav.slow_dist = slow
            restaurar()
            pp.max_linear = self.dp.v_regreso
        self._dist_a = info["dist"]
        return lin, ang, info

    # --- internos -------------------------------------------------------------
    def _rumbo_tras_giro(self, t):
        self._rumbo_pend, self._rumbo_lect = t, []

    def _corregir_rumbo(self, t, odom_pose, pose, r, a):
        """Primera vez tras un giro que las dos paredes del pasillo se ven rectas, paralelas y
        al ancho medido: el rumbo real es el del pasillo menos el angulo de las paredes. Se
        corrige la pose de Piloto en el punto actual (x, y no cambian). Devuelve la pose.
        Espera rumbo_espera s tras el giro (el LiDAR tarda ~0.1 s por vuelta: girando a
        20 grados/s saldrian 2-3 grados de error) y usa la mediana de 3 lecturas."""
        dp = self.dp
        if self._rumbo_pend is None or not dp.rumbo_paredes or t - self._rumbo_pend < dp.rumbo_espera:
            return pose
        ap = angulo_pasillo(r, a)
        if ap is None:
            return pose
        ang_i, d_i, ang_d, d_d = ap
        if abs(ang_i - ang_d) > math.radians(dp.paredes_paralelas_deg):
            return pose
        if self._ancho_n and abs(d_i + d_d - self.ancho) > 0.08:
            return pose
        self._rumbo_lect.append(nc.wrap(self.rumbo - 0.5 * (ang_i + ang_d) - pose[2]))
        if len(self._rumbo_lect) < 3:
            return pose
        delta = float(np.median(self._rumbo_lect))
        self._rumbo_pend = None
        if abs(delta) > math.radians(dp.paredes_max_deg):
            self.eventos.append(f"rumbo por paredes descartado: {math.degrees(delta):+.0f} grados")
            return pose
        nueva = (pose[0], pose[1], nc.wrap(pose[2] + delta))
        self.piloto.corr = nc.compose(nueva, nc.inverse(nc.relative(self.piloto.origin, odom_pose)))
        if abs(delta) > math.radians(2.0):
            self.eventos.append(f"rumbo corregido con las paredes: {math.degrees(delta):+.0f} grados")
        return nueva

    def _info(self, pose, accion):
        return dict(pose=pose, accion=accion, estado_directo=self.estado, llegada=False)

    def _llegar(self, motivo):
        self.estado = LLEGADA
        self.motivo_llegada = motivo
        self.pendiente = None     # una flecha sin usar no debe cambiar los umbrales del regreso
        self.eventos.append(f"llegada a B: {motivo}")

    def _girar_a(self, rumbo):
        # rumbos cardinales en el marco de A: no acumula error de giros anteriores
        self.rumbo_obj = nc.wrap(round(nc.wrap(rumbo) / (math.pi / 2.0)) * (math.pi / 2.0))
        self.estado = GIRAR
        self._t_giro0 = None

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
        umbral = max(self.dp.lado_min, 0.5 * self.ancho + self.dp.lado_extra)
        front = rango_en(r, a, nc.wrap(self.rumbo - pose[2]), math.radians(4.0))
        for g in (75.0, 90.0, 105.0):
            d = rango_en(r, a, nc.wrap(self.rumbo + signo * math.radians(g) - pose[2]), math.radians(4.0))
            if d is not None and d < umbral:
                # Pegado al fondo el rayo de 75 grados pega en la pared del FRENTE, no en una
                # lateral (a 0.30 m y 5 grados torcido por la deriva: 0.88 m < umbral). Con el
                # impacto a la altura del frente (tolerancia por lo torcido) no tapa la salida.
                if front is not None and d * math.cos(math.radians(g)) >= front - 0.10:
                    continue
                return False
        return True

    def _flecha_pendiente(self):
        return (self.dp.pegarse_flecha and self.pendiente is not None
                and self.pendiente["tipo"] in ("izq", "der"))

    def _d_frente(self):
        if self._flecha_pendiente():
            return self.dp.d_min      # con flecha: hasta la pared, no decidir al verla
        return float(np.clip(0.5 * self.ancho + 0.03, self.dp.d_min, self.dp.d_frente))

    def _d_giro(self):
        if self._flecha_pendiente():
            return self.dp.d_min
        return float(np.clip(0.5 * self.ancho + 0.03, self.dp.d_min, self.dp.d_giro))

    def _medir_ancho(self, pose, r, a):
        """Ancho del pasillo: paredes a +-90 grados del rumbo (solo si se ven las dos)."""
        izq = pared_en(r, a, nc.wrap(self.rumbo + math.pi / 2 - pose[2]), math.radians(5.0))
        der = pared_en(r, a, nc.wrap(self.rumbo - math.pi / 2 - pose[2]), math.radians(5.0))
        if izq is None or der is None or max(izq, der) > self.dp.ancho_inicial:
            # un lado abierto (cruce, sala) o la ventana cae en el borde de un hueco (la
            # mediana mezcla pared y hueco: 0.28 + 1.04 en el intento real): no es el ancho
            return
        w = izq + der
        if self._ancho_n < 5 or abs(w - self.ancho) < 0.25:
            # las primeras lecturas fijan el ancho; despues solo cambios graduales (al asomar
            # una salida lateral el rayo de +-90 salta y no debe agrandar los umbrales)
            self.ancho = w if self._ancho_n == 0 else 0.7 * self.ancho + 0.3 * w
            self._ancho_n += 1

    def _quizas_decidir(self, t, pose, r, a, front, fwd):
        p = self.pendiente
        por_flecha = False
        if p is not None and fwd is not None:
            if p["tipo"] == "prohibido":
                # prohibido al frente: decide en la primera salida lateral o al llegar al letrero
                por_flecha = fwd <= self._d_giro() or (
                    fwd <= self.dp.d_prohibido
                    and (self._lado_libre(r, a, pose, "izq") or self._lado_libre(r, a, pose, "der")))
            else:
                por_flecha = fwd <= self._d_giro() and self._lado_libre(r, a, pose, p["tipo"])
        por_frente = front <= self._d_frente()
        lateral = self._lado_libre(r, a, pose, "izq") or self._lado_libre(r, a, pose, "der")
        lejos = (self._xy_decision is None
                 or math.dist(pose[:2], self._xy_decision) >= self.dp.dist_cruce)
        # Salida lateral sin flecha: se ignora (es lo rapido) mientras no haya pruebas de que el
        # camino recto estaba mal; tras un callejon sin caras, cada salida lateral se decide
        # con la memoria de visitas (asi vuelve por la rama que se salto).
        por_lateral = lateral and lejos and self.callejones > 0
        if not (por_flecha or por_frente or por_lateral):
            if lateral and lejos:
                self._pasar_cruce(pose)
            return None
        if por_frente and t - self.t_cara < self.dp.cara_vigencia:
            self._llegar("caras al frente")
            return "llegada_b"

        # salidas con umbrales relativos al ancho medido (pasillo.py usa 1.15 m fijos)
        opciones = {lado for lado in ("izq", "der") if self._lado_libre(r, a, pose, lado)}
        if not por_frente and front >= self._d_frente() + self.dp.lado_extra:
            opciones.add("recto")
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
                                    motivo=("flecha" if por_flecha else
                                            "frente" if por_frente else "lateral")))
        self._xy_decision = (pose[0], pose[1])
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

    def _pasar_cruce(self, pose):
        """Paso de largo por una salida lateral: queda en la memoria como cruce con el origen
        y el recto visitados, y como ultimo cruce (un callejon despues agota este recto)."""
        key = self._clave(pose)
        if key == self._clave_pasada:
            return
        self._clave_pasada = key
        vis = self.visitas.setdefault(key, {})
        atras, recto = cardinal(self.rumbo + math.pi), cardinal(self.rumbo)
        vis[atras] = max(vis.get(atras, 0), 1)
        vis[recto] = vis.get(recto, 0) + 1
        self.ultimo_cruce = (key, recto)

    def _clave(self, pose, radio=0.8):
        x, y = pose[0], pose[1]
        cerca = [k for k in set(self.visitas) | set(self.agotadas) if math.hypot(x - k[0], y - k[1]) < radio]
        if cerca:
            return min(cerca, key=lambda k: math.hypot(x - k[0], y - k[1]))
        return (round(x, 2), round(y, 2))
