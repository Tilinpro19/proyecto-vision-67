"""
Seguimiento de pasillo y deteccion de puntos de decision (cruces, esquinas y callejones)
con el LiDAR 2D para la Tarea A guiada (Fase 2). Sin ROS: solo numpy + nav_core.
"""
import math
from dataclasses import dataclass, field

import numpy as np

import nav_core as nc


@dataclass
class PasilloParams:
    open_front: float = 0.90     # m libres al frente para considerar disponible "recto"
    block_front: float = 0.80    # m al frente por debajo de los cuales se obliga a decidir
    open_side: float = 1.15      # m libres a izq/der para considerar un pasillo lateral abierto
    wall_max: float = 1.05       # m maximos para usar una pared en el centrado lateral
    k_center: float = 0.85       # rad de correccion de rumbo por m de descentrado
    max_corr_rad: float = 0.45   # limite de correccion angular por centrado


@dataclass
class PuntoDecision:
    opciones: set = field(default_factory=set)   # subconjunto de {"izq", "recto", "der"}
    en_cruce: bool = False                       # True si hay bifurcacion lateral o frente cerrado
    dist_frente: float = 0.0
    dist_izq: float = 0.0
    dist_der: float = 0.0
    error_centro: float = 0.0


_FRONT_BEARINGS = np.radians([-15.0, -7.5, 0.0, 7.5, 15.0])
_LEFT_BEARINGS = np.radians([60.0, 72.0, 84.0, 96.0, 108.0])
_RIGHT_BEARINGS = np.radians([-108.0, -96.0, -84.0, -72.0, -60.0])


def analizar_pasillo(ranges, angles, pp=PasilloParams(), nav=nc.NavParams()):
    """Analiza un scan LiDAR en el marco del robot (0=frente, +pi/2=izq, -pi/2=der)
    y devuelve `PuntoDecision` con las salidas disponibles y si estamos en un cruce."""
    r, a = nc.clean_scan(ranges, angles, nav.scan_min)
    if len(r) == 0:
        return PuntoDecision(opciones={"izq", "recto", "der"}, en_cruce=False,
                             dist_frente=nav.max_range, dist_izq=nav.max_range, dist_der=nav.max_range)

    f_free = float(np.max(nc.free_distance(r, a, _FRONT_BEARINGS, nav)))
    l_free = float(np.max(nc.free_distance(r, a, _LEFT_BEARINGS, nav)))
    r_free = float(np.max(nc.free_distance(r, a, _RIGHT_BEARINGS, nav)))

    opciones = set()
    if f_free >= pp.open_front:
        opciones.add("recto")
    if l_free >= pp.open_side:
        opciones.add("izq")
    if r_free >= pp.open_side:
        opciones.add("der")

    # Distancia perpendicular a las paredes laterales inmediatas (70..110 grados) para centrarse
    mask_l = (a >= math.radians(65)) & (a <= math.radians(115))
    mask_r = (a <= -math.radians(65)) & (a >= -math.radians(115))
    d_pared_izq = float(np.median(r[mask_l])) if np.any(mask_l) else l_free
    d_pared_der = float(np.median(r[mask_r])) if np.any(mask_r) else r_free

    err = 0.0
    if d_pared_izq < pp.wall_max and d_pared_der < pp.wall_max:
        err = d_pared_izq - d_pared_der
    elif d_pared_izq < pp.wall_max:
        err = d_pared_izq - 0.55
    elif d_pared_der < pp.wall_max:
        err = 0.55 - d_pared_der

    hay_lateral = ("izq" in opciones) or ("der" in opciones)
    frente_cerrado = f_free < pp.block_front
    en_cruce = hay_lateral or frente_cerrado

    return PuntoDecision(
        opciones=opciones,
        en_cruce=en_cruce,
        dist_frente=f_free,
        dist_izq=l_free,
        dist_der=r_free,
        error_centro=err,
    )


def comando_pasillo(ranges, angles, max_linear=0.18, max_angular=1.1,
                    rumbo_objetivo=0.0, pp=PasilloParams(), nav=nc.NavParams()):
    """Calcula (lin, ang, pto_decision) para avanzar centrado por el pasillo o girar
    hacia `rumbo_objetivo` (en rad en el marco del robot)."""
    pto = analizar_pasillo(ranges, angles, pp, nav)
    r, a = nc.clean_scan(ranges, angles, nav.scan_min)
    if len(r) == 0:
        return max_linear, 0.0, pto

    # Si vamos recto por el pasillo, corregir suavemente hacia el centro
    corr = float(np.clip(pp.k_center * pto.error_centro, -pp.max_corr_rad, pp.max_corr_rad))
    deseado = nc.wrap(rumbo_objetivo + (corr if abs(rumbo_objetivo) < 0.25 else 0.0))

    h = nc.choose_heading(r, a, deseado, prev=deseado, p=nav)
    if h is None:
        h = nc.most_open(r, a, nav)
    front_direct = float(nc.free_distance(r, a, 0.0, nav)[0])
    eff_free = min(float(nc.free_distance(r, a, h, nav)[0]), front_direct)
    lin, ang = nc.speed_command(h, eff_free, max_linear, max_angular, nav)
    ang = float(np.clip(ang + nc.wall_repulsion(r, a, nav), -max_angular, max_angular))
    return lin, ang, pto
