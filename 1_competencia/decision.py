"""
Politica pura de decision en cruces: senal > exploracion (con prioridad fisica del LiDAR).
Sin ROS ni OpenCV (Fase 3).

Reglas:
  1. Senal "bandera" -> retorna "llegada_b" (si senal_bandera == "llegada_b").
  2. Senal "qr"      -> registra el hito y decide con el LiDAR / exploracion.
  3. Senal "prohibido" -> marca la rama frontal ("recto") como agotada y elige otra salida libre.
  4. Senal "izq" / "der" -> si la salida existe en las opciones fisicas del LiDAR, la obedece;
                            si contradice al LiDAR (hay pared), gana el LiDAR y emite aviso.
  5. Sin senal (o tras descartar/registrar) -> elige la opcion menos visitada entre las libres.
  6. Empate -> orden fijo `preferencia` (por defecto: "recto", "izq", "der").
  7. Callejon (ninguna opcion libre no agotada) -> retorna "retroceder".
"""
from dataclasses import dataclass

PREFERENCIA_DEFAULT = ("recto", "izq", "der")


@dataclass
class DecisionParams:
    senal_prohibido: str = "buscar_otro"   # "buscar_otro" | "retroceder"
    senal_bandera: str = "llegada_b"       # "llegada_b" | "ignorar"
    senal_qr: str = "registrar"            # "registrar" | "ignorar"
    regreso_modo: str = "migas"            # "migas" (sin obedecer flechas al volver)
    flechas_marco: str = "robot"           # "robot" (izq/der relativas al robot)


def _extraer_senal(senal):
    if senal is None:
        return None, None
    if isinstance(senal, str):
        return senal, None
    return getattr(senal, "tipo", None), getattr(senal, "texto", None)


def _visitas_de(visitado, op):
    if visitado is None:
        return 0
    if isinstance(visitado, dict):
        return int(visitado.get(op, 0))
    return 1 if op in visitado else 0


def decidir(opciones, senal=None, visitado=None, preferencia=PREFERENCIA_DEFAULT,
            avisos=None, agotadas=None, params=None):
    """Funcion pura de decision en un punto de decision.
    - `opciones`: iterable/set de salidas fisicamente libres segun el LiDAR (ej. {"izq", "recto"}).
    - `senal`: `None`, `str` ("izq"|"der"|"prohibido"|"bandera"|"qr") o instancia de `Senal`.
    - `visitado`: `dict[op -> int]` con conteo de visitas por rama (o `set` de ramas visitadas).
    - `preferencia`: tupla con orden de desempate (por defecto `("recto", "izq", "der")`).
    - `avisos`: lista opcional donde se agregan logs de contradiccion o hitos QR.
    - `agotadas`: set opcional de ramas bloqueadas/agotadas en este cruce (se actualiza si hay "prohibido").
    Devuelve la accion elegida: `"recto"`, `"izq"`, `"der"`, `"retroceder"` o `"llegada_b"`."""
    p = params or DecisionParams()
    libres = set(opciones or ())
    bloqueadas = agotadas if agotadas is not None else set()
    tipo, texto = _extraer_senal(senal)

    # 1. Bandera de cuadros -> llegada a B
    if tipo == "bandera" and p.senal_bandera == "llegada_b":
        if avisos is not None:
            avisos.append("senal bandera: llegada a B confirmada")
        return "llegada_b"

    # 2. QR de hito -> solo registrar y seguir con exploracion
    if tipo == "qr":
        if p.senal_qr == "registrar" and avisos is not None:
            avisos.append(f"hito QR registrado: {texto or 'sin_texto'}")
        tipo = None

    # 3. Prohibido -> marcar camino frontal (o unico camino) como agotado y buscar otro
    if tipo == "prohibido":
        rama_prohibida = "recto" if "recto" in libres else (next(iter(libres)) if len(libres) == 1 else "recto")
        bloqueadas.add(rama_prohibida)
        if avisos is not None:
            avisos.append(f"senal prohibido: rama '{rama_prohibida}' marcada como agotada")
        if p.senal_prohibido == "retroceder":
            return "retroceder"
        tipo = None

    disponibles = libres - bloqueadas

    # 4. Flecha izquierda / derecha
    if tipo in ("izq", "der"):
        if tipo in disponibles:
            if avisos is not None:
                avisos.append(f"senal '{tipo}' obedecida")
            return tipo
        if avisos is not None:
            avisos.append(
                f"AVISO: senal '{tipo}' contradice al LiDAR (disponibles={sorted(disponibles)}); gana el LiDAR"
            )

    # 5. Callejon o todas las ramas agotadas -> retroceder al ultimo cruce
    if not disponibles:
        if avisos is not None:
            avisos.append("callejon sin salidas disponibles: retroceder por migas")
        return "retroceder"

    # 6. Sin senal (o senal contradicha): elegir la rama menos explorada; desempate por `preferencia`
    orden_pref = {op: idx for idx, op in enumerate(preferencia)}
    elegida = min(
        disponibles,
        key=lambda op: (_visitas_de(visitado, op), orden_pref.get(op, 99), op),
    )
    return elegida
