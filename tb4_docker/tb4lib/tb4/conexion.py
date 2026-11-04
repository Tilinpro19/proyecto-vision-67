"""Variables de entorno DDS (lo que hacia tb4_connect.sh) y utilidades de descubrimiento.

configurar_entorno() debe llamarse ANTES de rclpy.init(): Fast DDS lee el entorno al crear
el participante. Dentro del Docker el entrypoint ya lo hace con TB4_IP / ROS_DOMAIN_ID.
"""
import os
import time

from .limites import DISCOVERY_WAIT

_PERFILES = ("/opt/tb4/fastdds_pc.xml",
             os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                          "fastdds_pc.xml"))


def configurar_entorno(ip=None, domain_id=None, modo="auto"):
    """Fija ROS_DOMAIN_ID, RMW y, en modo 'peers' (WSL / redes sin multicast),
    ROS_STATIC_PEERS + el perfil Fast DDS de 100 peers. 'auto' = peers si hay IP."""
    ip = ip or os.environ.get("TB4_IP")
    if domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(int(domain_id))
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    if modo == "auto":
        modo = os.environ.get("TB4_MODE", "auto")
        if modo == "auto":
            modo = "peers" if ip else "multicast"
    if modo == "peers" and ip:
        os.environ["ROS_STATIC_PEERS"] = ip
        perfil = next((p for p in _PERFILES if os.path.exists(p)), None)
        if perfil:
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = perfil
    else:
        os.environ.pop("ROS_STATIC_PEERS", None)
    if ip:
        os.environ["TB4_IP"] = ip
    return modo


def topicos(node):
    """{nombre: tipo} de los topicos visibles."""
    return {n: t[0] for n, t in node.get_topic_names_and_types() if t}


def esperar(fn, que, timeout=DISCOVERY_WAIT, node=None):
    """Repite fn() hasta que devuelva algo verdadero; si no, aborta con un mensaje util."""
    import rclpy
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        res = fn()
        if res:
            return res
        if node is not None:
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            time.sleep(0.2)
    raise TimeoutError(f"No aparece {que}. Revisa: misma red, ping al robot, mismo "
                       "ROS_DOMAIN_ID que el robot y ROS_STATIC_PEERS (TB4_MODE=peers).")
