"""Reloj Pi / Create 3 / PC. Sin internet la Pi arranca con la hora desfasada y SLAM
descarta los /scan. Reune chequeo_reloj_tb4.py y la parte de hora de tb4_connect.sh."""
import os
import subprocess
import time


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def medir_desfase(node, timeout=15.0):
    """{'pc_pi': s, 'pc_create3': s, 'pi_create3': s} (None si no llega el topico).
    pc_pi se mide con /scan (reloj de la Pi) y pc_create3 con /odom (Create 3)."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    lag = {}
    subs = [
        node.create_subscription(LaserScan, "/scan", lambda m: lag.setdefault("scan", time.time() - _stamp(m)),
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, "/odom", lambda m: lag.setdefault("odom", time.time() - _stamp(m)),
                                 qos_profile_sensor_data),
    ]
    end = time.time() + timeout
    while len(lag) < 2 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    for s in subs:
        node.destroy_subscription(s)
    s, o = lag.get("scan"), lag.get("odom")
    return {"pc_pi": s, "pc_create3": o,
            "pi_create3": (o - s) if s is not None and o is not None else None}


def sincronizar_hora(ip=None, user=None, reiniciar_bringup=True):
    """Pone la hora de esta PC en la Pi por SSH (pide contrasenas si no hay clave/sudo -n)
    y reinicia turtlebot4.service. Devuelve el codigo de salida de ssh."""
    ip = ip or os.environ.get("TB4_IP")
    user = user or os.environ.get("TB4_USER", "ubuntu")
    if not ip:
        raise ValueError("Falta la IP del robot (TB4_IP)")
    cmd = f"date -u -s @{int(time.time())}"
    if reiniciar_bringup:
        cmd += " && systemctl restart turtlebot4.service"
    cmd += " && curl -s -X POST http://192.168.186.2/api/restart-ntpd >/dev/null 2>&1 || true"
    return subprocess.call(["ssh", "-t", f"{user}@{ip}", f"sudo bash -c '{cmd}'"])
