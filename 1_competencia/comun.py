"""
Piezas compartidas por los scripts de seguimiento_vision/.

- Sender: publica en /cmd_vel como Twist o TwistStamped (lo que use el robot),
  igual que teleop_wasd.py, pero sobre un nodo que ya existe (asi el mismo
  nodo puede tambien suscribirse a la camara del robot).
- Limites de velocidad del TurtleBot 4 Lite.
"""
import sys
import time

START_LINEAR = 0.1    # m/s   velocidad inicial (lenta a proposito)
START_ANGULAR = 1.0   # rad/s
STEP_LINEAR = 0.05
STEP_ANGULAR = 0.2
LIMIT_LINEAR = 0.31   # maximo de fabrica del Create3 (sin safety_override)
LIMIT_ANGULAR = 1.9   # maximo del TurtleBot 4
DISCOVERY_WAIT = 8.0  # s esperando que el robot aparezca


def clamp(value, low, high):
    return max(low, min(high, value))


class Sender:
    """Publica Twist o TwistStamped segun lo que use el topic del robot.

    node=None o dry=True -> no publica nada (modo prueba).
    """

    def __init__(self, node, topic, dry):
        self.node = node
        self.dry = dry or node is None
        if self.dry:
            return
        from geometry_msgs.msg import Twist, TwistStamped

        tname = self._wait(lambda: self._topic_type(topic), "el topic " + topic)
        self.stamped = tname.endswith("TwistStamped")
        self.msg_type = TwistStamped if self.stamped else Twist
        self.pub = node.create_publisher(self.msg_type, topic, 10)

        self._wait(lambda: node.count_subscribers(topic) > 0,
                   "alguien escuchando en " + topic)
        print(f"Conectado: {topic} ({tname}), "
              f"{node.count_subscribers(topic)} suscriptor(es)")

    def _topic_type(self, topic):
        for name, types in self.node.get_topic_names_and_types():
            if name == topic:
                return types[0]
        return None

    def _wait(self, fn, what):
        import rclpy
        end = time.monotonic() + DISCOVERY_WAIT
        while time.monotonic() < end:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception:
                pass
            res = fn()
            if res:
                return res
            time.sleep(0.1)
        sys.exit(f"No aparece {what}. Revisa: misma red, ping al robot, "
                 "mismo ROS_DOMAIN_ID que el robot y ROS_STATIC_PEERS.")

    def send(self, linear, angular):
        if self.dry:
            return
        msg = self.msg_type()
        body = msg
        if self.stamped:
            # stamp en 0 (como 'ros2 topic pub'): sin NTP el reloj de esta PC y el
            # del robot no coinciden y un stamp "viejo/futuro" puede descartarse
            msg.header.frame_id = "base_link"
            body = msg.twist
        body.linear.x = float(linear)
        body.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self):
        # Ctrl+C: rclpy ya cerro el contexto y publicar lanzaria RCLError
        if self.dry or not self.node.context.ok():
            return
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.02)


def apply_params(objs, config_path=None, overrides=()):
    """Ajusta campos de dataclasses (NavParams, PilotoParams, ...) desde un JSON plano
    {"campo": valor} y/o '--param campo=valor'. Avisa de nombres desconocidos."""
    import json
    values = {}
    if config_path:
        with open(config_path, encoding="utf-8") as f:
            values.update({k: v for k, v in json.load(f).items() if not k.startswith("_")})
    for item in overrides:
        k, _, v = item.partition("=")
        values[k.strip()] = v.strip()
    for k, v in values.items():
        targets = [o for o in objs if hasattr(o, k)]
        if not targets:
            print(f"[params] '{k}' no existe, se ignora")
        for o in targets:
            old = getattr(o, k)
            setattr(o, k, type(old)(v) if not isinstance(old, bool) else str(v).lower() in ("1", "true", "si"))
    return values
