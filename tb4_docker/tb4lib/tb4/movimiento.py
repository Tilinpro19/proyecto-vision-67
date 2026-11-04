"""Publicador de /cmd_vel que detecta solo si el robot usa Twist o TwistStamped."""
import time

from .conexion import esperar, topicos
from .limites import LIMIT_ANGULAR, LIMIT_LINEAR, TOPIC_CMD_VEL, clamp


class Sender:
    """Publica Twist o TwistStamped segun lo que use el topic del robot.

    node=None o dry=True -> no publica nada (modo prueba). Misma API que comun.Sender.
    tipo='stamped'|'twist' fuerza el tipo sin esperar al robot (p. ej. simuladores).
    """

    def __init__(self, node, topic=TOPIC_CMD_VEL, dry=False, tipo=None, limitar=True):
        self.node, self.topic, self.limitar = node, topic, limitar
        self.dry = dry or node is None
        self.last = (0.0, 0.0)
        if self.dry:
            return
        from geometry_msgs.msg import Twist, TwistStamped

        if tipo is None:
            tname = esperar(lambda: topicos(node).get(topic), "el topic " + topic)
            self.stamped = tname.endswith("TwistStamped")
        else:
            self.stamped = tipo == "stamped"
        self.msg_type = TwistStamped if self.stamped else Twist
        self.pub = node.create_publisher(self.msg_type, topic, 10)
        if tipo is None:
            esperar(lambda: node.count_subscribers(topic) > 0, "alguien escuchando en " + topic)
            print(f"[tb4] {topic} ({'TwistStamped' if self.stamped else 'Twist'}), "
                  f"{node.count_subscribers(topic)} suscriptor(es)")

    def send(self, linear, angular):
        if self.limitar:
            linear = clamp(linear, -LIMIT_LINEAR, LIMIT_LINEAR)
            angular = clamp(angular, -LIMIT_ANGULAR, LIMIT_ANGULAR)
        self.last = (float(linear), float(angular))
        if self.dry:
            return
        msg = self.msg_type()
        body = msg
        if self.stamped:
            # stamp en 0 (como 'ros2 topic pub'): sin NTP el reloj de la PC y el del robot
            # no coinciden y un stamp "viejo/futuro" puede descartarse
            msg.header.frame_id = "base_link"
            body = msg.twist
        body.linear.x = float(linear)
        body.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self):
        self.last = (0.0, 0.0)
        # Ctrl+C: rclpy ya cerro el contexto y publicar lanzaria RCLError
        if self.dry or not self.node.context.ok():
            return
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.02)
