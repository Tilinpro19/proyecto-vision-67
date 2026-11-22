#!/usr/bin/env python3
"""
Escucha el topic /nombre_reconocido publicado por el robot y lo anuncia
en voz alta inmediatamente a traves de los altavoces de la laptop (Windows SAPI / Linux TTS).
Ideal para la competencia cuando la mision corre on-board en la Raspberry Pi.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from facial import Voz

VENTANA_REPETIDO = 10.0  # s: mas que el burst de 3 publicaciones, menos que una nueva corrida


class EscuchadorAnuncio(Node):
    def __init__(self):
        super().__init__("escuchador_anuncio_laptop")
        self.voz = Voz("auto")
        self.sub = self.create_subscription(String, "/nombre_reconocido", self._on_nombre, 10)
        self.ultimo, self.t_ultimo = None, 0.0
        print(f"[ESCUCHANDO] Esperando anuncio en /nombre_reconocido (Voz: {self.voz.modo})...")

    def _on_nombre(self, msg):
        nombre = msg.data.strip()
        ahora = time.monotonic()
        # El robot publica cada anuncio 3 veces: se descartan solo las copias cercanas.
        # (Antes se ignoraba el mismo nombre para siempre y una 2a corrida quedaba muda.)
        if not nombre or (nombre == self.ultimo and ahora - self.t_ultimo < VENTANA_REPETIDO):
            return
        self.ultimo, self.t_ultimo = nombre, ahora
        print(f"\n{'='*50}\n  NOMBRE RECIBIDO: {nombre}\n{'='*50}\n", flush=True)
        frase = f"La persona identificada es {nombre}"
        self.voz.decir(frase)


def main():
    rclpy.init()
    node = EscuchadorAnuncio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.voz.cerrar()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
