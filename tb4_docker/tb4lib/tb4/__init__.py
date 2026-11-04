"""
tb4: todo lo que hace falta para hablar con el TurtleBot 4 Lite desde la PC.

    from tb4 import Robot
    with Robot() as bot:                 # usa TB4_IP / ROS_DOMAIN_ID del entorno
        bot.esperar_sensores()
        print(bot.odom.pose, bot.lidar.distancia(0.0))
        bot.avanzar(0.5)                 # m, con frenado por LiDAR
        bot.girar(math.pi / 2)           # rad
        img = bot.camara.ultima()        # BGR (numpy) o None

Piezas sueltas (para scripts que ya tienen su propio nodo):
    Sender, Lidar, Odom, Camara, Bumper, Anunciador, configurar_entorno, medir_desfase
"""
from .limites import (LIMIT_ANGULAR, LIMIT_LINEAR, START_ANGULAR, START_LINEAR,
                      STEP_ANGULAR, STEP_LINEAR, clamp)
from .conexion import configurar_entorno, topicos, esperar
from .movimiento import Sender
from .sensores import Lidar, Odom, Camara, Bumper, yaw_de_quaternion
from .anuncio import Anunciador
from .reloj import medir_desfase, sincronizar_hora
from .robot import Robot

__all__ = [
    "Robot", "Sender", "Lidar", "Odom", "Camara", "Bumper", "Anunciador",
    "configurar_entorno", "topicos", "esperar", "medir_desfase", "sincronizar_hora",
    "yaw_de_quaternion", "clamp", "LIMIT_LINEAR", "LIMIT_ANGULAR", "START_LINEAR",
    "START_ANGULAR", "STEP_LINEAR", "STEP_ANGULAR",
]
