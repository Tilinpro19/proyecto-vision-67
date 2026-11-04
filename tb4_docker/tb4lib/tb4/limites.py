"""Limites y valores por defecto del TurtleBot 4 Lite, y topicos estandar."""
START_LINEAR = 0.1    # m/s   velocidad inicial (lenta a proposito)
START_ANGULAR = 1.0   # rad/s
STEP_LINEAR = 0.05
STEP_ANGULAR = 0.2
LIMIT_LINEAR = 0.31   # maximo de fabrica del Create3 (sin safety_override)
LIMIT_ANGULAR = 1.9   # maximo del TurtleBot 4
DISCOVERY_WAIT = 8.0  # s esperando que el robot aparezca
LIDAR_ROT_DEG = 90.0  # el RPLIDAR del TB4 esta girado: +90 deja 0 = frente, + = izquierda

TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_SCAN = "/scan"
TOPIC_ODOM = "/odom"
TOPIC_CAMARA = "/oakd/rgb/preview/image_raw/compressed"
TOPIC_HAZARD = "/hazard_detection"
TOPICOS_CLAVE = (TOPIC_SCAN, TOPIC_ODOM, TOPIC_CMD_VEL, "/oakd/rgb/preview/image_raw")


def clamp(value, low, high):
    return max(low, min(high, value))
