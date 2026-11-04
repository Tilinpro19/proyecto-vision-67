import time
from tb4 import Robot

print("[..] Conectando con el robot...")
with Robot(ip="192.168.185.3", domain_id=67, camara=True, dry=True) as bot:
    bot.esperar_sensores(10.0, camara=True)
    print("=" * 50)
    print("CONEXION EXITOSA CON TURTLEBOT 4")
    print("=" * 50)
    print(f"Pose actual (odom): x={bot.pose[0]:.3f} m, y={bot.pose[1]:.3f} m, yaw={bot.pose[2]:.3f} rad")
    print(f"Distancia al frente: {bot.lidar.distancia(0.0):.2f} m")
    print(f"Distancia izquierda: {bot.lidar.distancia(1.57):.2f} m")
    print(f"Distancia derecha:   {bot.lidar.distancia(-1.57):.2f} m")
    img = bot.camara.latest()
    if img is not None:
        print(f"Camara OAK-D: frame recibido con resolucion {img.shape[1]}x{img.shape[0]}")
    print("=" * 50)
