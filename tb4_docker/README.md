# tb4_docker

Imagen con ROS 2 Jazzy, mensajes del TurtleBot 4 / Create 3, OpenCV y MediaPipe, más la libreria `tb4`.

## Construir
```bash
docker compose build
```

## Usar (laptop en la misma red que el robot)
```bash
TB4_IP=<IP_ROBOT> ROS_DOMAIN_ID=67 docker compose run --rm tb4            # shell con ROS listo
TB4_IP=<IP_ROBOT> ROS_DOMAIN_ID=67 docker compose run --rm tb4 tb4 check  # diagnostico
```
Comandos: `tb4 check | reloj | camara | lidar | teleop | mover V W SEG | avanzar M | girar GRADOS`.
Ventanas (camara/lidar): antes ejecutar `xhost +local:` en el host.

`TB4_MODE`: `peers` (unicast, por defecto si hay TB4_IP) o `multicast`.
El compose monta `../paquete_proyecto_final` en `/ws`; ajusta esa ruta si tu proyecto esta en otro sitio.

## Libreria
```python
from tb4 import Robot
with Robot(camara=False) as bot:
    bot.esperar_sensores()
    bot.avanzar(0.5); bot.girar(1.57)
    print(bot.pose, bot.lidar.distancia(0.0))
```
`info/recolectar.sh` vuelca topicos, nodos, servicios, acciones, mensajes y parametros del robot.
