# Métodos Implementados y Justificación Técnica · Proyecto Visión 67

Este documento resume los algoritmos, arquitecturas y mecanismos de infraestructura utilizados en el **TurtleBot 4 Lite** (base iRobot Create 3 + RPLIDAR A1 + cámara OAK-D Lite + Raspberry Pi 4), explicando **por qué benefician al robot** y **por qué se eligieron** frente a alternativas más pesadas.

---

## 1. Navegación Reactiva y Evasión de Obstáculos (`1_competencia/nav_core.py`, `navegacion.py`)

| Método / Componente | Implementación en el Código | Beneficio para el Robot | Justificación Técnica |
|---|---|---|---|
| **VFH-lite con inflado geométrico** | `NavParams` (`robot_radius=0.17` m, `safety_margin=0.09` m, `n_headings=73`), `free_distance()`, `side_margin()`, `choose_heading()`, `wall_repulsion()` | Evalúa 73 rumbos candidatos (-180° a +180°) considerando el radio real del Create 3 y prioriza avanzar por el centro de pasillos (`side_margin`). | A diferencia de Nav2 completo (costmap 2D + DWB/TEB), VFH-lite opera directamente sobre el arreglo 1D del LiDAR en `numpy` puro, reduciendo el uso de CPU y evitando latencia de planificación en la Raspberry Pi 4. |
| **Control cinemático suave y salvaguarda frontal** | `speed_command()`, `angular_limit()`, `Piloto.paso()` (`eff_free = min(free, info["front"])`), `config.json` (`max_linear=0.18`, `max_accel=0.35`, `max_decel=0.8`, `max_alpha=2.2`) | Evita aceleraciones bruscas que hacen patinar las ruedas del Create 3 y prohíbe avanzar si el frente físico está obstruido mientras gira. | El deslizamiento de ruedas degrada la odometría de los *encoders*. Limitar aceleración lineal y angular mantiene la odometría estable y evita colisiones por giro cercano a obstáculos. |
| **Memoria espacial, retorno A ← B e ICP 2D** | `SpatialKDTree2D`, `BreadcrumbGraph` (`prune_branch`), `VirtualObstacles`, `icp_2d()`, `pick_waypoint()` | Registra migas de pan durante la ida, poda ramas de callejones sin salida, recuerda obstáculos bajos detectados por el *bumper* y corrige la deriva de odometría alineando escaneos con *keyframes*. | En pistas desconocidas sin mapa previo y con tiempo limitado, SLAM + AMCL puede fallar o saturar la Pi si hay desfase de reloj. Las migas de pan + ICP 2D permiten volver al punto de partida con bajo costo computacional. |
| **Detección de concavidades y atascos** | `detect_concavity()`, `ProgressMonitor`, `most_open()` | Detecta trampas en U de forma temprana y activa maniobras de escape cuando el robot deja de progresar (`stuck_time=4.5` s en `config.json`). | Los métodos puramente reactivos pueden quedar atrapados en mínimos locales (callejones en U); combinar `detect_concavity` con `VirtualObstacles` fuerza la salida por la periferia. |

---

## 2. Visión Computacional e Interacción (`1_competencia/facial.py`, `gestos_teleop.py`, `mano_teleop.py`)

| Método / Componente | Implementación en el Código | Beneficio para el Robot | Justificación Técnica |
|---|---|---|---|
| **Detección y reconocimiento facial offline** | `Rostros` (YuNet `face_detection_yunet_2023mar.onnx` + SFace `face_recognition_sface_2021dec.onnx`), `calidad()`, `Galeria` (caché `.cache.npz`) | Identifica rostros en la meta B sin depender de internet, filtra *frames* borrosos mediante nitidez por Laplaciano y precarga embeddings en segundos. | Ambos modelos ONNX corren en CPU sin GPU dedicada. Evaluar nitidez y tamaño de cara antes de promediar los mejores *frames* (`TOP_FRAMES=3`) evita falsos positivos por vibración o movimiento del robot. |
| **Control gestual con seguridad LiDAR** | `classify_pose()`, `ControlGestos`, `Gestos`, `front_limit()` usando `HandDetector` (MediaPipe Hand Landmarker, 21 puntos) | Permite armar, dirigir y disparar acciones sin teclado ni mando físico, y bloquea el avance si hay un obstáculo delante dentro de `stop_dist`. | Clasificar poses mediante geometría angular relativa de articulaciones MCP tolera variaciones de distancia e inclinación de la mano mejor que comparar coordenadas crudas. `front_limit()` añade una barrera física sobre la orden humana. |

---

## 3. Infraestructura de Laboratorio, Red y Despliegue (`tb4_connect.sh`, `desplegar_en_robot.sh`, `fastdds_pc.xml`)

| Método / Componente | Implementación en el Código | Beneficio para el Robot | Justificación Técnica |
|---|---|---|---|
| **Ejecución a bordo (*Edge Execution*)** | `desplegar_en_robot.sh`, `tarea_a_onboard.sh`, `tarea_b_onboard.sh` | Ejecuta el lazo de control `/scan → /cmd_vel` directamente en la Raspberry Pi del robot. | Elimina la latencia de red en el lazo crítico de control y hace que la misión sea inmune a microcortes o saturación del Wi-Fi del laboratorio. |
| **Fast DDS en modo *unicast / static peers*** | `tb4_connect.sh`, `fastdds_pc.xml` (`ROS_STATIC_PEERS`, `FASTRTPS_DEFAULT_PROFILES_FILE`) | Garantiza el descubrimiento de tópicos ROS 2 entre laptop (WSL 2) y robot en redes donde el *multicast* está bloqueado. | En redes universitarias/WSL 2, el descubrimiento *multicast* por defecto de DDS suele fallar silenciosamente; el perfil estático asegura visibilidad bidireccional para telemetría y diagnóstico. |
| **Sincronización de reloj Laptop → Pi → Create 3** | `tb4_connect.sh --hora`, `scripts/chequeo_reloj_tb4.py`, `POST http://192.168.186.2/api/restart-ntpd` | Reduce y mitiga el desfase temporal entre los *timestamps* de `/scan` (Pi) y `/odom` (Create 3). | Sin internet en el laboratorio, el Create 3 y la Pi pueden arrancar con fechas distintas; sincronizar la Pi desde la laptop y reiniciar `ntpd` en el Create 3 evita que los nodos descarten mensajes por desfase de reloj. |
