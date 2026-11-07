# Descubrimientos del laboratorio (TurtleBot 4 Lite)

Lo que aprendimos a golpes y no está obvio en la documentación oficial. Cada punto indica el síntoma, la causa y qué hacer. Los scripts y guías que lo mitigan están enlazados.

Contexto fijo: TB4 Lite, ROS 2 Jazzy, Fast DDS, `ROS_DOMAIN_ID=67`, Pi en `wlan0` 192.168.0.104 / `eth0` 192.168.185.3, laptop con WSL 2.

## 1. Red

### 1.1 El Wi-Fi UTEC bloquea DDS
- **Síntoma:** `ping` y SSH a 192.168.0.104 funcionan, pero `ros2 topic list` solo muestra `/parameter_events` y `/rosout`.
- **Causa:** UTEC-Alumnos / UTEC-Eventos rutean IP pero no dejan pasar el descubrimiento DDS (diagnosticado con `tcpdump` en el robot).
- **Qué hacer:** trabajar en **Lab_Computech** o por cable. Cable directo: laptop `192.168.185.10/24` sin gateway, robot `192.168.185.3`.
- **Sin descartar a ciegas:** si DDS falla con ping OK, mira primero la red de Windows (`Get-NetConnectionProfile`) antes de tocar firewall, RMW o peers.

### 1.2 El bringup solo anuncia las IP que tenía al arrancar
- **Síntoma:** tras cambiar cable ↔ Wi-Fi el robot "desaparece" aunque haya ping.
- **Qué hacer:** `sudo systemctl restart turtlebot4.service` en el robot y esperar ~30 s.

### 1.3 Fast DDS con peers estáticos solo contacta 4 participantes
- **Causa:** con `ROS_STATIC_PEERS`, `maxInitialPeersRange` vale 4; el bringup lanza muchos procesos, así que `/cmd_vel`, `/odom`, `/oakd`... quedan fuera.
- **Qué hacer:** usar `fastdds_pc.xml` (sube el rango a 100) con `export FASTRTPS_DEFAULT_PROFILES_FILE=~/turtleclaude4/fastdds_pc.xml`. `tb4_connect.sh` lo deja armado.

### 1.4 El lab es una LAN sin internet y con 3 robots
- Sin internet no hay NTP ni `apt`/`pip`: instalar todo antes, en otra red.
- Cámara por Wi-Fi: usar siempre `/compressed`; 3 robots con imagen cruda saturan el router.
- Un `ROS_DOMAIN_ID` y un hostname por robot.
- DDS no cruza bien Docker ni WSL2 en NAT: usar red espejo (mirrored) o *Fast DDS Discovery Server*.

## 2. Reloj (el problema más traicionero)

- **Síntoma:** SLAM no genera `/map`; logs con `Message Filter dropping message ... queue is full` o errores de extrapolación de TF.
- **Causa:** sin internet ni RTC la Pi arranca siempre en **2025**, y el Create 3 tiene su propio reloj (`/odom` y `/battery_state` marcan `1753...`). Si Pi y Create 3 no coinciden, SLAM descarta todos los escaneos. El desfase con la laptop da igual; lo que importa es **Pi = Create 3**.
- **Trampa:** un salto pequeño en la Pi deja al Create 3 atrasado unos minutos y es peor que no tocar nada. Solo se corrige un desfase enorme (> 1 h). El Create 3 copia la hora de la Pi con retraso: esperar o reiniciar su app.
- **Mitigaciones en el repo:**
  - `source tb4_connect.sh --hora`: copia la hora de la laptop a la Pi por SSH; avisa si el desfase pasa de 2 s (`TB4_NOSYNC=1` lo omite).
  - `instalar_hora_persistente.sh` (en la Pi): guarda la hora cada 5 min y la restaura al arrancar.
  - `configurar_sync_automatico.sh` (en la laptop, una vez): clave SSH + sudo sin clave solo para `date -u -s` y reiniciar `turtlebot4.service`.
  - `chequeo_reloj_tb4.py [--esperar]`: mide el desfase `/scan` vs `/odom`.

## 3. SLAM y navegación

- **La Pi se satura:** SLAM + Nav2 juntos en la RPi4 dan `queue is full`. Usar solo SLAM (`slam_toolbox` asíncrono) salvo que haga falta el click-para-navegar; Nav2 aparte con `levantar_mapa_robot.sh nav2`.
- **El SSH se corta y mata SLAM:** lanzarlo dentro de `tmux`.
- **Qué falta si `/scan` u `/odom` no existen:** reiniciar `turtlebot4.service`.
- **Diagnóstico de "por qué no sigue la ruta":** `diagnostico_tb4.py` (laptop) mide frecuencias, retrasos de TF, `/cmd_vel` pedido vs real y estado de Nav2; `diag_robot.sh` (robot) registra CPU, temperatura, *throttling* y estado del Create 3.
- **Visor del mapa:** `ver_mapa_tb4.py` dibuja `/map`, el robot y un plan Dijkstra local al hacer click; con `--goto` manda la meta a Nav2.
- **Qué NO está probado:** `ver_mapa_tb4.py` se probó contra un robot simulado, no contra SLAM/Nav2 reales; `levantar_mapa_robot.sh` solo se verificó sintácticamente.
- Guías: `docs/guia_reset_conexion.md`, `docs/guia_occupancy_grid.md`, `docs/guia_completa_occupancy_grid.md`, `docs/guia_subir_script_robot.md`.

## 4. Mando Bluetooth (PS4/PS5) en la Pi

- **Síntoma:** `Pairing successful` pero `br-connection-unknown`; el mando conecta y se cae.
- **Causa:** se emparejó con el escaneo activo y mucho ruido BT; la consulta SDP expiró y BlueZ guardó una ficha HID inválida (`Could not parse HID SDP record`).
- **Arreglo:** `bluetoothctl remove <MAC>`, borrar la caché en `/var/lib/bluetooth/.../cache/`, poner `ClassicBondedOnly=false` en `/etc/bluetooth/input.conf`, reiniciar `bluetooth`, y emparejar con **`scan off` antes de `pair`**.
- `joy` corre en la Pi (no en WSL, donde `usbipd`/`joydev` es un lío) y sigue funcionando si cae el Wi-Fi.
- La MAC del adaptador de la Pi no se usa en `pair`. Un mando se empareja con un solo dispositivo a la vez.
- Detalle y pendientes en `mando.md`.

## 5. Visión (`seguimiento_vision/`)

- Control por gestos de mano y modo "sígueme" con esquiva de obstáculos; ver su `README.md`.
- Los modelos pesan: `*.onnx` (36 MB) y `*.task` (6–8 MB) **no se versionan**; se descargan con los `wget` del README de esa carpeta.
- El README dice que `hand_landmarker.task` "ya está en la carpeta": solo es cierto en la copia local, no en el repo.

## 6. Flujo de trabajo

- **WSL tiene una copia separada del repo.** Los scripts se ejecutan desde `~/turtleclaude4` (WSL), no desde `C:\Users\mando\turtleclaude4`. Tras editar en Windows hay que copiar: `cp /mnt/c/Users/mando/turtleclaude4/<ruta> ~/turtleclaude4/<ruta>`. Ya falló `sigueme.py` por tener el modelo solo en la copia de Windows.
- Antes de lanzar SLAM/Nav: verificar la hora y la red (secciones 1 y 2). Explica la mayoría de los fallos.

## 7. Objetivos abiertos

1. Robot y PC conectados, con hora coherente.
2. SLAM + Nav en el robot con un solo script.
3. SLAM + Nav en la PC (idealmente Docker) recibiendo cámara (H.264/H.265) y LiDAR del robot.
4. GUI en Python con `/map` y `/cmd_vel` (en Windows, mejor `rosbridge`/`foxglove_bridge` que `rclpy`).
5. Dejar el teleop del mando como servicio systemd junto a `turtlebot4.service`.
