# Mando (PS4/PS5) por Bluetooth en el TurtleBot 4

Manejar el TurtleBot 4 con un mando "Wireless Controller" (Sony) emparejado **directo a la Raspberry Pi del robot**. El nodo `joy` corre en la Pi y publica en `/cmd_vel` sin cruzar DDS por Wi-Fi.

Entorno de referencia: TB4 Lite, Ubuntu 24.04 (aarch64), usuario `ubuntu`. Reemplaza `<IP_ROBOT>`, `<DOMAIN_ID>` y las direcciones MAC por las de tu robot y mando. Ver también `tb4_connect.sh`.

## Por qué en la Pi y no en el laptop

- En WSL el mando por USB es un lío (`usbipd`, el kernel de WSL2 suele no traer `joydev`).
- Con `joy` en la Pi, los comandos van directo a `/cmd_vel` y siguen funcionando aunque se caiga el Wi-Fi.
- La red del lab no tiene internet: instala paquetes **antes**, en otra red.

## Requisitos (instalar antes, con internet)

```bash
sudo apt install ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-teleop-twist-joy
```

Comprobar: `ros2 pkg list | grep -E "^joy$|teleop_twist_joy"`.

No hace falta `linux-modules-extra-raspi`: `hid_playstation`, `hid_sony` y `/dev/uhid` ya vienen en el kernel `6.8.0-1031-raspi`.

## Emparejar el mando (una sola vez)

1. Entra a la Pi: `ssh ubuntu@<IP_ROBOT>`.
2. Pon el mando en modo emparejamiento: `Share` + `PS` hasta que la luz parpadee rápido. Acércalo a la Pi.
3. Abre `bluetoothctl` y escribe **un comando por vez** (no pegues varias líneas de golpe):

```
scan on
```

Cuando salga `[NEW] Device <MAC> Wireless Controller`, apaga el escaneo **antes** de emparejar:

```
scan off
pair <MAC>
trust <MAC>
connect <MAC>
```

Donde `<MAC_MANDO>` es la dirección que aparece en pantalla para tu mando (p. ej. `XX:XX:XX:XX:XX:XX`). La MAC que aparece como controller o adaptador por defecto es el adaptador Bluetooth de la propia Pi; no la uses en `pair`.

4. Comprobar: `ls /dev/input/js0`.

Una vez emparejado y con `trust`, el mando se reconecta solo con un toque corto de `PS` (no `Share`+`PS`).

## Problema que tuvimos y su arreglo

**Síntoma:** `Pairing successful` pero `Failed to connect: org.bluez.Error.Failed br-connection-unknown`; el mando hacía `Connected: yes` y se caía enseguida.

**Log** (`journalctl -u bluetooth`):

```
error updating services: Connection timed out (110)
Could not parse HID SDP record: No such file or directory
```

**Causa:** se emparejó con el escaneo corriendo y mucho ruido Bluetooth alrededor; la consulta SDP se agotó y BlueZ guardó una ficha de servicios inválida, así que no sabía que era un dispositivo HID.

**Arreglo:** borrar el emparejamiento y la caché, aflojar `ClassicBondedOnly`, y volver a emparejar con `scan off` antes de `pair`:

```bash
bluetoothctl remove <MAC_MANDO>
sudo rm -rf /var/lib/bluetooth/<MAC_ADAPTADOR_PI>/cache/<MAC_MANDO>
sudo sed -i 's/^#\?ClassicBondedOnly=.*/ClassicBondedOnly=false/' /etc/bluetooth/input.conf
grep ClassicBondedOnly /etc/bluetooth/input.conf   # debe decir ClassicBondedOnly=false
sudo systemctl restart bluetooth
```

Luego repetir la sección "Emparejar el mando".

## Probar y lanzar el teleop

Todo en la Pi:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# 1. Ver que el mando manda datos (ejes y botones deben cambiar)
ros2 run joy joy_node            # terminal 1
ros2 topic echo /joy             # terminal 2

# 2. Ver el topic de velocidad del robot (puede tener namespace)
ros2 topic list | grep cmd_vel

# 3. Lanzar el teleop (cierra antes el joy_node del paso 1)
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3-holonomic'
```

Si el topic tiene namespace (p. ej. `/robot<ID>/cmd_vel`), añadir `--ros-args -r cmd_vel:=/robot<ID>/cmd_vel`.

- Mantener pulsado el botón de habilitación (`L1`) mientras se mueve el stick izquierdo.
- Robot en el suelo, despejado y **fuera del dock** para la primera prueba.
- Si va lento, subir `scale_linear` y `scale_angular` en el yaml de `teleop_twist_joy`. Límite del Create 3: 0.31 m/s.
- Si los ejes salen cambiados, mirar `ros2 topic echo /joy` y ajustar `axis_linear` y `axis_angular`.

## Notas del lab

- Varios robots en un mismo router: cada mando va vinculado con su robot y su `ROS_DOMAIN_ID` para no interferir en `/joy` ni `/cmd_vel`.
- Un mando solo se empareja con un dispositivo a la vez. Para cambiarlo de robot hay que `remove` en el anterior y volver a emparejar.
- Alternativa sin Bluetooth: cable USB de datos mandos→Pi; no pasa por SDP.
- Para ver al robot desde WSL: estar en Lab_Computech o por cable, no en Wi-Fi UTEC (DDS bloqueado). Tras cambiar cable↔Wi-Fi: `sudo systemctl restart turtlebot4.service`.

## Pendiente

- Confirmar que el robot se mueve con `teleop_twist_joy` y anotar aquí el `joy_config` y el topic de `cmd_vel` que funcionaron.
- Dejarlo como servicio systemd que arranque con `turtlebot4.service`.
