# TurtleBot 4 Lite: inventario (2 de noviembre de 2026)

Recolectado desde el Docker `tb4:jazzy` con `info/recolectar.sh`. Los volcados completos están en esta carpeta.

## Red
- IP `172.30.132.101` (hotspot del teléfono, SSID `Lab_Computech_3_5G`), MAC `d8:3a:dd:7e:5c:9c`
- `ROS_DOMAIN_ID=67`, Fast DDS; funciona en unicast (`ROS_STATIC_PEERS`)
- SSH: `ubuntu@` pide contraseña (no hay clave instalada)

## Estado
| | |
|---|---|
| Batería | 84 %, 15.62 V, descargando a 0.77 A, 39.6 °C, 1.72/2.04 Ah |
| Dock | no está acoplado ni ve el dock |
| Ruedas | habilitadas, sin corriente |
| Odometría acumulada | x = -3.43, y = 2.86 m (no se ha reseteado: `/reset_pose`) |
| Reloj | Pi ≈ PC; **Create 3 atrasado unos 405 940 s (4.7 días)** → SLAM/Nav2 descartarán scans |

## Sensores
| Topico | Tipo | Hz | Notas |
|---|---|---|---|
| `/scan` | LaserScan | 7.4 | RPLIDAR, 720 puntos, 0.15–12 m, `rplidar_link`, 115200 baudios |
| `/odom` | Odometry | 20 | Create 3 |
| `/imu` | Imu | 33 | Create 3 |
| `/oakd/rgb/preview/image_raw/compressed` | CompressedImage | 30 | 250×250, USB SUPER_PLUS, pipeline RGB, sin NN ni profundidad |
| `/oakd/imu/data` | Imu | – | IMU de la OAK-D |
| `/hazard_detection`, `/ir_intensity`, `/mouse` | Create 3 | – | bumper, IR, sensor óptico |
| `/battery_state`, `/dock_status`, `/wheel_status`, `/interface_buttons` | | – | |
| `/joy` | Joy | – | `joy_linux_node` + `teleop_twist_joy_node` activos (mando PS) |

## Actuadores
- `/cmd_vel`: **TwistStamped** (también existe `/cmd_vel_unstamped` como Twist)
- `/cmd_audio` (notas), `/cmd_lightring` (LEDs)
- Acciones del Create 3: `drive_distance`, `rotate_angle`, `drive_arc`, `navigate_to_position`, `dock`, `undock`, `wall_follow`, `led_animation`, `audio_note_sequence`
- Servicios: `/e_stop`, `/robot_power`, `/reset_pose`, `/start_motor` y `/stop_motor` (LiDAR), `/oakd/start_camera` y `/oakd/stop_camera`

## Nodos
`turtlebot4_node` (model=lite, power_saver=true), `create3_repub` (namespace `_do_not_use`), `rplidar_composition`,
`oakd` / `oakd_container`, `robot_state_publisher`, `joint_state_publisher`, `joy_linux_node`, `teleop_twist_joy_node`,
`turtlebot4_diagnostics`, `analyzers`.

## Botones configurados
- Create 3 botón 1: Dock / Wall Follow Left (mantener 2 s)
- Create 3 botón 2: Undock / Wall Follow Right
- Power: EStop / Power (3 s)
- Mando: B = EStop, Home = Dock/Undock, L2/R2 = Wall Follow
