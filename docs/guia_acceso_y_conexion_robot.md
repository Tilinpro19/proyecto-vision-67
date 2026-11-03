# Guía Definitiva de Acceso y Conexión al TurtleBot 4 Lite 🤖🔌

Esta guía proporciona a todos los integrantes del equipo las instrucciones y parámetros exactos para conectarse, operar y enviar comandos al TurtleBot 4 (Robot ID: **67**) tanto por cable Ethernet como por Wi-Fi.

---

## 📌 Tabla de Credenciales y Parámetros del Robot

| Parámetro | Valor Oficial | Notas |
|---|---|---|
| **Hostname** | `turtlebot4` | Identificador de red |
| **Usuario SSH** | `ubuntu` | Clave por defecto de fábrica: `turtlebot4` |
| **ROS Distro** | **ROS 2 Jazzy Jalisco** | Ubuntu 24.04 LTS |
| **`ROS_DOMAIN_ID`** | **`67`** | **Obligatorio** en todas las terminales |
| **RMW Implementation** | `rmw_fastrtps_cpp` | Middleware Fast DDS |
| **IP Ethernet (Cable)** | **`192.168.185.3`** | Configuración estática recomendada |
| **IP Wi-Fi (Lab 3)** | `192.168.0.104` (DHCP) | Red `Lab_Computech_3_5G` |
| **Base iRobot Create 3** | `192.168.186.2` | Subred interna USB (`usb0`) |

---

## 🔌 Vía 1: Conexión por Cable Ethernet Directo (Recomendada)

Es el método más confiable para el laboratorio y la competencia: elimina interferencias, no satura la red y tiene latencia inferior a $1\text{ ms}$.

### 1.1 Configurar la IP en tu Laptop (Windows)
1. Conecta el cable de red entre tu laptop y el puerto Ethernet de la Raspberry Pi 4 del robot.
2. Abre la configuración de red de Windows:
   - Presiona `Win + R`, escribe `ncpa.cpl` y pulsa Enter.
   - Haz clic derecho en tu adaptador **Ethernet** $\to$ **Propiedades**.
   - Doble clic en **Protocolo de Internet versión 4 (TCP/IPv4)**.
   - Selecciona **"Usar la siguiente dirección IP"**:
     - **Dirección IP:** `192.168.185.10`
     - **Máscara de subred:** `255.255.255.0`
     - **Puerta de enlace predeterminada:** *(dejar en blanco)*
   - Acepta y cierra.

### 1.2 Probar Conexión desde WSL2
Abre tu terminal de WSL2 (Ubuntu 24.04) y ejecuta:
```bash
ping -c 3 192.168.185.3
```
*Si responde:* Conexión física y de red perfecta.

### 1.3 Entrar por SSH a la Raspberry Pi
```bash
ssh ubuntu@192.168.185.3
```
*(Si pide clave por primera vez y no tienes par de llaves: `turtlebot4`)*.

---

## 📶 Vía 2: Conexión por Wi-Fi en el Laboratorio

Si deseas operar sin cables durante pruebas de recorrido libre:

1. Conecta tu laptop a la red Wi-Fi del laboratorio:
   - **SSID:** `Lab_Computech_3_5G`
   - **Contraseña:** *(Dada por el docente en pizarra)*
2. Descubre la IP asignada al robot mediante escaneo rápido:
   ```bash
   ping -c 2 192.168.0.104
   ```
   *(Si el router le asignó otra IP, revisa con `arp -a | grep d8:3a:dd:7e:5c:9c`)*.
3. Conéctate por SSH:
   ```bash
   ssh ubuntu@192.168.0.104
   ```

> [!WARNING]
> Las redes universitarias abiertas (como Eduroam o UTEC Alumnos) activan **aislamiento de clientes** (*Client Isolation*), bloqueando el tráfico multicast de ROS 2. Nunca intentes enlazar el robot en la red Wi-Fi institucional abierta; usa siempre `Lab_Computech` o el cable Ethernet directo.

---

## ⏰ Sincronización del Reloj (Paso Crítico Obligatorio)

En redes sin internet, el reloj interno del Create 3 puede retrasarse más de 20 horas, haciendo que ROS 2 ignore todos los tópicos por timestamps caducados.

### Comando Definitivo de Sincronización (Ejecutar desde WSL):
```bash
# Sincroniza la hora de tu laptop a la Pi y refresca el demonio NTP interno del Create 3
now=$(date +%s)
ssh ubuntu@192.168.185.3 "sudo date -u -s @$now && curl -s -X POST http://192.168.186.2/api/restart-ntpd"
```
*Verificación:* El desfase debe bajar a menos de **0.1 segundos**.

---

## 🛠️ Diagnóstico Rápido de Tópicos (Checklist)

Para verificar que todos los sensores del robot se están recibiendo en tu máquina:

```bash
# En WSL:
export ROS_DOMAIN_ID=67
source /opt/ros/jazzy/setup.bash

# Ver lista de tópicos activos:
ros2 topic list
```

Deberías ver:
- `/scan` (LiDAR RPLIDAR A1)
- `/odom` (Odometría de ruedas Create 3)
- `/hazard_detection` (Bumpers de choque)
- `/oakd/rgb/preview/image_raw/compressed` (Cámara de visión)
- `/cmd_vel` (Comandos de movimiento)

### Ver frecuencia en vivo:
```bash
ros2 topic hz /scan
# Esperado: ~7.5 a 8.0 Hz
```
