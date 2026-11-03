# Manual de Operación y Laboratorio · TurtleBot 4

Guía de referencia rápida y estandarizada para operar, desplegar y competir con el TurtleBot 4 Lite en el laboratorio.

---

## 1. Conexión Física y Configuración de Red

Para evitar congestiones y bloqueos de descubrimiento DDS en las redes Wi-Fi del campus (p. ej. Eduroam/UTEC que filtran multicast), se utiliza **enlace Ethernet directo**:

| Dispositivo | Interfaz | Dirección IP | Máscara | Función |
|---|---|---|---|---|
| **Laptop de Desarrollo** | Ethernet (Cable) | `192.168.185.10` | `255.255.255.0` | Estación base / Monitoreo |
| **Raspberry Pi 4 (TB4)** | `eth0` | `192.168.185.3` | `255.255.255.0` | Computadora a bordo |
| **iRobot Create 3** | Red interna USB | `192.168.186.2` | `255.255.255.0` | Base móvil diferencial |

### Verificación de Enlace desde WSL
```bash
ping -c 3 192.168.185.3
```
*Latencia esperada:* $< 1\text{ ms}$.

---

## 2. Sincronización del Reloj (Create 3 ↔ Pi ↔ Laptop)

> [!CAUTION]
> En entornos sin conexión a internet, la Raspberry Pi y el Create 3 no pueden sincronizarse por NTP público. Si el desfase entre ellos supera los 2 segundos, ROS 2 descarta los mensajes de odometría y scans por timestamp incoherente.

### Solución con un Solo Comando (en WSL):
```bash
# Sincroniza la hora de la Laptop a la Pi y fuerza al Create 3 a refrescar su NTP interno
now=$(date +%s)
ssh ubuntu@192.168.185.3 "sudo date -u -s @$now && curl -s -X POST http://192.168.186.2/api/restart-ntpd"
```
*Resultado verificado:* Desfase reducido a **< 0.08 s**.

---

## 3. Despliegue de Código al Robot (*Edge Execution*)

Para no depender de la conexión de red durante la carrera de navegación de la competencia, el código se sincroniza y ejecuta directamente dentro de la Raspberry Pi:

### Comando de Sincronización Automática:
Desde la carpeta raíz del repositorio técnico (`~/turtleclaude4`):
```bash
./desplegar_en_robot.sh 192.168.185.3
```
Este script realiza automáticamente:
1. Prueba de ping y conexión SSH.
2. Sincronización `rsync` de `proyecto_final` y `tb4lib`.
3. Instalación de la librería `tb4lib` en Python de la Raspberry Pi.
4. Sincronización del reloj de la laptop con la Pi y el Create 3.
5. Creación de los ejecutables on-board (`tarea_a_onboard.sh` y `tarea_b_onboard.sh`).

---

## 4. Ejecución de las Tareas de la Competencia

### Tarea A · Navegación Autónoma ($A \to B \to A$)

#### Opción 1: Ejecución A Bordo del Robot (Recomendada para la Competencia)
Inmune a caídas de cable o interferencias de Wi-Fi:
```bash
ssh ubuntu@192.168.185.3 "cd ~/turtleclaude4/proyecto_final/1_competencia && ./tarea_a_onboard.sh --goal 3.0 0.0"
```

#### Opción 2: Ejecución desde la Laptop (Con Ventana Gráfica de Depuración)
Permite visualizar la cámara OAK-D Lite y la telemetría en tiempo real:
```bash
cd ~/turtleclaude4/proyecto_final/1_competencia
./tarea_a.sh --goal 3.0 0.0 --ver
```

*Parámetros ajustables al vuelo (sin editar código):*
- Modificar velocidad lineal: `--param max_linear=0.15`
- Aumentar margen de seguridad: `--param safety_margin=0.12`
- Modo simulado (sin mover motores): `--dry`

---

### Tarea B · Teleoperación por Gestos de Manos

Desde la laptop con la webcam apuntando al operador:
```bash
cd ~/turtleclaude4/proyecto_final/1_competencia
./tarea_b.sh --cam 0 --lineal 0.20
```
- **Armado:** Mostrar la palma abierta firme frente a la cámara por 2 segundos.
- **Movimiento:** Mover la mano para dirigir el TurtleBot hacia $B$.
- **Parada:** Cerrar el puño para detener el robot instantáneamente.
- **Escaneo:** Realizar el gesto de Victoria ("V") en la meta $B$ para activar el reconocimiento de la foto.

---

## 5. Preparación de la Galería de Caras

Cuando se revelen las fotografías de las personas el día de la competencia:
1. Crear una carpeta con el nombre de cada persona en `1_competencia/galeria/`:
   ```text
   galeria/
   ├── Juan_Perez/
   │   ├── foto1.jpg
   │   └── foto2.jpg
   └── Maria_Gomez/
       ├── foto1.jpg
       └── foto2.jpg
   ```
2. Si solo entregan una foto física impresa, capturarla directamente con la cámara del TurtleBot:
   ```bash
   python3 1_competencia/facial.py --capturar Juan_Perez --n 10
   ```
3. Validar la galería antes del intento:
   ```bash
   python3 1_competencia/facial.py --probar
   ```
