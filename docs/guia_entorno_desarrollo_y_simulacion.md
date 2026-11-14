# Guía de Entorno de Desarrollo y Simulación sin Robot 💻🧪

Esta guía permite a cualquier integrante del equipo configurar su entorno de desarrollo local (Windows + WSL 2) y probar **el 100% de la lógica de navegación, visión y control gestual desde casa o el aula, sin necesidad de tener el TurtleBot 4 físico conectado**.

---

## 1. Requisitos y Configuración de WSL 2 (Windows 10 / 11)

### 1.1 Configurar WSL en Modo Espejo (*Mirrored Networking*)
Para que ROS 2 pueda descubrir dispositivos y enviar paquetes UDP multicast sin problemas de firewall virtual en Windows:

1. En Windows, abre el Bloc de Notas y guarda el siguiente archivo en `C:\Users\<TU_USUARIO>\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
2. Reinicia WSL en PowerShell como Administrador:
   ```powershell
   wsl --shutdown
   ```
3. Permite el tráfico de red de Hyper-V (PowerShell Administrador):
   ```powershell
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   ```

### 1.2 Instalar Dependencias de Python en WSL2
Abre tu terminal Ubuntu 24.04 en WSL y ejecuta:
```bash
sudo apt update && sudo apt install -y python3-pip python3-opencv python3-numpy python3-pytest
pip install mediapipe --break-system-packages
```

---

## 2. Clonación de los Repositorios

Recomendamos mantener los dos repositorios en tu carpeta de usuario en WSL o Windows:

```bash
# 1. Repositorio técnico y código del robot:
git clone https://github.com/Tilinpro19/turtlebot67.git ~/turtleclaude4

# 2. Repositorio de avances y documentación:
git clone https://github.com/Tilinpro19/proyecto-vision-67.git ~/proyecto-vision-67
```

---

## 3. Verificación Rápida: Pruebas Unitarias Automáticas

Antes de tocar el robot físico, toda la lógica algorítmica debe pasar las 62 pruebas unitarias.

Ejecuta desde la raíz del repositorio:
```bash
python3 -m pytest 1_competencia/tests -v
```

### Lo que se evalúa automáticamente:
- **Evasión de obstáculos:** Pasillo estrecho, esquinas ciegas, obstáculos centrados y descentrados.
- **Estructuras de datos:** Búsqueda en `SpatialKDTree2D`, poda de callejones en `BreadcrumbGraph`, detector de trampas cóncavas `detect_concavity`.
- **Visión facial:** Detección de rostros en imágenes con baja resolución, cambios de iluminación y cálculo de similitud con SFace.
- **Control de gestos:** Clasificación de posturas (palma, puño, victoria, likes/dislikes) y máquina de seguridad de MediaPipe.
- **Misiones completas simuladas:** Escenarios sintéticos $A \to B \to A$ con obstáculos bajos invisibles y corrección de deriva por ICP.

*Resultado esperado:* **`62 passed in ~7.5s`**.

---

## 4. Simulación 2D Cinemática en Pantalla

El repositorio incluye un simulador cinemático completo (`robot_simulado.py`) que emula en tiempo real los tópicos `/scan`, `/odom` y `/cmd_vel` de un TurtleBot real en un laberinto configurable.

### Paso 1: Levantar el Robot Simulado (Terminal 1)
```bash
cd 2_laboratorio
python3 robot_simulado.py --circuito laberinto
```
*(Se abrirá una ventana mostrando el mapa, el robot virtual y los rayos LiDAR trazados en tiempo real).*

### Paso 2: Lanzar la Misión Autónoma (Terminal 2)
```bash
cd 1_competencia
./tarea_a.sh --goal 3.0 1.0 --ver
```
Verás al TurtleBot virtual:
1. Avanzar hacia la meta esquivando paredes.
2. Bordear obstáculos y evitar trampas en "U".
3. Al alcanzar la meta, simular el escaneo y retornar a $(0, 0)$ por las migas de pan registradas.

---

## 5. Pruebas de Visión y Gestos con tu Webcam

### 5.1 Probar la Galería de Rostros
```bash
cd 1_competencia
python3 facial.py --probar
```
Valida que el modelo YuNet + SFace cargue correctamente los pesos `.onnx` y evalúa la galería de prueba.

### 5.2 Probar el Control Gestual con tu Mano
```bash
cd 1_competencia
python3 mano_teleop.py --probar
```
Abre tu webcam de la laptop:
- Muestra la **palma abierta** para armar el sistema.
- Mueve la mano para generar vectores de velocidad angular y lineal.
- Cierra el **puño** para detenerlo de inmediato.
