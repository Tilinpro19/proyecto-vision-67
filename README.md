# Proyecto Visión 67 🤖👁️ · TurtleBot 4 Lite
## Sistema Autónomo de Navegación Reactiva, Visión Facial y Control Gestual (ROS 2 Jazzy)

Repositorio oficial del proyecto de robótica móvil autónoma y visión computacional para la competencia final en **TurtleBot 4 Lite**.

- **Plataforma:** TurtleBot 4 Lite (Base iRobot Create 3 + RPLIDAR A1 + Cámara OAK-D Lite + Raspberry Pi 4 Model B de 4 GB)
- **Sistema Operativo & Middleware:** Ubuntu 24.04 LTS, ROS 2 Jazzy Jalisco, Fast DDS (perfil unicast/estático)
- **Fecha de Competencia Final:** 22 de Noviembre de 2026
- **Resultados de Validación:** 62/62 pruebas unitarias aprobadas (100% pass) · Validación cinemática en simulación y despliegue a bordo (*edge execution*) listo en el robot.

---

## 👥 Equipo de Proyecto
- **Líder / Integrante:** Mando ([@Tilinpro19](https://github.com/Tilinpro19))
- **Colaboradores:** Invitaciones de acceso al repositorio en [Settings / Access](https://github.com/Tilinpro19/proyecto-vision-67/settings/access).

---

## 📍 1. Dónde Quedó el Proyecto (Punto de Partida para el Equipo)

Todo el trabajo algorítmico, arquitectónico y de despliegue ya fue implementado y verificado. Aquí está el estado exacto de cada módulo para que el equipo continúe sin fricción:

| Componente | Estado Actual | Dónde Probarlo | Qué Falta para el Día Final |
|---|---|---|---|
| **Evasión Reactiva (`nav_core.py` + `navegacion.py`)** | ✅ 100% implementado (VFH-lite, cinemática suave, `SpatialKDTree2D`, `BreadcrumbGraph` con poda de callejones y retorno ICP 2D). | `pytest 1_competencia/tests` y `robot_simulado.py`. | Realizar ensayo físico en la pista del lab con obstáculos reales. |
| **Coordinador de Misión (`mision.py`)** | ✅ Máquina de estados completa (`ESPERA → IR_A_B → ESCANEAR → VOLVER_A_A → ANUNCIAR → FIN`), cronómetro y métricas de puntaje. | `tests/test_mision.py` y `tarea_a.sh`. | Validar detección de llegada al punto $B$ según criterio del docente. |
| **Reconocimiento Facial (`facial.py`)** | ✅ YuNet (detección) + SFace (embeddings) en ONNX offline con filtro de nitidez por Laplaciano y anuncio por voz. | `python3 facial.py --probar` | Cargar las fotos de las personas en `galeria/` cuando se revelen. |
| **Control Gestual Tarea B (`gestos_teleop.py`)** | ✅ Clasificador de manos MediaPipe con evaluación angular por articulaciones MCP, armado/parado por gestos sin teclado. | `python3 gestos_teleop.py --laptop` | Ensayar con la persona que hará los gestos frente a la cámara. |
| **Sincronización Create 3 ↔ Pi** | ✅ Resuelto el desfase de 20.9 horas mediante sincronización directa HTTP API (`POST /api/restart-ntpd`) y scripts persistentes. | `scripts/chequeo_reloj_tb4.py` | Ejecutar al encender el robot en cada sesión de laboratorio. |
| **Despliegue On-Board (*Edge Execution*)** | ✅ Scripts y lanzadores a bordo (`tarea_a_onboard.sh`) instalados en la Pi; elimina la latencia de red y es inmune a caídas de Wi-Fi. | `desplegar_en_robot.sh` | Usar preferentemente en los intentos puntuados de la competencia. |
| **Dockerización Completa** | ✅ Entorno listo con Dockerfile y docker-compose en [`tb4_docker/`](tb4_docker/); monta el repo y no requiere instalar ROS 2 en la PC. | `docker compose run --rm tb4` | Listo para usar si alguien del equipo no tiene ROS 2 configurado. |

---

## 🧪 2. Guía Paso a Paso para Probar el Algoritmo de Autonomía

Cualquier integrante puede reproducir y verificar todo el proyecto siguiendo estos niveles:

### Nivel 0: Preparar el Entorno en tu Laptop (WSL 2)
1. Clona este repositorio en tu terminal Ubuntu de WSL 2:
   ```bash
   git clone https://github.com/Tilinpro19/proyecto-vision-67.git ~/proyecto-vision-67
   cd ~/proyecto-vision-67
   chmod +x *.sh 1_competencia/*.sh 2_laboratorio/*.sh
   ```
2. Instala dependencias (con internet, antes de ir al laboratorio):
   ```bash
   sudo apt update && sudo apt install -y python3-pip python3-opencv python3-numpy python3-pytest
   pip install mediapipe --break-system-packages
   ```

---

### Opción Alternativa: Entorno Listo con Docker 🐳 (Sin instalar ROS 2 en tu PC)
Si prefieres no instalar ROS 2 Jazzy ni paquetes en tu sistema, usa el contenedor en [`tb4_docker/`](tb4_docker/):

1. **Construir la imagen:**
   ```bash
   cd ~/proyecto-vision-67/tb4_docker
   docker compose build
   ```
2. **Entrar al contenedor con el proyecto montado en `/ws`:**
   ```bash
   # Para simulación / tests sin robot:
   docker compose run --rm tb4

   # Para conectarse al robot real en el lab:
   TB4_IP=192.168.185.3 ROS_DOMAIN_ID=67 docker compose run --rm tb4
   ```
3. **Dentro del contenedor puedes ejecutar directamente:**
   ```bash
   # Pruebas unitarias:
   python3 -m pytest 1_competencia/tests -v

   # Misión Tarea A:
   cd 1_competencia && ./tarea_a.sh --goal 3.0 0.5
   ```

---

### Nivel 1: Verificación de Pruebas Unitarias (Sin Robot)
Ejecuta la suite completa de 62 pruebas unitarias automatizadas:
```bash
cd ~/proyecto-vision-67
python3 -m pytest 1_competencia/tests -v
```
> **Resultado esperado:** `62 passed in ~10s`. Valida el inflado geométrico de obstáculos, cinemática continua, búsqueda en KD-Tree, poda de ramas en callejones, reconocimiento facial y corrección ICP.

---

### Nivel 2: Simulación 2D Cinemática en Pantalla (Sin Robot)
Prueba la navegación autónoma $A \to B \to A$ en un entorno virtual con obstáculos:

- **Terminal 1 (Lanzar el simulador de Create 3 + RPLIDAR):**
  ```bash
  cd ~/proyecto-vision-67/2_laboratorio
  python3 robot_simulado.py --circuito laberinto
  ```
- **Terminal 2 (Lanzar el algoritmo autónomo de la Tarea A):**
  ```bash
  cd ~/proyecto-vision-67/1_competencia
  ./tarea_a.sh --goal 3.0 0.5 --ver
  ```
> **Qué observar:** Se abrirá una ventana gráfica mostrando el robot navegando hacia $(3.0, 0.5)$, esquivando las cajas con suavidad, deteniéndose en $B$ para escanear y retornando a $(0, 0)$ por el camino libre podando callejones cóncavos.

---

### Nivel 3: Pruebas en el Laboratorio con el Robot Físico

#### 3.1 Conexión Inicial y Red
- **Opción A (Cable Ethernet - Recomendada):**
  1. Conecta el cable a tu laptop.
  2. Asigna a tu interfaz Ethernet la IP estática `192.168.185.10` con máscara `255.255.255.0` (sin puerta de enlace).
  3. IP del robot: `192.168.185.3`. Clave SSH: `turtlebot4`.
- **Opción B (Wi-Fi):**
  1. Conéctate a la red del laboratorio **Lab_Computech** (la red UTEC bloquea el descubrimiento DDS).
  2. IP del robot: `192.168.0.104`.

#### 3.2 Sincronización del Reloj (OBLIGATORIO)
En el laboratorio **no hay internet**, por lo que la Pi arranca en el año 2025. Sincroniza desde tu laptop:
```bash
cd ~/proyecto-vision-67
source tb4_connect.sh 192.168.185.3 67
# O fuerza la sincronización inmediata:
source tb4_connect.sh --hora
```
Verifica que los relojes de la Pi y el Create 3 estén alineados (<0.2 s):
```bash
python3 scripts/chequeo_reloj_tb4.py
./tb4_check.sh 192.168.185.3
```

#### 3.3 Carga de la Galería de Rostros
El día de la prueba, cuando entreguen la foto de la persona a identificar:
```bash
cd ~/proyecto-vision-67/1_competencia
# Captura 10 fotos con la cámara del robot apuntando a la fotografía:
./capturar_persona.sh Juan_Perez 10
```

#### 3.4 Ejecutar la Tarea A (Navegación Autónoma)

**Modalidad 1: A Bordo del Robot (*Edge Execution* - elimina la latencia de red, inmune a desconexiones de Wi-Fi):**
```bash
ssh ubuntu@192.168.185.3 "cd ~/turtleclaude4/1_competencia && ./tarea_a_onboard.sh --goal 3.0 0.5"
```

**Modalidad 2: Desde la Laptop (con ventana OpenCV en vivo):**
```bash
cd ~/proyecto-vision-67/1_competencia
./tarea_a.sh --goal 3.0 0.5 --ver
```

#### 3.5 Ejecutar la Tarea B (Control por Gestos)
```bash
cd ~/proyecto-vision-67/1_competencia
./tarea_b.sh --goal 3.0 0.5
```
- **Gestos reconocidos:**
  - ✋ **Palma abierta (1 s):** Arma el sistema de control.
  - ✊ **Puño cerrado:** Para inmediatamente y desarma por seguridad.
  - 👍 **Pulgar arriba:** Avanza hacia adelante.
  - 👎 **Pulgar abajo:** Retrocede con suavidad y guardián LiDAR activo.
  - 👉 / 👈 **Pistola:** Gira en el lugar en la dirección indicada.
  - ✌️ **Victoria (1.5 s):** Dispara el escaneo facial en $B$ o el anuncio al regresar a $A$.

#### 3.6 Parada de Emergencia
Si en cualquier momento necesitas abortar:
```bash
./parar.sh
```
O toma el control manual con teclado:
```bash
python3 2_laboratorio/teleop_wasd.py
```

---

## 🧠 3. Cómo Funciona el Algoritmo de Autonomía

El sistema implementa una arquitectura híbrida: **reactiva de alta frecuencia (20 Hz) para la ida ($A \to B$)** y **topológica con corrección de odometría por ICP 2D para la vuelta ($B \to A$)**, desacoplada de costmaps pesados:

```
                          ┌───────────────────────────┐
                          │   RPLIDAR A1 (720 rayos)  │
                          └─────────────┬─────────────┘
                                        │ /scan (filtrado y rotado +90°)
                                        ▼
                          ┌───────────────────────────┐
                          │    Inflado Geométrico     │
                          │   R_inf = 0.17 + 0.09 m   │
                          └─────────────┬─────────────┘
                                        │
             ┌──────────────────────────┴──────────────────────────┐
             ▼                                                     ▼
┌───────────────────────────┐                        ┌───────────────────────────┐
│ free_distance(θ, R_inf)   │                        │     side_margin(θ)        │
│ Distancia libre frontal   │                        │ Holgura lateral a paredes │
└────────────┬──────────────┘                        └─────────────┬─────────────┘
             │                                                     │
             └──────────────────────────┬──────────────────────────┘
                                        ▼
                          ┌───────────────────────────┐
                          │    Evaluación de Costo    │
                          │   J(θ) sobre 73 conos     │
                          └─────────────┬─────────────┘
                                        │ Rumbo óptimo θ*
                                        ▼
                          ┌───────────────────────────┐
                          │ Smooth Blending y Rampas  │
                          │ Acel: +0.4 / Frenado: -0.8│
                          └─────────────┬─────────────┘
                                        │ /cmd_vel @ 20 Hz
                                        ▼
                          ┌───────────────────────────┐
                          │   Base iRobot Create 3    │
                          └───────────────────────────┘
```

### 3.1. Inflado Geométrico del Robot (`free_distance`)
Para no tratar al robot como un punto adimensional, cada rayo láser $(r_i, \alpha_i)$ se proyecta en el sistema local de cada rumbo candidato $\theta$:
$$fwd_i = r_i \cos(\alpha_i - \theta), \quad lat_i = r_i \sin(\alpha_i - \theta)$$
Considerando el radio real del robot ($17\text{ cm}$) más un margen de seguridad ($9\text{ cm}$), el radio efectivo es $R_{inf} = 0.26\text{ m}$. Si $|lat_i| < R_{inf}$ y $fwd_i > 0$, el contacto ocurre cuando el borde del chasis toca el obstáculo:
$$d_{contacto} = fwd_i - \sqrt{R_{inf}^2 - lat_i^2}$$

### 3.2. Selección de Rumbo Óptimo (`choose_heading`)
El espacio se discretiza en **73 rumbos candidatos** ($[-180^\circ, +180^\circ]$ cada $5^\circ$). Se descartan los que presenten $d_{contacto} < 0.45\text{ m}$. Para los rumbos admisibles, se evalúa:
$$J(\theta) = w_{meta} \cdot |\Delta\theta_{meta}| + w_{giro} \cdot |\Delta\theta_{prev}| - w_{holgura} \cdot \text{side\_margin}(\theta)$$
- $w_{meta} = 1.0$: Fuerza de atracción gravitacional hacia el objetivo $B$.
- $w_{giro} = 0.4$: Inercia cinemática que elimina el cabeceo lateral (*chattering*).
- $w_{holgura} = 5.0$: Penalización severa para mantener al robot centrado en pasillos y lejos de paredes.

### 3.3. Cinemática Continua (*Smooth Blending* y Rampa Trapezoidal)
Se eliminaron las detenciones bruscas del Create 3 mediante dos innovaciones:
1. **Atenuación continua en curvas:**
   $$\text{turn\_scale} = \max\left(0.0, \frac{\cos(\theta) - \cos(60^\circ)}{1.0 - \cos(60^\circ)}\right)$$
2. **Rampa trapezoidal:** Modula la aceleración lineal ($+0.4\text{ m/s}^2$), desaceleración ante obstáculos ($-0.8\text{ m/s}^2$) y aceleración angular ($\pm 2.5\text{ rad/s}^2$).

### 3.4. Detección Preventiva de Trampas Cóncavas (`detect_concavity`)
Si el robot entra en una trampa en "U" o callejón cóncavo frontal ($\pm 40^\circ$ cerrado a $<0.75\text{ m}$), el detector clasifica la geometría y redirige anticipadamente el rumbo hacia el ángulo periférico más abierto ($>60^\circ$), impidiendo que el robot quede atrapado en el fondo.

### 3.5. Retorno por Migas Podadas (`BreadcrumbGraph`) y Corrección ICP 2D
- **Ida ($A \to B$):** Registra coordenadas relativas cada 20 cm en un grafo topológico. Si el robot retrocedió para salir de un obstáculo, `prune_branch` poda automáticamente ese callejón de la memoria.
- **Vuelta ($B \to A$):** El robot invierte la lista de migas y utiliza `pick_waypoint`: si existe línea de visión despejada hacia una miga más lejana, recorta camino en línea recta (*shortcut*).
- **Corrección de Odometría (ICP 2D):** Cada 80 cm se almacena un *keyframe* del escaneo láser. En el regreso, el algoritmo ICP (*Iterative Closest Point*) alinea la nube de puntos actual contra el *keyframe* histórico:
  $$E(R, t) = \sum_{j} \| q_j - (R p_j + t) \|^2$$
  Si la coincidencia supera el 60% de *inliers*, la corrección espacial $(\Delta x, \Delta y, \Delta\theta)$ se aplica a la odometría, eliminando cualquier deslizamiento de las ruedas acumulado en la carrera.

### 3.6. Máquina de Estados de Misión (`mision.py`)
1. `ESPERA`: Robot en $A=(0,0)$ listo para arrancar.
2. `IR_A_B`: Navegación reactiva hacia la meta esquivando obstáculos y sembrando migas.
3. `ESCANEAR`: Barrido angular progresivo frente al cartel de $B$, captura de frames con OAK-D Lite, filtrado por varianza Laplaciana y reconocimiento con YuNet + SFace.
4. `VOLVER_A_A`: Retorno acelerado por las migas optimizadas con corrección ICP.
5. `ANUNCIAR`: Al llegar a $A$, sintetiza el nombre de la persona en voz alta y publica en `/nombre_reconocido`.
6. `FIN`: Frenado completo y cálculo de tiempo oficial y penalizaciones.

---

## ⚖️ 4. Análisis Crítico: VFH-lite vs. SLAM/Nav2 (Pros y Contras)

El análisis experimental detallado se encuentra en [`entregables/informe_arquitectura_navegacion.md`](entregables/informe_arquitectura_navegacion.md). Aquí se resumen las razones técnicas de la decisión arquitectónica:

### 4.1. Por qué colapsó el stack clásico (Nav2 + SLAM Toolbox) en el TurtleBot 4 Lite:
1. **Saturación Crítica de CPU en la Raspberry Pi 4:** Correr simultáneamente `slam_toolbox` (resolvedor de grafos no lineales Ceres), `global_costmap`, `local_costmap` y el controlador `MPPI` satura los 4 núcleos de la Raspberry Pi al **100% de CPU constante**.
2. **Estrangulamiento Térmico (*Thermal Throttling*):** La temperatura de la Pi 4 alcanzaba entre **75 °C y 85 °C**, obligando al procesador a bajar su frecuencia (*undervoltage / throttling*), perdiendo paquetes USB del LiDAR y de la cámara.
3. **El Problema del Reloj sin Internet:** El laboratorio no tiene conexión externa (sin NTP). Cuando el reloj de la Pi y el microcontrolador Create 3 diferían por apenas 1 segundo, la cola de transformación de coordenadas `tf2` colapsaba y SLAM descartaba el 100% de los escaneos láser (`queue is full`), congelando el robot.
4. **Bloqueos en Pasillos Estrechos:** Los *costmaps* inflados de Nav2 interpretaban los bordes del pasillo del laboratorio (~80 cm) como colisiones inminentes, activando bucles infinitos de *recovery behavior* (giros sobre el propio eje) que sumaban penalizaciones fatales de tiempo.
5. **Consumo de Memoria:** Más de **1.2 GB de RAM** en la Pi vs. **<60 MB** de nuestra solución reactiva.

### 4.2. Por qué VFH-lite + Migas + ICP 2D es la Solución Óptima:

| Criterio | VFH-lite + Migas + ICP 2D (Nuestra Solución) | Stack Nav2 + SLAM Toolbox Tradicional |
|---|---|---|
| **Carga de CPU en Raspberry Pi** | **~24% de 1 solo núcleo** (Pi descansada). | **100% en los 4 núcleos** (saturación total). |
| **Temperatura de la Pi** | **55.5 °C estable** (`throttled=0x0`). | **75–85 °C** (riesgo de cuelgue en caliente). |
| **Tiempo de cómputo por ciclo** | **< 1 ms** (operaciones vectorizadas en NumPy). | **40–120 ms** (frecuentes retrasos de lazo). |
| **Frecuencia de `/cmd_vel`** | **20 Hz constantes y estables**. | Fluctuante (cae a <5 Hz bajo carga). |
| **Reacción ante obstáculos** | **Instantánea** (conos analizados directamente del LiDAR). | Retardada (debe inflarse en el costmap y replanificar). |
| **Tolerancia a desfase de hora** | **Alta** (navega por odometría e incrementos relativos). | **Nula** (desfase mínimo descarta mensajes de TF). |
| **Estrategia de Retorno ($B \to A$)**| **Determinista:** Vuelve por el camino seguro recortando esquinas. | Indeterminista: Planifica sobre un mapa parcial incompleto. |

### 4.3. Limitaciones Reconocidas y Mitigaciones Implementadas:
- *Limitación:* Al ser reactivo local, un obstáculo cóncavo muy profundo (laberinto de callejón cerrado) podría entramparlo momentáneamente.
- *Mitigación:* Se implementó `detect_concavity` (detección temprana de cuencos en U) y memoria de obstáculos virtuales con deduplicación por `SpatialKDTree2D`, que bloquean el camino cóncavo y forzan el escape por la periferia.

---

## 📂 5. Estructura Completa del Repositorio

```text
proyecto-vision-67/
├── README.md                           # Portada, guía paso a paso, arquitectura y comparativa
├── HANDOFF_PROYECTO_FINAL.md           # Guía rápida de traspaso operacional
├── DESCUBRIMIENTOS.md                  # Bitácora de trampas de hardware, reloj y DDS
├── TROUBLESHOOTING.md                  # Solución rápida a fallos de conexión y sensores
├── Makefile                            # Atajos para WSL/Ubuntu (test, sim, check, deploy, stop)
├── requirements.txt                    # Dependencias Python de desarrollo
├── desplegar_en_robot.sh               # Script maestro para sincronizar código a la Pi por rsync
├── tb4_connect.sh                      # Configura FastDDS, dominio y sincroniza hora (--hora)
├── tb4_check.sh                        # Diagnóstico en vivo de tópicos, frecuencias y reloj
├── fastdds_pc.xml                      # Perfil Fast DDS de 100 participantes (evita tópicos ocultos)
├── parar.sh                            # Freno de emergencia inmediato
│
├── scripts/                            # Herramientas de diagnóstico y sincronización de reloj
│   ├── chequeo_reloj_tb4.py            # Compara marcas temporales /scan vs /odom
│   ├── diagnostico_tb4.py              # Diagnóstico integral de hardware y red desde la PC
│   ├── diag_robot.sh                   # Diagnóstico integral a bordo de la Raspberry Pi
│   ├── configurar_sync_automatico.sh   # Configura SSH + sudo sin clave para sincronizar hora
│   └── instalar_hora_persistente.sh    # Servicio en la Pi para restaurar hora tras apagón
│
├── 1_competencia/                      # Código fuente de las tareas evaluadas
│   ├── nav_core.py                     # Algoritmo matemático puro VFH-lite, KDTree, ICP y rampas
│   ├── navegacion.py                   # Nodo ROS 2 de navegación y evasión reactiva
│   ├── mision.py                       # Coordinador de misión A->B->A y cronómetro
│   ├── facial.py                       # Reconocimiento facial (YuNet + SFace) y sintetizador
│   ├── gestos_teleop.py                # Teleoperación por gestos de mano con MediaPipe (Tarea B)
│   ├── tarea_a.sh                      # Lanzador Tarea A desde laptop
│   ├── tarea_a_onboard.sh              # Lanzador Tarea A a bordo de la Pi (Edge)
│   ├── tarea_b.sh                      # Lanzador Tarea B (gestos)
│   ├── capturar_persona.sh             # Script rápido para cargar fotos a la galería
│   ├── config.json                     # Parámetros cinemáticos y de seguridad
│   ├── modelos/                        # Modelos offline (YuNet, SFace y hand_landmarker.task)
│   ├── galeria/                        # Base de datos de rostros conocidos
│   └── tests/                          # Suite de pruebas unitarias automatizadas
│
├── 2_laboratorio/                      # Herramientas de laboratorio y simulación
│   ├── robot_simulado.py               # Simulador 2D de Create 3 + LiDAR con interfaz gráfica
│   ├── calibrar.py                     # Optimización automática de parámetros (tiempo vs choque)
│   └── teleop_wasd.py                  # Control manual por teclado para pruebas
│
├── 3_referencia/                       # Scripts de referencia y prototipos
│   └── sigueme.py                      # Modo sígueme por LiDAR y cámara
│
├── tb4_docker/                         # ENTORNO DOCKER LISTO PARA USAR
│   ├── Dockerfile                      # Imagen con ROS 2 Jazzy, OpenCV, MediaPipe y tb4lib
│   ├── docker-compose.yml              # Configuración de red host, dispositivos y volumen /ws
│   └── tb4lib/                         # Librería Python de alto nivel para el Create 3
│
├── docs/                               # Documentación complementaria y manuales
│   ├── arquitectura_sistema.md         # Diagrama detallado de software
│   ├── metodos_y_justificacion.md      # Métodos implementados y justificación técnica
│   ├── guia_acceso_y_conexion_robot.md # Guía de redes y conexión física
│   ├── guia_conexion_turtlebot4.md     # Guía rápida de conexión y red del laboratorio
│   ├── guia_entorno_desarrollo_y_simulacion.md # Configuración de WSL y simulador
│   ├── lineamientos_competencia.md     # Reglas y fórmulas de evaluación
│   ├── mando.md                        # Emparejamiento de mando PS4/PS5 en la Pi
│   ├── resolucion_problemas_tecnicos.md # Detalle de soluciones a cuellos de botella
│   └── tarjeta_offline_lab.md          # Referencia rápida sin internet en el laboratorio
│
├── bitacora/                           # Bitácora semanal de avance
│   ├── semana-01.md                    # Semana 1: Estructura inicial
│   ├── semana-02.md                    # Semana 2: Sincronización de reloj, cinemática y edge
│   └── semana-03.md                    # Semana 3: Cero colisiones, misión completa y gestos
│
└── entregables/                        # Informes formales del curso
    ├── informe_arquitectura_navegacion.md  # Informe técnico comparativo vs SLAM
    └── informe_estado_proyecto_70.md       # Informe de avance al 70%
```
