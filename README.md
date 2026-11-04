# Proyecto Visión 67 🤖👁️ · TurtleBot 4 Lite

Repositorio oficial de documentación, arquitectura, guías de operación y entregables del proyecto final de robótica móvil autónoma y visión computacional.

- **Plataforma:** TurtleBot 4 Lite (Base iRobot Create 3 + RPLIDAR A1 + Cámara OAK-D Lite + Raspberry Pi 4)
- **Framework:** ROS 2 Jazzy Jalisco (Ubuntu 24.04 LTS)
- **Repositorio Técnico de Código:** [turtlebot67](https://github.com/Tilinpro19/turtlebot67)
- **Fecha de Competencia Final:** 22 de Noviembre de 2026

---

## 👥 Equipo de Proyecto
- **Líder / Integrante:** Mando ([@Tilinpro19](https://github.com/Tilinpro19))
- **Colaboradores:** *(Invitaciones y acceso para el equipo en [Settings / Access](https://github.com/Tilinpro19/proyecto-vision-67/settings/access))*

---

## 🚀 Guía de Inicio Rápido para Compañeros de Equipo

Si te estás uniendo al proyecto o vas a operar el robot en el laboratorio, aquí tienes los accesos directos principales:

### 1. Parámetros de Conexión Inmediata
- **Conexión por Cable Ethernet:** Conecta el cable a tu laptop, fija tu IP en `192.168.185.10` y accede al robot en:
  ```bash
  ssh ubuntu@192.168.185.3
  # Clave por defecto de fábrica: turtlebot4
  ```
- **Variable de ROS 2:** `export ROS_DOMAIN_ID=67` (obligatorio en cualquier terminal).
- **Sincronización del Reloj (Create 3 ↔ Pi):**
  ```bash
  ssh ubuntu@192.168.185.3 "curl -s -X POST http://192.168.186.2/api/restart-ntpd"
  ```

### 2. Probar Todo en tu Laptop sin Robot (Simulador 2D)
No necesitas tener el TurtleBot físico para avanzar o probar código:
```bash
# Terminal 1: Simulador cinemático del robot con laberinto virtual
python3 2_laboratorio/robot_simulado.py --circuito laberinto

# Terminal 2: Misión autónoma de evasión en tiempo real
./1_competencia/tarea_a.sh --goal 3.0 1.0 --ver
```

### 3. Ejecutar la Misión de la Competencia en el Robot
- **A bordo del robot (Recomendado, inmune a caídas de red):**
  ```bash
  ssh ubuntu@192.168.185.3 "cd ~/turtleclaude4/proyecto_final/1_competencia && ./tarea_a_onboard.sh --goal 3.0 0.0"
  ```
- **Desde la laptop (con ventana de visualización en vivo):**
  ```bash
  cd ~/turtleclaude4/proyecto_final/1_competencia && ./tarea_a.sh --goal 3.0 0.0 --ver
  ```

---

## 📚 Índice Maestro de Guías y Documentación

Toda la información del proyecto está organizada modularmente dentro de [`docs/`](docs/) y [`entregables/`](entregables/):

| Documento | Descripción y Utilidad |
|---|---|
| 🔌 **[Guía de Acceso y Conexión](docs/guia_acceso_y_conexion_robot.md)** | Manual detallado de conexión por cable Ethernet, Wi-Fi del laboratorio, solución del reloj Create 3 y diagnóstico de tópicos. |
| 💻 **[Guía de Entorno y Simulación](docs/guia_entorno_desarrollo_y_simulacion.md)** | Cómo preparar tu laptop con WSL2, correr los 62 tests unitarios y simular la navegación, visión y gestos sin robot. |
| 📋 **[Lineamientos de la Competencia](docs/lineamientos_competencia.md)** | Reglas oficiales de la pista, tareas A y B, penalizaciones por choque y fórmula matemática de calificación. |
| 📐 **[Arquitectura Integral del Sistema](docs/arquitectura_sistema.md)** | Diagrama de flujo de `mision.py`, piloto reactivo VFH-lite, `SpatialKDTree2D`, `BreadcrumbGraph`, YuNet+SFace y MediaPipe. |
| 🛠️ **[Manual de Operación en Laboratorio](docs/manual_operacion_laboratorio.md)** | Protocolo paso a paso para el día de pruebas: captura de la galería de rostros, comandos de lanzamiento y parámetros. |
| 🩺 **[Registro de Problemas Resueltos](docs/resolucion_problemas_tecnicos.md)** | Diagnóstico a fondo del desfase de 20.9h en Create 3, por qué Nav2 saturaba la Pi al 100% y cómo se resolvió con Edge Execution. |
| 🚀 **[Consolidado Completo de Avances](docs/bitacora_completa_avances.md)** | Resumen histórico y evolución técnica de todos los hitos alcanzados por el equipo. |
| 📑 **[Informe Técnico Comparativo](entregables/informe_arquitectura_navegacion.md)** | Documento formal entregable: Comparativa técnica de Navegación Reactiva vs. SLAM/Nav2 tradicional. |

---

## 📂 Estructura del Repositorio

```text
proyecto-vision-67/
├── README.md               # Portada principal, guía rápida y enlaces maestros
├── 1_competencia/          # Código fuente oficial ROS 2 para la competencia (Tareas A y B)
│   ├── navegacion.py       # Piloto reactivo VFH-lite, inflado y rampa cinemática
│   ├── mision.py           # Máquina de estados A -> B -> Escaneo -> Retorno -> Anuncio
│   ├── facial.py           # Detección YuNet + SFace (ONNX) y timbre Create 3
│   ├── gestos_teleop.py    # Teleoperación MediaPipe con reversa y guardián LiDAR
│   ├── nav_core.py         # Algoritmos puros (KD-Tree, BreadcrumbGraph, rampa trapezoidal)
│   ├── config.json         # Parámetros conservadores anti-colisión
│   └── tests/              # Suite de 62 pruebas unitarias y simulaciones
├── 2_laboratorio/          # Scripts de prueba de sensores, simulador 2D y diagnóstico
├── 3_referencia/           # Documentos y algoritmos de referencia
├── tb4_docker/             # Entorno en contenedor y librería tb4lib
├── bitacora/               # Registro cronológico semanal
│   ├── plantilla_avance.md # Plantilla oficial para nuevas sesiones de trabajo
│   ├── semana-01.md        # Semana 1: Separación de repositorios y estructura base
│   ├── semana-02.md        # Semana 2: Sincronización de reloj, cinemática y edge deployment
│   └── semana-03.md        # Semana 3: Cero colisiones, misión completa, gestos y estado 70%
├── docs/                   # Guías técnicas, manuales y resolución de problemas
│   ├── guia_acceso_y_conexion_robot.md        # Guía para conectarse al robot
│   ├── guia_entorno_desarrollo_y_simulacion.md # Cómo trabajar en casa con simulación
│   ├── lineamientos_competencia.md            # Reglas y rúbrica oficial de puntaje
│   ├── arquitectura_sistema.md                # Diseño de software y algoritmos
│   ├── manual_operacion_laboratorio.md        # Procedimientos en laboratorio
│   ├── resolucion_problemas_tecnicos.md       # Diagnóstico y soluciones técnicas
│   └── bitacora_completa_avances.md           # Resumen acumulado de todos los hitos
├── entregables/            # Informes técnicos y entregas del curso
│   ├── README.md                          # Índice de entregables
│   ├── informe_arquitectura_navegacion.md  # Informe técnico comparativo
│   └── informe_estado_proyecto_70.md      # Informe de avance formal al 70%
└── evidencias/             # Registro de telemetría y pruebas
    └── README.md           # Frecuencias de sensores, salud de hardware y 62 tests
```

---

## 🏆 Estado General del Proyecto: 70% de Madurez

> [!NOTE]
> **Tope de Avance al 70%:** Todo el stack de software, modelos neuronales ONNX, guardián contra colisiones y suite de pruebas están integrados y operativos. No obstante, por rigor de ingeniería, **se califica en un 70%** hasta ejecutar las pruebas físicas en el piso real del circuito y capturar la galería presencial.

- **Navegación Autónoma (Tarea A - 75%):** VFH-lite con 73 direcciones, inflado de radio $R=26\text{ cm}$, rotación pura obligatoria si el frente está a $\le 32\text{ cm}$, desaceleración terminal a $40\text{ cm}$, retorno por migas y corrección ICP 2D.
- **Visión por Computador (75%):** YuNet + SFace en ONNX offline, barrido panorámico de búsqueda ($\pm 35^\circ$), confirmación por buzzer Do-Mi-Sol en Create 3 y anuncio verbal en laptop por SAPI.
- **Control por Gestos (Tarea B - 70%):** MediaPipe Hand Landmarker con reversa segura (👎) y guardián de proximidad LiDAR que bloquea avances o retrocesos si hay obstáculos a $< 32\text{ cm}$.
- **Pruebas Unitarias (100%):** 62 pruebas aprobadas y 1 omitida en suite pytest.
- **Despliegue a Bordo (*Edge Execution*):** Verificado en la Raspberry Pi 4 con 0 ms de latencia DDS interna y enlace Ethernet estable.
