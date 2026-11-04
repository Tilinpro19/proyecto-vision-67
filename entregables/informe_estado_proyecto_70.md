# Informe de Estado del Proyecto: TurtleBot 4 Lite (70% de Avance)

**Fecha:** Noviembre 2026  
**Robot:** TurtleBot 4 Lite (iRobot Create 3 + Raspberry Pi 4 + RPLIDAR A1 + OAK-D Lite)  
**Entorno de Software:** ROS 2 Jazzy, Fast DDS (`ROS_DOMAIN_ID=67`), Python 3.12, OpenCV DNN, MediaPipe  
**Repositorio Remoto:** `https://github.com/Tilinpro19/turtlebot67.git` (Rama: `master`)  

---

## 1. Resumen Ejecutivo del Nivel de Madurez (70%)

El proyecto se clasifica estrictamente en un **70% de madurez**. Todo el pipeline de software, arquitectura matemática de navegación, detección/reconocimiento facial por redes neuronales y control teleoperado por visión de gestos está **desarrollado, integrado, probado unitariamente (62 tests aprobados) y desplegado en la Raspberry Pi 4**.  

**No se declara finalizado** debido a que resta el 30% correspondiente a validaciones empíricas en arena física con piso real, carga de la galería de rostros definitivos y producción de los entregables multimedia.

```
Progreso General: [██████████████░░░░░░] 70%

  ├── [100%] Arquitectura Matemática de Navegación (VFH-lite + Migas + ICP 2D)
  ├── [100%] Pipeline de Visión Facial (YuNet + SFace + Audio Buzzer/SAPI)
  ├── [100%] Módulo de Teleoperación Gestual (MediaPipe + Guardián LiDAR)
  ├── [100%] Suite de Pruebas Unitarias y Simulación Sintética (62/63 pass)
  ├── [100%] Sincronización Git y Despliegue en Raspberry Pi
  ├── [ 20%] Banco de Datos Facial (Pendiente: Fotos de sujetos de evaluación)
  ├── [  0%] Calibración en Pista Real con Obstáculos Físicos
  └── [  0%] Material Audiovisual y Diapositivas de Sustentación
```

---

## 2. Desglose por Subsistemas de la Competencia

### A. Tarea A: Navegación Autónoma $A \rightarrow B \rightarrow A$ y Reconocimiento Facial (Avance: 75%)
* **Ida ($A \rightarrow B$):**
  - Navegación reactiva con discretización angular en 73 direcciones ($5^\circ$ de resolución).
  - Inflado de radio del robot ($R_{inf} = 26\text{ cm}$) con comprobación geométrica de colisión lateral.
  - Velocidad lineal máxima de $0.18\text{ m/s}$ con desaceleración suave al aproximarse a $40\text{ cm}$ de la meta.
  - Generación de migas de pan (*breadcrumbs*) cada $20\text{ cm}$ en el trayecto de ida.
* **Escaneo Facial en $B$:**
  - Al arribar a $B$, el robot frena a $0\text{ m/s}$ y activa el pipeline de visión sobre el tópico `/oakd/rgb/preview/image_raw`.
  - Detector YuNet localiza rostro y 5 puntos fiduciales; SFace extrae vector de características de 128 dimensiones.
  - Si no detecta rostro al frente, ejecuta un barrido panorámico controlado ($\pm 35^\circ$) a baja velocidad angular.
  - Al identificar a la persona, emite un acorde musical Do5-Mi5-Sol5 al buzzer del Create 3 (`/cmd_audio`) y publica el nombre en `/nombre_reconocido`.
* **Retorno ($B \rightarrow A$):**
  - Invierte la secuencia de migas de pan registradas en la ida.
  - Algoritmo de atajo (*shortcut*): si el robot ve directamente una miga más cercana al origen sin obstáculos intermedios, avanza directo hacia ella.
  - Alineación de nubes LiDAR con algoritmo ICP 2D (*Iterative Closest Point*) cada $80\text{ cm}$ para corregir la deriva acumulada por patinaje de ruedas.
* **Anuncio en $A$:**
  - Al llegar dentro del radio de tolerancia de $A$ ($15\text{ cm}$), el robot frena y el nodo anunciador (`escuchar_anuncio_laptop.py`) sintetiza la voz en español indicando el nombre de la persona encontrada.

### B. Tarea B: Teleoperación por Gestos de Mano (Avance: 70%)
* **Detección de Mano:** Mediante Google MediaPipe `HandLandmarker` (GPU/CPU laptop a 30 FPS).
* **Mapeo Cinemático:**
  - `pointing_up` (☝️): Avance recto ($+0.15\text{ m/s}$).
  - `thumb_down` / `dislike` (👎): Retroceso seguro ($-0.12\text{ m/s}$).
  - `victory` (✌️): Giro horario ($+0.5\text{ rad/s}$).
  - `love_you` (🤟): Giro antihorario ($-0.5\text{ rad/s}$).
  - `open_palm` (✋) / Mano ausente: Parada inmediata ($0, 0$).
* **Guardián de Colisiones Activo:**
  - Aunque el operador humano ordene avance o retroceso, el nodo analiza en tiempo real el LiDAR (`/scan`).
  - Si hay un obstáculo frontal a menos de $32\text{ cm}$, anula la velocidad de avance.
  - Si hay un obstáculo trasero a menos de $32\text{ cm}$, anula la velocidad de retroceso.

---

## 3. Estrategia "Cero Colisiones" (Prioridad Crítica de Puntuación)

Dado que las colisiones físicas conllevan penalizaciones drásticas en el puntaje de la competencia, se diseñó un esquema defensivo multicapa:

1. **Inflado Dinámico del Chasis:** Ningún rayo LiDAR dentro de una banda de $26\text{ cm}$ del eje de avance es permitido.
2. **Priorización de Giro en el Sitio:** Si la distancia frontal es inferior a la distancia de frenado seguro ($32\text{ cm}$), la velocidad de traslación se fuerza a cero absoluto, permitiendo únicamente giros sobre el propio eje.
3. **Filtro de Desaceleración Fuerte:** La aceleración está acotada a $+0.4\text{ m/s}^2$, pero el frenado tiene disponible hasta $-0.8\text{ m/s}^2$ para detenciones de emergencia instantáneas.
4. **Bumper Físico del Create 3:** En caso de roce con un obstáculo no detectable por el LiDAR (por debajo del plano de escaneo de 20 cm), el evento activa de inmediato la maniobra de escape reactivo (`_escape`) con reversa y giro compensatorio.

---

## 4. Estado de Verificación y Pruebas Unitarias

La suite de pruebas automatizadas en `proyecto_final/1_competencia/tests/` arrojó el siguiente resultado:

```
========================= 62 passed, 1 skipped in 11.23s =========================
```

* **`test_nav_core.py` (22 tests):** Cobertura matemática de `free_distance`, proyección trigonométrica de rayos, rampa trapezoidal y penalización angular.
* **`test_navegacion.py` (14 tests):** Comportamiento de escape, selección de rumbo en pasillos estrechos, recorte de migas (*shortcuts*) y convergencia de ICP 2D.
* **`test_mision.py` (10 tests):** Máquina de estados finitos ($A \rightarrow B$, escaneo, $B \rightarrow A$, anuncio) en simulaciones sintéticas de pasillo en L y laberinto simple.
* **`test_facial.py` (9 tests):** Carga de modelos ONNX (YuNet y SFace), extracción de descriptores faciales y matching con distancia coseno.
* **`test_gestos.py` (7 tests):** Reconocimiento de morfología de dedos, filtrado temporal de comandos y disparo de freno por límite LiDAR.

---

## 5. El 30% Restante: Plan de Acción para la Fase Presencial

| Porcentaje | Actividad | Requisitos / Dependencias | Riesgo Asociado |
| :---: | :--- | :--- | :--- |
| **10%** | **Pruebas físicas en pista real** | TurtleBot 4 con batería $>80\%$, arena de prueba con cajas/paredes, medición de odometría real. | Deriva excesiva de ruedas Create 3 si el piso resbala. Mitigación: ICP activo y velocidad moderada. |
| **10%** | **Registro de Galería de Rostros** | Fotografías reales de los integrantes y docentes usando `capturar_persona.sh`. | Iluminación deficiente en la pista. Mitigación: Normalización de contraste en YuNet. |
| **10%** | **Entregables Finales** | Informe en PDF compilado con LaTeX, diapositivas de presentación y video resumen editado. | Tiempos de renderizado y maquetación. |
