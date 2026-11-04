# Avance 03: Arquitectura Cero Colisiones, Misión A-B-A, Visión Facial y Teleoperación Gestual

- **Fecha:** 04/11/2026
- **Responsable(s):** Mando (Tilinpro19)
- **Objetivo del periodo:** Refinar la cinemática de evasión para asegurar cero colisiones, integrar el flujo completo de la misión ($A \to B \to \text{Reconocimiento} \to B \to A \to \text{Anuncio}$), pulir la teleoperación por gestos de mano con reversa protegida y evaluar el avance global con tope estricto al 70%.

---

## 🎯 1. Actividades Realizadas

### A) Refinamiento Cinemático Defensivo (Cero Colisiones)
- **Supresión de Avance Frontal:** En `1_competencia/navegacion.py`, se calculó la distancia efectiva libre como `eff_free = min(free, info["front"])`. Si el obstáculo frontal está a una distancia menor o igual a `stop_dist` ($32\text{ cm}$), la velocidad lineal se fuerza a $0.0\text{ m/s}$, obligando al robot a rotar sobre su propio eje y eliminando el riesgo de rozar esquinas al girar.
- **Rampa de Frenado Terminal:** Al aproximarse a menos de $40\text{ cm}$ de la meta, se aplica una modulación cuadrática de desaceleración para evitar sobrepasos o frenadas bruscas.
- **Multi-intento en ICP 2D:** Se configuró un contador de reintentos (`kf.attempts += 1`, hasta 3 veces) para la corrección de deriva por coincidencia de nubes de puntos LiDAR durante el retorno.
- **Respaldo por Bumper Físico:** Se integró la lectura de `/hazard_detection` (`s.bump()`) en la máquina de estados de `mision.py`, de modo que si un obstáculo fuera del plano del LiDAR roza el parachoques, el robot detiene inmediatamente el paso y ejecuta la maniobra reactiva de escape.

### B) Misión Autónoma Integral y Notificaciones Físicas
- **Encendido Automático de Sensores:** ROS 2 Jazzy apaga el motor del RPLidar y el sensor OAK-D Lite al bootear. Se incorporó en `mision.py` la invocación automática de los servicios `/start_motor` y `/oakd/start_camera`.
- **Feedback Acústico en Create 3:** Al confirmar una identificación facial positiva en el punto $B$, el robot publica en `/cmd_audio` una secuencia de notas (Do5, Mi5, Sol5) a través del zumbador interno de la base móvil.
- **Síntesis de Voz en Laptop:** Se creó el nodo complementario `escuchar_anuncio_laptop.py`, el cual se suscribe a `/nombre_reconocido` y utiliza el motor TTS local (Windows SAPI) para vocalizar en español el nombre de la persona identificada una vez que el robot retorna a la zona de partida $A$.

### C) Tarea B: Teleoperación por Gestos de Mano Refinada
- **Mapeo de Reversa:** Se reconfiguró el gesto `dislike` / pulgar abajo (👎) de parada a **retroceso continuo seguro ($-0.12\text{ m/s}$)**.
- **Guardián de Seguridad LiDAR:** Se implementó una verificación bidireccional en tiempo real:
  - Si el usuario ordena avance y el frente está a $< 32\text{ cm}$, la velocidad lineal se anula.
  - Si el usuario ordena retroceso y la parte trasera está a $< 32\text{ cm}$, la reversa se bloquea.
- **Catálogo de Gestos:**
  - ☝️ `pointing_up`: Avance recto ($+0.15\text{ m/s}$).
  - 👎 `thumb_down`: Retroceso seguro ($-0.12\text{ m/s}$).
  - ✌️ `victory`: Giro horario ($+0.5\text{ rad/s}$).
  - 🤟 `love_you`: Giro antihorario ($-0.5\text{ rad/s}$).
  - ✋ `open_palm` / Mano ausente: Detención inmediata ($0, 0$).

### D) Calibración de Gestos y Modo de Seguimiento Dinámico ("Pokémon")
- **Corrección Anatómica de Puño y Like:** Se rediseñó el clasificador de poses con holgura del 15% y discriminación estricta de la inclinación del pulgar, resolviendo la confusión entre puño cerrado (✊) y pulgar arriba (👍).
- **HUD con Telemetría de Seguridad:** La ventana de video ahora muestra en tiempo real advertencias de seguridad (`[ESPERANDO LIDAR]`, `[FRENO: libre X.XXm]`), facilitando el diagnóstico visual.
- **Modo Pokémon ("Sígueme"):** Se integró el modelo `pose_landmarker_lite.task` para seguimiento de entrenador en tiempo real, añadiendo el parámetro `--no-color` y tolerancia de agrupamiento LiDAR (`LEG_MIN_PTS=2`, `LEG_MAX_W=0.60 m`) para evitar pérdidas de rastreo por absorción o textura en prendas oscuras y jeans.

---

## 🔍 2. Resultados y Evidencias

- **Suite de Pruebas Unitarias:** Ejecución de `pytest 1_competencia/tests/ -v` arrojó **62 pruebas aprobadas y 1 omitida** en 11.23 segundos (100% de éxito en tests ejecutables).
- **Verificación de Sensores On-Board:**
  - Comunicación con Create 3 por Fast DDS verificado: Desfase temporal Create 3 $\leftrightarrow$ Pi = **0.0 s**.
  - Tópico `/scan` transmitiendo a 7.5 Hz (720 rayos filtrados).
  - Tópico `/oakd/rgb/preview/image_raw` transmitiendo a 20 FPS.
  - Nivel de batería verificado en $63\%$.

---

## ⚠️ 3. Evaluación Realista del Estado: Tope al 70%

> [!CAUTION]
> **El estado del proyecto se evalúa con prudencia técnica en un 70% de madurez.**
> Aunque todo el software, modelos ONNX y lógica de control están desarrollados y verificados en simulación/tests sintéticos, **NO se considera finalizado** hasta llevar a cabo:
> 1. Pruebas dinámicas en la pista física real con piso de laboratorio (el coeficiente de fricción y patinaje puede variar la odometría).
> 2. Carga de fotografías reales de los evaluadores/equipo en la carpeta `galeria/` con iluminación de prueba.

---

## 📋 4. Siguientes Pasos (El 30% Restante)

- [ ] **Validación en Pista Real (10%):** Probar navegación en el laboratorio con cajas reales, ajustando `safety_margin` y validando el retorno exacto a $A$ ($\le 15\text{ cm}$).
- [ ] **Población de Galería Facial (10%):** Utilizar `capturar_persona.sh` para registrar los rostros de los evaluadores con la cámara OAK-D Lite.
- [ ] **Producción de Entregables (10%):** Diseñar las diapositivas de la presentación final y editar el video demostrativo del robot completando la misión.
