# Registro Técnico de Problemas Resueltos y Diagnóstico

Este documento consolida los principales desafíos de ingeniería resueltos durante la preparación del TurtleBot 4 Lite para la competencia final.

---

## 1. Desfase del Reloj Interno del Create 3 (+20.9 horas)

### Diagnóstico del Problema
- En redes de laboratorio sin salida a internet (o en enlace directo por Ethernet), la Raspberry Pi y la base Create 3 no cuentan con acceso a servidores NTP externos.
- La Raspberry Pi mantenía la hora fijada manualmente, pero el Create 3 operaba con un tiempo retrasado por 75,368 segundos (~20.9 horas).
- En ROS 2 Jazzy, los nodos de navegación y transformaciones (`tf2`) descartan automáticamente cualquier mensaje cuyo sello de tiempo (`header.stamp`) difiera significativamente del tiempo del sistema, impidiendo el procesamiento de odometría y scans.

### Solución Implementada
- Se descubrió e integró el endpoint HTTP interno de la API web del Create 3 en la red USB privada:
  ```bash
  curl -s -X POST http://192.168.186.2/api/restart-ntpd
  ```
- Al invocar este endpoint tras sincronizar la hora de la Raspberry Pi, el demonio NTP del Create 3 reinicia y se sincroniza inmediatamente con la Pi como servidor horario local.
- **Resultado:** Desfase temporal reducido de +75,368 s a **0.08 s** en solo 2 segundos.

---

## 2. Saturación Térmica y de CPU en la Raspberry Pi 4 (Nav2 + SLAM)

### Diagnóstico del Problema
- Al intentar ejecutar el stack completo de **Nav2** (`nav2.launch.py`) junto con **SLAM Toolbox** a bordo de la Raspberry Pi 4, el robot experimentaba congelamientos frecuentes y tirones en las ruedas.
- Análisis de carga de CPU:
  - En ROS 2 Jazzy, el controlador predeterminado de Nav2 es **MPPI** (*Model Predictive Path Integral*), el cual simula miles de perturbaciones estocásticas Monte Carlo por ciclo de control (20 Hz).
  - Esto saturaba los 4 núcleos de la Raspberry Pi al **100%**, elevando la temperatura por encima de 75 °C (con *thermal throttling*).
  - Como consecuencia, el búfer serie del RPLIDAR (`/dev/ttyUSB0`) desbordaba por falta de atención de CPU, y los comandos de velocidad `/cmd_vel` sufrían retardos superiores a 500 ms, disparando el freno de emergencia automático del Create 3.

### Solución Implementada: Arquitectura Reactiva Liviana (*Edge Execution*)
- Se reemplazó el planificador pesado por un pipeline reactivo optimizado (**VFH-lite + migas de pan + ICP 2D** en [`nav_core.py`](file:///C:/Users/mando/turtleclaude4/proyecto_final/1_competencia/nav_core.py)).
- **Métricas de Rendimiento:**
  - Consumo de CPU a bordo: **< 5% de un solo núcleo** (reducción del 95%).
  - Memoria RAM: $< 35\text{ MB}$.
  - Temperatura de la Raspberry Pi: **55.5 °C estable** (throttled=0x0, sin caídas de voltaje).
  - Tiempo de cálculo de rumbo por ciclo: **< 0.8 ms**.

---

## 3. Movimiento Brusco, Cabeceo y Derrape de Ruedas

### Diagnóstico del Problema
- La lógica cinemática original aplicaba un corte abrupto de avance cuando el error angular superaba los $60^\circ$ (`turn_in_place`), provocando que el robot avanzara a trompicones cada vez que corregía rumbo.
- Además, al frenar bruscamente frente a un obstáculo, el Create 3 cabeceaba hacia adelante, despegando ligeramente las ruedas motrices y distorsionando la odometría de encoders.

### Solución Implementada
1. **Modulación Continua de Velocidad (*Smooth Blending*):**
   - Se escaló la velocidad lineal como una función continua suave en base al coseno del ángulo de giro.
2. **Rampa de Desaceleración Controlada:**
   - Se fijó `max_decel = 0.8 m/s²` para frenados programados (manteniendo frenado instantáneo únicamente ante peligro inminente $< 30\text{ cm}$).
3. **Rampa de Aceleración Angular:**
   - Se limitó la aceleración angular a `max_alpha = 2.5 rad/s²`, garantizando transiciones suaves de giro.
4. **Preservación de Inercia de Escape:**
   - Al salir de un retroceso por atasco, el rumbo previo `prev_heading` conserva el vector de escape en lugar de reiniciarse a 0, evitando que el robot vuelva a girar hacia el obstáculo recién evadido.

---

## 4. Resolución de Callejones Sin Salida y Concavidades en "U"

### Diagnóstico del Problema
- Los algoritmos de navegación puramente reactivos (campos potenciales / VFH básico) sufren de mínimos locales en trampas cóncavas o callejones en "U", pudiendo quedar atrapados o reingresar al callejón durante el camino de retorno.

### Solución Implementada
1. **Poda Topológica de Migas (`BreadcrumbGraph.prune_branch`):**
   - Cuando el robot detecta un obstáculo infranqueable o atasco, poda automáticamente las migas que registraron la entrada a ese callejón. Durante el regreso a $A$, el robot solo transita por el tronco principal libre de callejones.
2. **Sellado Virtual de Concavidades:**
   - Al detectar un atasco, el sistema proyecta una barrera virtual (`VirtualObstacles`) a lo ancho del callejón, impidiendo que el selector de rumbos vuelva a considerar esa dirección.
3. **Indexación Espacial con `SpatialKDTree2D`:**
   - Deduplica los puntos de obstáculos virtuales para mantener la memoria acotada y procesar las proyecciones en $O(\log N)$.
