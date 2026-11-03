# Avance 02: Optimización Integral de Navegación, Estructuras de Datos y Despliegue On-Board

- **Fecha:** 03/11/2026
- **Responsable(s):** Mando (Tilinpro19)
- **Objetivo del periodo:** Diagnosticar la saturación de la Raspberry Pi 4, solucionar el desfase de reloj con el Create 3, mejorar la fluidez cinemática, implementar estructuras de datos espaciales y topológicas ($O(\log N)$) y desplegar la navegación autónoma on-board a 0 ms de latencia.

---

## 🎯 1. Nuevas Estructuras de Datos y Algoritmos Implementados

Todas las mejoras se integraron en `nav_core.py` y `navegacion.py`, manteniendo el 100% de compatibilidad con las pruebas existentes y elevando la suite a 62 pruebas unitarias:

### A) `SpatialKDTree2D` (Indexación Espacial $O(\log N)$)
- **Implementación:** Árbol KD bidimensional puro en Python y NumPy (sin dependencias de `scipy`, ideal para ejecución ligera en arquitecturas ARM64 como la Raspberry Pi 4).
- **Operaciones:**
  - `nearest(query)`: Búsqueda de vecino más cercano en tiempo $O(\log N)$.
  - `query_radius(query, radius)`: Búsqueda de puntos en radio en tiempo logarítmico con poda de planos divisores.
- **Integración:** Incorporado en `VirtualObstacles` para **deduplicación inteligente de puntos virtuales** (tolerancia de 3 cm), evitando la acumulación innecesaria en memoria y reduciendo el tiempo de cómputo de proyección a $< 0.1\text{ ms}$.

### B) `BreadcrumbGraph` (Grafo Topológico de Migas con Poda de Ramas Muertas)
- **Estructura:** Hereda de `list` para preservar compatibilidad total con índices `crumbs[upto]`, slicing `crumbs[:upto+1]` y llamadas existentes.
- **Poda de Callejones (`prune_branch`):** Cuando el robot detecta un atasco o trampa cóncava y retrocede, poda automáticamente del grafo las migas que entraron en ese callejón sin salida.
- **Retorno Optimizado (`pick_waypoint`):** Al regresar de $B$ hacia $A$, el algoritmo ignora los nodos podados, garantizando que el robot jamás vuelva a desviarse hacia un callejón que ya exploró y descartó en la ida.

### C) Detector Preventivo de Trampas Cóncavas / Trampas en "U" (`detect_concavity`)
- **Algoritmo:** Analiza en tiempo real el perfil angular del LiDAR en el cono frontal hacia la meta ($\pm 40^\circ$).
- **Criterio de Disparo:** Si todo el frente hacia la meta está bloqueado a menos de `trap_dist = 0.75 m` y los laterales forman un cuenco cóncavo envolvente, clasifica la geometría como trampa.
- **Evasión Anticipada:** Calcula de antemano el rumbo de escape más abierto en la periferia ($> 60^\circ$) para rodear la trampa antes de entrar en atascamiento profundo.

### D) Batería de Pruebas Unitarias Automatizadas
Se agregaron pruebas unitarias dedicadas en `1_competencia/tests/test_nav_core.py`:
- `test_spatial_kdtree_2d_nearest_and_radius`
- `test_detect_concavity_in_u_trap`
- `test_breadcrumb_graph_pruning_and_pick_waypoint`
- **Resultado en WSL:** **62 pruebas pasadas exitosamente (100% pass) en 7.33 s**.

---

## 🚀 2. Despliegue a Bordo (*Edge Execution* en Raspberry Pi 4)

Para solucionar la dependencia de Wi-Fi y garantizar latencia de 0 ms en la navegación reactiva, se crearon e integraron los scripts de despliegue y ejecución a bordo:

### Script de Despliegue Maestro [`desplegar_en_robot.sh`](file:///C:/Users/mando/turtleclaude4/desplegar_en_robot.sh)
1. **Comprobación de Red:** Verifica la conexión Ethernet con el robot (`192.168.185.3`, ping $< 1\text{ ms}$).
2. **Sincronización `rsync`:** Transfiere el código optimizado de `proyecto_final` y la librería `tb4lib` a `~/turtleclaude4/` en la Raspberry Pi.
3. **Instalación de Librería:** Instala `tb4lib` en modo editable en el entorno Python de la Raspberry Pi.
4. **Sincronización de Relojes:** Sincroniza la Laptop $\to$ Raspberry Pi (`sudo date -u -s @$now`) $\to$ Create 3 (`POST http://192.168.186.2/api/restart-ntpd`).
5. **Generación de Lanzadores:** Configura permisos de ejecución para los scripts on-board.

### Lanzadores On-Board en el Robot:
- `tarea_a_onboard.sh`: Ejecuta la navegación $A \to B \to A$ directamente en la Pi (consumo $< 5\%$ de CPU de un solo núcleo, inmune a desconexiones de red).
- `tarea_b_onboard.sh`: Ejecuta la visión y reconocimiento a bordo.

---

## 🕹️ 3. Modos de Ejecución Disponibles para el Equipo

El despliegue ya quedó instalado y verificado en la Raspberry Pi. Se cuenta con dos formas de correr la misión:

### Opción A: A bordo del robot (Recomendado para la competencia — 0% dependencia de Wi-Fi)
Desde tu terminal en WSL:
```bash
ssh ubuntu@192.168.185.3 "cd ~/turtleclaude4/proyecto_final/1_competencia && ./tarea_a_onboard.sh --goal 3.0 0.0"
```

### Opción B: Desde la Laptop (Con visualización gráfica en pantalla)
Desde WSL en tu laptop conectada por Ethernet o Wi-Fi:
```bash
cd ~/turtleclaude4/proyecto_final/1_competencia && ./tarea_a.sh --goal 3.0 0.0 --ver
```

---

## 🔍 Resultados y Evidencias

- **Salud del Hardware:** `throttled=0x0` (sin caídas de voltaje ni sobrecalentamiento, 55.5 °C), puerto USB 3.0 a 5000 Mbps y batería óptima al ~78% (15.2 V).
- **Sensores en vivo validados:** `/scan` @ 7.6 Hz, `/odom` @ 17.2 Hz, `/oakd/.../compressed` @ 24.6 FPS, `/imu` @ 32.8 Hz.
- **Reloj Sincronizado:** Desfase Create 3 reducido de 20.9 horas a **0.08 s**.
- **Informe Técnico:** Comparativa completa archivada en [`../entregables/informe_arquitectura_navegacion.md`](../entregables/informe_arquitectura_navegacion.md).

---

## 📋 Siguientes Pasos

- [ ] Invitar a los compañeros de equipo en los ajustes de acceso de GitHub.
- [ ] Realizar la primera prueba física de avance suave (10 cm) con parada por sensor LiDAR.
- [ ] Integrar el reconocimiento facial de `facial.py` (YuNet + SFace) con la navegación autónoma para la Tarea B.
