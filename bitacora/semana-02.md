# Avance 02: Optimización Integral de Navegación, Sincronización de Reloj y Arquitectura On-Board

- **Fecha:** 03/11/2026
- **Responsable(s):** Mando
- **Objetivo del periodo:** Diagnosticar la saturación de la Raspberry Pi 4, solucionar el desfase de reloj con el Create 3, mejorar la fluidez de movimiento e implementar estructuras de datos optimizadas con despliegue autónomo a bordo.

---

## 🎯 Actividades Realizadas

1. **Diagnóstico y Corrección de Hardware / Enlace Ethernet:**
   - Conexión por cable Ethernet directo (`192.168.185.10` laptop $\leftrightarrow$ `192.168.185.3` TurtleBot 4, `ROS_DOMAIN_ID=67`) con latencia `< 1 ms`.
   - Verificación de salud: `throttled=0x0` (sin subvoltaje ni sobrecalentamiento, temperatura 55.5 °C), puerto USB 3.0 a 5000 Mbps y batería óptima al ~78% (15.2 V).
   - Detección del endpoint interno del Create 3 (`POST http://192.168.186.2/api/restart-ntpd`), reduciendo el desfase temporal de 20.9 horas a solo 0.08 s.

2. **Diagnóstico de Saturación de la Raspberry Pi:**
   - Se identificó la causa del "ahogamiento" de la Pi: la ejecución simultánea de `slam_toolbox` y el controlador **MPPI** de Nav2 en ROS 2 Jazzy, que saturaba los 4 núcleos al 100% y provocaba pérdidas de paquetes del RPLIDAR.
   - Se validó la alternativa reactiva ligera (**VFH-lite + migas + ICP 2D**) que consume menos del 5% de 1 solo núcleo de CPU.

3. **Mejoras de Fluidez Cinemática:**
   - Incorporación de rampas de desaceleración suave (`max_decel = 0.8 m/s²`) y aceleración angular (`max_alpha = 2.5 rad/s²`).
   - Escape reactivo ante bumpers (`/hazard_detection`) en `< 50 ms` con sellado de concavidades mediante barreras virtuales.

4. **Nuevas Estructuras de Datos y Algoritmos:**
   - **`SpatialKDTree2D`:** Indexación espacial $O(\log N)$ pura en NumPy/Python para deduplicación de obstáculos virtuales y consultas de proximidad.
   - **`BreadcrumbGraph`:** Grafo topológico con poda de ramas de callejones (`prune_branch`) para evitar reingresos cíclicos en el retorno $B \to A$.
   - **`detect_concavity`:** Detector predictivo de trampas en "U" y callejones cóncavos en el perfil LiDAR.
   - Creación de pruebas unitarias: **62 passed, 1 skipped (100% de éxito en 7.33 s)**.

5. **Infraestructura de Despliegue On-Board (Edge Execution):**
   - Creación del script maestro de sincronización `desplegar_en_robot.sh`.
   - Creación de lanzadores locales a bordo `tarea_a_onboard.sh` y `tarea_b_onboard.sh` en `ubuntu@192.168.185.3:~/turtleclaude4/`, permitiendo navegación autónoma con 0 ms de latencia y 0% dependencia de Wi-Fi.

---

## 🔍 Resultados y Evidencias

- **Informe Técnico Exhaustivo:** Generado y archivado en [`../entregables/informe_arquitectura_navegacion.md`](../entregables/informe_arquitectura_navegacion.md).
- **Código en el Robot:** Despliegue verificado y funcional en `~/turtleclaude4/proyecto_final/1_competencia/`.
- **Sensores activos validados:** `/scan` @ 7.6 Hz, `/odom` @ 17.2 Hz, `/oakd/.../compressed` @ 24.6 FPS, `/imu` @ 32.8 Hz.

---

## ⚠️ Dificultades y Soluciones

- **Problema encontrado:** El robot Create 3 acumulaba un desfase de reloj de +75,368 segundos (+20.9 horas) en redes aisladas sin servidor NTP externo, descartando mensajes en ROS 2.
  - **Solución implementada:** Se integró la llamada HTTP al endpoint interno `curl -s -X POST http://192.168.186.2/api/restart-ntpd` dentro de `tb4_connect.sh`, `reloj.py` y los lanzadores de tareas.
- **Problema encontrado:** Callejones sin salida y trampas en "U" causaban ciclos de búsqueda en planificadores puramente locales.
  - **Solución implementada:** Se diseñó `BreadcrumbGraph` con poda de ramas y sellado virtual de la boca del callejón, impidiendo que el robot reintente ramas bloqueadas.

---

## 📋 Siguientes Pasos

- [ ] Invitar a los compañeros de equipo en los ajustes de acceso de GitHub.
- [ ] Realizar la primera prueba física de avance suave (10 cm) con parada por sensor LiDAR.
- [ ] Integrar el reconocimiento facial de `facial.py` (YuNet + SFace) con la navegación autónoma para la Tarea B.
