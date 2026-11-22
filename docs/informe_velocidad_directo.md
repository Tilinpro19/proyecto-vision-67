# Informe: velocidad y robustez del modo directo (Tarea A)

Rama `velocidad-directo` (sale de `guiado-directo`). Fecha: 2026-11-22.
**Todos los números de este informe salen del simulador. Ninguno está medido en el robot real.**

## 1. Objetivo

El método del equipo ya era robusto. La meta ahora es hacerlo **rápido** sin perder eso, en
condiciones que no le faciliten las cosas al robot: pasillos estrechos, latencia de la red, un
LiDAR atrasado y odometría que deriva. La rúbrica premia el tiempo relativo,
`P = 4.5·(t_max − t)/(t_max − t_min)`. Un choque suma penalidad y un intento que no vuelve a A
vale 0, así que la condición para aceptar un cambio es **cero choques**.

## 2. Qué algoritmo se optimizó

El robot corre el **modo directo** (`1_competencia/directo.py`):

- Sigue el pasillo recto, centrado entre las dos paredes.
- Obedece las flechas confirmadas girando 90° en el sitio.
- En las esquinas sin flecha decide con el LiDAR.
- En B llega por la bandera o por las caras, y regresa deshaciendo los tramos.

La rama `movimiento-eficiente` (navegación hacia una meta `--goal`) quedó atrás. Lo único que se
trajo de ella es el arreglo del detector de atascos (sección 4.1).

## 3. Método

### 3.1 Simulador en condiciones duras (`tests/bench_directo.py`)

| Efecto real | Cómo se simula | Valor |
|---|---|---|
| Latencia del `cmd_vel` (DDS por Wi-Fi) | cola de comandos | 2 ciclos = 0.2 s |
| Ciclo lento de la Pi / LiDAR atrasado | el scan corresponde a una pose vieja | 3 ciclos = 0.3 s |
| Ruido del RPLIDAR A1 | ruido gaussiano en el rango | σ = 1 cm |
| Deriva de la odometría al girar | error de escala en el yaw | ±3 % |
| Cámara OAK-D Lite | modelo pinhole: FOV 69°, alcance 2.6 m, ángulo de visión ≤ 70°, línea de vista | `tests/camara_sim.py` |

Un perfil se **acepta** solo si, en **todos** los escenarios y condiciones, cumple las cuatro
cosas:

1. Completa A→B→A.
2. Tiene 0 choques.
3. Mantiene una holgura mínima de al menos **radio + 5 cm (0.22 m)**. El robot real cuenta como
   choque algo a 0.20 m del centro.
4. Termina a menos de 0.35 m de A.

### 3.2 Circuito de referencia (`tests/pista_video.py`)

El circuito se reconstruyó del video del grupo Carranza (00:00–01:05). El ancho del pasillo es
un parámetro; en el video mide ~0.75–0.90 m.

| Tramo | Video | Qué exige |
|---|---|---|
| 1 | sale recto, dobla a la izquierda **sin señal** | esquina por LiDAR |
| 2 | **flecha izquierda** en la caja del fondo | confirmar la flecha y girar 90° en el sitio |
| 3 | esquina en L **sin flecha**, libre a la derecha | esquina por LiDAR |
| 4 | **QR** en la pared del fondo, luego a la derecha | leer el QR antes de girar |
| 5 | esquina a la derecha | — |
| 6 | **flecha izquierda** en el zócalo del fondo | — |
| 7 | **QR** al fondo del pasillo largo, a la izquierda | — |
| 8 | **PROHIBIDO en la caja lateral izquierda** y **BANDERA** al fondo | no tomar el prohibido como cierre del pasillo |

Son 17.6 m de ida por el centro del pasillo y 7 decisiones. Se probó a 0.55, 0.60, 0.65, 0.75,
0.80 y 0.90 m, más el pasillo de 0.56 m medido en el primer intento real (`pista_angosta`).

**Límite:** las cotas salen del video a ojo, no de una medición.

## 4. Fallas encontradas y corregidas

### 4.1 Girar en el sitio se tomaba como atasco (commit `09ec5d9`)

- **Síntoma:** después de girar ~180° en B, el robot dejaba una barrera virtual *permanente*
  en el pasillo del regreso y daba vueltas hasta agotar `t_max`. Con latencia pasaba más seguido.
- **Causa:** `ProgressMonitor` solo medía desplazamiento. Junto a una pared la angular se reduce
  a ~0.55 rad/s, así que un giro de 180° tarda unos 5.7 s, más que `stuck_time` (4.5 s). El
  monitor declaraba "obstáculo bajo invisible".
- **Arreglo:** un giro neto de 60° o más dentro de la ventana cuenta como avance. Oscilar
  izquierda-derecha en el sitio sigue contando como atasco.
- **Test:** `test_progress_monitor_giro_en_el_sitio_cuenta_como_avance`.

### 4.2 Prohibido lateral antes de la bandera: media vuelta a 1.5 m de B (commit `438160a`)

- **Síntoma:** este es el caso del video. En el pasillo final de 0.8 m, con el prohibido en la
  pared izquierda, el robot decidía "prohibido" a 2.57 m y daba media vuelta. **Nunca llegaba a
  B** y la misión quedaba en 0 puntos.
- **Causa:** cualquier prohibido confirmado se trataba como "el camino de frente está cerrado",
  sin importar dónde estuviera.
- **Arreglo, con dos protecciones independientes:**
  1. La señal se ubica en el mundo (rumbo en la imagen + rango del LiDAR). Si queda a más de
     `max(0.15, ancho/2 − 0.15)` del eje del pasillo es lateral: se registra en el log y se
     ignora. Un prohibido al frente funciona igual que antes.
  2. Si la bandera está en el mismo cuadro, la bandera manda.
- **Tests:** `test_prohibido_lateral_antes_de_la_bandera_no_desvia` (con y sin salida lateral) y
  `test_prohibido_lateral_sin_bandera_a_la_vista_se_ignora_por_posicion`. Los dos tests de
  prohibido al frente siguen pasando.

### 4.3 Flecha impresa en una caja lateral (commit `5d34931`)

- **Síntoma:** caso del video en 00:46. Una flecha impresa al costado del pasillo, frente a una
  salida libre, se confirmaba y el robot doblaba a un callejón. Eso costaba **35 s** (61.8 s
  contra 26.8 s en el tramo de prueba).
- **Arreglo:** el mismo filtro lateral para flechas, con el parámetro `flechas_solo_frente`
  (`True` por defecto). En el video todas las flechas reales están en la pared o caja del fondo.
- **Test:** `test_flecha_impresa_en_caja_lateral_no_desvia`, que también comprueba que sin el
  filtro la flecha falsa se confirmaba.

## 5. Resultados de velocidad

Tiempo A→B→A en segundos, el **peor caso** de las dos condiciones duras (deriva +3 % y −3 %).
Todos los perfiles mostrados tienen 0 choques. La tabla completa por estado (recto, giro,
regreso) está en `docs/bench_directo.csv`.

| Perfil | Acepta | 0.55 | 0.60 | 0.65 | 0.75 | 0.80 | 0.90 | real 0.56 |
|---|---|---|---|---|---|---|---|---|
| base (`config.json`) | sí | 255 | 252 | 252 | 253 | 179 | 169 | 48 |
| rapido B | sí | 174 | 170 | 170 | 170 | 162 | 147 | 33 |
| **rapido C → `config_rapido.json`** | **sí** | 175 | 169 | 168 | **169** | 158 | 145 | 33 |
| rapido D → `config_agresivo.json` | al límite* | 166 | 158 | 157 | 156 | 142 | 140 | 31 |

\* D tiene 0 choques en todos los casos, pero una corrida termina a 0.35 m de A, justo en el
borde de la zona.

**Lectura de la tabla:**

- **El cuello de botella era el modo estrecho.** Bajo 0.80 m de ancho medido, todo el recorrido
  iba a `v_estrecho = 0.15 m/s`. Los pasillos del video (0.75 m) caen justo ahí: 253 s contra
  169 s a 0.90 m.
- **Mejora de `config_rapido.json`:** −33 % a 0.75 m (253 → 169 s), −31 % en el pasillo real de
  0.56 m (48 → 33 s) y −14 % a 0.90 m.
- **El tramo recto es ~90 % del tiempo.** El techo físico está cerca: el Create 3 llega a
  0.306 m/s y los perfiles van a 0.30 m/s.

### Qué cambia en cada perfil (sobre `config.json`)

| Parámetro | base | rapido (C) | agresivo (D) | Efecto |
|---|---|---|---|---|
| `v_recto` / `v_regreso` | 0.26 | 0.30 | 0.30 | tramo recto e ida/vuelta |
| `v_acercar` | 0.20 | 0.24 | 0.26 | con una flecha pendiente |
| `v_estrecho` | 0.15 | 0.24 | 0.26 | **pasillos < 0.80 m (la mayor ganancia)** |
| `v_cerca` | 0.10 | 0.16 | 0.18 | llegada al punto de giro y a A |
| `w_giro` | 1.4 | 1.8 | 2.0 | giros de 90° en el sitio |
| `k_giro` | 2.2 | 2.2 | 2.6 | — |
| `d_lento` | 1.2 | 0.9 | 0.8 | empieza a frenar más tarde ante la pared |
| `max_accel` | 0.5 | 0.7 | 0.8 | — |

El centrado, la franja de paso y los márgenes de seguridad **no se tocaron**. La holgura
mínima del perfil rápido es ≥ 0.222 m incluso a 0.55 m de ancho.

## 6. Estrategia de los 3 intentos

Cuenta el mejor intento de los tres. Por eso conviene asegurar la nota primero y arriesgar
después:

| Intento | Comando (a bordo, en `~/mision_mov`) | Por qué |
|---|---|---|
| 1 | `./competencia_onboard.sh` | `config.json`, ya validado en el robot real a 0.56 m: asegura la nota |
| 2 | `./competencia_onboard.sh --config config_rapido.json` | −33 % de tiempo en simulador, con todos los criterios con margen |
| 3 | `./competencia_onboard.sh --config config_agresivo.json` | solo si el 2 salió limpio, sin choques ni roces; si no, repetir el 2 |

**Antes de la competencia** hay que copiar a la Pi `directo.py`, `nav_core.py`,
`navegacion.py`, `config_rapido.json` y `config_agresivo.json` (por `scp` a
`ubuntu@192.168.185.3:~/mision_mov/`). No usar `desplegar_en_robot.sh`.

## 7. Lo que falta validar en el robot (no se pudo hacer aquí)

1. **Aceleración de 0.7–0.8 m/s².** Puede hacer patinar las ruedas y empeorar la odometría. El
   regreso por tramos corrige el rumbo con las paredes, pero hay que mirarlo en el robot.
2. **Detección real de las señales bajas** (10–25 cm del piso) con el ruido visual de las cajas:
   logos, códigos de barras, flechas impresas. El simulador detecta perfecto dentro de su FOV.
3. **Tiempo en B:** escaneo de caras con 4 frames si es claro, más el anuncio. No está en los
   tiempos simulados; se suma igual a todos los perfiles.
4. **Lectura del QR al fondo.** El robot decide a `ancho/2 + 0.03` ≈ 0.40–0.45 m de la pared, más
   cerca que los 0.50–0.60 m del video, así que el QR debería alcanzar a leerse.
   `senales.py` documenta el alcance real del QR de 10 cm.

**Procedimiento sugerido en el laboratorio:**
`python3 2_laboratorio/calibrar.py --intentos 2 --c 5 -- --tarea A --config config_rapido.json`.
Hacer 2 intentos por perfil y comparar el costo (`t + 5·choques`). Si hay un choque o un roce,
volver al perfil anterior.

## 8. Archivos

| Archivo | Qué es |
|---|---|
| `1_competencia/directo.py` | filtros de prohibido y flecha lateral, bandera manda |
| `1_competencia/nav_core.py`, `navegacion.py` | `ProgressMonitor` con giro neto |
| `1_competencia/config_rapido.json`, `config_agresivo.json` | perfiles de los intentos 2 y 3 |
| `1_competencia/tests/pista_video.py` | circuito del video |
| `1_competencia/tests/bench_directo.py` | banco de velocidad en condiciones duras |
| `1_competencia/tests/sim.py` | latencia, LiDAR cada N ciclos y ruido en `run()` |
| `docs/bench_directo.csv` | resultados completos: perfil × escenario × condición, tiempo por estado |

Tests: **257 pasan** y 1 se salta (`pytest 1_competencia/tests`). `test_scans_reales` necesita
`tests/scans_reales/`, que no está versionado: copiarlo del worktree `guiado-directo`.
