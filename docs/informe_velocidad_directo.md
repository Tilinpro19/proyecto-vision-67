# Informe: velocidad y robustez del modo directo (Tarea A)

Rama `velocidad-directo` (sale de `guiado-directo`). Fecha: 2026-11-22.

> **Estado de la validación.** Los parámetros base (`config.json`) se ejercitaron en el robot
> real el 2026-11-19, en un pasillo de 0.56 m. Ese intento **falló**: rebotó 3 veces y se trabó
> cerca de A. Las correcciones posteriores de `guiado-directo` y **todo lo de este informe**
> (arreglos, detector de señales azules, perfiles rápidos) están validados **solo en
> simulación**. Ningún tiempo de este informe está medido en el robot.

## 1. Objetivo

El método del equipo ya era robusto. La meta ahora es hacerlo **rápido** sin perder eso, en
condiciones que no le faciliten las cosas al robot: pasillos estrechos, latencia de la red, un
LiDAR atrasado, odometría que deriva y una cámara de baja resolución que no ve todos los
cuadros. La rúbrica premia el tiempo relativo, `P = 4.5·(t_max − t)/(t_max − t_min)`. Un choque
suma penalidad y un intento que no vuelve a A vale 0, así que la condición para aceptar un
cambio es **cero choques**.

## 2. Qué algoritmo se optimizó

El robot corre el **modo directo** (`1_competencia/directo.py`):

- Sigue el pasillo recto, centrado entre las dos paredes.
- Obedece las flechas confirmadas girando 90° en el sitio.
- En las esquinas sin flecha decide con el LiDAR.
- En B llega por la bandera o por las caras, y regresa deshaciendo los tramos.

La rama `movimiento-eficiente` (navegación hacia una meta `--goal`) quedó atrás. Lo único que se
trajo de ella es el arreglo del detector de atascos (sección 5.1).

## 3. Señales nuevas ("Copia de señales.docx")

Se renderizó el documento completo con Word (9 páginas):

| Página | Contenido |
|---|---|
| 1 | 3 flechas **azules** a la derecha (imágenes) |
| 2 | 2 prohibidos (blanco y negro) |
| 3 | 2 círculos **rojos lisos** |
| 4 | 1 flecha **azul** a la derecha (dibujo vectorial) + 1 círculo azul liso |
| 5 | 1 flecha **ROJA** a la derecha (vectorial) + 1 círculo rojo liso |
| 6 | bandera de cuadros |
| 7–9 | 6 QR `milestone_*`, idénticos a los anteriores byte a byte |

- Las flechas son **azules o rojas**: círculo de color con borde y flecha negros. Antes eran
  blanco y negro, y el detector anterior daba **0 detecciones** con las de color.
- **No hay ninguna flecha izquierda en el documento.** Las 6 formas `bentArrow` no tienen
  `flipH` ni rotación. La izquierda se probó como espejo de la derecha.
- **Preguntas para el profe:**
  1. ¿Habrá flechas izquierdas?
  2. ¿Qué significa una flecha **roja**?
  3. ¿Qué significan los **círculos lisos** rojo y azul?
  4. ¿Se imprimen a color?

  Mientras tanto, la flecha roja se detecta pero **por defecto se ignora** (`flecha_roja =
  "ignorar"`). En tránsito, rojo suele ser prohibición: ignorarla cuesta a lo sumo tiempo, y
  obedecer un "no gires" sería peor. Con `--param flecha_roja=obedecer` se trata como una azul.
  Los círculos lisos no se detectan como flecha (hay un test que lo verifica).

**Arreglo** (`senales_forma._a_gris`, commits `d9c008a` y `c71c92f`): antes de clasificar, los
píxeles azules **y rojos** (por tono y saturación en HSV) se pintan al **nivel de blanco del
papel en esa misma imagen**. El color se informa aparte, en `Senal.color`.
La señal queda igual a la versión blanco y negro, así que sirven las mismas plantillas y la
misma verificación geométrica que nunca confunde izq, der y prohibido. Se usa también en el
filtro previo `buscar_candidatos()`, el que `mision.py` corre antes de `detectar()`.

Con poca luz el papel no es 255 sino ~130. Pintar el azul de blanco puro hacía ver "oscuras" las
esquinas de la hoja y la forma se rechazaba; usar el nivel del papel lo resolvió.

Robustez, con flecha a 0.5–1.4 m y giro de 0° y 25°:

| Luz | Detección |
|---|---|
| normal, cálida, oscura (60 % del brillo), desaturada (50 %) | 8/8 en ambos sentidos |
| oscura + desaturada | 8/8 |
| muy apagada (brillo 50 % y saturación 35 %) | 3–4/8 (límite conocido) |

Si la hoja se imprime **en gris** no queda color que mapear y el disco sale gris medio. Para ese
caso se agregó un respaldo: si la clasificación normal falla, se reintenta con un umbral entre
el negro y el gris del disco. Detecta 3/3 de 0.5 a 1.1 m, sin falsos positivos nuevos en fondos,
caras, patrones ni texturas.

En ningún caso hubo confusiones entre izq, der y prohibido. Tests:
`test_flecha_azul_se_detecta_y_nunca_se_confunde` (64 casos: azul en imagen, azul vectorial y roja),
`test_circulos_de_color_lisos_no_son_flechas`, `test_flecha_impresa_en_escala_de_grises_tambien_se_detecta`,
`test_prohibido_y_bandera_nuevos_se_detectan` y `test_objetos_azules_sin_flecha_no_son_senales`.

### 3.1 Alcance real del detector y FOV del preview

`mision.py` usa por defecto `/oakd/rgb/preview`: **250×250 px, recortado del centro del
sensor**, ~37° horizontales (medido en el laboratorio, ver `3_referencia/sigueme.py`), no los 69°
del sensor completo.

Alcance máximo con detección en todos los cuadros (de frente y a 20°):

| Imagen | Flechas azules | Prohibido | Bandera |
|---|---|---|---|
| preview 250 px, 37° (**la del robot**) | **1.3 m** | 2.1 m | 1.0 m |
| 640 px, 69° | 1.7 m | 2.2 m | 1.2 m |

**Bug encontrado:** `directo.py` calculaba el rumbo de la señal con `fov_deg = 69` también sobre
el preview. Una señal en el borde de la imagen (18.5° reales) salía a 34.5°. Eso hacía buscar el
rango de la señal en el LiDAR en otra dirección y ubicarla mal en el mapa. Arreglo (commit
`70fa945`): con imagen cuadrada se usa `fov_cuadrado_deg = 37`. Test:
`test_preview_cuadrado_usa_el_fov_recortado`.

**Sin confirmar en el robot:** el tamaño real del preview. La Pi no respondía por SSH para
revisarlo en una foto guardada.

## 4. Método

### 4.1 Simulador en condiciones duras (`tests/bench_directo.py`)

| Efecto real | Cómo se simula | Valor |
|---|---|---|
| Latencia del `cmd_vel` (DDS por Wi-Fi) | cola de comandos | 2 ciclos = 0.2 s |
| Ciclo lento de la Pi / LiDAR atrasado | el scan corresponde a una pose vieja | 3 ciclos = 0.3 s |
| Ruido del RPLIDAR A1 | ruido gaussiano en el rango | σ = 1 cm |
| Deriva de la odometría al girar | error de escala en el yaw | ±3 % |
| Cámara del robot | preview 250 px, FOV 37°, alcance por tipo (tabla 3.1), ángulo de visión ≤ 70°, línea de vista, **30 % de cuadros sin detección** | `tests/camara_sim.py` |

Un perfil se **acepta** solo si, en **todos** los escenarios y condiciones, cumple las cuatro
cosas:

1. Completa A→B→A.
2. Tiene 0 choques.
3. Mantiene una holgura mínima de al menos **radio + 5 cm (0.22 m)**. El robot real cuenta como
   choque algo a 0.20 m del centro.
4. Termina a menos de 0.35 m de A.

### 4.2 Circuito de referencia (`tests/pista_video.py`)

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
0.80 y 0.90 m, más el pasillo de 0.56 m medido en el intento real (`pista_angosta`).

**Límite:** las cotas salen del video a ojo, no de una medición.

## 5. Fallas encontradas y corregidas

### 5.1 Girar en el sitio se tomaba como atasco (commit `09ec5d9`)

- **Síntoma:** después de girar ~180° en B, el robot dejaba una barrera virtual *permanente*
  en el pasillo del regreso y daba vueltas hasta agotar `t_max`.
- **Causa:** `ProgressMonitor` solo medía desplazamiento. Junto a una pared la angular se reduce
  a ~0.55 rad/s, así que un giro de 180° tarda unos 5.7 s, más que `stuck_time` (4.5 s). El
  monitor declaraba "obstáculo bajo invisible".
- **Arreglo:** un giro neto de 60° o más dentro de la ventana cuenta como avance. Oscilar en el
  sitio sigue contando como atasco.
- **Test:** `test_progress_monitor_giro_en_el_sitio_cuenta_como_avance`.

### 5.2 Prohibido lateral antes de la bandera: media vuelta a 1.5 m de B (commits `438160a` y `1a2988c`)

- **Síntoma:** este es el caso del video. En el pasillo final de 0.8 m, con el prohibido en la
  pared izquierda, el robot decidía "prohibido" a 2.57 m y daba media vuelta. **Nunca llegaba a
  B**: misión en 0 puntos.
- **Arreglo:** la señal se ubica en el mundo (rumbo en la imagen + rango del LiDAR). Cuenta como
  **lateral**, y se ignora con registro en el log, solo si cumple **las dos** condiciones:
  1. Está corrida del eje del pasillo más de `max(0.15, ancho/2 − 0.15)`.
  2. Queda **más de 0.30 m antes de la pared del frente** que mide el LiDAR.

  Una señal en la pared o caja del fondo, aunque esté descentrada, nunca es lateral. Además, si
  la bandera está **cerca** (≤ 2 m) en el mismo cuadro, la bandera manda.
- **Corrección en el camino:** la primera versión solo miraba el desplazamiento lateral. Un test
  nuevo mostró que a 0.56 m rechazaba una flecha **real** del fondo corrida 20 cm. Se cambió al
  criterio de dos condiciones.
- **Tests:**
  - `test_prohibido_lateral_*`: con y sin bandera visible, con y sin salida lateral.
  - `test_flecha_descentrada_en_la_pared_del_fondo_se_obedece`: 8 casos, a 0.56 y 0.80 m.
  - `test_prohibido_al_fondo_con_sesgo_de_camara_se_obedece`: ±30 px, centrado y corrido.

### 5.3 Flecha impresa en una caja lateral (commit `5d34931`)

- **Síntoma:** caso del video en 00:46. Una flecha impresa al costado del pasillo, frente a una
  salida libre, se confirmaba y el robot doblaba a un callejón. Costaba **35 s** (61.8 s contra
  26.8 s).
- **Arreglo:** el mismo filtro lateral para flechas (`flechas_solo_frente = True`).
- **Test:** `test_flecha_impresa_en_caja_lateral_no_desvia`.

### 5.4 Callejón aparente: no rendirse y usar la cámara (commit `9377aeb`)

- **Síntoma:** el que vio el equipo en el robot. Al llegar a una caja con flecha que sobresale
  de la pared del fondo (como en el video), el robot **confirmaba la flecha**, pero el LiDAR no
  veía una salida "perfecta". Decía "callejón sin salida", miraba y daba media vuelta. Se
  repetía 3 veces y **nunca llegaba a las caras**. Pasaba ya con la caja sobresaliendo 0.2 m sin
  flecha, y desde 0.35 m también con flecha.
- **Causa:** una salida lateral solo contaba si los rayos a 75°, 90° y 105° estaban libres justo
  donde el robot frenaba. Si la salida empieza un poco más adelante (al costado de la caja) o el
  robot está algo torcido, alguno de los tres pega en el borde.
- **Arreglo, el "rescate":** antes de mirar y dar media vuelta:
  1. Revisa cada costado con un abanico de rayos de 45° a 105°. Un lado está "abierto" si algún
     rayo llega bastante más lejos que la pared lateral esperada.
  2. Si hay lado abierto, **prefiere el de la flecha que vio la cámara**. Si no hubo flecha,
     toma el menos visitado.
  3. Gira hacia ese lado y el piloto con evasión (VFH) **rodea la esquina** durante
     `t_respaldo`, sin decidir de nuevo hasta avanzar 0.85 m.
  4. Un callejón de verdad (paredes en 3 lados) sigue dando media vuelta. Se rescata una sola
     vez por lugar.
- **Resultado:** con la caja sobresaliendo 0.2 y 0.35 m, con y sin flecha, llega a las caras
  con **0 callejones** y 0 choques. Si sobresale 0.45 m o más, el hueco entre caja y pared queda
  en 0.41 m para un robot de 0.34 m; el VFH no se mete y el robot vuelve, igual que antes.
- **El regreso también fallaba** (commit `8bc7529`). Lo encontró la grabación en condiciones
  duras: de vuelta, el piloto por migas usaba los márgenes completos de `config.json` y quedaba
  trabado en el mismo hueco que la ida había pasado con margen chico. Solo 2 de 6 corridas (3
  perfiles × 2 derivas) volvían a A. Ahora, a menos de 1 m de un punto de rescate, el regreso
  usa el margen chico y va a `v_estrecho`: **6 de 6 vuelven**, con 0 choques. Pero el regreso
  por ahí es **lento** (34 a 174 s en total según la corrida) y **justo**: la holgura mínima
  llega a 0.205 m del centro con el agresivo, unos 3.5 cm de la carrocería. Queda para mejorar.
- **Tests:** `test_caja_que_sobresale_no_es_callejon` (4 casos),
  `test_caja_que_sobresale_vuelve_a_a_en_condiciones_duras` (3 perfiles) y
  `test_callejon_de_verdad_no_se_rescata`. Los de callejón con y sin caras siguen pasando, y el
  circuito del video queda igual (0 callejones).

### 5.5 Revisión externa (Antigravity) y doble decisión tras un giro (commit `a815a2c`)

- **Hueco del LiDAR tomado como salida:** en el rescate, una ventana sin retornos (pared negra,
  cinta reflectiva) contaba como abierta y el robot podía girar contra la pared. Ahora exige un
  rayo válido. Test: `test_rescate_no_toma_un_hueco_del_lidar_como_salida`.
- **Señal lateral medida desde el robot y no desde el eje:** con el robot corrido 15 cm en un
  pasillo de 0.75 m, el cartel del lado cercano no se filtraba. Ahora se corrige con las dos
  paredes. Test: `test_senal_lateral_se_mide_desde_el_eje_aunque_el_robot_vaya_corrido`
  (falla sin el arreglo).
- **Doble decisión en la misma esquina:** con el LiDAR atrasado 0.3 s, justo al terminar un giro
  el último scan todavía mostraba la pared de antes del giro. El robot decidía otra vez en el
  mismo punto y volvía por donde vino; con `config_agresivo.json` a 0.90 m eso agotaba el
  tiempo. Ahora no decide por frente cerrado hasta 0.6 s después de un giro. Test:
  `test_tras_un_giro_no_decide_con_el_scan_viejo`.
- **Menores:** `w_giro` del agresivo bajó a 1.9, el máximo del TurtleBot 4. `detectar()` usa el
  FOV del preview también para la distancia estimada.

### 5.6 `t_max` interno de 300 s aseguraba un 0

`config.json` tenía `t_max: 300`. Al pasarlo, la misión pasa a `PARADA` y el robot **se queda
quieto en medio del circuito**. Pero en la rúbrica `t_max` no es un límite fijo: es el tiempo del
equipo más lento. El perfil base ya tardaba ~253 s simulados en pasillos de 0.75 m, sin el
escaneo en B. Ahora `t_max` es **900 s** en los tres perfiles, como red de seguridad por si se
traba para siempre. **Si el laboratorio anuncia un tiempo máximo oficial, hay que poner ese.**

## 6. Resultados de velocidad

Tiempo A→B→A en segundos, el **peor caso** de las dos condiciones duras (deriva +3 % y −3 %),
con la **cámara realista** de la sección 4.1. La tabla completa por estado (recto, giro,
regreso) está en `docs/bench_directo.csv`.

Todos los perfiles mostrados tienen **0 choques** y las 7 decisiones correctas.

| Perfil | Acepta | 0.55 | 0.60 | 0.65 | 0.75 | 0.80 | 0.90 | real 0.56 |
|---|---|---|---|---|---|---|---|---|
| base (`config.json`) | sí | 256 | 253 | 254 | 253 | 182 | 170 | 48 |
| **rapido B → `config_rapido.json`** | **sí** | 174 | 172 | 170 | **170** | 149 | 148 | 33 |
| rapido C | no* | 174 | 171 | 169 | 169 | 148 | 145 | 33 |
| rapido D → `config_agresivo.json` | no* | 167 | 159 | 163 | 156 | 150 | 142 | 31 |

\* C y D no fallan por choques. Fallan porque terminan a 0.38–0.41 m del punto de salida. El
robot igual declara la llegada a la zona de A a menos de 0.40 m (`a_suficiente`), pero no sabemos
cuánto mide la zona A real. Con la cámara perfecta, C pasaba todo. Con la cámara realista deja de
pasar, y por eso **el intento 2 usa B**, que es igual de rápido (170 s contra 169 s a 0.75 m),
acelera más suave (0.5 en vez de 0.7 m/s²) y no adelanta el frenado.

**Mejora de `config_rapido.json` sobre `config.json`:** −33 % a 0.75 m (253 → 170 s), −31 % en
el pasillo real de 0.56 m (48 → 33 s) y −13 % a 0.90 m (170 → 148 s).

**Lectura de la tabla:**

- **El cuello de botella era el modo estrecho.** Bajo 0.80 m de ancho medido, todo el recorrido
  iba a `v_estrecho = 0.15 m/s`. Los pasillos del video (0.75 m) caen justo ahí.
- **El tramo recto es ~90 % del tiempo.** El techo físico está cerca: el Create 3 llega a
  0.306 m/s y los perfiles van a 0.30 m/s.
- **La cámara realista no empeoró los tiempos ni la seguridad.** El alcance de 1.3 m de las
  flechas alcanza para confirmarlas (2 lecturas) antes del punto de giro, aun perdiendo el 30 %
  de los cuadros.

### Qué cambia en cada perfil (sobre `config.json`)

| Parámetro | base | rapido (B) | agresivo (D) | Efecto |
|---|---|---|---|---|
| `v_recto` / `v_regreso` | 0.26 | 0.30 | 0.30 | tramo recto e ida/vuelta |
| `v_acercar` | 0.20 | 0.24 | 0.26 | con una flecha pendiente |
| `v_estrecho` | 0.15 | 0.24 | 0.26 | **pasillos < 0.80 m (la mayor ganancia)** |
| `v_cerca` | 0.10 | 0.16 | 0.18 | llegada al punto de giro y a A |
| `w_giro` | 1.4 | 1.8 | 2.0 | giros de 90° en el sitio |
| `k_giro` | 2.2 | 2.2 | 2.6 | — |
| `d_lento` | 1.2 | 1.2 | 0.8 | empieza a frenar más tarde ante la pared |
| `max_accel` | 0.5 | 0.5 | 0.8 | — |

El centrado, la franja de paso y los márgenes de seguridad **no se tocaron**.

## 7. Estrategia de los 3 intentos

Cuenta el mejor intento de los tres. Por eso conviene asegurar la nota primero y arriesgar
después:

| Intento | Comando (a bordo, en `~/mision_mov`) | Por qué |
|---|---|---|
| 1 | `./competencia_onboard.sh` | `config.json`: velocidades conservadoras, para asegurar la nota |
| 2 | `./competencia_onboard.sh --config config_rapido.json` | −33 % de tiempo en simulación; cumple los 4 criterios en todos los casos, también con la cámara realista |
| 3 | `./competencia_onboard.sh --config config_agresivo.json` | −8 % más; 0 choques, pero puede parar a ~0.4 m del punto de salida. Solo si el 2 salió limpio; si no, repetir el 2 |

**Antes de la competencia:**

1. **Comparar lo que hay en la Pi**, sin modificar nada:
   `ssh ubuntu@192.168.185.3 'cd ~/mision_mov && md5sum *.py config*.json'` contra
   `md5sum 1_competencia/*.py 1_competencia/config*.json` de esta rama. `directo.py` importa
   `decision.py` y `pasillo.py`, y `mision.py` importa varios más: si **cualquiera** difiere, hay
   que copiarlo.
2. Copiar a `~/mision_mov/` todos los `.py` y `config*.json` que difieran (por `scp`). No usar
   `desplegar_en_robot.sh`.
3. En la primera corrida con `--config config_rapido.json`, revisar el log: la línea
   `ajustes {...}` no debe venir acompañada de `[params] '…' no existe`. Si aparece, la Pi tiene un
   `mision.py` viejo que descarta los parámetros del modo directo, y "rápido" correría a la
   velocidad base.
4. Poner una flecha azul impresa a ~1 m frente al robot y correr, a bordo,
   `./competencia_onboard.sh --dry`. `--dry` corre la misión completa sin publicar `/cmd_vel`,
   así que el robot no se mueve. En la consola o el log debe aparecer
   `flecha confirmada: izq a ~1.0 m`. **No usar `--ver` por SSH**: sin pantalla, `cv2.imshow`
   falla, y además `--ver` solo dibuja el estado, no las señales; `--ver` va solo desde la PC.
   Las flechas de color **nunca se probaron con la cámara real**.

## 8. Lo que falta validar en el robot

1. **Flechas azules con la cámara y la luz reales** (punto 4 de arriba). Con luz muy apagada el
   detector baja a 3–4/8.
2. **Tamaño real del preview**: 250×250 y ~37° según la medición anterior del equipo.
3. **Aceleración de 0.8 m/s² (solo el agresivo).** Puede hacer patinar las ruedas y empeorar la odometría.
4. **Tiempo en B:** escaneo de caras con 4 frames si es claro, más el anuncio. No está en los
   tiempos simulados; se suma igual a todos los perfiles.
5. **QR:** el de 10 cm casi no se lee en el preview de 250 px (`senales.py`). Solo se registran,
   no deciden el camino.

**Procedimiento sugerido en el laboratorio:**
`python3 2_laboratorio/calibrar.py --intentos 2 --c 5 -- --tarea A --config config_rapido.json`.
Si hay un choque o un roce, volver al perfil anterior.

## 9. Tarea B: gestos con la mano (commit `13582b6`)

1. **La Tarea B se caía al armar, con el robot real.** Desde el commit `08870c9`,
   `front_limit()` devuelve `(velocidad, aviso)`, pero `mision.tarea_b()` guardaba la tupla
   entera como velocidad lineal. Al armar con la palma, `send()` lanzaba `TypeError` y la misión
   terminaba. Con `--dry` no se veía, porque `send()` sale antes de convertir el valor. Además,
   la tupla siempre cuenta como "verdadera", así que el freno del LiDAR quedaba anulado. Ahora
   `gestos_teleop.comando()` devuelve números y la usan los dos programas. Tests:
   `test_comando_tarea_b_devuelve_numeros_y_respeta_el_freno` y
   `test_mision_usa_comando_y_no_la_tupla`.
2. **Un puño "de paso" desarmaba el robot.** Al pasar de like (adelante) a dislike (atrás), el
   pulgar cruza por la pose de puño 0.1–0.2 s y eso desarmaba. Había que volver a mostrar la
   palma 1 s. Ahora el puño **frena desde el primer cuadro**, pero desarma solo si se mantiene
   0.5 s (`PUNO_HOLD`). Test: `test_transicion_real_de_like_a_dislike_no_desarma`, con la
   transición del pulgar a 30 fps.
3. **Cada parpadeo del detector frenaba a tirones.** Si MediaPipe pierde la mano 1 o 2 cuadros,
   el robot sigue con el mismo movimiento hasta 0.15 s (`GRACIA`, ~4.5 cm a 0.3 m/s). Una pose
   distinta (puño, dislike, pistola) **no** entra en esa tolerancia: frena al momento.
4. El encabezado decía "dislike → parar", pero el código siempre lo usó para **retroceder**, con
   freno del LiDAR atrás. Se corrigió el texto.

## 10. Revisión de las reglas de la competencia

| Regla | Cómo lo cumple el código |
|---|---|
| Tarea A: cero intervención una vez iniciado | La misión no lee teclado; los `cv2.waitKey(1)` solo refrescan la ventana. El robot sale solo al lanzar el comando. Usar `/mision/parar` o Ctrl+C **termina el intento**: son solo para emergencias. |
| Tarea B: solo señales visuales, sin teclado, mouse ni joystick | Armar, mover, escanear (V) y anunciar se hacen con gestos. Al terminar (estado `FIN`) el programa se cierra solo: **no hay que cerrar la ventana con el mouse**. Se lanza antes de empezar el intento. |
| Choques con obstáculos y paredes | Todo cambio se aceptó solo con 0 choques y ≥ 5 cm de holgura en simulación. En la Tarea B el avance y el retroceso se frenan con el LiDAR (arreglo 9.1: antes ese freno estaba anulado). |
| Identificación incorrecta (penalidad severa) | Siempre anuncia el mejor candidato, aunque sea dudoso, nunca "desconocido". Así es máxima la probabilidad de acertar. |
| Puntaje 0 si no vuelve a A o si `t > t_max` | `t_max` interno a 900 s (sección 5.6). Ver la sección 7 para la escalera de intentos. |

## 11. Archivos

| Archivo | Qué es |
|---|---|
| `1_competencia/senales_forma.py` | flechas azules y rojas (`Senal.color`), respaldo para impresión en gris |
| `1_competencia/directo.py` | filtro de señal lateral, bandera manda de cerca, FOV del preview, `flecha_roja`, rescate de callejón aparente |
| `1_competencia/mision.py` | le pasa el alto de la imagen al modo directo; Tarea B con `comando()`; `t_max` 900 s |
| `1_competencia/gestos_teleop.py` | `comando()`, `PUNO_HOLD`, `GRACIA` |
| `1_competencia/nav_core.py`, `navegacion.py` | `ProgressMonitor` con giro neto |
| `1_competencia/config_rapido.json`, `config_agresivo.json` | perfiles de los intentos 2 y 3 |
| `1_competencia/tests/senales_ref_azul/` | imágenes y docx de las señales nuevas |
| `1_competencia/tests/pista_video.py` | circuito del video |
| `1_competencia/tests/bench_directo.py` | banco de velocidad (condiciones duras + cámara realista) |
| `1_competencia/tests/camara_sim.py` | resolución, FOV, alcance por tipo y pérdida de cuadros |
| `docs/bench_directo.csv` | resultados completos: perfil × escenario × condición |

Tests: **329 pasan** y 1 se salta (`pytest 1_competencia/tests`). `test_scans_reales` necesita `tests/scans_reales/`, que no está
versionado: copiarlo del worktree `guiado-directo`.
