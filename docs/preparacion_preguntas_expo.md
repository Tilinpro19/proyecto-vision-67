# Preparación para la exposición — TurtleBot 4 Lite

Guía para responder las preguntas del profesor. Cada valor viene del código (`1_competencia/`) o de los registros de las corridas reales del 23 de noviembre (`evidencias/experimentos/`). Lo que es **simulado** se dice explícitamente.

---

## 0. El proyecto en 30 segundos

> Un TurtleBot 4 Lite recorre un laberinto que no conoce, de A a B, guiándose por señales (flechas, prohibido, bandera) y por el LiDAR. En B gira, para frente a cada foto, reconoce a la persona buscada con YuNet + SFace, lee su QR y vuelve a A deshaciendo los giros de la ida. Al llegar anuncia el nombre y el texto del QR. Todo corre en la Raspberry Pi 4 del robot, sin mapa, sin SLAM y sin Nav2. La Tarea B hace lo mismo, pero manejado con gestos de la mano. Mejor corrida real: **105 s, persona correcta con su QR y 0 choques**.

---

## 1. Hardware y software

| Elemento | Detalle |
|---|---|
| Robot | TurtleBot 4 Lite: base iRobot Create 3, radio ~0.17 m, velocidad máx. ~0.31 m/s (usamos hasta 0.30) |
| LiDAR | RPLIDAR A1, 360°, ~7 Hz medido en pista |
| Cámara | OAK-D Lite; usamos el **preview de 250 × 250 px**, recortado del centro, con un campo de visión de ~37° |
| Computadora | Raspberry Pi 4 (a bordo), Ubuntu + **ROS 2 Jazzy**, Fast DDS, `ROS_DOMAIN_ID=67` |
| Lenguaje | Python 3.12 · NumPy · OpenCV 4.6 en la Pi (4.x/5.x en la PC) · MediaPipe (Tarea B, en la laptop) |
| Tópicos | `/scan`, `/odom`, `/cmd_vel` (TwistStamped), `/oakd/rgb/preview/image_raw/compressed`, bumper |

**¿Por qué todo a bordo?** La red del laboratorio es inestable y la de la universidad bloquea DDS. Si el control dependiera del Wi-Fi, un corte detendría la misión. La laptop solo lanza el programa por SSH (`tarea_a_onboard.sh`).

---

## 2. Arquitectura y por qué no usamos SLAM ni Nav2

- **Reactivo y local:** el robot no construye un mapa global. Decide con lo que ve en cada momento (LiDAR + cámara) y con una **memoria topológica pequeña**: los giros que hizo y cuántas veces tomó cada rama de un cruce.
- **Razones:**
  1. El circuito se revela el mismo día y no da tiempo de mapear.
  2. SLAM + Nav2 en la Pi 4 consume mucha CPU; la Pi ya va al 96–99 % con la visión.
  3. Un laberinto de pasillos de 90° se resuelve bien siguiendo el pasillo y decidiendo en las esquinas.
  4. Menos piezas implica menos puntos de falla (reloj, TF, DDS).
- **Archivo principal:** `mision.py` (máquina de estados). El modo directo de la Tarea A está en `directo.py`; el piloto de respaldo (VFH, migas, ICP) en `navegacion.py` y `nav_core.py`; la visión en `senales_forma.py` y `facial.py`; los gestos en `gestos_teleop.py` y `mano_teleop.py`.

---

## 3. Máquina de estados de la Tarea A

`ESPERA → SEGUIR → LLEGADA_B → ESCANEAR → VOLVER_A_A → ANUNCIAR → FIN` (y `PARADA` ante una emergencia o Ctrl+C).

| Estado | Qué hace | Cómo sale |
|---|---|---|
| SEGUIR | Va recto por el pasillo y decide en las esquinas | Llegada a B (ver abajo) |
| LLEGADA_B | Frena | Pasa a escanear |
| ESCANEAR | Revisa cara por cara y lee el QR del objetivo | Objetivo seguro + QR, o vuelta de 360°, o 45 s |
| VOLVER_A_A | Deshace los giros de la ida | A < 0.20 m de A, o «zona de A» (abajo) |
| ANUNCIAR | Nombre + texto del QR por terminal, `/nombre_reconocido` y buzzer | FIN |

**Cuándo considera que llegó a B:**
1. **Bandera confirmada:** 2 cuadros seguidos con la bandera a ≤ 1.0 m. La distancia se mide con el LiDAR en la dirección de la señal.
2. **Callejón con caras:** el frente cerrado y sin salidas; el robot mira 1.2 s, barre ±60° y ve una cara.
3. **Caras al frente:** en un callejón sin salidas basta una cara. Si hay una salida lateral, solo cuenta una cara **grande** (≥ 13 % del ancho de la imagen). Así se evita confundir a una persona del público detrás de la pared.
4. Tras 2 callejones sin caras escanea igual, para no perder el intento.

**Seguridad (en todos los estados):** el LiDAR frena antes de chocar. Si el paso queda bloqueado 2 s, toma el control el piloto VFH de respaldo durante 5 s. Un contador de choques registra cualquier contacto (< 0.20 m o bumper).

**`t_max` interno = 900 s:** es solo una red de seguridad. La rúbrica usa el tiempo del equipo más lento como `t_máx`, así que cortar antes aseguraría un 0.

---

## 4. Seguidor de pasillo (tramo recto)

Control proporcional, no planificador.

**Giro (velocidad angular):**

```
ω = k_r · ( e_rumbo + clip( k_c · (d_izq − d_der) / 2 , ±0.35 ) ) + repulsión_de_pared
```

- `e_rumbo`: diferencia entre el rumbo del pasillo (cardinal: 0°, 90°, 180°, 270° en el marco de A) y el rumbo actual de la odometría.
- `d_izq`, `d_der`: distancia a cada pared, medida con el LiDAR a ±90° del rumbo del pasillo. Solo se usa si la ventana de rayos es pareja (sin hueco ni borde).
- `k_r = 1.8 rad/s por rad` · `k_c = 1.5 rad por m` · corrección de centrado máx. 0.35 rad · `|ω| ≤ 0.9 rad/s`.

**Avance (velocidad lineal):**

```
v = mín( v_máx , k_f · (x_obs − d_parada) ) , con aceleración ≤ 0.5 m/s²
```

- `x_obs`: primer punto del LiDAR dentro de la **franja de paso** (ancho del robot + 3 cm por lado) frente al robot.
- `k_f = 0.8 s⁻¹` · `d_parada = 0.24 m`. Si va muy torcido (error > 0.6 rad), primero corrige el rumbo y avanza en 0.
- La franja bloqueada durante 2 s da paso al **piloto VFH**.

**Velocidades (perfil `config_rapido.json`):** recta 0.30 m/s · con flecha pendiente o cerca del frente 0.24 · pasillo angosto (< 0.80 m) 0.24 · llegada al punto de giro 0.16.

**Corrección del rumbo con las paredes:** después de cada giro, cuando ambas paredes se ven rectas (ajuste por mínimos cuadrados, residuo < 1.5 cm), paralelas (< 3°) y al ancho medido, la odometría se corrige una vez con el ángulo de las paredes (mediana de 3 lecturas). La odometría del Create 3 deriva 10–15° en una media vuelta.

---

## 5. Piloto de respaldo: VFH simplificado (`nav_core.py`)

Se usa para esquivar y escapar, y en el regreso por migas.

- Evalúa **73 rumbos candidatos**, cada 5° de −180° a +180°.
- Para cada rumbo calcula la **distancia libre**: hasta dónde puede avanzar un robot «inflado» (radio 0.17 m + margen de seguridad).
- Descarta los rumbos que no dejan avanzar al menos `min_free` = 0.45 m.
- Elige el de **menor costo**:

```
costo = w_meta · |rumbo − meta| + w_giro · |rumbo − rumbo_anterior| − w_holgura · holgura_lateral
w_meta = 1.0 · w_giro = 0.4 · w_holgura = 5.0 (holgura saturada en 0.40 m)
```

- **Velocidad:** 0 si el libre al frente es < `stop_dist` (0.30–0.34 m), máxima si es > `slow_dist` (0.90 m) y lineal entre ambos. Si el rumbo elegido está a más de 60° del actual, gira en el sitio.
- **Escape de atascos:** si en `stuck_time` (4.5–5 s) no avanza 8 cm, retrocede 0.15 m a 0.08 m/s y gira hacia lo más abierto hasta 2 s. Pone una **barrera virtual** de 45 s donde se trabó, para no volver a meterse.
- **Detección de concavidades:** reconoce trampas en «U» y sesga la meta hacia la salida.

> **Si preguntan «¿por qué VFH y no A\* o RRT?»:** sin mapa no hay grafo para A\*. VFH (Vector Field Histogram) es un método reactivo clásico que decide con la medición actual, y es barato en CPU.

---

## 6. Heurística de decisión en las esquinas

**Cuándo decide** (solo en esquinas; las salidas laterales sin flecha se ignoran para ir rápido):
- El **frente se cierra** a la distancia de decisión: `ancho/2 + 0.03 m`, entre 0.30 y 0.70 m.
- O una **flecha confirmada** queda a la distancia de giro y la salida que indica ya está libre.
- Tras un callejón sin caras, cada salida lateral se vuelve a decidir con la memoria de visitas.

**Cómo detecta las salidas libres:** rayos del LiDAR a **75°, 90° y 105°** a cada lado (ventana de ±4°). Una salida está libre si los tres rayos llegan más lejos que `máx(0.55 m, ancho/2 + 0.35 m)`. El rayo de 75° se ignora si pega en la pared del frente (robot pegado al fondo).

**Orden de prioridad:**
1. **Señal:** una flecha se **confirma con 2 lecturas iguales**. Se **ubica en el mundo** (rumbo en la imagen + rango del LiDAR en esa dirección) y se obedece al llegar a ella. Una flecha ya obedecida (a < 0.60 m) no se vuelve a obedecer.
2. **El LiDAR tiene la última palabra:** si la flecha apunta a una pared, se ignora y se registra un aviso. Una detección falsa nunca puede llevar a chocar.
3. **Sin flecha: la rama menos visitada** del cruce. Cada cruce (identificado por su posición, radio 0.8 m) guarda cuántas veces se tomó cada rumbo cardinal. Es una idea **análoga al algoritmo de Trémaux** para laberintos. En empate: recto › izquierda › derecha.
4. **Prohibido:** marca la rama de frente como agotada y busca otra salida; sin otra salida, media vuelta. Un prohibido pegado en una pared **lateral** no cierra el camino.
5. **Callejón** (ninguna salida): mira al frente 1.2 s, barre ±60° a 0.8 rad/s buscando caras y, si no hay, **media vuelta**; la rama queda marcada como agotada.
6. **Callejón aparente (rescate):** si una caja sobresale y la salida empieza un poco más adelante, antes de dar media vuelta prueba un abanico de rayos de 45° a 105°. Si un lado se abre (prefiere el de la flecha), gira hacia él y el VFH rodea la esquina.

**Detalles anti-error:**
- Tras un giro espera 0.6 s antes de volver a decidir por frente cerrado, porque el scan puede ser de antes del giro (el LiDAR llega atrasado ~0.3 s en la Pi).
- Las flechas pegadas al costado del pasillo (cajas impresas) se ignoran: solo cuentan las del frente.
- Las flechas **rojas** se ignoran por defecto; su significado no estaba confirmado.

---

## 7. Giro en arco

**Problema:** antes, en cada esquina, el robot frenaba en el centro del cruce, giraba 90° en el sitio y volvía a acelerar. En un circuito de muchas esquinas eso suma decenas de segundos.

**Solución:** empezar a girar **R metros antes del centro del cruce**, avanzando y girando a la vez:

```
v = R · ω     (si R·ω supera v_máx, se reduce ω = v_máx / R)
R = mín(0.25 m, ancho/2, lo que falta hasta el centro del cruce)
```

- **Geometría:** el robot recorre un cuarto de circunferencia de radio R que empieza R antes del centro y termina centrado en el pasillo nuevo. Si **R ≤ ancho/2**, la trayectoria libra la esquina interior y queda a ≥ ancho/2 de las paredes exteriores.
- **Seguridad:** en cada ciclo revisa con el LiDAR **todo el arco que falta**. Si algún punto queda a menos de radio + 6 cm del camino, ese ciclo gira en el sitio. La franja frontal sigue frenando el avance.
- **Cuándo no se usa:** en la media vuelta (180°) ni al salir de un callejón (rescate); tampoco cerca de donde se rodeó una caja, ni con caras a la vista (para no saltarse B).
- **Efecto simulado (perfil rápido):** −7 % de tiempo total a 0.75 m (170 → 158 s) y −14 % a 0.90 m (148 → 127 s), con 0 choques. Con el perfil agresivo, 145 s a 0.75 m.

**Timeout del giro por progreso:** el giro se considera trabado solo si **no avanza 5° en 2 s**, no por un tiempo fijo. Si un arco no avanza en 0.8 s, sigue girando en el sitio. Si quedó a menos de 12° del objetivo y oscila, el giro se da por hecho y las paredes corrigen el rumbo. Así se arregló el giro cortado de la corrida 3.

---

## 8. Regreso a A

**Por tramos (por defecto):** invierte la lista de giros de la ida. Para cada giro, en orden inverso, avanza con el mismo seguidor de pasillo hasta el punto donde giró y gira al rumbo opuesto (también en arco). Detecta el punto de giro proyectando la distancia que falta sobre el rumbo y comprobando que la salida lateral esté libre. Si se pasa más de 0.5 m, gira igual.

**Sin lazos:** si en la ida el robot volvió a pasar a menos de 0.5 m de un giro anterior, con al menos 3 giros en medio, todo ese tramo fue una vuelta y se elimina. En la corrida real del 23-nov habría bajado el regreso de 11 a 7 giros. Se exigen 3 giros en medio porque dos giros cercanos seguidos son un zigzag real, no una vuelta.

**Respaldo: migas de pan + ICP** (si la ida tuvo una media vuelta o el regreso por tramos se bloquea):
- Durante la ida deja **migas** cada 0.20 m y guarda **keyframes** del LiDAR cada 0.80 m.
- Al volver sigue las migas con el VFH, mirando hasta 2.5 m adelante para recortar camino.
- Al pasar a menos de 0.5 m de un keyframe corre **ICP punto a punto** (Iterative Closest Point) entre el scan actual y el guardado. Si el ajuste es bueno (≥ 60 % de inliers, error RMS ≤ 5 cm, salto ≤ 0.35 m), corrige la deriva de la odometría.
- **ICP en una línea:** empareja cada punto del scan con el más cercano del otro, calcula la rotación y traslación que mejor los alinea (solución cerrada con la SVD de la covarianza) y repite hasta converger.

**Llegada a A:** a menos de 0.20 m de A. O bien «zona de A»: a menos de 0.40 m y sin acercarse más durante 4 s, solo después de haberse alejado más de 1 m (para que una pista en «U» no lo dé por llegado en B).

---

## 9. Visión: detección de señales (`senales_forma.py`)

OpenCV clásico, **sin redes neuronales**: rápido en la Pi y explicable.

**Cadena para las flechas y el prohibido:**
1. **Filtro previo** barato (`buscar_candidatos`): un umbral adaptativo busca zonas de alto contraste; si no hay, ni se corre el detector.
2. **Candidatos circulares:**
   - Contornos de dos binarizaciones (Otsu y adaptativa) con tamaño ≥ 18 px, proporción entre 0.55 y 1.45, relleno ≥ 0.42, solidez ≥ 0.80 y circularidad `4πA/P²` ≥ 0.52.
   - **Respaldo con la transformada de Hough**, para anillos rotos o pegados al borde de la imagen; se evalúan después de los contornos.
3. **Recorte → 32×32 px → binarización de Otsu.**
4. **Comprobación del anillo:** oscuro ≥ 38 % en el anillo, en los 4 cuadrantes (tolera uno débil), esquinas claras e interior entre 12 % y 52 % oscuro.
5. **Correlación con plantillas** del interior del círculo: izquierda, derecha y prohibido, **en dos formas** (la del documento «↱/↰» y la impresa en la pista «↳/↲», espejadas verticalmente). Mínimo 0.42 y ventaja ≥ 0.14 sobre el mejor de otro tipo de la misma familia.
6. **Verificación geométrica** que asegura que izquierda y derecha **nunca** se confundan: de qué lado queda el tallo de la flecha, en qué cuadrante el codo y qué cuadrante queda vacío. Para el prohibido, la diagonal.

**Bandera de cuadros:**
- Primero con plantillas de damero (también giradas ±10°).
- La bandera real estaba **dibujada ondeando** y los cuadros salían deformados, así que se añadió un respaldo **por manchas**: ≥ 8 manchas oscuras sólidas (solidez ≥ 0.8), de tamaño parecido (variación ≤ 0.5) y con vecinas a una distancia del orden de su lado (damero).
- **Un QR no es bandera:** si aparecen ≥ 2 cuadrados anidados negro-blanco-negro (los cuadros de posicionamiento del QR), se descarta. Medido: 6/6 QR tienen 3 y 4/4 banderas tienen 0.
- Las «flechas» que caen dentro de la bandera se descartan.

**QR:** OpenCV `QRCodeDetector`, con reintentos (reescalado 2×, borde blanco, canal de tinta) en el modo de mayor esfuerzo. En la ida se lee en **1 de cada 3 cuadros**: solo se registran como hitos y cuestan 51 ms por cuadro en la Pi.

**Distancia y ubicación de una señal:**
- El rumbo sale de la columna en la imagen con un modelo **pinhole**: `rumbo = atan((ancho/2 − cx) / f)`, con `f = (ancho/2) / tan(FOV/2)` y FOV = 37° en el preview cuadrado.
- La distancia es la **mediana del LiDAR** en ese rumbo (±3°); si no hay rayo válido, se estima por el tamaño.

**Resultados:** en **46 cuadros reales** de la pista detectó **8 señales, todas correctas (6 flechas y 2 banderas), con 0 falsos positivos**. Alcance con el preview de 250 px: flechas ~1.3 m, prohibido ~2.0 m, bandera ~1.0 m. Más de 100 pruebas automáticas solo de señales, que incluyen perspectiva, ruido y luz variable.

---

## 10. Visión: reconocimiento facial (`facial.py`)

**Modelos (de OpenCV Zoo):**
- **YuNet** detecta las caras y entrega la caja y **5 puntos de referencia** (ojos, nariz, comisuras), con confianza mínima de 0.6.
- **SFace** alinea la cara con esos 5 puntos (`alignCrop`) y produce un **vector de 128 dimensiones** (embedding), normalizado.

**Galería offline:** 17 personas, 94 fotos. Cada foto también se usa **espejada**, lo que da 188 embeddings. Se guardan en una **caché por modelo**: la carga en la Pi bajó de 145 s a 0.6 s.

**Decisión:**

```
similitud = coseno(embedding_cámara, embedding_galería)   (promedio de los 3 mejores cuadros)
seguro  ⇔  similitud ≥ 0.363  y  supera al segundo candidato por ≥ 0.04
```

- 0.363 es el umbral recomendado por OpenCV para SFace con distancia coseno.
- **Calidad de un cuadro** = confianza de YuNet × tamaño de la cara (satura en 80 px) × nitidez (varianza del Laplaciano). Se promedian los 3 mejores.
- Si nadie supera el umbral, igual responde el más parecido, marcado como «dudoso»: responder algo es mejor que nada.
- Las imágenes chicas (el preview mide 250 px) se agrandan a 480 px antes de detectar.

**Escaneo en B con persona objetivo (`EscaneoObjetivo`, `--persona "Nombre"`):**
1. Gira a 0.8 rad/s. Si ve una cara nueva fuera del centro, **gira hacia ella** con control proporcional.
2. Con la cara **centrada** (< 12 % del ancho desde el centro), **para**, espera **0.4 s quieto** (recién frenado el cuadro sale movido) y junta **3 cuadros recortados solo alrededor de esa cara**, para que no se cuele la vecina.
3. Identifica. Si es el objetivo pero dudoso, junta 3 cuadros más.
4. Marca la dirección de esa cara como vista (±0.15 rad, ~9°) y sigue con la siguiente.
5. Al confirmar al objetivo lee su **QR** (solo el del objetivo; es lo más caro en la Pi). Si no se lee, hace un paneo de ±0.55 rad durante hasta 3 s.
6. Termina con el objetivo confirmado + QR, al completar 360° o a los 45 s.
7. Guarda una **foto** con la cara marcada, el nombre, el score y «SEGURO/DUDOSO» (evidencia).
- El nombre de `--persona` se normaliza: «Thiago Frías» equivale a «Thiago_Frias». Si no está en la galería, la misión no arranca.
- Probado con los modelos reales: revisa **4 de 4 caras** distintas cuando están separadas ≥ 13° (~25 cm a 1 m).

**Lección clave:** YuNet 2023 requiere OpenCV ≥ 4.8; la Pi tiene OpenCV 4.6 (paquete del sistema, que también usa ROS). En la Pi **fallaba toda detección** sin que se notara. Se usa **YuNet 2022mar**, la versión compatible, y el chequeo previo (`chequeo_pi.sh`) ahora exige detectar una cara real de la galería.

---

## 11. Tarea B: teleoperación por gestos

**MediaPipe Hand Landmarker** (en la laptop, con la webcam) entrega **21 puntos** de la mano por cuadro.

**Clasificación de la pose (reglas geométricas, sin entrenar nada):**
- Un dedo está **doblado** si su punta queda más cerca de la muñeca que su articulación media (con tolerancia).
- **Pulgar arriba o abajo:** según la dirección del vector base→punta del pulgar respecto al tamaño de la mano.
- Mano demasiado chica (< 25 px de muñeca a la base del dedo medio) = muy lejos, no se confía.

| Pose | Acción | Condición |
|---|---|---|
| Palma abierta | Armar el control | sostener 1.0 s |
| Pulgar arriba («like») | Avanzar | 3 cuadros estables |
| Pulgar abajo («dislike») | Retroceder | 3 cuadros estables |
| «Pistola» izquierda / derecha | Girar | 3 cuadros estables |
| V | Escanear en B · anunciar en A | sostener 1.5 s |
| Puño | Parar y desarmar | sostener 0.5 s |

**Seguridad:**
- Solo se mueve armado y con una pose estable; cualquier duda deja el robot quieto.
- Si MediaPipe pierde la mano hasta 0.15 s, mantiene el último movimiento.
- El **LiDAR frena el avance y el retroceso** antes de chocar (`front_limit`): la velocidad se limita con la distancia libre al frente o atrás.
- Al terminar (FIN) el programa se cierra solo; no hace falta el mouse.

---

## 12. Experimentación

### 12.1 Simulación (`tests/bench_directo.py`) — simulado

**Condiciones duras:** latencia de `/cmd_vel` de 0.2 s · LiDAR atrasado 0.3 s · ruido de 1 cm · deriva de odometría ±3 % · cámara de 250 px y 37° con 30 % de cuadros perdidos · circuito reconstruido del video de referencia, con anchos de 0.55 a 0.90 m y el pasillo real de 0.56 m.

**Criterio de aceptación:** completa A→B→A, **0 choques**, holgura mínima ≥ radio + 5 cm y termina a < 0.35 m de A.

| Perfil | 0.75 m | 0.90 m | Choques |
|---|---|---|---|
| Base (`config.json`) | 253 s | 170 s | 0 |
| Rápido | 170 s | 148 s | 0 |
| Rápido + arco (`config_rapido.json`) | 158 s | 127 s | 0 |
| Agresivo + arco (`config_agresivo.json`) | 145 s | 124 s | 0 |

−43 % de tiempo simulado (253 → 145 s) sin perder seguridad. Estos tiempos no incluyen el escaneo en B. La suite tiene **más de 340 pruebas automáticas** (pytest) en verde.

### 12.2 Robot real — 23 de noviembre

| Corrida | Perfil | Tiempo | Ida | Escaneo B | Regreso | Resultado |
|---|---|---|---|---|---|---|
| 1 | rápido | **105.3 s** | 46.5 s | 31.3 s | 27.2 s | Completa: Angel Mora «seguro» (0.58) + QR `milestone_beta`, 0 choques |
| 2 | agresivo | 69.1 s | 30.1 s | 17.5 s | 21.1 s | Fallida: tomó una cara del público como B |
| 3 | rápido | 149.1 s | 74.5 s | 26.0 s | 48.2 s | Completa pero lenta: giro cortado → vuelta extra; cara dudosa (0.33), sin QR |
| 4 | rápido | — | — | — | — | Detenida a los ~138 s: pasó por B sin reconocerla |

- **Ciclo de control medido en pista:** 80 ms de promedio, con 283–325 ms en el percentil 95 y la CPU al 96–99 %.
- **Costo en la Pi:** detector de señales 140 ms por cuadro (QR 51 ms, Hough 21 ms); identificación facial ~0.6 s.
- **Herramienta:** `scripts/recopilar_experimentos.py` baja los intentos de la Pi y arma `evidencias/experimentos/resumen.md` e `intentos.csv`. Cada intento guarda un JSON con los tiempos por estado, y en cada decisión un `.npz` con el scan, la pose y la foto.

---

## 13. Problemas encontrados y cómo se resolvieron

| Síntoma | Causa (diagnóstico con los logs) | Solución |
|---|---|---|
| La Pi no reconocía ninguna cara | YuNet 2023 requiere OpenCV ≥ 4.8 y la Pi tiene 4.6; `calentar()` escondía el error | YuNet 2022mar; el chequeo detecta una cara real |
| No leía las flechas impresas | La hoja trae la flecha espejada verticalmente respecto al documento | Plantillas de ambas formas |
| No detectaba una flecha con el anillo roto | El contorno no salía circular | Respaldo con la transformada de Hough |
| No veía la bandera de B | Bandera dibujada ondeando, cuadros deformados | Detector de damero por manchas |
| (Tras ese arreglo) algunos QR salían como bandera | Los módulos del QR parecen un damero | Rechazo por los 3 cuadros de posicionamiento anidados |
| Llegada falsa a B (corrida 2) | Cara del público detrás de la pared | Con salida lateral solo vale una foto grande (≥ 13 % del ancho) |
| Pasó de largo B (corrida 4) | B tiene una abertura lateral; el arco decidía 25 cm antes y la foto se veía chica | Con caras a la vista no adelanta la decisión |
| Giro cortado y vueltas extra (corrida 3) | Timeout fijo; con ciclos de 325 ms el giro iba lento pero avanzaba | Timeout por progreso; regreso sin lazos |
| Cara movida (score 0.33) | Capturaba apenas frenaba | Espera 0.4 s quieto; junta más cuadros si duda |
| CPU al 99 % | QR en cada cuadro | QR en 1 de cada 3 cuadros en la ida |
| Reloj de la Pi días atrasado | Sin internet ni NTP en el laboratorio | Sincronización de hora por SSH antes de correr |

---

## 14. Limitaciones (decirlas con honestidad)

- Los arreglos del final (bandera ondeando, cara del público, timeout por progreso, sin lazos, espera antes de capturar) se **probaron en simulación y con imágenes reales**, pero **no se validaron en una corrida completa en pista**.
- La simulación no modela la imagen real: le entrega al robot señales ya identificadas. Por eso los problemas de visión solo aparecieron en pista.
- El escaneo en B (31 s) es el tramo más largo después de la ida.
- Sin flecha y con dos salidas, la heurística puede meterse en un bolsillo; la memoria de visitas lo saca, pero cuesta tiempo.
- La odometría deriva; se corrige con las paredes después de cada giro y con ICP en el regreso por migas, pero no hay un mapa global.

---

## 15. Preguntas probables y respuestas cortas

1. **¿Por qué no usaron SLAM y Nav2?** El circuito se revela el mismo día, la Pi ya va al 96–99 % de CPU con la visión, y un laberinto de pasillos se resuelve bien de forma reactiva. Menos piezas implica menos fallas (reloj, TF, DDS).
2. **¿Qué heurística usan en un cruce sin flecha?** La rama menos visitada de ese cruce (memoria por cruce, idea tipo Trémaux), con desempate recto › izquierda › derecha. Tras un callejón sin caras vuelve por la rama que se saltó.
3. **¿Cómo saben que una salida está libre?** Tres rayos del LiDAR a 75°, 90° y 105° de ese lado tienen que llegar más lejos que máx(0.55 m, ancho/2 + 0.35 m).
4. **¿Qué pasa si la cámara ve una flecha falsa?** Se necesitan 2 lecturas iguales para confirmarla y, además, el LiDAR tiene la última palabra: si apunta a una pared, se ignora.
5. **¿Cómo distinguen izquierda de derecha sin una red neuronal?** Correlación con plantillas de 32×32 y una verificación geométrica independiente: de qué lado está el tallo de la flecha y en qué cuadrante el codo.
6. **¿Por qué el giro en arco no choca con la esquina?** Con radio R ≤ ancho/2, el arco que empieza R antes del centro del cruce libra la esquina interior; además se revisa con el LiDAR todo el arco por recorrer en cada ciclo.
7. **¿Cómo vuelve a A sin mapa?** Invierte la lista de giros de la ida y elimina los lazos. De respaldo, migas de pan cada 0.2 m más ICP contra keyframes del LiDAR.
8. **¿Qué es ICP?** Iterative Closest Point: alinea dos nubes de puntos emparejando cada punto con el más cercano y resolviendo la mejor rotación y traslación, y repite hasta converger. Lo usamos para corregir la deriva de la odometría.
9. **¿Qué es VFH?** Vector Field Histogram: evalúa rumbos alrededor del robot según cuánto espacio libre dejan y elige el de menor costo (cerca de la meta, sin cambiar mucho, con holgura). Es reactivo y no necesita mapa.
10. **¿Cómo funciona el reconocimiento facial?** YuNet detecta la cara y 5 puntos; SFace alinea y genera un vector de 128 dimensiones; se compara con la galería por similitud coseno. Es seguro si supera 0.363 y le gana al segundo por 0.04.
11. **¿Por qué 0.363?** Es el umbral recomendado por OpenCV para SFace con distancia coseno.
12. **¿Qué pasa si hay 4 caras juntas?** Para frente a cada una (centrada), recorta solo esa cara, la marca como vista (±9°) y sigue con la siguiente. Probado con los modelos reales: 4 de 4 a ≥ 13° de separación.
13. **¿Y si no reconoce a nadie?** Responde el más parecido marcado como «dudoso»: no anunciar nada vale 0.
14. **¿Cómo evitan una llegada falsa a B?** La bandera tiene que estar a ≤ 1 m (medido con el LiDAR) en 2 cuadros, y las caras solo cuentan en un callejón o, si hay salida lateral, si la foto se ve grande.
15. **¿Por qué la corrida 3 tardó 149 s?** Un giro se cortó por timeout y el robot quedó torcido; en un cruce sin flecha entró a un bolsillo y dio una vuelta. Se arregló con el timeout por progreso y el regreso sin lazos.
16. **¿Cómo validaron sin el robot?** Con un simulador 2D propio con latencia, LiDAR atrasado, ruido, deriva y una cámara con pérdida de cuadros; un perfil solo se aceptaba con 0 choques. Más de 340 pruebas automáticas.
17. **¿Qué tan lento es todo en la Pi?** Ciclo de 80 ms de promedio (p95 de ~300 ms); el detector de señales tarda 140 ms por cuadro y la identificación facial 0.6 s.
18. **¿Cómo clasifican los gestos?** Con reglas geométricas sobre los 21 puntos de MediaPipe: un dedo está doblado si su punta queda más cerca de la muñeca que su articulación media, y la dirección del pulgar define arriba o abajo. La pose se acepta con 3 cuadros estables; armar, acción y desarmar exigen sostenerla.
19. **¿Qué impide que el robot choque en la Tarea B?** El LiDAR limita el avance y el retroceso según la distancia libre, aunque el gesto pida avanzar.
20. **¿Qué mejorarían?** Validar los últimos arreglos en pista, acortar el escaneo en B (por ejemplo, decidir con menos cuadros cuando el score es alto) y, con más tiempo, un mapa local de ocupación para elegir mejor en cruces sin flecha.

---

## 16. Glosario rápido

- **Odometría:** posición estimada integrando el giro de las ruedas; deriva con el tiempo.
- **Inflado:** sumar el radio del robot (+ margen) a cada obstáculo para planificar el robot como un punto.
- **Pinhole:** modelo de cámara en el que el ángulo de un píxel es `atan((cx − u) / f)`.
- **Otsu:** umbral automático que separa claro y oscuro minimizando la varianza dentro de cada clase.
- **Transformada de Hough (círculos):** vota por centros y radios compatibles con los bordes de la imagen.
- **Embedding:** vector numérico que resume una cara; caras parecidas dan vectores cercanos.
- **Similitud coseno:** coseno del ángulo entre dos vectores (1 = iguales).
- **Trémaux:** método clásico de laberintos que marca los pasillos recorridos y prefiere los menos usados.
- **TwistStamped:** mensaje de ROS 2 con velocidad lineal y angular, más el sello de tiempo, que se publica en `/cmd_vel`.
