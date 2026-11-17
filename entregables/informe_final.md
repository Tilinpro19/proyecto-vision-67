# Informe Final Técnico · Proyecto TurtleBot 4 Lite

**Curso:** Robótica y Visión Computacional  
**Plataforma:** TurtleBot 4 Lite (iRobot Create 3 + Raspberry Pi 4 + RPLIDAR A1 + OAK-D Lite)  
**Software:** ROS 2 Jazzy, Fast DDS (`ROS_DOMAIN_ID=67`), Python 3, OpenCV DNN, MediaPipe, NumPy  
**Fecha de corte de software:** 17 de noviembre de 2026 (commit base `516392c` en `main`)  

---

## 1. Resumen Ejecutivo

Se implementó un sistema completo de navegación autónoma guiada por señales visuales y LiDAR (**Tarea A**), teleoperación segura por gestos de mano (**Tarea B**) y reconocimiento facial con anuncio por voz (**YuNet + SFace + SAPI/espeak/buzzer**).  

A diferencia de soluciones basadas en SLAM + Nav2, el sistema opera de forma **reactiva y ligera a bordo de la Raspberry Pi 4**, alcanzando **187 pruebas automatizadas aprobadas (1 omitida)** en simulación cinemática y de visión sin requerir mapa previo del circuito.

---

## 2. Arquitectura General del Sistema

```
Sensores (/scan, /odom, /oakd/rgb/preview/image_raw/compressed, /oakd/stereo/image_raw, /hazard_detection)
   │
   ├──► Tarea A (Autónoma, mision.py)
   │      ├── Con señales (sin --goal): senales_forma.py + pasillo.py + decision.py -> Piloto.paso_dir
   │      └── Respaldo (--goal X Y):    VFH-lite (nav_core.py) -> Piloto.paso
   │      └── Seguridad baja:           GuardianProfundidad bajo demanda (profundidad.py)
   │      └── Regreso B -> A:           Migas de pan + atajos + ICP 2D contra keyframes
   │
   ├──► Tarea B (Gestos, gestos_teleop.py + mano_teleop.py)
   │      └── MediaPipe HandLandmarker (webcam) + guardián LiDAR frontal/trasero
   │
   └──► Escaneo y Anuncio (facial.py)
          └── YuNet (detección) + SFace (embedding 128-D, coseno) -> /nombre_reconocido + voz + buzzer
```

### Máquina de estados (`1_competencia/mision.py`)
`ESPERA` $\rightarrow$ `IR_A_B` $\rightarrow$ `ESCANEAR` $\rightarrow$ `VOLVER_A_A` $\rightarrow$ `ANUNCIAR` $\rightarrow$ `FIN` (con parada de emergencia `PARADA` vía `/mision/parar`, `t_max`, `Ctrl+C` o puño cerrado en Tarea B).

---

## 3. Subsistemas Implementados

### 3.1 Detección de señales en blanco y negro por forma (`1_competencia/senales_forma.py`)
Las señales de competencia son impresiones en blanco y negro (sin color), por lo que se diseñó un detector basado exclusivamente en geometría y correlación con OpenCV + NumPy:
- **Flechas (`izq`, `der`) y `prohibido`:** detección de contornos externos circulares con hueco interior, normalización del interior a $40\times 40$ y correlación contra plantillas (`modelos/plantillas_forma.npz`). La plantilla derecha es el espejo de la izquierda.
- **Bandera de cuadros (`bandera`):** agrupación conexa de cuadrados oscuros compactos tras erosión morfológica; marca la llegada a la meta **B**.
- **Códigos QR (`qr`):** decodificación reutilizando `senales.py` para registrar hitos (`milestone_*`) en orden de aparición sin detener la marcha.
- **Robustez:** validado bajo perspectiva ($\pm 28^\circ$), inclinación ($10^\circ$), sobreexposición, poca luz, sombra lateral, resolución de preview (`250 px`) y rechazo de falsos positivos en texturas regulares, paredes y rostros.

### 3.2 Seguimiento de pasillo y detección de cruces (`1_competencia/pasillo.py`)
- Analiza el LiDAR (`/scan`) en conos frontal y laterales usando el disco inflado del robot ($R_{\text{inf}} = R_{\text{robot}} + \text{margen} = 0.17 + 0.14 = 0.31\text{ m}$ en `config.json`).
- Clasifica la topología local en tiempo real (`pasillo`, `callejon`, `L_izq`, `L_der`, `T`, `T_izq`, `T_der`, `cruz`) con confirmación temporal (`DetectorCruce`).
- Mantiene al robot centrado entre paredes laterales sin perder la evasión reactiva de obstáculos dentro del pasillo.

### 3.3 Política de decisión (`1_competencia/decision.py`)
Jerarquía estricta de decisión en cada cruce:
1. **`bandera`** (confirmada en 2 frames) $\rightarrow$ llegada a **B**, transición a `ESCANEAR`.
2. **`qr`** $\rightarrow$ registra hito único y continúa.
3. **`prohibido`** $\rightarrow$ marca la rama como cerrada y descarta `recto` / busca alternativa o retrocede.
4. **Flecha `izq` / `der`** $\rightarrow$ manda **si y solo si** el LiDAR confirma que esa rama está físicamente abierta; si contradice al LiDAR, gana el LiDAR con registro de aviso.
5. **Sin señal** $\rightarrow$ exploración por cruces con identidad espacial (`< 0.80 m`) y conteo de visitas por dirección cardinal para evitar bucles en callejones.

### 3.4 Navegación reactiva, regreso por migas y seguridad (`nav_core.py`, `navegacion.py`, `profundidad.py`)
- **Evasión VFH-lite:** 73 rumbos candidatos ($5^\circ$), holgura lateral (`side_margin`), escalado de velocidad en zonas estrechas (`side_scale`) y repulsión de paredes (`wall_repulsion`).
- **Guardián de profundidad OAK-D bajo demanda (`profundidad.py`):** se suscribe a `/oakd/stereo/image_raw` únicamente en pasillos estrechos, obstáculos cercanos o tras un atasco (con histéresis de 8 s), proyectando obstáculos bajos ($5\text{–}45\text{ cm}$ del suelo) en un scan virtual fusionado con el LiDAR.
- **Anti-atasco y barreras virtuales:** si el robot no avanza o activa `/hazard_detection` (bumper), retrocede con margen trasero seguro (`back_free > 0.20 m`), gira hacia el rumbo más abierto y coloca una barrera virtual.
- **Regreso $B \rightarrow A$:** sigue las migas en orden inverso con recorte de atajos en línea libre (`pick_waypoint`) y corrección de deriva de odometría mediante **ICP 2D** contra keyframes guardados en la ida.
- **Diagnóstico automático:** ante cualquier choque o atasco, `mision.py` guarda automáticamente un archivo `.npz` en `1_competencia/logs/` con `t`, `pose`, `ranges`, `angles` y `crumbs`.

### 3.5 Tarea B (Gestos) y Reconocimiento Facial (`gestos_teleop.py`, `facial.py`)
- **Gestos (MediaPipe `HandLandmarker`):** palma abierta `1 s` arma y arranca el cronómetro; puño cerrado para y desarma; 👍 avanzar, 👎 parar/retroceder, pistola izq/der girar, `V` `1.5 s` escanear/anunciar. Todos los comandos pasan por el guardián LiDAR.
- **Facial (YuNet + SFace):** al llegar a B recolecta frames nítidos (o ejecuta un barrido angular controlado si no ve rostro de frente), compara embeddings de 128-D por similitud coseno contra `1_competencia/galeria/` y anuncia el nombre al regresar a A.

---

## 4. Verificación Automatizada (Sin Robot)

Ejecución de la suite completa sobre `main` (`516392c`):
```bash
python -m pytest 1_competencia/tests/ -v
# Resultado: 187 passed, 1 skipped in 71.15s
```

| Archivo de pruebas | Cobertura principal |
|---|---|
| `test_senales_forma.py` | Hoja completa, perspectiva, luz, sombra, inclinación, preview 250 px, 0 falsos positivos |
| `test_senales.py` | Lectura de QR a 1 m y clasificación cromática base |
| `test_pasillo.py` | Clasificación de cruces y centrado en pasillo |
| `test_decision.py` | Tabla de verdad señal > exploración, contradicción LiDAR, prohibido, bandera, QR |
| `test_mision_guiada.py` | Laberinto completo con señales, callejón sin salida y regreso a A |
| `test_profundidad.py` | Proyección estéreo de obstáculos bajos y activación bajo demanda |
| `test_nav_core.py` / `test_navegacion.py` | VFH-lite, `side_scale`, barreras virtuales, escape, ICP 2D, `pick_waypoint` |
| `test_mision.py` / `test_eficiencia.py` | Máquina de estados, `drenar`, perfilado de CPU, `config.json` |
| `test_facial.py` / `test_gestos.py` | YuNet + SFace, selección por nitidez, clasificación de gestos y arming |

---

## 5. Resultados de Ensayos en Pista Real *(completar en laboratorio)*

> [!IMPORTANT]
> Completar esta tabla durante la sesión de laboratorio con los registros de `1_competencia/logs/` y `calibracion/`.

| Prueba | Intento 1 | Intento 2 | Intento 3 | Mejor tiempo | Choques | Nombre correcto |
|---|---|---|---|---|---|---|
| **Tarea A (guiada por señales)** | _pendiente_ | _pendiente_ | _pendiente_ | _pendiente_ | _pendiente_ | _pendiente_ |
| **Tarea B (gestos + webcam)** | _pendiente_ | _pendiente_ | _pendiente_ | _pendiente_ | _pendiente_ | _pendiente_ |

- **Distancia de detección medida con `--dry --ver`:** flechas `_ m`, prohibido `_ m`, bandera `_ m`, QR `_ m`.
- **Galería facial (`facial.py --probar`):** `_ / 11` personas verificadas sin errores.
- **Parámetros finales en `1_competencia/config.json`:** `max_linear = 0.14`, `safety_margin = 0.14`, `stop_dist = 0.38`, `wall_min = 0.38` *(actualizar tras `calibrar.py`)*.
