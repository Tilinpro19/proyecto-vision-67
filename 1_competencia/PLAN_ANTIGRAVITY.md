# Plan: mision autonoma guiada por senales (Tarea A sin `--goal`)

Documento de traspaso para Antigravity. Lee TODO antes de tocar codigo.
Idioma del proyecto: espanol (comentarios, logs, commits).

## 1. Objetivo

Hoy `mision.py --tarea A` exige `--goal X Y` (posicion de B medida con cinta). En la
competencia **no se conoce B**: el robot debe ir por los caminos (pasillos) y decidir en cada
punto de decision:

```
¿hay una senal visible (flecha / prohibido / bandera / QR)?
 ├─ SI  -> obedecerla
 └─ NO  -> decidir solo con el LiDAR (explorar)
```

Al llegar a B (bandera de cuadros) analiza las caras (`facial.py`), y **cuando termina de
analizarlas regresa a A** y anuncia (`facial.Anunciador`), como ya hace `mision.py`.

## 2. Senales (material de referencia en `tests/senales_ref/`)

Origen: `señales.docx` del profe. Todas son **blanco y negro** (no hay color, asi que la
clasificacion azul/rojo de `senales.py` NO sirve para esto).

| Senal | Cant. | Significado (SUPUESTO, ver sec. 7) |
|---|---|---|
| Circulo con flecha doblada a la izquierda | 5 | girar a la izquierda |
| Circulo con flecha doblada a la derecha | 5 | girar a la derecha |
| Circulo con barra diagonal (prohibido) | 2 | no pasar por ese camino |
| Bandera de cuadros | 1 | llegada a B -> escanear caras |
| QR `milestone_{alfa,beta,gamma,omega,phi,rho}` | 6 | hitos (solo contar/registrar) |

Archivos: `hoja_contacto.png` (vista de todo), `senales_completo.pdf` (9 paginas),
`qr_milestone_*.png`, `bandera_cuadros.png`. Las flechas y el prohibido son formas vectoriales
del docx: estan solo en el PDF / hoja de contacto. Renderiza el PDF con PyMuPDF
(`pymupdf`) para obtener imagenes de cada una.

## 3. Lo que YA existe (no reescribir)

| Pieza | Donde | Nota |
|---|---|---|
| Evasion VFH-lite, migas de pan, ICP, regreso a A | `navegacion.py` (`Piloto`) | `fijar_origen`, `ir_a`, `volver`, `paso` |
| Exploracion por fronteras (malla, Dijkstra) | `explorar.py` | hoy independiente de `mision.py`, sin senales |
| Lectura de QR (por texto) y color | `senales.py` | QR sirve; color NO |
| Maquina de estados ESPERA→IR_A_B→ESCANEAR→VOLVER_A_A→ANUNCIAR→FIN | `mision.py` | `tarea_a()` ~linea 362 |
| Reconocimiento facial y anuncio | `facial.py` | galeria en `galeria/` |
| Simulador | `2_laboratorio/robot_simulado.py`, `tests/sim.py` | probar aqui primero |
| Tests | `tests/test_*.py` | mantenerlos en verde |

## 4. Fases (en orden; cada una con su test; commit por fase)

### Fase 1 — Detector de senales por forma (sin ROS)
Archivo nuevo `senales_forma.py` (solo OpenCV + numpy, igual que `senales.py`).
- API: `detectar(img) -> list[Senal]`, `Senal(tipo, confianza, bbox, texto=None)`;
  `tipo ∈ {"izq","der","prohibido","bandera","qr"}`; para `qr`, `texto` = contenido.
- Circulo: `cv2.HoughCircles` o contorno con circularidad. Dentro: la flecha doblada;
  izquierda vs derecha por **masa de pixeles oscuros a cada lado del centroide** o espejo de
  plantilla (`cv2.matchTemplate` + `cv2.flip`). Prohibido: diagonal de borde a borde.
- Bandera: patron de cuadros (alta densidad de esquinas alternadas / transiciones B-N).
- QR: reutilizar `senales.leer` (agrega borde blanco; ver su docstring).
- Test `tests/test_senales_forma.py`: generar casos desde `senales_ref/` y desde
  `tests/render_senales.py` (ya simula la camara del TB4: distancia, perspectiva, ruido).
  Criterio: izq/der nunca se confunden; falsos positivos = 0 en imagenes de fondo.
- Requisito de distancia: la camara del TB4 es de baja resolucion; documentar a que
  distancia maxima se detecta cada senal (ver como lo hizo `senales.py` con QR de 10 cm).

### Fase 2 — Seguir pasillo y detectar puntos de decision (LiDAR)
En `navegacion.py` o archivo nuevo `pasillo.py` (preferir archivo nuevo; no engordar `Piloto`).
- Centrarse entre paredes izq/der; avanzar mientras haya espacio al frente.
- Detectar cruce: hueco lateral abierto (izq, der o ambos) o frente cerrado.
- Salida: `PuntoDecision(opciones={"izq","recto","der"})`.
- Tests en `tests/` con mapas sinteticos (L, T, cruz, callejon) usando `tests/sim.py`.

### Fase 3 — Politica de decision (senal > exploracion)
Archivo nuevo `decision.py`, funcion pura y testeable:
`decidir(opciones, senal, visitado, preferencia) -> accion`.
1. Senal valida y la opcion existe en el LiDAR → obedecerla.
2. Senal contradice al LiDAR (flecha a la izq. y hay pared) → **gana el LiDAR**, log de aviso.
3. Sin senal → la opcion menos explorada (usar fronteras de `explorar.py`).
4. Empate → orden fijo: recto, izquierda, derecha.
5. Callejon → retroceder por migas al ultimo cruce y marcar la rama como agotada.
6. Prohibido → marcar esa rama como agotada y elegir otra.
Tests de tabla con todos los casos.

### Fase 4 — Integrar en `mision.py`
- Nueva `tarea_a_guiada()` (o `--modo guiado`); **no romper** `--goal` ni Tarea B.
- Estados: `ESPERA → SEGUIR → DECIDIR → ... → LLEGADA_B → ESCANEAR → VOLVER_A_A → ANUNCIAR → FIN`.
- `LLEGADA_B` al detectar bandera (confirmar N frames seguidos). `ESCANEAR` usa `facial.py`.
  Al terminar de analizar las caras → `piloto.volver()` (migas + ICP, ya existe).
- La camara hoy se pausa mientras solo se navega (`s.cam.pausar()`, ahorra CPU en la Pi):
  en modo guiado hace falta camara activa; usar el preview chico y subir a alta resolucion
  solo cuando `buscar_manchas`/forma sugiera una senal (idea ya usada en `senales.py`).
- Registrar cada decision y cada senal en el log de `logs/` y en el JSON de resultado.
- Parada de emergencia, `t_max`, choques y bateria deben seguir funcionando igual.

### Fase 5 — Simulador y script unico
- Extender `robot_simulado.py` con un laberinto con senales (imagenes de `senales_ref/`).
- `competencia.sh` (opcional): pregunta A/B, sincroniza hora, verifica sensores, lanza `mision.py`.
- Variante `*_onboard.sh` para correr en la Pi.

## 5. Restricciones del entorno (importantes, ya nos costaron horas)

- **WSL tiene una copia separada del repo.** Los scripts corren en `~/turtleclaude4` de WSL:
  tras editar en Windows, copiar desde `/mnt/c/...` antes de probar.
- **Pi:** `ubuntu@192.168.185.3` por llave; `sudo` pide clave. Codigo del robot en
  `~/mision_mov`. **No usar `desplegar_en_robot.sh`.**
- **Hora:** la Pi vuelve a 2025 sin NTP y rompe TF/SLAM. Hay scripts de sync en el repo.
- **Red del laboratorio:** router sin internet, sin NTP/apt. UTEC bloquea DDS: si cambia cable/Wi-Fi,
  reiniciar bringup.
- **RPLIDAR:** NO llamar `/start_motor` con el LiDAR girando: tumba el driver. Ya arreglado
  en esta rama; no reintroducir.
- **CPU de la Pi (4 nucleos):** compartida con bringup y LiDAR. Evitar procesar imagenes a
  cada ciclo; mantener `OPENBLAS_NUM_THREADS=1`.
- `ROS_DOMAIN_ID=67` y la IP del Create 3 (`192.168.186.2`) estan fijos en `*_onboard.sh`.
- Probar **siempre** con `--dry` y en simulador antes de mover el robot real.
- Excluir `node_modules`, `.git`, `dist`, `build`, `__pycache__`, `logs/`, `galeria/` al buscar.

## 6. Criterios de aceptacion

1. `pytest 1_competencia/tests` en verde, con tests nuevos por fase.
2. En simulador: laberinto con T y cruz, flechas correctas → llega a la bandera, escanea, vuelve a A.
3. En simulador: laberinto **sin** senales → no entra en bucles, explora y llega.
4. En simulador: senal que contradice una pared → no choca; queda en el log.
5. `--dry` en el robot real: sensores y camara OK, el robot no se mueve, CPU de la Pi razonable.
6. Tarea B y `--goal` siguen funcionando igual.

## 7. Supuestos SIN confirmar (marcar y no asumir en silencio)

Pendientes de respuesta del profe/companero. Implementar como **parametros** en `config.json`:
1. Prohibido = "camino cerrado, busca otro".
2. Bandera de cuadros = llegada a B.
3. QR de hito = solo registrar (¿importa el orden alfa→beta→...?).
4. El regreso a A desanda el camino por migas, sin obedecer las flechas en sentido contrario.
5. Las flechas se interpretan **relativas al robot** (izquierda del robot), no al mundo.

Si algo de la lista cambia, avisar antes de reescribir la politica de la Fase 3.

## 8. Reglas de trabajo

- Cambios pequenos, un commit por fase, mensajes `feat(competencia): ...` en espanol.
- No reescribir lo que ya funciona (sec. 3). Diffs concisos.
- No tocar `galeria/` ni los archivos sin seguimiento del usuario.
- Si una fase requiere el robot fisico, **parar y pedirlo**; no inventar resultados.
