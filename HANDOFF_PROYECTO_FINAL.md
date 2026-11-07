# Traspaso del proyecto · TurtleBot 4 Lite

Guía para seguir trabajando el robot. Primero los **pasos a seguir** (sección 1); el resto es referencia.
Más detalle: `README.md` (flujo diario), `DESCUBRIMIENTOS.md` (trampas), `TROUBLESHOOTING.md`, `proyecto_final/README.md`.

**Entrega: 22 de noviembre de 2026.** Lineamientos: `proyecto_final/docs/lineamientos_proyecto_final.pdf`.

---

## 1. Pasos a seguir

### Paso 0 · Instalar el paquete (una vez, en la laptop)

Esta carpeta (`paquete_proyecto_final`) es una copia completa del trabajo. Los scripts ROS corren en **WSL**, así que cópiala allí:

```bash
cp -r /mnt/c/RUTA/A/paquete_proyecto_final ~/turtleclaude4
cd ~/turtleclaude4
chmod +x *.sh proyecto_final/1_competencia/*.sh proyecto_final/2_laboratorio/*.sh
python3 -c "import cv2, mediapipe, numpy; print('deps OK')"
```

Si falla el `import`, instalar **con internet, antes de ir al lab** (en el lab no hay `pip`/`apt`). ROS 2 Jazzy y la instalación de WSL están en `README.md` (Parte I).

### Paso 1 · Probar todo SIN robot (casa o lab, 15 min)

```bash
cd ~/turtleclaude4/proyecto_final
python3 -m pytest 1_competencia/tests -v          # lógica de evasión
cd 1_competencia
python3 facial.py --probar                         # revisa la galería (ver Paso 2)
```

Con el robot simulado (otra terminal): `python3 ../2_laboratorio/robot_simulado.py` y luego `./tarea_a.sh --goal 3.0 0.5`.
Todo debe correr sin errores antes de tocar el robot real.

### Paso 2 · Cargar la galería real de caras

En `proyecto_final/1_competencia/galeria/` crear **una carpeta por persona** (el nombre de la carpeta es lo que dice el robot; `_` se lee como espacio): `Juan_Perez/foto1.jpg`, `foto2.jpg`…
Si solo dan la foto impresa, capturarla con la cámara del robot:
`python3 facial.py --capturar Juan_Perez --n 15`. Luego `python3 facial.py --probar` (avisa fotos sin cara).

### Paso 3 · Sesión en el laboratorio: conectar

1. Red: **Lab_Computech o cable**. El Wi-Fi UTEC bloquea DDS (hay ping pero `ros2 topic list` casi vacío).
2. Si cambiaste cable↔Wi-Fi: en el robot `sudo systemctl restart turtlebot4.service` y esperar ~30 s.
3. En WSL:
   ```bash
   source ~/turtleclaude4/tb4_connect.sh 192.168.0.104 67
   ```
   (Si la hora desfasa más de 2 s ofrece sincronizarla; también `source tb4_connect.sh --hora`.)
4. Verificar:
   ```bash
   ./tb4_check.sh 192.168.0.104
   python3 chequeo_reloj_tb4.py
   ```
   Debes ver `/scan`, `/odom`, cámara y reloj coherente. Si no, ir a la sección 4.

### Paso 4 · Hora persistente (una sola vez por robot)

Sin esto la Pi vuelve a 2025 en cada arranque y SLAM/TF fallan.
- En la laptop: `./configurar_sync_automatico.sh` (clave SSH + sudo sin clave solo para fijar la hora).
- En la Pi: copiar y correr `instalar_hora_persistente.sh`.

### Paso 5 · Tarea A (autónoma) en el robot real

```bash
cd ~/turtleclaude4/proyecto_final/1_competencia
./tarea_a.sh --goal 3.0 0.5          # B a 3 m al frente y 0.5 m a la izquierda de A
```

- Empezar con velocidad baja: `--param max_linear=0.15`, con alguien con la mano lista para parar.
- Los logs quedan en `logs/`. Si hay choques, ajustar margen y velocidades en `config.json`.
- Parámetros distintos sin editar: `--param nombre=valor`. Otro robot: `ROBOT_IP=… ROBOT_ID=… ./tarea_a.sh …`.

### Paso 6 · Tarea B (gestos) con webcam real

```bash
./tarea_b.sh --goal 3.0 0.5
```

Gestos: palma 1 s = armar · puño = parar y desarmar · 👍 avanzar · pistola girar · 👎 parar · V 1.5 s = escanear/anunciar.
Probar con buena luz y fondo limpio; ajustar los tiempos de los gestos si hay falsos positivos. **Sin teclado, mouse ni joystick durante el intento.**

### Paso 7 · Calibrar

```bash
python3 ../2_laboratorio/calibrar.py
```
N intentos por juego de parámetros; costo = `t + c·choques + e·nombre_erróneo`. Resultados en `calibracion/`. Elegir los parámetros con menor costo y dejarlos en `config.json`.

### Paso 8 · Ensayo general (como el día de la competencia)

El circuito y la foto se revelan ese día: ensayar con un circuito nuevo que nadie haya visto, **3 intentos por tarea** (cuenta el mejor), cronometrando. Prioridad: **completar sin chocar, luego acelerar.** Fallar vale 0.

### Paso 9 · Entregables (empezar ya, en paralelo)

En `proyecto_final/4_entregables/`: **informe**, **presentación** y **video** (los `.mp4/.mov` no se versionan). Repartirse: uno graba video durante los Pasos 5–8.

### Check de avance

- [ ] Paso 1 sin errores (tests + simulador)
- [ ] Galería real cargada y `facial.py --probar` limpio
- [ ] Hora persistente instalada en el robot
- [ ] Tarea A: ida y vuelta sin choques en el lab
- [ ] Tarea B: ida y vuelta solo con gestos
- [ ] Nombre dicho correctamente al volver a A
- [ ] Calibración hecha y `config.json` actualizado
- [ ] Ensayo general con circuito nuevo
- [ ] Informe, presentación y video

---

## 2. Qué hay que lograr

- **Tarea A (autónoma):** ir de A a B esquivando obstáculos, reconocer la foto de una persona en B, volver a A y **decir el nombre**.
- **Tarea B (gestos):** lo mismo guiado solo por gestos.
- Solo puntúa si vuelve a A **y** dice bien el nombre. Choque con obstáculo/pared e identificación errónea suman segundos. Si supera `tmax`, 0.

## 3. Estado del código (`proyecto_final/`)

| Pieza | Estado |
|---|---|
| `mision.py` (estados `ESPERA → IR_A_B → ESCANEAR → VOLVER_A_A → ANUNCIAR → FIN`, log, parada de emergencia), `tarea_a.sh`, `tarea_b.sh`, `config.json` | Hecho. |
| `navegacion.py` + `nav_core.py` (evasión VFH-lite, regreso por migas con ICP) | Solo en simulador. **Falta robot real.** |
| `gestos_teleop.py` + `mano_teleop.py` | Solo con gestos sintéticos. **Falta webcam real.** |
| `facial.py` (YuNet + SFace, voz de Windows) | Probado offline. **Falta la galería real.** |
| `2_laboratorio/` (`calibrar.py`, `robot_simulado.py`, `grabar_scan.py`) | Listos. |

Los módulos de `1_competencia/` se importan entre sí: mantenerlos en la misma carpeta.

## 4. Si algo falla (mirar primero HORA y RED)

| Síntoma | Causa y arreglo |
|---|---|
| Ping/SSH OK pero `ros2 topic list` casi vacío | Wi-Fi UTEC bloquea DDS → Lab_Computech o cable. |
| Robot "desaparece" tras cambiar de red | `sudo systemctl restart turtlebot4.service` en el robot. |
| Solo se ven algunos tópicos (`/cmd_vel`, `/odom`… faltan) | Fast DDS con peers estáticos contacta solo 4 participantes → usar `fastdds_pc.xml` (`export FASTRTPS_DEFAULT_PROFILES_FILE=~/turtleclaude4/fastdds_pc.xml`; `tb4_connect.sh` lo deja armado). |
| `queue is full` / errores de TF | Pi y Create 3 con horas distintas. No hacer saltos pequeños de hora. Paso 4. |
| `Unable to open file …task` o script desactualizado | La copia de WSL no tiene el archivo/modelo: volver a hacer `cp` (Paso 0). |
| La Pi va lenta con SLAM + Nav2 | Tarea A es reactiva (sin SLAM). Si hace falta Nav2: `aligerar_nav2_robot.sh` (**sin probar en robot**) y `levantar_mapa_robot.sh nav2`, siempre dentro de `tmux`. |

Diagnóstico completo: `diagnostico_tb4.py` (laptop), `diag_robot.sh` (robot). Detalle en `DESCUBRIMIENTOS.md` y `TROUBLESHOOTING.md`.

## 5. Datos fijos

- TB4 Lite, ROS 2 Jazzy, Fast DDS, `ROS_DOMAIN_ID=67`.
- Pi: Wi-Fi `192.168.0.104` · cable `192.168.185.3` (laptop por cable: `192.168.185.10/24`, sin gateway).
- El Create 3 **no tiene sensores atrás**: retroceder despacio.
- El lab es una LAN **sin internet** con 3 robots: cada uno con su `ROS_DOMAIN_ID` y hostname; cámara siempre por `/compressed`.
- Mando PS4/PS5 (solo pruebas, **no en competencia**): emparejar en la Pi con `scan off` antes de `pair`; ver `mando.md`.

## 6. Objetivos opcionales (no necesarios para la entrega)

SLAM + Nav en el robot con un script · SLAM + Nav en la PC con Docker · GUI en Python con `/map` y `/cmd_vel` (en Windows, `rosbridge`/`foxglove_bridge` mejor que `rclpy`) · teleop del mando como servicio systemd.

## 7. Reglas de trabajo

- No ejecutar nada que mueva el robot sin avisar al equipo.
- Instalar y descargar todo antes, con internet.
- Esta carpeta es una **copia** hecha el 2026-10-29: `proyecto_final/` y `seguimiento_vision/` no están en GitHub. Pasen los cambios entre ustedes (zip/USB) o acuerden quitar esas líneas de `.git/info/exclude` y hacer commit.
