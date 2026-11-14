# Resumen de cambios en navegación y despliegue (Tarea A guiada)

## 1. Commits incluidos en esta entrega

En la rama `main` (`https://github.com/Tilinpro19/proyecto-vision-67`):

- `a6b1d44` — `feat(competencia): endurecer senales_forma para luz/inclinacion y preview 250px`
- `aa46438` — `feat(competencia): priorizar senal mas cercana por cuadro y probar callejon sin salida`
- `c0dfb82` — `feat(competencia): filtrar rejillas completas en preview y probar texturas/tiempos`
- `7eafd8d` — `fix(competencia): cerrar spin limpiamente cuando rclpy ya apago contexto`
- `70158ac` — `fix(competencia): evitar bucle de retroceso 180 grados en el mismo callejon`
- `35840ef` — `fix(competencia): seguir migas de regreso al cruce al retroceder en callejon`
- `89c16a8` — `fix(competencia): cooldown de 2.5s tras girar y mayor margen de seguridad en config.json`
- `e5614c8` — `fix(competencia): marcar rama de entrada en cruces, no cortar retroceso y guardar diagnostico npz`
- `fab57a1` — `fix(competencia): clave espacial de cruces (<0.8m) y margen back_free=0.20 en escape`

---

## 2. Explicación de las mejoras implementadas (`e5614c8` y `fab57a1`)

### A. Elección de rama al volver por un pasillo (`1_competencia/mision.py`)
- **Problema:** En `PilotoGuiado.paso`, al decidir en un cruce solo se sumaba visita a la rama elegida (`d_elegida`), pero no a la dirección por la que venía el robot (`rumbo_pasillo + pi`). Al regresar de un callejón sin salida, la dirección de vuelta al origen seguía con `0` visitas igual que la rama sin explorar, y `decision.py` desempataba por `preferencia` a favor de `"recto"`.
- **Solución:** Al tomar una decisión distinta de `"retroceder"`, se marca `d_origen = self._dir_cardinal(self.rumbo_pasillo + math.pi)` en `visitas_cruce` con al menos `1` visita (ver [`1_competencia/mision.py`](../1_competencia/mision.py#L690-L693)).

### B. Protección del retroceso por migas (`1_competencia/mision.py`)
- **Problema:** Si el LiDAR detectaba un cruce intermedio mientras el robot seguía las migas de vuelta (`retrocediendo_hasta_idx`), el bloque de decisión corría antes, podaba las migas y anulaba el retroceso a mitad de camino.
- **Solución:**
  - Se comprueba la llegada al cruce objetivo (`dist < 0.45 m`) antes del bloque de decisión ([`1_competencia/mision.py`](../1_competencia/mision.py#L619-L633)).
  - `puede_cruce` exige `self.retrocediendo_hasta_idx is None`, ignorando cruces intermedios hasta completar el regreso al cruce anterior (manteniendo `puede_frente` como respaldo si el camino de vuelta queda bloqueado).

### C. Identidad espacial de los cruces (`1_competencia/mision.py`)
- **Problema:** `key = (round(x), round(y))` partía un mismo cruce en dos celdas distintas si caía cerca de un borde de `0.5 m` o por drift de odometría, perdiendo el historial de visitas y ramas agotadas.
- **Solución:** `_clave_cruce(x, y, radio=0.80)` busca el cruce ya registrado más cercano a `< 0.80 m` y lo reutiliza; solo crea una clave nueva si no hay ninguno cerca ([`1_competencia/mision.py`](../1_competencia/mision.py#L543-L550)).

### D. Margen de retroceso en escape (`1_competencia/navegacion.py`)
- **Problema:** En `Piloto._escape` (fase `"atras"`), el robot retrocedía mientras `back_free > 0.10 m`, margen muy justo frente a obstáculos traseros durante un choque/atasco.
- **Solución:** Se subió el umbral a `back_free > 0.20` ([`1_competencia/navegacion.py`](../1_competencia/navegacion.py#L267)).

### E. Diagnóstico `.npz` en atascos y choques (`1_competencia/mision.py`)
- **Problema:** Con solo el `.log` y el `.json` no se podía distinguir si un `"atasco con frente libre"` tras un giro era un obstáculo bajo real o un falso positivo del detector de atasco.
- **Solución:** `Mision.guardar_diagnostico` guarda `t`, `pose`, `ranges`, `angles` y `crumbs` en `logs/*.npz` automáticamente cuando `comunes` detecta un choque (`choque_LiDAR` / `choque_bumper`) o cuando `_volcar_eventos_piloto` recibe un evento de `"atasco"` ([`1_competencia/mision.py`](../1_competencia/mision.py#L148-L164), [`1_competencia/mision.py`](../1_competencia/mision.py#L392-L423)).

---

## 3. Verificación (WSL y Raspberry Pi)

- **Tests en WSL** (`1_competencia/`):
  - `pytest -q` → **178 passed, 1 skipped** (71.77s), incluyendo 5 tests nuevos en [`1_competencia/tests/test_mision_guiada.py`](../1_competencia/tests/test_mision_guiada.py#L218-L333).
- **Despliegue en la Pi (`ubuntu@192.168.185.3:~/mision_mov/`)**:
  - Backups en `~/mision_mov/bak/`.
  - MD5 verificados (local = Pi):
    - `mision.py`: `cc8a37a1cd4a0798e9eb1c88706448d7`
    - `navegacion.py`: `5f6fc694b19c0802b15ce596341e4c47`
  - `python3 -m py_compile mision.py navegacion.py` → `COMPILE_OK`.

---

## 4. Pendiente tras la próxima corrida en la Pi

1. Correr en la Pi (desconectando el cable al arrancar):
   ```bash
   cd ~/mision_mov
   ./competencia_onboard.sh --param t_max=90
   ```
2. Traer el `.log` y los `logs/*.npz` para confirmar con datos:
   - Si los `"atasco con frente libre"` son obstáculos bajos o falsos positivos del detector.
   - Si conviene subir `max_linear` de `0.14` a `0.18` manteniendo los márgenes de seguridad (`stop_dist 0.38`, `safety_margin 0.14`).
